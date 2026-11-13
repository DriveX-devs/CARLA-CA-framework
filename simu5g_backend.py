#!/usr/bin/env python3
"""Lockstep Simu5G backend for an existing OpenCDA/CARLA scenario.

This module owns only the ZeroMQ protocol between OpenCDA and the OMNeT++
``CarlaCoordinator``.  It deliberately does not create CARLA actors, call
``world.tick()``, run perception/planning/control, or destroy scenario objects.
Those responsibilities remain in the normal CARLA-CA scenario runner.

The protocol is synchronous:

1. OMNeT++ sends INIT and waits for the initial state.
2. OMNeT++ sends STEP, including packets delivered in the last interval.
3. ``wait_for_step()`` returns that request to the CARLA-CA loop.
4. CARLA-CA advances one tick and performs its normal processing.
5. ``complete_step()`` replies with current actor states and outgoing packets.
6. The exchange repeats until OMNeT++ sends STOP.

The code is compatible with Python 3.7+ and does not depend on NumPy.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Set, Tuple

import zmq


# Keep this equal to the working smoke-test CarlaCoordinator.cc.  Multiple
# actors and messages do not require a version bump by themselves.
PROTOCOL_VERSION = 3


class Simu5gBackendError(RuntimeError):
    """Base exception raised by the Simu5G backend."""


class ProtocolError(Simu5gBackendError):
    """Raised when the peer sends an invalid or out-of-sequence request."""


@dataclass(frozen=True)
class InitRequest:
    """Validated initialization values supplied by OMNeT++."""

    dt: float
    duration: float
    seed: int


@dataclass(frozen=True)
class StepRequest:
    """One simulation-step request received from OMNeT++."""

    step: int
    sim_time: float
    delivered_messages: Tuple[Dict[str, Any], ...]


class Simu5gBackend:
    """ZeroMQ REP endpoint used inside the CARLA-CA scenario runner.

    Parameters
    ----------
    bind_endpoint:
        ZeroMQ endpoint on which OpenCDA waits for ``CarlaCoordinator``.
    protocol_version:
        Must match the value in the OMNeT++ coordinator.
    poll_interval_ms:
        Receive polling interval.  A finite interval lets ``request_stop()``
        interrupt a wait cleanly after Ctrl+C.
    message_log_path:
        Optional CSV file for generated and delivered-message events.  Pass
        ``None`` to disable this log.
    static_network_endpoints:
        Network endpoint names that have no CARLA mobility actor.  The edge
        server uses the existing V2X identifier ``BS``.
    context:
        Optional existing ZeroMQ context, primarily useful for tests.  A
        context supplied by the caller is not terminated by ``close()``.
    """

    def __init__(
        self,
        bind_endpoint: str = "tcp://127.0.0.1:5555",
        protocol_version: int = PROTOCOL_VERSION,
        poll_interval_ms: int = 250,
        message_log_path: Optional[str] = "results/opencda_messages.csv",
        static_network_endpoints: Iterable[str] = ("BS",),
        context: Optional[zmq.Context] = None,
    ) -> None:
        if poll_interval_ms <= 0:
            raise ValueError("poll_interval_ms must be positive")

        self.bind_endpoint = str(bind_endpoint)
        self.protocol_version = int(protocol_version)
        self.poll_interval_ms = int(poll_interval_ms)

        self._owns_context = context is None
        self._context = context if context is not None else zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(self.bind_endpoint)

        self._closed = False
        self._local_stop_requested = False
        self._remote_stopped = False
        self._initialized = False
        self._reply_pending = False
        self._pending_step: Optional[StepRequest] = None

        self.dt = 0.0
        self.duration = 0.0
        self.seed = 0
        self.step = 0
        self.frame = 0
        self.sim_time = 0.0

        self._actor_names: Tuple[str, ...] = tuple()
        static_names = []
        for raw_name in static_network_endpoints:
            name = str(raw_name).strip()
            if not name:
                raise ValueError("static network endpoint names cannot be empty")
            if name in static_names:
                raise ValueError(
                    "duplicate static network endpoint {!r}".format(name)
                )
            static_names.append(name)
        self._static_endpoint_names: Tuple[str, ...] = tuple(static_names)
        self._network_endpoint_names: Set[str] = set(static_names)
        self._next_message_id = 1
        self._generated_message_ids: Set[int] = set()
        # A message may eventually be broadcast to more than one receiver, so
        # de-duplicate deliveries by (message_id, receiver), not only by id.
        self._delivered_message_keys: Set[Tuple[int, str]] = set()

        self._message_log_file = None
        self._message_log_writer = None
        if message_log_path is not None:
            self._open_message_log(message_log_path)

        print(
            "Simu5G backend listening on {} (protocol v{})".format(
                self.bind_endpoint,
                self.protocol_version,
            ),
            flush=True,
        )

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def stopped(self) -> bool:
        return self._local_stop_requested or self._remote_stopped

    def request_stop(self) -> None:
        """Ask a blocking receive loop to return at its next poll interval."""

        self._local_stop_requested = True

    def initialize(
        self,
        actors: Mapping[str, Any],
        frame: int,
        sim_time: float = 0.0,
        expected_dt: Optional[float] = None,
        expected_duration: Optional[float] = None,
    ) -> Optional[InitRequest]:
        """Wait for INIT and reply with the already-created CARLA actors.

        ``actors`` must explicitly map stable network names such as ``Cav1``
        and ``Cav2`` to their CARLA vehicle objects.  Returning ``None`` means
        local shutdown was requested before INIT arrived.
        """

        self._ensure_open()
        if self._initialized:
            raise Simu5gBackendError("initialize() may only be called once")

        actor_map = self._validate_actor_map(actors, first_state=True)

        while not self._local_stop_requested:
            request = self._receive_json()
            if request is None:
                continue

            request_type = request.get("type")
            if request_type == "PING":
                self._send_json(
                    {
                        "type": "PONG",
                        "protocol_version": self.protocol_version,
                    }
                )
                continue

            if request_type != "INIT":
                self._fail_pending_request(
                    "expected INIT, got {!r}".format(request_type)
                )

            try:
                self._validate_protocol_version(request)
                dt = self._positive_float(request, "dt")
                duration = self._positive_float(request, "duration")
                seed = int(request.get("seed", 0))

                if expected_dt is not None:
                    self._require_close(
                        dt,
                        float(expected_dt),
                        "OMNeT++ dt does not match CARLA fixed delta",
                    )
                if expected_duration is not None:
                    self._require_close(
                        duration,
                        float(expected_duration),
                        "OMNeT++ duration does not match scenario duration",
                    )

                initial_time = float(sim_time)
                self._require_finite(initial_time, "sim_time")
                if abs(initial_time) > 1e-9:
                    raise ProtocolError(
                        "initial sim_time must be 0.0, got {}".format(
                            initial_time
                        )
                    )

                self.dt = dt
                self.duration = duration
                self.seed = seed
                self.step = 0
                self.frame = int(frame)
                self.sim_time = initial_time
                self._actor_names = tuple(actor_map.keys())
                overlap = set(self._actor_names).intersection(
                    self._static_endpoint_names
                )
                if overlap:
                    raise ProtocolError(
                        "CARLA actors and static network endpoints overlap: {}"
                        .format(sorted(overlap))
                    )
                self._network_endpoint_names = set(self._actor_names)
                self._network_endpoint_names.update(
                    self._static_endpoint_names
                )

                response = self._state_response(
                    actor_map,
                    frame=self.frame,
                    step=0,
                    sim_time=initial_time,
                    outgoing_messages=[],
                    status="RUNNING",
                )
                self._send_json(response)
                self._initialized = True
                return InitRequest(dt=dt, duration=duration, seed=seed)
            except Exception as exc:
                if self._reply_pending:
                    self._send_error(str(exc))
                raise

        return None

    def wait_for_step(self) -> Optional[StepRequest]:
        """Wait for STEP and return messages delivered by Simu5G.

        The caller must invoke ``complete_step()`` exactly once for every
        non-``None`` result.  ``None`` means STOP was received or local
        shutdown was requested.  Do not advance CARLA in that case.
        """

        self._ensure_ready()
        if self._pending_step is not None:
            raise Simu5gBackendError(
                "complete_step() is required before the next wait_for_step()"
            )

        while not self._local_stop_requested:
            request = self._receive_json()
            if request is None:
                continue

            request_type = request.get("type")
            if request_type == "PING":
                self._send_json(
                    {
                        "type": "PONG",
                        "protocol_version": self.protocol_version,
                    }
                )
                continue

            if request_type == "STOP":
                self._validate_optional_protocol_version(request)
                self._send_json(
                    {
                        "type": "STOPPED",
                        "protocol_version": self.protocol_version,
                    }
                )
                self._remote_stopped = True
                return None

            if request_type == "INIT":
                self._fail_pending_request("INIT is only valid once")
            if request_type != "STEP":
                self._fail_pending_request(
                    "expected STEP or STOP, got {!r}".format(request_type)
                )

            try:
                self._validate_optional_protocol_version(request)
                next_step = self.step + 1
                expected_time = next_step * self.dt
                requested_time = float(request.get("sim_time", expected_time))
                self._require_finite(requested_time, "sim_time")

                tolerance = max(1e-9, self.dt * 0.01)
                if abs(requested_time - expected_time) > tolerance:
                    raise ProtocolError(
                        "out-of-sequence STEP: got {:.9f}, expected {:.9f}".format(
                            requested_time,
                            expected_time,
                        )
                    )
                if requested_time > self.duration + tolerance:
                    raise ProtocolError(
                        "STEP at {:.9f}s exceeds duration {:.9f}s".format(
                            requested_time,
                            self.duration,
                        )
                    )

                delivered = request.get("delivered_messages", [])
                if not isinstance(delivered, list):
                    raise ProtocolError("delivered_messages must be a list")

                validated_deliveries = []
                for raw_message in delivered:
                    message = self._validate_delivered_message(raw_message)
                    validated_deliveries.append(message)
                    self._log_message(
                        "DELIVERED_TO_OPEN_CDA",
                        message,
                        python_delivery_time=requested_time,
                    )

                result = StepRequest(
                    step=next_step,
                    sim_time=requested_time,
                    delivered_messages=tuple(validated_deliveries),
                )
                self._pending_step = result
                return result
            except Exception as exc:
                if self._reply_pending:
                    self._send_error(str(exc))
                raise

        return None

    def complete_step(
        self,
        actors: Mapping[str, Any],
        frame: int,
        outgoing_messages: Optional[Iterable[Mapping[str, Any]]] = None,
    ) -> Dict[str, Any]:
        """Reply to the pending STEP after one CARLA-CA iteration.

        This method serializes mobility but never changes the actors.  The
        response dictionary is returned to make application-level testing and
        logging straightforward.
        """

        self._ensure_ready()
        if self._pending_step is None or not self._reply_pending:
            raise Simu5gBackendError(
                "complete_step() called without a pending STEP request"
            )

        try:
            actor_map = self._validate_actor_map(actors, first_state=False)
            messages = []
            for raw_message in outgoing_messages or []:
                message = self._validate_outgoing_message(raw_message)
                messages.append(message)
                self._log_message("GENERATED", message)

            request = self._pending_step
            status = (
                "FINISHED"
                if request.sim_time + max(1e-9, self.dt * 0.01)
                >= self.duration
                else "RUNNING"
            )
            response = self._state_response(
                actor_map,
                frame=int(frame),
                step=request.step,
                sim_time=request.sim_time,
                outgoing_messages=messages,
                status=status,
            )
            self._send_json(response)

            self.step = request.step
            self.frame = int(frame)
            self.sim_time = request.sim_time
            self._pending_step = None
            return response
        except Exception as exc:
            if self._reply_pending:
                self._send_error(str(exc))
            self._pending_step = None
            raise

    def make_message(
        self,
        source_actor: str,
        destination_actor: str,
        message_type: str,
        payload_size: int,
        generation_time: Optional[float] = None,
        payload: Optional[Any] = None,
    ) -> Dict[str, Any]:
        """Create a uniquely identified outgoing unicast message.

        ``payload`` is optional and must be JSON serializable.  Simu5G may use
        ``payload_size`` to construct the simulated packet while preserving
        the payload/metadata for delivery back to OpenCDA.
        """

        if generation_time is None:
            if self._pending_step is not None:
                generation_time = self._pending_step.sim_time
            else:
                generation_time = self.sim_time

        message = {
            "message_id": self._next_message_id,
            "source_actor": str(source_actor),
            "destination_actor": str(destination_actor),
            "message_type": str(message_type),
            "generation_time": float(generation_time),
            "payload_size": int(payload_size),
        }
        self._next_message_id += 1
        if payload is not None:
            message["payload"] = payload
        return message

    def close(self) -> None:
        """Close the endpoint without touching any CARLA-owned object."""

        if self._closed:
            return

        # A STEP normally must receive a reply.  During local error shutdown,
        # send one best-effort error so the OMNeT++ REQ socket does not hang.
        if self._reply_pending:
            try:
                self._send_error("OpenCDA Simu5G backend is shutting down")
            except Exception:
                pass

        if self._message_log_file is not None:
            self._message_log_file.close()
            self._message_log_file = None
            self._message_log_writer = None

        self._socket.close(linger=0)
        if self._owns_context:
            self._context.term()
        self._closed = True

    def __enter__(self) -> "Simu5gBackend":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def _receive_json(self) -> Optional[Dict[str, Any]]:
        while not self._local_stop_requested:
            events = self._socket.poll(self.poll_interval_ms, zmq.POLLIN)
            if not events:
                return None

            try:
                request = self._socket.recv_json()
            except (ValueError, zmq.ZMQError) as exc:
                raise ProtocolError("failed to receive JSON: {}".format(exc))

            self._reply_pending = True
            if not isinstance(request, dict):
                self._fail_pending_request("request must be a JSON object")
            return request
        return None

    def _send_json(self, response: Mapping[str, Any]) -> None:
        if not self._reply_pending:
            raise Simu5gBackendError("attempted to reply without a request")
        try:
            self._socket.send_json(dict(response))
        except (TypeError, ValueError, zmq.ZMQError) as exc:
            raise Simu5gBackendError(
                "failed to send JSON response: {}".format(exc)
            )
        finally:
            self._reply_pending = False

    def _send_error(self, message: str) -> None:
        self._send_json(
            {
                "type": "ERROR",
                "protocol_version": self.protocol_version,
                "message": str(message),
            }
        )

    def _fail_pending_request(self, message: str) -> None:
        if self._reply_pending:
            self._send_error(message)
        raise ProtocolError(message)

    def _state_response(
        self,
        actors: Mapping[str, Any],
        frame: int,
        step: int,
        sim_time: float,
        outgoing_messages: List[Dict[str, Any]],
        status: str,
    ) -> Dict[str, Any]:
        return {
            "type": "STATE",
            "protocol_version": self.protocol_version,
            "status": status,
            "step": int(step),
            "frame": int(frame),
            "sim_time": float(sim_time),
            "actors": [
                self._actor_state(name, actor)
                for name, actor in actors.items()
            ],
            "outgoing_messages": outgoing_messages,
        }

    def _actor_state(self, name: str, actor: Any) -> Dict[str, Any]:
        try:
            transform = actor.get_transform()
            velocity = actor.get_velocity()
            state = {
                "id": int(actor.id),
                "name": str(name),
                "x": float(transform.location.x),
                "y": float(transform.location.y),
                "z": float(transform.location.z),
                "vx": float(velocity.x),
                "vy": float(velocity.y),
                "vz": float(velocity.z),
                "yaw": float(transform.rotation.yaw),
            }
        except (AttributeError, TypeError, ValueError) as exc:
            raise ProtocolError(
                "cannot serialize CARLA actor {!r}: {}".format(name, exc)
            )

        for key in ("x", "y", "z", "vx", "vy", "vz", "yaw"):
            self._require_finite(state[key], "actor {} {}".format(name, key))
        return state

    def _validate_actor_map(
        self,
        actors: Mapping[str, Any],
        first_state: bool,
    ) -> Dict[str, Any]:
        if not isinstance(actors, Mapping) or not actors:
            raise ProtocolError(
                "actors must be a non-empty mapping of name to CARLA actor"
            )

        result: Dict[str, Any] = {}
        actor_ids: Set[int] = set()
        for raw_name, actor in actors.items():
            name = str(raw_name).strip()
            if not name:
                raise ProtocolError("actor names must not be empty")
            if name in result:
                raise ProtocolError("duplicate actor name {!r}".format(name))
            if actor is None:
                raise ProtocolError("actor {!r} is None".format(name))
            try:
                actor_id = int(actor.id)
            except (AttributeError, TypeError, ValueError):
                raise ProtocolError(
                    "actor {!r} has no valid CARLA id".format(name)
                )
            if actor_id in actor_ids:
                raise ProtocolError(
                    "CARLA actor id {} is bound more than once".format(actor_id)
                )
            actor_ids.add(actor_id)
            result[name] = actor

        if not first_state and tuple(result.keys()) != self._actor_names:
            raise ProtocolError(
                "actor mapping changed: expected {}, got {}".format(
                    list(self._actor_names),
                    list(result.keys()),
                )
            )
        return result

    def _validate_delivered_message(
        self,
        raw_message: Any,
    ) -> Dict[str, Any]:
        if not isinstance(raw_message, dict):
            raise ProtocolError("each delivered message must be a JSON object")

        message = dict(raw_message)
        message_id = self._message_id(message)
        source_actor = self._required_string(message, "source_actor")
        destination_actor = self._required_string(message, "destination_actor")
        receiver_actor = self._required_string(message, "receiver_actor")

        self._require_known_endpoint(source_actor, "source_actor")
        self._require_known_endpoint(destination_actor, "destination_actor")
        self._require_known_endpoint(receiver_actor, "receiver_actor")
        if receiver_actor != destination_actor:
            raise ProtocolError(
                "message {} expected receiver {}, but was received by {}".format(
                    message_id,
                    destination_actor,
                    receiver_actor,
                )
            )

        key = (message_id, receiver_actor)
        if key in self._delivered_message_keys:
            raise ProtocolError(
                "duplicate delivery of message {} to {}".format(
                    message_id,
                    receiver_actor,
                )
            )
        self._delivered_message_keys.add(key)
        return message

    def _validate_outgoing_message(
        self,
        raw_message: Mapping[str, Any],
    ) -> Dict[str, Any]:
        if not isinstance(raw_message, Mapping):
            raise ProtocolError("each outgoing message must be a JSON object")

        message = dict(raw_message)
        message_id = self._message_id(message)
        source_actor = self._required_string(message, "source_actor")
        destination_actor = self._required_string(message, "destination_actor")
        message_type = self._required_string(message, "message_type")
        default_generation_time = (
            self._pending_step.sim_time
            if self._pending_step is not None
            else self.sim_time
        )
        generation_time = float(
            message.get("generation_time", default_generation_time)
        )
        payload_size = int(message.get("payload_size", -1))

        self._require_known_endpoint(source_actor, "source_actor")
        self._require_known_endpoint(destination_actor, "destination_actor")
        self._require_finite(generation_time, "generation_time")
        if payload_size < 0:
            raise ProtocolError("payload_size must be non-negative")
        if message_id in self._generated_message_ids:
            raise ProtocolError(
                "duplicate outgoing message_id {}".format(message_id)
            )

        self._generated_message_ids.add(message_id)
        message.update(
            {
                "message_id": message_id,
                "source_actor": source_actor,
                "destination_actor": destination_actor,
                "message_type": message_type,
                "generation_time": generation_time,
                "payload_size": payload_size,
            }
        )
        return message

    def _message_id(self, message: Mapping[str, Any]) -> int:
        if "message_id" not in message:
            raise ProtocolError("message has no message_id")
        try:
            message_id = int(message["message_id"])
        except (TypeError, ValueError):
            raise ProtocolError("message_id must be an integer")
        if message_id < 0:
            raise ProtocolError("message_id must be non-negative")
        return message_id

    def _require_known_endpoint(self, actor_name: str, field: str) -> None:
        if actor_name not in self._network_endpoint_names:
            raise ProtocolError(
                "{} {!r} is not in configured network endpoints {}".format(
                    field,
                    actor_name,
                    sorted(self._network_endpoint_names),
                )
            )

    @staticmethod
    def _required_string(message: Mapping[str, Any], key: str) -> str:
        value = message.get(key)
        if value is None or not str(value).strip():
            raise ProtocolError("message has no {}".format(key))
        return str(value)

    def _validate_protocol_version(self, request: Mapping[str, Any]) -> None:
        version = request.get("protocol_version")
        if version != self.protocol_version:
            raise ProtocolError(
                "unsupported protocol_version {!r}; expected {}".format(
                    version,
                    self.protocol_version,
                )
            )

    def _validate_optional_protocol_version(
        self,
        request: Mapping[str, Any],
    ) -> None:
        if "protocol_version" in request:
            self._validate_protocol_version(request)

    @staticmethod
    def _positive_float(request: Mapping[str, Any], key: str) -> float:
        try:
            value = float(request[key])
        except (KeyError, TypeError, ValueError):
            raise ProtocolError("{} must be a positive number".format(key))
        if not math.isfinite(value) or value <= 0.0:
            raise ProtocolError("{} must be a positive number".format(key))
        return value

    @staticmethod
    def _require_finite(value: float, label: str) -> None:
        if not math.isfinite(float(value)):
            raise ProtocolError("{} must be finite".format(label))

    @staticmethod
    def _require_close(actual: float, expected: float, label: str) -> None:
        tolerance = max(1e-9, abs(expected) * 1e-9)
        if abs(actual - expected) > tolerance:
            raise ProtocolError(
                "{}: got {}, expected {}".format(label, actual, expected)
            )

    def _ensure_open(self) -> None:
        if self._closed:
            raise Simu5gBackendError("backend is closed")

    def _ensure_ready(self) -> None:
        self._ensure_open()
        if not self._initialized:
            raise Simu5gBackendError("initialize() must be called first")
        if self._remote_stopped:
            raise Simu5gBackendError("OMNeT++ has already stopped")

    def _open_message_log(self, message_log_path: str) -> None:
        path = Path(message_log_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._message_log_file = path.open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self._message_log_writer = csv.DictWriter(
            self._message_log_file,
            fieldnames=[
                "event",
                "message_id",
                "message_type",
                "source_actor",
                "destination_actor",
                "receiver_actor",
                "generation_time",
                "network_receive_time",
                "python_delivery_time",
                "payload_size",
            ],
        )
        self._message_log_writer.writeheader()
        self._message_log_file.flush()

    def _log_message(
        self,
        event: str,
        message: Mapping[str, Any],
        python_delivery_time: Any = "",
    ) -> None:
        if self._message_log_writer is None:
            return
        self._message_log_writer.writerow(
            {
                "event": event,
                "message_id": message.get("message_id", ""),
                "message_type": message.get("message_type", ""),
                "source_actor": message.get("source_actor", ""),
                "destination_actor": message.get("destination_actor", ""),
                "receiver_actor": message.get("receiver_actor", ""),
                "generation_time": message.get("generation_time", ""),
                "network_receive_time": message.get(
                    "network_receive_time",
                    "",
                ),
                "python_delivery_time": python_delivery_time,
                "payload_size": message.get("payload_size", ""),
            }
        )
        self._message_log_file.flush()


__all__ = [
    "PROTOCOL_VERSION",
    "InitRequest",
    "ProtocolError",
    "Simu5gBackend",
    "Simu5gBackendError",
    "StepRequest",
]
