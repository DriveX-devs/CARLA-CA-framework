"""Bridge-compatible transport adapter for the lockstep Simu5G backend.

``V2XLayer`` was originally written against :class:`Ns3BridgeClient`.  This
adapter implements the small interface that layer uses without changing the
existing ns-3 client or its UDP protocol:

* ``send_entities()`` -- a no-op because CARLA mobility is already included
  in every Simu5G ``STATE`` response;
* ``send_packet()`` -- queue one application packet for ``complete_step()``;
* ``pump()`` -- dispatch packets that OMNeT++ reported as delivered in the
  current ``STEP`` request;
* ``summary_lines()`` and ``close()`` -- retain the existing reporting and
  lifecycle shape.

Application bytes are base64 encoded in the ZeroMQ JSON envelope.  They are
released to the V2X receiver callback only after the simulated packet reaches
its destination.  Packets that never receive a delivery report expire in
simulation time and invoke their failure callback.
"""

from __future__ import annotations

import base64
from typing import Any, Dict, Iterable, List, Mapping, Optional


class Simu5gBridgeClient:
    """Adapt :class:`Simu5gBackend` to the existing V2X bridge interface."""

    backend_name = "Simu5G"

    def __init__(
        self,
        backend: Any,
        verbose: bool = False,
        packet_timeout_s: float = 1.0,
    ) -> None:
        if packet_timeout_s <= 0.0:
            raise ValueError("packet_timeout_s must be positive")

        self.backend = backend
        self.verbose = bool(verbose)
        self.packet_timeout_s = float(packet_timeout_s)
        self._pending: Dict[int, Dict[str, Any]] = {}
        self._outgoing: List[Dict[str, Any]] = []
        self._deliveries: List[Dict[str, Any]] = []
        self._sim_time = 0.0
        self._closed = False

        self.stats = {
            "sent": 0,
            "delivered": 0,
            "timeout": 0,
            "error": 0,
            "expired": 0,
        }
        self.latencies_ms: Dict[str, List[float]] = {}

    def send_entities(
        self,
        timestamp: float,
        entities: Iterable[Mapping[str, Any]],
    ) -> None:
        """Accept the legacy scene update without duplicating mobility.

        ``Simu5gBackend.complete_step()`` already sends every CAV position and
        velocity to ``CarlaCoordinator``.  Sending a second entity channel is
        therefore unnecessary for this transport.
        """

        self._sim_time = float(timestamp)
        del entities

    def send_packet(
        self,
        timestamp: float,
        sender: str,
        receiver: str,
        ptype: str,
        payload: bytes = b"",
        entities: Optional[Iterable[Mapping[str, Any]]] = None,
        size_bytes: Optional[int] = None,
        kind: Optional[str] = None,
        meta: Optional[Mapping[str, Any]] = None,
        on_delivered: Optional[Any] = None,
        on_failed: Optional[Any] = None,
    ) -> int:
        """Queue one packet for transmission in the pending CARLA step."""

        if self._closed:
            raise RuntimeError("Simu5G bridge client is closed")

        del entities
        payload_bytes = bytes(payload)
        if size_bytes is None:
            size_bytes = 12 + len(payload_bytes)

        message = self.backend.make_message(
            source_actor=str(sender),
            destination_actor=str(receiver),
            message_type=str(ptype),
            payload_size=int(size_bytes),
            generation_time=float(timestamp),
            payload={
                "encoding": "base64",
                "data": base64.b64encode(payload_bytes).decode("ascii"),
            },
        )
        packet_id = int(message["message_id"])
        self._pending[packet_id] = {
            "kind": str(kind or ptype),
            "sender": str(sender),
            "receiver": str(receiver),
            "type": str(ptype),
            "size": int(size_bytes),
            "payload": payload_bytes,
            "meta": dict(meta) if meta is not None else None,
            "on_delivered": on_delivered,
            "on_failed": on_failed,
            "generation_time": float(timestamp),
        }
        self._outgoing.append(message)
        self.stats["sent"] += 1

        if self.verbose:
            print(
                "[SIMU5G] --> id={} {} -> {} ({}/{}, {} B)".format(
                    packet_id,
                    sender,
                    receiver,
                    ptype,
                    kind or ptype,
                    size_bytes,
                ),
                flush=True,
            )
        return packet_id

    def accept_deliveries(
        self,
        sim_time: float,
        delivered_messages: Iterable[Mapping[str, Any]],
    ) -> None:
        """Stage one STEP request's delivery reports for the next ``pump``."""

        self._sim_time = float(sim_time)
        for message in delivered_messages:
            self._deliveries.append(dict(message))

    def take_outgoing(self) -> List[Dict[str, Any]]:
        """Return and clear packets queued since the previous STATE reply."""

        messages = self._outgoing
        self._outgoing = []
        return messages

    def pump(self, max_wait_s: float = 0.0) -> None:
        """Dispatch staged Simu5G deliveries and expire overdue packets.

        ``max_wait_s`` is intentionally ignored: OMNeT++ advances only at the
        ZeroMQ lockstep boundary, so no additional delivery can arrive while
        Python is processing the current step.
        """

        del max_wait_s
        deliveries = self._deliveries
        self._deliveries = []
        for message in deliveries:
            self._dispatch(message)
        self._expire_stale()

    def _dispatch(self, message: Mapping[str, Any]) -> None:
        try:
            packet_id = int(message["message_id"])
        except (KeyError, TypeError, ValueError):
            self.stats["error"] += 1
            print("[SIMU5G] delivery without a valid message_id discarded")
            return

        pending = self._pending.pop(packet_id, None)
        if pending is None:
            self.stats["error"] += 1
            print(
                "[SIMU5G] delivery for unknown packet id={} discarded".format(
                    packet_id
                )
            )
            return

        receive_time = float(
            message.get("network_receive_time", self._sim_time)
        )
        latency_ms = max(
            0.0,
            (receive_time - pending["generation_time"]) * 1000.0,
        )
        payload = self._decode_payload(message, pending["payload"])
        reply = {
            "msg_type": "reply",
            "status": "delivered",
            "packet_id": packet_id,
            "latency_ms": latency_ms,
            "sender": pending["sender"],
            "receiver": pending["receiver"],
        }

        self.stats["delivered"] += 1
        self.latencies_ms.setdefault(pending["kind"], []).append(latency_ms)
        if self.verbose:
            print(
                "[SIMU5G] <-- id={} delivered ({:.3f} ms)".format(
                    packet_id,
                    latency_ms,
                ),
                flush=True,
            )
        if pending["on_delivered"] is not None:
            pending["on_delivered"](reply, payload, pending["meta"])

    @staticmethod
    def _decode_payload(
        message: Mapping[str, Any],
        fallback: bytes,
    ) -> bytes:
        payload = message.get("payload")
        if not isinstance(payload, Mapping):
            return fallback
        if payload.get("encoding") != "base64":
            return fallback
        try:
            return base64.b64decode(str(payload.get("data", "")))
        except (TypeError, ValueError):
            return fallback

    def _expire_stale(self) -> None:
        stale = [
            packet_id
            for packet_id, pending in self._pending.items()
            if self._sim_time - pending["generation_time"]
            >= self.packet_timeout_s
        ]
        for packet_id in stale:
            pending = self._pending.pop(packet_id)
            self.stats["expired"] += 1
            reply = {
                "msg_type": "reply",
                "status": "timeout",
                "packet_id": packet_id,
            }
            if pending["on_failed"] is not None:
                pending["on_failed"](reply, pending["meta"])
            if self.verbose:
                print(
                    "[SIMU5G] packet id={} {} -> {} ({}) timed out".format(
                        packet_id,
                        pending["sender"],
                        pending["receiver"],
                        pending["kind"],
                    ),
                    flush=True,
                )

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def close(self, send_shutdown: bool = False) -> None:
        del send_shutdown
        self._pending.clear()
        self._deliveries = []
        self._outgoing = []
        self._closed = True

    def summary_lines(self) -> List[str]:
        lines = [
            "Simu5G bridge : {} sent, {} delivered, {} timeout, "
            "{} error, {} expired".format(
                self.stats["sent"],
                self.stats["delivered"],
                self.stats["timeout"],
                self.stats["error"],
                self.stats["expired"],
            )
        ]
        for kind in sorted(self.latencies_ms):
            latencies = self.latencies_ms[kind]
            lines.append(
                "  {:<10}: {} delivered, latency mean {:.2f} ms "
                "(min {:.2f}, max {:.2f})".format(
                    kind,
                    len(latencies),
                    sum(latencies) / len(latencies),
                    min(latencies),
                    max(latencies),
                )
            )
        return lines


__all__ = ["Simu5gBridgeClient"]
