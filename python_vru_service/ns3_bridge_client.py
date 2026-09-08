#
# ns3_bridge_client.py
#
# UDP client for the ns-3 "v2x-bridge" co-simulation program
# (ns-3-dev/scratch/v2x-bridge): it sends the JSON control datagrams that
# drive the simulated 5G NR network (scene/entity updates + one simulated
# packet per datagram) and collects the JSON replies (delivery confirmations,
# with the application payload as received at the destination node).
#
# Protocol rules implemented here (see the v2x-bridge README):
#   - one packet per datagram, each with a unique integer packet_id;
#   - request_reply is always true: every simulated packet produces a reply
#     (delivered / timeout / error) that is matched back via packet_id;
#   - the top-level "timestamp" carries the external (CARLA/OpenCDA)
#     simulation time in seconds: it drives the bridge's simulation clock in
#     its default external-clock mode;
#   - origin_IDs are stable, human-readable actor names (e.g. "Cav1",
#     "ped_3"); the reserved ID "BS" addresses the edge/MEC server;
#   - the application payload (VAM bytes, detection/warning JSON) is carried
#     base64-encoded in "packet.payload"; on delivery the bridge returns the
#     bytes extracted from the simulated packet at the destination node in
#     the reply's "payload" field, which is what gets dispatched to the
#     receiver-side handler.
#

import base64
import json
import select
import socket
import time


class Ns3BridgeClient:
    """Client of the ns-3 v2x-bridge control protocol (real UDP, JSON)."""

    def __init__(self, host="127.0.0.1", port=5555, verbose=False,
                 pending_expiry_wall_s=15.0):
        self.addr = (host, port)
        self.verbose = verbose
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self._next_packet_id = 1
        # packet_id -> dict(kind, sender, receiver, type, size, payload,
        #                   meta, on_delivered, on_failed, wall)
        self._pending = {}
        self._pending_expiry_wall_s = pending_expiry_wall_s
        self._dead_warned = False
        self.stats = {"sent": 0, "delivered": 0, "timeout": 0, "error": 0,
                      "expired": 0}
        self.latencies_ms = {}       # kind -> list of delivered latencies

    # ------------------------------------------------------------------
    def _send_json(self, message):
        self.sock.sendto(json.dumps(message).encode(), self.addr)

    def send_entities(self, timestamp, entities):
        """Scene-only datagram: refresh the position of the given entities
        (list of dicts in the bridge schema) at the given simulation time."""
        if not entities:
            return
        self._send_json({"msg_type": "control",
                         "timestamp": round(float(timestamp), 6),
                         "entities": entities})

    def send_packet(self, timestamp, sender, receiver, ptype, payload=b"",
                    entities=None, size_bytes=None, kind=None, meta=None,
                    on_delivered=None, on_failed=None):
        """Send one simulated packet command (one datagram, unique packet_id,
        request_reply always true). Returns the packet_id.

        payload are the actual application bytes to be carried inside the
        simulated packet; size_bytes defaults to the real simulated payload
        size (12-byte id+length header + application payload).
        on_delivered(reply_dict, payload_bytes, meta) is called when the
        bridge confirms the delivery (payload_bytes are the bytes extracted
        at the destination node); on_failed(reply_dict, meta) on any other
        outcome."""
        packet_id = self._next_packet_id
        self._next_packet_id += 1
        if size_bytes is None:
            size_bytes = 12 + len(payload)
        packet = {"sender": str(sender), "receiver": str(receiver),
                  "size_bytes": int(size_bytes), "packet_id": packet_id,
                  "type": str(ptype), "request_reply": True}
        if payload:
            packet["payload"] = base64.b64encode(bytes(payload)).decode("ascii")
        message = {"msg_type": "control",
                   "timestamp": round(float(timestamp), 6),
                   "packet": packet}
        if entities:
            message["entities"] = entities
        self._pending[packet_id] = {
            "kind": kind or ptype, "sender": str(sender),
            "receiver": str(receiver), "type": str(ptype),
            "size": int(size_bytes), "payload": bytes(payload),
            "meta": meta, "on_delivered": on_delivered,
            "on_failed": on_failed, "wall": time.monotonic()}
        self.stats["sent"] += 1
        self._send_json(message)
        if self.verbose:
            print("[NS3] --> id=%d %s -> %s (%s/%s, %d B)"
                  % (packet_id, sender, receiver, ptype, kind or ptype,
                     size_bytes))
        return packet_id

    # ------------------------------------------------------------------
    def _dispatch(self, reply):
        packet_id = reply.get("packet_id")
        pending = self._pending.pop(packet_id, None)
        if pending is None:
            return
        status = reply.get("status")
        if status == "delivered":
            self.stats["delivered"] += 1
            if reply.get("latency_ms") is not None:
                self.latencies_ms.setdefault(pending["kind"], []).append(
                    float(reply["latency_ms"]))
            # Application payload as received at the destination node inside
            # the simulated network; fall back to the sent bytes for
            # payload-less (metadata-only) packets.
            if "payload" in reply:
                payload = base64.b64decode(reply["payload"])
            else:
                payload = pending["payload"]
            if self.verbose:
                print("[NS3] <-- id=%d delivered (%.3f ms)"
                      % (packet_id, reply.get("latency_ms") or -1))
            if pending["on_delivered"] is not None:
                pending["on_delivered"](reply, payload, pending["meta"])
        else:
            key = "timeout" if status == "timeout" else "error"
            self.stats[key] += 1
            print("[NS3] packet id=%s %s -> %s (%s) failed: %s%s"
                  % (packet_id, pending["sender"], pending["receiver"],
                     pending["kind"], status,
                     " (%s)" % reply["error_msg"] if "error_msg" in reply
                     else ""))
            if pending["on_failed"] is not None:
                pending["on_failed"](reply, pending["meta"])

    def _drain(self):
        """Process every reply currently queued on the socket; return the
        number of dispatched replies."""
        dispatched = 0
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                reply = json.loads(data.decode())
            except ValueError:
                print("[NS3] malformed reply datagram discarded")
                continue
            if reply.get("msg_type") == "reply":
                self._dispatch(reply)
                dispatched += 1
            elif reply.get("msg_type") == "shutdown_ack":
                pass
            else:
                print("[NS3] unexpected message discarded: %s"
                      % reply.get("msg_type"))
        return dispatched

    def _expire_stale(self):
        """Drop pending packets whose reply never arrived (bridge not
        running / unreachable): they must not block pump() forever."""
        now = time.monotonic()
        stale = [pid for pid, p in self._pending.items()
                 if now - p["wall"] > self._pending_expiry_wall_s]
        for pid in stale:
            pending = self._pending.pop(pid)
            self.stats["expired"] += 1
            if pending["on_failed"] is not None:
                pending["on_failed"]({"status": "no_reply", "packet_id": pid},
                                     pending["meta"])
        if stale and not self._dead_warned:
            self._dead_warned = True
            print("[NS3] WARNING: %d packet(s) got no reply within %.1f s - "
                  "is the v2x-bridge running on udp://%s:%d?"
                  % (len(stale), self._pending_expiry_wall_s,
                     self.addr[0], self.addr[1]))

    def pump(self, max_wait_s=0.0):
        """Dispatch all the available replies. With max_wait_s > 0, keep
        waiting (bounded) until every pending packet has been resolved -
        the bridge resolves deliveries within its drain window right after
        the command datagrams, so this normally returns in a few ms."""
        self._drain()
        deadline = time.monotonic() + max_wait_s
        while self._pending and time.monotonic() < deadline:
            timeout = min(0.005, max(0.0, deadline - time.monotonic()))
            readable, _, _ = select.select([self.sock], [], [], timeout)
            if readable:
                self._drain()
        self._expire_stale()

    @property
    def pending_count(self):
        return len(self._pending)

    # ------------------------------------------------------------------
    def close(self, send_shutdown=False):
        if send_shutdown:
            try:
                self._send_json({"msg_type": "shutdown"})
            except OSError:
                pass
        self.sock.close()

    def summary_lines(self):
        lines = ["ns-3 bridge : %d sent, %d delivered, %d timeout, "
                 "%d error, %d without reply"
                 % (self.stats["sent"], self.stats["delivered"],
                    self.stats["timeout"], self.stats["error"],
                    self.stats["expired"])]
        for kind in sorted(self.latencies_ms):
            lat = self.latencies_ms[kind]
            lines.append("  %-10s: %d delivered, latency mean %.2f ms "
                         "(min %.2f, max %.2f)"
                         % (kind, len(lat), sum(lat) / len(lat),
                            min(lat), max(lat)))
        return lines
