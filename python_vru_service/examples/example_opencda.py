#!/usr/bin/env python3
#
# example_opencda.py - V2X simulation tool on top of an OpenCDA-driven CARLA
# server (reference OpenCDA fork: https://github.com/DriveX-devs/OpenCDA)
#
# This tool connects to the CARLA server ALREADY EXECUTED (and ticked) by
# OpenCDA: it never calls world.tick() and never spawns actors. It waits for
# the simulation steps with world.wait_for_tick() and discovers the actors
# that OpenCDA has already spawned, attaching a V2X role to each of them:
#
#  1. CAVs (sensorized vehicles): every four-wheeled vehicle which is not
#     background traffic (see --cav-role-names/--background-role-names/
#     --cav-ids). Each CAV keeps its own local LDM. The detections filling it
#     NEVER come from CARLA ground truth: they ALWAYS come from the OpenCDA
#     CA application (OpenCDA/CA: YOLOv8 detection + camera/LiDAR frustum
#     fusion + Kalman tracking), which streams its confirmed tracks to this
#     tool over a localhost UDP JSON feed (--detections-port, see the
#     "REQUIRED PATCH" block below). A callback prints every successful
#     insert/update, and every detection is forwarded to the edge server.
#
#  2. VRUs (pedestrian walkers and two-wheelers): each runs its own VRU Basic
#     Service which manages the VAM transmission (ETSI TS 103 300-3 triggering
#     conditions, simulation-time reference, manual stepping after every
#     tick). The VRU kinematic state is read from its own CARLA actor - this
#     is legitimate self-knowledge (the GNSS/IMU of the VRU device), not
#     perception. VAM transmission is controlled by --vams/--no-vams.
#
#  3. A single edge server: an LDM with object matching enabled, fed with the
#     CAV detections and the received VAMs. On every successful update its
#     callback prints the information about the object just inserted and the
#     distance from all the other objects in the LDM; when a distance drops
#     below --warning-distance, a "control" JSON packet (see the README) is
#     prepared and printed for BOTH objects involved.
#
# The places where the ns-3 network simulator will replace the current ideal
# (direct-call / print) links are marked with "NS-3 INTEGRATION POINT (n/4)"
# comment banners.
#
# Usage (with OpenCDA + the CA application running on localhost:2000):
#     python3 example_opencda.py [--host localhost] [--port 2000] \
#         [--detections-port 47500] [--warning-distance 10] [--no-vams]
#
# Without the carla package (or with --mock) the tool runs against a built-in
# mock world AND a built-in mock of the CA application feed (same UDP JSON
# interface), so the whole pipeline can be tested anywhere.
#
# ---------------------------------------------------------------------------
# REQUIRED PATCH IN THE OpenCDA CA APPLICATION (OpenCDA/CA/run_ca.py)
# ---------------------------------------------------------------------------
# The CAV detections consumed by this tool always come from the CA application
# (YOLOv8 + fusion + tracking). run_ca.py must therefore stream its confirmed
# tracks to this tool. Add once, near the top of run_ca.py:
#
#     import json, socket
#     V2X_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     V2X_ADDR = ("127.0.0.1", 47500)        # = --detections-port of this tool
#
# and, inside the per-CAV main loop, right after the line
#     tracks = cav['tracker'].step(merged)
# add:
#
#     V2X_SOCK.sendto(json.dumps({
#         "cav": cav['name'],
#         "cav_actor_id": cav['vehicle'].id,
#         "t": world.get_snapshot().timestamp.elapsed_seconds,
#         "tracks": [{
#             "track_id": int(t.track_id),
#             "category": t.category,            # vehicle | bike | pedestrian
#             "x": float(t.position[0]),         # CARLA world coordinates [m]
#             "y": float(t.position[1]),
#             "z": float(t.position[2]),
#             "speed": float(t.speed),           # [m/s]
#             "heading": float(t.heading) if t.heading_valid else None,
#         } for t in tracks],                    # heading: CARLA yaw [deg]
#     }).encode(), V2X_ADDR)
# ---------------------------------------------------------------------------

import argparse
import json
import math
import os
import random
import socket
import sys

# Allow running this file from anywhere: add the package root (parent folder)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ldm import LDM
from vru_basic_service import VRUBasicService
import vam_codec
import geo_utils

SIM_STEP_S = 0.05                 # mock world step (matches OpenCDA's 20 Hz)
MOCK_DURATION_S = 8.0
DEFAULT_DETECTIONS_PORT = 47500

# CARLA bicycle blueprints (same list used by the OpenCDA dataset generator)
BICYCLE_TYPE_IDS = ("vehicle.bh.crossbike", "vehicle.diamondback.century",
                    "vehicle.gazelle.omafiets")

STATION_TYPE_NAMES = {
    vam_codec.STATION_TYPE_UNKNOWN: "unknown",
    vam_codec.STATION_TYPE_PEDESTRIAN: "pedestrian",
    vam_codec.STATION_TYPE_CYCLIST: "bicyclist",
    vam_codec.STATION_TYPE_MOPED: "moped",
    vam_codec.STATION_TYPE_MOTORCYCLE: "motorcyclist",
    vam_codec.STATION_TYPE_PASSENGER_CAR: "cav",
}

# Categories used by the CA application tracker -> ETSI station types
CA_CATEGORY_TO_STATION_TYPE = {
    "vehicle": vam_codec.STATION_TYPE_PASSENGER_CAR,
    "bike": vam_codec.STATION_TYPE_CYCLIST,
    "pedestrian": vam_codec.STATION_TYPE_PEDESTRIAN,
}


# ---------------------------------------------------------------------------
# CARLA helpers (left-handed CARLA frame -> ENU x/y, compass heading)
# ---------------------------------------------------------------------------
def carla_xy(actor):
    loc = actor.get_location()
    return loc.x, -loc.y


def carla_speed(actor):
    vel = actor.get_velocity()
    return math.hypot(vel.x, vel.y)


def carla_heading(actor):
    # CARLA yaw: 0 = +x (east), clockwise -> compass heading from north
    return (actor.get_transform().rotation.yaw + 90.0) % 360.0


def carla_long_acceleration(actor):
    """Longitudinal acceleration (projection on the velocity direction), or
    None when the actor is almost still (direction undefined)."""
    try:
        acc = actor.get_acceleration()
        vel = actor.get_velocity()
    except AttributeError:
        return None
    speed = math.hypot(vel.x, vel.y)
    if speed < 0.1:
        return None
    return (acc.x * vel.x + acc.y * vel.y) / speed


def classify_actor(actor, cav_role_names, background_role_names, cav_ids):
    """Return the ETSI station type of a CARLA actor, or None if it must be
    ignored (background traffic, sensors, traffic lights, ...)."""
    type_id = actor.type_id
    if type_id.startswith("walker.pedestrian"):
        return vam_codec.STATION_TYPE_PEDESTRIAN
    if not type_id.startswith("vehicle."):
        return None

    attributes = getattr(actor, "attributes", {}) or {}
    role = attributes.get("role_name", "")

    # Two-wheelers are VRUs (bicycles from the known list, motorbikes otherwise)
    wheels = attributes.get("number_of_wheels", "4")
    if type_id in BICYCLE_TYPE_IDS or role == "bike":
        return vam_codec.STATION_TYPE_CYCLIST
    if str(wheels) == "2":
        return vam_codec.STATION_TYPE_MOTORCYCLE

    # Four-wheelers: CAV unless marked as background traffic
    if actor.id in cav_ids:
        return vam_codec.STATION_TYPE_PASSENGER_CAR
    if cav_role_names:
        return vam_codec.STATION_TYPE_PASSENGER_CAR if role in cav_role_names else None
    if role in background_role_names:
        return None
    return vam_codec.STATION_TYPE_PASSENGER_CAR


class SimClock:
    """Shared simulation-time reference, updated at every received tick."""

    def __init__(self):
        self.t = 0.0


# ---------------------------------------------------------------------------
# (1) Sensorized CAV: local LDM fed by the CA application detections
# ---------------------------------------------------------------------------
class CavManager:
    """Local LDM of one CAV. The detections NEVER come from CARLA ground
    truth: they always come from the OpenCDA CA application (YOLOv8 +
    camera/LiDAR fusion + Kalman tracking) through the UDP JSON feed."""

    def __init__(self, station_id, name, edge, quiet):
        self.station_id = station_id
        self.name = name
        self.edge = edge
        self.ldm = LDM()
        if not quiet:
            self.ldm.register_callback(self._print_callback)

    def _print_callback(self, station_id, station_type, position, speed,
                        heading, acceleration):
        # =================================================================
        # NS-3 INTEGRATION POINT (1/4) - CAV local LDM update notification
        #
        # For now this callback just prints. To integrate with ns-3, replace
        # the print below with a localhost socket call towards the ns-3
        # process (one UDP/TCP socket per CAV), sending the updated object
        # (stationID, type, position, speed, heading, acceleration) so that
        # ns-3 can generate the corresponding V2X message (e.g. a CPM) of
        # this CAV inside the simulated network.
        # =================================================================
        print("[CAV %s LDM] object %d (%s): position=%s(%.2f, %.2f) "
              "speed=%s heading=%s acceleration=%s"
              % (self.name, station_id,
                 STATION_TYPE_NAMES.get(station_type, station_type),
                 position.kind, position.c1, position.c2,
                 "%.2f m/s" % speed if speed is not None else "N/A",
                 "%.1f deg" % heading if heading is not None else "N/A",
                 "%.2f m/s^2" % acceleration if acceleration is not None else "N/A"))

    def on_tracks(self, tracks):
        """Handle one message of the CA application feed: insert/update every
        confirmed track in the local LDM and forward it to the edge server."""
        for track in tracks:
            station_type = CA_CATEGORY_TO_STATION_TYPE.get(
                track.get("category"), vam_codec.STATION_TYPE_UNKNOWN)
            # CARLA world frame -> ENU (x = east, y = north)
            x = float(track["x"])
            y = -float(track["y"])
            speed = track.get("speed")
            heading = track.get("heading")
            if heading is not None:
                # CARLA yaw -> compass heading from north
                heading = (float(heading) + 90.0) % 360.0
            # CAV-local object ID, unique at the edge across all the CAVs
            object_id = self.station_id * 100000 + int(track["track_id"])

            self.ldm.insert(object_id, station_type, x=x, y=y, speed=speed,
                            heading=heading, detected=True,
                            perceived_by=self.station_id)

            # =============================================================
            # NS-3 INTEGRATION POINT (2/4) - CAV -> edge detection upload
            #
            # For now the detection reaches the edge server through a direct
            # function call (ideal V2I link). To integrate with ns-3, replace
            # this call with the transmission of the detection through the
            # ns-3 simulated network (CAV node -> base station/edge node):
            # send the payload to the ns-3 process over a localhost socket,
            # and deliver it to EdgeServer.on_detection() only when (and if)
            # ns-3 reports the packet as received.
            # =============================================================
            self.edge.on_detection(object_id, station_type, x, y, speed,
                                   heading, None, self.station_id)


class DetectionFeedReceiver:
    """Receives the JSON messages streamed by the OpenCDA CA application
    (see the REQUIRED PATCH block at the top of this file) and routes them
    to the CavManager of the sending CAV."""

    def __init__(self, port, registry):
        self.registry = registry
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        print("[TOOL] listening for CA application detections on udp://127.0.0.1:%d"
              % port)

    def drain(self):
        """Process all the pending feed messages (called once per tick)."""
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                message = json.loads(data.decode())
                self.registry.on_ca_message(message)
            except (ValueError, KeyError, TypeError) as e:
                print("[TOOL] malformed CA feed message discarded (%s)" % e)

    def close(self):
        self.sock.close()


# ---------------------------------------------------------------------------
# (2) VRU: VAM transmission through the VRU Basic Service
# ---------------------------------------------------------------------------
class VruManager:
    def __init__(self, actor, station_type, carla_map, clock, edge, geo_origin,
                 quiet):
        self.actor = actor
        self.carla_map = carla_map
        self.quiet = quiet
        self.edge = edge
        self.srv = VRUBasicService(station_id=actor.id,
                                   station_type=station_type,
                                   proximity_check=False,
                                   ref_lat=geo_origin[0], ref_lon=geo_origin[1])
        # Time reference: the shared simulation clock, NOT the wall clock
        self.srv.set_timestamp_callback(lambda: clock.t)
        self.srv.set_tx_callback(self._on_tx)
        self._initialized = False

    def _on_tx(self, encoded_vam):
        if not self.quiet:
            print("[VRU %d] VAM TX trigger=%-19s %d bytes: %s"
                  % (self.srv.m_station_id, self.srv.m_trigg_cond.name,
                     len(encoded_vam), encoded_vam.hex()))
        # =================================================================
        # NS-3 INTEGRATION POINT (3/4) - VRU VAM transmission
        #
        # For now the UPER-encoded VAM is delivered to the edge server with
        # a direct function call (ideal link). To integrate with ns-3, send
        # the encoded VAM bytes to the ns-3 process over a localhost socket
        # (VRU node -> radio access network), and deliver them to
        # EdgeServer.on_vam() only when (and if) ns-3 reports the packet as
        # received by the edge/base station node.
        # =================================================================
        self.edge.on_vam(encoded_vam)

    def step(self):
        # The VRU reads its own kinematic state from its CARLA actor: this is
        # self-knowledge (GNSS/IMU of the VRU device), not perception
        geo = self.carla_map.transform_to_geolocation(self.actor.get_location())
        self.srv.set_ego_vru_kstatus(
            self.srv.m_station_id, self.srv.m_stationtype,
            lat=geo.latitude, lon=geo.longitude,
            speed=carla_speed(self.actor),
            heading=carla_heading(self.actor),
            acceleration=carla_long_acceleration(self.actor))
        if not self._initialized:
            self.srv.init_dissemination()     # initial VAM (DISSEMINATION_START)
            self._initialized = True
        else:
            self.srv.check_vam_conditions()   # manual stepping, one check per tick


# ---------------------------------------------------------------------------
# (3) Edge server: LDM fed with detections + VAMs, proximity warnings
# ---------------------------------------------------------------------------
class EdgeServer:
    # Entries closer than this are considered two representations of the same
    # physical object: no warning is generated between them
    SAME_OBJECT_EPSILON_M = 2.0
    # A detected entry and a connected (VAM) entry of the same station type are
    # considered the same physical object up to this distance: the VAM position
    # lags behind the live detection by up to the ETSI position-change
    # triggering threshold (4 m)
    DETECTED_VS_CONNECTED_EPSILON_M = 4.5

    def __init__(self, geo_converter, clock, warning_distance, warning_cooldown,
                 entities_provider, quiet):
        self.geo_converter = geo_converter
        self.clock = clock
        self.warning_distance = warning_distance
        self.warning_cooldown = warning_cooldown
        self.entities_provider = entities_provider
        self.quiet = quiet
        self.ldm = LDM()
        self.ldm.enable_object_matching(match_distance_m=2.0)
        self.ldm.register_callback(self._on_update)
        self._pair_last_warning = {}    # frozenset({sid_a, sid_b}) -> sim time
        self._packet_id = 0
        self.warnings_sent = 0

    # ---- inputs ----------------------------------------------------------
    def on_detection(self, local_id, station_type, x, y, speed, heading,
                     acceleration, cav_station_id):
        # With ns-3 in the loop, this method becomes the handler of the
        # detections received FROM the ns-3 simulated network (see the NS-3
        # INTEGRATION POINT (2/4) banner in CavManager.on_tracks())
        self.ldm.insert(local_id, station_type, x=x, y=y, speed=speed,
                        heading=heading, acceleration=acceleration,
                        detected=True, perceived_by=cav_station_id)

    def on_vam(self, encoded_vam):
        # With ns-3 in the loop, this method becomes the handler of the VAMs
        # received FROM the ns-3 simulated network (see the NS-3 INTEGRATION
        # POINT (3/4) banner in VruManager._on_tx())
        self.ldm.add_vru_from_vam(encoded_vam, geo_converter=self.geo_converter)

    # ---- LDM callback ----------------------------------------------------
    def _on_update(self, station_id, station_type, position, speed, heading,
                   acceleration):
        obj = self.ldm.lookup(station_id)
        if obj is None or obj.x is None:
            return

        # Distance of the object just inserted from all the other LDM objects
        distances = []
        for other_id in self.ldm.get_all_ids():
            if other_id == station_id:
                continue
            other = self.ldm.lookup(other_id)
            if other is None or other.x is None:
                continue
            distances.append((other_id, other,
                              math.hypot(other.x - obj.x, other.y - obj.y)))

        if not self.quiet:
            print("[EDGE LDM] station %d (%s)%s: position=(%.2f, %.2f) "
                  "speed=%s heading=%s | distances: %s"
                  % (station_id,
                     STATION_TYPE_NAMES.get(station_type, station_type),
                     " det.by %s" % sorted(obj.perceived_by) if obj.perceived_by
                     else " (VAM)",
                     obj.x, obj.y,
                     "%.2f m/s" % speed if speed is not None else "N/A",
                     "%.1f deg" % heading if heading is not None else "N/A",
                     ", ".join("%d: %.1f m" % (oid, d)
                               for oid, _, d in distances) or "none"))

        # Proximity warnings
        for other_id, other, d in distances:
            if d < self.warning_distance and not self._same_object(obj, other, d):
                self._maybe_warn(station_id, other_id, d)

    def _same_object(self, obj_a, obj_b, distance):
        """Heuristic telling whether two LDM entries are likely two
        representations of the same physical object."""
        if distance < self.SAME_OBJECT_EPSILON_M:
            return True
        return (obj_a.detected != obj_b.detected
                and obj_a.station_type == obj_b.station_type
                and distance < self.DETECTED_VS_CONNECTED_EPSILON_M)

    # ---- warning JSON ----------------------------------------------------
    def _maybe_warn(self, sid_a, sid_b, distance):
        pair = frozenset((sid_a, sid_b))
        last = self._pair_last_warning.get(pair)
        if last is not None and (self.clock.t - last) < self.warning_cooldown:
            return
        self._pair_last_warning[pair] = self.clock.t

        print("[EDGE] WARNING: stations %d and %d are %.2f m apart "
              "(< %.1f m) at t=%.2f s"
              % (sid_a, sid_b, distance, self.warning_distance, self.clock.t))

        entities = self.entities_provider()
        # One warning packet for each of the two objects involved
        for receiver in (sid_a, sid_b):
            self._packet_id += 1
            self.warnings_sent += 1
            message = {
                "msg_type": "control",
                "timestamp": round(self.clock.t, 3),
                "entities": entities,
                "packet": {
                    "sender": "BS",
                    "receiver": str(receiver),
                    "size_bytes": 200,
                    "packet_id": self._packet_id,
                    "type": "warning",
                    "request_reply": False,
                },
            }
            # =============================================================
            # NS-3 INTEGRATION POINT (4/4) - edge warning downlink
            #
            # For now the "control" JSON packet is just printed. To integrate
            # with ns-3, send this JSON to the ns-3 process over a localhost
            # socket: ns-3 will simulate the downlink transmission of the
            # warning from the base station ("sender": "BS") to the receiver
            # entity ("receiver" field), with the simulated size
            # ("size_bytes") and packet id.
            # =============================================================
            print(json.dumps(message, indent=2))


# ---------------------------------------------------------------------------
# Actor discovery / bookkeeping
# ---------------------------------------------------------------------------
class ActorRegistry:
    """Discovers the actors spawned by OpenCDA and keeps one manager per actor.
    rescan() is called periodically to attach to newly spawned actors too.
    CavManagers are created lazily, when the first CA-feed message of the
    corresponding CAV arrives."""

    def __init__(self, world, args, clock, edge, geo_origin):
        self.world = world
        self.args = args
        self.clock = clock
        self.edge = edge
        self.geo_origin = geo_origin
        self.carla_map = world.get_map()
        self.cavs = {}       # CAV actor id (from the CA feed) -> CavManager
        self.vrus = {}       # actor id -> VruManager
        self.actors = {}     # actor id -> (actor, station type)

    def rescan(self):
        cav_role_names = [r for r in self.args.cav_role_names.split(",") if r]
        background_role_names = [r for r in
                                 self.args.background_role_names.split(",") if r]
        cav_ids = {int(i) for i in self.args.cav_ids.split(",") if i}

        alive = set()
        for actor in self.world.get_actors():
            station_type = classify_actor(actor, cav_role_names,
                                          background_role_names, cav_ids)
            if station_type is None:
                continue
            alive.add(actor.id)
            if actor.id in self.actors:
                continue
            self.actors[actor.id] = (actor, station_type)
            if station_type == vam_codec.STATION_TYPE_PASSENGER_CAR:
                print("[TOOL] tracking CAV %d (%s, role '%s') - detections "
                      "expected from the CA application feed"
                      % (actor.id, actor.type_id,
                         (getattr(actor, "attributes", {}) or {})
                         .get("role_name", "")))
            else:
                if self.args.vams:
                    self.vrus[actor.id] = VruManager(
                        actor, station_type, self.carla_map, self.clock,
                        self.edge, self.geo_origin, self.args.quiet)
                print("[TOOL] attached to VRU %d (%s, %s)%s"
                      % (actor.id, actor.type_id,
                         STATION_TYPE_NAMES[station_type],
                         "" if self.args.vams else " - VAMs disabled"))

        # Drop the managers of destroyed actors
        for aid in list(self.actors):
            actor = self.actors[aid][0]
            if aid not in alive and not getattr(actor, "is_alive", True):
                self.actors.pop(aid, None)
                self.cavs.pop(aid, None)
                self.vrus.pop(aid, None)

    def on_ca_message(self, message):
        """Route one JSON message of the CA application feed to the manager of
        the sending CAV (created on first sight)."""
        cav_id = int(message["cav_actor_id"])
        name = str(message.get("cav", cav_id))
        manager = self.cavs.get(cav_id)
        if manager is None:
            manager = CavManager(cav_id, name, self.edge, self.args.quiet)
            self.cavs[cav_id] = manager
            print("[TOOL] CA detection feed active for %s (actor %d)"
                  % (name, cav_id))
        manager.on_tracks(message.get("tracks", []))

    def tracked(self):
        """All the tracked (actor, station_type) pairs (CAVs + VRUs)."""
        return list(self.actors.values())

    def entities(self):
        """Current position of all the spawned objects (CAVs, VRUs), in the
        format required by the warning JSON."""
        ents = []
        for actor, station_type in self.tracked():
            loc = actor.get_location()
            ents.append({
                "timestamp": round(self.clock.t, 3),
                "origin_ID": str(actor.id),
                "origin_vehicle_type": STATION_TYPE_NAMES[station_type],
                "Position": {"x_m": round(loc.x, 3), "y_m": round(loc.y, 3),
                             "z_m": round(loc.z, 3)},
                "Velocity": round(carla_speed(actor), 3),
                "Heading": round(carla_heading(actor), 2),
            })
        return ents


# ---------------------------------------------------------------------------
# Main loop (shared by the real-CARLA and mock paths)
# ---------------------------------------------------------------------------
def run(world, args, mock_ca_feed=None):
    clock = SimClock()
    carla_map = world.get_map()

    # Geo-origin of the map: reference of the shared ENU frame, so that the
    # x/y computed from the received VAMs matches the CAV detections
    origin_geo = carla_map.transform_to_geolocation(
        type("Origin", (), {"x": 0.0, "y": 0.0, "z": 0.0})())
    geo_origin = (origin_geo.latitude, origin_geo.longitude)
    geo_converter = geo_utils.GeoConverter(*geo_origin)

    registry_holder = {}
    edge = EdgeServer(geo_converter, clock, args.warning_distance,
                      args.warning_cooldown,
                      lambda: registry_holder["registry"].entities(),
                      args.quiet)
    registry = ActorRegistry(world, args, clock, edge, geo_origin)
    registry_holder["registry"] = registry
    feed = DetectionFeedReceiver(args.detections_port, registry)
    registry.rescan()

    print("[TOOL] running (waiting for OpenCDA ticks)...")
    start_time = None
    step = 0
    try:
        while True:
            try:
                snapshot = world.wait_for_tick(seconds=args.tick_timeout)
            except (RuntimeError, StopIteration):
                print("[TOOL] no tick received within %.0f s: OpenCDA stopped, "
                      "exiting." % args.tick_timeout)
                break
            clock.t = snapshot.timestamp.elapsed_seconds
            if start_time is None:
                start_time = clock.t
            if args.duration > 0 and (clock.t - start_time) >= args.duration:
                break

            step += 1
            if step % args.rescan_every == 0:
                registry.rescan()

            # The mock CA application (if any) emits its detections for this tick
            if mock_ca_feed is not None:
                mock_ca_feed.send(clock.t)

            # Route the detections received from the CA application
            feed.drain()

            for vru in registry.vrus.values():
                vru.step()
    finally:
        feed.close()

    print("[TOOL] done: %d CAV feed(s), %d VRU VAM service(s), edge LDM with "
          "%d object(s), %d warning packet(s) sent."
          % (len(registry.cavs), len(registry.vrus),
             edge.ldm.get_cardinality(), edge.warnings_sent))


# ---------------------------------------------------------------------------
# Real CARLA connection (server executed by OpenCDA)
# ---------------------------------------------------------------------------
def run_with_carla(args):
    import carla  # noqa: F401
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()   # never ticked here: OpenCDA owns the clock
    run(world, args)


# ---------------------------------------------------------------------------
# Built-in mock: a small OpenCDA-like scene (2 CAVs approaching a crossing
# pedestrian, 1 background vehicle, 1 bike) PLUS a mock of the CA application
# that emulates run_ca.py, streaming per-CAV tracks over the same localhost
# UDP JSON interface used by the real application
# ---------------------------------------------------------------------------
MOCK_REF_LAT, MOCK_REF_LON = 45.0, 7.6


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _MockActor:
    def __init__(self, actor_id, type_id, role_name, wheels, east, north,
                 speed, heading):
        self.id = actor_id
        self.type_id = type_id
        self.attributes = {"role_name": role_name, "number_of_wheels": wheels}
        self.is_alive = True
        self.east, self.north = east, north
        self.speed, self.heading = speed, heading

    def advance(self, dt):
        h = math.radians(self.heading)
        self.east += self.speed * math.sin(h) * dt
        self.north += self.speed * math.cos(h) * dt

    def get_location(self):
        return _Vec(self.east, -self.north)      # CARLA-style left-handed frame

    def get_velocity(self):
        h = math.radians(self.heading)
        return _Vec(self.speed * math.sin(h), self.speed * math.cos(h))

    def get_acceleration(self):
        return _Vec()

    def get_transform(self):
        transform = _Vec()
        transform.rotation = _Vec()
        transform.rotation.yaw = (self.heading - 90.0) % 360.0
        return transform


class _MockGeo:
    def __init__(self, latitude, longitude):
        self.latitude, self.longitude = latitude, longitude


class _MockMap:
    def __init__(self):
        self._conv = geo_utils.GeoConverter(MOCK_REF_LAT, MOCK_REF_LON)

    def transform_to_geolocation(self, location):
        lat, lon = self._conv.xy_to_latlon(location.x, -location.y)
        return _MockGeo(lat, lon)


class _MockWorld:
    def __init__(self):
        self._map = _MockMap()
        self._t = 0.0
        # An OpenCDA-like scene: 2 CAVs approaching the crossing pedestrian,
        # 1 background vehicle (must be ignored), 1 bike. As in run_ca.py,
        # the CAVs carry their name as role_name.
        self.actors = [
            _MockActor(11, "vehicle.lincoln.mkz_2017", "Cav1", "4",
                       east=-40.0, north=0.0, speed=8.0, heading=90.0),
            _MockActor(12, "vehicle.lincoln.mkz_2017", "Cav2", "4",
                       east=45.0, north=3.5, speed=7.0, heading=270.0),
            _MockActor(13, "vehicle.tesla.model3", "autopilot", "4",
                       east=0.0, north=-60.0, speed=10.0, heading=0.0),
            _MockActor(21, "walker.pedestrian.0001", "", "0",
                       east=0.0, north=12.0, speed=1.4, heading=180.0),
            _MockActor(22, "vehicle.bh.crossbike", "bike", "2",
                       east=15.0, north=-20.0, speed=4.5, heading=0.0),
        ]

    def get_map(self):
        return self._map

    def get_actors(self):
        return list(self.actors)

    def wait_for_tick(self, seconds=None):
        self._t += SIM_STEP_S
        for actor in self.actors:
            actor.advance(SIM_STEP_S)
        snapshot = _Vec()
        snapshot.timestamp = _Vec()
        snapshot.timestamp.elapsed_seconds = self._t
        return snapshot


class MockCaApplicationFeed:
    """Stands in for the OpenCDA CA application (OpenCDA/CA/run_ca.py): it
    emulates the per-CAV YOLOv8 + fusion + tracking output (with a small
    position noise) and streams it over the SAME localhost UDP JSON interface
    used by the real application, so the tool exercises its real receive path."""

    STATION_TYPE_TO_CA_CATEGORY = {
        vam_codec.STATION_TYPE_PASSENGER_CAR: "vehicle",
        vam_codec.STATION_TYPE_CYCLIST: "bike",
        vam_codec.STATION_TYPE_MOTORCYCLE: "bike",
        vam_codec.STATION_TYPE_PEDESTRIAN: "pedestrian",
    }

    def __init__(self, world, port):
        self.world = world
        self.addr = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._track_ids = {}      # (cav id, target actor id) -> track id
        self._next_track_id = 1

    def send(self, t):
        actors = [(a, classify_actor(a, [], ["autopilot"], set()))
                  for a in self.world.get_actors()]
        actors = [(a, st) for a, st in actors if st is not None]
        cavs = [a for a, st in actors
                if st == vam_codec.STATION_TYPE_PASSENGER_CAR]
        for cav in cavs:
            tracks = []
            for actor, station_type in actors:
                if actor.id == cav.id:
                    continue
                key = (cav.id, actor.id)
                if key not in self._track_ids:
                    self._track_ids[key] = self._next_track_id
                    self._next_track_id += 1
                loc = actor.get_location()
                speed = carla_speed(actor)
                tracks.append({
                    "track_id": self._track_ids[key],
                    "category": self.STATION_TYPE_TO_CA_CATEGORY[station_type],
                    "x": loc.x + random.gauss(0.0, 0.15),   # fusion-like noise
                    "y": loc.y + random.gauss(0.0, 0.15),
                    "z": loc.z,
                    "speed": speed,
                    # As in the CA tracker: heading only when the object moves
                    "heading": (actor.get_transform().rotation.yaw
                                if speed >= 1.0 else None),
                })
            message = {"cav": cav.attributes.get("role_name", str(cav.id)),
                       "cav_actor_id": cav.id, "t": t, "tracks": tracks}
            self.sock.sendto(json.dumps(message).encode(), self.addr)

    def close(self):
        self.sock.close()


def run_with_mock(args):
    print("[TOOL] carla package not available (or --mock): running against the "
          "built-in mock world + mock CA application feed.\n")
    if args.duration <= 0:
        args.duration = MOCK_DURATION_S
    world = _MockWorld()
    mock_feed = MockCaApplicationFeed(world, args.detections_port)
    try:
        run(world, args, mock_ca_feed=mock_feed)
    finally:
        mock_feed.close()


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="V2X simulation tool on top of an OpenCDA-driven CARLA server")
    parser.add_argument("--host", default="localhost", help="CARLA server host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA RPC port")
    parser.add_argument("--mock", action="store_true",
                        help="force the built-in mock world + mock CA feed "
                             "(no CARLA needed)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="simulated seconds to run (0 = until OpenCDA stops)")
    parser.add_argument("--detections-port", type=int,
                        default=DEFAULT_DETECTIONS_PORT,
                        help="localhost UDP port on which the OpenCDA CA "
                             "application streams the CAV detections")
    parser.add_argument("--warning-distance", type=float, default=10.0,
                        help="edge-server proximity warning threshold [m]")
    parser.add_argument("--warning-cooldown", type=float, default=2.0,
                        help="minimum simulated seconds between two warnings "
                             "for the same pair of objects")
    parser.add_argument("--vams", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="enable/disable VAM transmission by the VRUs")
    parser.add_argument("--cav-role-names", default="",
                        help="comma-separated role_name list identifying the "
                             "CAVs (empty = every four-wheeler which is not "
                             "background traffic; run_ca.py CAVs have their "
                             "name, e.g. Cav1, as role_name)")
    parser.add_argument("--background-role-names", default="autopilot",
                        help="comma-separated role_name list of the background "
                             "traffic to ignore")
    parser.add_argument("--cav-ids", default="",
                        help="comma-separated CARLA actor ids to force as CAVs")
    parser.add_argument("--rescan-every", type=int, default=20,
                        help="re-discover the CARLA actors every N ticks")
    parser.add_argument("--tick-timeout", type=float, default=10.0,
                        help="seconds without ticks after which the tool exits")
    parser.add_argument("--quiet", action="store_true",
                        help="suppress the per-update LDM/VAM prints (warnings "
                             "are always printed)")
    args = parser.parse_args()

    if not args.mock:
        try:
            run_with_carla(args)
            return
        except ImportError:
            pass
        except RuntimeError as e:
            print("[TOOL] could not connect to the CARLA server:", e)
    run_with_mock(args)


if __name__ == "__main__":
    main()
