#
# opencda_v2x.py
#
# In-process V2X layer for the OpenCDA CA application (OpenCDA/CA), gluing
# together:
#
#   - the per-CAV local LDM (ldm.py), fed with the confirmed tracks of the CA
#     perception pipeline (YOLOv8 + camera/LiDAR fusion + Kalman tracking);
#   - the per-VRU VRU Basic Service (vru_basic_service.py), transmitting
#     UPER-encoded VAMs on the ETSI TS 103 300-3 triggering conditions,
#     stepped on the CARLA/OpenCDA simulation clock;
#   - a single edge server: an LDM with object matching + the PyCA
#     centralized Collision Avoidance service (CA/PyCA/t2c.py, Algorithm 1:
#     t2c/s2c analytic check), fed with the CAV detections and (optionally)
#     the received VAMs;
#   - the ns-3 v2x-bridge (ns-3-dev/scratch/v2x-bridge) through
#     ns3_bridge_client.py: every CAV->edge detection upload, VRU->edge VAM
#     and edge->CAV warning is one simulated packet over the 5G NR network,
#     carrying the actual application payload; the receiver-side logic runs
#     only when (and if) the bridge reports the packet as delivered.
#
# Three application modes (used by OpenCDA/CA/run_ca_extended.py):
#   mode 2: CAVs detect VRUs (no VAMs), store the detections in their local
#           LDM and upload them to the edge over 5G; the edge LDM + PyCA CA
#           generate the warnings, sent back to the involved CAVs over 5G;
#   mode 3: same as mode 2, but the VRUs also transmit VAMs to the edge as
#           additional information for the collision avoidance;
#   mode 4: V2X-only. The VRUs transmit VAMs as in mode 3, but the CAVs keep
#           their detections strictly on board: their uplink carries the ego
#           state alone, so the edge LDM holds nothing but connected stations
#           (CAV egos + VAM VRUs). The edge checks CAV<->VRU pairs only; a
#           CAV<->CAV risk is never raised.
#
# Frames: the edge LDM / collision avoidance work in the ENU frame used by
# the whole python_vru_service package (x = east, y = north, compass heading
# from north): CARLA world coordinates map as x_enu = x, y_enu = -y and
# heading_enu = (carla_yaw + 90) % 360. The bridge entity positions stay in
# raw CARLA coordinates (they only feed the ns-3 radio geometry).
#

import csv
import json
import math
import os
import sys
import time

try:
    from . import vam_codec
    from .ldm import LDM
    from .vru_basic_service import VRUBasicService, VamStatsRecorder
    from .geo_utils import GeoConverter
except ImportError:                      # imported outside the CA package
    import vam_codec
    from ldm import LDM
    from vru_basic_service import VRUBasicService, VamStatsRecorder
    from geo_utils import GeoConverter

# PyCA collision avoidance (lives in the parent CA package)
try:
    from ..PyCA.t2c import CollisionAvoidanceService, EntityState
except (ImportError, ValueError):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from PyCA.t2c import CollisionAvoidanceService, EntityState

# CA tracker categories -> ETSI station types
CA_CATEGORY_TO_STATION_TYPE = {
    "vehicle": vam_codec.STATION_TYPE_PASSENGER_CAR,
    "bike": vam_codec.STATION_TYPE_CYCLIST,
    "pedestrian": vam_codec.STATION_TYPE_PEDESTRIAN,
}

STATION_TYPE_NAMES = {
    vam_codec.STATION_TYPE_UNKNOWN: "unknown",
    vam_codec.STATION_TYPE_PEDESTRIAN: "pedestrian",
    vam_codec.STATION_TYPE_CYCLIST: "cyclist",
    vam_codec.STATION_TYPE_MOPED: "moped",
    vam_codec.STATION_TYPE_MOTORCYCLE: "motorcyclist",
    vam_codec.STATION_TYPE_PASSENGER_CAR: "cav",
}

# Station-ID numbering plan (stable across the run, VAM-encodable)
CAV_STATION_BASE = 100
VRU_STATION_BASE = 1000
# CAV-local track IDs are namespaced per CAV so they stay unique at the edge
DETECTED_ID_STRIDE = 100000


# ---------------------------------------------------------------------------
# CARLA frame helpers (left-handed CARLA world -> ENU, compass heading)
# ---------------------------------------------------------------------------
def carla_speed(actor):
    vel = actor.get_velocity()
    return math.hypot(vel.x, vel.y)


def carla_compass_heading(actor):
    """CARLA yaw (0 = +x/east, clockwise) -> compass heading from north."""
    return (actor.get_transform().rotation.yaw + 90.0) % 360.0


def carla_long_acceleration(actor):
    """Longitudinal acceleration [m/s^2], or None when almost still."""
    try:
        acc = actor.get_acceleration()
        vel = actor.get_velocity()
    except AttributeError:
        return None
    speed = math.hypot(vel.x, vel.y)
    if speed < 0.1:
        return None
    return (acc.x * vel.x + acc.y * vel.y) / speed


def bridge_entity(name, type_name, actor, timestamp):
    """Entity-state dict in the v2x-bridge schema (raw CARLA coordinates;
    Heading in radians as per the bridge protocol)."""
    loc = actor.get_location()
    return {"timestamp": round(timestamp, 3),
            "origin_ID": name,
            "origin_vehicle_type": type_name,
            "Position": {"x_m": round(loc.x, 3), "y_m": round(loc.y, 3),
                         "z_m": round(loc.z, 3)},
            "Velocity": round(carla_speed(actor), 3),
            "Heading": round(math.radians(carla_compass_heading(actor)), 4)}


# ---------------------------------------------------------------------------
# Sensorized CAV: local LDM + detection upload to the edge over ns-3
# ---------------------------------------------------------------------------
class CavV2XManager:
    """Local LDM of one CAV, fed with the confirmed tracks of its own CA
    perception pipeline; every step the fresh detections (plus the CAV ego
    state, which is self-knowledge) are uploaded to the edge server as one
    simulated 5G packet through the ns-3 bridge.

    In mode 4 the local LDM is still fed (the on-board perception is
    unchanged) but the uplink carries the ego state alone: the edge never
    receives a CAV-perceived object."""

    def __init__(self, name, actor, station_id, layer, quiet):
        self.name = name
        self.actor = actor
        self.station_id = station_id
        self.layer = layer
        self.ldm = LDM()
        if not quiet:
            self.ldm.register_callback(self._print_callback)

    def _print_callback(self, station_id, station_type, position, speed,
                        heading, acceleration):
        print("[CAV %s LDM] object %d (%s): (%.2f, %.2f) speed=%s heading=%s"
              % (self.name, station_id,
                 STATION_TYPE_NAMES.get(station_type, station_type),
                 position.c1, position.c2,
                 "%.2f m/s" % speed if speed is not None else "N/A",
                 "%.1f deg" % heading if heading is not None else "N/A"))

    def ego_state_enu(self):
        """Ego kinematic state (self-knowledge: GNSS/IMU) in the ENU frame."""
        tf = self.actor.get_transform()
        center = tf.transform(self.actor.bounding_box.location)
        return {"station_id": self.station_id,
                "x": center.x, "y": -center.y,
                "speed": carla_speed(self.actor),
                "heading": (tf.rotation.yaw + 90.0) % 360.0,
                "acceleration": carla_long_acceleration(self.actor)}

    def on_tracks(self, tracks, sim_time):
        """Insert the confirmed tracks of this step in the local LDM and
        upload them (with the ego state) to the edge over the ns-3 network."""
        prep_start = time.perf_counter()
        track_dicts = []
        for track in tracks:
            station_type = CA_CATEGORY_TO_STATION_TYPE.get(
                track.category, vam_codec.STATION_TYPE_UNKNOWN)
            # CARLA world frame -> ENU
            x = float(track.position[0])
            y = -float(track.position[1])
            vx_enu = float(track.x[2])
            vy_enu = -float(track.x[3])
            speed = float(track.speed)
            if track.heading_valid and track.heading is not None:
                heading = (float(track.heading) + 90.0) % 360.0
            else:
                # motion direction from the Kalman velocity (compass), the
                # same fallback used by the local collision avoidance
                heading = math.degrees(math.atan2(vx_enu, vy_enu)) % 360.0
            object_id = self.station_id * DETECTED_ID_STRIDE \
                + int(track.track_id)
            self.ldm.insert(object_id, station_type, x=x, y=y, speed=speed,
                            heading=heading, detected=True,
                            perceived_by=self.station_id)
            if self.layer.upload_detections:
                track_dicts.append({"id": object_id, "type": station_type,
                                    "x": round(x, 3), "y": round(y, 3),
                                    "speed": round(speed, 3),
                                    "heading": round(heading, 2)})

        # CAV -> edge upload over the simulated 5G network: the payload really
        # reaches the edge only if/when ns-3 delivers it. In mode 4 the track
        # list is empty and the message degenerates to an ego-state report;
        # the packet kind stays "detection" so the uplink/latency chain of
        # every policy joins the same way offline.
        payload = json.dumps({"kind": "detections"
                              if self.layer.upload_detections else "ego",
                              "cav": self.name,
                              "t": round(sim_time, 3),
                              "ego": {k: (round(v, 3)
                                          if isinstance(v, float) else v)
                                      for k, v in self.ego_state_enu().items()
                                      if v is not None},
                              "tracks": track_dicts}).encode()
        prep_ms = (time.perf_counter() - prep_start) * 1000.0
        self.layer.send_packet(
            sender=self.name, receiver="BS", ptype="detection",
            kind="detection", payload=payload,
            entities=[bridge_entity(self.name, "cav", self.actor, sim_time)],
            on_delivered=self.layer.edge.on_detections_delivered,
            app_processing_ms=prep_ms)


# ---------------------------------------------------------------------------
# VRU: VAM transmission through the VRU Basic Service, over ns-3
# ---------------------------------------------------------------------------
class VruVamManager:
    """VRU Basic Service on board of one VRU (pedestrian walker or bike):
    VAMs are triggered on the ETSI conditions (simulation-time reference,
    manual stepping after every tick) and transmitted to the edge server as
    simulated 5G packets through the ns-3 bridge."""

    def __init__(self, name, actor, station_type, carla_map, layer,
                 geo_origin, quiet):
        self.name = name
        self.actor = actor
        self.carla_map = carla_map
        self.layer = layer
        self.quiet = quiet
        self.type_name = STATION_TYPE_NAMES.get(station_type, "pedestrian")
        self.srv = VRUBasicService(station_id=layer.station_of[name],
                                   station_type=station_type,
                                   proximity_check=False,
                                   ref_lat=geo_origin[0],
                                   ref_lon=geo_origin[1])
        # Time reference: the CARLA/OpenCDA simulation clock, not the wall clock
        self.srv.set_timestamp_callback(lambda: self.layer.sim_time)
        self.srv.set_tx_callback(self._on_tx)
        # Per-VAM statistics log (VBS_stats.csv), shared by every VRU of the run
        if layer.vam_stats is not None:
            self.srv.set_vam_stats(layer.vam_stats, station_label=name)
        self._initialized = False
        self._step_started = None

    def _on_tx(self, encoded_vam):
        if not self.quiet:
            print("[VRU %s] VAM TX trigger=%-19s %d bytes"
                  % (self.name, self.srv.m_trigg_cond.name, len(encoded_vam)))
        # VRU -> edge VAM over the simulated 5G network (the UPER bytes are
        # the application payload of the simulated packet)
        processing_ms = (time.perf_counter() - self._step_started) * 1000.0 \
            if self._step_started is not None else 0.0
        self.layer.send_packet(
            sender=self.name, receiver="BS", ptype="detection", kind="vam",
            payload=bytes(encoded_vam),
            entities=[bridge_entity(self.name, self.type_name, self.actor,
                                    self.layer.sim_time)],
            on_delivered=self.layer.edge.on_vam_delivered,
            app_processing_ms=processing_ms)

    def step(self):
        # The VRU reads its own kinematic state from its CARLA actor: this is
        # self-knowledge (the GNSS/IMU of the VRU device), not perception
        self._step_started = time.perf_counter()
        geo = self.carla_map.transform_to_geolocation(self.actor.get_location())
        self.srv.set_ego_vru_kstatus(
            self.srv.m_station_id, self.srv.m_stationtype,
            lat=geo.latitude, lon=geo.longitude,
            speed=carla_speed(self.actor),
            heading=carla_compass_heading(self.actor),
            acceleration=carla_long_acceleration(self.actor))
        if not self._initialized:
            # initial VAM (DISSEMINATION_START); retry on the next tick if
            # the generation failed (the error would otherwise surface as an
            # opaque RuntimeError at the first check_vam_conditions())
            err = self.srv.init_dissemination()
            if err is not None and err.value != 0:
                print("[VRU %s] initial VAM failed (%s), retrying next tick"
                      % (self.name, err.name))
            else:
                self._initialized = True
        else:
            self.srv.check_vam_conditions()  # one manual check per tick


# ---------------------------------------------------------------------------
# Edge server: LDM (object matching) + PyCA collision avoidance + warnings
# ---------------------------------------------------------------------------
class EdgeCAServer:
    """Centralized collision-avoidance application at the edge (the "BS"
    endpoint of the ns-3 bridge). Keeps an LDM with object matching enabled,
    fed with the CAV detection uploads (none in mode 4, where the CAVs report
    their ego state only) and (modes 3/4) the received VAMs; every
    LDM update feeds the PyCA CollisionAvoidanceService (analytic t2c/s2c,
    Algorithm 1) and each confirmed collision risk produces one warning
    packet per involved CAV, sent over the simulated 5G downlink."""

    # Two LDM entries closer than this are two representations of the same
    # physical object: no warning between them
    SAME_OBJECT_EPSILON_M = 2.0
    # A detected + connected (VAM/ego report) pair of the same station type
    # within the ETSI 4 m VAM position-update lag is the same object too
    DETECTED_VS_CONNECTED_EPSILON_M = 4.5
    STATE_TTL_S = 0.5

    def __init__(self, layer, geo_converter, ca_cfg, quiet):
        self.layer = layer
        self.geo_converter = geo_converter
        self.quiet = quiet
        self.ldm = LDM()
        self.ldm.enable_object_matching(
            match_distance_m=2.0,
            connected_match_distance_m=self.DETECTED_VS_CONNECTED_EPSILON_M,
            fuse_states=True,
            fusion_ttl_s=self.STATE_TTL_S)
        self.ldm.register_callback(self._on_ldm_update)
        if bool(ca_cfg.get("check_rear_end", False)):
            raise ValueError("Rear-end collision checks are disabled")
        self.cas = CollisionAvoidanceService(
            alpha_th_deg=float(ca_cfg.get("alpha_th_deg", 17.0)),
            t2c_th=float(ca_cfg.get("t2c_th", 4.8)),
            s2c_th=float(ca_cfg.get("s2c_th", 4.2)),
            stale_after_s=self.STATE_TTL_S)
        self.warning_cooldown = float(ca_cfg.get("warning_cooldown", 2.0))
        self._pair_last_warning = {}     # frozenset({a, b}) -> sim time
        # Context of the uplink packet currently being ingested (set while
        # its LDM inserts run): kind/packet_id/latency/tx_step/sender. It is
        # attached to every warning that ingest triggers, so the end-to-end
        # latency chain (sensing -> inference -> uplink -> edge CA ->
        # downlink) can be reconstructed offline per warning.
        self._uplink_ctx = None
        self.detections_received = 0
        self.vams_received = 0
        self.risks_detected = 0
        self.warnings_sent = 0

    # ---- receiver-side handlers (called on ns-3 delivery) ---------------
    def on_detections_delivered(self, reply, payload, meta):
        """One CAV detection upload delivered to the edge by ns-3: store the
        ego state (connected station) and the detections in the edge LDM."""
        try:
            data = json.loads(payload.decode())
            ego = data["ego"]
            tracks = data.get("tracks", [])
        except (ValueError, KeyError, UnicodeDecodeError) as e:
            print("[EDGE] malformed detection payload discarded (%s)" % e)
            return
        self.detections_received += 1
        cav_station = int(ego["station_id"])
        self._uplink_ctx = {"kind": meta["kind"],
                            "packet_id": reply.get("packet_id"),
                            "latency_ms": reply.get("latency_ms"),
                            "tx_step": meta.get("tx_step"),
                            "sender": meta.get("sender"),
                            "tx_sim_t": float(data.get(
                                "t", meta.get("tx_sim_t", self.layer.sim_time)))}
        observed_at = self._uplink_ctx["tx_sim_t"]
        try:
            self.ldm.insert(cav_station, vam_codec.STATION_TYPE_PASSENGER_CAR,
                            x=float(ego["x"]), y=float(ego["y"]),
                            speed=float(ego.get("speed", 0.0)),
                            heading=float(ego.get("heading", 0.0)),
                            acceleration=ego.get("acceleration"),
                            timestamp=observed_at)
            for track in tracks:
                local_id = int(track["id"])
                self.ldm.insert(local_id, int(track["type"]),
                                x=float(track["x"]), y=float(track["y"]),
                                speed=float(track.get("speed", 0.0)),
                                heading=float(track.get("heading", 0.0)),
                                detected=True, perceived_by=cav_station,
                                timestamp=observed_at)
                self.layer.log_edge_ldm_update(
                    "detection", meta.get("tx_step"), meta.get("sender"),
                    local_id, self.ldm.canonical_id(cav_station, local_id),
                    track.get("x"), track.get("y"),
                    reply.get("packet_id"))
        finally:
            self._uplink_ctx = None

    def on_vam_delivered(self, reply, payload, meta):
        """One VRU VAM delivered to the edge by ns-3: decode the UPER bytes
        and store/update the VRU in the edge LDM."""
        self._uplink_ctx = {"kind": meta["kind"],
                            "packet_id": reply.get("packet_id"),
                            "latency_ms": reply.get("latency_ms"),
                            "tx_step": meta.get("tx_step"),
                            "sender": meta.get("sender"),
                            "tx_sim_t": float(meta.get(
                                "tx_sim_t", self.layer.sim_time))}
        try:
            err, station_id = self.ldm.add_vru_from_vam(
                bytes(payload), geo_converter=self.geo_converter,
                timestamp=self._uplink_ctx["tx_sim_t"])
        finally:
            self._uplink_ctx = None
        if station_id is None:
            print("[EDGE] undecodable VAM discarded (%s)" % err)
            return
        for old_id, _ in self.ldm.consume_rekeyed_ids():
            self.cas.remove(str(old_id))
        obj = self.ldm.lookup(station_id)
        self.layer.log_edge_ldm_update(
            "vam", meta.get("tx_step"), meta.get("sender"), station_id,
            station_id, obj.x if obj else None, obj.y if obj else None,
            reply.get("packet_id"))
        self.vams_received += 1

    # ---- LDM callback -> collision avoidance ----------------------------
    def _on_ldm_update(self, station_id, station_type, position, speed,
                       heading, acceleration):
        if position.kind != "xy":
            return
        now = self.layer.sim_time
        # A fused object is expressed at the newest contributing observation,
        # which can differ from the packet currently being handled when packets
        # arrive out of order.  Read the canonical LDM timestamp rather than
        # reusing the triggering packet's transmit time.
        ldm_object = self.ldm.lookup(station_id)
        observed_at = float(
            ldm_object.timestamp if ldm_object is not None
            else (self._uplink_ctx or {}).get("tx_sim_t", now)
        )
        # Constant-velocity kinematic model for every entity fed to the edge
        # CA (ego CAVs, detected objects and VAM VRUs alike): accelerations are
        # forced to zero (acc_lon=None -> ax=ay=0), matching the GT reference
        # and the on-board CA. This is the single authoritative point where the
        # edge builds CA states, so it also makes _state_at project at constant
        # velocity. (CARLA/ETSI accelerations are noisy and previously only the
        # longitudinal ego component reached the edge, which suppressed ~1/3 of
        # the edge's true-positive warnings versus the on-board CA.)
        state = EntityState(station_id=str(station_id),
                            x=float(position.c1), y=float(position.c2),
                            speed_ms=float(speed) if speed is not None else 0.0,
                            heading_deg=float(heading) if heading is not None
                            else 0.0,
                            acc_lon=None,
                            timestamp=observed_at)
        t0 = time.perf_counter()
        warnings = self.cas.update(
            state, now=now, observation_time=observed_at)
        ca_ms = (time.perf_counter() - t0) * 1000.0
        self.layer.log_edge_ca_timing(
            (self._uplink_ctx or {}).get("kind", ""), station_id, ca_ms,
            len(warnings))
        for warning in warnings:
            self._handle_risk(warning, now, ca_ms)

    def _same_object(self, obj_a, obj_b):
        """Two LDM entries likely representing the same physical object."""
        if obj_a.x is None or obj_b.x is None:
            return False
        d = math.hypot(obj_a.x - obj_b.x, obj_a.y - obj_b.y)
        if d < self.SAME_OBJECT_EPSILON_M:
            return True
        return (obj_a.detected != obj_b.detected
                and obj_a.station_type == obj_b.station_type
                and d < self.DETECTED_VS_CONNECTED_EPSILON_M)

    def _handle_risk(self, warning, now, ca_ms):
        sid_a = int(warning.entity.station_id)
        sid_b = int(warning.other.station_id)
        obj_a = self.ldm.lookup(sid_a)
        obj_b = self.ldm.lookup(sid_b)
        if obj_a is None or obj_b is None or self._same_object(obj_a, obj_b):
            return
        # Mode 4 evaluates CAVs against VRUs only: the CAV ego states are at
        # the edge to be the ego side of a CAV<->VRU check, never to be
        # checked against each other. Filtered before the cooldown/counter
        # bookkeeping so a suppressed pair leaves no trace at all.
        if self.layer.vru_pairs_only and \
                sid_a in self.layer.cav_origin_of and \
                sid_b in self.layer.cav_origin_of:
            return
        pair = frozenset((sid_a, sid_b))
        last = self._pair_last_warning.get(pair)
        if last is not None and (now - last) < self.warning_cooldown:
            return
        # The warning goes to the involved vehicles: the connected CAVs of
        # the pair (detected objects and VRUs have no warning downlink here)
        targets = [sid for sid in (sid_a, sid_b)
                   if sid in self.layer.cav_origin_of]
        if not targets:
            return
        self._pair_last_warning[pair] = now
        self.risks_detected += 1
        print("[EDGE] %s risk between %d and %d: t2c=%.2f s s2c=%.2f m "
              "at t=%.2f s -> warning to %s"
              % (warning.collision_type, sid_a, sid_b, warning.t2c,
                 warning.s2c, now,
                 ", ".join(self.layer.cav_origin_of[t] for t in targets)))

        for target_sid in targets:
            origin = self.layer.cav_origin_of[target_sid]
            other_sid = sid_b if target_sid == sid_a else sid_a
            other = self.ldm.lookup(other_sid)
            ctx = self._uplink_ctx or {}
            payload = json.dumps({
                "kind": "warning", "t": round(now, 3), "target": origin,
                "collision_type": warning.collision_type,
                "t2c": round(warning.t2c, 3), "s2c": round(warning.s2c, 3),
                "station_id": target_sid, "other_station": other_sid,
                "other_type": STATION_TYPE_NAMES.get(
                    other.station_type if other else -1, "unknown"),
                "other_x": round(other.x, 3) if other else None,
                "other_y": round(other.y, 3) if other else None,
                # latency chain of this warning: the uplink packet whose
                # ingest triggered it + the edge CA execution time
                "uplink_kind": ctx.get("kind"),
                "uplink_packet_id": ctx.get("packet_id"),
                "uplink_latency_ms": ctx.get("latency_ms"),
                "uplink_tx_step": ctx.get("tx_step"),
                "uplink_sender": ctx.get("sender"),
                "edge_ca_ms": round(ca_ms, 3),
            }).encode()
            # edge -> CAV warning over the simulated 5G downlink
            packet_id = self.layer.send_packet(
                sender="BS", receiver=origin, ptype="warning",
                kind="warning", payload=payload,
                on_delivered=self.layer.on_warning_delivered)
            self.layer.log_edge_risk(
                sid_a, obj_a, sid_b, obj_b, target_sid, origin, other_sid,
                warning, ctx, ca_ms, packet_id)
            self.warnings_sent += 1


# ---------------------------------------------------------------------------
# The V2X layer glued into run_ca_extended.py
# ---------------------------------------------------------------------------
class V2XLayer:
    """Orchestrates CAV managers, VRU managers (modes 3/4), the edge server
    and the ns-3 bridge client inside the OpenCDA CA main loop."""

    def __init__(self, bridge, mode, carla_map, cav_actors, vru_actors,
                 ca_cfg, run_dir, pump_wait_s=0.25, quiet=True):
        """cav_actors: list of (name, carla_vehicle); vru_actors: list of
        (name, actor, etsi_station_type) — used only in modes 3 and 4."""
        self.bridge = bridge
        self.mode = int(mode)
        # Mode 4 (V2X-only policy): the CAVs report their ego state alone and
        # the edge checks CAV<->VRU pairs exclusively.
        self.upload_detections = self.mode != 4
        self.vru_pairs_only = self.mode == 4
        self.carla_map = carla_map
        self.pump_wait_s = float(pump_wait_s)
        self.quiet = quiet
        self.sim_time = 0.0
        self.step_no = -1

        # Geo-origin of the map: shared reference of the ENU frame so that
        # the x/y decoded from the VAMs matches the CAV detections
        try:
            import carla
            origin_loc = carla.Location(x=0.0, y=0.0, z=0.0)
        except ImportError:              # mock maps accept any x/y/z object
            origin_loc = type("Origin", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
        origin_geo = carla_map.transform_to_geolocation(origin_loc)
        self.geo_origin = (origin_geo.latitude, origin_geo.longitude)
        geo_converter = GeoConverter(*self.geo_origin)

        # Stable station-ID plan
        self.station_of = {}         # origin name -> numeric station id
        self.cav_origin_of = {}      # CAV station id -> origin name
        for i, (name, _) in enumerate(cav_actors, 1):
            self.station_of[name] = CAV_STATION_BASE + i
            self.cav_origin_of[CAV_STATION_BASE + i] = name
        for i, (name, _, _) in enumerate(vru_actors, 1):
            self.station_of[name] = VRU_STATION_BASE + i

        # Per-VAM log of the VRU Basic Services (VBS_stats.csv): one row per
        # transmitted VAM with its content, the time since the previous VAM of
        # the same VRU and the running average VAM periodicity, per VRU and
        # pooled over all of them. Created only in the modes where the VRUs do
        # transmit VAMs (3 and 4); the recorder is shared by every VRU service.
        self.vam_stats = None
        if self.mode >= 3 and vru_actors:
            self.vam_stats = VamStatsRecorder(
                os.path.join(run_dir, "VBS_stats.csv"),
                geo_converter=geo_converter)

        self.edge = EdgeCAServer(self, geo_converter, ca_cfg, quiet)
        self.cav_managers = [
            CavV2XManager(name, actor, self.station_of[name], self, quiet)
            for name, actor in cav_actors]
        self.vru_managers = []
        if self.mode >= 3:
            self.vru_managers = [
                VruVamManager(name, actor, station_type, carla_map, self,
                              self.geo_origin, quiet)
                for name, actor, station_type in vru_actors]

        # Entities refreshed on the bridge every tick: the connected actors
        self._entity_defs = [(name, "cav", actor)
                             for name, actor in cav_actors]
        if self.mode >= 3:
            self._entity_defs += [
                (name, STATION_TYPE_NAMES.get(st, "pedestrian"), actor)
                for name, actor, st in vru_actors]

        # Warnings received by each CAV since its last metrics row
        self._row_warnings = {name: [] for name, _ in cav_actors}
        self.warnings_received = {name: 0 for name, _ in cav_actors}
        self.warnings_min_t2c = {name: float("inf") for name, _ in cav_actors}

        # Per-packet outcome log + received-warning log (run output dir)
        self._v2x_csv_file = open(os.path.join(run_dir, "v2x_packets.csv"),
                                  "w", newline="")
        self._v2x_csv = csv.writer(self._v2x_csv_file)
        self._v2x_csv.writerow(["packet_id", "kind", "sender", "receiver",
                                "size_bytes", "tx_step", "tx_sim_t",
                                "app_processing_ms", "status", "latency_ms"])
        self._warn_csv_file = open(os.path.join(run_dir, "edge_warnings.csv"),
                                   "w", newline="")
        self._warn_csv = csv.writer(self._warn_csv_file)
        # latency_ms is the warning downlink; the uplink_* / edge_ca_ms
        # columns carry the rest of the end-to-end chain of this warning
        # (uplink packet that triggered it + edge CA execution time)
        self._warn_csv.writerow(["rx_step", "sim_t", "cav", "other_station",
                                 "other_type", "collision_type", "t2c",
                                 "s2c", "latency_ms", "packet_id",
                                 "uplink_kind", "uplink_packet_id",
                                 "uplink_latency_ms", "uplink_tx_step",
                                 "uplink_sender", "edge_ca_ms"])
        # dedicated wall-clock log of every edge CA execution (one row per
        # LDM update processed by the CollisionAvoidanceService)
        self._edge_ca_csv_file = open(
            os.path.join(run_dir, "edge_ca_timing.csv"), "w", newline="")
        self._edge_ca_csv = csv.writer(self._edge_ca_csv_file)
        self._edge_ca_csv.writerow(["step", "sim_t", "trigger_kind",
                                    "station_id", "ca_ms", "n_warnings"])
        self._risk_csv_file = open(os.path.join(run_dir, "edge_risks.csv"),
                                   "w", newline="")
        self._risk_csv = csv.writer(self._risk_csv_file)
        self._risk_csv.writerow([
            "risk_id", "step", "sim_t", "station_a", "type_a", "x_a",
            "y_a", "station_b", "type_b", "x_b", "y_b",
            "target_station", "target_cav", "other_station",
            "collision_type", "t2c", "s2c", "uplink_kind",
            "uplink_packet_id", "uplink_latency_ms", "uplink_tx_step",
            "uplink_sender", "edge_ca_ms", "warning_packet_id"])
        self._risk_sequence = 0
        self._ldm_update_csv_file = open(
            os.path.join(run_dir, "edge_ldm_updates.csv"), "w", newline="")
        self._ldm_update_csv = csv.writer(self._ldm_update_csv_file)
        self._ldm_update_csv.writerow([
            "rx_step", "sim_t", "kind", "tx_step", "sender",
            "local_object_id", "canonical_object_id", "x", "y",
            "uplink_packet_id"])

        n_ues = len(self._entity_defs)
        print("[V2X] mode %d: %d CAV LDM(s), %d VRU VAM service(s), edge "
              "server with PyCA collision avoidance; %d connected UEs "
              "(run the v2x-bridge with --maxUes >= %d)"
              % (self.mode, len(self.cav_managers), len(self.vru_managers),
                 n_ues, n_ues))
        if not self.upload_detections:
            print("[V2X] V2X-only policy: CAV detections are NOT uploaded "
                  "(ego state only) and the edge checks CAV<->VRU pairs only")

    # ------------------------------------------------------------------
    def send_packet(self, sender, receiver, ptype, kind, payload,
                    entities=None, on_delivered=None, app_processing_ms=0.0):
        """All simulated packets go through here so every outcome lands in
        v2x_packets.csv."""
        meta = {"kind": kind, "sender": sender, "receiver": receiver,
                "tx_step": self.step_no, "tx_sim_t": self.sim_time,
                "app_processing_ms": float(app_processing_ms)}

        def delivered(reply, rx_payload, _meta):
            self._log_packet(reply.get("packet_id"), meta,
                             12 + len(payload), "delivered",
                             reply.get("latency_ms"))
            if on_delivered is not None:
                on_delivered(reply, rx_payload, meta)

        def failed(reply, _meta):
            self._log_packet(reply.get("packet_id"), meta, 12 + len(payload),
                             reply.get("status", "no_reply"), None)

        return self.bridge.send_packet(
            self.sim_time, sender, receiver, ptype, payload=payload,
            entities=entities, kind=kind, meta=meta,
            on_delivered=delivered, on_failed=failed)

    def log_edge_ca_timing(self, trigger_kind, station_id, ca_ms, n_warnings):
        self._edge_ca_csv.writerow(
            [self.step_no, "%.3f" % self.sim_time, trigger_kind, station_id,
             "%.4f" % ca_ms, n_warnings])

    def log_edge_ldm_update(self, kind, tx_step, sender, local_id,
                            canonical_id, x, y, packet_id):
        self._ldm_update_csv.writerow([
            self.step_no, "%.6f" % self.sim_time, kind,
            tx_step if tx_step is not None else "", sender or "", local_id,
            canonical_id, "%.6f" % float(x) if x is not None else "",
            "%.6f" % float(y) if y is not None else "",
            packet_id if packet_id is not None else ""])

    def log_edge_risk(self, sid_a, obj_a, sid_b, obj_b, target_sid,
                      target_cav, other_sid, warning, ctx, ca_ms, packet_id):
        self._risk_sequence += 1
        self._risk_csv.writerow([
            self._risk_sequence, self.step_no, "%.6f" % self.sim_time,
            sid_a, STATION_TYPE_NAMES.get(obj_a.station_type, "unknown"),
            "%.6f" % obj_a.x, "%.6f" % obj_a.y,
            sid_b, STATION_TYPE_NAMES.get(obj_b.station_type, "unknown"),
            "%.6f" % obj_b.x, "%.6f" % obj_b.y,
            target_sid, target_cav, other_sid, warning.collision_type,
            "%.6f" % warning.t2c, "%.6f" % warning.s2c,
            ctx.get("kind") or "", ctx.get("packet_id") or "",
            "%.6f" % ctx["latency_ms"]
            if ctx.get("latency_ms") is not None else "",
            ctx.get("tx_step") if ctx.get("tx_step") is not None else "",
            ctx.get("sender") or "", "%.6f" % ca_ms, packet_id])
        self._risk_csv_file.flush()

    def _log_packet(self, packet_id, meta, size, status, latency_ms):
        self._v2x_csv.writerow(
            [packet_id if packet_id is not None else "", meta["kind"],
             meta["sender"], meta["receiver"], size, meta["tx_step"],
             "%.3f" % meta["tx_sim_t"],
             "%.6f" % meta.get("app_processing_ms", 0.0), status,
             "%.3f" % latency_ms if latency_ms is not None else ""])
        self._v2x_csv_file.flush()

    # ------------------------------------------------------------------
    def on_warning_delivered(self, reply, payload, meta):
        """An edge warning has been delivered to a CAV over the simulated
        downlink: log it (edge_warnings.csv + metrics row info)."""
        try:
            data = json.loads(payload.decode())
        except (ValueError, UnicodeDecodeError):
            print("[V2X] malformed warning payload discarded")
            return
        cav = data.get("target", meta["receiver"])
        latency = reply.get("latency_ms")
        record = {"type": data.get("collision_type", ""),
                  "t2c": data.get("t2c"), "s2c": data.get("s2c"),
                  "other": data.get("other_station"),
                  "latency_ms": latency}
        if cav in self._row_warnings:
            self._row_warnings[cav].append(record)
            self.warnings_received[cav] += 1
            if data.get("t2c") is not None:
                self.warnings_min_t2c[cav] = min(self.warnings_min_t2c[cav],
                                                 float(data["t2c"]))
        uplink_latency = data.get("uplink_latency_ms")
        self._warn_csv.writerow(
            [self.step_no, "%.3f" % self.sim_time, cav,
             data.get("other_station", ""), data.get("other_type", ""),
             data.get("collision_type", ""), data.get("t2c", ""),
             data.get("s2c", ""),
             "%.3f" % latency if latency is not None else "",
             reply.get("packet_id", ""),
             data.get("uplink_kind", "") or "",
             data.get("uplink_packet_id", "") or "",
             "%.3f" % uplink_latency if uplink_latency is not None else "",
             data.get("uplink_tx_step", "") if data.get("uplink_tx_step")
             is not None else "",
             data.get("uplink_sender", "") or "",
             data.get("edge_ca_ms", "")])
        self._warn_csv_file.flush()
        print("[%s] EDGE WARNING received: %s with station %s "
              "(t2c=%.2f s, s2c=%.2f m, downlink %.2f ms)"
              % (cav, data.get("collision_type", "?"),
                 data.get("other_station", "?"),
                 data.get("t2c") or -1, data.get("s2c") or -1,
                 latency or -1))

    def pop_row_info(self, cav_name):
        """Warnings received by this CAV since its previous metrics row:
        [count, type, t2c, s2c, latency_ms] CSV columns (empty if none)."""
        warnings = self._row_warnings.get(cav_name, [])
        if not warnings:
            return ["0", "", "", "", ""]
        last = warnings[-1]
        cols = [str(len(warnings)), last["type"],
                "%.2f" % last["t2c"] if last["t2c"] is not None else "",
                "%.2f" % last["s2c"] if last["s2c"] is not None else "",
                "%.3f" % last["latency_ms"]
                if last["latency_ms"] is not None else ""]
        self._row_warnings[cav_name] = []
        return cols

    # ------------------------------------------------------------------
    def step(self, step_no, sim_time, tracks_by_cav):
        """One co-simulation tick: refresh the scene on the bridge, upload
        the CAV detections, step the VRU services, then resolve the ns-3
        deliveries (uplink first, then the warnings they may have caused)."""
        self.step_no = step_no
        self.sim_time = float(sim_time)

        # Scene update for the radio geometry (one entities-only datagram)
        self.bridge.send_entities(
            self.sim_time,
            [bridge_entity(name, tname, actor, self.sim_time)
             for name, tname, actor in self._entity_defs])

        for manager in self.cav_managers:
            manager.on_tracks(tracks_by_cav.get(manager.name, []),
                              self.sim_time)
        for vru in self.vru_managers:
            vru.step()

        # 1st pump: detections/VAMs delivered at the edge -> LDM + collision
        # avoidance -> warning packets queued; 2nd pump: warning deliveries.
        self.bridge.pump(self.pump_wait_s)
        self.bridge.pump(self.pump_wait_s)

    # ------------------------------------------------------------------
    def finalize(self):
        """Flush the layer CSVs and return the summary lines."""
        self._v2x_csv_file.close()
        self._warn_csv_file.close()
        self._edge_ca_csv_file.close()
        self._risk_csv_file.close()
        self._ldm_update_csv_file.close()
        if self.vam_stats is not None:
            self.vam_stats.close()
        edge = self.edge
        lines = ["", "V2X layer (mode %d, ns-3 v2x-bridge in the loop):"
                 % self.mode]
        lines += ["  " + line for line in self.bridge.summary_lines()]
        lines.append("  edge server : %d detection upload(s) and %d VAM(s) "
                     "received, LDM with %d object(s)"
                     % (edge.detections_received, edge.vams_received,
                        edge.ldm.get_cardinality()))
        lines.append("  edge CA     : %d collision risk(s) confirmed, "
                     "%d warning packet(s) sent"
                     % (edge.risks_detected, edge.warnings_sent))
        for manager in self.cav_managers:
            n = self.warnings_received[manager.name]
            if n:
                lines.append("  %-6s: %d edge warning(s) received "
                             "(min t2c %.2f s), local LDM with %d object(s)"
                             % (manager.name, n,
                                self.warnings_min_t2c[manager.name],
                                manager.ldm.get_cardinality()))
            else:
                lines.append("  %-6s: no edge warnings received, local LDM "
                             "with %d object(s)"
                             % (manager.name, manager.ldm.get_cardinality()))
        for vru in self.vru_managers:
            if vru.srv.m_vam_sent:
                lines.append("  %-6s: %d VAM(s) sent (pos %d, speed %d, "
                             "head %d, time %d)"
                             % (vru.name, vru.srv.m_vam_sent,
                                vru.srv.m_pos_sent, vru.srv.m_speed_sent,
                                vru.srv.m_head_sent, vru.srv.m_time_sent))
        return lines
