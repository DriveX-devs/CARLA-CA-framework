#
# ldm.py
#
# Python porting of a simplified Local Dynamic Map (LDM), based on the C++
# implementation of VaN3Twin and of the S-LDM (github.com/DriveX-devs)
#
# The LDM stores, for each object, its stationID, stationType, position
# (x/y and/or lat/lon), speed, heading and (optionally) acceleration.
# It supports insertion, update, lookup by stationID, lookup by circular
# geographical area and removal. A callback mechanism notifies registered
# functions every time an insertion or update succeeds.
# VRUs can also be added directly from received VAMs (UPER-encoded) via
# add_vru_from_vam(), leveraging asn1tools and the ASN.1 files in ns-3-dev.
#

import math
import threading
import time
from collections import namedtuple
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

try:
    from . import vam_codec
    from . import geo_utils
except ImportError:
    import vam_codec
    import geo_utils


class LDMError(Enum):
    """Same error semantics as LDM::LDM_error_t in the C++ implementation."""
    LDM_OK = 0
    LDM_UPDATED = 1
    LDM_ITEM_NOT_FOUND = 2
    LDM_MAP_FULL = 3
    LDM_UNKNOWN_ERROR = 4


# Position handed over to the callbacks: kind is either "xy" (c1=x, c2=y) or,
# only when x/y is not available, "latlon" (c1=lat, c2=lon)
LDMPosition = namedtuple("LDMPosition", ["kind", "c1", "c2"])


@dataclass
class LDMObject:
    """Python counterpart of (a subset of) vehicleData_t (from the C++ implementation)."""
    station_id: int
    station_type: int
    x: Optional[float] = None
    y: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    speed: Optional[float] = None          # [m/s]
    heading: Optional[float] = None        # [deg from north, 0..360)
    acceleration: Optional[float] = None   # [m/s^2], optional
    detected: bool = False                 # True for objects detected by the sensors of a vehicle
    perceived_by: Optional[set] = None     # stationIDs of the vehicles perceiving this object
    timestamp: float = field(default_factory=time.time)  # entry last update

    def position(self):
        """Return the preferred position representation for the callbacks:
        x/y if available, lat/lon otherwise."""
        if self.x is not None and self.y is not None:
            return LDMPosition("xy", self.x, self.y)
        return LDMPosition("latlon", self.lat, self.lon)


class LDM:
    def __init__(self):
        self._db = {}
        self._lock = threading.RLock()
        self._callbacks = []
        # Object matching (disabled by default): merges the detections of the
        # same physical object coming from different vehicles into a single entry
        self._object_matching = False
        self._match_distance_m = 1.0
        self._connected_match_distance_m = None
        self._fuse_states = False
        self._fusion_ttl_s = 0.5
        # (perceiving stationID, object ID assigned by that vehicle) -> canonical
        # stationID under which the object is stored in the database
        self._match_map = {}
        # Latest observation from each independent source, grouped by canonical
        # object ID.  This is enabled only for the edge LDM; ordinary LDM users
        # retain insert-or-update (latest writer) semantics.
        self._source_states = {}
        # Only connected VRU identities received through VAM are eligible for
        # sensor-to-connected association.  CAV ego/CAM records must never
        # absorb a nearby passenger-car detection merely because they share a
        # station type and happen to be inside the wider connected gate.
        self._connected_match_ids = set()
        self._recently_rekeyed = []

    # ------------------------------------------------------------------
    # Object matching
    # ------------------------------------------------------------------
    def enable_object_matching(self, match_distance_m=2.0,
                               connected_match_distance_m=None,
                               fuse_states=False, fusion_ttl_s=0.5):
        """Enable the object matching logic: if multiple vehicles detect the same
        physical object (entries inserted with detected=True and a perceived_by
        stationID), the detections are merged and the object counts as a single
        entry in the LDM, stored under the stationID with which it was first
        inserted. Two detections coming from different vehicles are considered
        the same object if their positions are closer than match_distance_m."""
        with self._lock:
            self._object_matching = True
            self._match_distance_m = match_distance_m
            self._connected_match_distance_m = connected_match_distance_m
            self._fuse_states = bool(fuse_states)
            self._fusion_ttl_s = float(fusion_ttl_s)

    def disable_object_matching(self):
        with self._lock:
            self._object_matching = False
            self._match_map.clear()
            self._source_states.clear()
            self._connected_match_ids.clear()
            self._recently_rekeyed.clear()

    def canonical_id(self, perceived_by, station_id):
        """Return the edge-canonical ID for a reporter's local object ID.

        This read-only lookup is used exclusively by raw experiment logging;
        it does not affect matching or application decisions.
        """
        with self._lock:
            return self._match_map.get((perceived_by, station_id), station_id)

    def consume_rekeyed_ids(self):
        """Return and clear sensor-only IDs promoted to connected station IDs."""
        with self._lock:
            result = list(self._recently_rekeyed)
            self._recently_rekeyed.clear()
            return result

    def _find_matching_object(self, perceived_by, station_type,
                              x, y, lat, lon):
        """Return the closest detected object within the matching distance which
        is not already perceived by the given vehicle, or None."""
        best = None
        best_d = None
        for obj in self._db.values():
            if obj.station_type != station_type:
                continue
            if not obj.detected:
                if self._connected_match_distance_m is None or \
                        obj.station_id not in self._connected_match_ids:
                    continue
            if obj.perceived_by is not None and perceived_by in obj.perceived_by:
                # This vehicle already reports this object under another local ID:
                # it must be a different physical object
                continue
            if x is not None and y is not None and obj.x is not None and obj.y is not None:
                d = math.hypot(obj.x - x, obj.y - y)
            elif lat is not None and lon is not None and \
                    obj.lat is not None and obj.lon is not None:
                d = geo_utils.geodesic_distance_m(lat, lon, obj.lat, obj.lon)
            else:
                continue
            gate = self._match_distance_m if obj.detected \
                else self._connected_match_distance_m
            if d <= gate and (best_d is None or d < best_d):
                best = obj
                best_d = d
        return best

    def _find_detected_match(self, station_type, x, y, lat, lon):
        """Closest sensor-only canonical object for a connected report."""
        best = None
        best_d = None
        if self._connected_match_distance_m is None:
            return None
        for obj in self._db.values():
            if not obj.detected or obj.station_type != station_type:
                continue
            if x is not None and y is not None and obj.x is not None and obj.y is not None:
                d = math.hypot(obj.x - x, obj.y - y)
            elif lat is not None and lon is not None and \
                    obj.lat is not None and obj.lon is not None:
                d = geo_utils.geodesic_distance_m(lat, lon, obj.lat, obj.lon)
            else:
                continue
            if d <= self._connected_match_distance_m and \
                    (best_d is None or d < best_d):
                best = obj
                best_d = d
        return best

    @staticmethod
    def _source_key(station_id, detected, perceived_by):
        return ("detected", int(perceived_by), int(station_id)) \
            if detected else ("connected", int(station_id))

    @staticmethod
    def _project_xy(obj, timestamp):
        if obj.x is None or obj.y is None:
            return obj.x, obj.y
        dt = max(0.0, float(timestamp) - float(obj.timestamp))
        heading = math.radians(float(obj.heading or 0.0))
        speed = float(obj.speed or 0.0)
        acceleration = float(obj.acceleration or 0.0)
        vx = speed * math.sin(heading)
        vy = speed * math.cos(heading)
        ax = acceleration * math.sin(heading)
        ay = acceleration * math.cos(heading)
        return (obj.x + vx * dt + 0.5 * ax * dt * dt,
                obj.y + vy * dt + 0.5 * ay * dt * dt)

    def _fused_object(self, target_id):
        sources = list(self._source_states.get(target_id, {}).values())
        if not sources:
            return self._db.get(target_id)
        fusion_time = max(float(obj.timestamp) for obj in sources)
        fresh = [obj for obj in sources
                 if fusion_time - float(obj.timestamp) <= self._fusion_ttl_s]
        if not fresh:
            fresh = [max(sources, key=lambda obj: obj.timestamp)]
        weights = [3.0 if not obj.detected else 1.0 for obj in fresh]
        projected = [self._project_xy(obj, fusion_time) for obj in fresh]
        xy = [(point, weight) for point, weight in zip(projected, weights)
              if point[0] is not None and point[1] is not None]
        x = sum(point[0] * weight for point, weight in xy) / \
            sum(weight for _, weight in xy) if xy else None
        y = sum(point[1] * weight for point, weight in xy) / \
            sum(weight for _, weight in xy) if xy else None

        def weighted_optional(attribute):
            values = [(float(getattr(obj, attribute)), weight)
                      for obj, weight in zip(fresh, weights)
                      if getattr(obj, attribute) is not None]
            return sum(value * weight for value, weight in values) / \
                sum(weight for _, weight in values) if values else None

        angles = [(math.radians(float(obj.heading)), weight)
                  for obj, weight in zip(fresh, weights)
                  if obj.heading is not None]
        if angles:
            sin_mean = sum(math.sin(angle) * weight for angle, weight in angles)
            cos_mean = sum(math.cos(angle) * weight for angle, weight in angles)
            heading = math.degrees(math.atan2(sin_mean, cos_mean)) % 360.0
        else:
            heading = None

        connected = [obj for obj in fresh if not obj.detected]
        identity = max(connected or fresh, key=lambda obj: obj.timestamp)
        reporters = {int(obj.perceived_by) for obj in fresh
                     if isinstance(obj.perceived_by, int)}
        return LDMObject(
            station_id=target_id,
            station_type=identity.station_type,
            x=x,
            y=y,
            lat=identity.lat,
            lon=identity.lon,
            speed=weighted_optional("speed"),
            heading=heading,
            acceleration=weighted_optional("acceleration"),
            detected=not bool(connected),
            perceived_by=reporters or None,
            timestamp=fusion_time,
        )

    def _purge_match_map(self, station_id):
        for key in [k for k, v in self._match_map.items() if v == station_id]:
            del self._match_map[key]

    def _promote_to_connected_id(self, old_id, connected_id):
        """Re-key a sensor-only canonical object to its VAM/CAM station ID."""
        if old_id == connected_id:
            return
        self._db.pop(old_id, None)
        for key, value in list(self._match_map.items()):
            if value == old_id:
                self._match_map[key] = connected_id
        old_sources = self._source_states.pop(old_id, {})
        self._source_states.setdefault(connected_id, {}).update(old_sources)
        self._recently_rekeyed.append((old_id, connected_id))

    # ------------------------------------------------------------------
    # Callback management
    # ------------------------------------------------------------------
    def register_callback(self, callback):
        """Register a function called on every successful insert/update as:
        callback(station_id, station_type, position, speed, heading, acceleration)
        where position is an LDMPosition (x/y or, only if not available, lat/lon)
        and acceleration is None when not available."""
        with self._lock:
            self._callbacks.append(callback)

    def unregister_callback(self, callback):
        with self._lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)

    def _notify(self, obj):
        for cb in list(self._callbacks):
            cb(obj.station_id, obj.station_type, obj.position(),
               obj.speed, obj.heading, obj.acceleration)

    # ------------------------------------------------------------------
    # Insert / update / lookup / remove
    # ------------------------------------------------------------------
    def insert(self, station_id, station_type, x=None, y=None, lat=None, lon=None,
               speed=None, heading=None, acceleration=None, detected=False,
               perceived_by=None, timestamp=None, match_connected=False):
        """Insert a new object or update an existing one (same behavior as
        LDM::insert() in C++). Returns LDM_OK on insertion, LDM_UPDATED on update.

        Objects detected by the sensors of a vehicle (as opposed to connected
        objects transmitting their own messages) should be inserted with
        detected=True and perceived_by=<stationID of the detecting vehicle>.
        When the object matching logic is enabled (enable_object_matching()),
        detections of the same physical object coming from different vehicles
        are merged into a single LDM entry.
        This method is actually an insert_or_update()."""
        if (x is None or y is None) and (lat is None or lon is None):
            return LDMError.LDM_UNKNOWN_ERROR

        with self._lock:
            target_id = station_id
            matched_existing = False
            if detected and self._object_matching and perceived_by is not None:
                key = (perceived_by, station_id)
                if key in self._match_map and self._match_map[key] in self._db:
                    # This vehicle already reported this object: keep updating the
                    # same (canonical) entry
                    target_id = self._match_map[key]
                else:
                    match = self._find_matching_object(
                        perceived_by, station_type, x, y, lat, lon)
                    if match is not None:
                        target_id = match.station_id
                        matched_existing = True
                    self._match_map[key] = target_id
            elif not detected and match_connected and self._object_matching and \
                    station_id not in self._db:
                match = self._find_detected_match(
                    station_type, x, y, lat, lon)
                if match is not None:
                    self._promote_to_connected_id(match.station_id, station_id)
                    target_id = station_id
                    matched_existing = True
            if not detected and match_connected:
                self._connected_match_ids.add(station_id)

            retval = LDMError.LDM_UPDATED \
                if target_id in self._db or matched_existing else LDMError.LDM_OK

            # Keep track of all the vehicles perceiving this object
            prev = self._db.get(target_id)
            reporters = set() if prev is None or prev.perceived_by is None \
                else set(prev.perceived_by)
            if perceived_by is not None:
                reporters.add(perceived_by)

            observed_at = time.time() if timestamp is None else float(timestamp)
            obj = LDMObject(station_id=target_id, station_type=station_type,
                            x=x, y=y, lat=lat, lon=lon, speed=speed,
                            heading=heading, acceleration=acceleration,
                            detected=detected,
                            perceived_by=perceived_by,
                            timestamp=observed_at)
            if self._fuse_states:
                source_key = self._source_key(station_id, detected, perceived_by)
                source_map = self._source_states.setdefault(target_id, {})
                previous_source = source_map.get(source_key)
                if previous_source is None or \
                        obj.timestamp >= previous_source.timestamp:
                    source_map[source_key] = obj
                obj = self._fused_object(target_id)
            elif reporters:
                obj.perceived_by = reporters
            self._db[target_id] = obj
            self._notify(obj)
        return retval

    def update(self, station_id, station_type, x=None, y=None, lat=None, lon=None,
               speed=None, heading=None, acceleration=None, detected=False,
               perceived_by=None, timestamp=None, match_connected=False):
        """Update an existing object; returns LDM_ITEM_NOT_FOUND if it does not exist."""
        with self._lock:
            if station_id not in self._db:
                return LDMError.LDM_ITEM_NOT_FOUND
            return self.insert(station_id, station_type, x=x, y=y, lat=lat, lon=lon,
                               speed=speed, heading=heading, acceleration=acceleration,
                               detected=detected, perceived_by=perceived_by,
                               timestamp=timestamp,
                               match_connected=match_connected)

    def lookup(self, station_id):
        """Return the LDMObject with the given stationID, or None if not found."""
        with self._lock:
            return self._db.get(station_id)

    def remove(self, station_id):
        """Remove the entry with the given stationID."""
        with self._lock:
            if station_id not in self._db:
                return LDMError.LDM_ITEM_NOT_FOUND
            del self._db[station_id]
            self._purge_match_map(station_id)
            self._source_states.pop(station_id, None)
            self._connected_match_ids.discard(station_id)
        return LDMError.LDM_OK

    def clear(self):
        with self._lock:
            self._db.clear()
            self._match_map.clear()
            self._source_states.clear()
            self._connected_match_ids.clear()
            self._recently_rekeyed.clear()

    # ------------------------------------------------------------------
    # Geographical (circular) area lookups
    # ------------------------------------------------------------------
    def range_select(self, range_m, lat, lon):
        """Return the list of LDMObjects within range_m meters from (lat, lon).
        Same semantics as LDM::rangeSelect(range_m, lat, lon, ...) in C++.
        Entries without lat/lon are skipped."""
        selected = []
        with self._lock:
            for obj in self._db.values():
                if obj.lat is None or obj.lon is None:
                    continue
                if geo_utils.geodesic_distance_m(lat, lon, obj.lat, obj.lon) <= range_m:
                    selected.append(obj)
        return selected

    def range_select_xy(self, range_m, x, y):
        """Return the list of LDMObjects within range_m meters from cartesian (x, y).
        Entries without x/y are skipped."""
        selected = []
        with self._lock:
            for obj in self._db.values():
                if obj.x is None or obj.y is None:
                    continue
                if math.hypot(obj.x - x, obj.y - y) <= range_m:
                    selected.append(obj)
        return selected

    def range_select_by_station(self, range_m, station_id):
        """Return the list of LDMObjects within range_m meters from the object with
        the given stationID (included), or LDM_ITEM_NOT_FOUND if it is not stored.
        Same semantics as LDM::rangeSelect(range_m, stationID, ...) in C++."""
        with self._lock:
            center = self._db.get(station_id)
            if center is None:
                return LDMError.LDM_ITEM_NOT_FOUND
            if center.lat is not None and center.lon is not None:
                return self.range_select(range_m, center.lat, center.lon)
            return self.range_select_xy(range_m, center.x, center.y)

    # ------------------------------------------------------------------
    # Utility operations (ported from the C++ LDM)
    # ------------------------------------------------------------------
    def get_all_ids(self):
        with self._lock:
            return set(self._db.keys())

    def get_cardinality(self):
        with self._lock:
            return len(self._db)

    def execute_on_all_contents(self, oper_fcn, additional_args=None):
        """Execute oper_fcn(obj, additional_args) on every stored object."""
        with self._lock:
            objs = list(self._db.values())
        for obj in objs:
            oper_fcn(obj, additional_args)

    def delete_older_than(self, time_milliseconds):
        """Delete all the entries older than time_milliseconds. Returns the list
        of removed stationIDs."""
        now = time.time()
        removed = []
        with self._lock:
            # sid is the statioID = the key of the map
            for sid in list(self._db.keys()):
                if (now - self._db[sid].timestamp) * 1000.0 > time_milliseconds:
                    del self._db[sid]
                    self._purge_match_map(sid)
                    self._source_states.pop(sid, None)
                    self._connected_match_ids.discard(sid)
                    removed.append(sid)
        return removed

    # ------------------------------------------------------------------
    # VAM handling
    # ------------------------------------------------------------------
    def add_vru_from_vam(self, buffer, geo_converter=None, timestamp=None):
        """Decode a UPER-encoded VAM (bytes) using asn1tools and the ASN.1
        specifications imported by vam_codec, extract stationID, stationType,
        position, speed, heading and (if available) longitudinal acceleration,
         and insert/update the VRU in the database (porting of vLDM_handler()).

        If a geo_utils.GeoConverter is provided, the x/y position of the object
        is also computed and stored alongside lat/lon.

        It requires the ASN.1 files ETSI-ITS-CDD.asn and VAM-PDU-Descriptions.asn
        to be available in the directory of this Python file (or in the parent
        directory).

        Returns a tuple (LDMError, station_id); station_id is None on decoding errors.
        """
        try:
            vam = vam_codec.decode_vam(buffer)
        except Exception:
            return LDMError.LDM_UNKNOWN_ERROR, None

        try:
            if vam["header"]["messageId"] != vam_codec.FIX_VAMID:
                return LDMError.LDM_UNKNOWN_ERROR, None

            station_id = vam["header"]["stationId"]
            params = vam["vam"]["vamParameters"]
            basic = params["basicContainer"]
            hf = params["vruHighFrequencyContainer"]

            station_type = basic["stationType"]
            lat = vam_codec.etsi_to_lat_deg(basic["referencePosition"]["latitude"])
            lon = vam_codec.etsi_to_lon_deg(basic["referencePosition"]["longitude"])
            heading = vam_codec.etsi_to_heading_deg(hf["heading"]["value"])
            speed = vam_codec.etsi_to_speed_ms(hf["speed"]["speedValue"])
            acceleration = vam_codec.etsi_to_acceleration_ms2(
                hf["longitudinalAcceleration"]["longitudinalAccelerationValue"])
        except (KeyError, TypeError):
            return LDMError.LDM_UNKNOWN_ERROR, None

        x = y = None
        if geo_converter is not None and lat is not None and lon is not None:
            x, y = geo_converter.latlon_to_xy(lat, lon)

        retval = self.insert(station_id, station_type, x=x, y=y, lat=lat, lon=lon,
                             speed=speed, heading=heading, acceleration=acceleration,
                             timestamp=timestamp, match_connected=True)
        return retval, station_id
