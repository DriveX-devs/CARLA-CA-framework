"""
t2c.py - Python port of the Collision Avoidance algorithm

The kinematic state of the entities (position, speed, heading, acceleration)
is passed in explicitly via `EntityState`.

Position may be given either in Cartesian (projected) metres via x/y, or in
geographic degrees via lat/lon, in which case it is projected to a shared local
Cartesian frame with geographiclib, which becomes a required dependency.

Accelerations are optional: when unavailable (None) the entities are treated as
moving at constant velocity and the algorithm still runs.

CSV metric logging is performed only when enabled through
setT2Cmetricsfile(..., collect_metrics=True)
In TimeToCollision(), the complex-roots branch uses atan2(imm, real)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional

NO_COLLISION = -1.0
PI = math.pi

# ETSI CDD unit converstion constants and unavailable values
# Each raw CAM field is an integer in a fixed unit; divide by the factor to get
# SI/degrees, and treat the unavailable values as "value not provided"
ETSI_LAT_FACTOR = 1e7          # Latitude in 0.1 microdegrees
ETSI_LON_FACTOR = 1e7          # Longitude in 0.1 microdegrees
ETSI_SPEED_FACTOR = 100.0      # SpeedValue in 0.01 m/s
ETSI_HEADING_FACTOR = 10.0     # HeadingValue in 0.1 deg
ETSI_ACCEL_FACTOR = 10.0       # LongitudinalAccelerationValue in 0.1 m/s^2
ETSI_ALTITUDE_FACTOR = 100.0   # AltitudeValue in 0.01 m

LATITUDE_UNAVAILABLE = 900000001
LONGITUDE_UNAVAILABLE = 1800000001
SPEED_UNAVAILABLE = 16383
HEADING_UNAVAILABLE = 3601
LONGITUDINAL_ACCELERATION_UNAVAILABLE = 161
ALTITUDE_UNAVAILABLE = 800001


# --------------------------------------------------------------------------
# Geographic projection (lat/lon -> local Cartesian x,y)
# --------------------------------------------------------------------------
# The collision math works in a projected Cartesian frame (meters). When the
# input is geographic (lat, lon) we project it onto a local tangent plane using
# geographiclib. geographiclib is imported optionally, so the module
# keeps working with pure x/y input even if the package is not installed

try:
    from geographiclib.geodesic import Geodesic
    _HAS_GEOGRAPHICLIB = True
except ImportError: # pragma: no cover - optional dependency
    Geodesic = None
    _HAS_GEOGRAPHICLIB = False


class GeoProjection:
    """
    Azimuthal-equidistant projection onto a local tangent plane, built on
    geographiclib's geodesic solver on the WGS84 ellipsoid.

    A single projection instance defines one common reference origin so that
    every entity ends up in the same Cartesian frame (essential: collision
    geometry only makes sense when all positions share a frame). The axes match
    the ETSI heading convention used throughout this module:
        x = East [m], y = North [m]  ->  vx = v*sin(heading), vy = v*cos(heading)

    If the origin is not given explicitly it is auto-anchored on the first point
    projected. Distances and angles near the origin are preserved to high
    accuracy, which is all the detector needs over its few-hundred-metre range.
    """

    def __init__(self, lat0: Optional[float] = None, lon0: Optional[float] = None):
        if not _HAS_GEOGRAPHICLIB:
            raise ImportError(
                "geographiclib is required to use (lat, lon) input; install it "
                "with `pip3 install geographiclib`, or pass already projected x/y instead."
            )
        self.geod = Geodesic.WGS84
        self.lat0 = lat0
        self.lon0 = lon0

    def set_origin(self, lat0: float, lon0: float):
        self.lat0 = lat0
        self.lon0 = lon0

    def forward(self, lat: float, lon: float):
        # Project (lat, lon) [deg] to (x_east, y_north) [m] in the local frame
        if self.lat0 is None or self.lon0 is None:
            # auto-anchor at first point for which projection is requested
            self.lat0, self.lon0 = lat, lon
        g = self.geod.Inverse(self.lat0, self.lon0, lat, lon)
        s = g["s12"] # geodesic distance [m]
        azi = math.radians(g["azi1"]) # azimuth from North, clockwise
        return s * math.sin(azi), s * math.cos(azi)


# Shared default projection: entities built from lat/lon without an explicit
# projection are all anchored to the first geographic point ever seen, so they
# land in one consistent frame.
_default_projection: Optional[GeoProjection] = None


def get_default_projection() -> GeoProjection:
    # Return (creating on first use) the shared, auto-anchored projection
    global _default_projection
    if _default_projection is None:
        _default_projection = GeoProjection()
    return _default_projection


def reset_default_projection(lat0: Optional[float] = None,
                             lon0: Optional[float] = None) -> None:
    # Reset the shared projection, optionally pinning its origin. Call this before
    # building any lat/lon entities to get a fixed, reproducible frame origin
    # instead of auto-anchoring on the first point seen
    global _default_projection
    _default_projection = GeoProjection(lat0, lon0)


# --------------------------------------------------------------------------
# Data containers
# --------------------------------------------------------------------------

@dataclass
class EntityState:
    """
    Kinematic state of a road user (vehicle, VRU).

    Position can be given in either of two ways:
    - already projected Cartesian metres, via x and y, or
    - geographic degrees, via lat and lon -> these are projected to x/y with
      geographiclib (see GeoProjection). You can pass a shared "projection"
      so that every entity lands in the same frame; if omitted, the module-wide
      default projection (auto-anchored on the first point) is used.

    Accelerations are optional: acc_lon/acc_lat can be set as None (or 0.0) when
    they are not available and the entity is treated as moving at constant
    velocity... and the algorithm still works!

    heading_deg follows the ETSI convention: degrees clockwise from North, so
    that vx = v * sin(heading), vy = v * cos(heading).
    """
    station_id: str
    x: Optional[float] = None                               # projected Cartesian x [m] (or derived from lon)
    y: Optional[float] = None                               # projected Cartesian y [m] (or derived from lat)
    speed_ms: float = 0.0                                   # module of the speed vector [m/s]
    heading_deg: float = 0.0                                # heading w.r.t. North [deg]
    acc_lon: Optional[float] = None                         # longitudinal acceleration [m/s^2]; None = N/A
    acc_lat: Optional[float] = None                         # lateral acceleration [m/s^2]; None = N/A
    z: Optional[float] = None                               # elevation [m]; None/unset -> 0.0
    timestamp: float = field(default_factory=time.time)     # t_last [s]
    lat: Optional[float] = None                             # latitude [deg]  (alternative to y)
    lon: Optional[float] = None                             # longitude [deg] (alternative to x)
    projection: Optional[GeoProjection] = None              # frame for the lat/lon -> x/y map

    def __post_init__(self):
        # Elevation is optional: default any unset value to 0 for all entities.
        if self.z is None:
            self.z = 0.0

        # Resolve position: accept either Cartesian x/y or geographic lat/lon.
        if self.x is None or self.y is None:
            if self.lat is None or self.lon is None:
                raise ValueError(
                    f"EntityState '{self.station_id}': must provide either x and y "
                    f"(projected metres) or lat and lon (degrees)."
                )
            proj = self.projection if self.projection is not None else get_default_projection()
            
            self.x, self.y = proj.forward(self.lat, self.lon)
            self.projection = proj

    @classmethod
    def from_cam(cls, station_id,
                 latitude, longitude, speed_value, heading_value,
                 longitudinal_acceleration_value=LONGITUDINAL_ACCELERATION_UNAVAILABLE,
                 altitude_value=ALTITUDE_UNAVAILABLE,
                 timestamp=None, projection=None):
        # Build a state directly from raw ETSI CAM fields:
        # latitude/longitude                in 0.1 microdegrees (unavailable sentinels ok)
        # speed_value                       in 0.01 m/s
        # heading_value                     in 0.1 deg
        # longitudinal_acceleration_value   in 0.1 m/s^2 (161 = unavailable)
        # altitude_value                    in 0.01 m (800001 = unavailable)

        # Position is geographic and gets projected to the shared Cartesian frame.
        # Any field carrying its "unavailable" value is decoded to a safe
        # default: 0 for speed/heading, None for acceleration/altitude (so the
        # entity is treated as having no acceleration or zero elevation).
        
        lat = None if latitude == LATITUDE_UNAVAILABLE else latitude/ETSI_LAT_FACTOR
        lon = None if longitude == LONGITUDE_UNAVAILABLE else longitude/ETSI_LON_FACTOR
        speed_ms = 0.0 if speed_value==SPEED_UNAVAILABLE else speed_value/ETSI_SPEED_FACTOR
        heading_deg = 0.0 if heading_value == HEADING_UNAVAILABLE else heading_value/ETSI_HEADING_FACTOR
        acc_lon = None if longitudinal_acceleration_value == LONGITUDINAL_ACCELERATION_UNAVAILABLE else longitudinal_acceleration_value/ETSI_ACCEL_FACTOR
        z = None if altitude_value == ALTITUDE_UNAVAILABLE else altitude_value/ETSI_ALTITUDE_FACTOR

        return cls(station_id=station_id, lat=lat, lon=lon, speed_ms=speed_ms,
                   heading_deg=heading_deg, acc_lon=acc_lon, z=z,
                   projection=projection,
                   timestamp=timestamp if timestamp is not None else time.time())

    @classmethod
    def from_direct(cls, station_id, x=None, y=None, lat=None, lon=None,
                    speed_ms=0.0, heading_deg=0.0, acc_lon=None, acc_lat=None,
                    z=None, timestamp=None, projection=None):
        # Build a state from real-world units: position as projected metres (x,y)
        # or geographic degrees (lat, lon), speed in m/s, heading in degrees,
        # accelerations in m/s^2, elevation in m. Accelerations/elevation may be
        # left as None when unavailable.

        return cls(station_id=station_id, x=x, y=y, lat=lat, lon=lon,
                   speed_ms=speed_ms, heading_deg=heading_deg,
                   acc_lon=acc_lon, acc_lat=acc_lat, z=z, projection=projection,
                   timestamp=timestamp if timestamp is not None else time.time())

    # Derived kinematics

    @property
    def has_acceleration(self) -> bool:
        # True if any acceleration component is available (not None)
        return self.acc_lon is not None or self.acc_lat is not None

    @property
    def heading_rad(self) -> float:
        return (self.heading_deg * PI) / 180.0

    @property
    def vx(self) -> float:
        return self.speed_ms * math.sin(self.heading_rad)

    @property
    def vy(self) -> float:
        return self.speed_ms * math.cos(self.heading_rad)

    @property
    def ax(self) -> float:
        # ax = acc_lon*sin(h) + acc_lat*cos(h) (as in the previous C++ implementation t2c.cc)
        # Missing (None) accelerations are treated as 0 -> constant velocity.
        acc_lon = self.acc_lon or 0.0
        acc_lat = self.acc_lat or 0.0
        return acc_lon * math.sin(self.heading_rad) + \
               acc_lat * math.cos(self.heading_rad)

    @property
    def ay(self) -> float:
        # ay = acc_lon*cos(h) + acc_lat*sin(h) (as in the previous C++ implementation t2c.cc)
        acc_lon = self.acc_lon or 0.0
        acc_lat = self.acc_lat or 0.0
        return acc_lon * math.cos(self.heading_rad) + \
               acc_lat * math.sin(self.heading_rad)


@dataclass
class CollisionWarning:
    # One detected collision risk between two entities: the pair to which a
    # warning should be sent.
    entity: EntityState       # the entity whose update triggered the check
    other: EntityState        # the other entity on collision course
    collision_type: str       # "Cross-Road-Collision" or "Rear-End-Collision"
    t2c: float                # Time-to-Collision [s]
    s2c: float                # Space-to-Collision [m]

    @property
    def id(self) -> str:
        # station_id of the triggering entity
        return self.entity.station_id

    @property
    def other_id(self) -> str:
        # station_id of the other entity on collision course
        return self.other.station_id


# --------------------------------------------------------------------------
# Possible useful helper functions
# --------------------------------------------------------------------------

def haversineDist(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    # Great-circle distance [m] between two (lat, lon) points in degrees.
    # Good for estimating relatively short distace between two points expressed
    # in geodetic coordinates
    R = 6371000.0
    phi1, phi2 = math.radians(lat_a), math.radians(lat_b)
    dphi = math.radians(lat_b - lat_a)
    dlmb = math.radians(lon_b - lon_a)
    a = math.sin(dphi/2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# --------------------------------------------------------------------------------------------
# Main CA algorithm class - computing the key metric t2c (or T2C), i.e., the time-to-collision
# --------------------------------------------------------------------------------------------

class t2c:
    # The receiver is stored as an EntityState set through setReceiverState()
    def __init__(self,
                 node_id: str = "null",
                 T2C_th: float = 1.0):
        self.m_id = node_id
        self.m_T2Cth = T2C_th

        self.m_t2c_metrics = False
        self.m_csv_t2c_file_name = ""

        self._receiver: Optional[EntityState] = None

    def setReceiverState(self, state: EntityState) -> None:
        self._receiver = state
        self.m_id = state.station_id

    def setT2Cmetricsfile(self, file_name: str, collect_metrics: bool) -> None:
        self.m_t2c_metrics = collect_metrics
        self.m_csv_t2c_file_name = file_name

    def _log_csv(self, line: str) -> None:
        if self.m_t2c_metrics and self.m_csv_t2c_file_name:
            with open(self.m_csv_t2c_file_name, "a") as f:
                f.write(line)

    # IsInRange function - Algorithm 2
    def IsInRange(self,
                  x_sender: float, y_sender: float,
                  vx_sender: float, vy_sender: float,
                  x_receiver: float, y_receiver: float,
                  vx_receiver: float, vy_receiver: float) -> bool:
        # Future positions of sender and receiver in T2Cmax (constant speed)
        pos_sender_x_fut = x_sender + vx_sender * self.m_T2Cth
        pos_sender_y_fut = y_sender + vy_sender * self.m_T2Cth
        pos_receiver_x_fut = x_receiver + vx_receiver * self.m_T2Cth
        pos_receiver_y_fut = y_receiver + vy_receiver * self.m_T2Cth

        # travelled distances, take the max
        distance_sender = math.hypot(pos_sender_x_fut - x_sender,
                                     pos_sender_y_fut - y_sender)
        distance_receiver = math.hypot(pos_receiver_x_fut - x_receiver,
                                       pos_receiver_y_fut - y_receiver)
        max_distance = max(distance_receiver, distance_sender)

        max_distance *= math.sqrt(2.0)  # d_max = sqrt(2*d^2), Algorithm 2 line 8

        # actual sender-receiver distance
        distance_sr = math.hypot(x_receiver - x_sender, y_receiver - y_sender)
        return distance_sr < max_distance

    # TimeToCollision - Algorithm 3
    def TimeToCollision(self,
                        x1: float, y1: float, vx1: float, vy1: float,
                        ax1: float, ay1: float,
                        x2: float, y2: float, vx2: float, vy2: float,
                        ax2: float, ay2: float) -> float:
        """
        This function returns the time [s] at which the distance D(t) between
        the two entities is minimum, assuming locally uniformly accelerated motion.
        Returns NO_COLLISION (-1) when no non-negative minimum exists or the
        entities are moving apart.

        Direct port of the C++ analytic solver: dD(t)/dt = 0 is a cubic in t,
        solved via Cardano's depressed cubic + Ruffini factorisation.
        """
        delta_x = x1 - x2
        delta_y = y1 - y2
        delta_vx = vx1 - vx2
        delta_vy = vy1 - vy2

        t2c_x = -delta_x * delta_vx
        t2c_y = -delta_y * delta_vy
        t2c_v = delta_vx * delta_vx + delta_vy * delta_vy

        delta_ax = ax1 - ax2
        delta_ay = ay1 - ay2

        # Constant-speed t2c: valid only if the entities are approaching
        # along both components (t2c_x, t2c_y >= 0) and moving relative to
        # each other (t2c_v != 0).
        if t2c_x >= 0 and t2c_y >= 0 and t2c_v != 0:
            t2c_val = (t2c_x + t2c_y) / t2c_v
        else:
            t2c_val = NO_COLLISION

        # Negligible relative acceleration -> constant-speed result
        if abs(delta_ax) < 0.05 and abs(delta_ay) < 0.05:
            return t2c_val

        # Acceleration branch: solve the cubic dD/dt = 0
        if t2c_val == NO_COLLISION:
            return NO_COLLISION

        t_min = NO_COLLISION

        # Cubic coefficients A t^3 + B t^2 + C t + D = 0
        A = delta_ax ** 2 + delta_ay ** 2
        B = 3 * (delta_vx * delta_ax + delta_vy * delta_ay)
        C = 2 * (delta_vx ** 2 + delta_vy ** 2 + delta_x * delta_ax + delta_y * delta_ay)
        D = 2 * (delta_x * delta_vx + delta_y * delta_vy)

        # Depressed cubic Y^3 + P Y + Q = 0  (t = Y - B/(3A))
        P = (C / A) - (B ** 2 / (3 * A ** 2))
        Q = (D / A) - ((B * C) / (3 * A ** 2)) + ((2 * B ** 3) / (27 * A ** 3))
        det = (Q ** 2 / 4) + (P ** 3 / 27)

        if abs(Q) < 0.0001:
            # Q = 0 --> rear-end collision case
            t1 = -(B / (3 * A))              # Y = 0
            # Ruffini with t1
            A1 = A
            B1 = B + A1 * t1
            C1 = C + B1 * t1
            det1 = B1 ** 2 - 4 * A1 * C1
            if det1 > 0:
                t2 = (-B1 - math.sqrt(det1)) / (2 * A1)
                t3 = (-B1 + math.sqrt(det1)) / (2 * A1)
            else:
                t2 = NO_COLLISION
                t3 = NO_COLLISION
        else:
            # Q != 0 --> crossing collisions
            if det > 0:
                # One real root
                Y = math.copysign(abs(-(Q / 2) + math.sqrt(det)) ** (1 / 3),
                                  -(Q / 2) + math.sqrt(det)) + \
                    math.copysign(abs(-(Q / 2) - math.sqrt(det)) ** (1 / 3),
                                  -(Q / 2) - math.sqrt(det))
                t1 = Y - (B / (3 * A))
                # Ruffini with t1
                A1 = A
                B1 = B + A1 * t1
                C1 = C + B1 * t1
                det1 = B1 ** 2 - 4 * A1 * C1
                if det1 > 0:
                    t2 = (-B1 - math.sqrt(det1)) / (2 * A1)
                    t3 = (-B1 + math.sqrt(det1)) / (2 * A1)
                else:
                    t2 = NO_COLLISION
                    t3 = NO_COLLISION
            else:
                # Complex conjugate pair --> three real roots (trigonometric)
                real = -Q / 2
                imm = math.sqrt(-det)
                # atan2(imm, real) == atan(imm/real)      if real > 0
                #                  == atan(imm/real) + PI  if real < 0
                # (safe when real == 0)
                teta = math.atan2(imm, real)
                Y3 = 2 * math.sqrt(-P / 3) * math.cos(teta / 3)
                t3 = Y3 - (B / (3 * A))
                Y2 = 2 * math.sqrt(-P / 3) * math.cos((teta + 2 * PI) / 3)
                t2 = Y2 - (B / (3 * A))
                Y1 = 2 * math.sqrt(-P / 3) * math.cos((teta + 4 * PI) / 3)
                t1 = Y1 - (B / (3 * A))

        # Smallest non-negative solution (same case structure as t2c.cc)
        if t2 == NO_COLLISION and t3 == NO_COLLISION:
            # CASE 1: paraboloid, single minimum in t1
            t_min = NO_COLLISION if t1 < 0 else t1
        elif t2 != NO_COLLISION and t3 != NO_COLLISION:
            # CASE 2: two minimum points (in t2 and t3)
            if t2 < 0 and t3 < 0:
                t_min = NO_COLLISION
            elif t2 < 0 and t3 > 0:
                t_min = t3
            elif t2 > 0 and t3 < 0:
                t_min = t2
            elif t2 > 0 and t3 > 0:
                t_min = min(t2, t3)

        return t_min

    def computeT2C(self, sender: EntityState,
                   receiver: Optional[EntityState] = None) -> float:
        rcv = receiver if receiver is not None else self._receiver
        if rcv is None:
            raise ValueError("Receiver state not set: call setReceiverState() first")

        t2c_val = self.TimeToCollision(
            rcv.x, rcv.y, rcv.vx, rcv.vy, rcv.ax, rcv.ay,
            sender.x, sender.y, sender.vx, sender.vy, sender.ax, sender.ay)

        # CSV metrics
        dist = math.hypot(rcv.x - sender.x, rcv.y - sender.y)
        ts_ms = int(time.time() * 1000)
        self._log_csv(f"{ts_ms},{dist},ped{sender.station_id},{self.m_id},"
                      f"{rcv.heading_rad},{rcv.vx},{rcv.vy},{rcv.ax},{rcv.ay},"
                      f"{sender.heading_rad},{sender.vx},{sender.vy},"
                      f"{sender.ax},{sender.ay},{t2c_val},")
        return t2c_val

    # Algorithm 4: Space-to-Collision (s2c) -> distance between the entities at t = t2c
    def computes2c(self,
                   x_sender: float, y_sender: float,
                   vx_sender: float, vy_sender: float,
                   ax_sender: float, ay_sender: float,
                   x_receiver: float, y_receiver: float,
                   vx_receiver: float, vy_receiver: float,
                   ax_receiver: float, ay_receiver: float,
                   t2c_val: float) -> float:
        # Space-to-Collision [m]: distance between the entities at t = t2c.
        x_sender_fut = x_sender + vx_sender * t2c_val + 0.5 * ax_sender * t2c_val ** 2
        y_sender_fut = y_sender + vy_sender * t2c_val + 0.5 * ay_sender * t2c_val ** 2
        x_receiver_fut = x_receiver + vx_receiver * t2c_val + 0.5 * ax_receiver * t2c_val ** 2
        y_receiver_fut = y_receiver + vy_receiver * t2c_val + 0.5 * ay_receiver * t2c_val ** 2
        return math.hypot(x_sender_fut - x_receiver_fut,
                          y_sender_fut - y_receiver_fut)

    # Convenience alias with the pseudocode name (Algorithm 4)
    def SpaceToCollision(self, sender: EntityState, receiver: EntityState,
                         t2c_val: float) -> float:
        return self.computes2c(sender.x, sender.y, sender.vx, sender.vy,
                               sender.ax, sender.ay,
                               receiver.x, receiver.y, receiver.vx, receiver.vy,
                               receiver.ax, receiver.ay, t2c_val)

    # ---- CheckCollision ---------------------------------------------------
    # Run the full t2c + s2c check on two entities given directly by their
    # kinematics (velocity components dx/dy [m/s] and accelerations ax/ay [m/s^2]).
    def CheckCollision(self,
                       x1: float, y1: float, dx1: float, dy1: float,
                       ax1: float, ay1: float,
                       x2: float, y2: float, dx2: float, dy2: float,
                       ax2: float, ay2: float,
                       t2c_threshold: Optional[float] = None,
                       s2c_threshold: float = 4.2,
                       scale_with_speed: bool = True):
        
        if t2c_threshold is None:
            t2c_threshold = self.m_T2Cth

        if scale_with_speed:
            # Adapt the t2c threshold to the speed of the faster car.
            # Campaign callers can disable this to match the edge service's
            # fixed self.t2c_th comparison exactly.
            max_speed = 13.89
            speed1 = math.hypot(dx1, dy1)
            speed2 = math.hypot(dx2, dy2)
            if speed1 >= max_speed:
                max_speed = speed1 * 0.7
            if speed2 >= max_speed:
                max_speed = speed2 * 0.7
            if max_speed < 13.89:
                max_speed = 13.89
            t2c_threshold = t2c_threshold / 13.89 * max_speed

        # Time to collision (time from now for the two to reach minimum distance)
        t2c_val = self.TimeToCollision(x1, y1, dx1, dy1, ax1, ay1,
                                       x2, y2, dx2, dy2, ax2, ay2)

        # No collision if t2c is NO_COLLISION or beyond the effective threshold
        if t2c_val == NO_COLLISION or t2c_val > t2c_threshold:
            return NO_COLLISION, NO_COLLISION

        # Space to collision (distance between the two at now + t2c)
        s2c_val = self.computes2c(x1, y1, dx1, dy1, ax1, ay1,
                                  x2, y2, dx2, dy2, ax2, ay2, t2c_val)

        # Safe if the closest-approach distance exceeds the threshold
        if s2c_val > s2c_threshold:
            return NO_COLLISION, NO_COLLISION

        # Both t2c and s2c are within their thresholds -> collision risk
        return t2c_val, s2c_val

# ------------------------------------------
# Collision Avoidance Service -- Algorithm 1
# ------------------------------------------
class CollisionAvoidanceService:
    """
    Centralized cross-road-only CAS: keeps a map of the latest EntityState of
    every known road user (each one either received from the network or locally
    sensed) and, every time a state is added or updated via update(), checks that
    entity against all other tracked entities for collision risk (Algorithm 1).
    Rear-end pairs are classified but always excluded before the kinematic risk
    calculation.

    Typical use -> feed states in as they arrive and react to the warnings:

        cas = CollisionAvoidanceService(on_warning=callable_fcn)
        while True:
            state = receive_or_sense()          # an EntityState
            cas.update(state)                    # -> list of CollisionWarning

    Default thresholds are the tuned values from the document (urban
    scenario, 50 km/h speed limit):
        alpha_th = 17 deg, t2c_th = 4.8 s, s2c_th = 4.2 m.
    """

    REAR_END = "Rear-End-Collision"
    CROSS_ROAD = "Cross-Road-Collision"

    def __init__(self,
                 alpha_th_deg: float = 17.0,
                 t2c_th: float = 4.8,
                 s2c_th: float = 4.2,
                 stale_after_s: float = 3.0,
                 prune_stale: bool = True,
                 on_warning=None):
        """
        prune_stale: drop entities from the map once they are older than
        stale_after_s (keeps the map from growing without bound). Stale entries
        are ignored when checking risk regardless of this flag.
        By default, stale entries are removed after 3 seconds.

        on_warning: optional callable invoked once per detected CollisionWarning
        as it is found. Warnings are also returned.
        """
        self.alpha_th_deg = alpha_th_deg
        self.t2c_th = t2c_th
        self.s2c_th = s2c_th
        self.stale_after_s = stale_after_s
        self.prune_stale = prune_stale
        self.on_warning = on_warning

        # map of the latest state of every known entity, keyed by station_id
        self.entities: Dict[str, EntityState] = {}
        # internal t2c engine: T2C_th plays the role of t2c_th in IsInRange
        self._t2c = t2c(T2C_th=t2c_th)

    # Helper methods
    @staticmethod
    def _heading_diff_deg(a: float, b: float) -> float:
        # This method computes the minimal absolute angular difference in [0, 180] deg
        d = abs(a - b) % 360.0
        return 360.0 - d if d > 180.0 else d

    def classify(self, v: EntityState, w: EntityState) -> str:
        # |alpha_v - alpha_w| < alpha_th  ->  rear-end, else cross-road.
        if self._heading_diff_deg(v.heading_deg, w.heading_deg) < self.alpha_th_deg:
            return self.REAR_END
        return self.CROSS_ROAD

    # Vehicles map (dict) management

    def get(self, station_id: str) -> Optional[EntityState]:
        # Return the last known state of an entity, or None if not tracked
        return self.entities.get(station_id)

    def remove(self, station_id: str) -> None:
        # Forget an entity (e.g. it left the area / stopped transmitting)
        self.entities.pop(station_id, None)

    def prune(self, now: Optional[float] = None) -> None:
        # Drop entities older than stale_after_s from the map
        now = now if now is not None else time.time()
        stale = [sid for sid, s in self.entities.items() if now - s.timestamp > self.stale_after_s]
        for sid in stale:
            del self.entities[sid]

    # Algorithm 1: add/update an entity and check collision risk ---------

    def update(self, state: EntityState,
               now: Optional[float] = None,
               observation_time: Optional[float] = None) -> List[CollisionWarning]:
        # Add or update one entity in the map and immediately check it against
        # every other tracked entity for collision risk

        # Call this each time a new EntityState arrives (from the network or a
        # local sensor). Returns the list of CollisionWarning found for this
        # entity (the <v, w> pairs a warning would be sent to); each warning is also
        # passed to the on_warning callback if one was provided.
        now = now if now is not None else time.time()

        # Preserve when the state was observed. A delayed network delivery is
        # projected to `now` below rather than treated as a fresh observation.
        state.timestamp = now if observation_time is None else observation_time
        previous = self.entities.get(state.station_id)
        if previous is not None and previous.timestamp > state.timestamp:
            # An out-of-order packet must not roll the canonical state backward.
            return []
        self.entities[state.station_id] = state

        # Optionally drop entries that have gone stale
        if self.prune_stale:
            self.prune(now)

        if now - state.timestamp > self.stale_after_s:
            self.entities.pop(state.station_id, None)
            return []

        warnings = self._check_against_all(self._state_at(state, now), now)

        if self.on_warning is not None:
            for w in warnings:
                self.on_warning(w)

        return warnings

    @staticmethod
    def _state_at(state: EntityState, timestamp: float) -> EntityState:
        """Project a sampled state to a common CA evaluation timestamp."""
        dt = max(0.0, float(timestamp) - float(state.timestamp))
        if dt == 0.0:
            return state
        return replace(
            state,
            x=state.x + state.vx * dt + 0.5 * state.ax * dt * dt,
            y=state.y + state.vy * dt + 0.5 * state.ay * dt * dt,
            timestamp=timestamp,
        )

    def _check_against_all(self, v: EntityState,
                           now: float) -> List[CollisionWarning]:
        # Core loop: check entity v against all other (fresh) entities in the map
        warnings: List[CollisionWarning] = []

        # for all b_w in map \ {v}
        for id_w, b_w in self.entities.items():
            if id_w == v.station_id:
                continue

            # Ignore outdated entries (older than stale_after_s)
            if now - b_w.timestamp > self.stale_after_s:
                continue
            b_w = self._state_at(b_w, now)

            # classify the potential collision type
            c = self.classify(v, b_w)

            # This research pipeline evaluates cross-road collisions only.
            if c == self.REAR_END:
                continue

            # coarse range check
            r = self._t2c.IsInRange(v.x, v.y, v.vx, v.vy,
                                    b_w.x, b_w.y, b_w.vx, b_w.vy)
            if not r:
                continue

            # t2c
            t2c_val = self._t2c.TimeToCollision(
                v.x, v.y, v.vx, v.vy, v.ax, v.ay,
                b_w.x, b_w.y, b_w.vx, b_w.vy, b_w.ax, b_w.ay)

            # discard invalid / too far in the future
            if t2c_val < 0 or t2c_val > self.t2c_th:
                continue

            # s2c
            s2c_val = self._t2c.computes2c(
                v.x, v.y, v.vx, v.vy, v.ax, v.ay,
                b_w.x, b_w.y, b_w.vx, b_w.vy, b_w.ax, b_w.ay, t2c_val)

            # collision risk detected -> "send DENM to v and w"
            if s2c_val <= self.s2c_th:
                warnings.append(CollisionWarning(
                    entity=v, other=b_w,
                    collision_type=c, t2c=t2c_val, s2c=s2c_val))

        return warnings

    # Convenience function: single-pair risk check
    def check_pair(self, v: EntityState, w: EntityState):
        # This function evaluate a single pair, ignoring the database and staleness
        # It returns (risk: bool, t2c: float, s2c: float, collision_type: str)
        c = self.classify(v, w)
        if c == self.REAR_END:
            return False, NO_COLLISION, float("nan"), c
        t2c_val = self._t2c.TimeToCollision(v.x, v.y, v.vx, v.vy, v.ax, v.ay,
                                            w.x, w.y, w.vx, w.vy, w.ax, w.ay)
        if t2c_val < 0 or t2c_val > self.t2c_th:
            return False, t2c_val, float("nan"), c
        s2c_val = self._t2c.computes2c(v.x, v.y, v.vx, v.vy, v.ax, v.ay,
                                       w.x, w.y, w.vx, w.vy, w.ax, w.ay, t2c_val)
        return s2c_val <= self.s2c_th, t2c_val, s2c_val, c
