#
# geo_utils.py
#
# Conversion utilities between local cartesian coordinates (x/y, in meters,
# with respect to a geographical reference point) and WGS84 latitude/longitude.
# geopy (geodesic model) is used when available; otherwise a simple
# equirectangular approximation is used as fallback.
#

import math

try:
    from geopy.distance import geodesic, distance as _geopy_distance
    GEOPY_AVAILABLE = True
except ImportError:
    GEOPY_AVAILABLE = False

EARTH_RADIUS_M = 6371008.8


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters between two WGS84 points."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def geodesic_distance_m(lat1, lon1, lat2, lon2):
    """Distance in meters between two WGS84 points (geodesic if geopy is available)."""
    if GEOPY_AVAILABLE:
        return geodesic((lat1, lon1), (lat2, lon2)).meters
    return haversine_m(lat1, lon1, lat2, lon2)


class GeoConverter:
    """Converts between a local ENU-like cartesian frame (x = east, y = north,
    in meters, centered on a reference lat/lon) and WGS84 coordinates."""

    def __init__(self, ref_lat=0.0, ref_lon=0.0):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon

    def set_reference(self, ref_lat, ref_lon):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon

    def xy_to_latlon(self, x, y):
        """(x [m east], y [m north]) -> (lat, lon)."""
        if GEOPY_AVAILABLE:
            # Move north (or south) first, then east (or west)
            p = _geopy_distance(meters=abs(y)).destination(
                (self.ref_lat, self.ref_lon), bearing=0 if y >= 0 else 180)
            p = _geopy_distance(meters=abs(x)).destination(
                (p.latitude, p.longitude), bearing=90 if x >= 0 else 270)
            return p.latitude, p.longitude
        lat = self.ref_lat + math.degrees(y / EARTH_RADIUS_M)
        lon = self.ref_lon + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(self.ref_lat))))
        return lat, lon

    def latlon_to_xy(self, lat, lon):
        """(lat, lon) -> (x [m east], y [m north]) with respect to the reference point."""
        if GEOPY_AVAILABLE:
            x = geodesic((lat, self.ref_lon), (lat, lon)).meters
            y = geodesic((self.ref_lat, lon), (lat, lon)).meters
        else:
            x = haversine_m(lat, self.ref_lon, lat, lon)
            y = haversine_m(self.ref_lat, lon, lat, lon)
        if lon < self.ref_lon:
            x = -x
        if lat < self.ref_lat:
            y = -y
        return x, y
