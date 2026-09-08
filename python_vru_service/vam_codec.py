#
# vam_codec.py
#
# ASN.1 UPER encoding/decoding of ETSI VAMs (VRU Awareness Messages) based on
# asn1tools and the ASN.1 specifications ETSI-ITS-CDD.asn and VAM-PDU-Descriptions.asn.
#
# This module also gathers all the unit conversion factors and the "unavailable"
# special values defined in ETSI-ITS-CDD.
#

import os
import threading

import asn1tools

# VAM messageId as defined in ETSI TS 103 300-3
FIX_VAMID = 16

# Unit conversion factors (same naming convention as asn_utils.h)
DOT_ONE_MICRO = 1e7 # degrees -> 0.1 microdegrees (latitude/longitude)
CENTI = 100.0 # m/s -> cm/s, m -> cm
DECI = 10.0 # degrees -> 0.1 degrees, m/s^2 -> 0.1 m/s^2

# "unavailable"/"out of range" special values from ETSI-ITS-CDD
LATITUDE_UNAVAILABLE = 900000001
LONGITUDE_UNAVAILABLE = 1800000001
ALTITUDE_VALUE_UNAVAILABLE = 800001
WGS84_ANGLE_VALUE_UNAVAILABLE = 3601 # HeadingValue/Wgs84AngleValue unavailable
WGS84_ANGLE_CONFIDENCE_UNAVAILABLE = 127
SPEED_VALUE_UNAVAILABLE = 16383
SPEED_CONFIDENCE_UNAVAILABLE = 127
LONG_ACCELERATION_VALUE_UNAVAILABLE = 161
ACCELERATION_CONFIDENCE_UNAVAILABLE = 102
SEMI_AXIS_LENGTH_UNAVAILABLE = 4095
SEMI_AXIS_ORIENTATION_UNAVAILABLE = 3601

# Common ETSI station types
STATION_TYPE_UNKNOWN = 0
STATION_TYPE_PEDESTRIAN = 1
STATION_TYPE_CYCLIST = 2
STATION_TYPE_MOPED = 3
STATION_TYPE_MOTORCYCLE = 4
STATION_TYPE_PASSENGER_CAR = 5

# ITS timestamps are expressed in milliseconds since 2004-01-01T00:00:00.000Z (TAI)
ITS_EPOCH_S = 1072915200.0

_ASN_FILES = ("ETSI-ITS-CDD.asn", "VAM-PDU-Descriptions.asn")

_spec = None
_spec_lock = threading.Lock()


def _asn_search_dirs():
    """ASN.1 files are looked up in $VAM_ASN_DIR first, then in the folder
    containing this module, then in its parent (the ns-3-dev root when the
    package lives there), then in the current directory."""
    dirs = []
    env_dir = os.environ.get("VAM_ASN_DIR")
    if env_dir:
        dirs.append(env_dir)
    dirs.append(os.path.dirname(os.path.abspath(__file__)))
    dirs.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    dirs.append(os.getcwd())
    return dirs


def get_spec():
    """Return the compiled (and cached) asn1tools UPER specification for VAMs.
    Caching is done to be more efficient and avoid calling multiple times
    compile_files()."""
    global _spec
    with _spec_lock:
        if _spec is None:
            last_err = None
            for d in _asn_search_dirs():
                files = [os.path.join(d, f) for f in _ASN_FILES]
                if all(os.path.isfile(f) for f in files):
                    try:
                        _spec = asn1tools.compile_files(files, "uper")
                        break
                    except Exception as e:
                        last_err = e
            if _spec is None:
                raise FileNotFoundError(
                    "Could not find/compile %s in any of %s (%s)"
                    % (_ASN_FILES, _asn_search_dirs(), last_err)
                )
        return _spec


def encode_vam(vam_dict):
    """UPER-encode a VAM given as a Python dictionary. Returns bytes."""
    return get_spec().encode("VAM", vam_dict)


def decode_vam(buffer):
    """UPER-decode a VAM from bytes. Returns a Python dictionary."""
    return get_spec().decode("VAM", bytes(buffer))


# ---------------------------------------------------------------------------
# Host units (degrees, m/s, m/s^2) -> ETSI units, with clamping and support
# for unavailable values (None)
# ---------------------------------------------------------------------------

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def lat_deg_to_etsi(lat_deg):
    if lat_deg is None:
        return LATITUDE_UNAVAILABLE
    return _clamp(int(round(lat_deg * DOT_ONE_MICRO)), -900000000, 900000000)


def lon_deg_to_etsi(lon_deg):
    if lon_deg is None:
        return LONGITUDE_UNAVAILABLE
    return _clamp(int(round(lon_deg * DOT_ONE_MICRO)), -1800000000, 1800000000)


def heading_deg_to_etsi(heading_deg):
    """Heading in degrees from north [0,360) -> Wgs84AngleValue (0.1 degrees)."""
    if heading_deg is None:
        return WGS84_ANGLE_VALUE_UNAVAILABLE
    return _clamp(int(round((heading_deg % 360.0) * DECI)), 0, 3599)


def speed_ms_to_etsi(speed_ms):
    """Speed in m/s -> SpeedValue (cm/s)."""
    if speed_ms is None:
        return SPEED_VALUE_UNAVAILABLE
    return _clamp(int(round(abs(speed_ms) * CENTI)), 0, 16382)


def acceleration_ms2_to_etsi(acc_ms2):
    """Longitudinal acceleration in m/s^2 -> AccelerationValue (0.1 m/s^2)."""
    if acc_ms2 is None:
        return LONG_ACCELERATION_VALUE_UNAVAILABLE
    return _clamp(int(round(acc_ms2 * DECI)), -160, 160)


# ---------------------------------------------------------------------------
# ETSI units -> host units (None is returned for unavailable values)
# ---------------------------------------------------------------------------

def etsi_to_lat_deg(v):
    return None if v==LATITUDE_UNAVAILABLE else v/DOT_ONE_MICRO


def etsi_to_lon_deg(v):
    return None if v==LONGITUDE_UNAVAILABLE else v/DOT_ONE_MICRO


def etsi_to_heading_deg(v):
    return None if v>=3600 else v/DECI


def etsi_to_speed_ms(v):
    return None if v==SPEED_VALUE_UNAVAILABLE else v/CENTI


def etsi_to_acceleration_ms2(v):
    return None if v==LONG_ACCELERATION_VALUE_UNAVAILABLE else v/DECI
