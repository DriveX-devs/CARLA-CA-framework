#
# example_edge_server.py - LDM on an edge server (e.g. an MEC host collecting
# V2X data from an RSU)
#
# The edge server instantiates an LDM populated with both:
#  - vehicle detections received from sensorized vehicles (provided directly as
#    position, speed, heading, ...);
#  - VAMs received from VRUs (UPER-encoded, decoded through asn1tools).
# Every successful update causes a callback to be called, which for now just
# prints the received information.
#
# The object matching logic of the LDM is enabled (enable_object_matching()):
# when multiple vehicles detect the same physical object, it counts as a single
# object in the LDM.
#
# Run from the python_vru_service folder with: python3 example_edge_server.py
#

import os
import sys

# Allow running this file from anywhere: add the package root (parent folder)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ldm import LDM
import vam_codec
import geo_utils

REF_LAT, REF_LON = 45.0, 7.6


def ldm_update_callback(station_id, station_type, position, speed, heading, acceleration):
    """Called by the LDM on every successful insertion/update: for now it just
    prints the received information."""
    print("  [Edge LDM callback] station %d (type %d): position=%s(%.7g, %.7g) "
          "speed=%s heading=%s acceleration=%s"
          % (station_id, station_type, position.kind, position.c1, position.c2,
             "%.2f m/s" % speed if speed is not None else "N/A",
             "%.1f deg" % heading if heading is not None else "N/A",
             "%.2f m/s^2" % acceleration if acceleration is not None else "N/A"))


def make_vam(station_id, lat, lon, speed_ms, heading_deg, acceleration_ms2=None):
    """Emulate a VAM received from a real VRU (UPER-encoded with asn1tools)."""
    return vam_codec.encode_vam({
        "header": {"protocolVersion": 3, "messageId": vam_codec.FIX_VAMID,
                   "stationId": station_id},
        "vam": {
            "generationDeltaTime": 4321,
            "vamParameters": {
                "basicContainer": {
                    "stationType": vam_codec.STATION_TYPE_PEDESTRIAN,
                    "referencePosition": {
                        "latitude": vam_codec.lat_deg_to_etsi(lat),
                        "longitude": vam_codec.lon_deg_to_etsi(lon),
                        "positionConfidenceEllipse": {
                            "semiMajorAxisLength": vam_codec.SEMI_AXIS_LENGTH_UNAVAILABLE,
                            "semiMinorAxisLength": vam_codec.SEMI_AXIS_LENGTH_UNAVAILABLE,
                            "semiMajorAxisOrientation": vam_codec.SEMI_AXIS_ORIENTATION_UNAVAILABLE,
                        },
                        "altitude": {"altitudeValue": vam_codec.ALTITUDE_VALUE_UNAVAILABLE,
                                     "altitudeConfidence": "unavailable"},
                    },
                },
                "vruHighFrequencyContainer": {
                    "heading": {"value": vam_codec.heading_deg_to_etsi(heading_deg),
                                "confidence": vam_codec.WGS84_ANGLE_CONFIDENCE_UNAVAILABLE},
                    "speed": {"speedValue": vam_codec.speed_ms_to_etsi(speed_ms),
                              "speedConfidence": vam_codec.SPEED_CONFIDENCE_UNAVAILABLE},
                    "longitudinalAcceleration": {
                        "longitudinalAccelerationValue":
                            vam_codec.acceleration_ms2_to_etsi(acceleration_ms2),
                        "longitudinalAccelerationConfidence":
                            vam_codec.ACCELERATION_CONFIDENCE_UNAVAILABLE,
                    },
                },
            },
        },
    })


def main():
    ldm = LDM()
    ldm.enable_object_matching(match_distance_m=2.0)
    ldm.register_callback(ldm_update_callback)

    # Converter used to also store the x/y position of the VRUs received via VAM
    converter = geo_utils.GeoConverter(REF_LAT, REF_LON)

    # ------------------------------------------------------------------
    # 1) Detections received from sensorized vehicles (direct data)
    # ------------------------------------------------------------------
    print("Vehicle 100 reports two detected objects:")
    # Vehicle 100 detects a pedestrian and a car
    ldm.insert(9001, vam_codec.STATION_TYPE_PEDESTRIAN, x=10.0, y=5.0,
               speed=1.4, heading=350.0, detected=True, perceived_by=100)
    ldm.insert(9002, vam_codec.STATION_TYPE_PASSENGER_CAR, x=42.0, y=-7.0,
               speed=12.5, heading=95.0, acceleration=0.4,
               detected=True, perceived_by=100)

    print("Vehicle 101 reports the same pedestrian (position ~60 cm apart) "
          "under its own local ID, plus a new cyclist:")
    # Vehicle 101 detects the SAME pedestrian (within the 2 m matching distance):
    # thanks to the object matching it is merged into entry 9001 instead of
    # creating a new object
    ldm.insert(9101, vam_codec.STATION_TYPE_PEDESTRIAN, x=10.5, y=5.3,
               speed=1.5, heading=352.0, detected=True, perceived_by=101)
    # ...and a cyclist far from any known object (new entry)
    ldm.insert(9102, vam_codec.STATION_TYPE_CYCLIST, x=-20.0, y=30.0,
               speed=5.1, heading=180.0, detected=True, perceived_by=101)

    # ------------------------------------------------------------------
    # 2) VAMs received from real VRUs (decoded with asn1tools)
    # ------------------------------------------------------------------
    print("Two VAMs are received from connected VRUs:")
    lat1, lon1 = converter.xy_to_latlon(60.0, 15.0)
    err, sid = ldm.add_vru_from_vam(
        make_vam(2001, lat1, lon1, 1.2, 45.0, acceleration_ms2=0.1),
        geo_converter=converter)
    lat2, lon2 = converter.xy_to_latlon(-35.0, -12.0)
    err, sid = ldm.add_vru_from_vam(
        make_vam(2002, lat2, lon2, 0.9, 270.0), geo_converter=converter)

    # ------------------------------------------------------------------
    # Resulting LDM content
    # ------------------------------------------------------------------
    print("\nLDM content (%d objects):" % ldm.get_cardinality())
    for sid in sorted(ldm.get_all_ids()):
        obj = ldm.lookup(sid)
        origin = ("detected by vehicles %s" % sorted(obj.perceived_by)
                  if obj.detected else "connected VRU (VAM)")
        print("  - station %d (type %d), %s" % (sid, obj.station_type, origin))
    print("\nNote: the pedestrian detected by both vehicle 100 and vehicle 101 "
          "counts as the single object 9001.")

    # Objects within 30 m from the RSU (placed at the origin of the local frame,
    # i.e. at the reference lat/lon): all the entries have an x/y position, since
    # the VAM ones are converted through the GeoConverter
    nearby = ldm.range_select_xy(30.0, 0.0, 0.0)
    print("Objects within 30 m from the RSU:", sorted(o.station_id for o in nearby))


if __name__ == "__main__":
    main()
