#
# test_ldm.py - unit tests for the Python LDM porting
#
# Run from the python_vru_service folder with:
#   python3 -m unittest test_ldm.py -v
#

import unittest

import os
import sys

# Allow running this file from anywhere: add the package root (parent folder)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ldm import LDM, LDMError
import vam_codec
import geo_utils


def make_test_vam(station_id=1000, station_type=vam_codec.STATION_TYPE_PEDESTRIAN,
                  lat=45.6214980, lon=7.6043412, speed_ms=1.5, heading_deg=90.0,
                  acceleration_ms2=None):
    """Encode a minimal valid VAM with asn1tools, for testing purposes."""
    vam = {
        "header": {"protocolVersion": 3, "messageId": vam_codec.FIX_VAMID,
                   "stationId": station_id},
        "vam": {
            "generationDeltaTime": 12345,
            "vamParameters": {
                "basicContainer": {
                    "stationType": station_type,
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
    }
    return vam_codec.encode_vam(vam)


class TestLDMBasicOperations(unittest.TestCase):
    def setUp(self):
        self.ldm = LDM()

    def test_insert_and_lookup(self):
        ret = self.ldm.insert(1, vam_codec.STATION_TYPE_PEDESTRIAN,
                              x=10.0, y=20.0, speed=1.2, heading=45.0)
        self.assertEqual(ret, LDMError.LDM_OK)
        obj = self.ldm.lookup(1)
        self.assertIsNotNone(obj)
        self.assertEqual(obj.station_id, 1)
        self.assertEqual(obj.station_type, vam_codec.STATION_TYPE_PEDESTRIAN)
        self.assertEqual(obj.x, 10.0)
        self.assertEqual(obj.y, 20.0)
        self.assertEqual(obj.speed, 1.2)
        self.assertEqual(obj.heading, 45.0)
        self.assertIsNone(obj.acceleration)

    def test_insert_updates_existing_entry(self):
        self.ldm.insert(1, 1, x=0.0, y=0.0, speed=1.0, heading=0.0)
        ret = self.ldm.insert(1, 1, x=5.0, y=5.0, speed=2.0, heading=10.0,
                              acceleration=0.3)
        self.assertEqual(ret, LDMError.LDM_UPDATED)
        obj = self.ldm.lookup(1)
        self.assertEqual(obj.speed, 2.0)
        self.assertEqual(obj.acceleration, 0.3)

    def test_update_missing_entry(self):
        ret = self.ldm.update(99, 1, x=0.0, y=0.0, speed=1.0, heading=0.0)
        self.assertEqual(ret, LDMError.LDM_ITEM_NOT_FOUND)

    def test_insert_without_position_fails(self):
        ret = self.ldm.insert(1, 1, speed=1.0, heading=0.0)
        self.assertEqual(ret, LDMError.LDM_UNKNOWN_ERROR)
        self.assertIsNone(self.ldm.lookup(1))

    def test_remove(self):
        self.ldm.insert(1, 1, x=0.0, y=0.0, speed=1.0, heading=0.0)
        self.assertEqual(self.ldm.remove(1), LDMError.LDM_OK)
        self.assertIsNone(self.ldm.lookup(1))
        self.assertEqual(self.ldm.remove(1), LDMError.LDM_ITEM_NOT_FOUND)

    def test_cardinality_and_ids(self):
        self.ldm.insert(1, 1, x=0.0, y=0.0)
        self.ldm.insert(2, 5, x=1.0, y=1.0)
        self.assertEqual(self.ldm.get_cardinality(), 2)
        self.assertEqual(self.ldm.get_all_ids(), {1, 2})


class TestLDMRangeSelect(unittest.TestCase):
    def setUp(self):
        self.ldm = LDM()
        # Two close stations (lat/lon ~ 45N/7.6E) and one far away
        self.ldm.insert(1, 1, lat=45.0000, lon=7.6000, speed=1.0, heading=0.0)
        self.ldm.insert(2, 5, lat=45.0002, lon=7.6000, speed=10.0, heading=0.0)  # ~22 m north
        self.ldm.insert(3, 5, lat=45.1000, lon=7.6000, speed=10.0, heading=0.0)  # ~11 km north
        # A station with x/y only
        self.ldm.insert(4, 1, x=3.0, y=4.0, speed=1.0, heading=0.0)

    def test_range_select_latlon(self):
        sel = self.ldm.range_select(100.0, 45.0000, 7.6000)
        ids = {o.station_id for o in sel}
        self.assertEqual(ids, {1, 2})

    def test_range_select_latlon_all(self):
        sel = self.ldm.range_select(50000.0, 45.0000, 7.6000)
        ids = {o.station_id for o in sel}
        self.assertEqual(ids, {1, 2, 3})

    def test_range_select_xy(self):
        sel = self.ldm.range_select_xy(6.0, 0.0, 0.0)  # distance of station 4 is 5 m
        ids = {o.station_id for o in sel}
        self.assertEqual(ids, {4})

    def test_range_select_by_station(self):
        sel = self.ldm.range_select_by_station(100.0, 1)
        ids = {o.station_id for o in sel}
        self.assertEqual(ids, {1, 2})

    def test_range_select_by_missing_station(self):
        self.assertEqual(self.ldm.range_select_by_station(100.0, 42),
                         LDMError.LDM_ITEM_NOT_FOUND)


class TestLDMCallbacks(unittest.TestCase):
    def setUp(self):
        self.ldm = LDM()
        self.calls = []
        self.ldm.register_callback(
            lambda sid, stype, pos, speed, heading, acc:
            self.calls.append((sid, stype, pos, speed, heading, acc)))

    def test_callback_on_insert_and_update(self):
        self.ldm.insert(7, 1, x=1.0, y=2.0, speed=1.0, heading=90.0, acceleration=0.1)
        self.ldm.insert(7, 1, x=2.0, y=3.0, speed=1.5, heading=90.0)
        self.assertEqual(len(self.calls), 2)
        sid, stype, pos, speed, heading, acc = self.calls[0]
        self.assertEqual(sid, 7)
        self.assertEqual(pos.kind, "xy")
        self.assertEqual((pos.c1, pos.c2), (1.0, 2.0))
        self.assertEqual(acc, 0.1)
        self.assertIsNone(self.calls[1][5])  # acceleration not available on update

    def test_callback_position_fallback_latlon(self):
        # If x/y is not available, the callback must provide lat/lon
        self.ldm.insert(8, 1, lat=45.0, lon=7.6, speed=1.0, heading=0.0)
        pos = self.calls[-1][2]
        self.assertEqual(pos.kind, "latlon")
        self.assertEqual((pos.c1, pos.c2), (45.0, 7.6))

    def test_no_callback_on_failed_insert(self):
        self.ldm.insert(9, 1, speed=1.0, heading=0.0)  # no position -> failure
        self.assertEqual(len(self.calls), 0)


class TestLDMAddVRUFromVAM(unittest.TestCase):
    def setUp(self):
        self.ldm = LDM()

    def test_add_vru_from_vam(self):
        buf = make_test_vam(station_id=1000, lat=45.6214980, lon=7.6043412,
                            speed_ms=1.5, heading_deg=90.0)
        err, sid = self.ldm.add_vru_from_vam(buf)
        self.assertEqual(err, LDMError.LDM_OK)
        self.assertEqual(sid, 1000)
        obj = self.ldm.lookup(1000)
        self.assertIsNotNone(obj)
        self.assertEqual(obj.station_type, vam_codec.STATION_TYPE_PEDESTRIAN)
        self.assertAlmostEqual(obj.lat, 45.6214980, places=6)
        self.assertAlmostEqual(obj.lon, 7.6043412, places=6)
        self.assertAlmostEqual(obj.speed, 1.5, places=2)
        self.assertAlmostEqual(obj.heading, 90.0, places=1)
        self.assertIsNone(obj.acceleration)  # encoded as unavailable

    def test_add_vru_from_vam_with_acceleration(self):
        buf = make_test_vam(station_id=1001, acceleration_ms2=1.2)
        err, sid = self.ldm.add_vru_from_vam(buf)
        self.assertEqual(err, LDMError.LDM_OK)
        self.assertAlmostEqual(self.ldm.lookup(1001).acceleration, 1.2, places=1)

    def test_add_vru_from_vam_update(self):
        self.ldm.add_vru_from_vam(make_test_vam(station_id=1000, speed_ms=1.0))
        err, _ = self.ldm.add_vru_from_vam(make_test_vam(station_id=1000, speed_ms=2.0))
        self.assertEqual(err, LDMError.LDM_UPDATED)
        self.assertAlmostEqual(self.ldm.lookup(1000).speed, 2.0, places=2)

    def test_add_vru_from_vam_triggers_callback(self):
        calls = []
        self.ldm.register_callback(lambda *args: calls.append(args))
        self.ldm.add_vru_from_vam(make_test_vam(station_id=1000))
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], 1000)
        self.assertEqual(calls[0][2].kind, "latlon")

    def test_add_vru_from_vam_with_converter(self):
        conv = geo_utils.GeoConverter(45.6214000, 7.6043412)
        buf = make_test_vam(station_id=1000, lat=45.6214980, lon=7.6043412)
        err, _ = self.ldm.add_vru_from_vam(buf, geo_converter=conv)
        self.assertEqual(err, LDMError.LDM_OK)
        obj = self.ldm.lookup(1000)
        self.assertIsNotNone(obj.x)
        # ~10.9 m north of the reference
        self.assertAlmostEqual(obj.x, 0.0, delta=0.5)
        self.assertAlmostEqual(obj.y, 10.9, delta=0.5)

    def test_add_vru_from_garbage(self):
        err, sid = self.ldm.add_vru_from_vam(b"\x00\x01\x02\x03")
        self.assertEqual(err, LDMError.LDM_UNKNOWN_ERROR)
        self.assertIsNone(sid)
        self.assertEqual(self.ldm.get_cardinality(), 0)


class TestObjectMatching(unittest.TestCase):
    def setUp(self):
        self.ldm = LDM()
        self.ldm.enable_object_matching(match_distance_m=2.0)

    def test_same_object_from_two_vehicles_is_merged(self):
        self.ldm.insert(9001, 1, x=10.0, y=5.0, speed=1.4, heading=350.0,
                        detected=True, perceived_by=100)
        # Same physical object, ~0.6 m apart, reported by another vehicle
        ret = self.ldm.insert(9101, 1, x=10.5, y=5.3, speed=1.5, heading=352.0,
                              detected=True, perceived_by=101)
        self.assertEqual(ret, LDMError.LDM_UPDATED)
        self.assertEqual(self.ldm.get_cardinality(), 1)
        obj = self.ldm.lookup(9001)  # stored under the first stationID
        self.assertIsNotNone(obj)
        self.assertEqual(obj.perceived_by, {100, 101})
        self.assertEqual(obj.speed, 1.5)  # data updated by the latest detection
        self.assertIsNone(self.ldm.lookup(9101))

    def test_far_objects_are_not_merged(self):
        self.ldm.insert(9001, 1, x=10.0, y=5.0, detected=True, perceived_by=100)
        ret = self.ldm.insert(9101, 1, x=20.0, y=5.0, detected=True, perceived_by=101)
        self.assertEqual(ret, LDMError.LDM_OK)
        self.assertEqual(self.ldm.get_cardinality(), 2)

    def test_two_objects_from_same_vehicle_are_not_merged(self):
        # The same vehicle reporting two nearby objects under different local IDs
        # means they are distinct physical objects
        self.ldm.insert(9001, 1, x=10.0, y=5.0, detected=True, perceived_by=100)
        ret = self.ldm.insert(9002, 1, x=10.5, y=5.3, detected=True, perceived_by=100)
        self.assertEqual(ret, LDMError.LDM_OK)
        self.assertEqual(self.ldm.get_cardinality(), 2)

    def test_subsequent_updates_follow_the_mapping(self):
        self.ldm.insert(9001, 1, x=10.0, y=5.0, detected=True, perceived_by=100)
        self.ldm.insert(9101, 1, x=10.5, y=5.3, detected=True, perceived_by=101)
        # Vehicle 101 keeps tracking "its" object 9101, which moved away more than
        # the matching distance: the update must still go to the merged entry 9001
        ret = self.ldm.insert(9101, 1, x=15.0, y=9.0, speed=2.0,
                              detected=True, perceived_by=101)
        self.assertEqual(ret, LDMError.LDM_UPDATED)
        self.assertEqual(self.ldm.get_cardinality(), 1)
        self.assertEqual(self.ldm.lookup(9001).x, 15.0)

    def test_connected_objects_are_not_matched(self):
        # Matching only applies to detected objects: two connected stations
        # (e.g. known via VAM/CAM) close to each other must stay separate
        self.ldm.insert(2001, 1, x=10.0, y=5.0, speed=1.0, heading=0.0)
        ret = self.ldm.insert(2002, 1, x=10.5, y=5.3, speed=1.0, heading=0.0)
        self.assertEqual(ret, LDMError.LDM_OK)
        self.assertEqual(self.ldm.get_cardinality(), 2)

    def test_matching_disabled_keeps_objects_separate(self):
        self.ldm.disable_object_matching()
        self.ldm.insert(9001, 1, x=10.0, y=5.0, detected=True, perceived_by=100)
        self.ldm.insert(9101, 1, x=10.5, y=5.3, detected=True, perceived_by=101)
        self.assertEqual(self.ldm.get_cardinality(), 2)

    def test_matching_with_latlon_positions(self):
        self.ldm.insert(9001, 1, lat=45.00000, lon=7.60000, detected=True,
                        perceived_by=100)
        # ~1.1 m north: same object
        self.ldm.insert(9101, 1, lat=45.00001, lon=7.60000, detected=True,
                        perceived_by=101)
        self.assertEqual(self.ldm.get_cardinality(), 1)
        self.assertEqual(self.ldm.lookup(9001).perceived_by, {100, 101})

    def test_mapping_cleared_on_remove(self):
        self.ldm.insert(9001, 1, x=10.0, y=5.0, detected=True, perceived_by=100)
        self.ldm.remove(9001)
        # After the removal, a new detection from vehicle 100 with the same local
        # ID must create a fresh entry
        ret = self.ldm.insert(9001, 1, x=50.0, y=50.0, detected=True, perceived_by=100)
        self.assertEqual(ret, LDMError.LDM_OK)
        self.assertEqual(self.ldm.lookup(9001).x, 50.0)

    def test_callback_reports_canonical_id(self):
        calls = []
        self.ldm.register_callback(lambda sid, *args: calls.append(sid))
        self.ldm.insert(9001, 1, x=10.0, y=5.0, detected=True, perceived_by=100)
        self.ldm.insert(9101, 1, x=10.5, y=5.3, detected=True, perceived_by=101)
        self.assertEqual(calls, [9001, 9001])


class TestEdgeFusion(unittest.TestCase):
    def setUp(self):
        self.ldm = LDM()
        self.ldm.enable_object_matching(
            match_distance_m=2.0,
            connected_match_distance_m=4.5,
            fuse_states=True,
            fusion_ttl_s=0.5,
        )

    def test_latest_per_source_is_fused(self):
        self.ldm.insert(9001, 1, x=0.0, y=0.0, speed=0.0,
                        detected=True, perceived_by=100, timestamp=10.0)
        self.ldm.insert(9101, 1, x=2.0, y=0.0, speed=0.0,
                        detected=True, perceived_by=101, timestamp=10.0)
        self.assertAlmostEqual(self.ldm.lookup(9001).x, 1.0)
        self.assertEqual(self.ldm.lookup(9001).perceived_by, {100, 101})

    def test_old_source_expires_after_fixed_ttl(self):
        self.ldm.insert(9001, 1, x=0.0, y=0.0, speed=0.0,
                        detected=True, perceived_by=100, timestamp=10.0)
        self.ldm.insert(9101, 1, x=2.0, y=0.0, speed=0.0,
                        detected=True, perceived_by=101, timestamp=10.6)
        self.assertAlmostEqual(self.ldm.lookup(9001).x, 2.0)

    def test_out_of_order_packet_does_not_replace_newer_source_state(self):
        self.ldm.insert(9001, 1, x=5.0, y=0.0, speed=0.0,
                        detected=True, perceived_by=100, timestamp=10.4)
        self.ldm.insert(9001, 1, x=1.0, y=0.0, speed=0.0,
                        detected=True, perceived_by=100, timestamp=10.1)
        self.assertAlmostEqual(self.ldm.lookup(9001).x, 5.0)
        self.assertAlmostEqual(self.ldm.lookup(9001).timestamp, 10.4)

    def test_vam_promotes_sensor_track_to_connected_identity(self):
        self.ldm.insert(9001, 1, x=1.0, y=2.0, speed=1.0,
                        detected=True, perceived_by=100, timestamp=10.0)
        ret = self.ldm.insert(1001, 1, x=1.2, y=2.0, speed=1.0,
                              timestamp=10.1, match_connected=True)
        self.assertEqual(ret, LDMError.LDM_UPDATED)
        self.assertIsNone(self.ldm.lookup(9001))
        self.assertIsNotNone(self.ldm.lookup(1001))
        self.assertFalse(self.ldm.lookup(1001).detected)
        self.assertEqual(self.ldm.canonical_id(100, 9001), 1001)
        self.assertEqual(self.ldm.consume_rekeyed_ids(), [(9001, 1001)])

    def test_cav_connected_record_does_not_absorb_sensor_track(self):
        self.ldm.insert(9001, 5, x=1.0, y=2.0,
                        detected=True, perceived_by=100, timestamp=10.0)
        self.ldm.insert(101, 5, x=1.2, y=2.0, timestamp=10.1)
        self.assertEqual(self.ldm.get_cardinality(), 2)
        self.assertIsNotNone(self.ldm.lookup(9001))
        self.assertIsNotNone(self.ldm.lookup(101))


class TestGeoConverter(unittest.TestCase):
    def test_roundtrip(self):
        conv = geo_utils.GeoConverter(45.0, 7.6)
        lat, lon = conv.xy_to_latlon(100.0, 200.0)
        x, y = conv.latlon_to_xy(lat, lon)
        self.assertAlmostEqual(x, 100.0, delta=0.1)
        self.assertAlmostEqual(y, 200.0, delta=0.1)

    def test_known_distance(self):
        conv = geo_converter = geo_utils.GeoConverter(45.0, 7.6)
        lat, lon = conv.xy_to_latlon(0.0, 111.0)
        self.assertAlmostEqual(
            geo_utils.geodesic_distance_m(45.0, 7.6, lat, lon), 111.0, delta=0.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
