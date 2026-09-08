#
# test_vru_basic_service.py - unit tests for the Python VRU Basic Service porting
#
# Run from the python_vru_service folder with:
#   python3 -m unittest test_vru_basic_service.py -v
#

import socket
import threading
import time
import unittest

import os
import sys

# Allow running this file from anywhere: add the package root (parent folder)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vru_basic_service import (VRUBasicService, VRUBasicServiceError, TriggCond,
                               CHECK_MODE_PERIODIC, CHECK_MODE_TRIGGERED)
from ldm import LDM
import vam_codec
import geo_utils

REF_LAT = 45.0
REF_LON = 7.6


class VamCollector:
    """TX callback helper collecting the encoded VAMs (thread-safe)."""

    def __init__(self):
        self._lock = threading.Lock()
        self.vams = []

    def __call__(self, encoded):
        with self._lock:
            self.vams.append(encoded)

    def count(self):
        with self._lock:
            return len(self.vams)

    def wait_for(self, n, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.count() >= n:
                return True
            time.sleep(0.02)
        return self.count() >= n

    def last_decoded(self):
        with self._lock:
            return vam_codec.decode_vam(self.vams[-1])


def make_service(**kwargs):
    kwargs.setdefault("station_id", 1000)
    kwargs.setdefault("station_type", vam_codec.STATION_TYPE_PEDESTRIAN)
    kwargs.setdefault("ref_lat", REF_LAT)
    kwargs.setdefault("ref_lon", REF_LON)
    srv = VRUBasicService(**kwargs)
    collector = VamCollector()
    srv.set_tx_callback(collector)
    return srv, collector


class TestVamGenerationAndEncoding(unittest.TestCase):
    def test_initial_vam_content(self):
        srv, collector = make_service(proximity_check=False)
        srv.set_ego_vru_kstatus(1000, vam_codec.STATION_TYPE_PEDESTRIAN,
                                lat=45.001, lon=7.601, speed=1.25, heading=93.7,
                                acceleration=0.4)
        err = srv.generate_and_encode_vam()
        self.assertEqual(err, VRUBasicServiceError.VAM_NO_ERROR)
        self.assertTrue(collector.wait_for(1))

        vam = collector.last_decoded()
        self.assertEqual(vam["header"]["messageId"], vam_codec.FIX_VAMID)
        self.assertEqual(vam["header"]["protocolVersion"], 3)
        self.assertEqual(vam["header"]["stationId"], 1000)
        basic = vam["vam"]["vamParameters"]["basicContainer"]
        self.assertEqual(basic["stationType"], vam_codec.STATION_TYPE_PEDESTRIAN)
        self.assertAlmostEqual(vam_codec.etsi_to_lat_deg(
            basic["referencePosition"]["latitude"]), 45.001, places=6)
        self.assertAlmostEqual(vam_codec.etsi_to_lon_deg(
            basic["referencePosition"]["longitude"]), 7.601, places=6)
        hf = vam["vam"]["vamParameters"]["vruHighFrequencyContainer"]
        self.assertAlmostEqual(vam_codec.etsi_to_speed_ms(hf["speed"]["speedValue"]),
                               1.25, places=2)
        self.assertAlmostEqual(vam_codec.etsi_to_heading_deg(hf["heading"]["value"]),
                               93.7, places=1)
        self.assertAlmostEqual(vam_codec.etsi_to_acceleration_ms2(
            hf["longitudinalAcceleration"]["longitudinalAccelerationValue"]),
            0.4, places=1)

    def test_acceleration_unavailable_when_not_provided(self):
        srv, collector = make_service(proximity_check=False)
        srv.set_ego_vru_kstatus(1000, 1, lat=45.001, lon=7.601, speed=1.0, heading=0.0)
        srv.generate_and_encode_vam()
        hf = collector.last_decoded()["vam"]["vamParameters"]["vruHighFrequencyContainer"]
        self.assertEqual(hf["longitudinalAcceleration"]["longitudinalAccelerationValue"],
                         vam_codec.LONG_ACCELERATION_VALUE_UNAVAILABLE)

    def test_xy_position_converted_to_latlon(self):
        # The ego position provided as x/y must be converted to lat/lon (geopy)
        srv, collector = make_service(proximity_check=False)
        srv.set_ego_vru_kstatus(1000, 1, x=100.0, y=200.0, speed=1.0, heading=0.0)
        srv.generate_and_encode_vam()
        basic = collector.last_decoded()["vam"]["vamParameters"]["basicContainer"]
        lat = vam_codec.etsi_to_lat_deg(basic["referencePosition"]["latitude"])
        lon = vam_codec.etsi_to_lon_deg(basic["referencePosition"]["longitude"])
        conv = geo_utils.GeoConverter(REF_LAT, REF_LON)
        exp_lat, exp_lon = conv.xy_to_latlon(100.0, 200.0)
        self.assertAlmostEqual(lat, exp_lat, places=5)
        self.assertAlmostEqual(lon, exp_lon, places=5)


class TestPeriodicTriggeringConditions(unittest.TestCase):
    """The service is started in periodic mode (100 ms check interval) and the
    ego kinematic status is changed to trigger each condition."""

    def setUp(self):
        self.srv, self.collector = make_service(check_mode=CHECK_MODE_PERIODIC,
                                                proximity_check=False)
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=1.0, heading=90.0)
        self.srv.start_vam_dissemination()
        # Wait for the initial VAM (DISSEMINATION_START)
        self.assertTrue(self.collector.wait_for(1))
        self.assertEqual(self.srv.m_trigg_cond, TriggCond.DISSEMINATION_START)

    def tearDown(self):
        self.srv.terminate_dissemination()

    def test_speed_change_triggers_vam(self):
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=2.0, heading=90.0)
        self.assertTrue(self.collector.wait_for(2))
        self.assertEqual(self.srv.m_trigg_cond, TriggCond.SPEED_CHANGE)
        self.assertGreaterEqual(self.srv.m_speed_sent, 1)

    def test_heading_change_triggers_vam(self):
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=1.0, heading=100.0)
        self.assertTrue(self.collector.wait_for(2))
        self.assertEqual(self.srv.m_trigg_cond, TriggCond.HEADING_CHANGE)
        self.assertGreaterEqual(self.srv.m_head_sent, 1)

    def test_position_change_triggers_vam(self):
        self.srv.set_ego_vru_kstatus(1000, 1, x=5.0, y=0.0, speed=1.0, heading=90.0)
        self.assertTrue(self.collector.wait_for(2))
        self.assertEqual(self.srv.m_trigg_cond, TriggCond.POSITION_CHANGE)
        self.assertGreaterEqual(self.srv.m_pos_sent, 1)

    def test_no_vam_when_nothing_changes(self):
        # Small variations below all the thresholds must not trigger any VAM
        self.srv.set_ego_vru_kstatus(1000, 1, x=1.0, y=0.0, speed=1.2, heading=91.0)
        time.sleep(0.5)
        self.assertEqual(self.collector.count(), 1)  # only the initial VAM

    def test_max_time_elapsed_triggers_vam(self):
        self.srv.set_t_gen_vam(400)  # shrink T_GenVam from 5 s to 400 ms for the test
        self.assertTrue(self.collector.wait_for(2, timeout=2.0))
        self.assertEqual(self.srv.m_trigg_cond, TriggCond.MAX_TIME_ELAPSED)
        self.assertGreaterEqual(self.srv.m_time_sent, 1)


class TestSafeDistanceProximityCheck(unittest.TestCase):
    """Proximity checking (SAFE DISTANCES) performed by reading the LDM content."""

    def tearDown(self):
        self.srv.terminate_dissemination()

    def _start(self, ldm):
        self.srv, self.collector = make_service(check_mode=CHECK_MODE_PERIODIC,
                                                proximity_check=True, ldm=ldm)
        # Ego VRU moving north at 1.5 m/s: longitudinal safe distance = 1.5*5 = 7.5 m
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=1.5, heading=0.0)
        self.srv.start_vam_dissemination()
        self.assertTrue(self.collector.wait_for(1))

    def test_nearby_vehicle_triggers_safe_distance_vam(self):
        ldm = LDM()
        # A vehicle ~1.5 m away from the ego VRU (within all the safe distances)
        ldm.insert(2000, vam_codec.STATION_TYPE_PASSENGER_CAR,
                   x=1.0, y=1.0, speed=10.0, heading=0.0)
        self._start(ldm)
        self.assertTrue(self.collector.wait_for(2))
        self.assertEqual(self.srv.m_trigg_cond, TriggCond.SAFE_DISTANCES)
        self.assertGreaterEqual(self.srv.m_safedist_sent, 1)
        # The nearest vehicle must have been identified in the min distance vector
        self.assertEqual(self.srv.m_min_dist[1].station_id, 2000)

    def test_far_vehicle_does_not_trigger(self):
        ldm = LDM()
        # A vehicle far away (outside the lateral safe distance)
        ldm.insert(2000, vam_codec.STATION_TYPE_PASSENGER_CAR,
                   x=200.0, y=200.0, speed=10.0, heading=0.0)
        self._start(ldm)
        time.sleep(0.5)
        self.assertEqual(self.collector.count(), 1)  # only the initial VAM
        self.assertEqual(self.srv.m_safedist_sent, 0)

    def test_proximity_check_disabled_flag(self):
        # With the proximity check flag disabled, a nearby vehicle must NOT
        # trigger any SAFE DISTANCES VAM
        ldm = LDM()
        ldm.insert(2000, vam_codec.STATION_TYPE_PASSENGER_CAR,
                   x=1.0, y=1.0, speed=10.0, heading=0.0)
        self.srv, self.collector = make_service(check_mode=CHECK_MODE_PERIODIC,
                                                proximity_check=False, ldm=ldm)
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=1.5, heading=0.0)
        self.srv.start_vam_dissemination()
        self.assertTrue(self.collector.wait_for(1))
        time.sleep(0.5)
        self.assertEqual(self.collector.count(), 1)
        self.assertEqual(self.srv.m_safedist_sent, 0)


class TestTriggeredMode(unittest.TestCase):
    """External triggering via localhost UDP packet containing the string "check"."""

    TRIGGER_PORT = 48222

    def setUp(self):
        self.srv, self.collector = make_service(check_mode=CHECK_MODE_TRIGGERED,
                                                proximity_check=False,
                                                trigger_port=self.TRIGGER_PORT)
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=1.0, heading=90.0)
        self.srv.start_vam_dissemination()
        self.assertTrue(self.collector.wait_for(1))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.sock.close()
        self.srv.terminate_dissemination()

    def _send_check(self):
        self.sock.sendto(b"check", ("127.0.0.1", self.TRIGGER_PORT))

    def test_no_check_without_trigger(self):
        # In triggered mode the conditions are NOT checked periodically: a speed
        # change alone must not produce any VAM
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=3.0, heading=90.0)
        time.sleep(0.5)
        self.assertEqual(self.collector.count(), 1)

    def test_udp_check_triggers_conditions(self):
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=3.0, heading=90.0)
        self._send_check()
        self.assertTrue(self.collector.wait_for(2))
        self.assertEqual(self.srv.m_trigg_cond, TriggCond.SPEED_CHANGE)

    def test_udp_other_payload_is_ignored(self):
        self.srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=3.0, heading=90.0)
        self.sock.sendto(b"nope", ("127.0.0.1", self.TRIGGER_PORT))
        time.sleep(0.3)
        self.assertEqual(self.collector.count(), 1)
        self._send_check()
        self.assertTrue(self.collector.wait_for(2))


class TestExternalTimeReference(unittest.TestCase):
    """The service can be stepped manually with an external (simulated) time
    reference, as in an OpenCDA/CARLA co-simulation: no internal threads, the
    triggering conditions follow the external clock."""

    def test_manual_stepping_with_simulated_clock(self):
        srv, collector = make_service(proximity_check=False)
        sim_time = {"s": 0.0}
        srv.set_timestamp_callback(lambda: sim_time["s"])

        srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=1.0, heading=0.0)
        srv.init_dissemination()   # initial VAM, no dissemination loop started
        self.assertEqual(collector.count(), 1)

        # 2 simulated seconds later, nothing changed: no VAM must be generated
        sim_time["s"] = 2.0
        self.assertFalse(srv.check_vam_conditions())
        self.assertEqual(collector.count(), 1)

        # 5 simulated seconds (= T_GenVam) after the initial VAM: even if no real
        # time has elapsed, the MAX_TIME_ELAPSED condition must trigger a VAM
        sim_time["s"] = 5.0
        self.assertTrue(srv.check_vam_conditions())
        self.assertEqual(srv.m_trigg_cond, TriggCond.MAX_TIME_ELAPSED)
        self.assertEqual(collector.count(), 2)

        # The generationDeltaTime must follow the simulated clock as well
        vam = collector.last_decoded()
        self.assertEqual(vam["vam"]["generationDeltaTime"], 5000 % 65536)

        # The kinematic conditions keep working when stepped manually
        sim_time["s"] = 5.1
        srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=2.5, heading=0.0)
        self.assertTrue(srv.check_vam_conditions())
        self.assertEqual(srv.m_trigg_cond, TriggCond.SPEED_CHANGE)


class TestVamReceptionAndLDMFeeding(unittest.TestCase):
    def test_receive_vam_updates_ldm_and_callback(self):
        # A first service generates a VAM...
        tx_srv, tx_collector = make_service(proximity_check=False)
        tx_srv.set_ego_vru_kstatus(1000, 1, lat=45.0005, lon=7.6005,
                                   speed=1.5, heading=90.0)
        tx_srv.generate_and_encode_vam()

        # ...a second service receives it, feeding its own LDM and RX callback
        rx_srv, _ = make_service(station_id=2000, proximity_check=True)
        received = []
        rx_srv.add_vam_rx_callback(lambda vam, addr: received.append((vam, addr)))
        decoded = rx_srv.receive_vam(tx_collector.vams[-1], from_address="10.0.0.1")

        self.assertIsNotNone(decoded)
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0][1], "10.0.0.1")
        obj = rx_srv.m_LDM.lookup(1000)
        self.assertIsNotNone(obj)
        self.assertAlmostEqual(obj.lat, 45.0005, places=6)
        self.assertAlmostEqual(obj.speed, 1.5, places=2)

    def test_receive_non_vam_is_discarded(self):
        rx_srv, _ = make_service(station_id=2000, proximity_check=True)
        self.assertIsNone(rx_srv.receive_vam(b"\x02\x05\x00\x00"))
        self.assertEqual(rx_srv.m_LDM.get_cardinality(), 0)


class TestUdpTransmission(unittest.TestCase):
    def test_vam_sent_over_udp(self):
        rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx_sock.bind(("127.0.0.1", 0))
        rx_sock.settimeout(2.0)
        port = rx_sock.getsockname()[1]

        srv = VRUBasicService(station_id=1000, proximity_check=False,
                              ref_lat=REF_LAT, ref_lon=REF_LON)
        srv.set_tx_udp("127.0.0.1", port)
        srv.set_ego_vru_kstatus(1000, 1, lat=45.001, lon=7.601, speed=1.0, heading=0.0)
        err = srv.generate_and_encode_vam()
        self.assertEqual(err, VRUBasicServiceError.VAM_NO_ERROR)

        data, _ = rx_sock.recvfrom(2048)
        vam = vam_codec.decode_vam(data)
        self.assertEqual(vam["header"]["stationId"], 1000)
        rx_sock.close()
        srv.terminate_dissemination()


if __name__ == "__main__":
    unittest.main(verbosity=2)
