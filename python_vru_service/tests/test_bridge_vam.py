#
# test_bridge_vam.py
#
# Cross-check of the VAMs DriveX's VRU Basic Service produces inside the
# drivex-bridge: the UPER bytes found in the GeoNetworking PDU the bridge
# returns are decoded here with asn1tools (vam_codec.py, an implementation
# that shares nothing with DriveX's ASN.1 code) and compared with the state
# the road user was given and with what the edge's DriveX stack decoded.
#
# It starts a bridge of its own and is skipped when none is built. The ns-3
# tree is taken from NS3_ROOT (default: <workspace>/ns-3.48-drivex).
#
import glob
import json
import math
import os
import socket
import subprocess
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import vam_codec  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
NS3_ROOT = os.environ.get('NS3_ROOT', os.path.join(os.path.dirname(REPO),
                                                    'ns-3.48-drivex'))
BUILD = os.environ.get('NS3_BUILD_DIR', os.path.join(NS3_ROOT, 'build'))
PORT = 5899
# basic (4) + common (8) + topologically-scoped broadcast (28) + BTP-B (4)
GN_BTP_HEADER_BYTES = 44


def bridge_binary():
    found = [f for f in glob.glob(os.path.join(
        BUILD, 'scratch', '*', 'ns3.*-drivex-bridge-*')) if not f.endswith('.o')]
    return sorted(found)[0] if found else None


@unittest.skipIf(bridge_binary() is None, 'no drivex-bridge binary built')
class BridgeVamTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import base64
        env = dict(os.environ)
        env['LD_LIBRARY_PATH'] = os.path.join(BUILD, 'lib')
        cls.proc = subprocess.Popen(
            [bridge_binary(), '--maxUes=4', '--listenPort=%d' % PORT, '--vbs=1',
             '--csvSend=/dev/null', '--csvRecv=/dev/null'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, text=True)
        deadline = time.time() + 120
        while time.time() < deadline:
            line = cls.proc.stdout.readline()
            if 'listening for JSON' in line:
                break
            if cls.proc.poll() is not None:
                raise RuntimeError('the bridge exited at start-up')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.02)
        addr = ('127.0.0.1', PORT)
        cls.tx, cls.rx = {}, {}
        cls.state = {}
        # a cyclist riding east at 6 m/s that turns south after 4 s
        for k in range(160):
            t = k * 0.05
            heading = 90.0 if t < 4.0 else 180.0
            x = 10.0 + 6.0 * min(t, 4.0)
            y = 30.0 + 6.0 * max(t - 4.0, 0.0)
            lat, lon = -y / 111319.49, x / 111319.49
            entity = {'timestamp': t, 'origin_ID': 'bike_1',
                      'origin_vehicle_type': 'cyclist',
                      'Position': {'x_m': x, 'y_m': y, 'z_m': 1.2},
                      'Velocity': 6.0,
                      'Heading': math.radians(heading - 90.0),
                      'heading_deg': heading, 'station_id': 4321,
                      'station_type': 2, 'Acceleration': 0.3,
                      'Geo': {'lat': lat, 'lon': lon, 'alt': 1.2}}
            sock.sendto(json.dumps({'msg_type': 'control',
                                    'timestamp': round(t, 3),
                                    'entities': [entity]}).encode(), addr)
            end = time.time() + 0.03
            while time.time() < end:
                try:
                    msg = json.loads(sock.recvfrom(65536)[0])
                except socket.timeout:
                    continue
                if msg.get('msg_type') == 'vam_tx':
                    cls.tx[msg['packet_id']] = msg
                elif msg.get('msg_type') == 'reply' and \
                        msg.get('type') == 'vam':
                    msg['_pdu'] = base64.b64decode(msg.get('payload', ''))
                    cls.rx[msg['packet_id']] = msg
        sock.sendto(b'{"msg_type":"shutdown"}', addr)
        cls.proc.wait(timeout=30)
        cls.delivered = [r for r in cls.rx.values()
                         if r['status'] == 'delivered']

    def test_vams_were_generated_and_delivered(self):
        self.assertEqual(self.proc.returncode, 0)
        self.assertGreaterEqual(len(self.tx), 8)
        self.assertGreaterEqual(len(self.delivered), 0.8 * len(self.tx))

    def test_position_rule_sets_the_period(self):
        # 4 m at 6 m/s, checked every 100 ms: one VAM every 0.7 s
        gaps = [e['since_previous_ms'] for e in self.tx.values()
                if e['trigger'] == 'position']
        self.assertTrue(gaps)
        for gap in gaps:
            self.assertAlmostEqual(gap, 700.0, delta=1.0)

    def test_turn_triggers_a_vam(self):
        self.assertIn('heading', [e['trigger'] for e in self.tx.values()])

    def test_uper_bytes_decode_to_what_was_sent(self):
        self.assertTrue(self.delivered)
        for reply in self.delivered:
            sent = self.tx[reply['packet_id']]
            pdu = reply['_pdu']
            self.assertEqual(len(pdu), sent['pdu_bytes'])
            vam = vam_codec.decode_vam(pdu[GN_BTP_HEADER_BYTES:])
            self.assertEqual(vam['header']['messageId'], vam_codec.FIX_VAMID)
            self.assertEqual(vam['header']['stationId'], 4321)
            params = vam['vam']['vamParameters']
            basic = params['basicContainer']
            hf = params['vruHighFrequencyContainer']
            self.assertEqual(basic['stationType'], 2)
            lat = vam_codec.etsi_to_lat_deg(
                basic['referencePosition']['latitude'])
            lon = vam_codec.etsi_to_lon_deg(
                basic['referencePosition']['longitude'])
            self.assertAlmostEqual(lat, sent['lat'], delta=1.5e-7)
            self.assertAlmostEqual(lon, sent['lon'], delta=1.5e-7)
            self.assertAlmostEqual(
                vam_codec.etsi_to_speed_ms(hf['speed']['speedValue']),
                sent['speed'], delta=0.011)
            self.assertAlmostEqual(
                vam_codec.etsi_to_heading_deg(hf['heading']['value']),
                sent['heading_deg'], delta=0.11)
            self.assertAlmostEqual(
                vam_codec.etsi_to_acceleration_ms2(
                    hf['longitudinalAcceleration']
                    ['longitudinalAccelerationValue']), 0.3, delta=0.051)
            # and DriveX's own decoder at the edge read the same
            edge = reply['vam']
            self.assertEqual(edge['station_id'], 4321)
            self.assertAlmostEqual(edge['lat'], lat, delta=1e-9)
            self.assertAlmostEqual(edge['lon'], lon, delta=1e-9)
            self.assertAlmostEqual(edge['heading_deg'], sent['heading_deg'],
                                   delta=0.11)


if __name__ == '__main__':
    unittest.main()
