#
# example_vru.py - LDM + VRU Basic Service on board of a VRU (e.g. a pedestrian
# with a smartphone)
#
# The VRU instantiates both an LDM and a VRU Basic Service for VAM transmission.
# The LDM is not populated except for the position of the ego VRU itself (VRUs
# are not expected to also receive VAMs). The VRU Basic Service transmits VAMs
# according to the ETSI TS 103 300-3 triggering conditions.
#
# Run from the python_vru_service folder with: python3 example_vru.py
#

import time

import os
import sys

# Allow running this file from anywhere: add the package root (parent folder)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ldm import LDM
from vru_basic_service import VRUBasicService, CHECK_MODE_PERIODIC
import vam_codec

EGO_VRU_ID = 1010
REF_LAT, REF_LON = 45.0, 7.6


def main():
    # LDM of the ego VRU: it will only contain the ego VRU position
    ldm = LDM()

    # VRU Basic Service (periodic checking of the triggering conditions, 100 ms)
    srv = VRUBasicService(station_id=EGO_VRU_ID,
                          station_type=vam_codec.STATION_TYPE_PEDESTRIAN,
                          check_mode=CHECK_MODE_PERIODIC,
                          ldm=ldm,
                          ref_lat=REF_LAT, ref_lon=REF_LON)
    srv.set_tx_callback(
        lambda encoded: print("[VAM TX] %d bytes, trigger=%s"
                              % (len(encoded), srv.m_trigg_cond.name)))

    def set_ego_status(x, y, speed, heading):
        # The kinematic status of the ego VRU is pushed to the VRU Basic Service
        # (x/y is converted internally to lat/lon through geopy)...
        srv.set_ego_vru_kstatus(EGO_VRU_ID, vam_codec.STATION_TYPE_PEDESTRIAN,
                                x=x, y=y, speed=speed, heading=heading)
        # ...and the LDM is kept updated with the ego VRU position only
        ldm.insert(EGO_VRU_ID, vam_codec.STATION_TYPE_PEDESTRIAN,
                   x=x, y=y, speed=speed, heading=heading)

    # Initial kinematic status, then start of the VAM dissemination
    x, y, speed, heading = 0.0, 0.0, 1.3, 0.0
    set_ego_status(x, y, speed, heading)
    srv.start_vam_dissemination()

    # Simulate 4 seconds of pedestrian motion (10 Hz updates): the pedestrian
    # first walks straight, then accelerates, then turns
    print("Simulating pedestrian motion for 4 seconds...")
    for step in range(40):
        time.sleep(0.1)
        y += speed * 0.1
        if step == 12:
            speed = 2.1        # speed change > 0.5 m/s -> VAM expected
        if step == 25:
            heading = 30.0     # heading change > 4 deg -> VAM expected
        set_ego_status(x, y, speed, heading)

    sent = srv.terminate_dissemination()
    print("\nDone. VAMs sent: %d (heading: %d, position: %d, speed: %d, "
          "safe distances: %d, max time: %d)"
          % (sent, srv.m_head_sent, srv.m_pos_sent, srv.m_speed_sent,
             srv.m_safedist_sent, srv.m_time_sent))
    ego = ldm.lookup(EGO_VRU_ID)
    print("LDM content (ego VRU only): %s -> position=(%.2f, %.2f), lat/lon=(%.7f, %.7f)"
          % (sorted(ldm.get_all_ids()), ego.x, ego.y,
             srv.m_ego_lat, srv.m_ego_lon))


if __name__ == "__main__":
    main()
