#
# example.py - end-to-end demo of the Python LDM + VRU Basic Service porting
#
# A "transmitting" pedestrian (ego VRU) disseminates VAMs according to the ETSI
# triggering conditions; a "receiving" station decodes them and feeds its LDM,
# whose callback prints every successful insertion/update.
#
# Run from the python_vru_service folder with: python3 example.py
#

import time

import os
import sys

# Allow running this file from anywhere: add the package root (parent folder)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ldm import LDM
from vru_basic_service import VRUBasicService, CHECK_MODE_PERIODIC
import vam_codec

REF_LAT, REF_LON = 45.0, 7.6


def ldm_callback(station_id, station_type, position, speed, heading, acceleration):
    print("  [LDM callback] stationID=%d stationType=%d position=%s(%.7f, %.7f) "
          "speed=%.2f m/s heading=%.1f deg acceleration=%s"
          % (station_id, station_type, position.kind, position.c1, position.c2,
             speed, heading, "%.1f m/s^2" % acceleration if acceleration is not None else "N/A"))


def main():
    # Receiving side: a station with an LDM fed by the received VAMs
    # (the LDM is passed explicitly, since by default the service creates none)
    rx_service = VRUBasicService(station_id=2000, ldm=LDM(),
                                 ref_lat=REF_LAT, ref_lon=REF_LON)
    rx_service.m_LDM.register_callback(ldm_callback)

    # Transmitting side: the ego VRU (pedestrian), periodic checking every 100 ms.
    # The "radio channel" is simulated by handing the encoded VAM to the receiver.
    tx_service = VRUBasicService(station_id=1000,
                                 station_type=vam_codec.STATION_TYPE_PEDESTRIAN,
                                 check_mode=CHECK_MODE_PERIODIC,
                                 proximity_check=True, ldm=LDM(),
                                 ref_lat=REF_LAT, ref_lon=REF_LON)
    tx_service.set_tx_callback(
        lambda encoded: (print("[TX] VAM sent (%d bytes), trigger=%s\n     bytes: %s"
                               % (len(encoded), tx_service.m_trigg_cond.name,
                                  encoded.hex())),
                         rx_service.receive_vam(encoded, "127.0.0.1")))

    # A nearby vehicle known to the ego VRU (for the safe-distance proximity check)
    tx_service.m_LDM.insert(3000, vam_codec.STATION_TYPE_PASSENGER_CAR,
                            x=100.0, y=100.0, speed=8.0, heading=180.0)

    # Ego VRU initial kinematic status (x/y -> converted internally to lat/lon)
    x, y, speed, heading = 0.0, 0.0, 1.2, 0.0
    tx_service.set_ego_vru_kstatus(1000, vam_codec.STATION_TYPE_PEDESTRIAN,
                                   x=x, y=y, speed=speed, heading=heading,
                                   acceleration=0.1)
    tx_service.start_vam_dissemination()

    # Simulate 3 seconds of pedestrian motion (10 Hz position updates)
    print("Simulating pedestrian motion for 3 seconds...")
    for step in range(30):
        time.sleep(0.1)
        y += speed * 0.1
        if step == 10:
            speed = 2.0        # speed change > 0.5 m/s -> VAM
        if step == 20:
            heading = 45.0     # heading change > 4 deg -> VAM
        tx_service.set_ego_vru_kstatus(1000, vam_codec.STATION_TYPE_PEDESTRIAN,
                                       x=x, y=y, speed=speed, heading=heading)

    sent = tx_service.terminate_dissemination()
    print("Done. VAMs sent: %d (heading: %d, position: %d, speed: %d, "
          "safe distances: %d, max time: %d)"
          % (sent, tx_service.m_head_sent, tx_service.m_pos_sent,
             tx_service.m_speed_sent, tx_service.m_safedist_sent,
             tx_service.m_time_sent))
    print("Receiver LDM content:", rx_service.m_LDM.get_all_ids())


if __name__ == "__main__":
    main()
