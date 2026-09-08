#
# example_sensorized_vehicle.py - LDM usage on board of a sensorized vehicle
#
# The vehicle instantiates an LDM only. The positions and status of the objects
# detected by the on-board sensors come from an external perception application:
# here they are emulated with fixed, reasonable values. A callback is registered
# and called on every successful insertion/update.
#
# Run from the python_vru_service folder with: python3 example_sensorized_vehicle.py
#

import time

import os
import sys

# Allow running this file from anywhere: add the package root (parent folder)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ldm import LDM
import vam_codec

EGO_VEHICLE_ID = 100


def ldm_update_callback(station_id, station_type, position, speed, heading, acceleration):
    """Called by the LDM on every successful insertion/update."""
    print("  [LDM callback] object %d (type %d): position=%s(%.2f, %.2f) "
          "speed=%.2f m/s heading=%.1f deg acceleration=%s"
          % (station_id, station_type, position.kind, position.c1, position.c2,
             speed, heading,
             "%.2f m/s^2" % acceleration if acceleration is not None else "N/A"))


def main():
    ldm = LDM()
    ldm.register_callback(ldm_update_callback)

    # Detections coming from the external perception application: local object ID,
    # station type, initial x/y position [m] in the vehicle reference frame,
    # speed [m/s], heading [deg], acceleration [m/s^2] (None if not available)
    detections = [
        (9001, vam_codec.STATION_TYPE_PEDESTRIAN, 12.0, 3.5, 1.3, 355.0, None),
        (9002, vam_codec.STATION_TYPE_CYCLIST, -8.0, 15.0, 4.2, 90.0, 0.2),
        (9003, vam_codec.STATION_TYPE_PASSENGER_CAR, 35.0, -2.0, 13.9, 182.0, -0.5),
    ]

    print("Feeding the LDM with the detections of the perception application "
          "(5 frames, 10 Hz)...")
    for frame in range(5):
        print("Frame %d:" % frame)
        for obj_id, obj_type, x, y, speed, heading, acc in detections:
            # Emulate the object motion between two frames (dead reckoning at 10 Hz
            # along the y axis, just to make the updates visible)
            y_now = y + speed * 0.1 * frame
            ldm.insert(obj_id, obj_type, x=x, y=y_now, speed=speed, heading=heading,
                       acceleration=acc, detected=True, perceived_by=EGO_VEHICLE_ID)
        time.sleep(0.1)

    # Lookup by station ID
    obj = ldm.lookup(9001)
    print("\nLookup of object 9001: type=%d position=(%.2f, %.2f) perceived by %s"
          % (obj.station_type, obj.x, obj.y, sorted(obj.perceived_by)))

    # Lookup by circular area: objects within 20 m from the ego vehicle (origin)
    nearby = ldm.range_select_xy(20.0, 0.0, 0.0)
    print("Objects within 20 m from the ego vehicle:",
          sorted(o.station_id for o in nearby))

    # Removal (e.g. the perception application lost track of the object)
    ldm.remove(9003)
    print("After removing object 9003, the LDM contains:", sorted(ldm.get_all_ids()))


if __name__ == "__main__":
    main()
