#!/usr/bin/env bash
# Convenience launcher for the plain CA demo (local collision avoidance only,
# no simulated network): run_extended.sh in application mode 1. Extra args
# are forwarded to run_ca_extended.py.
#
#   ./run.sh                                   # SiMO on LiDAR, 40 s
#   ./run.sh --seconds 20 --simo-cooperation ideal
#   ./run.sh --perception yolo --model x       # YOLOv8-x + camera/LiDAR fusion
#   CARLA_RENDER=1 ./run.sh                    # with a CARLA window
ANALYZE=${ANALYZE:-0} exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_extended.sh" \
    --app-mode 1 "$@"
