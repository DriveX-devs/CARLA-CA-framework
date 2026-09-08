#!/usr/bin/env bash
# Convenience launcher for the CA (Cooperative Awareness) demo: activates the
# conda env, makes sure a CARLA server is running, and runs run_ca.py from
# the OpenCDA root. Extra args are forwarded to run_ca.py.
#
#   ./CA/run.sh                       # yolov8n, 40 s
#   ./CA/run.sh --model x --seconds 20
#   CARLA_RENDER=1 ./CA/run.sh        # with a CARLA window
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Per-machine configuration (CARLA / ns-3 / conda paths). env.sh is gitignored:
# copy env.example.sh to env.sh and edit it once per machine.
[ -f "$SCRIPT_DIR/env.sh" ] && . "$SCRIPT_DIR/env.sh"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "${CA_CONDA_ENV:-msvan3t_carla}"

CARLA_PATH=${CARLA_PATH:-$WORKSPACE_ROOT/CARLA_0.9.12}
CARLA_PORT=${CARLA_PORT:-2000}

cd "$SCRIPT_DIR/.."

if [ "${CARLA_RENDER:-0}" = "1" ]; then
    RENDER_FLAG=""            # windowed
else
    RENDER_FLAG="-RenderOffScreen"
fi
if ! ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} "; then
    echo "No CARLA server on port ${CARLA_PORT}; launching one..."
    setsid "$CARLA_PATH/CarlaUE4.sh" ${RENDER_FLAG} -quality-level=Epic \
        -carla-rpc-port=${CARLA_PORT} >/tmp/carla_${CARLA_PORT}.log 2>&1 &
    echo "  CARLA log: /tmp/carla_${CARLA_PORT}.log"
    for i in $(seq 1 60); do
        ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} " && break
        sleep 2
    done
    sleep 5
else
    echo "Reusing CARLA server already listening on port ${CARLA_PORT}."
fi

python CA/run_ca.py --port ${CARLA_PORT} "$@"
