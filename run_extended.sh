#!/usr/bin/env bash
# Convenience launcher for the EXTENDED CA application (run_ca_extended.py):
# activates the conda env, makes sure a CARLA server is running, starts a
# fresh ns-3 v2x-bridge when the selected mode needs it (--app-mode 2/3/4),
# runs the application, then runs the post-run metric analysis
# (CA/analyze_ca_metrics.py) on the new run directory. run.sh is left
# untouched: use it for the plain run_ca.py demo.
#
#   ./CA/run_extended.sh                          # mode 1 (local CA only)
#   ./CA/run_extended.sh --app-mode 2 --seconds 20
#   ./CA/run_extended.sh --app-mode 3             # edge CA + VRU VAMs
#   ./CA/run_extended.sh --app-mode 4             # edge CA from VAMs only
#   CARLA_RENDER=1 ./CA/run_extended.sh ...       # with a CARLA window
#
# Extra args are forwarded to run_ca_extended.py. Environment overrides:
#   CARLA_PORT (2000)   CARLA RPC port
#   MANAGE_CARLA (0)    1 = require a free port, start a private server, and
#                       stop it after the run
#   NS3_ROOT            ns-3 tree with the built v2x-bridge
#   NS3_BUILD_DIR       ns-3 output directory
#   NS3_CMAKE_DIR       optional separate CMake cache/build-tree directory
#   BRIDGE_PORT (5555)  bridge control UDP port
#   MAXUES              UE pool size (default: 8 in mode 2, 32 in modes 3/4)
#   GNB_X/GNB_Y         gNB position (default -42/25, the scenario center)
#   ANALYZE (1)         0 = skip the post-run metric analysis
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
NS3_ROOT=${NS3_ROOT:-$WORKSPACE_ROOT/ns-3-dev}
NS3_BUILD_DIR=${NS3_BUILD_DIR:-$NS3_ROOT/build}
NS3_CMAKE_DIR=${NS3_CMAKE_DIR:-}
# Name of the scratch sub-directory holding the ns-3 v2x-bridge sources
# (github.com/DriveX-devs/ns-3-v2x-bridge). It also fixes the CMake target
# name, so keep the clone directory and this variable in sync.
NS3_BRIDGE_DIR=${NS3_BRIDGE_DIR:-v2x-bridge}
BRIDGE_PORT=${BRIDGE_PORT:-5555}
GNB_X=${GNB_X:--42}
GNB_Y=${GNB_Y:-25}
ANALYZE=${ANALYZE:-1}
MANAGE_CARLA=${MANAGE_CARLA:-0}

cd "$SCRIPT_DIR/.."

# ---- app mode (peeked from the forwarded args; default 1) -------------------
APP_MODE=1
RUN_OUTPUT_DIR=""
args=("$@")
for i in "${!args[@]}"; do
    if [ "${args[$i]}" = "--app-mode" ]; then
        APP_MODE="${args[$((i + 1))]:-1}"
    elif [[ "${args[$i]}" == --app-mode=* ]]; then
        APP_MODE="${args[$i]#--app-mode=}"
    elif [ "${args[$i]}" = "--run-dir" ]; then
        RUN_OUTPUT_DIR="${args[$((i + 1))]:-}"
    elif [[ "${args[$i]}" == --run-dir=* ]]; then
        RUN_OUTPUT_DIR="${args[$i]#--run-dir=}"
    fi
done
if [ -n "$RUN_OUTPUT_DIR" ]; then
    if [[ "$RUN_OUTPUT_DIR" != /* ]]; then
        RUN_OUTPUT_DIR="$PWD/$RUN_OUTPUT_DIR"
    fi
    mkdir -p "$RUN_OUTPUT_DIR"
fi

# ---- CARLA ------------------------------------------------------------------
CARLA_PID=""
cleanup_carla() {
    if [ -n "$CARLA_PID" ] && kill -0 "$CARLA_PID" 2>/dev/null; then
        echo "Stopping the campaign CARLA server (pid $CARLA_PID)..."
        kill -TERM -- "-$CARLA_PID" 2>/dev/null || true
        for _ in $(seq 1 30); do
            kill -0 "$CARLA_PID" 2>/dev/null || break
            sleep 1
        done
        if kill -0 "$CARLA_PID" 2>/dev/null; then
            kill -KILL -- "-$CARLA_PID" 2>/dev/null || true
        fi
        wait "$CARLA_PID" 2>/dev/null || true
    fi
    CARLA_PID=""
}

if [ "${CARLA_RENDER:-0}" = "1" ]; then
    RENDER_FLAG=""            # windowed
else
    RENDER_FLAG="-RenderOffScreen"
fi
if ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} "; then
    if [ "$MANAGE_CARLA" = "1" ]; then
        echo "ERROR: managed CARLA port ${CARLA_PORT} is already in use." >&2
        exit 1
    fi
    echo "Reusing CARLA server already listening on port ${CARLA_PORT}."
else
    echo "No CARLA server on port ${CARLA_PORT}; launching one..."
    CARLA_LOG=${RUN_OUTPUT_DIR:+$RUN_OUTPUT_DIR/carla.log}
    CARLA_LOG=${CARLA_LOG:-/tmp/carla_${CARLA_PORT}.log}
    setsid "$CARLA_PATH/CarlaUE4.sh" ${RENDER_FLAG} -quality-level=Epic \
        -carla-rpc-port=${CARLA_PORT} >"$CARLA_LOG" 2>&1 &
    CARLA_PID=$!
    echo "  CARLA log: $CARLA_LOG"
    for i in $(seq 1 60); do
        ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} " && break
        if ! kill -0 "$CARLA_PID" 2>/dev/null; then
            echo "ERROR: CARLA exited before opening port ${CARLA_PORT}." >&2
            tail -30 "$CARLA_LOG" >&2
            exit 1
        fi
        sleep 2
    done
    if ! ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} "; then
        echo "ERROR: CARLA did not open port ${CARLA_PORT}." >&2
        tail -30 "$CARLA_LOG" >&2
        cleanup_carla
        exit 1
    fi
    sleep 5
fi

# ---- ns-3 v2x-bridge (modes 2/3 only; one fresh instance per run) -----------
BRIDGE_PID=""
cleanup_bridge() {
    if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo "Stopping the v2x-bridge (pid $BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
    fi
    BRIDGE_PID=""
}
cleanup_all() {
    cleanup_bridge
    cleanup_carla
}
trap cleanup_all EXIT
if [ "$APP_MODE" -ge 2 ]; then
    # the UE mapping and clock epoch are per run: a fresh bridge is required
    if ss -uln 2>/dev/null | grep -q ":${BRIDGE_PORT} "; then
        echo "ERROR: UDP port ${BRIDGE_PORT} is already in use." >&2
        echo "A v2x-bridge must be started FRESH for every run (its UE" >&2
        echo "mapping and clock epoch are locked per run): stop the" >&2
        echo "existing one first, e.g.:" >&2
        echo "  echo '{\"msg_type\":\"shutdown\"}' | nc -u -w1 127.0.0.1 ${BRIDGE_PORT}" >&2
        exit 1
    fi
    BRIDGE_BIN=$(ls "$NS3_BUILD_DIR"/scratch/"$NS3_BRIDGE_DIR"/ns3.*-v2x-bridge-default \
                 2>/dev/null | head -1)
    BRIDGE_SOURCE="$NS3_ROOT/scratch/$NS3_BRIDGE_DIR/v2x-bridge.cc"
    if [ -z "$BRIDGE_BIN" ] || [ "$BRIDGE_SOURCE" -nt "$BRIDGE_BIN" ]; then
        echo "v2x-bridge missing or stale; building it..."
        if [ -n "$NS3_CMAKE_DIR" ] && \
                [ -f "$NS3_CMAKE_DIR/CMakeCache.txt" ]; then
            cmake --build "$NS3_CMAKE_DIR" \
                --target "scratch_${NS3_BRIDGE_DIR}_v2x-bridge" \
                -j "${NS3_BUILD_JOBS:-4}"
        else
            (cd "$NS3_ROOT" && ./ns3 build "scratch_${NS3_BRIDGE_DIR}_v2x-bridge")
        fi
        BRIDGE_BIN=$(ls "$NS3_BUILD_DIR"/scratch/"$NS3_BRIDGE_DIR"/ns3.*-v2x-bridge-default \
                     | head -1)
    fi
    if [ -z "$BRIDGE_BIN" ] || [ "$BRIDGE_SOURCE" -nt "$BRIDGE_BIN" ]; then
        echo "ERROR: no up-to-date default v2x-bridge binary in $NS3_ROOT" >&2
        exit 1
    fi
    MAXUES=${MAXUES:-$([ "$APP_MODE" -ge 3 ] && echo 32 || echo 8)}
    TS=$(date +%Y%m%d_%H%M%S)
    BRIDGE_OUTPUT_DIR=${RUN_OUTPUT_DIR:-CA/output}
    mkdir -p "$BRIDGE_OUTPUT_DIR"
    BRIDGE_LOG="$BRIDGE_OUTPUT_DIR/bridge.log"
    echo "Starting the v2x-bridge: maxUes=${MAXUES}, gNB (${GNB_X}, ${GNB_Y})," \
         "log ${BRIDGE_LOG}"
    LD_LIBRARY_PATH="$NS3_BUILD_DIR/lib" setsid "$BRIDGE_BIN" \
        --maxUes="$MAXUES" --gnbX="$GNB_X" --gnbY="$GNB_Y" \
        --listenPort="$BRIDGE_PORT" \
        --csvSend="$BRIDGE_OUTPUT_DIR/bridge_send.csv" \
        --csvRecv="$BRIDGE_OUTPUT_DIR/bridge_recv.csv" \
        >"$BRIDGE_LOG" 2>&1 </dev/null &
    BRIDGE_PID=$!
    for i in $(seq 1 30); do
        grep -q "listening for JSON" "$BRIDGE_LOG" 2>/dev/null && break
        if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
            echo "ERROR: the v2x-bridge died on startup:" >&2
            tail -5 "$BRIDGE_LOG" >&2
            exit 1
        fi
        sleep 2
    done
    if ! grep -q "listening for JSON" "$BRIDGE_LOG" 2>/dev/null; then
        echo "ERROR: v2x-bridge did not become ready:" >&2
        tail -20 "$BRIDGE_LOG" >&2
        exit 1
    fi
    echo "v2x-bridge ready."
fi

# ---- the application --------------------------------------------------------
python CA/run_ca_extended.py --port "${CARLA_PORT}" \
    --bridge-port "${BRIDGE_PORT}" "$@"

# Campaign runs defer their completion marker until the bridge process has
# exited cleanly and its CSV streams have been closed.  A failed application
# never reaches this block, so incomplete cells remain non-resumable.
if [ -n "$RUN_OUTPUT_DIR" ] && \
        [ -f "$RUN_OUTPUT_DIR/_APPLICATION_SUCCESS" ]; then
    cleanup_bridge
    cleanup_carla
    mv -- "$RUN_OUTPUT_DIR/_APPLICATION_SUCCESS" "$RUN_OUTPUT_DIR/_SUCCESS"
    touch -- "$RUN_OUTPUT_DIR/_SUCCESS"
fi

# ---- post-run metric analysis ----------------------------------------------
if [ "$ANALYZE" = "1" ]; then
    RUN_DIR=$(ls -d CA/output/run_* 2>/dev/null | sort | tail -1)
    if [ -n "$RUN_DIR" ] && [ -f "$RUN_DIR/gt.csv" ]; then
        echo ""
        echo "=== Post-run metric analysis (${RUN_DIR}) ==="
        python CA/analyze_ca_metrics.py "$RUN_DIR" || true
    fi
fi
