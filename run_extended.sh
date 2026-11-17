#!/usr/bin/env bash
# Launcher of the CA application (run_ca_extended.py): activates the conda
# env, makes sure a CARLA server is running, starts a fresh ns-3
# drivex-bridge when the selected mode needs it (--app-mode 2/3/4), runs the
# application, then runs the post-run metric analysis (analyze_ca_metrics.py)
# on the new run directory.
#
#   ./run_extended.sh                             # mode 1 (local CA only), SiMO
#   ./run_extended.sh --app-mode 2 --seconds 20   # edge CA over 5G NR Uu
#   ./run_extended.sh --app-mode 3                # edge CA + VRU VAMs
#   ./run_extended.sh --app-mode 4                # edge CA from VAMs only
#   ACCESS=11p ./run_extended.sh --app-mode 3     # ... over IEEE 802.11p
#   ACCESS=nrv2x TOPOLOGY=hybrid ./run_extended.sh --app-mode 3
#   CARLA_GPUS="0 1" ./run_extended.sh --simo-sensors both --simo-devices cuda:2,cuda:3
#   CARLA_RENDER=1 ./run_extended.sh ...          # with a CARLA window
#
# Extra args are forwarded to run_ca_extended.py. Environment (see
# launch_lib.sh and env.example.sh for all of them):
#   ACCESS (uu)         uu | 11p | 11bd | nrv2x: radio of the simulated network
#   TOPOLOGY (rsu)      rsu | hybrid: how the edge is reached when ACCESS != uu
#   CARLA_PORT (2000)   CARLA RPC port
#   CARLA_GPU           GPU of the CARLA server
#   CARLA_GPUS          several GPUs: CARLA's multi-GPU mode (started here)
#   MANAGE_CARLA (0)    1 = require a free port, start a private server, and
#                       stop it after the run
#   NS3_ROOT            ns-3 tree with the drivex-bridge
#   BRIDGE_PORT (5555)  bridge control UDP port
#   MAXUES              station pool (default: 8 in mode 2, 32 in modes 3/4)
#   GNB_X/GNB_Y         gNB / RSU position (default -42/25, the scenario centre)
#   INJECTED_LOSS_PERC  percentage of delivered packets the bridge discards
#   BRIDGE_RNG_RUN      ns-3 RngRun of the bridge
#   ANALYZE (1)         0 = skip the post-run metric analysis
set -e

. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/launch_lib.sh"
ca_activate_env
ANALYZE=${ANALYZE:-1}
cd "$CA_ROOT"

APP_MODE=$(arg_value --app-mode 1 "$@")
RUN_OUTPUT_DIR=$(arg_value --run-dir "" "$@")
if [ -n "$RUN_OUTPUT_DIR" ]; then
    [[ "$RUN_OUTPUT_DIR" != /* ]] && RUN_OUTPUT_DIR="$PWD/$RUN_OUTPUT_DIR"
    mkdir -p "$RUN_OUTPUT_DIR"
fi

cleanup_all() {
    bridge_stop
    carla_stop
}
trap cleanup_all EXIT

carla_start "$RUN_OUTPUT_DIR" "$(ca_town "$@")"

NETWORK_ARGS=()
if [ "$APP_MODE" -ge 2 ]; then
    bridge_start "$APP_MODE" "${RUN_OUTPUT_DIR:-$CA_ROOT/output}"
    NETWORK_ARGS=(--bridge-port "$BRIDGE_PORT" --access "$ACCESS")
    [ "$ACCESS" != uu ] && NETWORK_ARGS+=(--topology "$TOPOLOGY")
fi

# ---- the application --------------------------------------------------------
python run_ca_extended.py --port "${CARLA_PORT}" "${CARLA_EXTRA_APP_ARGS[@]}" \
    "${NETWORK_ARGS[@]}" "$@"

# Campaign runs defer their completion marker until the bridge process has
# exited cleanly and its CSV streams have been closed.  A failed application
# never reaches this block, so incomplete cells remain non-resumable.
if [ -n "$RUN_OUTPUT_DIR" ] && \
        [ -f "$RUN_OUTPUT_DIR/_APPLICATION_SUCCESS" ]; then
    bridge_stop
    carla_stop
    mv -- "$RUN_OUTPUT_DIR/_APPLICATION_SUCCESS" "$RUN_OUTPUT_DIR/_SUCCESS"
    touch -- "$RUN_OUTPUT_DIR/_SUCCESS"
fi

# ---- post-run metric analysis ----------------------------------------------
if [ "$ANALYZE" = "1" ]; then
    RUN_DIR=${RUN_OUTPUT_DIR:-$(ls -d output/run_* 2>/dev/null | sort | tail -1)}
    if [ -n "$RUN_DIR" ] && [ -f "$RUN_DIR/gt.csv" ]; then
        echo ""
        echo "=== Post-run metric analysis (${RUN_DIR}) ==="
        python analyze_ca_metrics.py "$RUN_DIR" || true
    fi
fi
