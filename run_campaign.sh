#!/usr/bin/env bash
# Generic driver for CA experiment campaigns.
#
# A *campaign* is a set of runs, one per random seed; each run collects one
# *cell* per (policy x YOLOv8 capacity) combination, optionally repeated over a
# sweep of injected packet-loss levels. Every cell of a given seed replays the
# same traffic manifest, so the policies are compared on byte-identical ground
# truth. Nothing is analysed or plotted here: this script only collects raw
# data and validates its structure.
#
#   ./CA/run_campaign.sh                                  # 1 seed, 3 policies, n/m/x
#   ./CA/run_campaign.sh --runs 10 --seconds 80           # 10 seeds x 9 cells
#   ./CA/run_campaign.sh --policies "1" --models "n"      # smallest useful campaign
#   ./CA/run_campaign.sh --config CA/ca_convoy_t2c3_s2c3.yaml   # tighter CA thresholds
#   ./CA/run_campaign.sh --policies "3" --loss "0 10 30 50 90"  # packet-loss sweep
#   ./CA/run_campaign.sh --out CA/output/mycampaign --attach --policies "4"
#   ./CA/run_campaign.sh --dry-run                        # print the commands only
#
# Options (each has an environment-variable equivalent):
#
#   --out DIR           campaign root directory              OUT_DIR
#                       (default CA/output/campaign_<timestamp>)
#   --runs N            number of seeds                      NUM_RUNS         (1)
#   --base-seed N       first seed; run k uses BASE_SEED+k-1 BASE_SEED        (11)
#   --seconds N         duration of one cell                 RUN_DURATION_S   (80)
#   --policies "1 2 3"  app modes to collect, see below      APP_MODES        (1 2 3)
#   --models "n m x"    YOLOv8 capacities                    MODELS           (n m x)
#   --config PATH       scenario yaml passed to the app      CA_CONFIG        (the app default)
#   --loss "0 10 50"    injected packet-loss levels [%];     LOSS_PERCENTAGES (none)
#                       when given, cells are nested under loss_<pp>/ and the
#                       bridge is driven through CA/run_extended_pl.sh
#   --save-stride N     annotated camera frames every N      SAVE_STRIDE      (0)
#                       steps; 0 = none. Frames are ~97 % of a cell's size and
#                       no metric reads them.
#   --attach            do not create runs: extend the runs already present in
#                       --out with the requested policies, replaying their
#                       seeds and traffic manifests            ATTACH         (0)
#   --no-smoke          skip the 2 s sanity cell per policy    SMOKE          (1)
#   --min-free-gb N     abort when free disk drops below       MIN_FREE_GB    (60)
#   --max-attempts N    retries of a cell before giving up     CELL_MAX_ATTEMPTS (3)
#   --dry-run           print the per-cell commands and exit
#
# Policies (--policies takes app modes; the cell directory name is fixed by
# CA/validate_campaign_raw.py):
#
#   1  policy_0_onboard        on-board CA only, no V2X
#   2  policy_1_collaborative  detections uploaded to the edge over 5G
#   3  policy_2_vam            as 2, plus VRUs broadcasting VAMs
#   4  policy_3_vam_only       edge CA fed by VAMs and CAV ego states only
#
# App modes >= 2 need the ns-3 v2x-bridge; it is built and probed automatically.
# A campaign with app mode 1 alone needs no ns-3 at all.
#
# Completed cells are resumable: a cell holding _SUCCESS is re-validated, not
# re-run. Interrupted cells are moved under <run>/_failed/ before a retry.
#
# Per-machine paths (CARLA, ns-3, conda) come from CA/env.sh — see env.example.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPEN_CDA_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$OPEN_CDA_ROOT/.." && pwd)"

# Per-machine configuration (CARLA / ns-3 / conda paths). env.sh is gitignored:
# copy env.example.sh to env.sh and edit it once per machine.
[ -f "$SCRIPT_DIR/env.sh" ] && . "$SCRIPT_DIR/env.sh"

NS3_ROOT=${NS3_ROOT:-$WORKSPACE_ROOT/ns-3-dev}
NS3_BUILD_DIR=${NS3_BUILD_DIR:-$NS3_ROOT/build-campaign}
NS3_CMAKE_DIR=${NS3_CMAKE_DIR:-$NS3_ROOT/cmake-cache-campaign}
NS3_BRIDGE_DIR=${NS3_BRIDGE_DIR:-v2x-bridge}
CAMPAIGN_CARLA_PORT=${CAMPAIGN_CARLA_PORT:-2100}
CAMPAIGN_BRIDGE_PORT=${CAMPAIGN_BRIDGE_PORT:-5555}

OUT_DIR=${OUT_DIR:-}
NUM_RUNS=${NUM_RUNS:-1}
BASE_SEED=${BASE_SEED:-11}
RUN_DURATION_S=${RUN_DURATION_S:-80}
APP_MODES=${APP_MODES:-"1 2 3"}
MODELS=${MODELS:-"n m x"}
CA_CONFIG=${CA_CONFIG:-}
LOSS_PERCENTAGES=${LOSS_PERCENTAGES:-}
SAVE_STRIDE=${SAVE_STRIDE:-0}
ATTACH=${ATTACH:-0}
SMOKE=${SMOKE:-1}
MIN_FREE_GB=${MIN_FREE_GB:-60}
CELL_MAX_ATTEMPTS=${CELL_MAX_ATTEMPTS:-3}
DISK_CHECK_PERIOD_S=${DISK_CHECK_PERIOD_S:-60}
DRY_RUN=0

FIXED_DELTA_S=0.05
STEPS_PER_S=20
SMOKE_DURATION_S=2

usage() { sed -n '2,58p' "$0" >&2; }

while [ "$#" -gt 0 ]; do
    case "$1" in
        --out)          [ "$#" -ge 2 ] || { usage; exit 2; }; OUT_DIR=$2; shift 2 ;;
        --runs)         [ "$#" -ge 2 ] || { usage; exit 2; }; NUM_RUNS=$2; shift 2 ;;
        --base-seed)    [ "$#" -ge 2 ] || { usage; exit 2; }; BASE_SEED=$2; shift 2 ;;
        --seconds)      [ "$#" -ge 2 ] || { usage; exit 2; }; RUN_DURATION_S=$2; shift 2 ;;
        --policies)     [ "$#" -ge 2 ] || { usage; exit 2; }; APP_MODES=$2; shift 2 ;;
        --models)       [ "$#" -ge 2 ] || { usage; exit 2; }; MODELS=$2; shift 2 ;;
        --config)       [ "$#" -ge 2 ] || { usage; exit 2; }; CA_CONFIG=$2; shift 2 ;;
        --loss)         [ "$#" -ge 2 ] || { usage; exit 2; }; LOSS_PERCENTAGES=$2; shift 2 ;;
        --save-stride)  [ "$#" -ge 2 ] || { usage; exit 2; }; SAVE_STRIDE=$2; shift 2 ;;
        --min-free-gb)  [ "$#" -ge 2 ] || { usage; exit 2; }; MIN_FREE_GB=$2; shift 2 ;;
        --max-attempts) [ "$#" -ge 2 ] || { usage; exit 2; }; CELL_MAX_ATTEMPTS=$2; shift 2 ;;
        --attach)       ATTACH=1; shift ;;
        --no-smoke)     SMOKE=0; shift ;;
        --dry-run)      DRY_RUN=1; shift ;;
        -h|--help)      usage; exit 0 ;;
        *)              echo "unknown option: $1" >&2; usage; exit 2 ;;
    esac
done

EXPECTED_STEPS=$((RUN_DURATION_S * STEPS_PER_S))
SMOKE_STEPS=$((SMOKE_DURATION_S * STEPS_PER_S))

# ---- policy naming (must match CA/validate_campaign_raw.py) -----------------
policy_dir_for() {
    case "$1" in
        1) echo policy_0_onboard ;;
        2) echo policy_1_collaborative ;;
        3) echo policy_2_vam ;;
        4) echo policy_3_vam_only ;;
        *) echo "ERROR: unknown app mode '$1' (expected 1..4)" >&2; exit 2 ;;
    esac
}

POLICY_INDICES=""
NEEDS_NS3=0
for mode in $APP_MODES; do
    policy_dir_for "$mode" >/dev/null
    POLICY_INDICES="${POLICY_INDICES:+$POLICY_INDICES,}$((mode - 1))"
    [ "$mode" -ge 2 ] && NEEDS_NS3=1
done
[ -n "$POLICY_INDICES" ] || { echo "ERROR: --policies is empty" >&2; exit 2; }
for model in $MODELS; do
    case "$model" in
        n|m|x) ;;
        *) echo "ERROR: unknown model capacity '$model' (expected n, m or x)" >&2; exit 2 ;;
    esac
done
MODEL_LIST=$(echo $MODELS | tr ' ' ',')

# ---- output directory ------------------------------------------------------
if [ -z "$OUT_DIR" ]; then
    if [ "$ATTACH" = "1" ]; then
        echo "ERROR: --attach needs --out pointing at the campaign to extend" >&2
        exit 2
    fi
    OUT_DIR="$OPEN_CDA_ROOT/CA/output/campaign_$(date +%Y%m%d_%H%M%S)"
elif [[ "$OUT_DIR" != /* ]]; then
    OUT_DIR="$OPEN_CDA_ROOT/$OUT_DIR"
fi
MANIFEST_DIR="$OUT_DIR/manifests"

# ---- packet-loss sweep -----------------------------------------------------
# LOSS_ITER holds one token per level, or the single token "none" when no loss
# is injected; "none" selects run_extended.sh instead of run_extended_pl.sh.
if [ -n "$LOSS_PERCENTAGES" ]; then
    LOSS_ITER="$LOSS_PERCENTAGES"
    LAUNCHER=./CA/run_extended_pl.sh
else
    LOSS_ITER="none"
    LAUNCHER=./CA/run_extended.sh
fi

run_root_for() {   # run index, seed, loss token
    if [ "$3" = "none" ]; then
        echo "$OUT_DIR/run_$1_seed$2"
    else
        printf '%s/loss_%02d/run_%s_seed%s\n' "$OUT_DIR" "$3" "$1" "$2"
    fi
}

# ---- the list of runs to collect: "<run root>|<seed>|<loss token>" ---------
RUN_SPECS=()
if [ "$ATTACH" = "1" ]; then
    [ -d "$OUT_DIR" ] || { echo "ERROR: no campaign to extend at $OUT_DIR" >&2; exit 2; }
    shopt -s nullglob
    for candidate in "$OUT_DIR"/run_*_seed* "$OUT_DIR"/loss_*/run_*_seed*; do
        [ -d "$candidate" ] || continue
        seed=${candidate##*_seed}
        parent=$(basename "$(dirname "$candidate")")
        case "$parent" in
            loss_*) token=$((10#${parent#loss_})) ;;
            *)      token=none ;;
        esac
        RUN_SPECS+=("$candidate|$seed|$token")
    done
    shopt -u nullglob
    [ "${#RUN_SPECS[@]}" -gt 0 ] || \
        { echo "ERROR: no run_*_seed* directories under $OUT_DIR" >&2; exit 2; }
else
    for run_idx in $(seq 1 "$NUM_RUNS"); do
        seed=$((BASE_SEED + run_idx - 1))
        for token in $LOSS_ITER; do
            RUN_SPECS+=("$(run_root_for "$run_idx" "$seed" "$token")|$seed|$token")
        done
    done
fi

manifest_for() {   # run root, seed
    if [ -f "$1/traffic_manifest.json" ]; then
        echo "$1/traffic_manifest.json"          # layout of an existing campaign
    else
        echo "$MANIFEST_DIR/traffic_manifest_seed$2.json"
    fi
}

# ---- optional extra arguments common to every cell -------------------------
COMMON_ARGS=(--raw-only --defer-success-marker
             --no-warning-cooldown --apply-isinrange --fixed-t2c-threshold
             --save-stride "$SAVE_STRIDE")
[ -n "$CA_CONFIG" ] && COMMON_ARGS+=(--config "$CA_CONFIG")

# ---- dry run ---------------------------------------------------------------
if [ "$DRY_RUN" = "1" ]; then
    dry_cell() {   # loss env prefix, app mode, model, seconds, seed, run dir,
                   # manifest, extra args
        echo "$1ANALYZE=0 MANAGE_CARLA=1 CARLA_PORT=$CAMPAIGN_CARLA_PORT" \
             "NS3_ROOT=$NS3_ROOT NS3_BUILD_DIR=$NS3_BUILD_DIR NS3_CMAKE_DIR=$NS3_CMAKE_DIR" \
             "$LAUNCHER ${COMMON_ARGS[*]} --app-mode $2 --model $3" \
             "--seconds $4 --seed $5 --run-dir $6 --traffic-manifest $7${8:+ $8}"
    }
    SEEN_SEEDS=" "
    for spec in "${RUN_SPECS[@]}"; do
        IFS='|' read -r run_root seed token <<<"$spec"
        manifest=$(manifest_for "$run_root" "$seed")
        loss_env=""
        [ "$token" != "none" ] && loss_env="INJECTED_LOSS_PERC=$token BRIDGE_RNG_RUN=$seed "
        if [ "$ATTACH" != "1" ] && [[ "$SEEN_SEEDS" != *" $seed "* ]]; then
            SEEN_SEEDS="$SEEN_SEEDS$seed "
            dry_cell "" 1 n "$SMOKE_DURATION_S" "$seed" \
                "$OUT_DIR/_manifest_smoke/seed$seed/mode_1" "$manifest" \
                "--create-traffic-manifest"
        fi
        if [ "$SMOKE" = "1" ]; then
            for mode in $APP_MODES; do
                dry_cell "$loss_env" "$mode" n "$SMOKE_DURATION_S" "$seed" \
                    "$run_root/_smoke/mode_$mode" "$manifest" ""
            done
        fi
        for mode in $APP_MODES; do
            policy_dir=$(policy_dir_for "$mode")
            for model in $MODELS; do
                dry_cell "$loss_env" "$mode" "$model" "$RUN_DURATION_S" "$seed" \
                    "$run_root/$policy_dir/$model" "$manifest" ""
            done
        done
    done
    exit 0
fi

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "${CA_CONDA_ENV:-msvan3t_carla}"
cd "$OPEN_CDA_ROOT"
unset MAXUES || true

mkdir -p "$OUT_DIR" "$MANIFEST_DIR"
exec > >(tee -a "$OUT_DIR/batch.log") 2>&1

echo "Campaign directory : $OUT_DIR"
echo "Runs               : ${#RUN_SPECS[@]} cell groups"\
     "($([ "$ATTACH" = 1 ] && echo "attached to the existing runs" || echo "seeds $BASE_SEED..$((BASE_SEED + NUM_RUNS - 1))"))"
echo "Duration           : ${RUN_DURATION_S}s (${EXPECTED_STEPS} steps at ${FIXED_DELTA_S}s)"
echo "Policies           : $APP_MODES   models: $MODELS   save-stride: $SAVE_STRIDE"
[ -n "$CA_CONFIG" ] && echo "Config             : $CA_CONFIG"
[ "$LOSS_ITER" != "none" ] && echo "Injected loss [%]  : $LOSS_PERCENTAGES (bridge --RngRun = seed)"

# ---- disk watchdog ---------------------------------------------------------
# Aborts the campaign, and the CARLA / bridge / python processes it started, as
# soon as free space on the filesystem holding OUT_DIR falls below MIN_FREE_GB.
MAIN_PID=$$
WATCHDOG_PID=""
DISK_ABORT_FILE="$OUT_DIR/DISK_ABORT"
MIN_FREE_KB=$((MIN_FREE_GB * 1024 * 1024))
rm -f "$DISK_ABORT_FILE"

free_kb() { df -Pk "$1" | awk 'NR==2 {print $4}'; }
free_gb() { echo $(( $(free_kb "$1") / 1024 / 1024 )); }

kill_campaign_processes() {
    pkill -f "run_ca_extended\.py.*${OUT_DIR}" 2>/dev/null || true
    pkill -f "run_extended.*\.sh.*${OUT_DIR}" 2>/dev/null || true
    pkill -f "v2x-bridge-default.*${OUT_DIR}" 2>/dev/null || true
    pkill -f "carla-rpc-port=${CAMPAIGN_CARLA_PORT}" 2>/dev/null || true
}

start_watchdog() {
    (
        while true; do
            sleep "$DISK_CHECK_PERIOD_S"
            avail=$(free_kb "$OUT_DIR" 2>/dev/null || echo 0)
            if [ "${avail:-0}" -lt "$MIN_FREE_KB" ]; then
                {
                    echo "ERROR: free space on the filesystem holding $OUT_DIR"
                    echo "ERROR: dropped to $((avail / 1024 / 1024)) GiB, below the ${MIN_FREE_GB} GiB floor."
                    echo "ERROR: aborting the campaign at $(date -Is)."
                    df -h "$OUT_DIR"
                } > "$DISK_ABORT_FILE" 2>&1
                cat "$DISK_ABORT_FILE" >&2
                kill -TERM "$MAIN_PID" 2>/dev/null || true
                sleep 10
                kill_campaign_processes
                sleep 5
                kill -KILL "$MAIN_PID" 2>/dev/null || true
                exit 1
            fi
        done
    ) &
    WATCHDOG_PID=$!
    echo "Disk watchdog      : abort below ${MIN_FREE_GB} GiB free, checked every" \
         "${DISK_CHECK_PERIOD_S}s (currently $(free_gb "$OUT_DIR") GiB free)"
}

on_exit() {
    status=$?
    if [ -n "$WATCHDOG_PID" ] && kill -0 "$WATCHDOG_PID" 2>/dev/null; then
        kill "$WATCHDOG_PID" 2>/dev/null || true
    fi
    if [ -f "$DISK_ABORT_FILE" ]; then
        kill_campaign_processes
        echo "" >&2
        echo "==== CAMPAIGN ABORTED: LOW DISK SPACE ====" >&2
        cat "$DISK_ABORT_FILE" >&2
        exit 1
    fi
    exit "$status"
}
trap on_exit EXIT
trap 'exit 143' TERM INT
start_watchdog

# ---- ns-3 v2x-bridge -------------------------------------------------------
BRIDGE_BIN=""
build_bridge() {
    echo "Building the workspace v2x-bridge..."
    if [ ! -f "$NS3_CMAKE_DIR/CMakeCache.txt" ] || \
            ! grep -Fqx "CMAKE_HOME_DIRECTORY:INTERNAL=$NS3_ROOT" \
                "$NS3_CMAKE_DIR/CMakeCache.txt" || \
            ! grep -Fqx "NS3_OUTPUT_DIRECTORY:STRING=$NS3_BUILD_DIR" \
                "$NS3_CMAKE_DIR/CMakeCache.txt"; then
        cmake -S "$NS3_ROOT" -B "$NS3_CMAKE_DIR" -G Ninja \
            -DCMAKE_BUILD_TYPE=default \
            -DNS3_EXAMPLES=OFF -DNS3_TESTS=OFF \
            -DNS3_OUTPUT_DIRECTORY="$NS3_BUILD_DIR"
    fi
    cmake --build "$NS3_CMAKE_DIR" \
        --target "scratch_${NS3_BRIDGE_DIR}_v2x-bridge" -j "${NS3_BUILD_JOBS:-4}"
    BRIDGE_BIN=$(find "$NS3_BUILD_DIR/scratch/$NS3_BRIDGE_DIR" -maxdepth 1 \
        -type f -name 'ns3.*-v2x-bridge-default' -print | sort | head -1)
    if [ -z "$BRIDGE_BIN" ] || \
            [ "$NS3_ROOT/scratch/$NS3_BRIDGE_DIR/v2x-bridge.cc" -nt "$BRIDGE_BIN" ]; then
        echo "ERROR: workspace v2x-bridge is missing or stale after build" >&2
        exit 1
    fi
}

# Start a throw-away bridge and check that a packet carrying an application
# payload comes back with the expected status and identical bytes.
probe_bridge() {   # injected loss %, expected status ("delivered" / "lost")
    local probe_dir probe_pid
    local probe_port=5556
    if ss -uln 2>/dev/null | grep -q ":${probe_port} "; then
        echo "ERROR: bridge probe UDP port ${probe_port} is already in use" >&2
        return 1
    fi
    probe_dir=$(mktemp -d)
    LD_LIBRARY_PATH="$NS3_BUILD_DIR/lib" setsid "$BRIDGE_BIN" \
        --maxUes=2 --listenPort="$probe_port" \
        --injected-loss-perc="$1" --RngRun=1 \
        --csvSend="$probe_dir/send.csv" --csvRecv="$probe_dir/recv.csv" \
        >"$probe_dir/bridge.log" 2>&1 </dev/null &
    probe_pid=$!
    for _ in $(seq 1 30); do
        grep -q "listening for JSON" "$probe_dir/bridge.log" && break
        kill -0 "$probe_pid" 2>/dev/null || break
        sleep 1
    done
    if ! grep -q "listening for JSON" "$probe_dir/bridge.log"; then
        tail -20 "$probe_dir/bridge.log" >&2
        kill "$probe_pid" 2>/dev/null || true
        wait "$probe_pid" 2>/dev/null || true
        return 1
    fi
    PROBE_PORT_VALUE=$probe_port PROBE_EXPECT_STATUS=$2 python - <<'PY'
import base64
import json
import os
import socket

port = int(os.environ["PROBE_PORT_VALUE"])
expect = os.environ["PROBE_EXPECT_STATUS"]
payload = b"campaign-payload-probe"
entities = [
    {"timestamp": 0.0, "origin_ID": "Cav1",
     "origin_vehicle_type": "cav",
     "Position": {"x_m": -42, "y_m": 25, "z_m": 1.5},
     "Velocity": 0, "Heading": 0},
    {"timestamp": 0.0, "origin_ID": "BS",
     "origin_vehicle_type": "base_station",
     "Position": {"x_m": -42, "y_m": 25, "z_m": 0},
     "Velocity": 0, "Heading": 0},
]
message = {
    "msg_type": "control", "timestamp": 0.0, "entities": entities,
    "packet": {"sender": "Cav1", "receiver": "BS",
               "size_bytes": 12 + len(payload), "packet_id": 987654,
               "type": "detection", "request_reply": True,
               "payload": base64.b64encode(payload).decode("ascii")},
}
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 0))
sock.settimeout(10)
sock.sendto(json.dumps(message).encode(), ("127.0.0.1", port))
reply = json.loads(sock.recvfrom(65535)[0].decode())
if reply.get("status") != expect:
    raise SystemExit("bridge probe: expected status %r, got %r" % (expect, reply))
if expect == "delivered":
    if "payload" not in reply:
        raise SystemExit("bridge payload probe failed: %r" % reply)
    if base64.b64decode(reply["payload"]) != payload:
        raise SystemExit("bridge payload probe returned different bytes")
sock.sendto(b'{"msg_type":"shutdown"}', ("127.0.0.1", port))
print("v2x-bridge probe passed (expected status %s)" % expect)
PY
    wait "$probe_pid"
    rm -r -- "$probe_dir"
}

if [ "$NEEDS_NS3" = "1" ]; then
    build_bridge
    probe_bridge 0 delivered
    [ "$LOSS_ITER" != "none" ] && probe_bridge 100 lost
else
    echo "ns-3                : not needed (app mode 1 only)"
fi

# ---- cell execution --------------------------------------------------------
# CARLA 0.9.12 segfaults now and then, after which the client blocks on 60 s
# per-actor timeouts. A crashed cell is retried rather than aborting the run.
retry_cleanup() {
    kill_campaign_processes
    local i
    for i in $(seq 1 60); do
        if ! ss -ltn 2>/dev/null | grep -q ":${CAMPAIGN_CARLA_PORT} " && \
           ! ss -uln 2>/dev/null | grep -q ":${CAMPAIGN_BRIDGE_PORT} "; then
            return 0
        fi
        sleep 2
    done
    echo "WARNING: ports ${CAMPAIGN_CARLA_PORT}/${CAMPAIGN_BRIDGE_PORT} still busy" >&2
    return 0
}

preserve_failed_cell() {   # run dir, root to make the path relative to, suffix
    if [ -d "$1" ] && [ -n "$(find "$1" -mindepth 1 -print -quit)" ]; then
        local failed_rel failed_dir
        failed_rel=${1#"$2"/}
        failed_dir="$2/_failed/${failed_rel}$3_$(date +%Y%m%d_%H%M%S)"
        mkdir -p "$(dirname "$failed_dir")"
        echo "Preserving incomplete cell at: $failed_dir"
        mv -- "$1" "$failed_dir"
    fi
}

# run_cell <app mode> <model> <duration> <steps> <run dir> <create manifest>
#          <loss token> <seed> <manifest> <failed root>
run_cell() {
    local app_mode=$1 model=$2 duration=$3 expected_steps=$4 run_dir=$5
    local create_manifest=$6 token=$7 seed=$8 manifest=$9 failed_root=${10}

    if [ -f "$run_dir/_SUCCESS" ]; then
        echo "Validating and skipping completed cell: $run_dir"
        python CA/validate_campaign_raw.py run "$run_dir" \
            --app-mode "$app_mode" --steps "$expected_steps"
        return 0
    fi
    preserve_failed_cell "$run_dir" "$failed_root" ""
    mkdir -p "$run_dir"

    local attempt=1
    while : ; do
        local cell_args=("${COMMON_ARGS[@]}" --traffic-manifest "$manifest")
        if [ "$create_manifest" = "1" ]; then
            rm -f "$manifest"        # drop a half-written one before a retry
            cell_args+=(--create-traffic-manifest)
        fi
        local loss_env=()
        if [ "$token" != "none" ]; then
            loss_env=(INJECTED_LOSS_PERC="$token" BRIDGE_RNG_RUN="$seed")
        fi
        echo "Starting app-mode=$app_mode model=$model duration=${duration}s" \
             "seed=$seed loss=${token} attempt=$attempt/$CELL_MAX_ATTEMPTS" \
             "($(free_gb "$OUT_DIR") GiB free) at $(date -Is)"
        if env "${loss_env[@]}" \
                BRIDGE_PORT="$CAMPAIGN_BRIDGE_PORT" \
                ANALYZE=0 MANAGE_CARLA=1 CARLA_PORT="$CAMPAIGN_CARLA_PORT" \
                NS3_ROOT="$NS3_ROOT" NS3_BUILD_DIR="$NS3_BUILD_DIR" \
                NS3_CMAKE_DIR="$NS3_CMAKE_DIR" NS3_BRIDGE_DIR="$NS3_BRIDGE_DIR" \
                "$LAUNCHER" \
                --app-mode "$app_mode" --model "$model" \
                --seconds "$duration" --seed "$seed" \
                --run-dir "$run_dir" "${cell_args[@]}" && \
           python CA/validate_campaign_raw.py run "$run_dir" \
                --app-mode "$app_mode" --steps "$expected_steps"; then
            return 0
        fi
        if [ -f "$DISK_ABORT_FILE" ]; then
            echo "ERROR: cell aborted by the disk watchdog: $run_dir" >&2
            return 1
        fi
        if [ "$attempt" -ge "$CELL_MAX_ATTEMPTS" ]; then
            echo "ERROR: cell failed $CELL_MAX_ATTEMPTS times, giving up: $run_dir" >&2
            return 1
        fi
        echo "WARNING: attempt $attempt failed (CARLA crash or validation error)," \
             "retrying: $run_dir" >&2
        retry_cleanup
        preserve_failed_cell "$run_dir" "$failed_root" "_attempt${attempt}"
        mkdir -p "$run_dir"
        attempt=$((attempt + 1))
        sleep 30
    done
}

# The traffic layout of a seed is generated once and replayed by every cell of
# that seed, whatever the policy or the loss level.
ensure_manifest() {   # seed, manifest path
    [ -f "$2" ] && return 0
    if [ "$ATTACH" = "1" ]; then
        echo "ERROR: --attach needs the traffic manifest of the existing run: $2" >&2
        exit 1
    fi
    echo "---- Creating the shared traffic manifest for seed $1 ----"
    run_cell 1 n "$SMOKE_DURATION_S" "$SMOKE_STEPS" \
        "$OUT_DIR/_manifest_smoke/seed$1/mode_1" 1 none "$1" "$2" "$OUT_DIR"
    if [ ! -f "$2" ]; then
        echo "ERROR: the traffic manifest was not created: $2" >&2
        exit 1
    fi
}

write_campaign_json() {   # run root, seed, loss token
    [ -f "$1/campaign.json" ] && return 0
    CAMPAIGN_DIR_VALUE="$1" SEED_VALUE="$2" LOSS_VALUE="$3" \
    RUN_DURATION_VALUE="$RUN_DURATION_S" DELTA_VALUE="$FIXED_DELTA_S" \
    MODELS_VALUE="$MODELS" APP_MODES_VALUE="$APP_MODES" \
    CONFIG_VALUE="$CA_CONFIG" SAVE_STRIDE_VALUE="$SAVE_STRIDE" \
    NS3_ROOT_VALUE="$NS3_ROOT" NS3_BUILD_DIR_VALUE="$NS3_BUILD_DIR" \
    NS3_CMAKE_DIR_VALUE="$NS3_CMAKE_DIR" CARLA_PORT_VALUE="$CAMPAIGN_CARLA_PORT" \
    python - <<'PY'
import json
import os
from datetime import datetime

names = {1: "onboard", 2: "collaborative", 3: "collaborative_vam",
         4: "vam_only"}
modes = [int(m) for m in os.environ["APP_MODES_VALUE"].split()]
loss = os.environ["LOSS_VALUE"]
value = {
    "schema_version": 2,
    "created_at": datetime.now().isoformat(),
    "duration_seconds": int(os.environ["RUN_DURATION_VALUE"]),
    "fixed_delta_seconds": float(os.environ["DELTA_VALUE"]),
    "seed": int(os.environ["SEED_VALUE"]),
    "models": os.environ["MODELS_VALUE"].split(),
    "policies": {str(mode - 1): names[mode] for mode in modes},
    "config": os.environ["CONFIG_VALUE"] or "CA/ca_convoy.yaml",
    "save_stride": int(os.environ["SAVE_STRIDE_VALUE"]),
    "injected_loss_perc": None if loss == "none" else int(loss),
    "ns3_root": os.environ["NS3_ROOT_VALUE"],
    "ns3_build_dir": os.environ["NS3_BUILD_DIR_VALUE"],
    "ns3_cmake_dir": os.environ["NS3_CMAKE_DIR_VALUE"],
    "carla_port": int(os.environ["CARLA_PORT_VALUE"]),
    "managed_carla_per_run": True,
    "analysis_enabled": False,
    "warning_cooldown": 0.0,
    "apply_isinrange": True,
    "fixed_t2c_threshold": True,
}
path = os.path.join(os.environ["CAMPAIGN_DIR_VALUE"], "campaign.json")
with open(path, "w") as handle:
    json.dump(value, handle, indent=2, sort_keys=True)
    handle.write("\n")
PY
}

# ---- main loop -------------------------------------------------------------
for spec in "${RUN_SPECS[@]}"; do
    IFS='|' read -r RUN_ROOT SEED TOKEN <<<"$spec"
    MANIFEST=$(manifest_for "$RUN_ROOT" "$SEED")
    echo "==== seed=$SEED loss=$TOKEN -> $RUN_ROOT ===="
    mkdir -p "$RUN_ROOT"
    write_campaign_json "$RUN_ROOT" "$SEED" "$TOKEN"
    ensure_manifest "$SEED" "$MANIFEST"

    if [ "$SMOKE" = "1" ]; then
        for mode in $APP_MODES; do
            run_cell "$mode" n "$SMOKE_DURATION_S" "$SMOKE_STEPS" \
                "$RUN_ROOT/_smoke/mode_$mode" 0 "$TOKEN" "$SEED" \
                "$MANIFEST" "$RUN_ROOT"
        done
    fi

    for mode in $APP_MODES; do
        policy_dir=$(policy_dir_for "$mode")
        for model in $MODELS; do
            run_cell "$mode" "$model" "$RUN_DURATION_S" "$EXPECTED_STEPS" \
                "$RUN_ROOT/$policy_dir/$model" 0 "$TOKEN" "$SEED" \
                "$MANIFEST" "$RUN_ROOT"
        done
    done

    python CA/validate_campaign_raw.py campaign "$RUN_ROOT" \
        --steps "$EXPECTED_STEPS" --policies "$POLICY_INDICES" \
        --models "$MODEL_LIST"
    echo "Raw-data run complete: $RUN_ROOT"
done

echo "Campaign complete: ${#RUN_SPECS[@]} run(s) under $OUT_DIR"
