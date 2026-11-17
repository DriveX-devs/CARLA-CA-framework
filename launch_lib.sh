# Shared by the launchers (run.sh, run_extended.sh, run_campaign.sh): paths,
# the CARLA server(s) and the ns-3 drivex-bridge. Sourced, never executed.
#
# Everything is configured through environment variables, with defaults that
# follow the workspace layout of the README; env.sh (gitignored, see
# env.example.sh) is the place to set them per machine.

CA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$CA_ROOT/.." && pwd)"

# Per-machine configuration (CARLA / ns-3 / SiMO / conda paths).
[ -f "$CA_ROOT/env.sh" ] && . "$CA_ROOT/env.sh"

CA_CONDA_ENV=${CA_CONDA_ENV:-ca_carla916}
CARLA_PATH=${CARLA_PATH:-$WORKSPACE_ROOT/CARLA_0.9.16}
CARLA_PORT=${CARLA_PORT:-2000}
CARLA_QUALITY=${CARLA_QUALITY:-Epic}
# One GPU for the CARLA server (CARLA_GPU=2), or several for CARLA's multi-GPU
# mode (CARLA_GPUS="0 1 2": one rendering server per GPU behind a primary
# server that renders nothing). Unset = whatever CARLA picks.
CARLA_GPU=${CARLA_GPU:-}
CARLA_GPUS=${CARLA_GPUS:-}
MANAGE_CARLA=${MANAGE_CARLA:-0}

NS3_ROOT=${NS3_ROOT:-$WORKSPACE_ROOT/ns-3.48-drivex}
NS3_BUILD_DIR=${NS3_BUILD_DIR:-$NS3_ROOT/build}
NS3_CMAKE_DIR=${NS3_CMAKE_DIR:-}
# Name of the scratch sub-directory holding drivex-bridge.cc. It also fixes
# the CMake target name, so keep the directory and this variable in sync.
NS3_BRIDGE_DIR=${NS3_BRIDGE_DIR:-drivex-bridge}
BRIDGE_PORT=${BRIDGE_PORT:-5555}
GNB_X=${GNB_X:--42}
GNB_Y=${GNB_Y:-25}
# Access technology of the simulated network and how the edge is reached:
#   ACCESS    uu (5G NR Uu) | 11p (IEEE 802.11p) | 11bd (IEEE 802.11bd) |
#             nrv2x (NR-V2X sidelink, PC5 mode 2)
#   TOPOLOGY  rsu    the RSU is the edge's only radio: CAV uplinks, VRU VAMs
#                    and edge warnings all use ACCESS (not with uu)
#             hybrid only the CAV uplinks use ACCESS; VRUs and warnings stay
#                    on 5G NR Uu
ACCESS=${ACCESS:-uu}
TOPOLOGY=${TOPOLOGY:-rsu}
INJECTED_LOSS_PERC=${INJECTED_LOSS_PERC:-0}
BRIDGE_RNG_RUN=${BRIDGE_RNG_RUN:-1}
BRIDGE_EXTRA_ARGS=${BRIDGE_EXTRA_ARGS:-}

export SIMO_PACKAGE_DIR=${SIMO_PACKAGE_DIR:-$WORKSPACE_ROOT/simo_proto}

ca_activate_env() {
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate "$CA_CONDA_ENV"
}

port_listening() {      # TCP port -> 0 when something listens on it
    ss -ltn 2>/dev/null | grep -Eq ":$1\b"
}

udp_port_bound() {
    ss -uln 2>/dev/null | grep -Eq ":$1\b"
}

# Value of a `--name value` / `--name=value` option among the given args.
arg_value() {
    local name=$1 default=$2; shift 2
    local args=("$@") i value=$default
    for i in "${!args[@]}"; do
        if [ "${args[$i]}" = "$name" ]; then
            value="${args[$((i + 1))]:-$default}"
        elif [[ "${args[$i]}" == "$name="* ]]; then
            value="${args[$i]#"$name"=}"
        fi
    done
    echo "$value"
}

# The CARLA map of a run: --town, else scenario.town of the --config yaml.
ca_town() {
    local town config
    town=$(arg_value --town "" "$@")
    if [ -z "$town" ]; then
        config=$(arg_value --config "ca_convoy.yaml" "$@")
        [[ "$config" != /* ]] && config="$CA_ROOT/$config"
        town=$(python -c "
import sys
from omegaconf import OmegaConf
print(OmegaConf.load(sys.argv[1])['scenario']['town'])" "$config")
    fi
    echo "$town"
}

# ---- CARLA ------------------------------------------------------------------
CARLA_PIDS=()
CARLA_EXTRA_APP_ARGS=()     # what the application must be told about the server

_carla_spawn() {            # log file, then the CarlaUE4.sh arguments
    local log=$1; shift
    setsid "$CARLA_PATH/CarlaUE4.sh" "$@" >"$log" 2>&1 &
    CARLA_PIDS+=($!)
}

_carla_wait_port() {        # port, pid, log
    local port=$1 pid=$2 log=$3 i
    for i in $(seq 1 90); do
        port_listening "$port" && return 0
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "ERROR: CARLA exited before opening port $port." >&2
            tail -30 "$log" >&2
            return 1
        fi
        sleep 2
    done
    echo "ERROR: CARLA did not open port $port." >&2
    tail -30 "$log" >&2
    return 1
}

# carla_start <log directory or ""> <town>
# Reuses a server already listening on CARLA_PORT (unless MANAGE_CARLA=1),
# otherwise starts one: a single server, or with CARLA_GPUS holding more than
# one GPU a multi-GPU set (a primary server without rendering plus one
# rendering server per GPU, all started on <town>: in that mode a map loaded
# from the client would not reach the rendering servers).
carla_start() {
    local log_dir=$1 town=$2 render gpu n i sec_port log
    CARLA_EXTRA_APP_ARGS=()
    read -r -a gpus <<<"$CARLA_GPUS"
    n=${#gpus[@]}
    if port_listening "$CARLA_PORT"; then
        if [ "$MANAGE_CARLA" = "1" ]; then
            echo "ERROR: managed CARLA port ${CARLA_PORT} is already in use." >&2
            return 1
        fi
        echo "Reusing the CARLA server already listening on port ${CARLA_PORT}."
        # a multi-GPU server cannot take a map from the client
        [ "$n" -gt 1 ] && CARLA_EXTRA_APP_ARGS=(--no-reload-world)
        return 0
    fi
    if [ "${CARLA_RENDER:-0}" = "1" ]; then render=(); else render=(-RenderOffScreen); fi
    [ -n "$log_dir" ] || log_dir=/tmp
    log="$log_dir/carla.log"
    [ "$log_dir" = /tmp ] && log="/tmp/carla_${CARLA_PORT}.log"

    if [ "$n" -le 1 ]; then
        gpu=${gpus[0]:-$CARLA_GPU}
        echo "No CARLA server on port ${CARLA_PORT}; launching one" \
             "${gpu:+on GPU $gpu }(log $log)..."
        _carla_spawn "$log" "${render[@]}" -nosound -quality-level="$CARLA_QUALITY" \
            -carla-rpc-port="$CARLA_PORT" ${gpu:+-graphicsadapter="$gpu"}
        _carla_wait_port "$CARLA_PORT" "${CARLA_PIDS[-1]}" "$log" || return 1
    else
        local primary_port=$((CARLA_PORT + 2)) map="/Game/Carla/Maps/$town"
        echo "Launching CARLA in multi-GPU mode on $town: primary on port" \
             "${CARLA_PORT}, rendering servers on GPUs ${gpus[*]} (log $log)..."
        _carla_spawn "$log" "$map" -nullrhi -nosound \
            -carla-rpc-port="$CARLA_PORT" -carla-primary-port="$primary_port"
        _carla_wait_port "$CARLA_PORT" "${CARLA_PIDS[-1]}" "$log" || return 1
        sleep 5
        for i in "${!gpus[@]}"; do
            sec_port=$((CARLA_PORT + 10 * (i + 1)))
            _carla_spawn "${log%.log}_gpu${gpus[$i]}.log" "$map" "${render[@]}" -nosound \
                -quality-level="$CARLA_QUALITY" -carla-rpc-port="$sec_port" \
                -carla-primary-host=127.0.0.1 -carla-primary-port="$primary_port" \
                "-ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=${gpus[$i]}"
        done
        for i in "${!gpus[@]}"; do
            _carla_wait_port "$((CARLA_PORT + 10 * (i + 1)))" "${CARLA_PIDS[$((i + 1))]}" \
                "${log%.log}_gpu${gpus[$i]}.log" || return 1
        done
        CARLA_EXTRA_APP_ARGS=(--no-reload-world)
        sleep 20        # the rendering servers finish loading the map
    fi
    sleep 5
}

carla_stop() {
    # Each server is the leader of its own process group (setsid): the group
    # holds the CarlaUE4.sh wrapper and the engine binary, and it is the
    # group that has to be gone, not the wrapper alone.
    local pid
    for pid in "${CARLA_PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    for pid in "${CARLA_PIDS[@]}"; do
        for _ in $(seq 1 20); do
            kill -0 -- "-$pid" 2>/dev/null || break
            sleep 1
        done
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    [ "${#CARLA_PIDS[@]}" -gt 0 ] && echo "Stopped the CARLA server(s) of this run."
    CARLA_PIDS=()
}

# ---- ns-3 drivex-bridge -----------------------------------------------------
BRIDGE_PID=""

bridge_binary() {
    ls "$NS3_BUILD_DIR"/scratch/"$NS3_BRIDGE_DIR"/ns3.*-drivex-bridge-* 2>/dev/null \
        | grep -v '\.o$' | head -1
}

bridge_build() {
    if [ -n "$NS3_CMAKE_DIR" ] && [ -f "$NS3_CMAKE_DIR/CMakeCache.txt" ]; then
        cmake --build "$NS3_CMAKE_DIR" \
            --target "scratch_${NS3_BRIDGE_DIR}_drivex-bridge" -j "${NS3_BUILD_JOBS:-4}"
    else
        (cd "$NS3_ROOT" && ./ns3 build drivex-bridge)
    fi
}

# The bridge arguments that select the access technology and the topology.
bridge_access_args() {
    local topo=()
    [ "$TOPOLOGY" = "rsu" ] && topo=(--topology=rsu)
    case "$TOPOLOGY" in
        rsu|hybrid) ;;
        *) echo "ERROR: TOPOLOGY must be rsu or hybrid, got '$TOPOLOGY'" >&2; return 1 ;;
    esac
    case "$ACCESS" in
        uu)    echo "" ;;
        11p)   echo "--rsuLink=11p ${topo[*]}" ;;
        11bd)  echo "--rsuLink=bd ${topo[*]}" ;;
        nrv2x) if [ "$TOPOLOGY" = "rsu" ]; then
                   echo "--rsuLink=pc5 --v2vMode=pc5 ${topo[*]}"
               else
                   echo "--rsuLink=pc5 --v2vMode=relay"
               fi ;;
        *) echo "ERROR: ACCESS must be uu, 11p, 11bd or nrv2x, got '$ACCESS'" >&2
           return 1 ;;
    esac
}

# bridge_start <app mode> <output directory>
# A fresh bridge for the run: its station mapping and clock epoch are per run.
bridge_start() {
    local app_mode=$1 out_dir=$2 bin source access_args vbs i
    if udp_port_bound "$BRIDGE_PORT"; then
        echo "ERROR: UDP port ${BRIDGE_PORT} is already in use." >&2
        echo "A drivex-bridge must be started FRESH for every run (its station" >&2
        echo "mapping and clock epoch are locked per run): stop the existing" >&2
        echo "one first, e.g.:" >&2
        echo "  echo '{\"msg_type\":\"shutdown\"}' | nc -u -w1 127.0.0.1 ${BRIDGE_PORT}" >&2
        return 1
    fi
    source="$NS3_ROOT/scratch/$NS3_BRIDGE_DIR/drivex-bridge.cc"
    bin=$(bridge_binary)
    if [ -z "$bin" ] || [ "$source" -nt "$bin" ]; then
        echo "drivex-bridge missing or stale; building it..."
        bridge_build || return 1
        bin=$(bridge_binary)
    fi
    if [ -z "$bin" ] || [ "$source" -nt "$bin" ]; then
        echo "ERROR: no up-to-date drivex-bridge binary in $NS3_BUILD_DIR" >&2
        return 1
    fi
    access_args=$(bridge_access_args) || return 1
    # modes 3/4: the connected VRUs run DriveX's VRU Basic Service in ns-3
    vbs=0; [ "$app_mode" -ge 3 ] && vbs=1
    MAXUES=${MAXUES:-$([ "$app_mode" -ge 3 ] && echo 32 || echo 8)}
    mkdir -p "$out_dir"
    BRIDGE_LOG="$out_dir/bridge.log"
    echo "Starting the drivex-bridge: access ${ACCESS}$([ "$ACCESS" != uu ] && echo ", topology ${TOPOLOGY}")," \
         "VBS $([ $vbs = 1 ] && echo on || echo off), maxUes=${MAXUES}," \
         "gNB/RSU (${GNB_X}, ${GNB_Y}), injected loss ${INJECTED_LOSS_PERC} %, log ${BRIDGE_LOG}"
    # shellcheck disable=SC2086
    LD_LIBRARY_PATH="$NS3_BUILD_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" setsid "$bin" \
        --maxUes="$MAXUES" --gnbX="$GNB_X" --gnbY="$GNB_Y" \
        --listenPort="$BRIDGE_PORT" --vbs="$vbs" \
        --strictClock=1 \
        --injected-loss-perc="$INJECTED_LOSS_PERC" --RngRun="$BRIDGE_RNG_RUN" \
        --csvSend="$out_dir/bridge_send.csv" --csvRecv="$out_dir/bridge_recv.csv" \
        $access_args $BRIDGE_EXTRA_ARGS \
        >"$BRIDGE_LOG" 2>&1 </dev/null &
    BRIDGE_PID=$!
    for i in $(seq 1 60); do
        grep -q "listening for JSON" "$BRIDGE_LOG" 2>/dev/null && break
        if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
            echo "ERROR: the drivex-bridge died on startup:" >&2
            tail -5 "$BRIDGE_LOG" >&2
            return 1
        fi
        sleep 2
    done
    if ! grep -q "listening for JSON" "$BRIDGE_LOG" 2>/dev/null; then
        echo "ERROR: the drivex-bridge did not become ready:" >&2
        tail -20 "$BRIDGE_LOG" >&2
        return 1
    fi
    echo "drivex-bridge ready."
}

bridge_stop() {
    if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo "Stopping the drivex-bridge (pid $BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
    fi
    BRIDGE_PID=""
}
