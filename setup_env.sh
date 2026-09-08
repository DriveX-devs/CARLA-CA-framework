#!/usr/bin/env bash
# Pre-flight check for the CA experiments: verifies that every dependency this
# repository needs is in place and prints exactly what is missing.
#
#   ./CA/setup_env.sh              # check everything
#   ./CA/setup_env.sh --no-ns3     # skip the ns-3 / v2x-bridge section
#                                  # (enough for --app-mode 1, local CA only)
#   ./CA/setup_env.sh --no-carla   # skip the CARLA installation check
#
# Exit status: 0 = everything required is present, 1 = at least one FAIL.
# WARNs are things you only need for part of the pipeline.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OPENCDA_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

[ -f "$SCRIPT_DIR/env.sh" ] && . "$SCRIPT_DIR/env.sh"

CHECK_NS3=1
CHECK_CARLA=1
for a in "$@"; do
    case "$a" in
        --no-ns3)   CHECK_NS3=0 ;;
        --no-carla) CHECK_CARLA=0 ;;
        -h|--help)  sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown option: $a" >&2; exit 2 ;;
    esac
done

CA_CONDA_ENV=${CA_CONDA_ENV:-msvan3t_carla}
CARLA_PATH=${CARLA_PATH:-$WORKSPACE_ROOT/CARLA_0.9.12}
CARLA_PORT=${CARLA_PORT:-2000}
NS3_ROOT=${NS3_ROOT:-$WORKSPACE_ROOT/ns-3-dev}
NS3_BRIDGE_DIR=${NS3_BRIDGE_DIR:-v2x-bridge}
NS3_BUILD_DIR=${NS3_BUILD_DIR:-$NS3_ROOT/build-campaign}

FAILS=0
WARNS=0
if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'
else R=""; G=""; Y=""; B=""; N=""; fi

ok()   { printf '  %sPASS%s  %s\n' "$G" "$N" "$1"; }
warn() { printf '  %sWARN%s  %s\n' "$Y" "$N" "$1"; WARNS=$((WARNS + 1)); }
bad()  { printf '  %sFAIL%s  %s\n' "$R" "$N" "$1"; FAILS=$((FAILS + 1)); }
head_() { printf '\n%s== %s%s\n' "$B" "$1" "$N"; }

# --- 1. repository layout ----------------------------------------------------
head_ "Repository layout"
echo "  workspace : $WORKSPACE_ROOT"
echo "  OpenCDA   : $OPENCDA_ROOT"
echo "  CA        : $SCRIPT_DIR"
if [ "$(basename "$SCRIPT_DIR")" = "CA" ]; then
    ok "this repository is checked out as <OpenCDA>/CA"
else
    bad "this repository must be checked out as <OpenCDA>/CA (found '$(basename "$SCRIPT_DIR")')"
fi
for d in opencda dataset_generator; do
    if [ -d "$OPENCDA_ROOT/$d" ]; then
        ok "$d/ found in the parent OpenCDA checkout"
    else
        bad "$OPENCDA_ROOT/$d is missing — clone https://github.com/DriveX-devs/OpenCDA first"
    fi
done

# --- 2. simulation conda environment ----------------------------------------
head_ "Simulation environment ($CA_CONDA_ENV)"
if ! command -v conda >/dev/null 2>&1; then
    bad "conda not found in PATH — install miniconda/anaconda"
else
    . "$(conda info --base)/etc/profile.d/conda.sh"
    if conda env list | awk '{print $1}' | grep -qx "$CA_CONDA_ENV"; then
        ok "conda env '$CA_CONDA_ENV' exists"
        PY="$(conda run -n "$CA_CONDA_ENV" python -c 'import sys; print(sys.version.split()[0])' 2>/dev/null)"
        [ -n "$PY" ] && ok "python $PY" || bad "cannot run python inside '$CA_CONDA_ENV'"
        for m in carla numpy cv2 torch ultralytics omegaconf; do
            if conda run -n "$CA_CONDA_ENV" python -c "import $m" >/dev/null 2>&1; then
                ok "import $m"
            else
                bad "import $m fails in '$CA_CONDA_ENV'"
            fi
        done
        for m in asn1tools geopy; do
            if conda run -n "$CA_CONDA_ENV" python -c "import $m" >/dev/null 2>&1; then
                ok "import $m"
            else
                warn "import $m fails — needed for VAM encoding (--app-mode 3/4): pip install $m"
            fi
        done
        if PYTHONPATH="$OPENCDA_ROOT" conda run -n "$CA_CONDA_ENV" \
                python -c "import opencda" >/dev/null 2>&1; then
            ok "import opencda (from the parent checkout)"
        else
            bad "import opencda fails — run 'pip install -e .' inside $OPENCDA_ROOT"
        fi
    else
        bad "conda env '$CA_CONDA_ENV' does not exist — see README, section 'Environment'"
    fi
fi

# --- 3. YOLOv8 weights -------------------------------------------------------
head_ "YOLOv8 weights ($SCRIPT_DIR/models)"
for w in yolov8n.pt yolov8m.pt yolov8x.pt; do
    if [ -s "$SCRIPT_DIR/models/$w" ]; then
        ok "$w ($(du -h "$SCRIPT_DIR/models/$w" | cut -f1))"
    else
        warn "$w missing — ultralytics downloads it on first use of that capacity"
    fi
done

# --- 4. CARLA ----------------------------------------------------------------
if [ "$CHECK_CARLA" = "1" ]; then
    head_ "CARLA 0.9.12"
    if [ -x "$CARLA_PATH/CarlaUE4.sh" ]; then
        ok "CarlaUE4.sh found in $CARLA_PATH"
    else
        bad "$CARLA_PATH/CarlaUE4.sh not found — set CARLA_PATH in CA/env.sh"
    fi
    if ss -ltn 2>/dev/null | grep -qE ":${CARLA_PORT}\b"; then
        ok "a server is already listening on RPC port $CARLA_PORT"
    else
        warn "no server on port $CARLA_PORT — run.sh/run_extended.sh will start one"
    fi
fi

# --- 5. ns-3 + 5G-LENA + v2x-bridge -----------------------------------------
if [ "$CHECK_NS3" = "1" ]; then
    head_ "ns-3.46 + 5G-LENA + v2x-bridge (needed for --app-mode 2/3/4)"
    if [ -d "$NS3_ROOT" ]; then
        ok "ns-3 tree: $NS3_ROOT"
        if [ -d "$NS3_ROOT/contrib/nr" ] || [ -d "$NS3_ROOT/src/nr" ]; then
            ok "5G-LENA (nr) module present"
        else
            bad "the nr module is missing — clone 5g-lena-v4.1.1 into $NS3_ROOT/contrib/nr"
        fi
        if [ -f "$NS3_ROOT/scratch/$NS3_BRIDGE_DIR/v2x-bridge.cc" ]; then
            ok "v2x-bridge sources: scratch/$NS3_BRIDGE_DIR/"
        else
            bad "scratch/$NS3_BRIDGE_DIR/v2x-bridge.cc missing — git clone https://github.com/DriveX-devs/ns-3-v2x-bridge into $NS3_ROOT/scratch/$NS3_BRIDGE_DIR (or set NS3_BRIDGE_DIR)"
        fi
        BIN=$(find "$NS3_BUILD_DIR/scratch/$NS3_BRIDGE_DIR" -maxdepth 1 \
              -type f -name 'ns3.*-v2x-bridge-default' 2>/dev/null | sort | head -1)
        if [ -n "$BIN" ]; then
            ok "bridge binary built: ${BIN#"$NS3_ROOT/"}"
        else
            warn "no bridge binary under $NS3_BUILD_DIR — the campaign scripts build it on first run"
        fi
    else
        bad "$NS3_ROOT does not exist — set NS3_ROOT in CA/env.sh"
    fi
fi

# --- summary -----------------------------------------------------------------
printf '\n%s== Summary%s\n' "$B" "$N"
if [ "$FAILS" -eq 0 ] && [ "$WARNS" -eq 0 ]; then
    printf '  %severything in place%s\n' "$G" "$N"
elif [ "$FAILS" -eq 0 ]; then
    printf '  %s%d warning(s)%s, nothing blocking\n' "$Y" "$WARNS" "$N"
else
    printf '  %s%d blocking problem(s)%s and %d warning(s) — see above\n' \
        "$R" "$FAILS" "$N" "$WARNS"
fi
[ "$FAILS" -eq 0 ]
