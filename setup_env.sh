#!/usr/bin/env bash
# Pre-flight check for the CA experiments: verifies that every dependency this
# repository relies on is where the launchers will look for it.
#
#   ./setup_env.sh              # check everything
#   ./setup_env.sh --no-ns3     # skip ns-3 / bridge (enough for --app-mode 1)
#   ./setup_env.sh --no-carla   # skip the CARLA installation check
#
# Exit status: 0 = everything required is present, 1 = at least one FAIL.
# WARNs are things you only need for part of the pipeline.
#
# Nothing is installed or modified: it only reports what is missing and how to
# get it (see README.md, "Setup from scratch").
set -u

CHECK_NS3=1
CHECK_CARLA=1
for arg in "$@"; do
    case "$arg" in
        --no-ns3)   CHECK_NS3=0 ;;
        --no-carla) CHECK_CARLA=0 ;;
        -h|--help)  sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/launch_lib.sh"

if [ -t 1 ]; then G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; N=$'\e[0m'; else G=; Y=; R=; N=; fi
FAILS=0
WARNS=0
ok()   { printf '  %sPASS%s  %s\n' "$G" "$N" "$1"; }
warn() { printf '  %sWARN%s  %s\n' "$Y" "$N" "$1"; WARNS=$((WARNS + 1)); }
bad()  { printf '  %sFAIL%s  %s\n' "$R" "$N" "$1"; FAILS=$((FAILS + 1)); }

echo "Repository   : $CA_ROOT"
echo "Workspace    : $WORKSPACE_ROOT"
[ -f "$CA_ROOT/env.sh" ] && echo "env.sh       : found" || echo "env.sh       : none (defaults)"

echo; echo "Python environment ($CA_CONDA_ENV)"
if ! command -v conda >/dev/null 2>&1; then
    bad "conda not found on PATH"
elif ! conda env list | awk '{print $1}' | grep -qx "$CA_CONDA_ENV"; then
    bad "conda env '$CA_CONDA_ENV' does not exist (README 2.3)"
else
    ok "conda env '$CA_CONDA_ENV' exists"
    ca_activate_env
    python - <<'PY'
import importlib, sys
G, Y, R, N = ("\033[32m", "\033[33m", "\033[31m", "\033[0m") if sys.stdout.isatty() else ("",) * 4
fails = 0
def line(tag, colour, text):
    print("  %s%s%s  %s" % (colour, tag, N, text))
if sys.version_info[:2] < (3, 10) or sys.version_info[:2] > (3, 12):
    line("FAIL", R, "Python %s: the CARLA 0.9.16 client needs 3.10-3.12" % sys.version.split()[0]); fails += 1
else:
    line("PASS", G, "Python %s" % sys.version.split()[0])
required = [("carla", "CARLA client (the wheel in <CARLA>/PythonAPI/carla/dist)"),
            ("torch", "PyTorch"), ("torchvision", "torchvision"), ("numpy", "numpy"),
            ("scipy", "scipy"), ("cv2", "opencv"), ("omegaconf", "omegaconf"),
            ("yaml", "PyYAML"), ("PIL", "Pillow"), ("einops", "einops (SiMO)"),
            ("efficientnet_pytorch", "efficientnet_pytorch (SiMO)"),
            ("asn1tools", "asn1tools (VAM codec)"), ("geopy", "geopy")]
for name, what in required:
    try:
        mod = importlib.import_module(name)
        line("PASS", G, "%s %s" % (what, getattr(mod, "__version__", "")))
    except Exception as exc:
        line("FAIL", R, "%s: import %s failed (%s)" % (what, name, type(exc).__name__)); fails += 1
try:
    import efficientnet_pytorch
    if efficientnet_pytorch.__version__ != "0.7.0":
        line("FAIL", R, "efficientnet_pytorch %s: SiMO's camera branch needs 0.7.0 "
             "(0.7.1 changes its output)" % efficientnet_pytorch.__version__); fails += 1
except Exception:
    pass
try:
    import torch
    if torch.cuda.is_available():
        line("PASS", G, "CUDA: %d GPU(s)" % torch.cuda.device_count())
    else:
        line("FAIL", R, "PyTorch sees no CUDA device"); fails += 1
except Exception:
    pass
import importlib.util
if importlib.util.find_spec("ultralytics") is not None:
    line("PASS", G, "ultralytics (optional yolo backend)")
else:
    line("WARN", Y, "ultralytics missing: only needed for --perception yolo")
sys.exit(1 if fails else 0)
PY
    [ $? -eq 0 ] || FAILS=$((FAILS + 1))
fi

echo; echo "SiMO package ($SIMO_PACKAGE_DIR)"
if [ -d "$SIMO_PACKAGE_DIR/simo_proto" ]; then
    ok "simo_proto/ found"
    n=$(ls "$SIMO_PACKAGE_DIR"/weights/*.pth 2>/dev/null | wc -l)
    [ "$n" -gt 0 ] && ok "weights: $(ls "$SIMO_PACKAGE_DIR"/weights/*.pth | xargs -n1 basename | tr '\n' ' ')" \
        || bad "no .pth file in $SIMO_PACKAGE_DIR/weights"
    [ -f "$SIMO_PACKAGE_DIR/configs/simo_mm10.yaml" ] && ok "configs/simo_mm10.yaml" \
        || bad "configs/simo_mm10.yaml missing"
else
    bad "no simo_proto/ in $SIMO_PACKAGE_DIR (set SIMO_PACKAGE_DIR in env.sh)"
fi

if [ "$CHECK_CARLA" = 1 ]; then
    echo; echo "CARLA ($CARLA_PATH)"
    if [ -x "$CARLA_PATH/CarlaUE4.sh" ]; then
        ok "CarlaUE4.sh found ($(cat "$CARLA_PATH/VERSION" 2>/dev/null || echo 'version unknown'))"
        [ "$(cat "$CARLA_PATH/VERSION" 2>/dev/null)" = "0.9.16" ] || \
            warn "the framework is tested with CARLA 0.9.16"
        if [ -d "$CARLA_PATH/CarlaUE4/Content/Carla/Maps/Town12" ]; then
            ok "AdditionalMaps installed (Town06, Town07, Town11, Town12, Town13, Town15)"
        else
            warn "AdditionalMaps not installed: only the base maps are available"
        fi
    else
        bad "no CarlaUE4.sh in $CARLA_PATH (set CARLA_PATH in env.sh)"
    fi
    command -v nvidia-smi >/dev/null 2>&1 && \
        ok "GPUs: $(nvidia-smi --query-gpu=name --format=csv,noheader | sort | uniq -c | sed 's/^ *//' | tr '\n' ';')" \
        || warn "nvidia-smi not found"
fi

if [ "$CHECK_NS3" = 1 ]; then
    echo; echo "ns-3 + DriveX ($NS3_ROOT)   [app modes 2 / 3 / 4]"
    if [ ! -x "$NS3_ROOT/ns3" ]; then
        bad "no ns-3 tree at $NS3_ROOT (set NS3_ROOT in env.sh)"
    else
        ok "ns-3 tree found ($(cat "$NS3_ROOT/VERSION" 2>/dev/null))"
        [ -d "$NS3_ROOT/contrib/nr" ] && ok "5G-LENA (contrib/nr)" || bad "contrib/nr missing"
        for m in facilities metrics access-80211p access-80211bd access-nrv2x; do
            [ -e "$NS3_ROOT/contrib/drivex-$m/CMakeLists.txt" ] && ok "DriveX module drivex-$m" \
                || bad "contrib/drivex-$m missing or a broken link"
        done
        src="$NS3_ROOT/scratch/$NS3_BRIDGE_DIR/drivex-bridge.cc"
        if [ -f "$src" ]; then
            ok "bridge source scratch/$NS3_BRIDGE_DIR/drivex-bridge.cc"
            grep -q '"vbs"' "$src" && ok "the bridge has the VRU Basic Service (--vbs)" \
                || bad "this drivex-bridge has no --vbs: it predates the framework"
            bin=$(bridge_binary)
            if [ -z "$bin" ]; then
                warn "bridge not built yet (the launchers build it on first use)"
            elif [ "$src" -nt "$bin" ]; then
                warn "bridge binary older than its source (rebuilt on next use)"
            else
                ok "bridge binary up to date"
            fi
        else
            bad "no scratch/$NS3_BRIDGE_DIR/drivex-bridge.cc (NS3_BRIDGE_DIR?)"
        fi
        vbs="$NS3_ROOT/contrib/drivex-facilities/model/vru-basic-service.cc"
        if [ -f "$vbs" ]; then
            awk '/VRUBasicService::initDissemination/,/^}/' "$vbs" | grep -q 'generateAndEncodeVam' \
                && ok "DriveX VRU Basic Service sends its first VAM" \
                || bad "DriveX VRU Basic Service lacks its first VAM: apply patches/drivex-vbs-first-vam.diff of the bridge"
        fi
    fi
fi

echo
if [ "$FAILS" -eq 0 ] && [ "$WARNS" -eq 0 ]; then
    printf '  %sall checks passed%s\n' "$G" "$N"
elif [ "$FAILS" -eq 0 ]; then
    printf '  %s%d warning(s)%s, nothing blocking\n' "$Y" "$WARNS" "$N"
else
    printf '  %s%d blocking problem(s)%s, %d warning(s)\n' "$R" "$FAILS" "$N" "$WARNS"
fi
[ "$FAILS" -eq 0 ]
