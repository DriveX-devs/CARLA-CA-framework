# Per-machine configuration for the CA experiments — NOT tracked by git.
#
#   cp env.example.sh env.sh   &&   edit env.sh
#
# env.sh is sourced (when present) by every launcher before anything else, so
# every variable below can also be overridden ad hoc on the command line, e.g.
#
#   CARLA_PORT=2100 ACCESS=11p ./run_extended.sh --app-mode 3
#
# Run ./setup_env.sh to check that what you set here actually exists.

# ---------------------------------------------------------------------------
# CARLA
# ---------------------------------------------------------------------------
# Root of the CARLA 0.9.16 package (the directory containing CarlaUE4.sh).
# Default when unset: <workspace>/CARLA_0.9.16, i.e. a sibling of this repo.
#export CARLA_PATH="$HOME/CARLA_0.9.16"

# RPC port of the CARLA server (default 2000). The campaign scripts use their
# own port (CAMPAIGN_CARLA_PORT, default 2100) so a campaign never collides
# with an interactive server.
#export CARLA_PORT=2000

# GPU of the CARLA server the launchers start (default: CARLA's own choice).
#export CARLA_GPU=0

# CARLA's multi-GPU mode: one rendering server per listed GPU behind a primary
# server that renders nothing. The cameras are spread over them; worth it with
# camera-based perception (SiMO with cameras, YOLOv8), not for LiDAR alone.
# See the README for what this mode cannot do.
#export CARLA_GPUS="0 1"

# ---------------------------------------------------------------------------
# conda environment
# ---------------------------------------------------------------------------
# Python 3.10, CARLA 0.9.16 client, PyTorch, the SiMO package's requirements.
#export CA_CONDA_ENV=ca_carla916

# ---------------------------------------------------------------------------
# SiMO
# ---------------------------------------------------------------------------
# Directory holding simo_proto/, configs/ and weights/.
# Default when unset: <workspace>/simo_proto.
#export SIMO_PACKAGE_DIR="$HOME/simo_proto"

# ---------------------------------------------------------------------------
# ns-3 + DriveX + drivex-bridge  (only needed for --app-mode 2 / 3 / 4)
# ---------------------------------------------------------------------------
# ns-3.48 tree with 5G-LENA, WiGig and the DriveX modules.
# Default when unset: <workspace>/ns-3.48-drivex.
#export NS3_ROOT="$HOME/ns-3.48-drivex"

# Name of the sub-directory of $NS3_ROOT/scratch/ holding drivex-bridge.cc.
# This name is part of the CMake target, so it must match the directory.
#export NS3_BRIDGE_DIR=drivex-bridge

# Build tree (default: the tree's own build/ and cmake-cache/).
#export NS3_BUILD_DIR="$NS3_ROOT/build"
#export NS3_CMAKE_DIR="$NS3_ROOT/cmake-cache"
#export NS3_BUILD_JOBS=4

# Access technology and topology of the simulated network:
#   ACCESS    uu (5G NR Uu, default) | 11p | 11bd | nrv2x
#   TOPOLOGY  rsu (default: the RSU is the edge's only radio) | hybrid (only
#             the CAV uplinks use ACCESS; VRUs and warnings stay on NR Uu)
#export ACCESS=uu
#export TOPOLOGY=rsu

# UDP control port of the bridge (default 5555) and gNB / RSU position, in
# CARLA world coordinates — (-42, 25) is the centre of the Town10HD_Opt
# intersection; another map needs its own.
#export BRIDGE_PORT=5555
#export GNB_X=-42
#export GNB_Y=25

# Further arguments for the bridge, e.g. a moving 5G channel (see the README).
#export BRIDGE_EXTRA_ARGS="--channelUpdateMs=100"
