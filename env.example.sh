# Per-machine configuration for the CA experiments — NOT tracked by git.
#
#   cp env.example.sh env.sh   &&   edit env.sh
#
# env.sh is sourced (when present) by run.sh, run_extended.sh and
# run_extended_pl.sh before anything else, so every variable below can also be
# overridden ad hoc on the command line, e.g.
#
#   CARLA_PORT=2100 ./CA/run_extended.sh --app-mode 3
#
# Run ./CA/setup_env.sh to check that what you set here actually exists.

# ---------------------------------------------------------------------------
# CARLA
# ---------------------------------------------------------------------------
# Root of the CARLA 0.9.12 package (the directory containing CarlaUE4.sh).
# Default when unset: <workspace>/CARLA_0.9.12, i.e. a sibling of OpenCDA/.
#export CARLA_PATH="$HOME/CARLA_0.9.12"

# RPC port of the CARLA server (default 2000). The campaign scripts use their
# own port (CAMPAIGN_CARLA_PORT, default 2100) so a campaign never collides
# with an interactive server.
#export CARLA_PORT=2000

# ---------------------------------------------------------------------------
# conda environment
# ---------------------------------------------------------------------------
# Simulation env: Python 3.7, CARLA 0.9.12 client, OpenCDA deps, ultralytics.
#export CA_CONDA_ENV=msvan3t_carla

# ---------------------------------------------------------------------------
# ns-3 + 5G-LENA + v2x-bridge  (only needed for --app-mode 2 / 3 / 4)
# ---------------------------------------------------------------------------
# ns-3.46 source tree. Default when unset: <workspace>/ns-3-dev.
#export NS3_ROOT="$HOME/ns-3-dev"

# Name of the sub-directory of $NS3_ROOT/scratch/ holding v2x-bridge.cc.
# `git clone https://github.com/DriveX-devs/ns-3-v2x-bridge` creates
# "ns-3-v2x-bridge"; cloning into an explicit "v2x-bridge" directory keeps the
# default. This name is part of the CMake target, so it must match the clone.
#export NS3_BRIDGE_DIR=v2x-bridge

# Build tree used by the campaign scripts. Keeping a dedicated cache/output
# pair leaves your day-to-day `./ns3 build` tree untouched.
#export NS3_BUILD_DIR="$NS3_ROOT/build-campaign"
#export NS3_CMAKE_DIR="$NS3_ROOT/cmake-cache-campaign"
#export NS3_BUILD_JOBS=4

# UDP control port of the bridge (default 5555) and gNB position, in CARLA
# world coordinates — (-42, 25) is the centre of the Town10HD_Opt intersection.
#export BRIDGE_PORT=5555
#export GNB_X=-42
#export GNB_Y=25
