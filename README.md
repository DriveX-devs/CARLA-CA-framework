# CA — Cooperative Awareness with camera/LiDAR perception over a simulated 5G network

Experimental setup for **cooperative collision avoidance** between connected
vehicles (CAVs) and vulnerable road users (VRUs) at an urban intersection.
Four CAVs perceive the scene with **YOLOv8 + LiDAR fusion**, share what they see
with an **edge server over a simulated 5G NR network** (ns-3 + 5G-LENA), and the
edge runs a **centralised collision-avoidance algorithm** whose warnings travel
back over the same simulated network. Every decision is logged against CARLA
ground truth.

This repository contains what is needed to **run the experiments and collect
their raw results**. Turning those results into figures and reports is a
separate concern and is not published here.

The pipeline compares cooperation policies under identical, byte-for-byte
replayable traffic:

| policy | app mode | cell directory | what the collision avoidance sees |
|---|---|---|---|
| **On-board** | `1` | `policy_0_onboard` | each CAV decides alone, from its own perception |
| **Collaborative** | `2` | `policy_1_collaborative` | CAV detections uploaded to the edge over 5G; edge decides |
| **Collaborative + VAM** | `3` | `policy_2_vam` | as above, plus VRUs broadcasting ETSI **VAMs** to the edge |
| **VAM-only** | `4` | `policy_3_vam_only` | edge fed **only** by VAMs and CAV ego states |

The full technical description — sensor rig, fusion maths, coordinate frames,
tracker, evaluation rules and the **complete configuration reference** — is in
**[PIPELINE.md](PIPELINE.md)**.

---

## ⚠️ This repository is a component, not a standalone program

The code imports `opencda.*` from
[DriveX-devs/OpenCDA](https://github.com/DriveX-devs/OpenCDA). It **must be
checked out as the `CA/` sub-directory of an OpenCDA clone**, and every command
below is run **from the OpenCDA root**, not from inside `CA/`.

---

## 1. Architecture

```
                    ┌───────────────────────────────────────────┐
   CARLA 0.9.12 ◄───┤  OpenCDA (DriveX-devs)                    │
   (Town10HD_Opt)   │    └── CA/  ← THIS REPOSITORY             │
        ▲           │          ├── perception  (YOLOv8 + LiDAR) │
        │  sensors  │          ├── PyCA        (t2c / s2c)      │
        │  + GT     │          └── python_vru_service (LDM/VAM) │
        └───────────┤                     │                     │
                    └─────────────────────┼─────────────────────┘
                                          │ UDP/JSON control
                                          ▼
                    ┌───────────────────────────────────────────┐
                    │  ns-3.46 + 5G-LENA v4.1.1                 │
                    │    scratch/v2x-bridge  (DriveX-devs)      │
                    │    → simulated 5G NR uplink/downlink      │
                    └───────────────────────────────────────────┘
```

### External dependencies

| # | component | source | needed for |
|---|---|---|---|
| 1 | **OpenCDA** | `github.com/DriveX-devs/OpenCDA` | always — this repo lives inside it |
| 2 | **CARLA 0.9.12** | `github.com/carla-simulator/carla/releases/tag/0.9.12` | always |
| 3 | **conda env `msvan3t_carla`** (Python 3.7) | built from OpenCDA's `environment.yml` + extras | always |
| 4 | **ns-3.46** | `gitlab.com/nsnam/ns-3-dev` | app modes 2 / 3 / 4 |
| 5 | **5G-LENA `5g-lena-v4.1.1`** | `gitlab.com/cttc-lena/nr` | app modes 2 / 3 / 4 |
| 6 | **v2x-bridge** | `github.com/DriveX-devs/ns-3-v2x-bridge` | app modes 2 / 3 / 4 |

A campaign that only collects the on-board policy (app mode 1) needs no ns-3 at
all: items 4–6 can be skipped entirely.

### Expected directory layout

The scripts derive every path from this layout, so keeping it saves you from
configuring anything:

```
<workspace>/                     # any directory you like
├── OpenCDA/                     # 1. github.com/DriveX-devs/OpenCDA
│   ├── opencda/
│   └── CA/                      #    ← THIS REPOSITORY
├── CARLA_0.9.12/                # 2. CARLA package (CarlaUE4.sh inside)
└── ns-3-dev/                    # 4. ns-3.46
    ├── contrib/nr/              # 5. 5G-LENA
    └── scratch/v2x-bridge/      # 6. the bridge
```

Anything placed elsewhere is fine too — set the corresponding variable in
`CA/env.sh` (see [§2.6](#26-per-machine-configuration)).

---

## 2. Setup from scratch

### 2.0. Prerequisites

Linux (tested on Ubuntu), an NVIDIA GPU with a working driver (CARLA renders
four 1920×1080 cameras per CAV), `conda`, `git`, `cmake ≥ 3.13`, `ninja`, a
C++17 compiler, and enough free disk for the campaign you intend to collect
(see [§3.3](#33-disk)).

```bash
export WS="$HOME/workspace"          # pick your workspace directory
mkdir -p "$WS" && cd "$WS"
```

### 2.1. OpenCDA

```bash
cd "$WS"
git clone https://github.com/DriveX-devs/OpenCDA.git
```

### 2.2. This repository

```bash
cd "$WS/OpenCDA"
git clone https://github.com/DriveX-devs/<CA-REPO-NAME>.git CA
```

> The clone directory **must** be named `CA`: the entry points are executed as
> `python CA/run_ca.py` from the OpenCDA root and the package is imported as
> `CA.fusion`, `CA.tracker`, …

### 2.3. CARLA 0.9.12

```bash
cd "$WS"
# package from the 0.9.12 release page (github.com/carla-simulator/carla/releases/tag/0.9.12)
wget https://github.com/carla-simulator/carla/releases/download/0.9.12/CARLA_0.9.12.tar.gz
mkdir -p CARLA_0.9.12 && tar -xzf CARLA_0.9.12.tar.gz -C CARLA_0.9.12
./CARLA_0.9.12/CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000   # smoke test
```

Additional maps are not needed: the scenario uses **Town10HD_Opt**, which ships
with the base package.

### 2.4. Python environment (`msvan3t_carla`)

CARLA 0.9.12's Python client is **Python 3.7 only**, which caps some packages —
in particular `ultralytics` at **8.0.145** (YOLO11+ requires Python ≥ 3.8), so
the experiments use **YOLOv8**.

```bash
cd "$WS/OpenCDA"
conda env create -f environment.yml        # creates msvan3t_carla (Python 3.7)
conda activate msvan3t_carla

pip install -e .                           # make `import opencda` work
pip install "ultralytics==8.0.145" torch torchvision   # perception
pip install asn1tools geopy                # VAM UPER codec + geodesy (modes 3/4)
```

Reference versions of a known-good environment: Python 3.7.15, torch 1.8.0,
ultralytics 8.0.145, numpy 1.21.6, pandas 1.3.5, opencv 4.5.2.

The YOLOv8 weights (`yolov8n/m/x.pt`) are downloaded automatically into
`CA/models/` the first time each capacity is used; they are not committed.

### 2.5. ns-3.46 + 5G-LENA + v2x-bridge  *(skip for app mode 1)*

```bash
cd "$WS"
git clone https://gitlab.com/nsnam/ns-3-dev.git
cd ns-3-dev && git checkout ns-3.46

# 5G-LENA
git clone https://gitlab.com/cttc-lena/nr.git contrib/nr
cd contrib/nr && git checkout 5g-lena-v4.1.1 && cd ../..

# the bridge — note the explicit target directory name
git clone https://github.com/DriveX-devs/ns-3-v2x-bridge.git scratch/v2x-bridge

./ns3 configure -d default --disable-examples --disable-tests
./ns3 build scratch_v2x-bridge_v2x-bridge
```

> **The clone directory name matters.** ns-3 derives the CMake target from it
> (`scratch/<dir>` → `scratch_<dir>_v2x-bridge`). Cloning without the trailing
> `scratch/v2x-bridge` argument produces `scratch/ns-3-v2x-bridge/` instead —
> valid, but you must then set `NS3_BRIDGE_DIR=ns-3-v2x-bridge` in `CA/env.sh`
> so the scripts build and locate the right target.

Follow [the 5G-LENA installation guide](https://gitlab.com/cttc-lena/nr) if the
build fails; the bridge itself is documented in its own repository.

`run_campaign.sh` does not touch this build tree: it configures a dedicated pair
(`build-campaign/` + `cmake-cache-campaign/`) and builds the bridge itself, so
your day-to-day `./ns3 build` stays untouched. A **release/optimized** ns-3
profile is strongly recommended for long campaigns — a debug build lags on the
first 30-UE burst.

### 2.6. Per-machine configuration

```bash
cd "$WS/OpenCDA/CA"
cp env.example.sh env.sh
$EDITOR env.sh
```

`env.sh` is **gitignored** and sourced by every launcher. Set only what deviates
from the layout of [§1](#expected-directory-layout):

| variable | default | meaning |
|---|---|---|
| `CARLA_PATH` | `<workspace>/CARLA_0.9.12` | directory containing `CarlaUE4.sh` |
| `CARLA_PORT` | `2000` | CARLA RPC port for interactive runs |
| `CA_CONDA_ENV` | `msvan3t_carla` | conda environment |
| `NS3_ROOT` | `<workspace>/ns-3-dev` | ns-3.46 source tree |
| `NS3_BRIDGE_DIR` | `v2x-bridge` | name of the scratch sub-directory of the bridge |
| `NS3_BUILD_DIR` / `NS3_CMAKE_DIR` | `build-campaign` / `cmake-cache-campaign` | campaign build tree |
| `BRIDGE_PORT`, `GNB_X`, `GNB_Y` | `5555`, `-42`, `25` | bridge control port and gNB position |

### 2.7. Verify

```bash
./CA/setup_env.sh              # everything
./CA/setup_env.sh --no-ns3     # enough for app mode 1
```

It prints one `PASS` / `WARN` / `FAIL` line per dependency and exits non-zero if
anything blocking is missing. Fix every `FAIL` before running.

---

## 3. Running experiments

All commands are run **from the OpenCDA root** (`$WS/OpenCDA`).

### 3.1. A single run

```bash
./CA/run.sh                                    # local CA only, 40 s, YOLOv8-n
./CA/run.sh --model m --seconds 20             # capacities: n | m | x
CARLA_RENDER=1 ./CA/run.sh                     # with a CARLA window

./CA/run_extended.sh --app-mode 2 --seconds 20 # + edge CA over simulated 5G
./CA/run_extended.sh --app-mode 3              # + VRU VAMs
./CA/run_extended.sh --app-mode 4              # VAM-only
```

These launchers start CARLA if no server is listening, start a **fresh**
v2x-bridge when the mode needs one (its UE mapping and clock epoch are locked
per run), execute the scenario and, unless `ANALYZE=0`, run
`CA/analyze_ca_metrics.py` on the resulting run directory.

`run_extended_pl.sh` is the packet-loss variant: it adds
`--injected-loss-perc` and `--RngRun` to the bridge; the Python side is
unchanged.

Useful flags of `run_ca_extended.py` (forwarded through the launchers):
`--config`, `--model {n,m,x}`, `--seconds`, `--seed`, `--run-dir`,
`--traffic-manifest` / `--create-traffic-manifest` (replayable background
traffic), `--save-stride` (`0` = no annotated frames — they are ~97 % of a run's
size), `--warning-cooldown` / `--no-warning-cooldown`, `--apply-isinrange`,
`--fixed-t2c-threshold`, `--raw-only`, `--v2x-verbose`.

### 3.2. A campaign — `run_campaign.sh`

A *campaign* is a set of runs, one per random seed; each run collects one *cell*
per (policy × YOLOv8 capacity), optionally repeated over a sweep of injected
packet-loss levels. Every cell of a seed replays the same traffic manifest, so
the policies are compared on byte-identical ground truth.

```bash
./CA/run_campaign.sh                                     # 1 seed, 3 policies, n/m/x
./CA/run_campaign.sh --runs 10 --seconds 80              # 10 seeds × 9 cells
./CA/run_campaign.sh --policies "1" --models "n"         # smallest useful campaign
./CA/run_campaign.sh --config CA/ca_convoy_t2c3_s2c3.yaml  # tighter CA thresholds
./CA/run_campaign.sh --policies "3" --loss "0 10 30 50 90" # packet-loss sweep
./CA/run_campaign.sh --out CA/output/mycampaign --attach --policies "4"
./CA/run_campaign.sh --dry-run                           # print the commands only
```

| option | env | default | meaning |
|---|---|---|---|
| `--out DIR` | `OUT_DIR` | `CA/output/campaign_<timestamp>` | campaign root |
| `--runs N` | `NUM_RUNS` | `1` | number of seeds |
| `--base-seed N` | `BASE_SEED` | `11` | run *k* uses seed `BASE_SEED + k - 1` |
| `--seconds N` | `RUN_DURATION_S` | `80` | duration of one cell |
| `--policies "1 2 3"` | `APP_MODES` | `1 2 3` | app modes to collect |
| `--models "n m x"` | `MODELS` | `n m x` | YOLOv8 capacities |
| `--config PATH` | `CA_CONFIG` | the app default | scenario yaml |
| `--loss "0 10 50"` | `LOSS_PERCENTAGES` | none | injected packet loss [%]; cells nest under `loss_<pp>/` |
| `--save-stride N` | `SAVE_STRIDE` | `0` | annotated frames every N steps; `0` = none |
| `--attach` | `ATTACH` | `0` | add policies to the runs already in `--out`, replaying their seeds and manifests |
| `--no-smoke` | `SMOKE` | on | skip the 2 s sanity cell per policy |
| `--min-free-gb N` | `MIN_FREE_GB` | `60` | abort when free disk drops below this |
| `--max-attempts N` | `CELL_MAX_ATTEMPTS` | `3` | retries of a cell before giving up |

What it does for you:

* builds and **probes** the v2x-bridge before starting (a payload round-trip,
  plus a 100 %-loss probe when a loss sweep is requested), and skips ns-3
  entirely when only app mode 1 is collected;
* generates one **traffic manifest per seed** with a 2 s run, then replays it in
  every cell of that seed — across policies and across loss levels;
* is **resumable**: a cell holding `_SUCCESS` is re-validated, not re-run;
* **retries** a cell when CARLA segfaults, preserving the failed attempt under
  `<run>/_failed/`;
* runs a **disk watchdog** that aborts the campaign, and everything it spawned,
  before the filesystem fills up;
* validates every cell and every completed run with
  `CA/validate_campaign_raw.py`;
* writes a `campaign.json` descriptor and a `batch.log` per campaign.

Campaign layout:

```
<out>/
├── batch.log
├── manifests/traffic_manifest_seed<S>.json    one traffic layout per seed
├── _manifest_smoke/seed<S>/mode_1             the 2 s run that created it
└── [loss_<pp>/]run_<k>_seed<S>/
    ├── campaign.json
    ├── _smoke/mode_<m>                        2 s sanity cell per policy
    └── <policy dir>/<model>/                  the cells
```

Always start a new setup with `--dry-run`, then with a single short cell
(`--policies "1" --models "n" --seconds 4`), before committing to a long
campaign.

### 3.3. Disk

A cell writing annotated camera frames is ~2.9 GB, of which ~2.57 GB are the
frames; no metric reads them, which is why `--save-stride` defaults to `0`. At
that default a 10-seed × 9-cell campaign costs a few GB; with frames it costs
~250 GB. The disk watchdog (`--min-free-gb`, default 60 GiB) stops a campaign
before it fills the filesystem.

---

## 4. Collecting results

Everything a run produces goes under `CA/output/` and is **gitignored**.

Per run directory (`CA/output/run_<timestamp>/` or a campaign cell):

| file | written in | content |
|---|---|---|
| `metrics.csv` | always | one row per step, per CAV, per identified object: estimated position/speed/heading, matched GT actor (`gt_id`, `gt_name`, `gt_category`), errors, collision output (`t2c`, `s2c`, `ca_warning`, `ca_type`), edge-warning columns, and the per-step timing split (`t_detect_ms`, `t_fuse_ms`, `t_track_ms`, `t_ca_ms`, `t_total_ms`) |
| `gt.csv` | always | ground truth of every dynamic object at every step |
| `gt_ca.csv` | always | the ground-truth collision decision for every (step, CAV, actor) pair: `distance_m`, `t2c`, `s2c`, `in_range`, `gt_ca_warning` — the reference every policy is scored against |
| `track_associations.csv` | always | which perception track was matched to which ground-truth actor, and at what distance |
| `stations.csv` | always | the ITS stations of the run (CAVs, VRUs) and whether they are connected |
| `timing.csv` | always | per-step perception timing |
| `run_metadata.json` | always | app mode, collision thresholds and the full resolved configuration of the run |
| `carla.log` | always | the CARLA server log of that run |
| `_SUCCESS` | on completion | marker used for resumability; its absence means the cell must be re-run |
| `edge_warnings.csv` | modes 2/3/4 | edge collision warnings with their end-to-end latency components |
| `v2x_packets.csv` | modes 2/3/4 | every simulated 5G packet (uplink detections, VAMs, downlink warnings) |
| `bridge.log`, `bridge_send.csv`, `bridge_recv.csv` | modes 2/3/4 | ns-3 bridge logs |
| `VBS_stats.csv` | mode 3 | one row per generated VAM |
| `summary.txt` | without `--raw-only` | mean/RMSE errors and mean per-operation timing, per CAV and category |
| `<Cav>/<camera>_<step>.png` | `--save-stride > 0` | annotated frames |

Two tools operate on these outputs:

```bash
# per-run metrics with 95 % confidence intervals: VRUs detected within a radius,
# misdetection probability (Wilson), and total warning latency (sensing +
# perception + uplink + edge CA + downlink + actuation)
python CA/analyze_ca_metrics.py CA/output/run_<timestamp> --vru-distance 40

# structural validation, no metrics: required files, headers, step counts, and
# identical ground-truth digests across the cells of a run
python CA/validate_campaign_raw.py run <cell_dir> --app-mode 3 --steps 1600
python CA/validate_campaign_raw.py campaign <run_dir> --steps 1600 \
    --policies 0,1,2 --models n,m,x
```

`run_campaign.sh` calls the validator itself, after every cell and at the end of
every run.

> **Analysis is out of scope here.** Producing figures, tables and reports from
> these CSVs is done outside this repository.

---

## 5. Repository layout

| path | role |
|---|---|
| `run_ca.py` | main loop, local CA only (world setup, CAVs, background actors, evaluation) |
| `run_ca_extended.py` | the V2X application: app modes 1–4, LDMs, VAMs, edge server |
| `sensors.py` | camera/LiDAR rig, queue-synced with world ticks |
| `detector.py` | batched YOLOv8 wrapper (one instance per CAV, capacities n/m/x) |
| `fusion.py` | LiDAR→camera frustum association + multi-camera merge |
| `tracker.py` | constant-velocity Kalman tracker → velocity + heading |
| `experiment_support.py` | shared scenario helpers (traffic manifests, actor naming) |
| `PyCA/` | collision-avoidance algorithm: analytic `t2c` / `s2c` solver + tests |
| `python_vru_service/` | LDM, ETSI VRU Basic Service (VAM), UPER codec, ns-3 bridge client, edge server |
| `ca_convoy.yaml` | scenario / sensors / fusion / tracking / collision configuration |
| `ca_convoy_t2c3_s2c3.yaml` | same with tightened CA thresholds (3 s / 3 m) |
| `run.sh`, `run_extended.sh`, `run_extended_pl.sh` | single-run launchers |
| `run_campaign.sh` | generic campaign driver |
| `analyze_ca_metrics.py` | per-run metrics with 95 % CIs |
| `validate_campaign_raw.py` | structural validation of runs and campaigns |
| `setup_env.sh`, `env.example.sh` | dependency check and per-machine configuration |
| `PIPELINE.md` | full technical documentation + configuration reference |

---

## 6. Troubleshooting

**`Town10HD_Opt is not found in your CARLA repo!` followed by
`'ScenarioManager' object has no attribute 'world'`.** Misleading message: a
crashed run left the server in **synchronous mode with no ticker**, so
`load_world` timed out. Restart the CARLA server.

**CARLA dies at boot with `Signal 11`.** Check the run's `carla.log`; the server
can segfault repeatedly right after a crash — relaunch until the RPC port
listens. Note that `ss | grep ':2000'` also matches `:20000`; use
`grep -E ':2000\b'`. `run_campaign.sh` retries a cell up to `--max-attempts`
times for exactly this reason.

**`ERROR: UDP port 5555 is already in use`.** A bridge must be started fresh for
every run. Stop the previous one:
`echo '{"msg_type":"shutdown"}' | nc -u -w1 127.0.0.1 5555`.

**`no up-to-date default v2x-bridge binary`.** `NS3_BRIDGE_DIR` does not match
the directory you cloned the bridge into, or the ns-3 build failed. Re-run
`./CA/setup_env.sh`.

**ninja manifest still dirty / CMake cache mismatch.** A CMake cache is bound to
the absolute path it was configured with; a moved or copied ns-3 tree needs a
fresh `cmake -S … -B …`. `run_campaign.sh` detects this and reconfigures its own
cache automatically.

**The bridge lags on the first burst in modes 3/4.** 30 UEs on a *debug* ns-3
build is slow; use an optimized profile for campaigns.

**`import opencda` fails.** `pip install -e .` was not run inside the OpenCDA
root, or the wrong conda environment is active.

**Unmatched perception tracks in `metrics.csv`.** Town10HD_Opt contains static
decorative scenery vehicles that are map meshes, not CARLA actors: they are
correctly detected but have no ground truth. Not a bug.

---

## 7. License and credits

This repository is released under the [MIT License](LICENSE).

It builds on components with their own terms, which apply to their own code:

* **OpenCDA** — Academic Software License, © 2021 UCLA Mobility Lab: academic
  and non-profit research use only. This repository does not redistribute
  OpenCDA; you clone it yourself.
* **ns-3-v2x-bridge** and **ns-3** — GPLv2.
* **5G-LENA (nr)** — GPLv2.
* **CARLA** — MIT (the simulator; its assets have their own terms).
* **Ultralytics YOLOv8** — AGPL-3.0. Weights are downloaded at runtime, not
  redistributed here.

The LDM and VRU Basic Service in `python_vru_service/` are Python ports of the
corresponding C++ implementations in **VaN3Twin / OScar**
([DriveX-devs](https://github.com/DriveX-devs)).
