# User guide — Collision Avoidance simulation with LDM, VAMs and ns-3 5G

This guide covers the **integrated Collision Avoidance simulation** launched by
`OpenCDA/CA/run_ca_extended.py`: the run_ca.py scenario (4 sensorized CAVs +
background traffic, bikes and pedestrians in CARLA Town10HD_Opt), the modules
of this package (LDM + VRU Basic Service), the PyCA collision-avoidance
algorithm (`OpenCDA/CA/PyCA`) and the ns-3 `v2x-bridge`
(`ns-3-dev/scratch/v2x-bridge`) simulating the 5G NR network between
CAVs/VRUs and an edge server.

Four cases are supported (`--app-mode`):

| Mode | Name | Collision avoidance runs... | VAMs |
|---|---|---|---|
| 1 | **Decentralized CA** | on board of each CAV, on its own perception | no |
| 2 | **Centralized CA, no VAMs** | at the edge server, on the detections uploaded by the CAVs over 5G | no |
| 3 | **Centralized CA + VAMs** | at the edge server, on the CAV detections **and** the VAMs transmitted by the VRUs over 5G | yes |
| 4 | **Centralized CA, VAMs only** | at the edge server, on the VAMs and the CAV ego states **only** (no CAV detection is uploaded), pairing CAVs with VRUs only | yes |

In modes 2/3/4 the decentralized CA of mode 1 keeps running and being logged,
so local and edge collision avoidance can be compared on the same run.

---

## 1. Components

```
                       ┌──────────────────────────── one Python process ───────────────────────────┐
 ┌─────────┐  CARLA    │  run_ca_extended.py                                                       │
 │  CARLA  │  RPC      │  ┌────────────────────────┐        ┌────────────────────────────────────┐ │
 │ server  │◄─────────►│  │ per-CAV perception     │        │ V2X layer (opencda_v2x.V2XLayer)   │ │
 │ (ticked │  :2000    │  │ 4 cams + LiDAR         │ tracks │  CavV2XManager (x4): local LDM     │ │
 │  by the │           │  │ YOLOv8 + fusion +      ├───────►│  VruVamManager (x26, mode 3):      │ │
 │  app)   │           │  │ Kalman tracking        │        │    VRU Basic Service → VAMs        │ │
 └─────────┘           │  │ + local PyCA CA        │        │  EdgeCAServer ("BS"): LDM with     │ │
                       │  └────────────────────────┘        │    object matching + PyCA          │ │
                       │                                    │    CollisionAvoidanceService       │ │
                       │                                    │  Ns3BridgeClient ─────────┐        │ │
                       │                                    └───────────────────────────┼────────┘ │
                       └────────────────────────────────────────────────────────────────┼──────────┘
                                                                    real UDP, JSON      │ :5555
                                                                                        ▼
                                                     ┌──────────────────────────────────────────────┐
                                                     │ ns-3 v2x-bridge (external process)           │
                                                     │  simulated 5G NR: UE ↔ gNB ↔ EPC ↔ "BS"/MEC  │
                                                     │  external-clock mode: sim clock follows the  │
                                                     │  CARLA timestamps sent by the client         │
                                                     └──────────────────────────────────────────────┘
```

| Component | File(s) | Role |
|---|---|---|
| **CA perception pipeline** | `OpenCDA/CA/run_ca_extended.py` (+ `sensors.py`, `detector.py`, `fusion.py`, `tracker.py`) | Spawns the scenario, ticks CARLA, runs per CAV: YOLOv8 on 4 Full-HD cameras + camera/LiDAR frustum fusion + constant-velocity Kalman tracking → confirmed tracks (position, speed, heading per object). Validated live against ground truth. |
| **Local (decentralized) CA** | `OpenCDA/CA/PyCA/t2c.py` (`t2c` class) | Every step, each CAV checks its own true kinematics against every confirmed track: analytic time-to-collision (`t2c`) and space-to-collision (`s2c`), thresholded warning (`ca_warning` in `metrics.csv`). Always active, in every mode. |
| **CAV local LDM** | `ldm.py`, used by `opencda_v2x.CavV2XManager` | Modes 2/3: each CAV stores its confirmed tracks in its own LDM (`detected=True`, `perceived_by=<CAV station>`), then uploads them to the edge. |
| **VRU Basic Service** | `vru_basic_service.py`, used by `opencda_v2x.VruVamManager` | Mode 3: each VRU (20 pedestrians + 6 bikes) triggers ETSI TS 103 300-3 VAMs (heading/position/speed change, max elapsed time), UPER-encoded with `vam_codec.py`. Stepped manually on the CARLA simulation clock (no wall-clock threads). The VRU state comes from its own CARLA actor (self-knowledge = its GNSS/IMU), never from perception. |
| **Edge server** | `opencda_v2x.EdgeCAServer` + `ldm.py` + `PyCA/t2c.py` (`CollisionAvoidanceService`) | Modes 2/3: the application behind the bridge's reserved `"BS"` endpoint. One LDM with **object matching** (detections of the same object from different CAVs merge into one entry) fed by the delivered detection uploads and VAMs; every LDM update feeds the centralized cross-road-only PyCA CA. Confirmed cross-road risks produce one warning packet per involved CAV, sent over the simulated downlink; rear-end pairs are classified only for diagnostics and skipped. |
| **ns-3 bridge client** | `ns3_bridge_client.Ns3BridgeClient` | Sends the JSON control datagrams to the bridge and matches the JSON replies back by `packet_id`. Implements the protocol rules: CARLA simulation time as clock reference, stable actor IDs, `"BS"` as the edge, one packet per datagram, unique `packet_id`s, `request_reply: true` on every packet. |
| **ns-3 v2x-bridge** | `ns-3-dev/scratch/v2x-bridge/v2x-bridge.cc` | Separate process. Simulates the 5G NR network (band n78, 60 MHz, numerology 1, TDD) between the UEs (CAVs/VRUs) and the edge server, in **external-clock mode**: its simulation clock advances exactly as dictated by the `timestamp` of the incoming commands. It carries the actual application payload inside the simulated packets and reports each delivery (with latency) back to the client. Full protocol reference: `ns-3-dev/scratch/v2x-bridge/README.md`. |

> `examples/example_opencda.py` is a *separate, standalone* variant of this
> integration (a passive CARLA client in its own process, with a UDP detection
> feed); it is **not** used by `run_ca_extended.py`. See
> `examples/README_opencda.md`.

### Identifiers, frames, clock

- **Origin IDs (bridge)**: stable, human-readable actor names used as ns-3
  `origin_ID` for the whole run: `Cav1..Cav4`, `ped_1..ped_20`,
  `bike_1..bike_6`, plus the reserved `BS` for the edge server. The first
  appearance of an ID permanently claims one UE of the bridge pool.
- **Numeric station IDs (LDM / VAM / CA)**: `Cav<i>` → `100+i` (101..104),
  i-th VRU (pedestrians first, then bikes) → `1000+i` (1001..1026). A
  detection gets the ID `cav_station * 100000 + track_id` (e.g. `10200058` =
  track 58 of Cav2, station 102), which keeps CAV-local track IDs unique at
  the edge.
- **Frames**: the LDM / collision avoidance work in an ENU frame (x = east,
  y = north, compass heading clockwise from north): CARLA world maps as
  `x_enu = x`, `y_enu = -y`, `heading = (carla_yaw + 90) % 360`. VAM
  positions are WGS84 lat/lon, converted back to the same ENU frame at the
  edge using the CARLA map geo-origin. The bridge `entities` positions stay
  in raw CARLA coordinates (they only drive the ns-3 radio geometry).
- **Clock**: everything runs on the CARLA simulation time
  (`world.get_snapshot().timestamp.elapsed_seconds`): the VAM triggering
  conditions, the CA staleness/cooldowns, and the ns-3 simulation clock
  itself (external-clock mode). Wall-clock speed is irrelevant.

---

## 2. Interface formats (localhost UDP)

There is a single localhost UDP interface: the **JSON control protocol** of
the v2x-bridge (default `127.0.0.1:5555`), plus the **application payloads**
carried inside it. One UDP datagram = one JSON object; one simulated packet
per datagram.

### 2.1 Client → bridge: scene update (entities only)

Sent once per CARLA tick to refresh the position of every connected actor
(CAVs in mode 2; CAVs + VRUs in modes 3/4). Units: metres, m/s, radians.

```json
{
  "msg_type": "control",
  "timestamp": 12.35,
  "entities": [
    {"timestamp": 12.35, "origin_ID": "Cav1", "origin_vehicle_type": "cav",
     "Position": {"x_m": -45.2, "y_m": 92.5, "z_m": 0.2},
     "Velocity": 7.8, "Heading": 1.5708}
  ]
}
```

`timestamp` is the CARLA simulation time in seconds — it drives the bridge's
simulation clock.

### 2.2 Client → bridge: simulated packet command

One datagram per simulated packet, always with `request_reply: true` and a
unique integer `packet_id`. The sender's entity state is included in the same
datagram (it must be known to the bridge; same-datagram introduction is
allowed). The **application payload** travels base64-encoded in
`packet.payload`, embedded inside the simulated NR packet (after a 12-byte
id+length header), so the actual information — not only the size — reaches
the simulated receiver. `size_bytes` = 12 + payload length.

```json
{
  "msg_type": "control",
  "timestamp": 12.35,
  "entities": [ { "...sender state as above..." } ],
  "packet": {
    "sender": "Cav1", "receiver": "BS",
    "size_bytes": 250, "packet_id": 42,
    "type": "detection", "request_reply": true,
    "payload": "<base64 bytes>"
  }
}
```

The three uses of this message:

| Flow | `sender` → `receiver` | `type` | payload |
|---|---|---|---|
| CAV detection upload | `Cav<i>` → `BS` | `detection` | detection JSON (§2.4) |
| VRU VAM (modes 3/4) | `ped_<i>`/`bike_<i>` → `BS` | `detection` | UPER-encoded VAM bytes (§2.5) |
| Edge warning downlink | `BS` → `Cav<i>` | `warning` | warning JSON (§2.6) |

### 2.3 Bridge → client: delivery reply

Sent to the source address:port of the command, once the packet is delivered
inside the simulation (or fails). The receiver-side logic (edge LDM/CA, CAV
warning logging) runs **only** on `status == "delivered"`, using the
`payload` extracted at the destination node.

```json
{
  "msg_type": "reply", "packet_id": 42, "type": "detection",
  "status": "delivered", "latency_ms": 5.065, "sim_time_ms": 12850.065,
  "payload": "<base64 bytes as received at the destination>"
}
```

`status` ∈ `delivered | timeout | error | error_duplicate` (`latency_ms` is
`null` unless delivered; the client additionally logs `no_reply` if no reply
arrives within 15 wall seconds, e.g. bridge not running).

### 2.4 Application payload: CAV detection upload (JSON, UTF-8)

The confirmed tracks of one CAV at one step, plus its ego state
(self-knowledge). All positions/headings in the shared ENU frame; `type` is
the ETSI station type (1 pedestrian, 2 cyclist, 5 passenger car, ...).

```json
{
  "kind": "detections", "cav": "Cav1", "t": 12.35,
  "ego": {"station_id": 101, "x": -45.2, "y": -92.5,
          "speed": 7.8, "heading": 0.0, "acceleration": 0.4},
  "tracks": [
    {"id": 10100007, "type": 1, "x": -41.2, "y": -88.7,
     "speed": 1.35, "heading": 192.8}
  ]
}
```

At the edge: `ego` is inserted in the LDM as a connected station, every
track as a detected object (`detected=True, perceived_by=101`), subject to
object matching across CAVs.

### 2.5 Application payload: VAM (binary, UPER)

The raw UPER encoding of an ETSI VAM (`VAM-PDU-Descriptions` /
`ETSI-ITS-CDD`, ~34 bytes), produced by `vru_basic_service.py` +
`vam_codec.py`: header (`stationId`), `basicContainer` (station type, WGS84
reference position) and `vruHighFrequencyContainer` (heading, speed,
longitudinal acceleration). The edge decodes it with
`LDM.add_vru_from_vam()` and stores the VRU as a connected station.

### 2.6 Application payload: edge warning (JSON, UTF-8)

Generated by the edge CA for each involved CAV of a confirmed risk pair
(per-pair warning cooldown, configurable via `--warning-cooldown` /
`--no-warning-cooldown`; historical default 2 s, but the comparability campaign
runs with it **disabled** so edge warnings are per-frame like the on-board/GT
streams):

```json
{
  "kind": "warning", "t": 12.45, "target": "Cav2",
  "collision_type": "Cross-Road-Collision",
  "t2c": 2.32, "s2c": 3.13,
  "station_id": 102, "other_station": 10200058,
  "other_type": "pedestrian", "other_x": -73.1, "other_y": 3.6
}
```

On delivery, the CAV logs it to `edge_warnings.csv` and to the
`edge_warn_*` columns of `metrics.csv`.

---

## 3. Prerequisites

- CARLA **0.9.12** server binaries, and the `msvan3t_carla` conda env
  (OpenCDA deps + `carla==0.9.12`) with, additionally:

  ```bash
  conda activate msvan3t_carla
  pip install asn1tools geopy
  ```

- ns-3 (**ns-3.46**) with **5G-LENA (nr) v4.1.1** and the `v2x-bridge`
  scratch program built:

  ```bash
  cd <ns-3-root>          # on this machine: ~/ns-3-dev (build from this
  ./ns3 build v2x-bridge  # path — the CMake cache is anchored here)
  ```

- The ASN.1 files `ETSI-ITS-CDD.asn` and `VAM-PDU-Descriptions.asn` (already
  inside `python_vru_service/`; a different folder can be set with the
  `VAM_ASN_DIR` environment variable).

---

## 4. Step-by-step: the four cases

All commands assume the repo layout of this project; `<ns-3-root>` is the
ns-3 tree containing `scratch/v2x-bridge`.

> **All-in-one launcher.** `OpenCDA/CA/run_extended.sh` automates every step
> below for any mode: it activates the conda env, launches CARLA if needed,
> starts a **fresh** v2x-bridge when `--app-mode` is 2/3/4 (with the right
> `--maxUes`, gNB at the scenario center, logs/CSVs under `CA/output/`),
> runs the application, runs the post-run metric analysis (§6.1) on the new
> run directory, and stops the bridge on exit. Extra args are forwarded to
> `run_ca_extended.py`; environment overrides: `CARLA_PORT`, `CARLA_RENDER`,
> `NS3_ROOT`, `BRIDGE_PORT`, `MAXUES`, `GNB_X`/`GNB_Y`, `ANALYZE=0` to skip
> the analysis. The original `CA/run.sh` (plain run_ca.py) is unchanged.
>
> ```bash
> ./CA/run_extended.sh --app-mode 3 --seconds 40
> ```
>
> The manual steps below remain useful when you want to keep the bridge in a
> separate terminal (e.g. custom radio parameters or `--verbose`).

### Case 1 — Decentralized Collision Avoidance (`--app-mode 1`, default)

Each CAV runs the PyCA t2c/s2c check on board, on its own perception. No
LDM, no VAMs, no network: identical to `run_ca.py`.

```bash
# 1. CARLA server (skip if already running; run_ca.sh-style headless launch)
./CarlaUE4.sh -RenderOffScreen -quality-level=Epic -carla-rpc-port=2000

# 2. The application
conda activate msvan3t_carla
cd OpenCDA
python CA/run_ca_extended.py --app-mode 1 --model n --seconds 40
```

What to look for: `ca_warning`/`t2c`/`s2c`/`ca_type` columns in
`metrics.csv`, red boxes in the annotated frames, per-CAV `CA :` lines in
`summary.txt`. The `edge_warn_*` columns stay empty and no V2X files are
produced.

### Case 2 — Centralized Collision Avoidance without VAMs (`--app-mode 2`)

CAVs keep their detections in their local LDM and upload them to the edge
server through the simulated 5G network; the edge LDM + PyCA centralized CA
generate the warnings, sent back to the involved CAVs over the simulated
downlink and logged on reception. VRUs stay silent.

```bash
# 1. CARLA server (as above)

# 2. ns-3 bridge — start BEFORE the application. Only the 4 CAVs connect:
cd <ns-3-root>
./ns3 run "v2x-bridge --gnbX=-42 --gnbY=25 --maxUes=4"
#          gNB at the Town10 central intersection (the scenario center)

# 3. The application
conda activate msvan3t_carla
cd OpenCDA
python CA/run_ca_extended.py --app-mode 2 --seconds 40
```

Console: `[EDGE] <type> risk between A and B ... -> warning to CavX` when
the edge CA confirms a risk, and `[CavX] EDGE WARNING received ...` when the
warning is delivered over the downlink. End-of-run V2X statistics land in
`summary.txt`.

### Case 3 — Centralized Collision Avoidance with VAMs (`--app-mode 3`)

Like case 2, plus every VRU (20 pedestrians + 6 bikes) runs a VRU Basic
Service transmitting UPER-encoded VAMs to the edge over 5G, as additional
input to the centralized CA (VRUs appear at the edge as connected stations,
e.g. warnings against station `1021` = bike_1, alongside the detected
objects).

```bash
# 1. CARLA server (as above)

# 2. ns-3 bridge — 30 UEs now (4 CAVs + 20 pedestrians + 6 bikes):
cd <ns-3-root>
./ns3 run "v2x-bridge --gnbX=-42 --gnbY=25 --maxUes=32"

# 3. The application
conda activate msvan3t_carla
cd OpenCDA
python CA/run_ca_extended.py --app-mode 3 --seconds 40
```

Additional console lines with `--v2x-verbose`: `[VRU ped_N] VAM TX
trigger=<condition> ...` per transmitted VAM. `summary.txt` reports the VAMs
sent per VRU, split by ETSI triggering condition.

### Case 4 — VAM-only Centralized Collision Avoidance (`--app-mode 4`)

The V2X-only policy: the VRUs transmit VAMs exactly as in case 3, but the CAVs
**never upload a perceived object** — their uplink carries the ego state alone
— and the edge raises **CAV<->VRU risks only** (two CAV ego states are never
checked against each other). The on-board perception and its local CA keep
running and being logged, so the same run still carries the decentralized
baseline. Background vehicles transmit nothing and are therefore invisible to
the edge.

```bash
# Same CARLA + 32-UE bridge as case 3, then:
conda activate msvan3t_carla
cd OpenCDA
python CA/run_ca_extended.py --app-mode 4 --seconds 40
```

`run_metadata.json` records `policy: 3`, `app_mode: 4`,
`edge_uses_cav_detections: false`, `edge_pairs: "cav_vru_only"`. The campaign
form of this policy is `CA/run_campaign.sh --policies "4"` (see
`CA/PIPELINE.md` §9).

Between runs the bridge can stay up **only if** restarted per run is not
needed — the `origin_ID → UE` mapping and the external-clock epoch are
locked per run, so in practice: **restart the bridge for every run**
(Ctrl-C, or `echo '{"msg_type":"shutdown"}' | nc -u -w1 127.0.0.1 5555`).

---

## 5. Command-line reference

### `run_ca_extended.py` (the scenario/perception options are those of run_ca.py)

| Option | Default | Meaning |
|---|---|---|
| `--app-mode {1,2,3,4}` | `1` | 1 = decentralized CA; 2 = centralized CA over ns-3 (no VAMs); 3 = centralized CA + VRU VAMs; 4 = centralized CA from VAMs + CAV ego states only, CAV<->VRU pairs only |
| `--bridge-host` / `--bridge-port` | `127.0.0.1` / `5555` | v2x-bridge control endpoint (modes 2/3/4), must match the bridge `--listenPort` |
| `--bridge-wait` | `0.25` | max wall seconds per tick spent waiting for the ns-3 delivery replies (two waits per tick: uplink, then warnings) |
| `--v2x-verbose` | off | per-update V2X prints (local LDM inserts, VAM TX lines, per-packet bridge traffic) |
| `--config` | `CA/ca_convoy.yaml` | scenario + sensors + fusion + tracking + collision config (thresholds `collision.t2c_th`/`s2c_th`/`alpha_th_deg` are used by BOTH the local and the edge CA) |
| `--model {n,m,x}` | yaml | YOLOv8 capacity |
| `--seconds N` | yaml (40) | simulated duration |
| `--save-stride N` | yaml (`20`) | annotated camera frames every N steps, `0` = none. Illustrative only (no metric reads them) but ~97% of a run's size |
| `-p/--port` | yaml (2000) | CARLA RPC port |
| `-tm/--tm_port` | `8000` | Traffic Manager port |

### `v2x-bridge` (most relevant; full list in its README)

| Option | Suggested here | Meaning |
|---|---|---|
| `--gnbX/--gnbY/--gnbZ` | `-42 / 25 / 10` | gNB position [m] — the Town10 central intersection where the scenario converges |
| `--maxUes` | `4` (mode 2), `32` (modes 3/4) | UE pool size ≥ number of connected actors (4 CAVs; +26 VRUs in modes 3/4) |
| `--listenPort` | `5555` | control UDP port |
| `--timeoutMs` | `500` | per-packet delivery timeout (a `timeout` reply = packet lost on the radio link) |
| `--drainMs` | `20` | keep < the CARLA tick (50 ms) so deliveries resolve within the tick |
| `--csvSend/--csvRecv` | — | bridge-side per-packet CSV logs |
| `--verbose` | — | per-packet bridge prints |

---

## 6. Inputs and outputs

**Inputs**: `CA/ca_convoy.yaml` — CARLA world (20 Hz sync mode), scenario
(CAV spawn points, 10 background vehicles, 6 bikes, 20 pedestrians), sensor
rig, YOLO/fusion/tracking parameters, and the collision thresholds
(`t2c_th` 4.8 s, `s2c_th` 4.2 m, `alpha_th_deg` 17°) shared by the local and
the edge CA. See `CA/PIPELINE.md` for every parameter.

**Outputs** (`CA/output/run_<timestamp>/`):

| File | Modes | Content |
|---|---|---|
| `metrics.csv` | all | One line per step per CAV per identified object: estimates, matched ground truth + errors, **local CA** output (`t2c`, `s2c`, `ca_warning`, `ca_type`), per-step timing split, and the **edge CA** columns `edge_warnings` (count received since that CAV's previous row — warnings surface one step after the edge processes the uplink, network latency included), `edge_warn_type`, `edge_warn_t2c`, `edge_warn_s2c`, `edge_warn_latency_ms` (empty in mode 1) |
| `gt.csv` | all | Ground truth of every dynamic object at every step |
| `summary.txt` | all | Error/timing statistics per CAV and category; modes 2/3 add the V2X section: packets sent/delivered/timeout + per-kind latency (detection / vam / warning), edge LDM size, risks confirmed, warnings sent/received per CAV, VAMs per VRU by triggering condition |
| `timing.csv` | all | Dedicated per-step per-CAV **wall-clock timing** log: `step, cav, t_detect_ms (YOLO inference), t_fuse_ms, t_track_ms, t_ca_ms (local CA)` — consumed by the latency analysis (§6.1) |
| `v2x_packets.csv` | 2/3 | One line per simulated packet: `packet_id, kind (detection/vam/warning), sender, receiver, size_bytes, tx_step, tx_sim_t, status (delivered/timeout/error/no_reply), latency_ms` |
| `edge_warnings.csv` | 2/3 | One line per warning **delivered** to a CAV: `rx_step, sim_t, cav, other_station, other_type, collision_type, t2c, s2c, latency_ms (warning downlink), packet_id`, plus the **latency chain** of the warning: `uplink_kind (detection/vam), uplink_packet_id, uplink_latency_ms, uplink_tx_step, uplink_sender` (the uplink packet whose ingest at the edge triggered it) and `edge_ca_ms` (edge CA execution wall time) |
| `VBS_stats.csv` | 3 | One line per **VAM generated** by a VRU Basic Service (`VamStatsRecorder`, §3): `gen_time_s (simulation clock), sender_id, sender_name, station_type`, the position/kinematics **as carried by the message** (`latitude_deg, longitude_deg, x_m, y_m, speed_ms, heading_deg`, decoded back from the UPER bytes sent), `dt_last_vam_s` (time since the previous VAM of that VRU), `avg_period_vru_s` (average VAM periodicity of that VRU so far), `avg_period_all_s` (average periodicity pooled over all the VRUs so far), `trigger` (`TriggCond` name), `vam_count_vru, vam_count_all` |
| `edge_ca_timing.csv` | 2/3 | Dedicated wall-clock log of every **edge CA execution** (one row per LDM update processed by the `CollisionAvoidanceService`): `step, sim_t, trigger_kind, station_id, ca_ms, n_warnings` |
| `<Cav>/<camera>_<step>.png` | all | Annotated frames (red box = local CA warning) |
| bridge `--csvSend/--csvRecv` | 2/3 | Bridge-side logs (join on `packet_id` with `v2x_packets.csv`): tx/rx sim time, latency, status |

Reference throughput (8 s mode-3 run, debug ns-3 build): ~1000 simulated
packets, uplink detection ≈ 4.7 ms, VAM ≈ 5 ms, warning downlink ≈ 1.3–3 ms.

### 6.1 Post-run metric analysis (`CA/analyze_ca_metrics.py`)

A dedicated script computes the aggregate metrics (with **95% confidence
intervals**) from the log files of one run directory — no re-simulation
needed, and it also works on older runs (metrics 1–2 only need
`gt.csv` + `metrics.csv`):

```bash
cd OpenCDA
python CA/analyze_ca_metrics.py CA/output/run_<timestamp> \
    [--vru-distance 30] [--sensing-ms 50] [--actuation-ms 100]
```

The report is printed and saved to `<run_dir>/analysis.txt`. Metrics:

1. **Detected VRUs** — average number of VRUs (pedestrians + bikes)
   detected *per CAV* among those within `--vru-distance` metres of the CAV
   (ground-truth distance from `gt.csv`). One sample per (step, CAV); a VRU
   counts as detected when a confirmed track of that CAV is matched to it in
   `metrics.csv` at that step. Mean ± 95% CI, overall and per CAV (the mean
   number of VRUs actually in range is reported for context).

2. **Misdetection probability** — probability that a VRU within
   `--vru-distance` of a CAV is *not* detected by it: one Bernoulli sample
   per (step, CAV, VRU-in-range) built from `gt.csv` (who should be
   detectable) vs `metrics.csv` (who was matched). 95% **Wilson score**
   interval, overall, per category (pedestrian/bike) and per CAV. Note that
   occlusions count as misdetections: the threshold is a distance criterion,
   not a visibility one.

3. **Total warning latency** (modes 2/3) — for every warning delivered to a
   CAV (`edge_warnings.csv`), the end-to-end chain is reconstructed and
   averaged (mean ± 95% CI, plus the per-component breakdown and the split
   by triggering uplink kind):

   | Component | Source |
   |---|---|
   | sensing period | `--sensing-ms` (default 50 ms = one 20 Hz frame period) |
   | perception inference | `timing.csv` at the warning's `uplink_tx_step`/`uplink_sender`: YOLO + fusion + tracking wall time (0 for VAM-triggered warnings — the VRU state is self-knowledge, not perception) |
   | detection/VAM uplink | `uplink_latency_ms` (simulated 5G, ns-3) |
   | edge CA execution | `edge_ca_ms` (wall clock at the edge) |
   | warning downlink | `latency_ms` (simulated 5G, ns-3) |
   | actuation time | `--actuation-ms`, modelled constant. Default **100 ms** (the comparability campaign value); ~**300 ms** ≈ brake-system pressure build-up of an automated emergency brake (AEB literature / UN R152 report ~0.2–0.3 s), or ~**1200 ms** to model a human driver reacting to an HMI warning |

   For comparison the **decentralized equivalent** is also computed from the
   local CA warnings of `metrics.csv` (sensing + perception + local CA +
   actuation, no network component).

---

## 7. Troubleshooting

- **`[NS3] WARNING: ... no reply ... is the v2x-bridge running?`** — start
  the bridge before the application, and check `--bridge-host/--bridge-port`
  vs `--listenPort`.
- **`ue_pool_exhausted` on the bridge stderr** — raise `--maxUes` (≥ 4 in
  mode 2, ≥ 30 in modes 3/4).
- **A few `no_reply` rows at step 0** — a debug ns-3 build can need several
  wall seconds to process the first 30-UE burst; harmless (the run
  continues), or build an optimized ns-3 profile.
- **`timeout` status** — a real simulated radio loss (RLC UM: HARQ
  exhaustion is a permanent loss); it is data, not an error.
- **Warnings look "late" in `metrics.csv`** — by design: the edge receives
  the uplink of step N, and its warning is delivered on the simulated
  downlink and recorded on the rows of step N+1 (`edge_warnings.csv` has the
  exact reception step/time).
- **VAM encode/decode errors** — check `asn1tools` is installed and the two
  `.asn` files are in this folder (or set `VAM_ASN_DIR`).
- **Restart the bridge between runs** — the UE mapping and clock epoch are
  per run.
