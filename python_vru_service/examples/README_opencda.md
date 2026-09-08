# example_opencda.py — V2X simulation tool for OpenCDA/CARLA scenarios

`example_opencda.py` is a standalone Python tool that attaches to a CARLA
server **already executed and ticked by OpenCDA** (reference fork:
[DriveX-devs/OpenCDA](https://github.com/DriveX-devs/OpenCDA)) and adds a V2X
layer on top of the running scenario, using the modules of
`python_vru_service` (LDM + VRU Basic Service).

It is a *passive* CARLA client: it never calls `world.tick()` (OpenCDA owns
the simulation clock) and never spawns actors. It follows the simulation with
`world.wait_for_tick()` and connects to the actors that OpenCDA has already
spawned (CAVs, background traffic with `role_name` `autopilot`, bicycles with
`role_name` `bike`, AI pedestrian walkers), re-scanning periodically so that
actors spawned later are picked up as well.

## What it simulates

| Role | Assigned to | Behavior |
|------|-------------|----------|
| **Sensorized CAV** | every four-wheeler that is not background traffic | Keeps a local LDM. The detections filling it **never come from CARLA ground truth: they always come from the OpenCDA CA application** (`OpenCDA/CA`: YOLOv8 detection + camera/LiDAR frustum fusion + Kalman tracking), which streams its confirmed tracks to this tool over a localhost UDP JSON feed (`--detections-port`, see below). A callback prints every successful insert/update — in the future it will become a localhost socket call towards **ns-3** to simulate the network. Detections are also forwarded to the edge server (ideal V2I link). |
| **VRU** | pedestrian walkers and two-wheelers (bicycles, motorbikes) | Runs its own **VRU Basic Service**: VAMs are UPER-encoded (asn1tools) and transmitted according to the ETSI TS 103 300-3 triggering conditions, stepped on the **simulation clock** (not the wall clock). The VRU reads its own kinematic state from its CARLA actor — this is legitimate self-knowledge (the GNSS/IMU of the VRU device), not perception. Enabled/disabled with `--vams/--no-vams`. VAMs are delivered to the edge server. |
| **Edge server** (single) | — | An LDM with **object matching** enabled (multiple CAVs detecting the same object produce a single entry), fed with the CAV detections and the received VAMs. On every successful update, its callback prints the inserted object and its distance from all the other LDM objects; when a distance drops below `--warning-distance`, a `"control"` **JSON warning packet** is prepared and printed for *both* objects involved (see below). |

## The CA application detection feed

The CAV detections are produced by the Cooperative Awareness application in
`OpenCDA/CA` (`run_ca.py`: one YOLOv8 model per CAV over its 4 cameras,
camera–LiDAR frustum fusion, constant-velocity Kalman tracking producing
position, velocity and heading per confirmed track). The application streams
its confirmed tracks, per CAV and per tick, as one JSON datagram over
localhost UDP to this tool (`--detections-port`, default `47500`):

```json
{
  "cav": "Cav1",
  "cav_actor_id": 123,
  "t": 12.35,
  "tracks": [
    {"track_id": 7, "category": "pedestrian",
     "x": -41.2, "y": 24.7, "z": 0.9,
     "speed": 1.35, "heading": -167.2}
  ]
}
```

- `category` is the CA tracker category (`vehicle` | `bike` | `pedestrian`),
  mapped by the tool to the ETSI station types;
- `x/y/z` are CARLA world coordinates (the tool converts them to its ENU
  frame); `heading` is the CARLA-yaw-convention heading, or `null` while the
  track's heading is not yet valid (young or near-stationary track);
- `track_id` is the CAV-local track id; the tool namespaces it per CAV
  (`<CAV actor id> * 100000 + track_id`) so entries stay unique at the edge,
  where the LDM object matching merges the tracks of different CAVs that
  correspond to the same physical object.

### Required patch in `run_ca.py`

`run_ca.py` does not stream its tracks out of the box. Add once, near the top:

```python
import json, socket
V2X_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
V2X_ADDR = ("127.0.0.1", 47500)          # = --detections-port of the tool
```

and, inside the per-CAV main loop, right after
`tracks = cav['tracker'].step(merged)`:

```python
V2X_SOCK.sendto(json.dumps({
    "cav": cav['name'],
    "cav_actor_id": cav['vehicle'].id,
    "t": world.get_snapshot().timestamp.elapsed_seconds,
    "tracks": [{
        "track_id": int(t.track_id),
        "category": t.category,            # vehicle | bike | pedestrian
        "x": float(t.position[0]),         # CARLA world coordinates [m]
        "y": float(t.position[1]),
        "z": float(t.position[2]),
        "speed": float(t.speed),           # [m/s]
        "heading": float(t.heading) if t.heading_valid else None,
    } for t in tracks],                    # heading: CARLA yaw [deg]
}).encode(), V2X_ADDR)
```

(The same block is included as a comment at the top of
`example_opencda.py`.)

## Requirements

- The Python environment used to run the tool must have:
  - `asn1tools` and `geopy` (`pip install asn1tools geopy`);
  - the `carla` Python package **matching the server version** used by OpenCDA
    (e.g. `pip install carla==0.9.12`). The simplest option is to run the tool
    inside the same conda environment used by OpenCDA (e.g. `msvan3t_carla`),
    after installing `asn1tools`/`geopy` in it.
- The ASN.1 files `ETSI-ITS-CDD.asn` and `VAM-PDU-Descriptions.asn` must be in
  the ns-3-dev root (they are looked up automatically in the parent folder of
  `python_vru_service`; a different folder can be set with the `VAM_ASN_DIR`
  environment variable).
- No CARLA/OpenCDA at all is needed to *try* the tool: see the mock mode below.

## Step-by-step user guide

### 1. Start the CARLA server

```bash
cd <CARLA_ROOT>
./CarlaUE4.sh -prefernvidia          # add -RenderOffScreen for headless runs
```

By default CARLA listens on RPC port 2000.

### 2. Launch the OpenCDA CA application (patched)

In a second terminal, start the CA scenario, after applying the `run_ca.py`
patch above (it spawns the CAVs, the background traffic, the bikes and the
pedestrians, ticks the simulation, and runs YOLO + fusion + tracking per CAV):

```bash
conda activate msvan3t_carla
cd OpenCDA
python CA/run_ca.py --port 2000            # plus your usual CA options
```

Wait until the CAVs are spawned and the per-tick processing loop has started
(the simulation must be ticking).

### 3. Launch the V2X tool

In a third terminal:

```bash
conda activate msvan3t_carla         # or any env with carla+asn1tools+geopy
cd <ns-3-dev>/python_vru_service
python3 examples/example_opencda.py --host localhost --port 2000
```

Useful variants:

```bash
# quieter output: only attach lines and proximity warnings
python3 examples/example_opencda.py --quiet

# custom warning threshold and CA feed port
python3 examples/example_opencda.py --warning-distance 15 --detections-port 47500

# disable VAM transmission by the VRUs (CAV detections + edge server only)
python3 examples/example_opencda.py --no-vams

# stop automatically after 60 simulated seconds
python3 examples/example_opencda.py --duration 60
```

The tool exits by itself when OpenCDA stops ticking (no tick received for
`--tick-timeout` seconds, 10 by default), when `--duration` simulated seconds
have elapsed, or with Ctrl-C.

### 4. Read the output

On startup the tool lists the actors it is tracking; the CAV feeds are
announced when their first datagram arrives:

```text
[TOOL] listening for CA application detections on udp://127.0.0.1:47500
[TOOL] tracking CAV 123 (vehicle.lincoln.mkz_2017, role 'Cav1') - detections expected from the CA application feed
[TOOL] attached to VRU 141 (walker.pedestrian.0001, pedestrian)
[TOOL] running (waiting for OpenCDA ticks)...
[TOOL] CA detection feed active for Cav1 (actor 123)
```

Then, for every simulation tick:

- `[CAV <name> LDM] object ...` — a CAV inserted/updated in its local LDM an
  object detected by the CA application (YOLO + fusion + tracking);
- `[VRU <id>] VAM TX trigger=... <hex>` — a VRU transmitted a VAM (with the
  ETSI triggering condition and the UPER-encoded bytes);
- `[EDGE LDM] station <id> (...) | distances: ...` — the edge server LDM was
  updated; the line shows the inserted object and its distance from every other
  object in the edge LDM (`det.by [11, 12]` marks a detection merged by the
  object matching; `(VAM)` marks a connected VRU);
- `[EDGE] WARNING ...` followed by two JSON packets — two objects are closer
  than `--warning-distance`:

```json
{
  "msg_type": "control",
  "timestamp": 3.45,
  "entities": [
    {
      "timestamp": 3.45,
      "origin_ID": "11",
      "origin_vehicle_type": "cav",
      "Position": {"x_m": -12.4, "y_m": 0.0, "z_m": 0.0},
      "Velocity": 8.0,
      "Heading": 90.0
    }
  ],
  "packet": {
    "sender": "BS",
    "receiver": "21",
    "size_bytes": 200,
    "packet_id": 1,
    "type": "warning",
    "request_reply": false
  }
}
```

`entities` contains the current position of **all** the spawned objects (CAVs
and VRUs, in CARLA coordinates); one packet is printed for each of the two
objects involved (`receiver` field). A per-pair cooldown (`--warning-cooldown`,
2 s by default) avoids repeating the same warning at every tick.

### 5. (Optional) Dry run without CARLA/OpenCDA

```bash
python3 examples/example_opencda.py --mock            # full output
python3 examples/example_opencda.py --mock --quiet    # warnings only
```

The mock reproduces a small OpenCDA-like scene (2 CAVs approaching a crossing
pedestrian, 1 background vehicle that must be ignored, 1 bike) **plus a mock
of the CA application** that emulates the `run_ca.py` output (per-CAV tracks
with fusion-like position noise and CA-style heading validity) and streams it
over the same localhost UDP JSON interface — so the tool exercises exactly the
same receive path used with the real application.

## ns-3 integration points

The tool currently uses ideal links (direct function calls / prints). The four
places to modify to put the **ns-3** simulated network in the loop are marked
in `example_opencda.py` with `NS-3 INTEGRATION POINT (n/4)` comment banners:

1. **CAV local LDM callback** (`CavManager._print_callback`) — replace the
   print with a localhost socket call towards ns-3, so the CAV's V2X messages
   (e.g. CPMs) are generated inside the simulated network;
2. **CAV → edge detection upload** (`CavManager.on_tracks`) — replace the
   direct `EdgeServer.on_detection()` call with a transmission through ns-3
   (CAV node → base station/edge node), delivering the detection only when
   ns-3 reports the packet as received;
3. **VRU VAM transmission** (`VruManager._on_tx`) — send the UPER-encoded VAM
   bytes into ns-3 instead of calling `EdgeServer.on_vam()` directly;
4. **Edge warning downlink** (`EdgeServer._maybe_warn`) — send the `"control"`
   JSON packet to ns-3, which simulates the downlink transmission from the
   base station (`"sender": "BS"`) to the `receiver` entity with the given
   `size_bytes`.

## Command-line reference

| Option | Default | Description |
|--------|---------|-------------|
| `--host`, `--port` | `localhost`, `2000` | CARLA server executed by OpenCDA |
| `--mock` | off | Use the built-in mock world + mock CA feed (no CARLA needed) |
| `--duration` | `0` | Simulated seconds to run (0 = until OpenCDA stops; the mock defaults to 8 s) |
| `--detections-port` | `47500` | Localhost UDP port on which the CA application streams the CAV detections |
| `--warning-distance` | `10` | Edge-server proximity warning threshold [m] |
| `--warning-cooldown` | `2` | Minimum simulated seconds between two warnings for the same pair |
| `--vams` / `--no-vams` | enabled | Enable/disable VAM transmission by the VRUs |
| `--cav-role-names` | empty | Comma-separated `role_name` list identifying the CAVs (empty = every four-wheeler that is not background traffic; `run_ca.py` CAVs carry their name, e.g. `Cav1`, as `role_name`) |
| `--background-role-names` | `autopilot` | `role_name` list of the background traffic to ignore |
| `--cav-ids` | empty | Comma-separated CARLA actor ids to force as CAVs |
| `--rescan-every` | `20` | Re-discover the CARLA actors every N ticks |
| `--tick-timeout` | `10` | Seconds without ticks after which the tool exits |
| `--quiet` | off | Suppress per-update LDM/VAM prints (warnings always shown) |

## How actors are classified

1. `walker.pedestrian.*` → **pedestrian** (VRU);
2. two-wheelers (`number_of_wheels == 2`, the known CARLA bicycle blueprints
   `vehicle.bh.crossbike` / `vehicle.diamondback.century` /
   `vehicle.gazelle.omafiets`, or `role_name == bike`) → **bicyclist /
   motorcyclist** (VRU);
3. remaining vehicles: **CAV**, unless their `role_name` is in
   `--background-role-names` (both `run_ca.py` and the dataset generator mark
   the background traffic with `autopilot`; `run_ca.py` CAVs carry their
   configured name, e.g. `Cav1`, as `role_name`). If your scenario uses
   different conventions, pass `--cav-role-names` (only those roles become
   CAVs) or pin exact actors with `--cav-ids`.

Note that the CAV classification only selects which actors appear in the
warning `entities` list: the actual detections always arrive through the CA
application feed, which identifies the sending CAV by `cav_actor_id`.

## Notes and troubleshooting

- **No `[CAV ... LDM]` lines**: the CA application feed is not arriving —
  check that `run_ca.py` has been patched (see above), that it is running,
  and that the port matches `--detections-port`.
- **No actors attached**: check the `role_name` conventions of your scenario
  (see above); `--background-role-names ""` disables the background filter
  entirely.
- **"no tick received ... exiting"**: OpenCDA is not running/ticking yet, or it
  finished. Start the tool while the scenario loop is active, or raise
  `--tick-timeout`.
- **`carla` version errors**: the pip `carla` package must match the server
  version (`0.9.12` for the DriveX-devs OpenCDA fork).
- **Coordinates**: the tool works in an ENU frame (x = east, y = north)
  centered on the geo-origin of the CARLA map (`Map.transform_to_geolocation`),
  so the x/y decoded from the VAMs is consistent with the CA detections (which
  arrive in CARLA world coordinates and are converted on reception); the
  `Position` fields of the warning JSON are raw CARLA coordinates instead.
- **Duplicate warnings between representations of the same object**: entries
  closer than 2 m — or a detected + connected (VAM) pair of the same station
  type closer than 4.5 m, i.e. within the ETSI 4 m VAM position-update lag —
  are treated as the same physical object and never generate warnings between
  themselves.
