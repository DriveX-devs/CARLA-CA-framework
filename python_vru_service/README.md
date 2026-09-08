# python_vru_service

Python porting of a simplified **LDM** (Local Dynamic Map) and of the **VRU Basic
Service** (VAM management, ETSI TS 103 300-3), based on the C++ implementation in
`src/automotive` (`model/Facilities/LDM.cc/.h`, `model/Facilities/VRUBasicService.cc/.h`).

VAMs are UPER encoded/decoded with **asn1tools**, using the ASN.1 specifications
already available in the ns-3-dev root folder (`ETSI-ITS-CDD.asn` and
`VAM-PDU-Descriptions.asn`). Cartesian (x/y) to WGS84 (lat/lon) conversions are
performed with **geopy** (geodesic model), with an internal fallback if geopy is
not installed.

## Requirements

```bash
pip3 install asn1tools geopy    # add --user --break-system-packages on Debian/Ubuntu
```

Python >= 3.8. The ASN.1 files are automatically located in the parent folder of
this package (the ns-3-dev root); a different folder can be set through the
`VAM_ASN_DIR` environment variable.

## Content

| File | Description |
|------|-------------|
| `ldm.py` | Simplified LDM (dict-based object database) |
| `vru_basic_service.py` | VRU Basic Service porting (VAM triggering + dissemination) |
| `vam_codec.py` | asn1tools-based VAM UPER codec + ETSI unit conversions |
| `geo_utils.py` | x/y <-> lat/lon conversions (geopy) |
| `ns3_bridge_client.py` | UDP client of the ns-3 `v2x-bridge` (simulated 5G NR network): one simulated packet per datagram, unique `packet_id`s, `request_reply` always true, CARLA simulation time as the bridge clock reference, application payload (VAM bytes / detection / warning JSON) carried base64-encoded inside the simulated packets |
| `opencda_v2x.py` | In-process V2X layer for `OpenCDA/CA/run_ca_extended.py`: per-CAV local LDMs fed by the CA perception tracks, per-VRU VRU Basic Services, and an edge server (LDM with object matching + the PyCA centralized Collision Avoidance from `OpenCDA/CA/PyCA`), with every CAV→edge detection upload, VRU→edge VAM and edge→CAV warning simulated by the ns-3 bridge |
| `tests/test_ldm.py`, `tests/test_vru_basic_service.py` | Unit tests (49 tests) |
| `examples/example.py` | End-to-end demo (TX pedestrian -> RX station with LDM) |
| `examples/example_sensorized_vehicle.py` | LDM only, fed with the detections of an external perception application |
| `examples/example_vru.py` | LDM + VRU Basic Service on board of a VRU (VAM transmission only) |
| `examples/example_edge_server.py` | Edge server LDM fed with vehicle detections and VAMs, with object matching |
| `examples/example_opencda.py` | V2X simulation tool attaching to an OpenCDA-driven CARLA server (CAV LDMs + VRU VAMs + edge server with proximity warnings) |

## LDM

The LDM stores, for each object: `station_id`, `station_type`, position (`x`/`y`
and/or `lat`/`lon`), `speed`, `heading` and, only if available, `acceleration`.

```python
from ldm import LDM, LDMError

ldm = LDM()

# Insertion / update (insert() updates an existing entry, like the C++ LDM::insert();
# update() fails with LDM_ITEM_NOT_FOUND if the entry does not exist)
ldm.insert(1000, 1, x=10.0, y=20.0, speed=1.2, heading=45.0)          # -> LDM_OK
ldm.insert(1000, 1, x=11.0, y=21.0, speed=1.4, heading=46.0)          # -> LDM_UPDATED

# Lookup by station ID
obj = ldm.lookup(1000)

# Lookup by circular geographical area
ldm.range_select(100.0, 45.0, 7.6)          # 100 m around (lat, lon)
ldm.range_select_xy(100.0, 0.0, 0.0)        # 100 m around cartesian (x, y)
ldm.range_select_by_station(100.0, 1000)    # 100 m around a stored station

# Removal
ldm.remove(1000)

# Callback on every successful insertion/update:
# callback(station_id, station_type, position, speed, heading, acceleration)
# position is an LDMPosition namedtuple: kind="xy" (c1=x, c2=y) or, only when
# x/y is not available, kind="latlon" (c1=lat, c2=lon); acceleration is None
# when not available.
ldm.register_callback(my_callback)

# Insertion of a VRU directly from a UPER-encoded VAM (asn1tools decoding):
err, station_id = ldm.add_vru_from_vam(vam_bytes)
```

### Detected objects and object matching

Objects detected by the sensors of a vehicle (as opposed to connected stations
transmitting their own messages) can be inserted with `detected=True` and
`perceived_by=<stationID of the detecting vehicle>`:

```python
ldm.insert(9001, 1, x=10.0, y=5.0, speed=1.4, heading=350.0,
           detected=True, perceived_by=100)
```

With the object matching logic enabled, detections of the same physical object
coming from **different** vehicles are merged into a single LDM entry (stored
under the stationID with which the object was first inserted, with
`perceived_by` accumulating the set of detecting vehicles):

```python
ldm.enable_object_matching(match_distance_m=2.0)   # optional, disabled by default
```

Two detections from different vehicles are considered the same object if their
positions are closer than `match_distance_m`; once matched, subsequent updates
from a vehicle keep following its own local object ID. Two objects reported by
the *same* vehicle under different IDs, and connected stations (VAM/CAM), are
never merged.

## VRU Basic Service

```python
from vru_basic_service import VRUBasicService, CHECK_MODE_PERIODIC, CHECK_MODE_TRIGGERED

srv = VRUBasicService(station_id=1000, ref_lat=45.0, ref_lon=7.6)
srv.set_tx_callback(lambda encoded_vam: ...)   # or srv.set_tx_udp(host, port)

# Kinematic status of the ego VRU, set by other services. The position can be
# given as x/y (converted to lat/lon through geopy, using the reference set by
# ref_lat/ref_lon or set_reference_position()) or directly as lat/lon.
# acceleration is optional (encoded as "unavailable" when not provided).
srv.set_ego_vru_kstatus(1000, 1, x=0.0, y=0.0, speed=1.2, heading=90.0,
                        acceleration=0.1)

srv.start_vam_dissemination()
...
vams_sent = srv.terminate_dissemination()
```

The service checks the VAM triggering conditions ported from
`VRUBasicService::checkVamConditions()`:

1. heading change > 4°,
2. position change > 4 m,
3. speed change > 0.5 m/s,
4. safe distances (proximity check through the LDM),
5. time elapsed since the last VAM >= `T_GenVam` (5 s),

plus the VAM redundancy mitigation of `checkVamRedundancyMitigation()`.
The thresholds are the ETSI TS 103 300-3 ones and can be tuned via the
`m_heading_threshold_deg`, `m_position_threshold_m`, `m_speed_threshold_ms`
attributes (note: the C++ code currently uses 10° instead of 4° for the heading).

### Check mode (global flag, periodic by default)

The conditions are checked either **periodically** every 100 ms
(`T_CheckVamGen`), or **when externally triggered** by a localhost UDP packet
containing the string `"check"` (port `48110` by default, configurable with the
`trigger_port` constructor parameter):

```python
import vru_basic_service
vru_basic_service.DEFAULT_CHECK_MODE = vru_basic_service.CHECK_MODE_TRIGGERED  # global flag
# or per instance:
srv = VRUBasicService(..., check_mode=CHECK_MODE_TRIGGERED, trigger_port=48110)
```

```bash
echo -n "check" | nc -u -w0 127.0.0.1 48110   # external trigger
```

### Proximity check (global flag, enabled by default)

If `vru_basic_service.PROXIMITY_CHECK_ENABLED` is `True` (default), the service
imports the LDM module (creating an internal `LDM` instance if none is passed
via the `ldm` constructor parameter) and performs the *SAFE DISTANCES* check of
the C++ code by reading the LDM content: a VAM is triggered when the nearest
vehicle/pedestrian is within the longitudinal (`|v| * 5 s`), lateral (2 m) and
vertical (5 m) safe distances. Received VAMs passed to `receive_vam()` feed the
LDM automatically (porting of `vLDM_handler()`).

### External time reference (OpenCDA/CARLA co-simulation)

By default the service uses the wall clock (`time.monotonic()`), which is wrong
in a synchronous OpenCDA/CARLA co-simulation, where the simulation time advances
by a fixed step at every `world.tick()` and can run faster or slower than real
time. The service therefore supports an **external time reference**, the Python
equivalent of the `real_time` flag of the C++ implementation:

```python
srv.set_timestamp_callback(callback)   # callback() -> current time in seconds (float)
```

When set, the elapsed-time condition (`T_GenVam`), the redundancy mitigation and
the VAM `generationDeltaTime` all follow the external clock. Combine it with
**manual stepping**: do not call `start_vam_dissemination()` (no internal thread,
no real-time sleeps); instead call `init_dissemination()` once and then
`check_vam_conditions()` after every simulation tick. The recommended pattern
inside OpenCDA (which is Python, so the service can be instantiated in-process,
e.g. inside a manager wrapping a CARLA walker) is:

```python
# Inside an OpenCDA scenario (CARLA synchronous mode)
import math
from vru_basic_service import VRUBasicService
import vam_codec

class VruVamManager:
    """Wraps a CARLA walker and manages its VAM dissemination, driven by the
    OpenCDA/CARLA simulation clock."""

    def __init__(self, world, walker, station_id):
        self.world = world
        self.walker = walker
        self.carla_map = world.get_map()
        self.srv = VRUBasicService(station_id=station_id)
        # Time reference: CARLA simulation time (seconds), NOT the wall clock
        self.srv.set_timestamp_callback(
            lambda: self.world.get_snapshot().timestamp.elapsed_seconds)
        self.srv.set_tx_callback(self._send_vam)
        self._update_kstatus()
        self.srv.init_dissemination()      # initial VAM (DISSEMINATION_START)

    def _send_vam(self, encoded_vam):
        # Deliver the UPER-encoded VAM: OpenCDA V2XManager, UDP socket, ...
        ...

    def _update_kstatus(self):
        # CARLA can provide lat/lon directly (preferred, no axis pitfalls)
        geo = self.carla_map.transform_to_geolocation(self.walker.get_location())
        vel = self.walker.get_velocity()
        yaw = self.walker.get_transform().rotation.yaw
        self.srv.set_ego_vru_kstatus(
            self.srv.m_station_id, vam_codec.STATION_TYPE_PEDESTRIAN,
            lat=geo.latitude, lon=geo.longitude,
            speed=math.hypot(vel.x, vel.y),
            # CARLA yaw: 0 = +x (east), clockwise -> compass heading from north
            heading=(yaw + 90.0) % 360.0)

    def step(self):
        """To be called after every world.tick() of the co-simulation loop."""
        self._update_kstatus()
        self.srv.check_vam_conditions()


# Co-simulation loop
managers = [VruVamManager(world, walker, 1000 + i)
            for i, walker in enumerate(walkers)]
while scenario_running:
    world.tick()                  # advances CARLA/OpenCDA by one time step
    for m in managers:
        m.step()                  # conditions checked on the simulation clock
```

Notes:

- Positions can also be passed as x/y, but remember that CARLA uses a
  left-handed frame (y pointing south): pass `x=loc.x, y=-loc.y` and set the
  reference position (`ref_lat`/`ref_lon`) to the map geo-origin so that the
  internal x/y -> lat/lon conversion is consistent.
- If the service runs in a **separate process** from OpenCDA, use the triggered
  mode instead of manual stepping: start it with
  `check_mode=CHECK_MODE_TRIGGERED` and `start_vam_dissemination()`, and after
  every `world.tick()` send the trigger from the OpenCDA side:
  `sock.sendto(b"check", ("127.0.0.1", 48110))`. In this case the simulation
  time must also cross the process boundary: send it along with the kinematic
  status and let the timestamp callback return the last received value —
  otherwise the elapsed-time condition falls back to the wall clock.
- `tests/test_vru_basic_service.py::TestExternalTimeReference` shows the full
  pattern driven by a fake clock (a VAM is triggered by advancing the simulated
  time only, with no real waiting).

### `examples/example_opencda.py`: V2X simulation tool on an OpenCDA scenario

> A dedicated README with the full description, a step-by-step user guide
> (launching CARLA, OpenCDA and the tool), the command-line reference and
> troubleshooting notes is available in
> [`examples/README_opencda.md`](examples/README_opencda.md).

`examples/example_opencda.py` is a complete tool that attaches to the CARLA
server **already executed and ticked by OpenCDA** (reference fork:
[DriveX-devs/OpenCDA](https://github.com/DriveX-devs/OpenCDA), e.g. while
`dataset_generator/generate_dataset.py` is running). It never calls
`world.tick()` and never spawns actors: it follows the simulation with
`world.wait_for_tick()` and connects to the already existing actors,
re-scanning periodically so that actors spawned later are picked up too.
Discovered actors get a V2X role:

1. **CAVs** (four-wheelers which are not background traffic — the background
   has `role_name` `autopilot`; the selection can be overridden with
   `--cav-role-names`, `--background-role-names` or `--cav-ids`): each keeps a
   local LDM. The detections filling it never come from CARLA ground truth:
   they always come from the **OpenCDA CA application** (`OpenCDA/CA`,
   YOLOv8 + camera/LiDAR fusion + Kalman tracking), which streams its
   confirmed tracks to the tool over a localhost UDP JSON feed
   (`--detections-port`, see the dedicated README for the small `run_ca.py`
   patch). A callback prints every successful insert/update (for now — it will
   become a localhost socket call towards ns-3 to simulate the network).
   Detections are also forwarded to the edge server.
2. **VRUs** (pedestrian walkers, bicycles such as `vehicle.bh.crossbike` /
   `role_name` `bike`, and other two-wheelers): each runs a VRU Basic Service
   transmitting VAMs on the ETSI triggering conditions, stepped on the
   simulation clock. VAM transmission is controlled by `--vams/--no-vams`
   (enabled by default).
3. **A single edge server**: an LDM with object matching enabled, fed with the
   CAV detections and the received VAMs. Its callback prints every inserted
   object together with the distance from all the other LDM objects; when a
   distance drops below `--warning-distance` (10 m by default, with a per-pair
   cooldown of `--warning-cooldown` seconds), it prepares and prints a
   `"control"` JSON packet for **both** objects involved:

   ```json
   {
     "msg_type": "control",
     "timestamp": 3.45,
     "entities": [
       {"timestamp": 3.45, "origin_ID": "11", "origin_vehicle_type": "cav",
        "Position": {"x_m": -12.4, "y_m": 0.0, "z_m": 0.0},
        "Velocity": 8.0, "Heading": 90.0}
     ],
     "packet": {"sender": "BS", "receiver": "21", "size_bytes": 200,
                "packet_id": 1, "type": "warning", "request_reply": false}
   }
   ```

   `entities` lists the current position of all the spawned objects (CAVs and
   VRUs). Entries closer than 2 m (or a detected + connected pair of the same
   type closer than 4.5 m, i.e. within the VAM position-update lag) are treated
   as the same physical object and do not generate warnings between themselves.

The four places where ns-3 will replace the current ideal links are marked in
the code with `NS-3 INTEGRATION POINT (n/4)` comment banners (CAV LDM
callback, CAV→edge detection upload, VRU VAM transmission, edge warning
downlink).

```bash
# with OpenCDA + the patched CA application running on localhost:2000
python3 examples/example_opencda.py --warning-distance 10

# without CARLA: built-in mock scene + mock CA application feed
python3 examples/example_opencda.py --mock [--quiet]
```

## ns-3 integration (`ns3_bridge_client.py` + `opencda_v2x.py`)

> **[USER_GUIDE.md](USER_GUIDE.md)** is the dedicated guide for the
> integrated simulation: all the components, the UDP interface formats
> (bridge protocol + application payloads), the step-by-step launch
> procedure for the four collision-avoidance cases, and the full
> command-line / output-metrics reference.

`OpenCDA/CA/run_ca_extended.py` puts everything together **in-process** (no
separate tool needed): the run_ca.py scenario + the modules of this package +
the ns-3 `v2x-bridge` (`ns-3-dev/scratch/v2x-bridge`, external-clock mode)
simulating the 5G NR network between the CAVs/VRUs and the edge server. Four
application modes (`--app-mode`):

1. **local** — each CAV runs the PyCA collision avoidance on its own
   perception output (identical to `run_ca.py`);
2. **edge** — no VAMs; each CAV keeps a local LDM and uploads its detections
   (with its ego state) to the edge server as simulated 5G packets; the edge
   LDM (object matching) + PyCA `CollisionAvoidanceService` (analytic
   t2c/s2c) generate warnings, sent back to the involved CAVs over the
   simulated downlink and logged in the run metrics when delivered;
3. **edge + VAMs** — like 2, plus every VRU (pedestrians, bikes) runs a VRU
   Basic Service transmitting UPER-encoded VAMs to the edge over 5G;
4. **VAMs only** — like 3, but the CAVs upload their ego state alone (no
   perceived object ever reaches the edge) and the edge pairs CAVs with VRUs
   only: what the centralized CA achieves from cooperative messages alone.

The application payloads (detection JSON, VAM bytes, warning JSON) are
carried base64-encoded **inside** the simulated packets (`packet.payload` in
the bridge protocol) and dispatched to the receiver-side logic only when the
bridge reports the delivery — so the actual information, not only the packet
size, traverses the simulated network.

```bash
# terminal 1 — the bridge (gNB at the Town10 central intersection):
cd <ns-3-root> && ./ns3 run "v2x-bridge --gnbX=-42 --gnbY=25 --maxUes=32"

# terminal 2 — the extended CA application:
conda activate msvan3t_carla && cd OpenCDA
python CA/run_ca_extended.py --app-mode 3 --seconds 40
```

## Tests and demo

```bash
cd python_vru_service
python3 -m unittest discover -s tests -v
python3 examples/example.py
```
