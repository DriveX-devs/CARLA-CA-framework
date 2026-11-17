# python_vru_service

The V2X side of the CA framework in Python: a simplified **LDM** (Local
Dynamic Map, with object matching), the client of the ns-3 **drivex-bridge**,
the **edge collision-avoidance server** and the glue to the CA main loop.

The **VRU Basic Service** (VAM management, ETSI TS 103 300-3) is no longer
implemented here: it is DriveX-sim's, and it runs inside ns-3
(`drivex-bridge --vbs=1`). This package tells the bridge where each connected
VRU is, and receives what the edge decoded. `vam_codec.py` remains as an
independent UPER codec (asn1tools), used to read a VAM outside the simulator
and to cross-check the one DriveX encodes (`tests/test_bridge_vam.py`).

## Requirements

```bash
pip install asn1tools geopy
```

The ASN.1 files (`ETSI-ITS-CDD.asn`, `VAM-PDU-Descriptions.asn`) ship in this
folder; a different folder can be set through `VAM_ASN_DIR`.

## Content

| File | Description |
|------|-------------|
| `ldm.py` | Simplified LDM (dict-based object database, object matching, state fusion) |
| `ca_v2x.py` | In-process V2X layer of `run_ca_extended.py`: per-CAV local LDMs fed by the perception tracks, the connected VRUs (entity state for their VRU Basic Service in ns-3, `VBS_stats.csv`), and the edge server (LDM + PyCA centralized collision avoidance), with every CAV→edge upload, VRU→edge VAM and edge→CAV warning simulated by the bridge |
| `ns3_bridge_client.py` | UDP client of the drivex-bridge: one simulated packet per datagram, unique `packet_id`s, CARLA simulation time as the bridge clock, application payloads base64-encoded inside the simulated packets, and the messages the bridge originates (`vam_tx`, VAM / beacon replies) |
| `vam_codec.py` | asn1tools-based VAM UPER codec + ETSI unit conversions |
| `geo_utils.py` | x/y <-> lat/lon conversions (geopy) |
| `tests/` | LDM unit tests; bridge VAM cross-check (needs a built bridge) |
| `examples/` | LDM on a sensorized vehicle and on an edge server |
| `USER_GUIDE.md` | interface formats of the bridge protocol as this package uses it |

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


## VAMs: DriveX's VRU Basic Service in ns-3

In application modes 3 and 4 the VRUs are connected. Every tick `ca_v2x.py`
sends the bridge one entity per VRU carrying, besides the position used for
the radio geometry, what the service needs: station id and ETSI station type,
geographic position, compass heading, speed and longitudinal acceleration
(the VRU's own GNSS/IMU: self-knowledge, not perception). The bridge runs one
`VRUBasicService` per VRU on that state:

* the service checks the ETSI triggering conditions every 100 ms of
  simulated time (heading, 4 m position change, 0.5 m/s speed change, or
  `T_GenVam` elapsed) and encodes a VAM when one holds;
* BTP and GeoNetworking wrap it, and the PDU travels to the edge as a bridge
  packet of type `vam` over the access technology of the run;
* the edge's DriveX stack decodes it, and the reply of that packet carries the
  decoded fields, which `EdgeCAServer.on_vam_delivered` stores in the edge
  LDM as a connected station.

`VBS_stats.csv` has one row per generated VAM (`vam_tx` messages). Its
`trigger_condition` is the bridge's reading of why the VAM was due — the
first condition, in the order the service checks them, that the state crossed
since the previous VAM — because the service does not expose its own.

`Ns3BridgeClient.send_vbs_control(t, origin_id, t_gen_vam_ms, duration_ms)`
changes `T_GenVam` of one VRU (e.g. 100 ms for one second): the hook for an
application that asks a VRU for more VAMs.

## Tests and examples

```bash
python -m pytest python_vru_service/tests/test_ldm.py
python -m pytest python_vru_service/tests/test_bridge_vam.py   # starts a bridge
python python_vru_service/examples/example_edge_server.py
```
