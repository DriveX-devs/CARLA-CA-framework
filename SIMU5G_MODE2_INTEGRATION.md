# CARLA-CA application mode 2 over Simu5G

This bundle connects the existing CARLA-CA V2X application flow to Simu5G.
The perception, LDM, edge collision-avoidance, and warning code remains in
`opencda_v2x.py`. Only its transport implementation is selected at runtime.
The existing ns-3/5G-LENA client and UDP protocol are not changed.

## File placement

| Bundle file | Target location |
|---|---|
| `run_ca_extended.py` | `OpenCDA/CA/run_ca_extended.py` |
| `simu5g_backend.py` | `OpenCDA/CA/simu5g_backend.py` |
| `simu5g_bridge_client.py` | `OpenCDA/CA/python_vru_service/simu5g_bridge_client.py` |
| `opencda_v2x.py` | `OpenCDA/CA/python_vru_service/opencda_v2x.py` |
| `CarlaCoordinator.cc/.h/.ned` | the corresponding files in the OMNeT++ project |
| `omnetpp.ini` | the Simu5G scenario configuration |

The remaining bundled files are included for completeness and are unchanged
by this milestone.

## Runtime flow

1. A CAV manager in `V2XLayer` creates its normal detection/ego update.
2. `Simu5gBridgeClient.send_packet()` queues the application payload and its
   simulated byte size.
3. `Simu5gBackend.complete_step()` includes the queued messages in the single
   lockstep `STATE` response.
4. `CarlaCoordinator` resolves a source application and destination host, then
   injects an INET packet through Simu5G.
5. The receiving `ExternalUdpApp` reports delivery to `CarlaCoordinator`.
6. The next `STEP` request returns the delivery and original payload to
   Python. The adapter invokes the same callback used by `Ns3BridgeClient`.
7. At `BS`, that callback updates the edge LDM and runs PyCA. Any warning is
   queued for Simu5G downlink delivery to the affected CAV.

`BS` is a static application endpoint on `server.app[0]`; it is not a CARLA
actor and does not consume one of the mobile UE slots.

## Configuration checks

- `*.numUe`, `*.coordinator.numActors`, and the number of CAVs in the OpenCDA
  scenario must be equal. This bundle is configured for four CAVs.
- `*.server.numApps` must be `1`, with `server.app[0].actorName = "BS"`.
- The OMNeT++ duration and coordinator step must match OpenCDA's duration and
  fixed delta. The supplied values are 40 seconds and 0.05 seconds.
- Protocol version is `3` in both `CarlaCoordinator.cc` and
  `simu5g_backend.py`.
- Simu5G currently supports application modes 1 and 2 in this topology.
  Modes 3 and 4 need additional UE/application mappings for every pedestrian
  and bicycle.

## Build and run

Rebuild the OMNeT++ project after replacing the C++/NED files. Then use this
startup order:

1. Start the CARLA server.
2. From the OpenCDA root, start application mode 2:

   ```bash
   python CA/run_ca_extended.py \
       --app-mode 2 \
       --network-backend simu5g \
       --seconds 40
   ```

3. Wait until OpenCDA prints `waiting for Simu5G on tcp://127.0.0.1:5555`.
4. Start the OMNeT++ `OpenCda5gSmoke` configuration.

To re-test mobility and the original synthetic packet without the edge V2X
layer, use `--app-mode 1 --network-backend simu5g`.

To use the existing ns-3 behavior, use:

```bash
python CA/run_ca_extended.py \
    --app-mode 2 \
    --network-backend 5g-lena \
    --seconds 40
```

## Expected evidence

For a Simu5G mode-2 run, check:

- `simu5g_messages.csv`: OpenCDA generation and delivery events;
- `omnet_messages.csv`: Simu5G injection and reception events;
- `v2x_packets.csv`: application-level packet result and network latency;
- `edge_ldm_updates.csv`: CAV updates received at `BS`;
- `edge_risks.csv`: collision risks evaluated by edge PyCA;
- `edge_warnings.csv`: warnings delivered from `BS` to a CAV.

Message IDs should match across the first three logs. A detection upload must
appear as `CavN -> BS`; an edge warning must appear as `BS -> CavN`.
