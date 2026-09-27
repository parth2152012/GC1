# Ten-drone live SITL baseline — 2026-09-27

The 180-second run launched ten independent ArduCopter SITL instances through MAVProxy. All ten produced telemetry. Drone 1 completed both random POIs that spawned during the run and a manual operator POI, then resumed the interrupted automatic task. **This baseline does not satisfy the sample scenario.**

Source: `sample scenario for GC 1.pptx.pdf`, page 1. The scenario calls for a 45-minute mission, at most 20 minutes per flight, 100 m communication range, a 1000 m × 1000 m area, takeoff/landing at the operational center, at most 100 m altitude and 5 m/s speed, at least 20 m between vehicles, reporting within 10 seconds, and 10 randomly timed/positioned POIs.

| Check | Observed result |
|---|---|
| Simulator telemetry | 10/10 vehicles; about 675 position samples per vehicle |
| Fleet control | Only Drone 1 flew. Drone 2 remained reserve. Drones 3–10 are not represented in the production controller. |
| Automatic POIs | 2/2 spawned POIs completed; remaining 8 had not spawned within this short run |
| Manual commands | Guided/arm commands accepted; manual POI #3 completed; automatic POI #2 subsequently resumed and completed |
| Maximum speed ≤5 m/s | **FAIL:** measured 9.982 m/s from MAVLink velocity |
| Maximum altitude ≤100 m | No breach observed; maximum 15.092 m |
| Communication hops ≤100 m | **Geometric failure:** airborne Drone 1 became disconnected from the center even when all grounded drones were allowed as relay vertices |
| Minimum vehicle separation ≥20 m | Multiple-airborne separation not exercised. Production launch positions are only 5 m apart on the ground. |
| Arena | No breach of the controller's centered ±500 m geofence; PDF operational-center offset is not modeled |
| Flight duration ≤20 minutes | Not tested |
| All drones landed by 45 minutes | Not tested; disposable simulations stopped after the 3-minute audit |
| POI reporting within 10 seconds | Not verified; no per-POI end-to-end reporting acknowledgement was measured |

Radio results are based on measured positions and a 100 m adjacency graph. MAVLink control uses unrestricted localhost sockets, so successful command delivery does not demonstrate radio connectivity. Simulator default flight parameters and the controller's normal random spawn timing were retained. This run used the same five-metre-spaced launch layout as `deploy_swarm.py`; it was not a full recreation of the PDF diagram.

The scalability limitation is present in `src/gps.rs` (two telemetry stores/listeners and two control locks) and `src/main.rs` (one active node and one reserve node). Increasing the launcher count alone does not create a ten-node mission controller. Scenario readiness requires dynamic fleet state/task allocation, a verified speed cap, relay routing/planning, separation management, reporting measurements, and a full endurance/landing run.

Reproduce after `cargo build`:

```sh
python3 scripts/test_sitl_scenario.py --drones 10 --seconds 180
```

Raw logs and position samples for this run: `/tmp/gc1-sitl-10-de4svhcr/`. A durable summary is in `reports/sitl_10_drone_results.json`. The harness terminated and waited for every simulator, MAVProxy instance, and controller it launched; no existing user flight session was used.
