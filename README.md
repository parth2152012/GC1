# UVAX-1: Deep Scan Protocol

Rust/Tokio controller for 1–20 ArduCopter SITL vehicles, with autonomous POI allocation and UDP relay forwarding. This is a simulator prototype; it is not a certified implementation of every sample-scenario constraint.

## Run

Requires Rust, Python 3, ArduPilot SITL, and MAVProxy. The graphical launcher also needs `qterminal` or `x-terminal-emulator`.

```sh
python3 deploy_swarm.py
```

Choose a count from 1 to 20. The launcher passes that count to the controller. To start the controller separately with existing simulators:

```sh
GC1_DRONE_COUNT=20 cargo run
```

Ports for vehicle ID `id`:

- Telemetry: UDP `14550 + 10*(id-1)`.
- MAVProxy control: TCP `14600 + (id-1)`.
- Radio relay endpoint: UDP `16000 + id`; operational center: UDP `16000`.

Telemetry freshness, command serialization, battery state, flight jobs, POI ownership, and return-to-launch state are maintained per vehicle. Takeoff/command acknowledgement waits run independently so one drone cannot block the fleet loop. Setup failures release POIs for retry. Guided navigation requests a 4.5 m/s speed setting, but the live run still exceeded 5 m/s; this is not a verified speed cap.

## Operator commands

```text
drone 20
mode guided
arm throttle
takeoff 15
poi 40 20 15
status
fail battery 20 10
fail link
recover
```

`drone <id>` selects the vehicle for mode, arm, takeoff and POI commands; the default is Drone 1. `status` prints the whole configured fleet. Manual POI offsets are relative to the selected vehicle's current position. Targets are rejected while that vehicle has a flight setup job or is returning. `recover` clears link and battery fault overrides. `snap`/`image` still requests a frame from `camera.py`.

## Actual packet hops in SITL

`src/radio.rs` binds one UDP socket per vehicle and one for the center. The source computes a route from fresh telemetry. Every intermediate node receives and forwards the packet through its own socket. Every incoming and outgoing hop must be at most 100 m; positions older than three seconds are excluded. A missing route fails delivery instead of bypassing the range restriction through localhost. The center returns an acknowledgement over the reverse path; each report attempt has a ten-second timeout. `fail link` stops forwarding.

The mission allocator deploys available vehicles along approximately 75 m relay corridors and assigns an endpoint surveyor. Arrival alone does not complete a POI: the center must acknowledge its report. The fleet retries reports when routes are unavailable. The old self-addressed mesh heartbeat is no longer started.

This is actual UDP forwarding with simulated radio reachability, not physical RF. All endpoints run as separate async tasks in the controller process. Flight-control MAVLink and telemetry still use dedicated direct MAVProxy connections; POI reports use the multi-hop path. No claim is made that commands or images traverse this mesh.

## Tests

```sh
cargo test --offline
cargo build --offline
python3 scripts/test_sitl_scenario.py --drones 20 --seconds 120
```

Socket tests require local networking permission. They check twenty-hop delivery and that removing an essential relay prevents delivery until it returns. The headless harness requires `pymavlink`, `mavproxy.py`, and a built SITL under `~/ardupilot` (override using `--ardupilot`). It starts only its own simulator processes and writes raw telemetry, controller logs, and results to the printed `/tmp/gc1-sitl-*` directory before stopping those processes.

## Remaining scenario limits

- Relay corridor planning is simple; it does not guarantee collision-free trajectories or repair every failed airborne relay. Launch spacing remains 5 m, and the 20 m separation requirement is not guaranteed.
- Small fleets may lack enough vehicles for a long relay corridor; such POIs wait for capacity.
- A disconnected report can exceed ten seconds from first detection even though individual attempts time out at ten seconds.
- RTL is requested with a distance-based return margin before flight/mission limits; full 20-minute endurance and 45-minute landing compliance still require a complete scenario test.
- The arena is centered on Drone 1's origin; the sample diagram's operational-center offset is not modeled.
- Historical baseline results in `reports/sitl_10_drone_results.*` describe the earlier two-node controller, not this fleet implementation.
