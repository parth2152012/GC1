# 🚁 UVAX-1: Deep Scan Protocol

UVAX-1 is an async Rust controller for a two-node ArduCopter SITL swarm. It receives MAVLink telemetry, autonomously assigns and dispatches PoIs, tracks swarm safety/battery state, performs a corrective collision-avoidance maneuver, logs standardized metrics, supports fault injection for testing, accepts operator commands, and reassembles + saves camera-image datagrams.

> This is a research/SITL project. Validate configuration and safety constraints before connecting it to a real aircraft.

## What works

- MAVLink telemetry listeners for Drone 1 (`UDP 14550`) and Drone 2 (`UDP 14560`).
- An operator CLI available as soon as the controller starts.
- **Autonomous multi-PoI mission planning**: 10 PoIs are spawned at random positions across the 45-minute mission window (the first two are released immediately after GPS lock so a demo does not sit at `0/0`; the remaining PoIs emerge at random times), prioritized (High/Medium/Low), and automatically assigned + flown to by whichever drone currently holds the `LeadSurveyor` role — including the reserve drone after a relay handoff.
- **Generalized flight control**: both Drone 1 and Drone 2 can be commanded over MAVLink (`send_guided_target_for`, etc.), not just Drone 1, so a promoted reserve is actually flown, not just relabeled.
- **Real collision-avoidance action**: a 20 m separation breach now triggers a corrective nudge command (not just a log warning), rate-limited to avoid command spam.
- **Measured metrics logging**: packet identities, send/receive timestamps, measured latency samples, mission progress, relay reallocations, recovery time, separation/link warnings, avoidance maneuvers, photo captures, mesh health, and fault injections are appended as JSON lines to `swarm_metrics.jsonl`.
- **Fault injection for testing**: `fail battery <1|2> <percent>` and `fail link` / `recover` let you simulate a UAV or comms failure and measure recovery time.
- GPS-gated `poi` commands, bounded to ±500 m north/east and 0–100 m relative altitude (matches the organizer's 100 m operational height cap).
- A bidirectional MAVLink control route to each drone's SITL TCP endpoint (5760 + 10×instance).
- On-demand camera snapshots: `snap` signals `camera.py` on `UDP 5002`; captured JPEG fragments are reassembled and **saved to `captures/frame_<id>.jpg`** (previously discarded).

## Requirements

- Rust toolchain (edition 2021).
- Python 3; `camera.py` additionally needs `opencv-python` and a working webcam.
- An ArduPilot checkout with `Tools/autotest/sim_vehicle.py`.
- A graphical terminal such as `qterminal` or `x-terminal-emulator` when using `deploy_swarm.py`.

## Start a local SITL swarm

`deploy_swarm.py` starts two visible ArduCopter SITL instances and runs the Rust controller in the current terminal. Update `ARDUPILOT_DIR` and `RUST_APP_DIR` at the top of the script if your checkouts are elsewhere.

```bash
cd /path/to/GC1
python3 deploy_swarm.py
```

The launcher prompts for the ArduPilot checkout (press Enter for `~/ardupilot`), the drone count, and an origin. Drones are placed 5 m apart north/south; their telemetry ports begin at UDP 14550 and increase by 10 per drone. The controller currently displays telemetry for the first two drones.

To also enable `snap`, start the camera producer in another terminal before running the controller:

```bash
python3 camera.py
```

Alternatively, `deploy_swarm.sh` starts `camera.py` automatically when it is run from the repository root.

## Operator CLI

```text
swarm-cli> help
Commands: mode guided | arm throttle | takeoff <alt_m> | poi <north_m> <east_m> <alt_m> | snap | status | fail battery <1|2> <percent> | fail link | recover | help

swarm-cli> status
--- FLEET TELEMETRY STATUS ---
🛸 Drone 1: Pos (19.076000, 72.877700) | Bat: 100%
🛸 Drone 2: Pos (19.076090, 72.877700) | Bat: 100%

swarm-cli> poi 200 85 15
🎯 Dispatched target: North 200m, East 85m (...)

swarm-cli> fail battery 1 12
💥 Injected battery fault on Drone 1 -> 12%

swarm-cli> snap
📸 TRIGGERING CAMERA SNAPSHOT CAPTURE...
```

### Command reference

| Command | Description |
| --- | --- |
| `poi <north_m> <east_m> <alt_m>` | Manually sends Drone 1 a global-relative-altitude MAVLink target. |
| `mode guided` | Sets Drone 1 to ArduCopter Guided mode. |
| `arm throttle` | Arms Drone 1. Complete pre-arm checks and use this only in a safe SITL/test environment. |
| `takeoff <alt_m>` | Commands Drone 1 to take off, 0–100 m relative altitude. |
| `status` | Prints the latest position and battery values from both telemetry streams. |
| `snap` / `image` | Requests one JPEG frame from `camera.py`, saved to `captures/`. |
| `fail battery <1\|2> <percent>` | Injects a battery fault on a drone, for testing reconfiguration/recovery-time measurement. |
| `fail link` | Simulates a mesh communications outage (dropped packets). |
| `recover` | Clears a simulated link outage. |
| `help` | Prints the command summary. |

Meanwhile, in the background, the orchestrator autonomously spawns PoIs, assigns them to whichever drone is currently `LeadSurveyor`, flies to them, marks them complete on arrival, and logs everything to `swarm_metrics.jsonl`.

## Metrics log

`swarm_metrics.jsonl` (one JSON object per line) is written next to the binary and contains a timestamped, typed event stream: `MissionStart`, `MissionProgress`, `PoiAssigned`, `PoiCompleted`, `RelayReallocation`, `RecoveryTime`, `SeparationBreach`, `LinkWarning`, `CollisionAvoidanceManeuver`, `PhotoCaptured`, `FaultInjected`, `MeshStatus`. This is the basis for filling in the official Evaluation & Performance Metrics table (mission completion rate/time, relay reallocations, recovery time, collision count, etc.) — aggregate it with a short offline script per run.

## Development checks

```bash
cargo fmt --check
cargo test
python3 -m py_compile camera.py deploy_swarm.py
```

Note: `cargo test` now also runs `mission.rs`'s test suite, which previously never compiled because the module wasn't declared in `main.rs`.

The controller enforces the 45-minute mission deadline: it stops spawning/dispatching PoIs and sends both SITL vehicles RTL when the deadline is reached.

## Known remaining limitations

- Still fundamentally a 2-node prototype (`node_active`/`node_reserve`); a true N-UAV swarm would need `Vec<SwarmNode>` and a generalized WaveManager.
- The mesh has no acknowledgement/retransmission or route-quality optimization. Its measured PDR/latency currently covers the controller's local heartbeat health probe; an end-to-end UAV-to-GCS PDR requires separate per-drone application traffic.
- The demo script now exercises battery handoff, link outage/recovery, optional camera capture, and final metrics aggregation; a full Stage-2 scenario harness is still not implemented.
- `discharge_rate_per_min` is still a fixed constant rather than measured from telemetry, so endurance estimates are approximate.

## License

Developed for academic research, competitive benchmarking, and participation in the PUSHPAK National Mission on Drone Technology challenge.
