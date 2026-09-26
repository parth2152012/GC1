# 🚁 UVAX-1: Deep Scan Protocol

UVAX-1 is an async Rust controller for a two-node ArduCopter SITL swarm. It receives MAVLink telemetry, tracks basic swarm safety and battery state, accepts operator commands, and reassembles camera-image datagrams.

> This is a research/SITL project. Validate configuration and safety constraints before connecting it to a real aircraft.

## What works

- MAVLink telemetry listeners for Drone 1 (`UDP 14550`) and Drone 2 (`UDP 14560`).
- An operator CLI available as soon as the controller starts—**it no longer waits for both GPS locks** before accepting `help`, `status`, or camera commands.
- GPS-gated `poi` commands, bounded to ±500 m north/east and 0–120 m relative altitude.
- A bidirectional MAVLink control route to Drone 1's SITL TCP endpoint (`TCP 5760`). This replaces the old, incorrect attempt to send commands back to the one-way UDP telemetry output.
- On-demand camera snapshots: `snap` signals `camera.py` on `UDP 5002`; captured JPEG fragments arrive at the controller on `UDP 5001`.

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

Enter an origin when prompted, or press Enter to use the Mumbai default. The launcher supplies telemetry on UDP ports 14550 and 14560. Drone 1's normal SITL MAVLink TCP port 5760 is reserved for CLI control commands.

To also enable `snap`, start the camera producer in another terminal before running the controller:

```bash
python3 camera.py
```

Alternatively, `deploy_swarm.sh` starts `camera.py` automatically when it is run from the repository root.

## Operator CLI

The prompt appears immediately after the controller boots. `poi` remains unavailable until Drone 1 reports a non-zero global position, preventing invalid targets while SITL obtains GPS lock.

```text
swarm-cli> help
Commands: poi <north_m> <east_m> <alt_m> | snap | status | help

swarm-cli> status
--- FLEET TELEMETRY STATUS ---
🛸 Drone 1: Pos (19.076000, 72.877700) | Bat: 100%
🛸 Drone 2: Pos (19.076090, 72.877700) | Bat: 100%

swarm-cli> poi 200 85 15
🎯 Dispatched target: North 200m, East 85m (...)

swarm-cli> snap
📸 TRIGGERING CAMERA SNAPSHOT CAPTURE...
```

### Command reference

| Command | Description |
| --- | --- |
| `poi <north_m> <east_m> <alt_m>` | Sends Drone 1 a global-relative-altitude MAVLink target. The vehicle must be armed and in Guided mode; offsets are relative to Drone 1's current position. |
| `status` | Prints the latest position and battery values from both telemetry streams. |
| `snap` / `image` | Requests one JPEG frame from `camera.py`. The producer fragments it for the Rust listener. |
| `help` | Prints the command summary. |

To fly a waypoint in SITL, set Drone 1 to Guided mode and arm/take off in its MAVProxy console first:

```text
MAV> mode guided
MAV> arm throttle
MAV> takeoff 15
```

## Development checks

```bash
cargo fmt --check
cargo test
python3 -m py_compile camera.py deploy_swarm.py
```

## License

Developed for academic research, competitive benchmarking, and participation in the PUSHPAK National Mission on Drone Technology challenge.
