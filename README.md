```markdown
# 🚁 UVAX-1: Deep Scan Protocol
> **Next-Generation Async Drone Intrusion Detection System (IDS) & Swarm Orchestrator**

[![Rust](https://img.shields.io/badge/Rust-1.75%2B-orange?logo=rust)](https://www.rust-lang.org/)
[![Tokio](https://img.shields.io/badge/Async-Tokio-blue?logo=tokio)](https://tokio.rs/)
[![ArduPilot](https://img.shields.io/badge/SITL-ArduCopter-red?logo=ardupilot)](https://ardupilot.org/)
[![Challenge](https://img.shields.io/badge/PUSHPAK%20GC3-Security%20of%20Drones-green)](https://www.techfest.org/)

**UVAX-1** (Deep Scan Protocol) is an indigenous, high-throughput, sub-millisecond Drone Intrusion Detection System (Drone IDS) and multi-agent swarm manager[cite: 1]. Built completely in **async Rust (Tokio)**, UVAX-1 monitors high-frequency MAVLink streams (`GLOBAL_POSITION_INT`, `ATTITUDE`, `SYS_STATUS`), detects real-time cyber threats (command injection, coordinate spoofing, boundary breaches), and dynamically manages active/reserve drone state handoffs in GPS-denied or hostile environments[cite: 1, 2].

Designed specifically for the **IIT Bombay PUSHPAK Grand Challenge 3 (Security of Drones — Objective 2)**[cite: 1].

---

## 🏗️ System Architecture


```

```
                ┌──────────────────────────────────────────────┐
                │      Deployer Script (deploy_swarm.py)       │
                │   Prompts Lat/Lon → Calculates +10m North    │
                └──────────────────────┬───────────────────────┘
                                       │ Spawns Visible SITL Windows
               ┌───────────────────────┴───────────────────────┐
               ▼                                               ▼
 ┌───────────────────────────┐                   ┌───────────────────────────┐
 │   Drone 1 (Instance 0)    │                   │   Drone 2 (Instance 1)    │
 │ ArduCopter SITL @ Mumbai  │                   │ ArduCopter SITL (+10m N)  │
 │ MAVLink UDP Port: 14550   │                   │ MAVLink UDP Port: 14560   │
 └─────────────┬─────────────┘                   └─────────────┬─────────────┘
               │                                               │
               └───────────────────────┬───────────────────────┘
                                       │ MAVLink Telemetry Streams
                                       ▼

```

┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                 UVAX-1 Tokio Engine                                    │
│                                                                                        │
│  ┌─────────────────────────────┐   ┌─────────────────────────┐   ┌──────────────────┐  │
│  │     gps::telemetry_loop     │   │  network::routing_mesh  │   │ camera::listener │  │
│  │ (Atomic LAT/LON/BAT Stores) │   │ (Inter-Agent Comms Hub) │   │  (Snapshot Core) │  │
│  └──────────────┬──────────────┘   └────────────┬────────────┘   └──────────────────┘  │
│                 │                               │                                      │
│                 ▼                               ▼                                      │
│  ┌───────────────────────────────────────────────────────────┐                         │
│  │            poll_for_gps_lock() Non-Blocking Loop          │                         │
│  │        (Holds CLI until EKF3 non-zero coordinates lock)   │                         │
│  └──────────────────────────────┬────────────────────────────┘                         │
│                                 │                                                      │
│                                 ▼                                                      │
│  ┌───────────────────────────────────────────────────────────┐                         │
│  │                  Deep Scan WaveManager                    │                         │
│  │      - Inter-Agent Safety Separation (Euclidean Mesh)    │                         │
│  │      - Anomaly & Cyber Threat Detection Engine            │                         │
│  │      - Dynamic Battery Handoff & Reserve Dispatch         │                         │
│  └──────────────────────────────┬────────────────────────────┘                         │
│                                 │                                                      │
│                                 ▼                                                      │
│  ┌───────────────────────────────────────────────────────────┐                         │
│  │               Interactive CLI (swarm-cli>)                │                         │
│  └───────────────────────────────────────────────────────────┘                         │
└────────────────────────────────────────────────────────────────────────────────────────┘

```

---

## Key Features

* **⚡ Zero-Lock Atomic Telemetry Parsing:** Utilizes Rust atomic integers (`AtomicI32`) to process high-frequency MAVLink packets without mutex lock contention across async Tokio tasks.
* **🔒 Dynamic Non-Blocking Origin Sync:** `poll_for_gps_lock()` actively monitors telemetry streams and unlocks the operator command terminal (`swarm-cli>`) only when all active swarm nodes establish valid EKF3 global position locks.
* **🛡️ Built-in Cyber Threat Detection (IDS):** Real-time checking for:
  * Out-of-bounds trajectory commands ($N, E, Alt$).
  * Unauthenticated MAVLink command injections[cite: 1, 2].
  * Rapid coordinate drift / GPS spoofing jump anomalies[cite: 2].
  * Inter-agent safety distance breaches ($<10\text{m}$ proximity warnings).
* **🤖 Automated Swarm Wave Management:** Real-time distance-to-base and state evaluation automatically transitions active survey drones to `STATIONARY TOWER` or `ReturnToBase` modes while dispatching reserve pool drones[cite: 2].
* **🖥️ Interactive Multi-Window Deployment:** `deploy_swarm.py` handles runtime origin input, opens visible MAVProxy console/map windows for each drone instance, and launches the Rust controller seamlessly.

---

## 🛠️ Tech Stack & Requirements

* **Language/Framework:** Rust (Edition 2021) + Tokio Async Framework
* **Flight Simulator:** ArduPilot SITL (`ArduCopter`) + MAVProxy
* **Orchestration:** Python 3 (with `qterminal` or `x-terminal-emulator`)
* **Environment:** Kali Linux / Ubuntu 22.04 LTS

---

## 🚀 Quick Start

### 1. Clone & Set Up Directory Structure

Ensure your project is structured as follows:

```text
/home/kali/
 ├── ardupilot/
 └── GC1/                # UVAX-1 Core Workspace
      ├── deploy_swarm.py
      ├── Cargo.toml
      └── src/
           ├── main.rs
           ├── gps.rs
           ├── deep_scan.rs
           ├── peer.rs
           ├── network.rs
           ├── camera.rs
           └── bat_management.rs

```

### 2. Launch the System

Run the Python deployment orchestrator:

```bash
cd /home/kali/GC1
python3 deploy_swarm.py

```

### 3. Configure Swarm Origin

When prompted, input your desired survey latitude and longitude (or press `Enter` to use default Mumbai coordinates):

```text
==================================================
📍 SWARM LOCATION CONFIGURATION
==================================================
Enter Origin Latitude [19.0760]: 
Enter Origin Longitude [72.8777]: 

🚀 Launching Drone 1 at: 19.076000, 72.877700
🚀 Launching Drone 2 at: 19.076090, 72.877700 (+10m North)
==================================================

```

`deploy_swarm.py` will open two visible SITL terminal windows with MAVProxy console/map displays, and then boot the UVAX-1 Rust controller in your main terminal.

---

## 🎮 Operator Command Interface (`swarm-cli`)

Once both drones acquire valid EKF3 origin locks, the interactive terminal unlocks:

```text
==================================================
🎮 SWARM OPERATOR COMMAND TERMINAL ONLINE
Commands:
  poi <north_m> <east_m> <alt_m>  -> Send drone to local waypoint offset
  snap OR image                   -> Capture camera snapshot
  status                          -> Display fleet telemetry
  help                            -> Display command menu
==================================================

swarm-cli> 

```

### Commands & Usage

#### Send Waypoint Target Offset (`poi`)

Computes linear meter offsets ($1\text{m} \approx 0.000009^\circ$) relative to current drone coordinates and dispatches MAVLink target commands:

```text
# Move 200m North, 85m East at 15m Altitude (e.g., Mithi River Target)
swarm-cli> poi 200 85 15

🎯 DISPATCHING Target Offset: North 200m, East 85m (Lat 19.077800, Lon 72.878465, Alt 15m)

```

#### View Fleet Telemetry (`status`)

Outputs real-time positions and battery percentages parsed directly from MAVLink streams:

```text
swarm-cli> status

--- FLEET TELEMETRY STATUS ---
🛸 Drone 1: Pos (19.076000, 72.877700) | Bat: 98%
🛸 Drone 2: Pos (19.076090, 72.877700) | Bat: 100%
------------------------------

```

#### Trigger Camera Payload (`snap`)

Sends a trigger packet to the local camera module listener over UDP port `5001`:

```text
swarm-cli> snap
📸 TRIGGERING CAMERA SNAPSHOT CAPTURE...

```

---

## 🛡️ PUSHPAK Grand Challenge 3 Test Scenario

To execute a full flight test for the Techfest presentation / Stage 1 submission:

1. Launch the system via `python3 deploy_swarm.py`.
2. In **Drone 1's MAVProxy terminal window**, arm and launch the drone:
```text
MAV> mode guided
MAV> arm throttle
MAV> takeoff 15

```


3. Once airborne, switch back to the main terminal and dispatch the drone to the target POI:
```text
swarm-cli> poi 200 85 15

```


4. Observe real-time trajectory tracking on the MAVProxy map and anomaly check pass-throughs in the Rust logs.



---

## 📜 License

Developed for academic research, competitive benchmarking, and participation in the **PUSHPAK National Mission on Drone Technology (MeitY / IIT Bombay)**.

```

```
