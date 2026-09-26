#!/usr/bin/env bash

set -e

ARDUPILOT_DIR="/home/kali/ardupilot"
RUST_APP_DIR="/home/kali/GC1"

echo "=================================================="
echo "🧹 Cleaning up background SITL, MAVProxy & Rust processes..."
echo "=================================================="
sudo fuser -k 14550/udp 14560/udp 5760/tcp 5770/tcp 2>/dev/null || true
pkill -f arducopter 2>/dev/null || true
pkill -f sim_vehicle.py 2>/dev/null || true
pkill -f GC1 2>/dev/null || true
sleep 1

echo "=================================================="
echo "🚀 Booting SITL Swarm Nodes (Mumbai Origin: 19.0760, 72.8777)..."
echo "=================================================="

if [ ! -d "$ARDUPILOT_DIR" ]; then
  echo "❌ Error: ArduPilot directory not found at $ARDUPILOT_DIR"
  exit 1
fi

cd "$ARDUPILOT_DIR/ArduCopter"

# Launch Drone 1 (Instance 0) with console output to UDP 14550
echo "  └─ [Drone 1] Spawning SITL Instance 0..."
"$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter \
  -I 0 \
  --console \
  --map \
  --out=udp:127.0.0.1:14550 \
  --custom-location=19.0760,72.8777,0,0 \
  --no-rebuild \
  --wipe-eeprom >/tmp/sitl_drone1.log 2>&1 &
DRONE1_PID=$!

# Launch Drone 2 (Instance 1) with console output to UDP 14560
echo "  └─ [Drone 2] Spawning SITL Instance 1..."
"$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter \
  -I 1 \
  --console \
  --map \
  --out=udp:127.0.0.1:14560 \
  --custom-location=19.0760,72.8780,0,0 \
  --no-rebuild \
  --wipe-eeprom >/tmp/sitl_drone2.log 2>&1 &
DRONE2_PID=$!

echo "  └─ SITL PIDs: $DRONE1_PID | $DRONE2_PID"

echo "=================================================="
echo "⚡ Handing execution to Async Rust Swarm Controller..."
echo "=================================================="
cd "$RUST_APP_DIR"
cargo run
