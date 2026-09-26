#!/bin/bash

DEFAULT_ARDUPILOT="$HOME/ardupilot"
read -p "Enter ArduPilot directory path [$DEFAULT_ARDUPILOT]: " ARDUPILOT_PATH
ARDUPILOT_PATH=${ARDUPILOT_PATH:-$DEFAULT_ARDUPILOT}

if [ ! -d "$ARDUPILOT_PATH" ]; then
    echo "❌ Error: ArduPilot directory not found at $ARDUPILOT_PATH"
    exit 1
fi

SIM_VEHICLE="$ARDUPILOT_PATH/Tools/autotest/sim_vehicle.py"

echo "🚀 Booting SITL Swarm Nodes at Local Map Origin (Mumbai)..."

# Launch SITL Instance 0 (Drone 1)
$SIM_VEHICLE -v ArduCopter -I 0 --sysid 1 --location=19.076000,72.877700,0,0 --no-rebuild > /dev/null 2>&1 &
SITL0_PID=$!
echo "  └─ [Drone 1] SITL Instance 0 launched (PID: $SITL0_PID | MAVLink: 14550)"

# Launch SITL Instance 1 (Drone 2) with 30m offset
$SIM_VEHICLE -v ArduCopter -I 1 --sysid 2 --location=19.076000,72.878000,0,0 --no-rebuild > /dev/null 2>&1 &
SITL1_PID=$!
echo "  └─ [Drone 2] SITL Instance 1 launched (PID: $SITL1_PID | MAVLink: 14560)"

trap "echo 'Stopping background tasks...'; kill $SITL0_PID $SITL1_PID 2>/dev/null" EXIT

echo "⏳ Waiting 6 seconds for MAVLink parameters and GPS lock..."
sleep 6

# Launch camera feed listener if available
if [ -f "camera.py" ]; then
    python3 camera.py > /dev/null 2>&1 &
    CAM_PID=$!
    echo "📸 Camera feed listener active (PID: $CAM_PID)"
    trap "kill $SITL0_PID $SITL1_PID $CAM_PID 2>/dev/null" EXIT
fi

echo "=================================================="
echo "⚡ Launching Async Rust Swarm Controller..."
echo "=================================================="

cargo run