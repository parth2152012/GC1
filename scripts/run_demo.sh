#!/usr/bin/env bash
# Deterministic, repeatable UAV-X demo sequence.
#
# Usage:
#   bash scripts/run_demo.sh | python3 deploy_swarm.py
#
# deploy_swarm.py first asks three setup questions (ArduPilot directory, drone count,
# origin lat/lon) before it launches SITL and `cargo run`. This script answers those
# with blank lines (accepting the defaults: ~/ardupilot, 2 drones, the default Mumbai
# origin) and then, once the Rust controller's CLI comes up, feeds it a fixed, timed
# sequence of operator commands — including one deliberate fault injection — so the
# exact same demo can be re-run for the competition video and to produce comparable
# swarm_metrics.jsonl runs across attempts.
#
# If your ArduPilot checkout isn't at ~/ardupilot, edit deploy_swarm.py's
# DEFAULT_ARDUPILOT_DIR (or type the path manually instead of piping this script).
#
# The controller keeps running after this script's input ends (its background tasks
# are infinite loops) — swarm_metrics.jsonl is written continuously, so you can inspect
# it from another terminal at any point, or press Ctrl+C here when you're done and then
# run: python3 scripts/aggregate_metrics.py swarm_metrics.jsonl

set -euo pipefail

echo ""   # ArduPilot directory -> accept default
echo ""   # drone count -> accept default (2)
echo ""   # origin latitude -> accept default
echo ""   # origin longitude -> accept default

sleep 25  # SITL boot + GPS lock
echo "mode guided"
sleep 2
echo "arm throttle"
sleep 2
echo "takeoff 15"
sleep 15  # let the autonomous mission loop start assigning/flying PoIs

echo "status"
sleep 60  # observe autonomous multi-PoI survey + normal relay/role behaviour

# Controlled, repeatable fault injection — demonstrates recovery instead of just
# claiming it (Robustness / Fault-Recovery rubric items, 15% of the score).
echo "fail battery 1 10"
sleep 30  # WaveManager detects it, promotes the reserve, stalled PoIs get reassigned

echo "status"
sleep 30  # let the reserve continue the survey

echo "fail link"
sleep 12  # produce measured sent/received loss during the simulated outage
echo "recover"
sleep 12
echo "snap"  # succeeds when camera.py/OpenCV support is available
echo "status"
sleep 10
# stderr is intentionally not fed back into the controller command pipe.
python3 scripts/aggregate_metrics.py swarm_metrics.jsonl >&2 || true
