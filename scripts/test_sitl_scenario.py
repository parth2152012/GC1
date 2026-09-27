#!/usr/bin/env python3
"""Headless live SITL audit against sample scenario for GC 1.pptx.pdf.

Launches isolated simulator processes, observes actual MAVLink telemetry and writes
logs/results. Does not change flight parameters or pretend localhost is a radio.
Requires pymavlink, MAVProxy, a built ArduCopter SITL and target/debug/GC1.
"""
import argparse
import itertools
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import tempfile
import time

from pymavlink import mavutil

REPO = Path(__file__).resolve().parents[1]
HOME_LAT, HOME_LON = 19.076, 72.8777


def position(lat, lon, altitude):
    return ((lat - HOME_LAT) * math.pi / 180 * 6378137,
            (lon - HOME_LON) * math.pi / 180 * 6378137 * math.cos(math.radians(HOME_LAT)),
            altitude)


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def reachable_from_center(states):
    """Optimistic geometric reachability; this is not a packet-delivery test."""
    points = {i: s['position'] for i, s in states.items()}
    points[0] = (0, 0, 0)
    seen, pending = {0}, [0]
    while pending:
        current = pending.pop()
        for other, point in points.items():
            if other not in seen and distance(points[current], point) <= 100:
                seen.add(other)
                pending.append(other)
    return seen


def run(args):
    root = Path(tempfile.mkdtemp(prefix=f'gc1-sitl-{args.drones}-'))
    print(f'Artifacts: {root}', flush=True)
    processes, files, monitors = [], [], []
    states, stats = {}, {i: {'samples': 0, 'max_altitude_m': 0, 'max_speed_mps': 0,
                           'max_distance_from_home_m': 0, 'airborne': False}
                         for i in range(1, args.drones+1)}
    min_air_separation = None
    disconnected = set()
    outside_area = set()
    manual_sent = False
    controller = None
    started = time.monotonic()
    failure = None

    def launch(command, cwd, log, stdin=False):
        stream = open(cwd / log, 'w')
        files.append(stream)
        proc = subprocess.Popen(command, cwd=cwd, stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
                                stdout=stream, stderr=subprocess.STDOUT, text=True, start_new_session=True,
                                env={**os.environ, "GC1_DRONE_COUNT": str(args.drones)})
        processes.append(proc)
        return proc

    try:
        # Fail instead of attaching to an existing flight session.
        for i in range(args.drones):
            for port in (5760+10*i, 14600+i):
                with socket.socket() as check:
                    check.settimeout(.15)
                    if check.connect_ex(('127.0.0.1', port)) == 0:
                        raise RuntimeError(f'Port {port} is occupied; refusing to touch an existing session')
            monitors.append(mavutil.mavlink_connection(f'udpin:127.0.0.1:{15000+i}'))
        for i in range(args.drones):
            directory = root / f'drone-{i+1}'
            directory.mkdir()
            launch([str(args.ardupilot / 'build/sitl/bin/arducopter'), '--model', '+',
                    '--defaults', str(args.ardupilot / 'Tools/autotest/default_params/copter.parm'),
                    '--home', f'{HOME_LAT+i*5/111319.49},{HOME_LON},0,0',
                    '--sysid', str(i+1), '-I', str(i), '-w'], directory, 'sitl.log')
            launch([shutil.which('mavproxy.py'), '--master', f'tcp:127.0.0.1:{5760+i*10}',
                    '--out', f'udp:127.0.0.1:{14550+i*10}',
                    '--out', f'tcpin:127.0.0.1:{14600+i}',
                    '--out', f'udp:127.0.0.1:{15000+i}', '--non-interactive'], directory, 'mavproxy.log')
        time.sleep(5)
        controller = launch([str(REPO / 'target/debug/GC1')], root, 'controller.log', True)
        started = time.monotonic()
        last_print = 0
        with open(root / 'telemetry.jsonl', 'w') as telemetry:
            while time.monotonic()-started < args.seconds:
                elapsed = time.monotonic()-started
                for i, monitor in enumerate(monitors, 1):
                    for _ in range(250):
                        msg = monitor.recv_match(blocking=False)
                        if msg is None:
                            break
                        if msg.get_type() != 'GLOBAL_POSITION_INT' or msg.get_srcSystem() != i:
                            continue
                        point = position(msg.lat/1e7, msg.lon/1e7, msg.relative_alt/1000)
                        speed = math.sqrt(msg.vx**2+msg.vy**2+msg.vz**2)/100
                        states[i] = {'t': elapsed, 'position': point, 'speed_mps': speed}
                        telemetry.write(json.dumps({'id': i, **states[i]})+'\n')
                        entry = stats[i]
                        entry['samples'] += 1
                        entry['max_altitude_m'] = max(entry['max_altitude_m'], point[2])
                        entry['max_speed_mps'] = max(entry['max_speed_mps'], speed)
                        entry['max_distance_from_home_m'] = max(entry['max_distance_from_home_m'], distance(point, ((i-1)*5, 0, 0)))
                        entry['airborne'] |= point[2] > 2
                        if abs(point[0]) > 500 or abs(point[1]) > 500:
                            outside_area.add(i)
                fresh = {i:s for i,s in states.items() if elapsed-s['t'] < 2}
                airborne = {i:s for i,s in fresh.items() if s['position'][2] > 2}
                for a,b in itertools.combinations(airborne.values(), 2):
                    value = distance(a['position'], b['position'])
                    min_air_separation = value if min_air_separation is None else min(min_air_separation, value)
                if len(fresh) == args.drones:
                    disconnected.update(set(airborne)-reachable_from_center(fresh))
                log = (root / 'controller.log').read_text()
                if not manual_sent and 'completed PoI #' in log:
                    controller.stdin.write('mode guided\narm throttle\npoi 40 20 15\nstatus\n')
                    controller.stdin.flush()
                    manual_sent = True
                    print('Sent manual waypoint after first automatic completion', flush=True)
                if elapsed-last_print >= 15:
                    last_print = elapsed
                    print(f'{elapsed:.0f}s: telemetry={len(states)}/{args.drones}, '
                          f'airborne={[i for i,s in stats.items() if s["airborne"]]}, '
                          f'completed={len(re.findall(r"completed PoI #",log))}', flush=True)
                if controller.poll() is not None:
                    raise RuntimeError(f'Controller exited {controller.returncode}')
                time.sleep(.1)
    except Exception as error:
        failure = repr(error)
        print(f'RUN ERROR: {failure}', flush=True)
    finally:
        elapsed = time.monotonic()-started
        # Stop only processes created by this harness. These are disposable SITL instances.
        for proc in reversed(processes):
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for monitor in monitors:
            monitor.close()
        for stream in files:
            stream.close()
    log = (root/'controller.log').read_text() if (root/'controller.log').exists() else ''
    assigned = sorted(set(map(int, re.findall(r'Auto-dispatched Drone (\d+)', log))))
    complete = sorted(set(map(int, re.findall(r'completed PoI #(\d+)', log))))
    manual = re.findall(r'dispatched to operator PoI #(\d+)', log)
    result = {
        'scenario_pdf': 'sample scenario for GC 1.pptx.pdf',
        'test_type': 'Live ArduCopter SITL baseline; unmodified production mission timing and flight parameters',
        'drones_requested': args.drones, 'duration_seconds': round(elapsed, 2), 'run_error': failure,
        'telemetry_drones': sorted(states), 'controller_assigned_drones': assigned,
        'spawned_random_pois': log.count('New PoI spawned'), 'completed_poi_ids': complete,
        'manual_poi_completed': bool(manual) and int(manual[-1]) in complete,
        'radio_ack_hops': list(map(int, re.findall(r'center ACK via (\d+) radio hops', log))),
        'per_drone': stats,
        'speed_limit_5mps': {'status': 'FAIL' if any(s['max_speed_mps']>5.1 for s in stats.values()) else ('NO_BREACH_OBSERVED' if states else 'NOT_TESTED'),
                              'measurement_tolerance_mps': .1},
        'height_limit_100m': {'status': 'FAIL' if any(s['max_altitude_m']>100 for s in stats.values()) else ('NO_BREACH_OBSERVED' if states else 'NOT_TESTED')},
        'separation_20m': {'minimum_between_airborne_m': min_air_separation,
                           'launcher_ground_spacing_m': 5,
                           'status': 'NOT_EXERCISED' if min_air_separation is None else ('FAIL' if min_air_separation<20 else 'NO_BREACH_OBSERVED')},
        'radio_hops_100m': {'status': 'FAIL_GEOMETRY' if disconnected else 'NOT_VERIFIED',
                            'disconnected_airborne_drones': sorted(disconnected),
                            'note': 'Geometric audit is separate from actual radio_ack_hops. Flight MAVLink uses direct localhost links; POI reports use the UDP radio emulator.'},
        'arena': {'outside_current_centered_geofence': sorted(outside_area),
                  'note': 'Controller centers arena on Drone 1; diagram operational-center offset is not modeled.'},
        'flight_limit_20min': 'NOT_TESTED' if elapsed<1200 else 'REQUIRES_FLIGHT_INTERVAL_AUDIT',
        'land_all_by_45min': 'NOT_TESTED' if elapsed<2700 else 'REQUIRES_LANDING_AUDIT',
        'report_to_center_within_10s': 'NOT_VERIFIED: ACK hop counts observed, but first-detection deadline not audited',
        'launch_layout': 'Production launcher baseline: 5m-spaced ground positions near common origin',
        'full_scenario_pass': None,
        'scope_note': 'Partial live baseline audit; unmeasured requirements cannot be certified.',
    }
    (root/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    print(f'Results: {root}/results.json', flush=True)
    return 2 if failure else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drones', type=int, default=10)
    parser.add_argument('--seconds', type=int, default=180)
    parser.add_argument('--ardupilot', type=Path, default=Path.home()/'ardupilot')
    args = parser.parse_args()
    if not 1 <= args.drones <= 20 or args.seconds <= 0:
        parser.error('Use 1..20 drones and a positive duration')
    raise SystemExit(run(args))
