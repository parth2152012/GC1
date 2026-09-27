#!/usr/bin/env python3
"""Aggregate measured JSONL evidence from a GC1 controller run.

Missing instrumentation is reported as N/A; the script never fabricates PDR or
latency values.
"""
import json
import statistics
import sys
from collections import Counter


def percentile(values, p):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * p
    lo, hi = int(rank), min(int(rank) + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (rank - lo)


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <path-to-swarm_metrics.jsonl>")
        sys.exit(1)
    events = []
    with open(sys.argv[1], "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    if not events:
        print("No events found in log.")
        return

    events.sort(key=lambda event: event.get("ts_ms", 0))
    by_type = Counter(e.get("event") for e in events)
    timestamps = [e["ts_ms"] for e in events if isinstance(e.get("ts_ms"), (int, float))]
    total_wall_s = (max(timestamps) - min(timestamps)) / 1000.0 if timestamps else None
    starts = [e for e in events if e.get("event") == "MissionStart"]
    mission_start_ms = starts[0].get("ts_ms") if starts else None
    progress = [e for e in events if e.get("event") == "MissionProgress"]
    final_progress = progress[-1] if progress else None
    completions = [e for e in events if e.get("event") == "PoiCompleted"]
    faults = [e for e in events if e.get("event") == "FaultInjected"]

    print("=== UAV-X Stage 1 measured metrics summary ===")
    print(f"Log span: {total_wall_s:.1f}s ({len(events)} events)" if total_wall_s is not None else f"Log span: N/A ({len(events)} events)")

    print("\n-- Mission --")
    if final_progress:
        total = final_progress.get("total", 0)
        completed = final_progress.get("completed", 0)
        failed = final_progress.get("failed", 0)
        rate = (100.0 * completed / total) if total else None
        score = final_progress.get("priority_weighted_score_pct")
        min_sep_values = [e.get("min_separation_m") for e in progress if isinstance(e.get("min_separation_m"), (int, float))]
        min_sep = min(min_sep_values) if min_sep_values else None
        completion_time = None
        if mission_start_ms and completions:
            completion_time = (max(e["ts_ms"] for e in completions) - mission_start_ms) / 1000.0
        print(f"Completion rate: {rate:.1f}% ({completed}/{total}, {failed} failed)" if rate is not None else "Completion rate: N/A")
        print(f"Completion time: {completion_time:.1f}s" if completion_time is not None else "Completion time: N/A")
        print(f"Priority-weighted mission score: {score:.1f}%" if isinstance(score, (int, float)) else "Priority-weighted mission score: N/A")
        print(f"Elapsed at last snapshot: {final_progress.get('elapsed_s', 0):.1f}s")
        print(f"Measured minimum inter-UAV separation: {min_sep:.1f}m" if min_sep is not None else "Measured minimum inter-UAV separation: N/A")
    else:
        print("No MissionProgress events found — run long enough to emit a snapshot.")

    print("\n-- Communication --")
    sent = {e.get("sequence") for e in events if e.get("event") == "PacketSent" and e.get("sequence") is not None}
    received = {e.get("sequence") for e in events if e.get("event") == "PacketReceived" and e.get("sequence") is not None}
    matched = sent & received
    if sent:
        pdr = 100.0 * len(matched) / len(sent)
        print(f"Origin packets sent: {len(sent)}")
        print(f"Unique packets received: {len(received)}")
        print(f"Packet delivery ratio: {pdr:.2f}% ({len(matched)}/{len(sent)}, missing={len(sent - received)})")
    else:
        print("Origin packets sent: N/A")
        print("Unique packets received: N/A")
        print("Packet delivery ratio: N/A (no PacketSent events)")
    latencies = [e["latency_ms"] for e in events if e.get("event") == "PacketReceived" and isinstance(e.get("latency_ms"), (int, float))]
    if latencies:
        print(f"Latency: count={len(latencies)}, mean={statistics.mean(latencies):.2f}ms, min={min(latencies):.2f}ms, max={max(latencies):.2f}ms, p95={percentile(latencies, 0.95):.2f}ms")
    else:
        print("Latency: N/A (no measured PacketReceived samples)")
    mesh = [e for e in events if e.get("event") == "MeshStatus"]
    if mesh:
        last_mesh = mesh[-1]
        print(f"Mesh counters: received={last_mesh.get('received', 0)}, forwarded={last_mesh.get('forwarded', 0)}, dropped_outage={last_mesh.get('dropped_outage', 0)}")
        intervals = [(a, b, max(0.0, (b.get("ts_ms", 0) - a.get("ts_ms", 0)) / 1000.0)) for a, b in zip(mesh, mesh[1:])]
        observed_s = sum(interval[2] for interval in intervals)
        downtime_s = sum(interval[2] for interval in intervals if interval[0].get("link_down"))
        availability = 100.0 * (1.0 - downtime_s / observed_s) if observed_s > 0 else None
        print(f"Connectivity availability: {availability:.2f}%" if availability is not None else "Connectivity availability: N/A")
        print(f"Communication downtime: {downtime_s:.2f}s")
    else:
        print("Connectivity availability: N/A (no MeshStatus events)")
        print("Communication downtime: N/A")

    print("\n-- Autonomy --")
    recovery = [e.get("seconds") for e in events if e.get("event") == "RecoveryTime" and isinstance(e.get("seconds"), (int, float))]
    print(f"Relay reallocations: {by_type.get('RelayReallocation', 0)}")
    print(f"Recovery time(s): {', '.join(f'{v:.2f}s' for v in recovery) if recovery else 'N/A'}")
    if faults and completions:
        first_fault = min(e.get("ts_ms", 0) for e in faults)
        before = sum(e.get("ts_ms", 0) < first_fault for e in completions)
        after = sum(e.get("ts_ms", 0) >= first_fault for e in completions)
        print(f"Post-failure mission performance: {after} PoI completions after first injected fault (pre-fault completions={before})")
    else:
        print("Post-failure mission performance: N/A (requires fault and mission run)")

    print("\n-- Robustness --")
    print(f"Faults injected: {len(faults)}")
    for fault in faults:
        print(f"  {fault.get('kind')}: {fault.get('detail')}")

    print("\n-- Safety --")
    print(f"Separation breaches / collision warnings: {by_type.get('SeparationBreach', 0)}")
    print(f"Collision-avoidance maneuvers triggered: {by_type.get('CollisionAvoidanceManeuver', 0)}")
    print(f"Link-range warnings logged: {by_type.get('LinkWarning', 0)}")
    print(f"Photos captured and saved: {by_type.get('PhotoCaptured', 0)}")


if __name__ == "__main__":
    main()
