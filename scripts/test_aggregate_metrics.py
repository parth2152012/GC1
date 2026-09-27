#!/usr/bin/env python3
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class AggregateMetricsTest(unittest.TestCase):
    def test_measured_pdr_latency_and_downtime(self):
        root = Path(__file__).resolve().parent
        events = [
            {"ts_ms": 1000, "event": "MissionStart", "mission_id": 1, "poi_count": 1},
            {"ts_ms": 1000, "event": "PacketSent", "sequence": 1, "sender_id": 1, "target_id": 255},
            {"ts_ms": 1000, "event": "PacketSent", "sequence": 2, "sender_id": 1, "target_id": 255},
            {"ts_ms": 1001, "event": "PacketReceived", "sequence": 1, "sender_id": 1, "target_id": 255, "latency_ms": 1},
            {"ts_ms": 2000, "event": "MeshStatus", "received": 1, "forwarded": 0, "dropped_outage": 0, "link_down": False, "known_peers": 1},
            {"ts_ms": 3000, "event": "MeshStatus", "received": 1, "forwarded": 0, "dropped_outage": 1, "link_down": True, "known_peers": 1},
            {"ts_ms": 4000, "event": "MeshStatus", "received": 1, "forwarded": 0, "dropped_outage": 1, "link_down": False, "known_peers": 1},
        ]
        with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as handle:
            for event in events:
                handle.write(json.dumps(event) + "\n")
            path = handle.name
        result = subprocess.run([sys.executable, str(root / "aggregate_metrics.py"), path], capture_output=True, text=True, check=True)
        self.assertIn("Packet delivery ratio: 50.00%", result.stdout)
        self.assertIn("mean=1.00ms", result.stdout)
        self.assertIn("Communication downtime: 1.00s", result.stdout)


if __name__ == "__main__":
    unittest.main()
