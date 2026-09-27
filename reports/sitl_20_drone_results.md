# Twenty-drone controller and packet-relay test

A 120-second live run with 20 ArduCopter SITL instances observed telemetry from **20/20** drones. Seven flew: IDs 1, 14, 15, 16, 17, 19, 20; others remained available. This verifies a fleet larger than the previous hardcoded two-node controller; it does not claim all twenty flew simultaneously.

- Drone 14 completed random POI #2 after a center acknowledgement over **2 UDP hops**.
- Drone 16 completed random POI #1 after a center acknowledgement over **4 UDP hops**.
- Drone 1 completed operator POI #3 after a center acknowledgement over **1 UDP hop**.
- A separate socket test delivered a packet and its acknowledgement across **20 hops**. Another socket test removed an essential relay, verified failure, restored it, and verified delivery. All **17 Rust tests passed**.
- The updated launcher passes its selected 1–20 drone count to the controller. `drone 20` selects that drone for commands; `status` covers the configured fleet.

Forwarding uses separate per-node UDP sockets in one controller process, and every link is checked against fresh measured positions and the 100 m bound. These are actual packet hops under simulated radio-range constraints. Physical RF is not tested. Flight-control and telemetry MAVLink connections remain direct to MAVProxy.

**Remaining scenario failures:** maximum speed was 9.98 m/s despite the Guided speed request; minimum measured airborne separation was 2.43 m. The simple corridor planner does not guarantee collision avoidance, and launch spacing remains 5 m. Endurance, landing deadlines, and latency from first detection still need dedicated validation. These limitations are not reported as passing.

Run logs: `/tmp/gc1-sitl-20-yvorb_dw/`. The harness stopped all processes it launched. Structured results: `reports/sitl_20_drone_results.json`.
