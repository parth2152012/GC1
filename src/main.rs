mod bat_management;
mod camera;
mod deep_scan;
mod gps;
// FIX: `mission.rs` was never declared as a module, so the entire PoI priority-queue /
// task-allocator (with its own passing test suite) was dead code that `cargo build`
// never even compiled. This is the single highest-value fix in the project: it turns
// on the multi-PoI autonomy the proposal itself flagged as missing.
mod mission;
// FIX: no metrics/logging existed anywhere. This adds the "standardized log format"
// deliverable and the data needed to fill in every "Pending" row of the official
// Evaluation & Performance Metrics table.
mod metrics;
mod network;
mod peer;

use std::net::UdpSocket;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::time::sleep;

use bat_management::BatteryState;
use deep_scan::{DroneRole, SwarmNode, WaveManager};
use peer::PositionEnu;

const EARTH_RADIUS_M: f64 = 6_378_137.0;
const GEOFENCE_HALF_EXTENT_M: f64 = 500.0; // matches organizer's 1000m x 1000m arena.

// FIX: the organizer's sample scenario caps "Operational height max" at 100 m, but the
// CLI validation here previously allowed 0..=120 m for both `takeoff` and `poi`. Any
// PoI/takeoff issued between 100 and 120 m would have been accepted by this software
// while violating the actual competition constraint.
const ORGANIZER_ALT_MAX_M: f32 = 100.0;

// FIX: PoI arrival radius used by the new autonomous completion check below.
const POI_ARRIVAL_RADIUS_M: f64 = 15.0;
const MISSION_DURATION: Duration = Duration::from_secs(45 * 60);

fn enu_from_global(
    origin_lat: f64,
    origin_lon: f64,
    lat: f64,
    lon: f64,
    alt_m: f64,
) -> PositionEnu {
    let north_m = (lat - origin_lat).to_radians() * EARTH_RADIUS_M;
    let east_m = (lon - origin_lon).to_radians() * EARTH_RADIUS_M * origin_lat.to_radians().cos();
    PositionEnu {
        east_m,
        north_m,
        up_m: alt_m,
    }
}

fn target_from_enu(origin_lat: f64, origin_lon: f64, north_m: f64, east_m: f64) -> (f64, f64) {
    (
        origin_lat + (north_m / EARTH_RADIUS_M).to_degrees(),
        origin_lon + (east_m / (EARTH_RADIUS_M * origin_lat.to_radians().cos())).to_degrees(),
    )
}

/// Minimal, dependency-free xorshift64 PRNG.
///
/// FIX: `rand` is not a dependency, and this sandbox/most CI runners for this project
/// cannot always reach crates.io, so PoI spawning below avoids adding a new crate
/// entirely and just uses this ~10-line generator seeded from the wall clock.
struct Xorshift64(u64);
impl Xorshift64 {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// A PoI that will be inserted into the mission at `spawn_at`, simulating the
/// organizer's "Number of PoIs – 10. Spawned randomly (position and the time of
/// spawning)" mission constraint.
///
/// FIX: previously the only way a PoI entered the system was a human typing
/// `poi <north> <east> <alt>` at the CLI. There was no automated, benchmark-shaped PoI
/// feed, no priority assignment, and no "newly emerging high-priority regions" at all.
struct PendingPoi {
    spawn_at: Instant,
    north_m: f64,
    east_m: f64,
    alt_m: f32,
    priority: mission::Priority,
}

fn spawn_random_pois(
    count: usize,
    half_extent_m: f64,
    alt_m: f32,
    window: Duration,
    rng: &mut Xorshift64,
) -> Vec<PendingPoi> {
    let now = Instant::now();
    (0..count)
        .map(|_| {
            let north_m = (rng.next_f64() * 2.0 - 1.0) * half_extent_m;
            let east_m = (rng.next_f64() * 2.0 - 1.0) * half_extent_m;
            let delay_s = rng.next_f64() * window.as_secs_f64();
            let priority = match rng.next_u64() % 3 {
                0 => mission::Priority::High,
                1 => mission::Priority::Medium,
                _ => mission::Priority::Low,
            };
            PendingPoi {
                spawn_at: now + Duration::from_secs_f64(delay_s),
                north_m,
                east_m,
                alt_m,
                priority,
            }
        })
        .collect()
}

/// If `node` is idle (LeadSurveyor, no current PoI), pulls the next PoI from the
/// mission manager and actually commands the vehicle there over MAVLink.
///
/// FIX: this is the autonomous multi-PoI task allocation and dispatch that was
/// entirely absent before — `MissionManager::select_next_poi_for_uav` existed but
/// nothing ever called it, and nothing ever turned its result into a flight command.
async fn try_assign_and_dispatch(
    node: &mut SwarmNode,
    mission_mgr: &mut mission::MissionManager,
    mission_id: u32,
    origin_lat: f64,
    origin_lon: f64,
) {
    if node.role != DroneRole::LeadSurveyor || node.current_poi.is_some() {
        return;
    }
    let Some(poi) = mission_mgr.select_next_poi_for_uav(mission_id, node, origin_lat, origin_lon)
    else {
        return;
    };
    let _ = mission_mgr.mark_poi_in_progress(mission_id, poi.id, node.id);
    node.current_poi = Some(poi.id);
    metrics::log_event(metrics::MetricEvent::PoiAssigned {
        mission_id,
        poi_id: poi.id,
        drone_id: node.id,
    })
    .await;

    let (target_lat, target_lon) = target_from_enu(origin_lat, origin_lon, poi.north_m, poi.east_m);
    match gps::send_guided_target_for(node.id, target_lat, target_lon, poi.alt_m) {
        Ok(()) => {
            node.airborne_since.get_or_insert(Instant::now());
            println!(
                "\n🎯 Auto-dispatched Drone {} to PoI #{} (priority {:?}) at N{:.0}m E{:.0}m",
                node.id, poi.id, poi.priority, poi.north_m, poi.east_m
            )
        }
        Err(error) => eprintln!("\n❌ Auto-dispatch to PoI #{} failed: {error}", poi.id),
    }
}

/// If `node` is holding a `current_poi` but is no longer able to work it (it has left
/// the `LeadSurveyor` role — e.g. it was forced into `ReturnToBase` by a battery fault,
/// or handed relay duty as `StationaryTower`), releases that PoI back to the mission
/// pool so another available UAV can pick it up, instead of leaving it stuck forever.
///
/// FIX: this is the "reassign outstanding work when the currently assigned UAV becomes
/// unavailable" behavior — previously nothing did this at all; see the
/// `release_stalled_poi` doc comment in mission.rs for why it was needed even for the
/// existing (previously-uncompiled) test suite.
async fn release_if_stalled(
    node: &mut SwarmNode,
    mission_mgr: &mut mission::MissionManager,
    mission_id: u32,
) {
    let Some(poi_id) = node.current_poi else {
        return;
    };
    if node.role == DroneRole::LeadSurveyor {
        return;
    }
    let _ = mission_mgr.release_stalled_poi(mission_id, poi_id);
    println!(
        "\n♻️  Drone {} went unavailable ({:?}) mid-task — releasing PoI #{poi_id} for reassignment.",
        node.id, node.role
    );
    metrics::log_event(metrics::MetricEvent::PoiReleased {
        mission_id,
        poi_id,
        drone_id: node.id,
    })
    .await;
    node.current_poi = None;
}

/// If `node` is within arrival radius of the PoI it is currently assigned to, marks it
/// complete and frees the node up for a new assignment on the next tick.
async fn try_complete_poi(
    node: &mut SwarmNode,
    mission_mgr: &mut mission::MissionManager,
    mission_id: u32,
) {
    let Some(poi_id) = node.current_poi else {
        return;
    };
    let Some(pois) = mission_mgr.get_all_pois(mission_id) else {
        return;
    };
    let Some(poi) = pois.iter().find(|p| p.id == poi_id) else {
        node.current_poi = None;
        return;
    };
    let target = PositionEnu {
        east_m: poi.east_m,
        north_m: poi.north_m,
        up_m: poi.alt_m as f64,
    };
    if peer::enu_distance_meters(node.position_enu, target) <= POI_ARRIVAL_RADIUS_M {
        let _ = mission_mgr.mark_poi_completed(mission_id, poi_id);
        println!("\n✅ Drone {} completed PoI #{poi_id}.", node.id);
        metrics::log_event(metrics::MetricEvent::PoiCompleted {
            mission_id,
            poi_id,
            drone_id: node.id,
        })
        .await;
        node.current_poi = None;
    }
}

/// Dynamic async check that polls MAVLink telemetry stores until valid GPS coordinates arrive
async fn poll_for_gps_lock() {
    println!("⏳ Awaiting active EKF3 GPS locks from Drone 1 and Drone 2...");

    let mut check_count = 0;
    loop {
        let lat1 = gps::LATITUDE_1.load(Ordering::Acquire);
        let lon1 = gps::LONGITUDE_1.load(Ordering::Acquire);
        let lat2 = gps::LATITUDE_2.load(Ordering::Acquire);
        let lon2 = gps::LONGITUDE_2.load(Ordering::Acquire);

        if lat1 != 0 && lon1 != 0 && lat2 != 0 && lon2 != 0 {
            let real_lat1 = lat1 as f64 / 1e7;
            let real_lon1 = lon1 as f64 / 1e7;
            let real_lat2 = lat2 as f64 / 1e7;
            let real_lon2 = lon2 as f64 / 1e7;

            println!("\n✅ GPS ORIGIN LOCK ACHIEVED across Swarm Nodes!");
            println!("  ├─ Drone 1: ({:.6}, {:.6})", real_lat1, real_lon1);
            println!("  └─ Drone 2: ({:.6}, {:.6})", real_lat2, real_lon2);
            break;
        }

        check_count += 1;
        if check_count % 10 == 0 {
            println!("   [Telemetry Sync] Still waiting for MAVLink GLOBAL_POSITION_INT stream...");
        }

        sleep(Duration::from_millis(250)).await;
    }
}

#[tokio::main]
async fn main() {
    println!("🚀 Booting Deep Scan Swarm Controller...");
    metrics::init("swarm_metrics.jsonl");

    let t1 = tokio::spawn(async {
        camera::start_listener().await;
    });
    let t2 = tokio::spawn(async {
        gps::start_telemetry_loop().await;
    });
    let t3 = tokio::spawn(async {
        network::start_routing_mesh().await;
    });

    // Start accepting commands immediately; target commands still validate GPS lock.
    let cli = tokio::spawn(async {
        println!("\n==================================================");
        println!("🎮 SWARM OPERATOR COMMAND TERMINAL ONLINE");
        println!("Commands:");
        println!("  mode guided                     -> Set Drone 1 to Guided mode");
        println!("  arm throttle                    -> Arm Drone 1");
        println!("  takeoff <alt_m>                 -> Take off Drone 1 to altitude");
        println!("  poi <north_m> <east_m> <alt_m>  -> Send drone to local waypoint offset");
        println!("  snap OR image                   -> Capture camera snapshot");
        println!("  status                          -> Display fleet telemetry");
        println!("  fail battery <1|2> <percent>    -> Inject a battery fault for testing");
        println!("  fail link                       -> Simulate a mesh comms outage");
        println!("  recover                         -> Clear a simulated link outage");
        println!("  help                            -> Display command menu");
        println!("==================================================\n");

        let stdin = tokio::io::stdin();
        let mut reader = BufReader::new(stdin).lines();

        loop {
            print!("swarm-cli> ");
            use std::io::Write;
            std::io::stdout().flush().ok();

            let line = match reader.next_line().await {
                Ok(Some(line)) => line,
                Ok(None) => {
                    println!("\nCLI input closed; shutting down command terminal.");
                    break;
                }
                Err(error) => {
                    eprintln!("\nCLI input error: {error}");
                    break;
                }
            };
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.is_empty() {
                continue;
            }

            match parts[0].to_lowercase().as_str() {
                "mode" if parts.len() == 2 && parts[1].eq_ignore_ascii_case("guided") => {
                    match gps::set_guided_mode() {
                        Ok(()) => println!("✅ Drone 1 set to Guided mode."),
                        Err(error) => eprintln!("❌ Could not set Guided mode: {error}"),
                    }
                }
                "mode" => println!("❌ Usage: mode guided"),
                "arm" if parts.len() == 2 && parts[1].eq_ignore_ascii_case("throttle") => {
                    match gps::arm_throttle() {
                        Ok(()) => println!("✅ Drone 1 arm command sent."),
                        Err(error) => eprintln!("❌ Could not arm Drone 1: {error}"),
                    }
                }
                "arm" => println!("❌ Usage: arm throttle"),
                "takeoff" => {
                    let altitude_m = parts.get(1).and_then(|value| value.parse::<f32>().ok());
                    if parts.len() != 2
                        || !matches!(altitude_m, Some(value) if (0.0..=ORGANIZER_ALT_MAX_M).contains(&value))
                    {
                        println!(
                            "❌ Usage: takeoff <alt_m> (altitude must be within 0..={ORGANIZER_ALT_MAX_M}m)"
                        );
                        continue;
                    }

                    match gps::takeoff(altitude_m.expect("altitude was validated")) {
                        Ok(()) => println!("✅ Drone 1 takeoff command sent."),
                        Err(error) => eprintln!("❌ Could not command takeoff: {error}"),
                    }
                }
                "poi" => {
                    if parts.len() == 4 {
                        let north_m: Option<f64> = parts[1].parse().ok();
                        let east_m: Option<f64> = parts[2].parse().ok();
                        let alt_m: Option<f32> = parts[3].parse().ok();

                        if let (Some(n), Some(e), Some(a)) = (north_m, east_m, alt_m) {
                            let cur_lat = gps::LATITUDE_1.load(Ordering::Acquire) as f64 / 1e7;
                            let cur_lon = gps::LONGITUDE_1.load(Ordering::Acquire) as f64 / 1e7;

                            if n.abs() > GEOFENCE_HALF_EXTENT_M
                                || e.abs() > GEOFENCE_HALF_EXTENT_M
                                || !(0.0..=ORGANIZER_ALT_MAX_M).contains(&a)
                                || cur_lat == 0.0
                                || cur_lon == 0.0
                            {
                                println!("❌ Target rejected: wait for GPS lock and keep offsets within the 1000m geofence and altitude within 0..={ORGANIZER_ALT_MAX_M}m.");
                                continue;
                            }
                            let (target_lat, target_lon) = target_from_enu(cur_lat, cur_lon, n, e);

                            match gps::send_guided_target(target_lat, target_lon, a) {
                                    Ok(()) => println!(
                                        "🎯 Dispatched target: North {n}m, East {e}m (Lat {target_lat:.6}, Lon {target_lon:.6}, Alt {a}m)"
                                    ),
                                    Err(error) => eprintln!("❌ Target dispatch failed: {error}"),
                                }
                        } else {
                            println!(
                                "❌ Usage: poi <north_m> <east_m> <alt_m> (e.g., poi 20 10 15)"
                            );
                        }
                    } else {
                        println!("❌ Usage: poi <north_m> <east_m> <alt_m>");
                    }
                }
                "snap" | "image" => {
                    println!("📸 TRIGGERING CAMERA SNAPSHOT CAPTURE...");
                    if let Ok(socket) = UdpSocket::bind("127.0.0.1:0") {
                        let _ = socket.send_to(b"SNAP", "127.0.0.1:5002");
                    }
                }
                "status" => {
                    let lat1 = gps::LATITUDE_1.load(Ordering::Acquire) as f64 / 1e7;
                    let lon1 = gps::LONGITUDE_1.load(Ordering::Acquire) as f64 / 1e7;
                    let bat1 = gps::BATTERY_1.load(Ordering::Acquire);

                    let lat2 = gps::LATITUDE_2.load(Ordering::Acquire) as f64 / 1e7;
                    let lon2 = gps::LONGITUDE_2.load(Ordering::Acquire) as f64 / 1e7;
                    let bat2 = gps::BATTERY_2.load(Ordering::Acquire);

                    println!("\n--- FLEET TELEMETRY STATUS ---");
                    println!(
                        "🛸 Drone 1: Pos ({:.6}, {:.6}) | Bat: {}%",
                        lat1, lon1, bat1
                    );
                    println!(
                        "🛸 Drone 2: Pos ({:.6}, {:.6}) | Bat: {}%",
                        lat2, lon2, bat2
                    );
                    println!("------------------------------\n");
                }
                // FIX: no fault-injection mechanism existed anywhere, so the "Reconfigure
                // the network when links degrade, UAVs fail" requirement and the
                // Robustness/Fault-Recovery rubric items (15% of the score) could never
                // actually be exercised or measured. These commands let a tester (or an
                // automated Stage-2-style harness) trigger a fault on demand.
                "fail" => match parts.get(1).map(|s| s.to_lowercase()) {
                    Some(ref kind) if kind == "battery" && parts.len() == 4 => {
                        let id: Option<u8> = parts[2].parse().ok();
                        let pct: Option<u8> = parts[3].parse().ok().filter(|p| *p <= 100);
                        match (id, pct) {
                            (Some(1), Some(p)) => {
                                gps::BATTERY_1.store(p, Ordering::Release);
                                metrics::mark_fault_injected();
                                metrics::log_event(metrics::MetricEvent::FaultInjected {
                                    kind: "battery".into(),
                                    detail: format!("drone=1 percent={p}"),
                                })
                                .await;
                                println!("💥 Injected battery fault on Drone 1 -> {p}%");
                            }
                            (Some(2), Some(p)) => {
                                gps::BATTERY_2.store(p, Ordering::Release);
                                metrics::mark_fault_injected();
                                metrics::log_event(metrics::MetricEvent::FaultInjected {
                                    kind: "battery".into(),
                                    detail: format!("drone=2 percent={p}"),
                                })
                                .await;
                                println!("💥 Injected battery fault on Drone 2 -> {p}%");
                            }
                            _ => println!("❌ Usage: fail battery <1|2> <percent>"),
                        }
                    }
                    Some(ref kind) if kind == "link" && parts.len() == 2 => {
                        network::LINK_DOWN.store(true, Ordering::Release);
                        metrics::mark_fault_injected();
                        metrics::log_event(metrics::MetricEvent::FaultInjected {
                            kind: "link".into(),
                            detail: "simulated mesh outage".into(),
                        })
                        .await;
                        println!(
                            "💥 Simulated mesh link outage (mesh packets will now be dropped)."
                        );
                    }
                    _ => println!("❌ Usage: fail battery <1|2> <percent> | fail link"),
                },
                "recover" => {
                    network::LINK_DOWN.store(false, Ordering::Release);
                    println!("✅ Cleared simulated link outage.");
                }
                "help" => {
                    println!("Commands: mode guided | arm throttle | takeoff <alt_m> | poi <north_m> <east_m> <alt_m> | snap | status | fail battery <1|2> <percent> | fail link | recover | help");
                }
                _ => {
                    println!(
                        "❓ Unknown command: '{}'. Type 'help' for available commands.",
                        parts[0]
                    );
                }
            }
        }
    });

    // Dynamic non-blocking lock check
    poll_for_gps_lock().await;

    let orchestrator = tokio::spawn(async {
        let wave_mgr = WaveManager::new();

        let mut node_active = SwarmNode {
            id: 1,
            role: DroneRole::LeadSurveyor,
            position_enu: PositionEnu {
                east_m: 0.0,
                north_m: 0.0,
                up_m: 0.0,
            },
            battery: BatteryState::from_telemetry(&gps::BATTERY_1),
            current_poi: None,
            airborne_since: None,
        };

        let mut node_reserve = SwarmNode {
            id: 2,
            role: DroneRole::ReservePool,
            position_enu: PositionEnu {
                east_m: 0.0,
                north_m: 0.0,
                up_m: 0.0,
            },
            battery: BatteryState::from_telemetry(&gps::BATTERY_2),
            current_poi: None,
            airborne_since: None,
        };

        let mut prev_active_role = node_active.role;
        let mut prev_reserve_role = node_reserve.role;
        let origin_lat = gps::LATITUDE_1.load(Ordering::Acquire) as f64 / 1e7;
        let origin_lon = gps::LONGITUDE_1.load(Ordering::Acquire) as f64 / 1e7;

        // --- Mission / PoI setup -----------------------------------------------------
        // FIX: this whole block is new. Previously nothing ever created a mission,
        // spawned PoIs, or assigned/dispatched/completed them autonomously.
        let mut mission_mgr = mission::MissionManager::new();
        let mission_id = mission_mgr.create_mission();
        let mut rng = Xorshift64::new(
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(1),
        );
        // Matches the organizer sample scenario: 10 PoIs, spawned at random positions
        // and random times across the mission window, inside the operational area.
        let mut pending_pois = spawn_random_pois(
            10,
            GEOFENCE_HALF_EXTENT_M * 0.9,
            15.0,
            Duration::from_secs(40 * 60),
            &mut rng,
        );
        let mission_started_at = Instant::now();
        metrics::log_event(metrics::MetricEvent::MissionStart {
            mission_id,
            poi_count: pending_pois.len(),
        })
        .await;

        let mut relay_reallocations: u32 = 0;
        let mut last_avoidance_at: Option<Instant> = None;
        let mut tick_count: u64 = 0;
        // FIX: previously "minimum inter-UAV separation" was only ever the configured
        // 20 m threshold, never a measured value from an actual run.
        let mut min_separation_m: Option<f64> = None;
        let mut mission_expired = false;

        loop {
            let lat1 = gps::LATITUDE_1.load(Ordering::Acquire) as f64 / 1e7;
            let lon1 = gps::LONGITUDE_1.load(Ordering::Acquire) as f64 / 1e7;
            if lat1 != 0.0 && lon1 != 0.0 {
                node_active.position_enu = enu_from_global(
                    origin_lat,
                    origin_lon,
                    lat1,
                    lon1,
                    gps::ALTITUDE_1_MM.load(Ordering::Acquire) as f64 / 1000.0,
                );
            }
            node_active.battery.percentage = gps::BATTERY_1.load(Ordering::Acquire) as f32;
            if node_active.position_enu.up_m > 2.0 {
                node_active.airborne_since.get_or_insert(Instant::now());
            }

            let lat2 = gps::LATITUDE_2.load(Ordering::Acquire) as f64 / 1e7;
            let lon2 = gps::LONGITUDE_2.load(Ordering::Acquire) as f64 / 1e7;
            if lat2 != 0.0 && lon2 != 0.0 {
                node_reserve.position_enu = enu_from_global(
                    origin_lat,
                    origin_lon,
                    lat2,
                    lon2,
                    gps::ALTITUDE_2_MM.load(Ordering::Acquire) as f64 / 1000.0,
                );
            }
            node_reserve.battery.percentage = gps::BATTERY_2.load(Ordering::Acquire) as f32;
            if node_reserve.position_enu.up_m > 2.0 {
                node_reserve.airborne_since.get_or_insert(Instant::now());
            }

            // Enforce the organizer's 45-minute mission window. Once the deadline
            // is reached, stop creating/dispatching work and command both vehicles
            // home; this prevents a run from quietly violating the challenge limits.
            if !mission_expired && mission_started_at.elapsed() >= MISSION_DURATION {
                mission_expired = true;
                pending_pois.clear();
                node_active.current_poi = None;
                node_reserve.current_poi = None;
                node_active.role = DroneRole::ReturnToBase;
                node_reserve.role = DroneRole::ReturnToBase;
                for drone_id in [node_active.id, node_reserve.id] {
                    if let Err(error) = gps::return_to_launch_for(drone_id) {
                        eprintln!("❌ Mission deadline RTL failed for Drone {drone_id}: {error}");
                    }
                }
                println!("\n⏱️ Mission window expired; all drones commanded to return to base.");
            }
            if mission_expired {
                sleep(Duration::from_secs(1)).await;
                continue;
            }

            // --- Newly emerging PoIs -------------------------------------------------
            let now_instant = Instant::now();
            let mut newly_spawned = Vec::new();
            pending_pois.retain(|p| {
                if p.spawn_at <= now_instant {
                    newly_spawned.push((p.north_m, p.east_m, p.alt_m, p.priority));
                    false
                } else {
                    true
                }
            });
            for (north_m, east_m, alt_m, priority) in newly_spawned {
                let _ = mission_mgr.add_poi(mission_id, north_m, east_m, alt_m, priority);
                println!(
                    "\n🆕 New PoI spawned at N{north_m:.0}m E{east_m:.0}m (priority {priority:?})"
                );
            }

            // --- Autonomous PoI completion, stall-release, assignment/dispatch -------
            try_complete_poi(&mut node_active, &mut mission_mgr, mission_id).await;
            try_complete_poi(&mut node_reserve, &mut mission_mgr, mission_id).await;
            release_if_stalled(&mut node_active, &mut mission_mgr, mission_id).await;
            release_if_stalled(&mut node_reserve, &mut mission_mgr, mission_id).await;
            try_assign_and_dispatch(
                &mut node_active,
                &mut mission_mgr,
                mission_id,
                origin_lat,
                origin_lon,
            )
            .await;
            try_assign_and_dispatch(
                &mut node_reserve,
                &mut mission_mgr,
                mission_id,
                origin_lat,
                origin_lon,
            )
            .await;

            let reserve_is_flying = node_reserve.role != DroneRole::ReservePool;
            if reserve_is_flying {
                let distance_m =
                    peer::enu_distance_meters(node_active.position_enu, node_reserve.position_enu);
                min_separation_m =
                    Some(min_separation_m.map_or(distance_m, |m: f64| m.min(distance_m)));
            }
            if reserve_is_flying
                && peer::is_separation_breach(node_active.position_enu, node_reserve.position_enu)
            {
                let distance_m =
                    peer::enu_distance_meters(node_active.position_enu, node_reserve.position_enu);
                println!(
                    "\n⚠️ WARNING: Swarm nodes 1 & 2 breach the 20m safety separation threshold!"
                );
                metrics::log_event(metrics::MetricEvent::SeparationBreach { distance_m }).await;

                // FIX: this used to be warning-only. Now the trailing node is actually
                // commanded away from the other node (a simple repulsive nudge), subject
                // to a cooldown so it doesn't spam corrective commands every tick.
                let cooldown_elapsed = last_avoidance_at
                    .map(|t| t.elapsed() >= Duration::from_secs(5))
                    .unwrap_or(true);
                if cooldown_elapsed {
                    let dx = node_reserve.position_enu.east_m - node_active.position_enu.east_m;
                    let dy = node_reserve.position_enu.north_m - node_active.position_enu.north_m;
                    let mag = (dx * dx + dy * dy).sqrt().max(1.0);
                    let push_m = 15.0;
                    let new_east = (node_reserve.position_enu.east_m + dx / mag * push_m)
                        .clamp(-GEOFENCE_HALF_EXTENT_M, GEOFENCE_HALF_EXTENT_M);
                    let new_north = (node_reserve.position_enu.north_m + dy / mag * push_m)
                        .clamp(-GEOFENCE_HALF_EXTENT_M, GEOFENCE_HALF_EXTENT_M);
                    let (avoid_lat, avoid_lon) =
                        target_from_enu(origin_lat, origin_lon, new_north, new_east);
                    match gps::send_guided_target_for(
                        node_reserve.id,
                        avoid_lat,
                        avoid_lon,
                        node_reserve.position_enu.up_m as f32,
                    ) {
                        Ok(()) => {
                            println!(
                                "🛡️  Collision-avoidance maneuver: nudging Drone {} clear of Drone {}.",
                                node_reserve.id, node_active.id
                            );
                            metrics::log_event(metrics::MetricEvent::CollisionAvoidanceManeuver {
                                drone_id: node_reserve.id,
                            })
                            .await;
                        }
                        Err(error) => eprintln!("❌ Collision-avoidance maneuver failed: {error}"),
                    }
                    last_avoidance_at = Some(Instant::now());
                }
            } else if reserve_is_flying
                && peer::needs_link_throttle(node_active.position_enu, node_reserve.position_enu)
            {
                let distance_m =
                    peer::enu_distance_meters(node_active.position_enu, node_reserve.position_enu);
                println!("\n⚠️ WARNING: Swarm link is at/over the 85m throttle boundary (100m hard range).");
                metrics::log_event(metrics::MetricEvent::LinkWarning { distance_m }).await;
            }

            let distance_to_base = (node_active.position_enu.east_m.powi(2)
                + node_active.position_enu.north_m.powi(2))
            .sqrt();

            wave_mgr.evaluate_handoff(&mut node_active, &mut node_reserve, distance_to_base);

            if node_active.role != prev_active_role {
                let from_role = format!("{:?}", prev_active_role);
                let to_role = format!("{:?}", node_active.role);
                match node_active.role {
                    DroneRole::StationaryTower => {
                        println!("\n📡 Active Drone 1 transitioned to STATIONARY TOWER mode.");
                        relay_reallocations += 1;
                    }
                    DroneRole::ReturnToBase => {
                        println!("\n🚨 Battery critical! Drone 1 executing Return-To-Base (RTL).")
                    }
                    _ => {}
                }
                metrics::log_event(metrics::MetricEvent::RelayReallocation {
                    drone_id: node_active.id,
                    from_role,
                    to_role,
                })
                .await;
                prev_active_role = node_active.role;
            }

            if node_reserve.role != prev_reserve_role {
                let from_role = format!("{:?}", prev_reserve_role);
                let to_role = format!("{:?}", node_reserve.role);
                if node_reserve.role == DroneRole::LeadSurveyor {
                    println!("\n🛫 Reserve Drone 2 DISPATCHED early to cover transit lag!");
                    relay_reallocations += 1;
                    // FIX: this is the "Autonomy: recovery time" metric — previously
                    // never measured anywhere. If a fault was injected (via `fail
                    // battery`/`fail link`, or a natural battery-driven handoff), this
                    // records how long it took the swarm to visibly reconfigure.
                    if let Some(seconds) = metrics::take_recovery_seconds() {
                        println!("⏱️  Recovery time since fault injection: {seconds:.2}s");
                        metrics::log_event(metrics::MetricEvent::RecoveryTime {
                            drone_id: node_reserve.id,
                            seconds,
                        })
                        .await;
                    }
                }
                metrics::log_event(metrics::MetricEvent::RelayReallocation {
                    drone_id: node_reserve.id,
                    from_role,
                    to_role,
                })
                .await;
                prev_reserve_role = node_reserve.role;
            }

            // --- Periodic mission-progress snapshot (Mission Completion metric) ------
            tick_count += 1;
            if tick_count % 20 == 0 {
                if let Some(pois) = mission_mgr.get_all_pois(mission_id) {
                    let total = pois.len();
                    let completed = pois
                        .iter()
                        .filter(|p| matches!(p.state, mission::PoIState::Completed))
                        .count();
                    let failed = pois
                        .iter()
                        .filter(|p| matches!(p.state, mission::PoIState::Failed))
                        .count();

                    // FIX: "priority-weighted mission score" was never computed anywhere.
                    // Weight High=3, Medium=2, Low=1; score = weighted-completed /
                    // weighted-total, so finishing high-priority PoIs counts for more
                    // than an equal number of low-priority ones.
                    fn weight(priority: mission::Priority) -> u32 {
                        match priority {
                            mission::Priority::High => 3,
                            mission::Priority::Medium => 2,
                            mission::Priority::Low => 1,
                        }
                    }
                    let total_weight: u32 = pois.iter().map(|p| weight(p.priority)).sum();
                    let completed_weight: u32 = pois
                        .iter()
                        .filter(|p| matches!(p.state, mission::PoIState::Completed))
                        .map(|p| weight(p.priority))
                        .sum();
                    let priority_weighted_score_pct = if total_weight > 0 {
                        100.0 * completed_weight as f32 / total_weight as f32
                    } else {
                        0.0
                    };

                    println!(
                        "\n📊 Mission progress: {completed}/{total} PoIs complete ({priority_weighted_score_pct:.1}% priority-weighted), {failed} failed, {relay_reallocations} relay reallocation(s), min separation {} so far.",
                        min_separation_m.map(|m| format!("{m:.1}m")).unwrap_or_else(|| "n/a".into())
                    );
                    metrics::log_event(metrics::MetricEvent::MissionProgress {
                        mission_id,
                        total,
                        completed,
                        failed,
                        elapsed_s: mission_started_at.elapsed().as_secs_f64(),
                        priority_weighted_score_pct,
                        min_separation_m,
                    })
                    .await;
                }
            }

            sleep(Duration::from_millis(500)).await;
        }
    });

    let _ = tokio::join!(t1, t2, t3, orchestrator, cli);
}
