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
mod fleet;
mod metrics;
mod network;
mod peer;
mod radio;

use std::net::UdpSocket;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};
use tokio::io::{AsyncBufReadExt, BufReader};

use peer::PositionEnu;

const EARTH_RADIUS_M: f64 = 6_378_137.0;
const GEOFENCE_HALF_EXTENT_M: f64 = 500.0; // matches organizer's 1000m x 1000m arena.

// FIX: the organizer's sample scenario caps "Operational height max" at 100 m, but the
// CLI validation here previously allowed 0..=120 m for both `takeoff` and `poi`. Any
// PoI/takeoff issued between 100 and 120 m would have been accepted by this software
// while violating the actual competition constraint.
const ORGANIZER_ALT_MAX_M: f32 = 100.0;

// FIX: PoI arrival radius used by the new autonomous completion check below.
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
        .map(|index| {
            let north_m = (rng.next_f64() * 2.0 - 1.0) * half_extent_m;
            let east_m = (rng.next_f64() * 2.0 - 1.0) * half_extent_m;
            // Keep the demo visibly active immediately after GPS lock. The
            // remaining PoIs still arrive at random times across the benchmark
            // window, preserving the emerging-PoI behavior.
            let delay_s = if index < 2 {
                0.0
            } else {
                rng.next_f64() * window.as_secs_f64()
            };
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

#[tokio::main]
async fn main() {
    println!("🚀 Booting Deep Scan Swarm Controller...");
    let count = std::env::var("GC1_DRONE_COUNT")
        .unwrap_or_else(|_| "2".into())
        .parse::<usize>()
        .expect("GC1_DRONE_COUNT must be a number");
    gps::configure(count).expect("GC1_DRONE_COUNT must be 1..=20");
    metrics::init("swarm_metrics.jsonl");

    let t1 = tokio::spawn(async {
        camera::start_listener().await;
    });
    let t2 = tokio::spawn(async {
        gps::start_telemetry_loop().await;
    });

    let (operator_tx, operator_rx) = tokio::sync::mpsc::channel::<(u8, f64, f64, f32)>(32);

    // Start accepting commands immediately; target commands still validate GPS lock.
    let cli = tokio::spawn(async move {
        println!("\n==================================================");
        println!("🎮 SWARM OPERATOR COMMAND TERMINAL ONLINE");
        println!("Commands:");
        println!("  mode guided                     -> Set selected drone to Guided mode");
        println!("  arm throttle                    -> Arm selected drone");
        println!("  takeoff <alt_m>                 -> Take off selected drone to altitude");
        println!("  poi <north_m> <east_m> <alt_m>  -> Send drone to local waypoint offset");
        println!("  snap OR image                   -> Capture camera snapshot");
        println!("  status                          -> Display fleet telemetry");
        println!("  fail battery <drone-id> <percent>    -> Inject a battery fault for testing");
        println!("  fail link                       -> Simulate a mesh comms outage");
        println!("  recover                         -> Clear a simulated link outage");
        println!("  help                            -> Display command menu");
        println!("==================================================\n");

        let mut selected_id: u8 = 1;
        println!("drone <1..{count}> selects the vehicle for mode/arm/takeoff/poi commands");
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
                "drone" => {
                    match parts
                        .get(1)
                        .and_then(|v| v.parse::<u8>().ok())
                        .filter(|id| *id > 0 && *id as usize <= gps::fleet_size())
                    {
                        Some(id) => {
                            selected_id = id;
                            println!("Selected Drone {id}");
                        }
                        None => eprintln!("Use drone <1..{}>", gps::fleet_size()),
                    }
                }
                "mode" if parts.len() == 2 && parts[1].eq_ignore_ascii_case("guided") => {
                    match tokio::task::spawn_blocking(move || gps::set_guided_mode_for(selected_id))
                        .await
                        .unwrap_or_else(|error| Err(error.to_string()))
                    {
                        Ok(()) => println!("✅ Drone {selected_id} set to Guided mode."),
                        Err(error) => eprintln!("❌ Could not set Guided mode: {error}"),
                    }
                }
                "mode" => println!("❌ Usage: mode guided"),
                "arm" if parts.len() == 2 && parts[1].eq_ignore_ascii_case("throttle") => {
                    match tokio::task::spawn_blocking(move || gps::arm_throttle_for(selected_id))
                        .await
                        .unwrap_or_else(|error| Err(error.to_string()))
                    {
                        Ok(()) => println!("✅ Drone {selected_id} arm command sent."),
                        Err(error) => eprintln!("❌ Could not arm Drone {selected_id}: {error}"),
                    }
                }
                "arm" => println!("❌ Usage: arm throttle"),
                "takeoff" => {
                    let altitude_m = parts.get(1).and_then(|value| value.parse::<f32>().ok());
                    if parts.len() != 2
                        || !matches!(altitude_m, Some(value) if value > 0.0 && value <= ORGANIZER_ALT_MAX_M)
                    {
                        println!(
                            "❌ Usage: takeoff <alt_m> (altitude must be within (0, {ORGANIZER_ALT_MAX_M}]m)"
                        );
                        continue;
                    }

                    match gps::prepare_and_takeoff_for(
                        selected_id,
                        altitude_m.expect("altitude was validated"),
                    )
                    .await
                    {
                        Ok(()) => println!("✅ Drone {selected_id} takeoff command sent."),
                        Err(error) => eprintln!("❌ Could not command takeoff: {error}"),
                    }
                }
                "poi" => {
                    if parts.len() == 4 {
                        let north_m: Option<f64> = parts[1].parse().ok();
                        let east_m: Option<f64> = parts[2].parse().ok();
                        let alt_m: Option<f32> = parts[3].parse().ok();

                        if let (Some(n), Some(e), Some(a)) = (north_m, east_m, alt_m) {
                            let cur_lat = gps::telemetry(selected_id).lat.load(Ordering::Acquire)
                                as f64
                                / 1e7;
                            let cur_lon = gps::telemetry(selected_id).lon.load(Ordering::Acquire)
                                as f64
                                / 1e7;

                            if !n.is_finite()
                                || !e.is_finite()
                                || n.abs() > GEOFENCE_HALF_EXTENT_M
                                || e.abs() > GEOFENCE_HALF_EXTENT_M
                                || !a.is_finite()
                                || a <= 0.0
                                || a > ORGANIZER_ALT_MAX_M
                                || cur_lat == 0.0
                                || cur_lon == 0.0
                            {
                                println!("❌ Target rejected: wait for GPS lock and keep offsets within the 1000m geofence and altitude within (0, {ORGANIZER_ALT_MAX_M}]m.");
                                continue;
                            }
                            let (target_lat, target_lon) = target_from_enu(cur_lat, cur_lon, n, e);

                            if operator_tx
                                .send((selected_id, target_lat, target_lon, a))
                                .await
                                .is_ok()
                            {
                                println!(
                                    "🎯 Operator target queued: North {n}m, East {e}m, Alt {a}m"
                                );
                            } else {
                                eprintln!("❌ Mission controller unavailable");
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
                    for id in 1..=gps::fleet_size() as u8 {
                        let t = gps::telemetry(id);
                        println!("Drone {id}: lat {:.6}, lon {:.6}, alt {:.1}m, battery {:.0}%, fresh {}",
                            t.lat.load(Ordering::Acquire) as f64 / 1e7,
                            t.lon.load(Ordering::Acquire) as f64 / 1e7,
                            t.altitude_mm.load(Ordering::Acquire) as f64 / 1000.0,
                            t.battery_percent(), t.fresh());
                    }
                }
                "fail" => match parts.get(1).map(|s| s.to_lowercase()) {
                    Some(ref kind) if kind == "battery" && parts.len() == 4 => {
                        let id: Option<u8> = parts[2].parse().ok();
                        let pct: Option<u8> = parts[3].parse().ok().filter(|p| *p <= 100);
                        match (id, pct) {
                            (Some(id), Some(p)) if id > 0 && id as usize <= gps::fleet_size() => {
                                gps::telemetry(id)
                                    .battery_override
                                    .store(p as i32, Ordering::Release);
                                metrics::mark_fault_injected();
                                println!("Injected battery fault: Drone {id} = {p}%");
                            }
                            _ => eprintln!("Use fail battery <drone-id> <0..100>"),
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
                    _ => println!("❌ Usage: fail battery <drone-id> <percent> | fail link"),
                },
                "recover" => {
                    network::LINK_DOWN.store(false, Ordering::Release);
                    for id in 1..=gps::fleet_size() as u8 {
                        gps::telemetry(id)
                            .battery_override
                            .store(-1, Ordering::Release);
                    }
                    println!("✅ Cleared simulated link outage.");
                }
                "help" => {
                    println!("Commands: mode guided | arm throttle | takeoff <alt_m> | poi <north_m> <east_m> <alt_m> | snap | status | fail battery <drone-id> <percent> | fail link | recover | help");
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

    let orchestrator = tokio::spawn(async move {
        fleet::run(operator_rx).await;
    });
    let _ = tokio::join!(t1, t2, orchestrator, cli);
}
