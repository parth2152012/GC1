mod bat_management;
mod camera;
mod deep_scan;
mod gps;
mod network;
mod peer;

use std::net::UdpSocket;
use std::sync::atomic::Ordering;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::time::sleep;

use bat_management::BatteryState;
use deep_scan::{DroneRole, SwarmNode, WaveManager};
use peer::PositionEnu;

const EARTH_RADIUS_M: f64 = 6_378_137.0;
const GEOFENCE_HALF_EXTENT_M: f64 = 500.0;

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

    let t1 = tokio::spawn(async {
        camera::start_listener().await;
    });
    let t2 = tokio::spawn(async {
        gps::start_telemetry_loop().await;
    });
    let t3 = tokio::spawn(async {
        network::start_routing_mesh().await;
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
            battery: BatteryState::new(),
        };

        let mut node_reserve = SwarmNode {
            id: 2,
            role: DroneRole::ReservePool,
            position_enu: PositionEnu {
                east_m: 0.0,
                north_m: 0.0,
                up_m: 0.0,
            },
            battery: BatteryState::new(),
        };

        let mut prev_active_role = node_active.role;
        let mut prev_reserve_role = node_reserve.role;
        let origin_lat = gps::LATITUDE_1.load(Ordering::Acquire) as f64 / 1e7;
        let origin_lon = gps::LONGITUDE_1.load(Ordering::Acquire) as f64 / 1e7;

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

            let reserve_is_flying = node_reserve.role != DroneRole::ReservePool;
            if reserve_is_flying
                && peer::is_separation_breach(node_active.position_enu, node_reserve.position_enu)
            {
                println!(
                    "\n⚠️ WARNING: Swarm nodes 1 & 2 breach the 20m safety separation threshold!"
                );
            } else if reserve_is_flying
                && peer::needs_link_throttle(node_active.position_enu, node_reserve.position_enu)
            {
                println!("\n⚠️ WARNING: Swarm link is at/over the 85m throttle boundary (100m hard range).");
            }

            let distance_to_base = (node_active.position_enu.east_m.powi(2)
                + node_active.position_enu.north_m.powi(2))
            .sqrt();

            wave_mgr.evaluate_handoff(&mut node_active, &mut node_reserve, distance_to_base);

            if node_active.role != prev_active_role {
                match node_active.role {
                    DroneRole::StationaryTower => {
                        println!("\n📡 Active Drone 1 transitioned to STATIONARY TOWER mode.")
                    }
                    DroneRole::ReturnToBase => {
                        println!("\n🚨 Battery critical! Drone 1 executing Return-To-Base (RTL).")
                    }
                    _ => {}
                }
                prev_active_role = node_active.role;
            }

            if node_reserve.role != prev_reserve_role {
                match node_reserve.role {
                    DroneRole::LeadSurveyor => {
                        println!("\n🛫 Reserve Drone 2 DISPATCHED early to cover transit lag!")
                    }
                    _ => {}
                }
                prev_reserve_role = node_reserve.role;
            }

            sleep(Duration::from_millis(500)).await;
        }
    });

    let cli = tokio::spawn(async {
        println!("\n==================================================");
        println!("🎮 SWARM OPERATOR COMMAND TERMINAL ONLINE");
        println!("Commands:");
        println!("  poi <north_m> <east_m> <alt_m>  -> Send drone to local waypoint offset");
        println!("  snap OR image                   -> Capture camera snapshot");
        println!("  status                          -> Display fleet telemetry");
        println!("  help                            -> Display command menu");
        println!("==================================================\n");

        let stdin = tokio::io::stdin();
        let mut reader = BufReader::new(stdin).lines();

        loop {
            print!("swarm-cli> ");
            use std::io::Write;
            std::io::stdout().flush().ok();

            if let Ok(Some(line)) = reader.next_line().await {
                let parts: Vec<&str> = line.trim().split_whitespace().collect();
                if parts.is_empty() {
                    continue;
                }

                match parts[0].to_lowercase().as_str() {
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
                                    || !(0.0..=120.0).contains(&a)
                                    || cur_lat == 0.0
                                    || cur_lon == 0.0
                                {
                                    println!("❌ Target rejected: wait for GPS lock and keep offsets within the 1000m geofence and altitude within 0..=120m.");
                                    continue;
                                }
                                let (target_lat, target_lon) =
                                    target_from_enu(cur_lat, cur_lon, n, e);

                                println!("🎯 DISPATCHING Target Offset: North {}m, East {}m (Lat {:.6}, Lon {:.6}, Alt {}m)", n, e, target_lat, target_lon, a);
                                gps::send_guided_target(target_lat, target_lon, a);
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
                            let _ = socket.send_to(b"SNAP", "127.0.0.1:5001");
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
                    "help" => {
                        println!("Commands: poi <north_m> <east_m> <alt_m> | snap | status | help");
                    }
                    _ => {
                        println!(
                            "❓ Unknown command: '{}'. Type 'help' for available commands.",
                            parts[0]
                        );
                    }
                }
            }
        }
    });

    let _ = tokio::join!(t1, t2, t3, orchestrator, cli);
}
