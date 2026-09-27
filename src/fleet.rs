use crate::{
    bat_management::BatteryState,
    deep_scan::{DroneRole, SwarmNode},
    gps, metrics,
    mission::{MissionManager, PoIState, Priority},
    peer::{enu_distance_meters, PositionEnu},
    radio::Radio,
};
use std::{
    sync::atomic::Ordering,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use tokio::{sync::mpsc, task::JoinHandle, time::sleep};

struct Vehicle {
    node: SwarmNode,
    flight: Option<JoinHandle<Result<(), String>>>,
    report: Option<JoinHandle<Result<usize, String>>>,
    retry_at: Instant,
    returning: bool,
    rtl_sent: bool,
}
fn start_target(v: &mut Vehicle, target: PositionEnu, origin: (f64, f64)) {
    let id = v.node.id;
    let (lat, lon) = crate::target_from_enu(origin.0, origin.1, target.north_m, target.east_m);
    v.flight = Some(tokio::spawn(async move {
        gps::prepare_and_send_guided_target_for(id, lat, lon, target.up_m as f32).await
    }));
}
fn point(v: &gps::Telemetry, origin: (f64, f64)) -> PositionEnu {
    crate::enu_from_global(
        origin.0,
        origin.1,
        v.lat.load(Ordering::Acquire) as f64 / 1e7,
        v.lon.load(Ordering::Acquire) as f64 / 1e7,
        v.altitude_mm.load(Ordering::Acquire) as f64 / 1000.0,
    )
}
fn chain(target: PositionEnu) -> Vec<PositionEnu> {
    let steps = ((target.north_m.hypot(target.east_m) / 75.0).ceil() as usize).max(1);
    (1..=steps)
        .map(|step| PositionEnu {
            north_m: target.north_m * step as f64 / steps as f64,
            east_m: target.east_m * step as f64 / steps as f64,
            up_m: target.up_m,
        })
        .collect()
}

pub async fn run(mut commands: mpsc::Receiver<(u8, f64, f64, f32)>) {
    println!(
        "Fleet controller: {} vehicles; waiting for Drone 1 origin",
        gps::fleet_size()
    );
    while !gps::telemetry(1).fresh() {
        sleep(Duration::from_millis(250)).await;
    }
    let origin = (
        gps::telemetry(1).lat.load(Ordering::Acquire) as f64 / 1e7,
        gps::telemetry(1).lon.load(Ordering::Acquire) as f64 / 1e7,
    );
    let radio = match Radio::start(gps::fleet_size(), 16000).await {
        Ok(radio) => radio,
        Err(e) => {
            eprintln!("Cannot start radio relay endpoints: {e}");
            return;
        }
    };
    let mut vehicles: Vec<_> = (1..=gps::fleet_size() as u8)
        .map(|id| Vehicle {
            node: SwarmNode {
                id,
                role: DroneRole::ReservePool,
                position_enu: PositionEnu {
                    north_m: 0.0,
                    east_m: 0.0,
                    up_m: 0.0,
                },
                battery: BatteryState {
                    percentage: 100.0,
                    discharge_rate_per_min: 5.0,
                },
                current_poi: None,
                airborne_since: None,
            },
            flight: None,
            report: None,
            retry_at: Instant::now(),
            returning: false,
            rtl_sent: false,
        })
        .collect();
    let mut manager = MissionManager::new();
    let mission_id = manager.create_mission();
    let mut rng = crate::Xorshift64::new(
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64,
    );
    let mut pending =
        crate::spawn_random_pois(10, 450.0, 15.0, Duration::from_secs(2400), &mut rng);
    let began = Instant::now();
    metrics::log_event(metrics::MetricEvent::MissionStart {
        mission_id,
        poi_count: 10,
    })
    .await;
    let mut tick = 0;
    loop {
        radio
            .update(
                0,
                Some(PositionEnu {
                    north_m: 0.0,
                    east_m: 0.0,
                    up_m: 0.0,
                }),
            )
            .await;
        for vehicle in &mut vehicles {
            let id = vehicle.node.id;
            let telemetry = gps::telemetry(id);
            let fresh = telemetry.fresh();
            if fresh {
                vehicle.node.position_enu = point(telemetry, origin);
                vehicle.node.battery.percentage = telemetry.battery_percent();
                if vehicle.node.position_enu.up_m > 2.0 {
                    vehicle.node.airborne_since.get_or_insert(Instant::now());
                }
            }
            radio
                .update(
                    id as usize,
                    if fresh {
                        Some(vehicle.node.position_enu)
                    } else {
                        None
                    },
                )
                .await;
            if vehicle.flight.as_ref().is_some_and(|job| job.is_finished()) {
                let result = vehicle
                    .flight
                    .take()
                    .unwrap()
                    .await
                    .unwrap_or_else(|e| Err(e.to_string()));
                match result {
                    Ok(()) => {
                        if vehicle.returning {
                            vehicle.rtl_sent = true;
                        } else {
                            println!(
                                "Flight target accepted by Drone {id} ({:?})",
                                vehicle.node.role
                            );
                        }
                    }
                    Err(e) => {
                        eprintln!("Drone {id} dispatch failed: {e}");
                        if let Some(poi) = vehicle.node.current_poi.take() {
                            let _ = manager.release_stalled_poi(mission_id, poi);
                        }
                        if !vehicle.returning {
                            vehicle.node.role = DroneRole::ReservePool;
                        }
                        vehicle.retry_at = Instant::now() + Duration::from_secs(5);
                    }
                }
            }
            // Reserve return time rather than initiating RTL at the landing deadline.
            let return_budget = Duration::from_secs_f64(
                vehicle
                    .node
                    .position_enu
                    .north_m
                    .hypot(vehicle.node.position_enu.east_m)
                    / 4.5
                    + 45.0,
            );
            let due = began.elapsed() + return_budget >= crate::MISSION_DURATION
                || vehicle.node.airborne_since.is_some_and(|time| {
                    time.elapsed() + return_budget >= Duration::from_secs(20 * 60)
                })
                || vehicle.node.battery.must_return_to_base();
            if due && !vehicle.returning {
                vehicle.returning = true;
                vehicle.node.role = DroneRole::ReturnToBase;
                if let Some(poi) = vehicle.node.current_poi.take() {
                    let _ = manager.release_stalled_poi(mission_id, poi);
                }
                if let Some(report) = vehicle.report.take() {
                    report.abort();
                }
            }
            if vehicle.returning
                && !vehicle.rtl_sent
                && vehicle.flight.is_none()
                && Instant::now() >= vehicle.retry_at
            {
                vehicle.flight = Some(tokio::spawn(async move {
                    tokio::task::spawn_blocking(move || gps::return_to_launch_for(id))
                        .await
                        .map_err(|e| e.to_string())?
                }));
            }
            if vehicle.report.as_ref().is_some_and(|job| job.is_finished()) {
                let result = vehicle
                    .report
                    .take()
                    .unwrap()
                    .await
                    .unwrap_or_else(|e| Err(e.to_string()));
                if let Some(poi) = vehicle.node.current_poi {
                    match result {
                        Ok(hops) => {
                            let _ = manager.mark_poi_completed(mission_id, poi);
                            metrics::log_event(metrics::MetricEvent::PoiCompleted {
                                mission_id,
                                poi_id: poi,
                                drone_id: id,
                            })
                            .await;
                            println!(
                                "Drone {id} completed PoI #{poi}: center ACK via {hops} radio hops"
                            );
                            vehicle.node.current_poi = None;
                            vehicle.node.role = DroneRole::ReservePool;
                        }
                        Err(e) => {
                            eprintln!("Drone {id} PoI #{poi} awaiting report route: {e}");
                            vehicle.retry_at = Instant::now() + Duration::from_secs(3);
                        }
                    }
                }
            }
            if fresh
                && !vehicle.returning
                && vehicle.flight.is_none()
                && vehicle.report.is_none()
                && Instant::now() >= vehicle.retry_at
            {
                if let Some(poi) = vehicle.node.current_poi.and_then(|id| {
                    manager
                        .get_all_pois(mission_id)?
                        .into_iter()
                        .find(|p| p.id == id)
                }) {
                    let target = PositionEnu {
                        north_m: poi.north_m,
                        east_m: poi.east_m,
                        up_m: poi.alt_m as f64,
                    };
                    if enu_distance_meters(vehicle.node.position_enu, target) <= 10.0 {
                        let mesh = radio.clone();
                        vehicle.report = Some(tokio::spawn(async move {
                            mesh.report(id as usize, format!("poi={} detected_by={id}", poi.id))
                                .await
                        }));
                    }
                }
            }
        }
        // Keep random spawn timing independent of vehicle takeoff/ACK waits.
        pending.retain(|p| {
            if p.spawn_at <= Instant::now() {
                let _ = manager.add_poi(mission_id, p.north_m, p.east_m, p.alt_m, p.priority);
                println!("New PoI spawned at N{:.0} E{:.0}", p.north_m, p.east_m);
                false
            } else {
                true
            }
        });
        if let Ok((id, lat, lon, alt)) = commands.try_recv() {
            let vehicle = &mut vehicles[id as usize - 1];
            if vehicle.flight.is_some() || vehicle.returning || vehicle.report.is_some() {
                eprintln!("Drone {id} busy or returning; operator target rejected");
            } else {
                if let Some(old) = vehicle.node.current_poi.take() {
                    let _ = manager.release_stalled_poi(mission_id, old);
                }
                let target = crate::enu_from_global(origin.0, origin.1, lat, lon, alt as f64);
                let poi = manager
                    .add_poi(
                        mission_id,
                        target.north_m,
                        target.east_m,
                        alt,
                        Priority::High,
                    )
                    .unwrap();
                manager.mark_poi_in_progress(mission_id, poi, id).unwrap();
                vehicle.node.current_poi = Some(poi);
                vehicle.node.role = DroneRole::LeadSurveyor;
                start_target(vehicle, target, origin);
                println!("Drone {id} dispatched to operator PoI #{poi}");
            }
        }
        // Allocate a relay corridor and a surveyor from the entire fleet.
        if began.elapsed() < crate::MISSION_DURATION - Duration::from_secs(180) {
            let mut candidates: Vec<_> = manager
                .get_all_pois(mission_id)
                .unwrap()
                .into_iter()
                .filter(|p| matches!(p.state, PoIState::Pending))
                .collect();
            candidates.sort_by_key(|p| {
                (
                    match p.priority {
                        Priority::High => 0,
                        Priority::Medium => 1,
                        Priority::Low => 2,
                    },
                    p.id,
                )
            });
            for poi in candidates {
                let target = PositionEnu {
                    north_m: poi.north_m,
                    east_m: poi.east_m,
                    up_m: poi.alt_m as f64,
                };
                let waypoints = chain(target);
                let mut free: Vec<_> = vehicles
                    .iter()
                    .enumerate()
                    .filter(|(_, v)| {
                        !v.returning
                            && v.node.role == DroneRole::ReservePool
                            && v.flight.is_none()
                            && v.node.current_poi.is_none()
                            && v.node.battery.percentage > 20.0
                            && gps::telemetry(v.node.id).fresh()
                            && Instant::now() >= v.retry_at
                    })
                    .map(|(i, _)| i)
                    .collect();
                let mut plan = Vec::new();
                for (step, waypoint) in waypoints.iter().enumerate() {
                    let last = step + 1 == waypoints.len();
                    if !last
                        && vehicles.iter().any(|v| {
                            v.node.role == DroneRole::StationaryTower
                                && gps::telemetry(v.node.id).fresh()
                                && enu_distance_meters(v.node.position_enu, *waypoint) < 15.0
                        })
                    {
                        continue;
                    }
                    if let Some(index) = free.pop() {
                        plan.push((index, *waypoint, last));
                    } else {
                        plan.clear();
                        break;
                    }
                }
                if plan.is_empty() {
                    continue;
                }
                for (index, waypoint, surveyor) in plan {
                    let vehicle = &mut vehicles[index];
                    vehicle.node.role = if surveyor {
                        DroneRole::LeadSurveyor
                    } else {
                        DroneRole::StationaryTower
                    };
                    if surveyor {
                        vehicle.node.current_poi = Some(poi.id);
                        manager
                            .mark_poi_in_progress(mission_id, poi.id, vehicle.node.id)
                            .unwrap();
                        metrics::log_event(metrics::MetricEvent::PoiAssigned {
                            mission_id,
                            poi_id: poi.id,
                            drone_id: vehicle.node.id,
                        })
                        .await;
                        println!(
                            "Auto-dispatched Drone {} to PoI #{}",
                            vehicle.node.id, poi.id
                        );
                    }
                    start_target(vehicle, waypoint, origin);
                }
            }
        }
        tick += 1;
        if tick % 20 == 0 {
            let all = manager.get_all_pois(mission_id).unwrap();
            println!(
                "Fleet {}: {} fresh, {} airborne, {}/{} POIs reported",
                vehicles.len(),
                vehicles
                    .iter()
                    .filter(|v| gps::telemetry(v.node.id).fresh())
                    .count(),
                vehicles
                    .iter()
                    .filter(|v| v.node.position_enu.up_m > 2.0)
                    .count(),
                all.iter()
                    .filter(|p| matches!(p.state, PoIState::Completed))
                    .count(),
                all.len()
            );
        }
        sleep(Duration::from_millis(500)).await;
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn corridor_fits_twenty_and_keeps_hops_below_range() {
        let target = PositionEnu {
            north_m: 450.0,
            east_m: 450.0,
            up_m: 15.0,
        };
        let points = chain(target);
        assert!(points.len() <= 10);
        let mut previous = PositionEnu {
            north_m: 0.0,
            east_m: 0.0,
            up_m: 0.0,
        };
        for next in points {
            assert!(enu_distance_meters(previous, next) <= 80.0);
            previous = next;
        }
        assert_eq!(previous, target);
    }
}
