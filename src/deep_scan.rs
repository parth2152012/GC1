use crate::bat_management::BatteryState;
use crate::peer::PositionEnu;
use std::time::{Duration, Instant};

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DroneRole {
    LeadSurveyor,
    StationaryTower,
    ReservePool,
    ReturnToBase,
}

#[derive(Debug)]
pub struct SwarmNode {
    pub id: u8,
    pub role: DroneRole,
    pub position_enu: PositionEnu,
    pub battery: BatteryState,
    /// The PoI this node is currently flying to, if any. `None` means the node is
    /// idle and eligible for a new assignment from the `MissionManager`.
    ///
    /// FIX: previously there was no way to track which PoI a UAV was working on, so
    /// there was no automatic assignment/completion loop possible at all.
    pub current_poi: Option<u32>,
    /// Set when telemetry first shows the UAV airborne or when a guided mission
    /// dispatch succeeds. Used to enforce the challenge flight-duration ceiling.
    pub airborne_since: Option<Instant>,
}

pub struct WaveManager {
    pub max_transit_speed_mps: f64,
    pub max_comms_range_meters: f64,
    pub max_flight_duration: Duration,
    pub flight_preemption_margin: Duration,
}
impl WaveManager {
    pub fn new() -> Self {
        Self {
            max_transit_speed_mps: 5.0,
            max_comms_range_meters: 100.0,
            max_flight_duration: Duration::from_secs(20 * 60),
            flight_preemption_margin: Duration::from_secs(30),
        }
    }

    /// Test/configuration constructor. Production defaults remain the organizer's
    /// 20-minute maximum with a 30-second replacement margin.
    pub fn with_max_flight_duration(max_flight_duration: Duration) -> Self {
        Self {
            max_flight_duration,
            ..Self::new()
        }
    }

    pub fn flight_time_exceeded(&self, node: &SwarmNode) -> bool {
        node.airborne_since
            .is_some_and(|started| started.elapsed() >= self.max_flight_duration)
    }

    /// Applies irreversible battery transitions. RTL takes priority over relay mode so a
    /// critical active vehicle can never be held in a tower state by a handoff race.
    pub fn evaluate_handoff(
        &self,
        active: &mut SwarmNode,
        reserve: &mut SwarmNode,
        distance_to_field: f64,
    ) {
        let transit_lag = (distance_to_field.max(0.0) / self.max_transit_speed_mps) as f32;
        let flight_near_limit = active.airborne_since.is_some_and(|started| {
            started.elapsed() + self.flight_preemption_margin >= self.max_flight_duration
        });
        let dispatch_needed = active.battery.trigger_wave_takeoff()
            || active.battery.estimate_remaining_seconds() <= transit_lag
            || flight_near_limit;
        if dispatch_needed && reserve.role == DroneRole::ReservePool {
            reserve.role = DroneRole::LeadSurveyor;
        }

        if active.battery.must_return_to_base() || self.flight_time_exceeded(active) {
            active.role = DroneRole::ReturnToBase;
            return;
        }
        if self.flight_time_exceeded(reserve) {
            reserve.role = DroneRole::ReturnToBase;
        }

        // Keep scanning until a dispatched reserve is close enough to preserve the mesh.
        if reserve.role == DroneRole::LeadSurveyor
            && distance_to_field < self.max_comms_range_meters
            && active.role == DroneRole::LeadSurveyor
        {
            active.role = DroneRole::StationaryTower;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn node(id: u8, role: DroneRole, percent: f32) -> SwarmNode {
        SwarmNode {
            id,
            role,
            position_enu: PositionEnu {
                east_m: 0.0,
                north_m: 0.0,
                up_m: 0.0,
            },
            battery: BatteryState {
                percentage: percent,
                discharge_rate_per_min: 5.0,
            },
            current_poi: None,
            airborne_since: None,
        }
    }
    #[test]
    fn critical_battery_cannot_stall_in_tower_mode() {
        let manager = WaveManager::new();
        let mut active = node(1, DroneRole::StationaryTower, 15.0);
        let mut reserve = node(2, DroneRole::ReservePool, 100.0);
        manager.evaluate_handoff(&mut active, &mut reserve, 50.0);
        assert_eq!(active.role, DroneRole::ReturnToBase);
        assert_eq!(reserve.role, DroneRole::LeadSurveyor);
    }
    #[test]
    fn configurable_flight_limit_triggers_rtl() {
        let manager = WaveManager::with_max_flight_duration(Duration::from_secs(60));
        let mut active = node(1, DroneRole::LeadSurveyor, 100.0);
        active.airborne_since = Some(Instant::now() - Duration::from_secs(61));
        let mut reserve = node(2, DroneRole::ReservePool, 100.0);
        manager.evaluate_handoff(&mut active, &mut reserve, 0.0);
        assert_eq!(active.role, DroneRole::ReturnToBase);
        assert_eq!(reserve.role, DroneRole::LeadSurveyor);
    }
}
