use crate::bat_management::BatteryState;
use crate::peer::PositionEnu;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DroneRole {
    LeadSurveyor,
    StationaryTower,
    ReservePool,
    ReturnToBase,
}

#[derive(Debug)]
pub struct SwarmNode {
    #[allow(dead_code)]
    pub id: u8,
    pub role: DroneRole,
    pub position_enu: PositionEnu,
    pub battery: BatteryState,
}

pub struct WaveManager {
    pub max_transit_speed_mps: f64,
    pub max_comms_range_meters: f64,
}
impl WaveManager {
    pub fn new() -> Self {
        Self {
            max_transit_speed_mps: 5.0,
            max_comms_range_meters: 100.0,
        }
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
        let dispatch_needed = active.battery.trigger_wave_takeoff()
            || active.battery.estimate_remaining_seconds() <= transit_lag;
        if dispatch_needed && reserve.role == DroneRole::ReservePool {
            reserve.role = DroneRole::LeadSurveyor;
        }

        if active.battery.must_return_to_base() {
            active.role = DroneRole::ReturnToBase;
            return;
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
}
