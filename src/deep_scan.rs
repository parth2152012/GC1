use crate::bat_management::BatteryState;

#[derive(Debug, PartialEq, Clone, Copy)]
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
    pub position_enu: (f64, f64),
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

    pub fn evaluate_handoff(&self, active_drone: &mut SwarmNode, reserve_drone: &mut SwarmNode, distance_to_field: f64) {
        let transit_lag_seconds = (distance_to_field / self.max_transit_speed_mps) as f32;
        let active_remaining_seconds = active_drone.battery.estimate_remaining_seconds();

        if active_drone.battery.trigger_wave_takeoff() || active_remaining_seconds <= transit_lag_seconds {
            if reserve_drone.role == DroneRole::ReservePool {
                reserve_drone.role = DroneRole::LeadSurveyor;
            }
        }

        if reserve_drone.role == DroneRole::LeadSurveyor && distance_to_field < self.max_comms_range_meters {
            if active_drone.role == DroneRole::LeadSurveyor {
                active_drone.role = DroneRole::StationaryTower;
            }
        }

        if active_drone.battery.must_return_to_base() {
            active_drone.role = DroneRole::ReturnToBase;
        }
    }
}