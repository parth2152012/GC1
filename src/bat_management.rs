use std::sync::atomic::Ordering;

pub const RESERVE_DISPATCH_PERCENT: f32 = 45.0;
pub const RTL_PERCENT: f32 = 15.0;

#[derive(Debug, Clone)]
pub struct BatteryState {
    pub percentage: f32,
    pub discharge_rate_per_min: f32,
}

impl BatteryState {
    pub fn new() -> Self {
        Self {
            percentage: 100.0,
            discharge_rate_per_min: 5.0,
        }
    }

    pub fn from_telemetry(battery: &std::sync::atomic::AtomicU8) -> Self {
        Self {
            percentage: battery.load(Ordering::Acquire) as f32,
            discharge_rate_per_min: 5.0,
        }
    }

    pub fn trigger_wave_takeoff(&self) -> bool {
        self.percentage <= RESERVE_DISPATCH_PERCENT
    }
    pub fn must_return_to_base(&self) -> bool {
        self.percentage <= RTL_PERCENT
    }

    pub fn estimate_remaining_seconds(&self) -> f32 {
        if !self.percentage.is_finite()
            || !self.discharge_rate_per_min.is_finite()
            || self.discharge_rate_per_min <= 0.0
        {
            return 0.0;
        }
        (self.percentage.max(0.0) / self.discharge_rate_per_min) * 60.0
    }
}
