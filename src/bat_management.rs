use std::sync::atomic::Ordering;
use crate::gps::BATTERY_1;

#[derive(Debug)]
pub struct BatteryState {
    pub percentage: f32,
    pub discharge_rate_per_min: f32,
}

impl BatteryState {
    pub fn new() -> Self {
        Self {
            percentage: BATTERY_1.load(Ordering::Relaxed) as f32,
            discharge_rate_per_min: 5.0,
        }
    }

    pub fn refresh(&mut self) {
        self.percentage = BATTERY_1.load(Ordering::Relaxed) as f32;
    }

    pub fn trigger_wave_takeoff(&self) -> bool {
        self.percentage <= 45.0
    }

    pub fn must_return_to_base(&self) -> bool {
        self.percentage <= 15.0
    }

    pub fn estimate_remaining_seconds(&self) -> f32 {
        if self.discharge_rate_per_min <= 0.0 {
            return 0.0;
        }
        (self.percentage / self.discharge_rate_per_min) * 60.0
    }
}