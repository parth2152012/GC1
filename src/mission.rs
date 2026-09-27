use crate::bat_management::BatteryState;
use crate::deep_scan::DroneRole;
use crate::gps;
use crate::peer::PositionEnu;

use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum Priority {
    High,
    Medium,
    Low,
}

impl Priority {
    pub fn all_desc() -> [Priority; 3] {
        [Priority::High, Priority::Medium, Priority::Low]
    }
}

#[derive(Debug, Clone)]
pub enum PoIState {
    Pending,
    Assigned(u8),
    InProgress(u8),
    Completed,
    Failed,
}

#[derive(Debug, Clone)]
pub struct PointOfInterest {
    pub id: u32,
    pub north_m: f64,
    pub east_m: f64,
    pub alt_m: f32,
    pub priority: Priority,
    pub state: PoIState,
    pub attempts: u8,
    pub added_at: Instant,
}

#[derive(Debug, Clone)]
pub struct Mission {
    pub id: u32,
    pub pois: Vec<PointOfInterest>,
    pub created_at: Instant,
    pub active: bool,
}

#[derive(Debug)]
pub struct MissionManager {
    missions: HashMap<u32, Mission>,
    next_poi_id: u32,
    pub assignment_battery_threshold: f32,
    pub max_attempts_per_poi: u8,
}

impl MissionManager {
    pub fn new() -> Self {
        Self {
            missions: HashMap::new(),
            next_poi_id: 1,
            assignment_battery_threshold: 20.0,
            max_attempts_per_poi: 3,
        }
    }

    pub fn create_mission(&mut self) -> u32 {
        let id = self.missions.len() as u32 + 1;
        let mission = Mission {
            id,
            pois: Vec::new(),
            created_at: Instant::now(),
            active: true,
        };
        self.missions.insert(id, mission);
        id
    }

    pub fn add_poi(
        &mut self,
        mission_id: u32,
        north_m: f64,
        east_m: f64,
        alt_m: f32,
        priority: Priority,
    ) -> Result<u32, String> {
        let mission = self
            .missions
            .get_mut(&mission_id)
            .ok_or("mission not found")?;
        let poi = PointOfInterest {
            id: self.next_poi_id,
            north_m,
            east_m,
            alt_m,
            priority,
            state: PoIState::Pending,
            attempts: 0,
            added_at: Instant::now(),
        };
        mission.pois.push(poi);
        self.next_poi_id += 1;
        Ok(self.next_poi_id - 1)
    }

    pub fn get_all_pois(&self, mission_id: u32) -> Option<Vec<PointOfInterest>> {
        self.missions.get(&mission_id).map(|m| m.pois.clone())
    }

    /// Select next PoI for a UAV, honoring priority and availability. Recalculates each call.
    pub fn select_next_poi_for_uav(
        &mut self,
        mission_id: u32,
        uav: &crate::deep_scan::SwarmNode,
        origin_lat: f64,
        origin_lon: f64,
    ) -> Option<PointOfInterest> {
        let mission = self.missions.get_mut(&mission_id)?;

        // UAV must be available
        if !Self::uav_available(uav, self.assignment_battery_threshold) {
            return None;
        }

        // Build candidate queues for each priority level
        for &prio in Priority::all_desc().iter() {
            // find first pending poi with this priority
            if let Some(idx) = mission
                .pois
                .iter()
                .position(|p| matches!(p.state, PoIState::Pending) && p.priority == prio)
            {
                // found candidate
                let mut poi = mission.pois[idx].clone();

                // assign
                poi.state = PoIState::Assigned(uav.id);
                mission.pois[idx].state = poi.state.clone();
                mission.pois[idx].attempts += 1;

                return Some(poi);
            }
        }

        None
    }

    pub fn mark_poi_in_progress(
        &mut self,
        mission_id: u32,
        poi_id: u32,
        uav_id: u8,
    ) -> Result<(), String> {
        let mission = self
            .missions
            .get_mut(&mission_id)
            .ok_or("mission not found")?;
        let poi = mission
            .pois
            .iter_mut()
            .find(|p| p.id == poi_id)
            .ok_or("poi not found")?;
        poi.state = PoIState::InProgress(uav_id);
        Ok(())
    }

    pub fn mark_poi_completed(&mut self, mission_id: u32, poi_id: u32) -> Result<(), String> {
        let mission = self
            .missions
            .get_mut(&mission_id)
            .ok_or("mission not found")?;
        let poi = mission
            .pois
            .iter_mut()
            .find(|p| p.id == poi_id)
            .ok_or("poi not found")?;
        poi.state = PoIState::Completed;
        Ok(())
    }

    pub fn mark_poi_failed(&mut self, mission_id: u32, poi_id: u32) -> Result<(), String> {
        let mission = self
            .missions
            .get_mut(&mission_id)
            .ok_or("mission not found")?;
        let poi = mission
            .pois
            .iter_mut()
            .find(|p| p.id == poi_id)
            .ok_or("poi not found")?;
        if poi.attempts >= self.max_attempts_per_poi {
            poi.state = PoIState::Failed;
        } else {
            poi.state = PoIState::Pending;
        }
        Ok(())
    }

    /// FIX: previously nothing ever released a PoI that was `Assigned`/`InProgress` on
    /// a UAV that has since become unavailable (RTL, battery-critical, comms lost) —
    /// `select_next_poi_for_uav` only ever considers `Pending` PoIs, so such a PoI
    /// would stay permanently stuck to a UAV that can no longer complete it, and no
    /// other UAV could ever pick it up. The caller (the orchestrator loop in main.rs)
    /// is responsible for detecting that a UAV holding a `current_poi` has become
    /// unavailable and calling this to put the PoI back into circulation.
    ///
    /// This does not count against `max_attempts_per_poi` / does not risk marking the
    /// PoI `Failed`, since the interruption was caused by the UAV, not by anything
    /// wrong with the PoI itself.
    pub fn release_stalled_poi(&mut self, mission_id: u32, poi_id: u32) -> Result<(), String> {
        let mission = self
            .missions
            .get_mut(&mission_id)
            .ok_or("mission not found")?;
        let poi = mission
            .pois
            .iter_mut()
            .find(|p| p.id == poi_id)
            .ok_or("poi not found")?;
        if matches!(poi.state, PoIState::Assigned(_) | PoIState::InProgress(_)) {
            poi.state = PoIState::Pending;
        }
        Ok(())
    }

    pub fn uav_available(
        uav: &crate::deep_scan::SwarmNode,
        assignment_battery_threshold: f32,
    ) -> bool {
        if uav.role == DroneRole::ReturnToBase || uav.role == DroneRole::ReservePool {
            return false;
        }
        if uav.battery.must_return_to_base() {
            return false;
        }
        if uav.battery.percentage < assignment_battery_threshold {
            return false;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bat_management::BatteryState;
    use crate::deep_scan::{DroneRole, SwarmNode};

    fn make_node(id: u8, role: DroneRole, percent: f32) -> SwarmNode {
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
    fn priority_ordering() {
        let mut mgr = MissionManager::new();
        let mid = mgr.create_mission();
        let _ = mgr.add_poi(mid, 10.0, 0.0, 10.0, Priority::Low);
        let _ = mgr.add_poi(mid, 20.0, 0.0, 10.0, Priority::High);
        let _ = mgr.add_poi(mid, 30.0, 0.0, 10.0, Priority::Medium);

        let uav = make_node(1, DroneRole::LeadSurveyor, 100.0);
        let poi = mgr
            .select_next_poi_for_uav(mid, &uav, 0.0, 0.0)
            .expect("should assign high priority first");
        assert_eq!(poi.priority, Priority::High);
    }

    #[test]
    fn equal_priority_fifo() {
        let mut mgr = MissionManager::new();
        let mid = mgr.create_mission();
        let id1 = mgr.add_poi(mid, 10.0, 0.0, 10.0, Priority::Medium).unwrap();
        let id2 = mgr.add_poi(mid, 20.0, 0.0, 10.0, Priority::Medium).unwrap();

        let uav = make_node(1, DroneRole::LeadSurveyor, 100.0);
        let poi1 = mgr.select_next_poi_for_uav(mid, &uav, 0.0, 0.0).unwrap();
        assert_eq!(poi1.id, id1);
        // mark in progress and then complete
        mgr.mark_poi_in_progress(mid, poi1.id, uav.id).unwrap();
        mgr.mark_poi_completed(mid, poi1.id).unwrap();

        let poi2 = mgr.select_next_poi_for_uav(mid, &uav, 0.0, 0.0).unwrap();
        assert_eq!(poi2.id, id2);
    }

    #[test]
    fn unavailable_uav_not_assigned() {
        let mut mgr = MissionManager::new();
        let mid = mgr.create_mission();
        let _ = mgr.add_poi(mid, 10.0, 0.0, 10.0, Priority::High);

        let uav = make_node(1, DroneRole::ReturnToBase, 100.0);
        assert!(mgr.select_next_poi_for_uav(mid, &uav, 0.0, 0.0).is_none());
    }

    #[test]
    fn low_battery_not_assigned() {
        let mut mgr = MissionManager::new();
        let mid = mgr.create_mission();
        let _ = mgr.add_poi(mid, 10.0, 0.0, 10.0, Priority::High);

        let uav = make_node(1, DroneRole::LeadSurveyor, 10.0);
        assert!(mgr.select_next_poi_for_uav(mid, &uav, 0.0, 0.0).is_none());
    }

    #[test]
    fn reassignment_after_unavailable() {
        let mut mgr = MissionManager::new();
        let mid = mgr.create_mission();
        let id = mgr.add_poi(mid, 10.0, 0.0, 10.0, Priority::High).unwrap();

        let mut uav1 = make_node(1, DroneRole::LeadSurveyor, 100.0);
        let uav2 = make_node(2, DroneRole::LeadSurveyor, 100.0);

        let poi = mgr.select_next_poi_for_uav(mid, &uav1, 0.0, 0.0).unwrap();
        assert_eq!(poi.id, id);

        // Simulate uav1 becoming unavailable (return to base) mid-flight. This mirrors
        // what the orchestrator loop in main.rs does: it detects the role transition on
        // a node still holding a `current_poi` and releases that PoI back to the pool.
        uav1.role = DroneRole::ReturnToBase;
        mgr.release_stalled_poi(mid, id).unwrap();

        // Now reassignment to a different, available UAV should succeed.
        let poi2 = mgr.select_next_poi_for_uav(mid, &uav2, 0.0, 0.0).unwrap();
        assert_eq!(poi2.id, id);
    }

    #[test]
    fn release_stalled_poi_is_a_noop_for_pending_or_completed() {
        let mut mgr = MissionManager::new();
        let mid = mgr.create_mission();
        let id = mgr.add_poi(mid, 10.0, 0.0, 10.0, Priority::High).unwrap();

        // Pending PoI: releasing it should not error and should leave it assignable.
        mgr.release_stalled_poi(mid, id).unwrap();
        let uav = make_node(1, DroneRole::LeadSurveyor, 100.0);
        let poi = mgr.select_next_poi_for_uav(mid, &uav, 0.0, 0.0).unwrap();
        assert_eq!(poi.id, id);

        mgr.mark_poi_in_progress(mid, id, uav.id).unwrap();
        mgr.mark_poi_completed(mid, id).unwrap();

        // Completed PoI: releasing it must not resurrect it as Pending.
        mgr.release_stalled_poi(mid, id).unwrap();
        let pois = mgr.get_all_pois(mid).unwrap();
        assert!(matches!(
            pois.iter().find(|p| p.id == id).unwrap().state,
            PoIState::Completed
        ));
    }
}
