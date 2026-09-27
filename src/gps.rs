use mavlink::common::{
    MavCmd, MavMessage, MavMode, PositionTargetTypemask, COMMAND_LONG_DATA, SET_MODE_DATA,
    SET_POSITION_TARGET_GLOBAL_INT_DATA,
};
use mavlink::error::MessageReadError;
use std::sync::atomic::{AtomicI32, AtomicU8, Ordering};
use std::time::Duration;
use tokio::time::sleep;

const DEFAULT_CONTROL_PORT_BASE: u16 = 14600;
const COMMAND_SETTLE_DELAY: Duration = Duration::from_millis(500);
const ARM_SETTLE_DELAY: Duration = Duration::from_secs(1);

pub static LATITUDE_1: AtomicI32 = AtomicI32::new(0);
pub static LONGITUDE_1: AtomicI32 = AtomicI32::new(0);
pub static ALTITUDE_1_MM: AtomicI32 = AtomicI32::new(0);
pub static BATTERY_1: AtomicU8 = AtomicU8::new(100);

pub static LATITUDE_2: AtomicI32 = AtomicI32::new(0);
pub static LONGITUDE_2: AtomicI32 = AtomicI32::new(0);
pub static ALTITUDE_2_MM: AtomicI32 = AtomicI32::new(0);
pub static BATTERY_2: AtomicU8 = AtomicU8::new(100);

pub async fn start_telemetry_loop() {
    println!("📡 Dual MAVLink telemetry listeners active (14550 & 14560)...");

    tokio::task::spawn_blocking(move || {
        listen_mavlink(
            "udpin:127.0.0.1:14550",
            &LATITUDE_1,
            &LONGITUDE_1,
            &ALTITUDE_1_MM,
            &BATTERY_1,
        );
    });

    tokio::task::spawn_blocking(move || {
        listen_mavlink(
            "udpin:127.0.0.1:14560",
            &LATITUDE_2,
            &LONGITUDE_2,
            &ALTITUDE_2_MM,
            &BATTERY_2,
        );
    });
}

fn listen_mavlink(
    endpoint: &str,
    lat_store: &'static AtomicI32,
    lon_store: &'static AtomicI32,
    alt_store: &'static AtomicI32,
    bat_store: &'static AtomicU8,
) {
    let mut link = match mavlink::connect::<MavMessage>(endpoint) {
        Ok(conn) => conn,
        Err(e) => {
            eprintln!(
                "❌ Failed to connect to MAVLink endpoint {}: {:?}",
                endpoint, e
            );
            return;
        }
    };

    let _ = link.set_protocol_version(mavlink::MavlinkVersion::V2);

    loop {
        match link.recv() {
            Ok((_header, msg)) => match msg {
                MavMessage::GLOBAL_POSITION_INT(data) => {
                    lat_store.store(data.lat, Ordering::Release);
                    lon_store.store(data.lon, Ordering::Release);
                    alt_store.store(data.relative_alt, Ordering::Release);
                }
                MavMessage::SYS_STATUS(data) => {
                    if data.battery_remaining >= 0 {
                        bat_store.store(data.battery_remaining as u8, Ordering::Release);
                    }
                }
                _ => {}
            },
            Err(MessageReadError::Io(ref e)) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(5));
            }
            Err(MessageReadError::Io(ref e)) if e.kind() == std::io::ErrorKind::TimedOut => {
                std::thread::sleep(Duration::from_millis(5));
            }
            Err(_) => {
                std::thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

/// Sends a guided target through MAVProxy's dedicated bidirectional TCP output.
///
/// The UDP telemetry ports are one-way `--out` destinations used by the listeners,
/// so writing a command back to them only sends it to another local UDP socket. SITL
/// also has a single-client TCP endpoint already occupied by MAVProxy. The launcher
/// therefore exposes separate MAVProxy `tcpin` outputs starting at port 14600.
pub fn send_guided_target(target_lat: f64, target_lon: f64, alt_m: f32) -> Result<(), String> {
    let link = control_link()?;

    let header = mavlink::MavHeader {
        system_id: 255,
        component_id: 190,
        sequence: 0,
    };

    let set_pos_msg =
        MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
            time_boot_ms: 0,
            target_system: 1,
            target_component: 1,
            coordinate_frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask: PositionTargetTypemask::from_bits_truncate(0b0000111111111000),
            lat_int: (target_lat * 1e7) as i32,
            lon_int: (target_lon * 1e7) as i32,
            alt: alt_m,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.0,
            yaw_rate: 0.0,
        });

    link.send(&header, &set_pos_msg)
        .map(|_| ())
        .map_err(|error| format!("could not send guided target: {error}"))
}

/// Selects ArduCopter Guided mode for Drone 1.
pub fn set_guided_mode() -> Result<(), String> {
    let link = control_link()?;
    let header = control_header();
    let message = MavMessage::SET_MODE(SET_MODE_DATA {
        target_system: 1,
        base_mode: MavMode::MAV_MODE_GUIDED_DISARMED,
        // ArduCopter's custom mode number for Guided.
        custom_mode: 4,
    });

    link.send(&header, &message)
        .map(|_| ())
        .map_err(|error| format!("could not select Guided mode: {error}"))
}

/// Arms Drone 1 using the MAVLink arm/disarm command.
pub fn arm_throttle() -> Result<(), String> {
    send_command(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        .map_err(|error| format!("could not arm throttle: {error}"))
}

/// Commands Drone 1 to take off to a relative altitude in metres.
pub fn takeoff(altitude_m: f32) -> Result<(), String> {
    send_command(MavCmd::MAV_CMD_NAV_TAKEOFF, 0.0, altitude_m)
        .map_err(|error| format!("could not command takeoff: {error}"))
}

fn control_link() -> Result<Box<dyn mavlink::MavConnection<MavMessage> + Send>, String> {
    control_link_for(1)
}

fn control_header() -> mavlink::MavHeader {
    mavlink::MavHeader {
        system_id: 255,
        component_id: 190,
        sequence: 0,
    }
}

fn send_command(command: MavCmd, param1: f32, param7: f32) -> Result<(), String> {
    let link = control_link()?;
    let header = control_header();
    let message = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command,
        confirmation: 0,
        param1,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7,
    });

    link.send(&header, &message)
        .map(|_| ())
        .map_err(|error| error.to_string())
}

// ---------------------------------------------------------------------------------
// FIX: the functions above only ever command Drone 1 (hardcoded endpoint, sysid 1).
// That meant that once WaveManager promoted the reserve (Drone 2) to LeadSurveyor,
// nothing in the codebase could actually fly it — the "relay handoff" was only a
// role label, never a real command. The functions below route each system ID through
// its corresponding MAVProxy TCP output so the mission-dispatch loop in main.rs can
// command whichever node currently holds the LeadSurveyor role.
// ---------------------------------------------------------------------------------

fn control_endpoint_for(drone_id: u8) -> String {
    let base = std::env::var("GC1_CONTROL_PORT_BASE")
        .ok()
        .and_then(|value| value.parse::<u16>().ok())
        .unwrap_or(DEFAULT_CONTROL_PORT_BASE);
    let port = base + drone_id.saturating_sub(1) as u16;
    format!("tcpout:127.0.0.1:{port}")
}

fn control_link_for(
    drone_id: u8,
) -> Result<Box<dyn mavlink::MavConnection<MavMessage> + Send>, String> {
    let endpoint = control_endpoint_for(drone_id);
    let mut link = mavlink::connect::<MavMessage>(&endpoint).map_err(|error| {
        format!("could not connect to Drone {drone_id} control endpoint ({endpoint}): {error}")
    })?;
    link.set_protocol_version(mavlink::MavlinkVersion::V2);
    Ok(link)
}

/// Same as `send_guided_target`, but for an arbitrary drone (by 1-based ID) instead
/// of always Drone 1.
pub fn send_guided_target_for(
    drone_id: u8,
    target_lat: f64,
    target_lon: f64,
    alt_m: f32,
) -> Result<(), String> {
    let link = control_link_for(drone_id)?;
    let header = control_header();
    let set_pos_msg =
        MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
            time_boot_ms: 0,
            target_system: drone_id,
            target_component: 1,
            coordinate_frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask: PositionTargetTypemask::from_bits_truncate(0b0000111111111000),
            lat_int: (target_lat * 1e7) as i32,
            lon_int: (target_lon * 1e7) as i32,
            alt: alt_m,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.0,
            yaw_rate: 0.0,
        });

    link.send(&header, &set_pos_msg)
        .map(|_| ())
        .map_err(|error| format!("could not send guided target to drone {drone_id}: {error}"))
}

/// Same as `set_guided_mode`, but for an arbitrary drone.
pub fn set_guided_mode_for(drone_id: u8) -> Result<(), String> {
    let link = control_link_for(drone_id)?;
    let header = control_header();
    let message = MavMessage::SET_MODE(SET_MODE_DATA {
        target_system: drone_id,
        base_mode: MavMode::MAV_MODE_GUIDED_DISARMED,
        custom_mode: 4,
    });
    link.send(&header, &message)
        .map(|_| ())
        .map_err(|error| format!("could not select Guided mode for drone {drone_id}: {error}"))
}

/// Same as `arm_throttle`, but for an arbitrary drone.
pub fn arm_throttle_for(drone_id: u8) -> Result<(), String> {
    send_command_for(drone_id, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        .map_err(|error| format!("could not arm drone {drone_id}: {error}"))
}

/// Same as `takeoff`, but for an arbitrary drone.
/// Commands an arbitrary drone to return to its launch point.
pub fn return_to_launch_for(drone_id: u8) -> Result<(), String> {
    send_command_for(drone_id, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0)
        .map_err(|error| format!("could not command RTL for drone {drone_id}: {error}"))
}

pub fn takeoff_for(drone_id: u8, altitude_m: f32) -> Result<(), String> {
    send_command_for(drone_id, MavCmd::MAV_CMD_NAV_TAKEOFF, 0.0, altitude_m)
        .map_err(|error| format!("could not command takeoff for drone {drone_id}: {error}"))
}

fn relative_altitude_mm_for(drone_id: u8) -> i32 {
    match drone_id {
        1 => ALTITUDE_1_MM.load(Ordering::Acquire),
        2 => ALTITUDE_2_MM.load(Ordering::Acquire),
        _ => 0,
    }
}

/// Select Guided mode, arm, and initiate takeoff in the order ArduCopter requires.
pub async fn prepare_and_takeoff_for(drone_id: u8, altitude_m: f32) -> Result<(), String> {
    set_guided_mode_for(drone_id)?;
    sleep(COMMAND_SETTLE_DELAY).await;
    arm_throttle_for(drone_id)?;
    sleep(ARM_SETTLE_DELAY).await;
    takeoff_for(drone_id, altitude_m)
}

/// Ensure a grounded aircraft takes off before dispatching its PoI target.
pub async fn prepare_and_send_guided_target_for(
    drone_id: u8,
    target_lat: f64,
    target_lon: f64,
    alt_m: f32,
) -> Result<(), String> {
    set_guided_mode_for(drone_id)?;
    sleep(COMMAND_SETTLE_DELAY).await;
    if relative_altitude_mm_for(drone_id) < 2_000 {
        arm_throttle_for(drone_id)?;
        sleep(ARM_SETTLE_DELAY).await;
        takeoff_for(drone_id, alt_m)?;
        sleep(Duration::from_secs(2)).await;
    }
    send_guided_target_for(drone_id, target_lat, target_lon, alt_m)
}

fn send_command_for(drone_id: u8, command: MavCmd, param1: f32, param7: f32) -> Result<(), String> {
    let link = control_link_for(drone_id)?;
    let header = control_header();
    let message = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: drone_id,
        target_component: 1,
        command,
        confirmation: 0,
        param1,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7,
    });
    link.send(&header, &message)
        .map(|_| ())
        .map_err(|error| error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_endpoints_are_distinct_mavproxy_outputs() {
        std::env::remove_var("GC1_CONTROL_PORT_BASE");
        assert_eq!(control_endpoint_for(1), "tcpout:127.0.0.1:14600");
        assert_eq!(control_endpoint_for(2), "tcpout:127.0.0.1:14601");
    }
}
