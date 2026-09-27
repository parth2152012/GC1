use mavlink::common::{
    MavCmd, MavMessage, MavResult, PositionTargetTypemask, COMMAND_LONG_DATA,
    SET_POSITION_TARGET_GLOBAL_INT_DATA,
};
use mavlink::error::MessageReadError;
use std::sync::atomic::{AtomicI32, AtomicU8, Ordering};
use std::time::{Duration, Instant};
use tokio::time::sleep;

const DEFAULT_CONTROL_PORT_BASE: u16 = 14600;
pub const MAX_DRONES: usize = 20;
static CONTROL_LOCKS: [std::sync::Mutex<()>; MAX_DRONES] =
    [const { std::sync::Mutex::new(()) }; MAX_DRONES];
static FLEET_SIZE: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
const COMMAND_TIMEOUT: Duration = Duration::from_secs(5);

pub struct Telemetry {
    pub lat: AtomicI32,
    pub lon: AtomicI32,
    pub altitude_mm: AtomicI32,
    pub battery: AtomicU8,
    pub battery_override: AtomicI32,
    updated_ms: std::sync::atomic::AtomicU64,
}
impl Telemetry {
    const fn new() -> Self {
        Self {
            lat: AtomicI32::new(0),
            lon: AtomicI32::new(0),
            altitude_mm: AtomicI32::new(0),
            battery: AtomicU8::new(100),
            battery_override: AtomicI32::new(-1),
            updated_ms: std::sync::atomic::AtomicU64::new(0),
        }
    }
    pub fn fresh(&self) -> bool {
        let stamp = self.updated_ms.load(Ordering::Acquire);
        stamp != 0 && now_ms().saturating_sub(stamp) < 3000
    }
    pub fn battery_percent(&self) -> f32 {
        let injected = self.battery_override.load(Ordering::Acquire);
        if injected >= 0 {
            injected as f32
        } else {
            self.battery.load(Ordering::Acquire) as f32
        }
    }
}
fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}
static TELEMETRY: [Telemetry; MAX_DRONES] = [const { Telemetry::new() }; MAX_DRONES];
pub fn configure(count: usize) -> Result<(), String> {
    if !(1..=MAX_DRONES).contains(&count) {
        return Err("drone count must be 1..=20".into());
    }
    FLEET_SIZE
        .set(count)
        .map_err(|_| "fleet already configured".into())
}
pub fn fleet_size() -> usize {
    *FLEET_SIZE.get().unwrap_or(&2)
}
pub fn telemetry(id: u8) -> &'static Telemetry {
    &TELEMETRY[id as usize - 1]
}
fn control_lock(id: u8) -> Result<std::sync::MutexGuard<'static, ()>, String> {
    if id == 0 || id as usize > fleet_size() {
        return Err(format!("Unsupported drone ID {id}"));
    }
    CONTROL_LOCKS[id as usize - 1]
        .lock()
        .map_err(|e| e.to_string())
}
pub async fn start_telemetry_loop() {
    println!("Telemetry listeners starting for {} drones", fleet_size());
    for id in 1..=fleet_size() as u8 {
        tokio::task::spawn_blocking(move || listen_mavlink(id));
    }
}
fn listen_mavlink(id: u8) {
    let endpoint = format!("udpin:127.0.0.1:{}", 14550 + 10 * (id as u16 - 1));
    let mut link = match mavlink::connect::<MavMessage>(&endpoint) {
        Ok(link) => link,
        Err(e) => {
            eprintln!("Drone {id} telemetry bind failed: {e}");
            return;
        }
    };
    link.set_protocol_version(mavlink::MavlinkVersion::V2);
    let state = telemetry(id);
    loop {
        match link.recv() {
            Ok((header, message)) if header.system_id == id => match message {
                MavMessage::GLOBAL_POSITION_INT(data) => {
                    state.lat.store(data.lat, Ordering::Release);
                    state.lon.store(data.lon, Ordering::Release);
                    state
                        .altitude_mm
                        .store(data.relative_alt, Ordering::Release);
                    state.updated_ms.store(now_ms(), Ordering::Release);
                }
                MavMessage::SYS_STATUS(data) if data.battery_remaining >= 0 => {
                    state
                        .battery
                        .store(data.battery_remaining as u8, Ordering::Release);
                }
                _ => {}
            },
            Err(_) => std::thread::sleep(Duration::from_millis(10)),
            _ => {}
        }
    }
}

/// Select Guided mode for Drone 1 and wait for the autopilot acknowledgement.
pub fn set_guided_mode() -> Result<(), String> {
    set_guided_mode_for(1)
}

pub fn arm_throttle() -> Result<(), String> {
    arm_throttle_for(1)
}

fn control_header() -> mavlink::MavHeader {
    mavlink::MavHeader {
        system_id: 255,
        component_id: 190,
        sequence: 0,
    }
}

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

/// Send a position target through the drone's dedicated MAVProxy TCP output.
pub fn send_guided_target_for(
    drone_id: u8,
    target_lat: f64,
    target_lon: f64,
    alt_m: f32,
) -> Result<(), String> {
    let _guard = control_lock(drone_id)?;
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
    // ArduCopter requires CUSTOM_MODE_ENABLED (1), not GUIDED_DISARMED (88).
    send_command_data(guided_mode_command(drone_id))
}

/// Same as `arm_throttle`, but for an arbitrary drone.
pub fn arm_throttle_for(drone_id: u8) -> Result<(), String> {
    send_command_for(drone_id, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        .map_err(|error| format!("could not arm drone {drone_id}: {error}"))
}

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
    telemetry(drone_id).altitude_mm.load(Ordering::Acquire)
}

/// Each setup command must be accepted before the next is sent.
pub async fn prepare_and_takeoff_for(drone_id: u8, altitude_m: f32) -> Result<(), String> {
    if !altitude_m.is_finite() || altitude_m <= 0.0 || altitude_m > 100.0 {
        return Err("takeoff altitude must be greater than 0 and at most 100m".into());
    }
    tokio::task::spawn_blocking(move || {
        set_guided_mode_for(drone_id)?;
        arm_throttle_for(drone_id)?;
        takeoff_for(drone_id, altitude_m)
    })
    .await
    .map_err(|error| error.to_string())?
}

/// Wait for measured climb before sending a position target; a fixed delay can
/// cancel takeoff while the motors are still spooling up.
pub async fn prepare_and_send_guided_target_for(
    drone_id: u8,
    target_lat: f64,
    target_lon: f64,
    alt_m: f32,
) -> Result<(), String> {
    if !target_lat.is_finite()
        || !target_lon.is_finite()
        || !alt_m.is_finite()
        || alt_m <= 0.0
        || alt_m > 100.0
    {
        return Err("target coordinates must be finite and altitude within (0, 100]m".into());
    }
    if relative_altitude_mm_for(drone_id) < 2_000 {
        prepare_and_takeoff_for(drone_id, alt_m).await?;
        let deadline = Instant::now() + Duration::from_secs(45);
        let required_mm = (alt_m * 1000.0 * 0.9) as i32;
        while relative_altitude_mm_for(drone_id) < required_mm {
            if Instant::now() >= deadline {
                return Err(format!(
                    "Drone {drone_id} did not reach takeoff altitude; PoI remains pending"
                ));
            }
            sleep(Duration::from_millis(200)).await;
        }
    } else {
        tokio::task::spawn_blocking(move || set_guided_mode_for(drone_id))
            .await
            .map_err(|error| error.to_string())??;
    }
    tokio::task::spawn_blocking(move || {
        // Groundspeed cap for Guided position navigation.
        send_command_data(COMMAND_LONG_DATA {
            param1: 1.0,
            param2: 4.5,
            param3: -1.0,
            ..command_data(drone_id, MavCmd::MAV_CMD_DO_CHANGE_SPEED, 0.0, 0.0)
        })?;
        send_guided_target_for(drone_id, target_lat, target_lon, alt_m)
    })
    .await
    .map_err(|e| e.to_string())?
}

fn guided_mode_command(drone_id: u8) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        param1: 1.0, // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        param2: 4.0, // ArduCopter Guided
        ..command_data(drone_id, MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 0.0)
    }
}

fn command_data(drone_id: u8, command: MavCmd, param1: f32, param7: f32) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
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
    }
}

fn send_command_for(drone_id: u8, command: MavCmd, param1: f32, param7: f32) -> Result<(), String> {
    send_command_data(command_data(drone_id, command, param1, param7))
}

fn send_command_data(data: COMMAND_LONG_DATA) -> Result<(), String> {
    let _guard = control_lock(data.target_system)?;
    let link = control_link_for(data.target_system)?;
    link.send(&control_header(), &MavMessage::COMMAND_LONG(data.clone()))
        .map_err(|error| error.to_string())?;
    wait_for_ack(link.as_ref(), &data, COMMAND_TIMEOUT)
}

fn wait_for_ack(
    link: &dyn mavlink::MavConnection<MavMessage>,
    data: &COMMAND_LONG_DATA,
    timeout: Duration,
) -> Result<(), String> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match link.recv() {
            Ok((header, MavMessage::COMMAND_ACK(ack)))
                if header.system_id == data.target_system
                    && header.component_id == data.target_component
                    && ack.command == data.command =>
            {
                match ack.result {
                    MavResult::MAV_RESULT_ACCEPTED => return Ok(()),
                    MavResult::MAV_RESULT_IN_PROGRESS => continue,
                    result => {
                        return Err(format!(
                            "Drone {} rejected {:?}: {result:?}",
                            data.target_system, data.command
                        ))
                    }
                }
            }
            Err(MessageReadError::Io(error))
                if error.kind() != std::io::ErrorKind::WouldBlock
                    && error.kind() != std::io::ErrorKind::TimedOut =>
            {
                return Err(error.to_string())
            }
            _ => {}
        }
    }
    Err(format!(
        "Drone {} did not acknowledge {:?}",
        data.target_system, data.command
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    struct AckLink(std::sync::Mutex<std::collections::VecDeque<(mavlink::MavHeader, MavMessage)>>);

    impl mavlink::MavConnection<MavMessage> for AckLink {
        fn recv(&self) -> Result<(mavlink::MavHeader, MavMessage), MessageReadError> {
            self.0.lock().unwrap().pop_front().ok_or_else(|| {
                MessageReadError::Io(std::io::Error::from(std::io::ErrorKind::TimedOut))
            })
        }
        fn send(
            &self,
            _: &mavlink::MavHeader,
            _: &MavMessage,
        ) -> Result<usize, mavlink::error::MessageWriteError> {
            Ok(1)
        }
        fn set_protocol_version(&mut self, _: mavlink::MavlinkVersion) {}
        fn get_protocol_version(&self) -> mavlink::MavlinkVersion {
            mavlink::MavlinkVersion::V2
        }
    }

    fn ack(id: u8, command: MavCmd, result: MavResult) -> (mavlink::MavHeader, MavMessage) {
        (
            mavlink::MavHeader {
                system_id: id,
                component_id: 1,
                sequence: 0,
            },
            MavMessage::COMMAND_ACK(mavlink::common::COMMAND_ACK_DATA { command, result }),
        )
    }

    #[test]
    fn ack_rejection_and_missing_ack_are_errors() {
        let data = guided_mode_command(1);
        let link = AckLink(std::sync::Mutex::new(
            [ack(1, data.command, MavResult::MAV_RESULT_DENIED)].into(),
        ));
        assert!(wait_for_ack(&link, &data, COMMAND_TIMEOUT)
            .unwrap_err()
            .contains("DENIED"));
        assert!(wait_for_ack(&link, &data, Duration::ZERO)
            .unwrap_err()
            .contains("did not acknowledge"));
    }

    #[test]
    fn ack_must_match_drone_and_command_and_finish() {
        let data = guided_mode_command(2);
        let link = AckLink(std::sync::Mutex::new(
            [
                ack(1, data.command, MavResult::MAV_RESULT_DENIED),
                ack(2, MavCmd::MAV_CMD_NAV_TAKEOFF, MavResult::MAV_RESULT_DENIED),
                ack(2, data.command, MavResult::MAV_RESULT_IN_PROGRESS),
                ack(2, data.command, MavResult::MAV_RESULT_ACCEPTED),
            ]
            .into(),
        ));
        assert!(wait_for_ack(&link, &data, COMMAND_TIMEOUT).is_ok());
        assert!(link.0.lock().unwrap().is_empty());
    }

    #[test]
    fn guided_mode_enables_arducopter_custom_mode() {
        let data = guided_mode_command(2);
        assert_eq!(data.target_system, 2);
        assert_eq!(data.command, MavCmd::MAV_CMD_DO_SET_MODE);
        assert_eq!(data.param1 as u8 & 1, 1);
        assert_eq!(data.param2, 4.0);
    }
}
