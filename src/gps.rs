use mavlink::common::{MavMessage, PositionTargetTypemask, SET_POSITION_TARGET_GLOBAL_INT_DATA};
use mavlink::error::MessageReadError;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicI32, AtomicU8, Ordering};
use std::time::Duration;

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

pub fn send_guided_target(target_lat: f64, target_lon: f64, alt_m: f32) {
    let socket = match UdpSocket::bind("127.0.0.1:0") {
        Ok(s) => s,
        Err(_) => return,
    };

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

    let mut buffer = Vec::new();
    if mavlink::write_v2_msg(&mut buffer, header, &set_pos_msg).is_ok() {
        let _ = socket.send_to(&buffer, "127.0.0.1:14550");
    }
}
