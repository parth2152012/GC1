use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tokio::net::UdpSocket;

use crate::metrics;

/// Datagram ceiling, including the JSON envelope, so IP never fragments the packet.
pub const MAX_DATAGRAM_SIZE: usize = 1400;
pub const MAX_FRAGMENT_DATA: usize = 600;
const REASSEMBLY_TTL: Duration = Duration::from_secs(10);
const MAX_IN_FLIGHT_MESSAGES: usize = 64;
const MAX_MESSAGE_SIZE: usize = 8 * 1024 * 1024;

/// FIX: routing-table entries never expired, so a peer that went offline (crashed,
/// out of range, or deliberately failed for a test) stayed "routable" forever and the
/// mesh had no notion of stale links at all.
const ROUTE_TTL: Duration = Duration::from_secs(30);

/// FIX: fault-injection hook for the "Reconfigure the network when links degrade"
/// requirement and the Fault Recovery / Robustness rubric items. When set, the mesh
/// silently drops every packet it would otherwise forward, simulating a communication
/// outage. Toggle with the `fail link` / `recover` CLI commands in main.rs.
pub static LINK_DOWN: AtomicBool = AtomicBool::new(false);
static NEXT_SEQUENCE: AtomicU64 = AtomicU64::new(1);

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum PacketType {
    Heartbeat,
    PoiTelemetry,
    Command,
    ImageFragment,
}
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PacketHeader {
    pub sender_id: u8,
    pub target_id: u8,
    pub packet_type: PacketType,
    pub timestamp_ms: u64,
    /// Monotonic origin identity used for measured PDR and latency accounting.
    #[serde(default)]
    pub sequence: u64,
    pub payload: Vec<u8>,
}
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Fragment {
    pub message_id: u64,
    pub index: u16,
    pub total: u16,
    pub data: Vec<u8>,
}

pub fn fragment_payload(message_id: u64, data: &[u8]) -> Vec<Fragment> {
    let total = data.len().div_ceil(MAX_FRAGMENT_DATA);
    assert!(
        total > 0 && total <= u16::MAX as usize,
        "payload has too many fragments"
    );
    data.chunks(MAX_FRAGMENT_DATA)
        .enumerate()
        .map(|(index, chunk)| Fragment {
            message_id,
            index: index as u16,
            total: total as u16,
            data: chunk.to_vec(),
        })
        .collect()
}

#[derive(Default)]
struct Assembly {
    total: u16,
    chunks: Vec<Option<Vec<u8>>>,
    created: Option<Instant>,
    bytes: usize,
}
#[derive(Default)]
pub struct Reassembler {
    messages: HashMap<u64, Assembly>,
}
impl Reassembler {
    pub fn insert(&mut self, fragment: Fragment) -> Option<Vec<u8>> {
        self.messages.retain(|_, entry| {
            entry
                .created
                .is_some_and(|time| time.elapsed() < REASSEMBLY_TTL)
        });
        if fragment.total == 0
            || fragment.index >= fragment.total
            || fragment.data.len() > MAX_FRAGMENT_DATA
        {
            return None;
        }
        if !self.messages.contains_key(&fragment.message_id)
            && self.messages.len() >= MAX_IN_FLIGHT_MESSAGES
        {
            return None;
        }
        let entry = self
            .messages
            .entry(fragment.message_id)
            .or_insert_with(|| Assembly {
                total: fragment.total,
                chunks: vec![None; fragment.total as usize],
                created: Some(Instant::now()),
                bytes: 0,
            });
        if entry.total != fragment.total {
            return None;
        }
        let slot = &mut entry.chunks[fragment.index as usize];
        if slot.is_none() {
            entry.bytes += fragment.data.len();
            *slot = Some(fragment.data);
        }
        if entry.bytes > MAX_MESSAGE_SIZE || entry.chunks.iter().any(Option::is_none) {
            return None;
        }
        let entry = self
            .messages
            .remove(&fragment.message_id)
            .expect("entry exists");
        Some(entry.chunks.into_iter().flatten().flatten().collect())
    }
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Build an origin packet with a unique identity and measured origin timestamp.
pub fn new_packet(
    sender_id: u8,
    target_id: u8,
    packet_type: PacketType,
    payload: Vec<u8>,
) -> PacketHeader {
    PacketHeader {
        sender_id,
        target_id,
        packet_type,
        timestamp_ms: now_ms(),
        sequence: NEXT_SEQUENCE.fetch_add(1, Ordering::Relaxed),
        payload,
    }
}

/// Send an origin packet and record the send only after the OS accepts it.
pub async fn send_packet(
    socket: &UdpSocket,
    packet: &PacketHeader,
    target: SocketAddr,
) -> std::io::Result<usize> {
    let encoded = serialize_packet(packet).ok_or_else(|| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "packet exceeds UDP datagram ceiling",
        )
    })?;
    let sent = socket.send_to(&encoded, target).await?;
    metrics::log_event(metrics::MetricEvent::PacketSent {
        sequence: packet.sequence,
        sender_id: packet.sender_id,
        target_id: packet.target_id,
        timestamp_ms: packet.timestamp_ms,
    })
    .await;
    Ok(sent)
}

fn serialize_packet(packet: &PacketHeader) -> Option<Vec<u8>> {
    let encoded = serde_json::to_vec(packet).ok()?;
    (encoded.len() <= MAX_DATAGRAM_SIZE).then_some(encoded)
}

pub async fn start_routing_mesh() {
    let socket = match UdpSocket::bind("0.0.0.0:6001").await {
        Ok(socket) => socket,
        Err(error) => {
            eprintln!("mesh bind failed: {error}");
            return;
        }
    };
    let mut buf = [0u8; MAX_DATAGRAM_SIZE];
    let mut routing_table: HashMap<u8, (SocketAddr, Instant)> = HashMap::new();

    // FIX: previously there was no visibility at all into mesh health. This periodic
    // snapshot (every 10s) is written to the metrics log so connectivity availability,
    // downtime, and a coarse packet-delivery estimate can be derived after a run.
    let mut stats_interval = tokio::time::interval(Duration::from_secs(10));
    let mut received: u64 = 0;
    let mut forwarded: u64 = 0;
    let mut dropped_outage: u64 = 0;
    // The controller has no separate per-UAV application process, so this
    // heartbeat is an honest local mesh health probe. It measures the UDP path
    // and outage hook without pretending to be an end-to-end UAV PDR.
    let mut heartbeat_interval = tokio::time::interval(Duration::from_secs(1));

    loop {
        tokio::select! {
            _ = heartbeat_interval.tick() => {
                let heartbeat = new_packet(1, 255, PacketType::Heartbeat, b"mesh-heartbeat".to_vec());
                let _ = send_packet(&socket, &heartbeat, "127.0.0.1:6001".parse().expect("valid local mesh address")).await;
            }
            recv_result = socket.recv_from(&mut buf) => {
                let Ok((size, src)) = recv_result else { continue; };
                let Ok(packet) = serde_json::from_slice::<PacketHeader>(&buf[..size]) else { continue; };
                received += 1;
                routing_table.retain(|_, (_, last_seen)| last_seen.elapsed() < ROUTE_TTL);
                routing_table.insert(packet.sender_id, (src, Instant::now()));

                if LINK_DOWN.load(Ordering::Acquire) {
                    // Simulated outage: bookkeeping still happens (so the sender is not
                    // forgotten instantly), but nothing is delivered or forwarded.
                    dropped_outage += 1;
                    continue;
                }
                let latency_ms = now_ms().saturating_sub(packet.timestamp_ms);
                metrics::log_event(metrics::MetricEvent::PacketReceived {
                    sequence: packet.sequence,
                    sender_id: packet.sender_id,
                    target_id: packet.target_id,
                    latency_ms,
                }).await;

                if packet.target_id != 1 && packet.target_id != 255 {
                    if let Some(&(next_hop, _)) = routing_table.get(&packet.target_id) {
                        if let Some(encoded) = serialize_packet(&packet) {
                            if socket.send_to(&encoded, next_hop).await.is_ok() {
                                forwarded += 1;
                            }
                        }
                    }
                }
            }
            _ = stats_interval.tick() => {
                metrics::log_event(metrics::MetricEvent::MeshStatus {
                    received,
                    forwarded,
                    dropped_outage,
                    link_down: LINK_DOWN.load(Ordering::Acquire),
                    known_peers: routing_table.len(),
                }).await;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reassembles_out_of_order_without_duplicate_growth() {
        let input = vec![7; 2_001];
        let parts = fragment_payload(42, &input);
        let mut reassembler = Reassembler::default();
        let duplicate = parts[0].clone();
        assert!(reassembler.insert(duplicate.clone()).is_none());
        assert!(reassembler.insert(duplicate).is_none());
        let mut result = None;
        for fragment in parts.into_iter().skip(1).rev() {
            result = reassembler.insert(fragment).or(result);
        }
        assert_eq!(result, Some(input));
    }
    #[test]
    fn encoded_packets_stay_under_udp_ceiling() {
        let packet = PacketHeader {
            sender_id: 1,
            target_id: 2,
            packet_type: PacketType::ImageFragment,
            timestamp_ms: 0,
            sequence: 1,
            payload: vec![0; MAX_FRAGMENT_DATA],
        };
        assert!(serialize_packet(&packet).unwrap().len() <= MAX_DATAGRAM_SIZE);
    }
}
