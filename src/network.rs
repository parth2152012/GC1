use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;

/// Datagram ceiling, including the JSON envelope, so IP never fragments the packet.
pub const MAX_DATAGRAM_SIZE: usize = 1400;
pub const MAX_FRAGMENT_DATA: usize = 600;
const REASSEMBLY_TTL: Duration = Duration::from_secs(10);
const MAX_IN_FLIGHT_MESSAGES: usize = 64;
const MAX_MESSAGE_SIZE: usize = 8 * 1024 * 1024;

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
    let mut routing_table: HashMap<u8, SocketAddr> = HashMap::new();
    loop {
        let Ok((size, src)) = socket.recv_from(&mut buf).await else {
            continue;
        };
        let Ok(packet) = serde_json::from_slice::<PacketHeader>(&buf[..size]) else {
            continue;
        };
        routing_table.insert(packet.sender_id, src);
        if packet.target_id != 1 && packet.target_id != 255 {
            if let Some(&next_hop) = routing_table.get(&packet.target_id) {
                if let Some(encoded) = serialize_packet(&packet) {
                    let _ = socket.send_to(&encoded, next_hop).await;
                }
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
            payload: vec![0; MAX_FRAGMENT_DATA],
        };
        assert!(serialize_packet(&packet).unwrap().len() <= MAX_DATAGRAM_SIZE);
    }
}
