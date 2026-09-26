use tokio::net::UdpSocket;
use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use std::net::SocketAddr;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum PacketType {
    Heartbeat,
    PoiTelemetry,
    Command,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PacketHeader {
    pub sender_id: u8,
    pub target_id: u8,
    pub packet_type: PacketType,
    pub timestamp_ms: u64,
    pub payload: Vec<u8>,
}

pub async fn start_routing_mesh() {
    let socket = UdpSocket::bind("0.0.0.0:6001").await.unwrap();
    let mut buf = [0u8; 2048];
    let mut routing_table: HashMap<u8, SocketAddr> = HashMap::new();

    loop {
        if let Ok((size, src)) = socket.recv_from(&mut buf).await {
            if let Ok(packet) = serde_json::from_slice::<PacketHeader>(&buf[..size]) {
                routing_table.insert(packet.sender_id, src);

                let local_node_id = 1; // Node identifier
                if packet.target_id == local_node_id || packet.target_id == 255 {
                    // Handle incoming payload (Direct / Broadcast)
                } else if let Some(&next_hop) = routing_table.get(&packet.target_id) {
                    // Forward payload over multi-hop mesh
                    let _ = socket.send_to(&buf[..size], next_hop).await;
                }
            }
        }
    }
}