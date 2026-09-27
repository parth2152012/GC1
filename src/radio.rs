//! Real UDP forwarding between simulated nodes. Link availability is derived from
//! fresh telemetry, not localhost reachability. This is a radio-range emulator.
use crate::{
    network,
    peer::{enu_distance_meters, PositionEnu},
};
use serde::{Deserialize, Serialize};
use std::{
    collections::{HashMap, VecDeque},
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};
use tokio::{
    net::UdpSocket,
    sync::{oneshot, Mutex, RwLock},
};

#[derive(Clone, Copy)]
struct Location {
    point: PositionEnu,
    updated: Instant,
}
#[derive(Serialize, Deserialize)]
struct Datagram {
    sequence: u64,
    route: Vec<usize>,
    hop: usize,
    ack: bool,
    payload: String,
}
pub struct Radio {
    sockets: Vec<Arc<UdpSocket>>,
    locations: RwLock<Vec<Option<Location>>>,
    pending: Mutex<HashMap<u64, oneshot::Sender<usize>>>,
    sequence: AtomicU64,
}
impl Radio {
    pub async fn start(count: usize, port: u16) -> Result<Arc<Self>, String> {
        let mut sockets = Vec::new();
        for id in 0..=count {
            sockets.push(Arc::new(
                UdpSocket::bind((
                    std::net::Ipv4Addr::LOCALHOST,
                    if port == 0 { 0 } else { port + id as u16 },
                ))
                .await
                .map_err(|e| e.to_string())?,
            ));
        }
        let radio = Arc::new(Self {
            sockets,
            locations: RwLock::new(vec![None; count + 1]),
            pending: Mutex::new(HashMap::new()),
            sequence: AtomicU64::new(1),
        });
        for id in 0..=count {
            let weak = Arc::downgrade(&radio);
            let socket = radio.sockets[id].clone();
            tokio::spawn(async move {
                let mut buffer = [0; 1400];
                loop {
                    let receive =
                        tokio::time::timeout(Duration::from_secs(1), socket.recv_from(&mut buffer))
                            .await;
                    let Some(radio) = weak.upgrade() else {
                        break;
                    };
                    let Ok(Ok((size, source))) = receive else {
                        continue;
                    };
                    let Ok(mut packet) = serde_json::from_slice::<Datagram>(&buffer[..size]) else {
                        continue;
                    };
                    if packet.route.len() < 2
                        || packet.route.len() > radio.sockets.len()
                        || packet.hop == 0
                        || packet.hop >= packet.route.len()
                        || packet.route[packet.hop] != id
                    {
                        continue;
                    }
                    let previous = packet.route[packet.hop - 1];
                    if previous >= radio.sockets.len()
                        || radio.sockets[previous].local_addr().ok() != Some(source)
                        || !radio.link(previous, id).await
                    {
                        continue;
                    }
                    if packet.hop + 1 == packet.route.len() {
                        if packet.ack {
                            if let Some(waiter) =
                                radio.pending.lock().await.remove(&packet.sequence)
                            {
                                let _ = waiter.send(packet.route.len() - 1);
                            }
                            continue;
                        }
                        println!(
                            "MESH delivered {} -> {id}, {} hops: {}",
                            packet.route[0],
                            packet.route.len() - 1,
                            packet.payload
                        );
                        packet.ack = true;
                        packet.route.reverse();
                        packet.hop = 0;
                    }
                    let _ = radio.forward(&mut packet).await;
                }
            });
        }
        Ok(radio)
    }
    pub async fn update(&self, id: usize, point: Option<PositionEnu>) {
        self.locations.write().await[id] = point.map(|point| Location {
            point,
            updated: Instant::now(),
        });
    }
    async fn link(&self, a: usize, b: usize) -> bool {
        if network::LINK_DOWN.load(Ordering::Acquire) {
            return false;
        }
        let locations = self.locations.read().await;
        match (
            locations.get(a).and_then(|x| *x),
            locations.get(b).and_then(|x| *x),
        ) {
            (Some(a), Some(b)) => {
                a.updated.elapsed() < Duration::from_secs(3)
                    && b.updated.elapsed() < Duration::from_secs(3)
                    && enu_distance_meters(a.point, b.point) <= 100.0
            }
            _ => false,
        }
    }
    pub async fn route(&self, source: usize, destination: usize) -> Option<Vec<usize>> {
        if source >= self.sockets.len() || destination >= self.sockets.len() {
            return None;
        }
        let mut previous = vec![None; self.sockets.len()];
        let mut queue = VecDeque::from([source]);
        previous[source] = Some(source);
        while let Some(id) = queue.pop_front() {
            if id == destination {
                let mut path = vec![id];
                while *path.last()? != source {
                    path.push(previous[*path.last()?]?);
                }
                path.reverse();
                return Some(path);
            }
            for (next, parent) in previous.iter_mut().enumerate() {
                if parent.is_none() && self.link(id, next).await {
                    *parent = Some(id);
                    queue.push_back(next);
                }
            }
        }
        None
    }
    async fn forward(&self, packet: &mut Datagram) -> Result<(), String> {
        let source = packet.route[packet.hop];
        let destination = packet.route[packet.hop + 1];
        if !self.link(source, destination).await {
            return Err("radio hop unavailable".into());
        }
        packet.hop += 1;
        let encoded = serde_json::to_vec(packet).map_err(|e| e.to_string())?;
        if encoded.len() > 1400 {
            return Err("mesh packet too large".into());
        }
        self.sockets[source]
            .send_to(
                &encoded,
                self.sockets[destination]
                    .local_addr()
                    .map_err(|e| e.to_string())?,
            )
            .await
            .map_err(|e| e.to_string())?;
        Ok(())
    }
    pub async fn report(&self, source: usize, payload: String) -> Result<usize, String> {
        let route = self
            .route(source, 0)
            .await
            .ok_or("no radio route to center")?;
        if route.len() < 2 {
            return Err("report must originate at a drone".into());
        }
        let sequence = self.sequence.fetch_add(1, Ordering::Relaxed);
        let (tx, rx) = oneshot::channel();
        self.pending.lock().await.insert(sequence, tx);
        let mut packet = Datagram {
            sequence,
            route,
            hop: 0,
            ack: false,
            payload,
        };
        let result = match self.forward(&mut packet).await {
            Err(e) => Err(e),
            Ok(()) => match tokio::time::timeout(Duration::from_secs(10), rx).await {
                Ok(Ok(hops)) => Ok(hops),
                _ => Err("center acknowledgement timed out".into()),
            },
        };
        self.pending.lock().await.remove(&sequence);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[tokio::test]
    async fn packets_cross_three_hops_and_fail_when_relay_disappears() {
        let radio = Radio::start(3, 0).await.unwrap();
        for id in 0..=3 {
            radio
                .update(
                    id,
                    Some(PositionEnu {
                        north_m: id as f64 * 75.0,
                        east_m: 0.0,
                        up_m: 0.0,
                    }),
                )
                .await;
        }
        assert_eq!(radio.report(3, "poi=42".into()).await.unwrap(), 3);
        radio.update(2, None).await;
        assert!(radio
            .report(3, "must not bypass missing relay".into())
            .await
            .is_err());
        radio
            .update(
                2,
                Some(PositionEnu {
                    north_m: 150.0,
                    east_m: 0.0,
                    up_m: 0.0,
                }),
            )
            .await;
        assert_eq!(radio.report(3, "recovered".into()).await.unwrap(), 3);
    }
    #[tokio::test]
    async fn twenty_nodes_deliver_over_nineteen_intermediate_hops() {
        let radio = Radio::start(20, 0).await.unwrap();
        for id in 0..=20 {
            radio
                .update(
                    id,
                    Some(PositionEnu {
                        north_m: id as f64 * 75.0,
                        east_m: 0.0,
                        up_m: 0.0,
                    }),
                )
                .await;
        }
        assert_eq!(
            radio.report(20, "twenty-node-path".into()).await.unwrap(),
            20
        );
    }
}
