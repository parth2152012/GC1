use std::collections::HashMap;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;

use crate::metrics;

const MAX_DATAGRAM_SIZE: usize = 1400;
const HEADER_SIZE: usize = 12;
const FRAME_TTL: Duration = Duration::from_secs(2);
const MAX_FRAMES: usize = 8;

struct Frame {
    chunks: Vec<Option<Vec<u8>>>,
    received: usize,
    created: Instant,
}

/// Receives the bounded `UVX1` datagrams produced by camera.py. Incomplete frames
/// expire, preventing an out-of-order or malicious sender from retaining memory.
pub async fn start_listener() {
    let socket = match UdpSocket::bind("127.0.0.1:5001").await {
        Ok(socket) => socket,
        Err(error) => {
            eprintln!("camera bind failed: {error}");
            return;
        }
    };
    let mut buf = [0u8; MAX_DATAGRAM_SIZE];
    let mut frames: HashMap<u32, Frame> = HashMap::new();
    loop {
        let Ok((size, _)) = socket.recv_from(&mut buf).await else {
            continue;
        };
        frames.retain(|_, frame| frame.created.elapsed() < FRAME_TTL);
        if size < HEADER_SIZE || &buf[..4] != b"UVX1" {
            continue;
        }
        let frame_id = u32::from_be_bytes(buf[4..8].try_into().expect("fixed header"));
        let index = u16::from_be_bytes(buf[8..10].try_into().expect("fixed header")) as usize;
        let total = u16::from_be_bytes(buf[10..12].try_into().expect("fixed header")) as usize;
        if total == 0
            || index >= total
            || total > 4096
            || (!frames.contains_key(&frame_id) && frames.len() >= MAX_FRAMES)
        {
            continue;
        }
        let frame = frames.entry(frame_id).or_insert_with(|| Frame {
            chunks: vec![None; total],
            received: 0,
            created: Instant::now(),
        });
        if frame.chunks.len() != total {
            continue;
        }
        if frame.chunks[index].is_none() {
            frame.received += 1;
            frame.chunks[index] = Some(buf[HEADER_SIZE..size].to_vec());
        }
        if frame.received == total {
            let frame = frames.remove(&frame_id).expect("frame exists");
            let jpeg: Vec<u8> = frame.chunks.into_iter().flatten().flatten().collect();

            // FIX: this used to be `let _ = jpeg;` — the reassembled image evidence was
            // decoded and then thrown away, even though the proposal's own §5.1/§9
            // treat imagery collection as a core mission requirement. Now it is saved
            // to disk and logged as a metrics event so a run's captured evidence is
            // actually reviewable/reproducible.
            let bytes = jpeg.len();
            let path = format!("captures/frame_{frame_id}.jpg");
            if let Err(error) = tokio::fs::create_dir_all("captures").await {
                eprintln!("⚠️  Could not create captures directory: {error}");
            } else if let Err(error) = tokio::fs::write(&path, &jpeg).await {
                eprintln!("⚠️  Could not save captured frame to {path}: {error}");
            } else {
                println!("📸 Saved captured frame #{frame_id} ({bytes} bytes) to {path}");
            }
            metrics::log_event(metrics::MetricEvent::PhotoCaptured {
                frame_id,
                bytes,
                path,
            })
            .await;
        }
    }
}
