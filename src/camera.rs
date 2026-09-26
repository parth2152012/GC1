use tokio::net::UdpSocket;

const MAX_CHUNK_SIZE: usize = 1400; // Safe MTU payload size for standard UDP

pub async fn start_listener() {
    let socket = UdpSocket::bind("127.0.0.1:5001").await.unwrap();
    let mut img_buf = [0u8; 65507];

    loop {
        if let Ok((amt, _src)) = socket.recv_from(&mut img_buf).await {
            let raw_frame = &img_buf[..amt];
            let chunks = chunk_payload(raw_frame, MAX_CHUNK_SIZE);

            // Re-transmit fragmented packet chunks across the high-speed p2p mesh network
            for (_idx, chunk) in chunks.iter().enumerate() {
                // Attach framing header or broadcast via network mesh module
                let _ = chunk;
            }
        }
    }
}

/// Splits large image frame byte slices into safe standard MTU network segments
fn chunk_payload(data: &[u8], chunk_size: usize) -> Vec<Vec<u8>> {
    data.chunks(chunk_size)
        .map(|chunk| chunk.to_vec())
        .collect()
}
