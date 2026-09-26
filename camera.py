"""Capture JPEG frames and send application-level UDP fragments to UVAX-1."""
import cv2
import socket
import time
import math
import struct

RUST_IPC_PORT = 5001
MAX_DATAGRAM_BYTES = 1400
# 16-byte frame header: magic, frame id, chunk index, chunk count.
HEADER = struct.Struct("!4sIHH")
MAX_CHUNK_BYTES = MAX_DATAGRAM_BYTES - HEADER.size
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = cv2.VideoCapture(0)
frame_id = 0

def calculate_distance_enu(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2
    return math.hypot(x2 - x1, y2 - y1)

detected_pois = []
DEDUPLICATION_RADIUS_METERS = 10.0

def is_duplicate_poi(new_coord):
    return any(calculate_distance_enu(poi, new_coord) < DEDUPLICATION_RADIUS_METERS for poi in detected_pois)

def send_frame(payload):
    """Send a JPEG as bounded UDP datagrams; no packet exceeds 1400 bytes."""
    global frame_id
    chunks = [payload[offset:offset + MAX_CHUNK_BYTES] for offset in range(0, len(payload), MAX_CHUNK_BYTES)]
    if not chunks or len(chunks) > 0xFFFF:
        return
    for index, chunk in enumerate(chunks):
        datagram = HEADER.pack(b"UVX1", frame_id, index, len(chunks)) + chunk
        assert len(datagram) <= MAX_DATAGRAM_BYTES
        sock.sendto(datagram, ("127.0.0.1", RUST_IPC_PORT))
    frame_id = (frame_id + 1) & 0xFFFFFFFF

try:
    while True:
        ret, frame = cap.read()
        if ret:
            resized = cv2.resize(frame, (640, 480))
            ok, encoded = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if ok:
                send_frame(encoded.tobytes())
        time.sleep(0.05)
finally:
    cap.release()
    sock.close()
