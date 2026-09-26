import cv2
import socket
import time
import math

RUST_IPC_PORT = 5001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = cv2.VideoCapture(0)

# Local 2D ENU / Cartesian Euclidean distance calculation (in meters)
def calculate_distance_enu(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2
    return math.hypot(x2 - x1, y2 - y1)

# Deduplication store: tracks identified POIs to avoid duplicate telemetry transmissions
detected_pois = []
DEDUPLICATION_RADIUS_METERS = 10.0  # Cluster radius for distinct POI identification

def is_duplicate_poi(new_coord):
    for poi in detected_pois:
        if calculate_distance_enu(poi, new_coord) < DEDUPLICATION_RADIUS_METERS:
            return True
    return False

while True:
    ret, frame = cap.read()
    if ret:
        # Preprocess frame for visual detection pipeline
        resized = cv2.resize(frame, (640, 480))
        
        # Compress JPEG to fit well below the MTU frame size
        _, encoded = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 65])
        payload = encoded.tobytes()
        
        if len(payload) < 64000:
            sock.sendto(payload, ("127.0.0.1", RUST_IPC_PORT))
            
    time.sleep(0.05)  # Frame acquisition rate (~20 FPS)