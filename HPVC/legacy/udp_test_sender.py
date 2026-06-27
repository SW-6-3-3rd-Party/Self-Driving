import socket
import struct
import time

SIMULINK_IP = "192.168.203.109"   # 여기에 Simulink가 실행 중인 PC/Mac IP 입력
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    lane_detected = 1.0
    offset_m = 0.2
    curvature_m = 100.0
    camera_status = 1.0

    data = struct.pack('<ffff', lane_detected, offset_m, curvature_m, camera_status)
    sock.sendto(data, (SIMULINK_IP, PORT))

    time.sleep(0.01)