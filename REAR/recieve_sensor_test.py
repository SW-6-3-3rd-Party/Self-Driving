import socket
import struct
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 5012
PAYLOAD_SIZE = 11

MOTOR_STATES = {
    0: "disabled",
    1: "idle",
    2: "running",
    3: "fault",
}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

print(f"Listening UDP {UDP_IP}:{UDP_PORT}")
print("Expected payload: <float left_m, float right_m, uint8 valid, uint8 motor_state, uint8 alive>")
print()

last_alive = None
last_rx_time = time.time()

while True:
    try:
        data, addr = sock.recvfrom(2048)
        now = time.time()
        dt_ms = (now - last_rx_time) * 1000.0
        last_rx_time = now

        if len(data) < PAYLOAD_SIZE:
            print(f"[WARN] short packet from {addr}: {len(data)} bytes")
            continue

        payload = data[:PAYLOAD_SIZE]
        left_m, right_m, valid, motor_state, alive = struct.unpack("<ffBBB", payload)

        left_valid = bool(valid & 0x01)
        right_valid = bool(valid & 0x02)

        alive_note = ""
        if last_alive is not None:
            expected = (last_alive + 1) & 0x0F
            if alive != expected:
                alive_note = f"  alive jump expected={expected}"
        last_alive = alive

        print(
            f"from={addr[0]}:{addr[1]}  "
            f"dt={dt_ms:6.1f} ms  "
            f"alive={alive:2d}{alive_note}  "
            f"valid=0x{valid:02X} "
            f"L={'OK' if left_valid else '--'} {left_m:6.3f} m  "
            f"R={'OK' if right_valid else '--'} {right_m:6.3f} m  "
            f"motor={motor_state}({MOTOR_STATES.get(motor_state, 'unknown')})"
        )

    except socket.timeout:
        print("[TIMEOUT] no Rear UDP packet for 1s")
    except KeyboardInterrupt:
        print("\nStopped")
        break