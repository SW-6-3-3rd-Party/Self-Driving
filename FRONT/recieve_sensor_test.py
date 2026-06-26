import socket
import struct
import time

PORT = 5011

FMT_V1 = "<fffBBB"
FMT_V2 = "<fffffBBBB"

VALID_LEFT = 1 << 0
VALID_RIGHT = 1 << 1
VALID_TOF = 1 << 2

def valid_text(mask):
    return "/".join([
        "LEFT" if mask & VALID_LEFT else "left--",
        "RIGHT" if mask & VALID_RIGHT else "right--",
        "TOF" if mask & VALID_TOF else "tof--",
    ])

def cm(value_m):
    return value_m * 100.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", PORT))
sock.settimeout(2.0)

print(f"Listening UDP 0.0.0.0:{PORT} ...")
print("Expect HPVC eth0 IP = 192.168.10.1, Front ECU = 192.168.10.11")

last_alive = None
last_time = time.time()

while True:
    try:
        data, addr = sock.recvfrom(2048)
    except socket.timeout:
        print("waiting... no UDP packet")
        continue

    now = time.time()
    dt_ms = (now - last_time) * 1000.0
    last_time = now

    if len(data) >= struct.calcsize(FMT_V2):
        (
            left_m,
            right_m,
            tof_m,
            filtered_tof_m,
            front_obstacle_m,
            distance_valid,
            sensor_valid,
            sensor_fault,
            alive_count,
        ) = struct.unpack(FMT_V2, data[:struct.calcsize(FMT_V2)])

        alive_ok = True if last_alive is None else alive_count == ((last_alive + 1) & 0x0F)
        last_alive = alive_count

        print(
            f"[v2] from={addr[0]} alive={alive_count:02d} "
            f"alive_ok={int(alive_ok)} dt={dt_ms:6.1f}ms | "
            f"valid=0x{sensor_valid:02X}({valid_text(sensor_valid)}) "
            f"fault=0x{sensor_fault:02X} distance_valid={distance_valid} | "
            f"LEFT={cm(left_m):6.1f}cm "
            f"RIGHT={cm(right_m):6.1f}cm "
            f"TOF={cm(tof_m):6.1f}cm "
            f"F_TOF={cm(filtered_tof_m):6.1f}cm "
            f"FRONT_OBS={cm(front_obstacle_m):6.1f}cm",
            flush=True,
        )

    elif len(data) >= struct.calcsize(FMT_V1):
        left_m, right_m, tof_m, sensor_valid, sensor_fault, alive_count = struct.unpack(
            FMT_V1, data[:struct.calcsize(FMT_V1)]
        )

        print(
            f"[v1] from={addr[0]} alive={alive_count:02d} dt={dt_ms:6.1f}ms | "
            f"valid=0x{sensor_valid:02X}({valid_text(sensor_valid)}) "
            f"fault=0x{sensor_fault:02X} | "
            f"LEFT={cm(left_m):6.1f}cm "
            f"RIGHT={cm(right_m):6.1f}cm "
            f"TOF={cm(tof_m):6.1f}cm",
            flush=True,
        )

    else:
        print(f"short packet from {addr}: {len(data)} bytes {data.hex()}")