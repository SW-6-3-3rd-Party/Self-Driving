import socket
import struct
import time

PORT = 5011

AEB1_SIZE = 22
FMT_AEB1_HEADER = "<4sBBHI"
FMT_AEB1_TAIL = "<IHHH"
FMT_LEGACY_V1 = "<fffBBB"
FMT_LEGACY_V2 = "<fffffBBBB"

VALID_LEFT = 1 << 0
VALID_RIGHT = 1 << 1
VALID_TOF = 1 << 2


def valid_text(mask):
    return "/".join(
        [
            "LEFT" if mask & VALID_LEFT else "left--",
            "RIGHT" if mask & VALID_RIGHT else "right--",
            "TOF" if mask & VALID_TOF else "tof--",
        ]
    )


def cm(value_m):
    return value_m * 100.0


def cm_x10_to_m(value):
    return float(value) / 1000.0


def decode_aeb1(data):
    if len(data) < AEB1_SIZE:
        raise ValueError("short AEB1 packet")
    magic, version, valid_mask, tof_diag, sequence = struct.unpack_from(
        FMT_AEB1_HEADER, data, 0
    )
    if magic != b"AEB1" or version != 1:
        raise ValueError("not an AEB1 packet")
    timestamp_ms, tof_cm_x10, left_cm_x10, right_cm_x10 = struct.unpack_from(
        FMT_AEB1_TAIL, data, 12
    )
    return {
        "version": "AEB1",
        "valid": valid_mask,
        "fault": tof_diag & 0xFF,
        "diag": tof_diag,
        "sequence": sequence,
        "timestamp_ms": timestamp_ms,
        "left_m": cm_x10_to_m(left_cm_x10),
        "right_m": cm_x10_to_m(right_cm_x10),
        "tof_m": cm_x10_to_m(tof_cm_x10),
        "filtered_tof_m": cm_x10_to_m(tof_cm_x10),
        "front_obstacle_m": cm_x10_to_m(tof_cm_x10),
    }


def decode_legacy(data):
    if len(data) >= struct.calcsize(FMT_LEGACY_V2):
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
        ) = struct.unpack(FMT_LEGACY_V2, data[: struct.calcsize(FMT_LEGACY_V2)])
        return {
            "version": "legacy-v2",
            "valid": sensor_valid,
            "fault": sensor_fault,
            "alive": alive_count,
            "distance_valid": distance_valid,
            "left_m": left_m,
            "right_m": right_m,
            "tof_m": tof_m,
            "filtered_tof_m": filtered_tof_m,
            "front_obstacle_m": front_obstacle_m,
        }

    if len(data) >= struct.calcsize(FMT_LEGACY_V1):
        left_m, right_m, tof_m, sensor_valid, sensor_fault, alive_count = struct.unpack(
            FMT_LEGACY_V1, data[: struct.calcsize(FMT_LEGACY_V1)]
        )
        return {
            "version": "legacy-v1",
            "valid": sensor_valid,
            "fault": sensor_fault,
            "alive": alive_count,
            "left_m": left_m,
            "right_m": right_m,
            "tof_m": tof_m,
            "filtered_tof_m": tof_m,
            "front_obstacle_m": tof_m,
        }

    raise ValueError(f"short packet: {len(data)} bytes")


def decode_packet(data):
    if len(data) >= 4 and data[:4] == b"AEB1":
        return decode_aeb1(data)
    return decode_legacy(data)


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

    try:
        packet = decode_packet(data)
    except ValueError as error:
        print(f"rejected packet from {addr}: {error}; {len(data)} bytes {data.hex()}")
        continue

    alive_note = ""
    alive = packet.get("alive")
    if alive is not None:
        alive_ok = True if last_alive is None else alive == ((last_alive + 1) & 0x0F)
        last_alive = alive
        alive_note = f" alive={alive:02d} alive_ok={int(alive_ok)}"

    sequence_note = ""
    if "sequence" in packet:
        sequence_note = f" seq={packet['sequence']}"

    print(
        f"[{packet['version']}] from={addr[0]}{alive_note}{sequence_note} "
        f"dt={dt_ms:6.1f}ms | "
        f"valid=0x{packet['valid']:02X}({valid_text(packet['valid'])}) "
        f"fault=0x{packet['fault']:02X} | "
        f"LEFT={cm(packet['left_m']):6.1f}cm "
        f"RIGHT={cm(packet['right_m']):6.1f}cm "
        f"TOF={cm(packet['tof_m']):6.1f}cm "
        f"F_TOF={cm(packet['filtered_tof_m']):6.1f}cm "
        f"FRONT_OBS={cm(packet['front_obstacle_m']):6.1f}cm",
        flush=True,
    )
