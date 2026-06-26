import socket
import struct
import time

PORT = 5005

VALID_TOF = 1 << 0
VALID_LEFT = 1 << 1
VALID_RIGHT = 1 << 2

def cm(valid, bit, value_x10):
    return f"{value_x10 / 10.0:.1f} cm" if (valid & bit) else "INVALID"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("0.0.0.0", PORT))
sock.settimeout(2.0)

print(f"Listening UDP 0.0.0.0:{PORT} ...")

last = time.time()

while True:
    try:
        data, addr = sock.recvfrom(2048)
    except socket.timeout:
        print("waiting... no UDP packet")
        continue

    if len(data) < 22:
        print(f"short packet from {addr}: {len(data)} bytes")
        continue

    if data[:4] != b"AEB1":
        print(f"unknown packet from {addr}: {data[:16].hex()}")
        continue

    magic, version, valid, tof_diag, seq, ms, tof, left, right = struct.unpack(
        "<4sBBHIIHHH", data[:22]
    )

    model_id = (tof_diag >> 8) & 0xFF
    diag_code = tof_diag & 0xFF

    print(
        f"from={addr[0]} seq={seq} ms={ms} valid=0x{valid:02X} "
        f"tof_diag=0x{tof_diag:04X}(model=0x{model_id:02X}, code=0x{diag_code:02X}) | "
        f"TOF={cm(valid, VALID_TOF, tof)} | "
        f"LEFT={cm(valid, VALID_LEFT, left)} | "
        f"RIGHT={cm(valid, VALID_RIGHT, right)}"
    )