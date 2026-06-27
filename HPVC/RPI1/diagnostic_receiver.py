"""Receive and print RPi #1 diagnostic UDP packets."""

from __future__ import annotations

import argparse
import socket
import struct
import time
import zlib


PACKET_SIZE = 40
MAGIC = b"R1DG"
VERSION = 1
HEADER_SIZE = 32


def encode_packet(
    packet_valid: bool,
    link_valid: bool,
    control_valid: bool,
    rpi2_flags: int,
    sequence: int,
    packet_age_s: float,
    steering_rad: float,
    rpi1_time_s: float,
) -> bytes:
    payload = bytearray(PACKET_SIZE)
    payload[0:4] = MAGIC
    payload[4] = VERSION
    status = (1 if packet_valid else 0)
    status |= (1 if link_valid else 0) << 1
    status |= (1 if control_valid else 0) << 2
    payload[5] = status & 0xFF
    payload[6] = rpi2_flags & 0xFF
    payload[7] = HEADER_SIZE
    struct.pack_into("<I", payload, 8, sequence & 0xFFFFFFFF)
    struct.pack_into("<d", payload, 12, float(packet_age_s))
    struct.pack_into("<f", payload, 20, float(steering_rad))
    struct.pack_into("<d", payload, 24, float(rpi1_time_s))
    struct.pack_into("<I", payload, 36, zlib.crc32(payload[:36]) & 0xFFFFFFFF)
    return bytes(payload)


def decode_packet(data: bytes) -> dict[str, object]:
    if len(data) != PACKET_SIZE:
        raise ValueError(f"expected {PACKET_SIZE} bytes, received {len(data)}")
    if data[:4] != b"R1DG" or data[4] != 1:
        raise ValueError("invalid diagnostic magic or version")
    expected_crc = struct.unpack_from("<I", data, 36)[0]
    actual_crc = zlib.crc32(data[:36]) & 0xFFFFFFFF
    if expected_crc != actual_crc:
        raise ValueError("diagnostic CRC mismatch")

    status = data[5]
    return {
        "packet_valid": bool(status & 0x01),
        "link_valid": bool(status & 0x02),
        "control_valid": bool(status & 0x04),
        "rpi2_flags": data[6],
        "sequence": struct.unpack_from("<I", data, 8)[0],
        "packet_age_s": struct.unpack_from("<d", data, 12)[0],
        "steering_rad": struct.unpack_from("<f", data, 20)[0],
        "rpi1_time_s": struct.unpack_from("<d", data, 24)[0],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5010)
    parser.add_argument("--timeout", type=float, default=None)
    parser.add_argument("--count", type=int, default=0, help="Exit after printing N samples")
    parser.add_argument(
        "--echo-back",
        action="store_true",
        help="Echo valid diagnostic packets back to the sender for roundtrip probing",
    )
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    if args.timeout is not None:
        sock.settimeout(args.timeout)
    print(f"Listening for RPi #1 diagnostics on {args.host}:{args.port}")

    last_print = 0.0
    printed = 0
    while True:
        try:
            data, address = sock.recvfrom(2048)
        except socket.timeout:
            raise SystemExit("Timed out waiting for RPi #1 diagnostics")
        try:
            values = decode_packet(data)
        except ValueError as error:
            print(f"Rejected packet from {address[0]}: {error}")
            continue
        if args.echo_back:
            try:
                sock.sendto(data, address)
            except OSError as error:
                print(f"Echo failed to {address[0]}: {error}")
        now = time.monotonic()
        if now - last_print >= 0.25:
            print(f"{address[0]}: {values}", flush=True)
            last_print = now
            printed += 1
            if args.count > 0 and printed >= args.count:
                return


if __name__ == "__main__":
    main()
