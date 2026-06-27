"""Console receiver for checking RPi #2 UDP packets before Simulink wiring."""

from __future__ import annotations

import argparse
import json
import socket

try:
    from .protocol import PACKET_SIZE, PAYLOAD_FIELDS, payload_values, unpack_packet
except ImportError:
    from protocol import PACKET_SIZE, PAYLOAD_FIELDS, payload_values, unpack_packet


def main():
    parser = argparse.ArgumentParser(description="Inspect RPi #2 perception UDP packets")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    print(f"Listening on {args.host}:{args.port} for {PACKET_SIZE}-byte packets")
    while True:
        data, address = sock.recvfrom(2048)
        try:
            packet = unpack_packet(data)
            values = payload_values(packet)
            output = {
                "source": f"{address[0]}:{address[1]}",
                "sequence": packet.sequence,
                "frame_timestamp_us": packet.frame_timestamp_us,
                "ultrasonic_timestamp_us": packet.ultrasonic_timestamp_us,
                "flags": packet.flags,
                **dict(zip(PAYLOAD_FIELDS, values)),
            }
            print(json.dumps(output, allow_nan=True))
        except ValueError as exc:
            print(f"Rejected {len(data)}-byte packet from {address}: {exc}")


if __name__ == "__main__":
    main()
