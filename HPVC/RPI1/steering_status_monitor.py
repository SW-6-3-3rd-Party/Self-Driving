"""Listen for Front TC375 5102 steering status packets and print them."""

from __future__ import annotations

import argparse
import json
import socket
import time

from TC375_front.status import StatusProtocolError, unpack_status


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5102)
    parser.add_argument("--timeout", type=float, default=0.20)
    parser.add_argument("--count", type=int, default=0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(args.timeout)
    print(f"Listening for steering status on {args.host}:{args.port}")

    last_print = 0.0
    printed = 0
    while True:
        try:
            data, address = sock.recvfrom(2048)
        except socket.timeout:
            if args.timeout > 0:
                print("Steering status timeout", flush=True)
            continue
        try:
            status = unpack_status(data)
        except StatusProtocolError as error:
            print(f"Rejected packet from {address[0]}: {error}")
            continue
        now = time.monotonic()
        if now - last_print >= 0.25:
            print(
                json.dumps(
                    {
                        "source": f"{address[0]}:{address[1]}",
                        "sequence": status.sequence,
                        "mode": status.mode,
                        "authorized": status.command_authorized,
                        "target_angle_rad": round(status.target_angle_rad, 6),
                        "packet_age_s": round(status.packet_age_s, 3),
                        "last_status": status.last_status,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            last_print = now
            printed += 1
            if args.count > 0 and printed >= args.count:
                return


if __name__ == "__main__":
    main()
