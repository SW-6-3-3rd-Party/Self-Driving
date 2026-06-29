"""Synthetic 20-second LKAS lane curve sender for Raspberry Pi #2.

This script generates a smooth curved lane in the existing MID2 perception
contract and sends the packets to HPVC over UDP. It is meant for bench testing
or closed-loop integration when the physical camera and ultrasonic sensors are
not connected.

The script can optionally listen for any UDP reply on a local ACK port, but the
real TC375 steering ACK still arrives at HPVC unless HPVC forwards it.
"""

from __future__ import annotations

import argparse
import math
import select
import socket
import time

try:
    from .protocol import (
        FLAG_CAMERA_VALID,
        FLAG_LANE_VALID,
        FLAG_LEFT_ULTRASONIC_VALID,
        FLAG_RIGHT_ULTRASONIC_VALID,
        LaneBoundary,
        PerceptionPacket,
        pack_packet,
    )
except ImportError:  # Allow `python3 virtual_lkas_curve.py` from this directory.
    from protocol import (
        FLAG_CAMERA_VALID,
        FLAG_LANE_VALID,
        FLAG_LEFT_ULTRASONIC_VALID,
        FLAG_RIGHT_ULTRASONIC_VALID,
        LaneBoundary,
        PerceptionPacket,
        pack_packet,
    )


DEFAULT_DEST_HOST = "192.168.10.1"
DEFAULT_DEST_PORT = 5005
DEFAULT_SOURCE_PORT = 5006
DEFAULT_ACK_PORT = 5102


def build_packet(
    sequence: int,
    elapsed_s: float,
    duration_s: float,
    lane_width_m: float,
    curve_amplitude_m: float,
    left_distance_m: float,
    right_distance_m: float,
) -> tuple[PerceptionPacket, dict[str, float]]:
    duration_s = max(duration_s, 0.1)
    phase = min(max(elapsed_s / duration_s, 0.0), 1.0)

    # Smooth single curve: straight -> bend -> straight over the requested span.
    shape = math.sin(math.pi * phase)
    slope = math.cos(math.pi * phase)
    center_offset_m = curve_amplitude_m * shape
    heading_rad = 0.28 * curve_amplitude_m * slope
    curvature_1pm = 0.12 * curve_amplitude_m * math.sin(2.0 * math.pi * phase) / duration_s
    curvature_derivative_1pm2 = 0.0

    left_offset_m = center_offset_m - (lane_width_m * 0.5)
    right_offset_m = center_offset_m + (lane_width_m * 0.5)

    left_boundary = LaneBoundary(
        curvature_1pm=curvature_1pm,
        curvature_derivative_1pm2=curvature_derivative_1pm2,
        heading_rad=heading_rad,
        lateral_offset_m=left_offset_m,
        strength=0.95,
    )
    right_boundary = LaneBoundary(
        curvature_1pm=curvature_1pm,
        curvature_derivative_1pm2=curvature_derivative_1pm2,
        heading_rad=heading_rad,
        lateral_offset_m=right_offset_m,
        strength=0.95,
    )

    packet = PerceptionPacket(
        sequence=sequence,
        frame_timestamp_us=time.monotonic_ns() // 1000,
        ultrasonic_timestamp_us=time.monotonic_ns() // 1000,
        flags=(
            FLAG_CAMERA_VALID
            | FLAG_LANE_VALID
            | FLAG_LEFT_ULTRASONIC_VALID
            | FLAG_RIGHT_ULTRASONIC_VALID
        ),
        left=left_boundary,
        right=right_boundary,
        side_left_distance_m=max(0.05, left_distance_m),
        side_right_distance_m=max(0.05, right_distance_m),
        person_count=0,
        persons=(),
    )
    metrics = {
        "phase": phase,
        "center_offset_m": center_offset_m,
        "heading_rad": heading_rad,
        "curvature_1pm": curvature_1pm,
    }
    return packet, metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--udp-host", default=DEFAULT_DEST_HOST)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_DEST_PORT)
    parser.add_argument("--udp-source-port", type=int, default=DEFAULT_SOURCE_PORT)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--period-sec", type=float, default=0.05)
    parser.add_argument("--lane-width-m", type=float, default=0.40)
    parser.add_argument("--curve-amplitude-m", type=float, default=0.16)
    parser.add_argument("--left-distance-m", type=float, default=1.25)
    parser.add_argument("--right-distance-m", type=float, default=1.25)
    parser.add_argument(
        "--listen-ack-port",
        type=int,
        default=0,
        help="Optional local UDP port to print any forwarded ACK or mirror packet",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Repeat the 20-second curve continuously instead of stopping once",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    dest = (args.udp_host, args.udp_port)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_sock.bind(("", args.udp_source_port))

    ack_sock = None
    if args.listen_ack_port:
        ack_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ack_sock.bind(("0.0.0.0", args.listen_ack_port))
        ack_sock.setblocking(False)
        print(f"ACK listener bound on 0.0.0.0:{args.listen_ack_port}")

    period_s = max(args.period_sec, 0.01)
    duration_s = max(args.duration_sec, period_s)
    print(f"Sending MID2 curve packets to {dest[0]}:{dest[1]}")
    print(f"UDP source port: {args.udp_source_port}")
    print(f"Duration: {duration_s:.2f}s, period: {period_s:.3f}s")

    sequence = 0
    start = time.monotonic()
    next_send = start

    try:
        while True:
            now = time.monotonic()
            if now >= next_send:
                elapsed = now - start
                if elapsed > duration_s and not args.loop:
                    break
                packet, metrics = build_packet(
                    sequence=sequence,
                    elapsed_s=elapsed % duration_s if args.loop else elapsed,
                    duration_s=duration_s,
                    lane_width_m=args.lane_width_m,
                    curve_amplitude_m=args.curve_amplitude_m,
                    left_distance_m=args.left_distance_m,
                    right_distance_m=args.right_distance_m,
                )
                payload = pack_packet(packet)
                send_sock.sendto(payload, dest)
                print(
                    f"seq={sequence} phase={metrics['phase']:.3f} "
                    f"center_offset_m={metrics['center_offset_m']:.3f} "
                    f"heading_rad={metrics['heading_rad']:.4f} "
                    f"curvature_1pm={metrics['curvature_1pm']:.5f} "
                    f"sent={len(payload)}B",
                    flush=True,
                )
                sequence = (sequence + 1) & 0xFFFFFFFF
                next_send += period_s
                if next_send < now:
                    next_send = now + period_s

            if ack_sock is not None:
                ready, _, _ = select.select([ack_sock], [], [], 0.0)
                if ready:
                    data, address = ack_sock.recvfrom(2048)
                    preview = data[:16].hex()
                    print(
                        f"ACK from {address[0]}:{address[1]} len={len(data)} "
                        f"head={preview}",
                        flush=True,
                    )

            sleep_s = min(next_send - time.monotonic(), 0.01)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        pass
    finally:
        send_sock.close()
        if ack_sock is not None:
            ack_sock.close()


if __name__ == "__main__":
    main()
