#!/usr/bin/env python3
"""Side-ultrasonic gated LCA steering test for RPi #1.

This is a temporary side-only LCA actuator test. It reads RPi #2 metrics,
checks the requested side ultrasonic distance, then sends the existing R1SC
steering command directly to the front TC375.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import subprocess
import sys
import time
import urllib.request
import zlib


MAGIC = b"R1SC"
VERSION = 1
HEADER_SIZE = 32
CONTROL_STEERING_ANGLE = 1
FLAG_STEERING_VALID = 1 << 0
FLAG_UPSTREAM_VALID = 1 << 2
BODY_FORMAT = "<4sBBBBIQffHHI"


def pack_steering(sequence: int, angle_rad: float, rate_rad_s: float, alive_count: int) -> bytes:
    body = struct.pack(
        BODY_FORMAT,
        MAGIC,
        VERSION,
        CONTROL_STEERING_ANGLE,
        FLAG_STEERING_VALID | FLAG_UPSTREAM_VALID,
        HEADER_SIZE,
        sequence & 0xFFFFFFFF,
        time.monotonic_ns() // 1000,
        float(angle_rad),
        float(rate_rad_s),
        alive_count & 0xFFFF,
        0,
        0,
    )
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def fetch_metrics(url: str, timeout_s: float) -> dict:
    with urllib.request.urlopen(url, timeout=timeout_s) as response:
        return json.loads(response.read().decode("utf-8"))


def read_side_distance(args: argparse.Namespace) -> tuple[float | None, bool]:
    if args.skip_sensor_check:
        return None, True
    if args.side_distance is not None:
        return args.side_distance, math.isfinite(args.side_distance)

    metrics = fetch_metrics(args.rpi2_metrics_url, args.http_timeout_s)
    if args.direction == "left":
        distance = metrics.get("left_ultrasonic_m")
        valid = bool(metrics.get("left_ultrasonic_valid"))
    else:
        distance = metrics.get("right_ultrasonic_m")
        valid = bool(metrics.get("right_ultrasonic_valid"))

    if distance is None:
        return None, False
    try:
        distance_f = float(distance)
    except (TypeError, ValueError):
        return None, False
    return distance_f, valid and math.isfinite(distance_f)


def stop_rpi1_deployment() -> None:
    subprocess.run(
        ["sudo", "pkill", "-f", "RPI1Deployment.elf"],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def send_profile(args: argparse.Namespace) -> None:
    if args.stop_rpi1_deployment:
        stop_rpi1_deployment()
        time.sleep(0.2)

    direction_sign = 1.0 if args.direction == "left" else -1.0
    steer_angle = direction_sign * abs(args.angle_rad)
    target = (args.tc375_host, args.tc375_port)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if args.source_ip:
            sock.bind((args.source_ip, 0))

        sequence = int(time.monotonic() * 1000) & 0xFFFFFFFF
        alive = 0
        period_s = 1.0 / args.hz

        phases = (
            (steer_angle, args.steer_seconds, "LCA_STEER"),
            (0.0, args.center_seconds, "CENTER"),
        )
        for angle, duration_s, label in phases:
            end_time = time.monotonic() + duration_s
            while time.monotonic() < end_time:
                packet = pack_steering(sequence, angle, args.rate_rad_s, alive)
                sock.sendto(packet, target)
                sequence = (sequence + 1) & 0xFFFFFFFF
                alive = (alive + 1) & 0xFFFF
                print(f"{label} angle_rad={angle:.3f} -> {target[0]}:{target[1]}")
                time.sleep(period_s)
    finally:
        sock.close()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a side-only LCA steering test")
    parser.add_argument("direction", choices=("left", "right"))
    parser.add_argument("--rpi2-metrics-url", default="http://192.168.202.104:8000/metrics.json")
    parser.add_argument("--http-timeout-s", type=float, default=1.0)
    parser.add_argument("--min-side-distance-m", type=float, default=0.35)
    parser.add_argument("--side-distance", type=float, default=None, help="Use a manual distance instead of RPi #2 HTTP")
    parser.add_argument("--skip-sensor-check", action="store_true", help="Send steering without checking ultrasonic distance")
    parser.add_argument("--tc375-host", default="192.168.10.2")
    parser.add_argument("--tc375-port", type=int, default=5100)
    parser.add_argument("--source-ip", default="192.168.10.10")
    parser.add_argument("--angle-rad", type=float, default=0.15)
    parser.add_argument("--rate-rad-s", type=float, default=1.0)
    parser.add_argument("--steer-seconds", type=float, default=2.0)
    parser.add_argument("--center-seconds", type=float, default=0.7)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument(
        "--stop-rpi1-deployment",
        action="store_true",
        help="Stop RPI1Deployment.elf before sending this direct test command",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if args.hz <= 0.0:
        raise SystemExit("--hz must be positive")
    if abs(args.angle_rad) > 0.5:
        raise SystemExit("--angle-rad must be within 0.5 rad")

    distance, valid = read_side_distance(args)
    if args.skip_sensor_check:
        print("sensor_check=SKIPPED")
    else:
        print(f"{args.direction}_ultrasonic distance_m={distance} valid={valid}")
        if not valid:
            print("LCA blocked: ultrasonic value is invalid")
            return 2
        if distance is None or distance < args.min_side_distance_m:
            print(
                "LCA blocked: side gap is too small "
                f"({distance:.3f}m < {args.min_side_distance_m:.3f}m)"
            )
            return 3

    send_profile(args)
    print("LCA side-only test complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
