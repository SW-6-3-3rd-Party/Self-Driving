#!/usr/bin/env python3
"""Side-ultrasonic gated LCA steering test for HPVC.

This is a temporary side-only LCA actuator test. It reads gate sensor values
from JSON/HTTP, checks the requested side ultrasonic clearance, then sends the
HPVC steering command directly to the front TC375.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import urllib.request
import zlib


MAGIC = b"HPSC"
VERSION = 1
HEADER_SIZE = 32
CONTROL_STEERING_ANGLE = 1
FLAG_STEERING_VALID = 1 << 0
FLAG_UPSTREAM_VALID = 1 << 2
BODY_FORMAT = "<4sBBBBIQffHHI"
DIAG_MAGIC = b"R1DG"
DIAG_VERSION = 1
DIAG_HEADER_SIZE = 32
DIAG_PACKET_SIZE = 40


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


def pack_diagnostic(
    packet_valid: bool,
    link_valid: bool,
    control_valid: bool,
    rpi2_flags: int,
    sequence: int,
    packet_age_s: float,
    steering_rad: float,
    rpi1_time_s: float,
) -> bytes:
    body = bytearray(DIAG_PACKET_SIZE)
    body[0:4] = DIAG_MAGIC
    body[4] = DIAG_VERSION
    status = (1 if packet_valid else 0)
    status |= (1 if link_valid else 0) << 1
    status |= (1 if control_valid else 0) << 2
    body[5] = status & 0xFF
    body[6] = rpi2_flags & 0xFF
    body[7] = DIAG_HEADER_SIZE
    struct.pack_into("<I", body, 8, sequence & 0xFFFFFFFF)
    struct.pack_into("<d", body, 12, float(packet_age_s))
    struct.pack_into("<f", body, 20, float(steering_rad))
    struct.pack_into("<d", body, 24, float(rpi1_time_s))
    struct.pack_into("<I", body, 36, zlib.crc32(body[:36]) & 0xFFFFFFFF)
    return bytes(body)


def probe_diagnostic_roundtrip(
    args: argparse.Namespace,
    packet_valid: bool,
    link_valid: bool,
    control_valid: bool,
    rpi2_flags: int,
    steering_rad: float,
) -> tuple[bool, float | None]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("", 0))
        sock.settimeout(args.roundtrip_timeout_s)
        sequence = int(time.monotonic() * 1000) & 0xFFFFFFFF
        packet = pack_diagnostic(
            packet_valid=packet_valid,
            link_valid=link_valid,
            control_valid=control_valid,
            rpi2_flags=rpi2_flags,
            sequence=sequence,
            packet_age_s=0.0,
            steering_rad=steering_rad,
            rpi1_time_s=time.monotonic(),
        )
        started = time.monotonic()
        sock.sendto(packet, (args.diagnostic_host, args.diagnostic_port))
        try:
            reply, _ = sock.recvfrom(2048)
        except socket.timeout:
            return False, None
        if len(reply) != DIAG_PACKET_SIZE or reply[:4] != DIAG_MAGIC or reply[4] != DIAG_VERSION:
            return False, None
        if zlib.crc32(reply[:36]) & 0xFFFFFFFF != struct.unpack_from("<I", reply, 36)[0]:
            return False, None
        reply_sequence = struct.unpack_from("<I", reply, 8)[0]
        if reply_sequence != sequence:
            return False, None
        return True, (time.monotonic() - started) * 1000.0
    finally:
        sock.close()


def fetch_json_source(url: str | None, file_path: str | None, timeout_s: float) -> dict:
    if file_path:
        return json.loads(Path(file_path).read_text(encoding="utf-8"))
    if url:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            return json.loads(response.read().decode("utf-8"))
    return {}


def _coerce_distance(value) -> tuple[float | None, bool]:
    if value is None:
        return None, False
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return None, False
    return distance, math.isfinite(distance)


def _resolve_first(metrics: dict, keys: list[str]) -> tuple[float | None, bool, str | None]:
    for key in keys:
        if key in metrics:
            distance, valid = _coerce_distance(metrics.get(key))
            if distance is not None:
                return distance, valid, key
    return None, False, None


def read_gate_distances(
    args: argparse.Namespace,
) -> tuple[dict[str, tuple[float | None, bool, str | None]], bool, str]:
    if args.skip_sensor_check:
        return {}, True, "skipped"

    if args.gate_distance is not None:
        distance = float(args.gate_distance)
        return {
            "rear": (distance, math.isfinite(distance), "manual"),
            "middle": (distance, math.isfinite(distance), "manual"),
            "front": (distance, math.isfinite(distance), "manual"),
        }, math.isfinite(distance), "manual"

    metrics = fetch_json_source(args.gate_source_url, args.gate_source_file, args.http_timeout_s)
    if not metrics:
        return {}, False, "empty"

    side = args.direction.lower()
    prefix = "left" if side == "left" else "right"
    gates = {
        "rear": [
            f"rear_{prefix}_m",
            f"rear_{prefix}_distance_m",
            f"rear_{prefix}_ultrasonic_m",
            f"{prefix}_rear_m",
            f"{prefix}_rear_distance_m",
        ],
        "middle": [
            f"middle_{prefix}_m",
            f"middle_{prefix}_distance_m",
            f"{prefix}_middle_m",
            f"{prefix}_middle_distance_m",
            f"{prefix}_ultrasonic_m",
            f"{prefix}_ultrasonic_distance_m",
        ],
        "front": [
            f"front_{prefix}_m",
            f"front_{prefix}_distance_m",
            f"front_{prefix}_diag_distance_m",
            f"{prefix}_front_m",
            f"{prefix}_front_distance_m",
            f"side_{prefix}_distance_m",
        ],
    }
    compatibility_keys = [
        f"{prefix}_ultrasonic_m",
        f"{prefix}_ultrasonic_distance_m",
    ]

    resolved = {}
    ok = True
    for name, keys in gates.items():
        distance, valid, source_key = _resolve_first(metrics, keys)
        if distance is None:
            distance, valid, source_key = _resolve_first(metrics, compatibility_keys)
        resolved[name] = (distance, valid, source_key)
        ok = ok and valid and distance is not None
    return resolved, ok, "metrics"


def stop_hpvc_deployment() -> None:
    subprocess.run(
        ["sudo", "pkill", "-f", "HPVCDeployment.elf"],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def send_profile(args: argparse.Namespace) -> None:
    if args.stop_hpvc_deployment:
        stop_hpvc_deployment()
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
    parser.add_argument("--gate-source-url", default="http://192.168.202.104:8000/metrics.json")
    parser.add_argument("--gate-source-file", default=None, help="Read gate distances from a local JSON file")
    parser.add_argument("--http-timeout-s", type=float, default=1.0)
    parser.add_argument("--min-side-distance-m", type=float, default=0.35)
    parser.add_argument(
        "--gate-distance",
        type=float,
        default=None,
        help="Use a manual distance for all three gate positions",
    )
    parser.add_argument("--skip-sensor-check", action="store_true", help="Send steering without checking ultrasonic distance")
    parser.add_argument("--tc375-host", default="192.168.10.11")
    parser.add_argument("--tc375-port", type=int, default=5100)
    parser.add_argument("--source-ip", default="192.168.10.1")
    parser.add_argument("--angle-rad", type=float, default=0.15)
    parser.add_argument("--rate-rad-s", type=float, default=1.0)
    parser.add_argument("--steer-seconds", type=float, default=2.0)
    parser.add_argument("--center-seconds", type=float, default=0.7)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument(
        "--verify-roundtrip",
        action="store_true",
        help="Probe the R1DG diagnostic echo path before sending steering packets",
    )
    parser.add_argument("--diagnostic-host", default="127.0.0.1")
    parser.add_argument("--diagnostic-port", type=int, default=5010)
    parser.add_argument("--roundtrip-timeout-s", type=float, default=1.0)
    parser.add_argument(
        "--stop-hpvc-deployment",
        action="store_true",
        help="Stop HPVCDeployment.elf before sending this direct test command",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if args.hz <= 0.0:
        raise SystemExit("--hz must be positive")
    if abs(args.angle_rad) > 0.5:
        raise SystemExit("--angle-rad must be within 0.5 rad")

    direction_sign = 1.0 if args.direction == "left" else -1.0

    gate_distances, gate_ok, gate_source = read_gate_distances(args)
    if args.skip_sensor_check:
        print("sensor_check=SKIPPED")
    else:
        if not gate_distances:
            print("LCA blocked: no gate sensor data available")
            return 2
        for name in ("rear", "middle", "front"):
            distance, valid, source_key = gate_distances.get(name, (None, False, None))
            print(f"{args.direction}_{name} distance_m={distance} valid={valid} source={source_key}")
            if not valid:
                print(f"LCA blocked: {name} gate ultrasonic value is invalid")
                return 2
            if distance is None or distance < args.min_side_distance_m:
                value_text = "nan" if distance is None else f"{distance:.3f}"
                print(
                    f"LCA blocked: {name} gate gap is too small "
                    f"({value_text}m < {args.min_side_distance_m:.3f}m)"
                )
                return 3

    if args.verify_roundtrip:
        roundtrip_ok, elapsed_ms = probe_diagnostic_roundtrip(
            args=args,
            packet_valid=True,
            link_valid=gate_ok,
            control_valid=True,
            rpi2_flags=0x0F if gate_ok else 0x00,
            steering_rad=direction_sign * abs(args.angle_rad),
        )
        if not roundtrip_ok:
            print(
                f"UDP roundtrip failed: {args.diagnostic_host}:{args.diagnostic_port} "
                f"did not echo a valid R1DG packet within {args.roundtrip_timeout_s:.2f}s"
            )
            return 4
        print(f"UDP roundtrip ok: {elapsed_ms:.1f} ms")

    send_profile(args)
    print("LCA side-only test complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
