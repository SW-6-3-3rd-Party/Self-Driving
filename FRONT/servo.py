#!/usr/bin/env python3
"""Keyboard servo command sender for the FRONT TC375 module.

Sends HPVC-style HPSC UDP steering packets to FRONT on UDP 5100.
Positive angle is left, negative angle is right.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time
import zlib
from dataclasses import dataclass

try:
    import msvcrt
except ImportError:  # pragma: no cover - Windows helper
    msvcrt = None

try:
    import select
except ImportError:  # pragma: no cover - Windows has msvcrt
    select = None


DEFAULT_FRONT_HOST = "192.168.10.11"
DEFAULT_FRONT_PORT = 5100
DEFAULT_SOURCE_PORT = 5101

HPSC_MAGIC = b"HPSC"
HPSC_VERSION = 1
HPSC_HEADER_SIZE = 32
HPSC_CONTROL_DISABLED = 0
HPSC_CONTROL_STEERING_ANGLE = 1
HPSC_FLAG_STEERING_VALID = 1 << 0
HPSC_FLAG_EMERGENCY_CENTER = 1 << 1
HPSC_FLAG_UPSTREAM_VALID = 1 << 2
HPSC_BODY_FORMAT = "<4sBBBBIQffHHI"
HPSC_PACKET_SIZE = 40

SERVO_LEFT_US = 1150
SERVO_CENTER_US = 1650
SERVO_RIGHT_US = 2000


@dataclass
class SteeringState:
    angle_rad: float = 0.0
    sequence: int = 0
    alive: int = 0
    armed: bool = True


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def angle_to_pulse_us(angle_rad: float, max_angle_rad: float) -> int:
    normalized = clamp(angle_rad / max_angle_rad, -1.0, 1.0)
    if normalized >= 0.0:
        span = SERVO_CENTER_US - SERVO_LEFT_US
        return round(SERVO_CENTER_US - normalized * span)

    span = SERVO_RIGHT_US - SERVO_CENTER_US
    return round(SERVO_CENTER_US + (-normalized) * span)


def pack_steering(
    sequence: int,
    timestamp_us: int,
    angle_rad: float,
    max_rate_rad_s: float,
    alive_count: int,
    *,
    steering_valid: bool,
    emergency_center: bool = False,
) -> bytes:
    flags = 0
    control_mode = HPSC_CONTROL_DISABLED

    if steering_valid:
        flags |= HPSC_FLAG_STEERING_VALID | HPSC_FLAG_UPSTREAM_VALID
        control_mode = HPSC_CONTROL_STEERING_ANGLE

    if emergency_center:
        flags |= HPSC_FLAG_EMERGENCY_CENTER
        control_mode = HPSC_CONTROL_DISABLED
        angle_rad = 0.0

    body = struct.pack(
        HPSC_BODY_FORMAT,
        HPSC_MAGIC,
        HPSC_VERSION,
        control_mode,
        flags,
        HPSC_HEADER_SIZE,
        sequence & 0xFFFFFFFF,
        timestamp_us & 0xFFFFFFFFFFFFFFFF,
        float(angle_rad),
        float(max_rate_rad_s),
        alive_count & 0xFFFF,
        0,
        0,
    )
    if len(body) != HPSC_PACKET_SIZE - 4:
        raise RuntimeError(f"HPSC body size mismatch: {len(body)} bytes")
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def open_socket(source_ip: str, source_port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((source_ip, source_port))
    return sock


def send_packet(
    sock: socket.socket,
    target: tuple[str, int],
    state: SteeringState,
    max_rate_rad_s: float,
    *,
    emergency_center: bool = False,
) -> None:
    angle = state.angle_rad if state.armed else 0.0
    packet = pack_steering(
        state.sequence,
        time.monotonic_ns() // 1000,
        angle,
        max_rate_rad_s,
        state.alive,
        steering_valid=state.armed and not emergency_center,
        emergency_center=emergency_center,
    )
    sock.sendto(packet, target)
    state.sequence = (state.sequence + 1) & 0xFFFFFFFF
    state.alive = (state.alive + 1) & 0xFFFF


def read_key() -> str | None:
    if msvcrt is not None:
        if not msvcrt.kbhit():
            return None
        key = msvcrt.getwch()
        if key in ("\x00", "\xe0"):
            second = msvcrt.getwch()
            return {
                "K": "left",
                "M": "right",
                "H": "up",
                "P": "down",
            }.get(second)
        return key.lower()

    if select is not None:
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if readable:
            return sys.stdin.read(1).lower()
    return None


def print_help(args: argparse.Namespace) -> None:
    print("FRONT servo keyboard test")
    print(f"Target FRONT : {args.front_host}:{args.front_port}")
    print(f"Source bind  : {args.source_ip or '0.0.0.0'}:{args.source_port}")
    print("Keys:")
    print("  a / left arrow  : step left")
    print("  d / right arrow : step right")
    print("  s / down arrow  : center")
    print("  1               : full left")
    print("  2               : center")
    print("  3               : full right")
    print("  space           : arm toggle")
    print("  h               : help")
    print("  q               : quit")
    print()


def print_status(state: SteeringState, max_angle_rad: float, prefix: str = "cmd") -> None:
    pulse_us = angle_to_pulse_us(state.angle_rad, max_angle_rad)
    print(
        f"{prefix:<6} angle={state.angle_rad:+.3f}rad "
        f"pulse~{pulse_us:4d}us armed={int(state.armed)} seq={state.sequence}",
        flush=True,
    )


def send_center_burst(
    sock: socket.socket,
    target: tuple[str, int],
    state: SteeringState,
    max_rate_rad_s: float,
    period_s: float,
) -> None:
    state.angle_rad = 0.0
    state.armed = True
    for _ in range(5):
        send_packet(sock, target, state, max_rate_rad_s)
        time.sleep(period_s)
    for _ in range(3):
        send_packet(sock, target, state, max_rate_rad_s, emergency_center=True)
        time.sleep(period_s)


def run(args: argparse.Namespace) -> int:
    if args.hz <= 0.0:
        raise SystemExit("--hz must be positive")
    if args.step_rad <= 0.0:
        raise SystemExit("--step-rad must be positive")
    if args.max_angle_rad <= 0.0:
        raise SystemExit("--max-angle-rad must be positive")
    if not math.isfinite(args.rate_rad_s) or args.rate_rad_s <= 0.0:
        raise SystemExit("--rate-rad-s must be positive")

    target = (args.front_host, args.front_port)
    state = SteeringState(
        sequence=int(time.monotonic() * 1000) & 0xFFFFFFFF,
        armed=not args.disarmed,
    )
    period_s = 1.0 / args.hz
    sock = open_socket(args.source_ip, args.source_port)

    print_help(args)
    print_status(state, args.max_angle_rad, "start")

    next_send = 0.0
    last_status = 0.0
    try:
        while True:
            now = time.monotonic()
            key = read_key()

            if key in ("q", "\x03"):
                break
            if key == "h":
                print_help(args)
            elif key == " ":
                state.armed = not state.armed
                print_status(state, args.max_angle_rad, "arm")
            elif key in ("a", "left"):
                state.angle_rad = clamp(
                    state.angle_rad + args.step_rad,
                    -args.max_angle_rad,
                    args.max_angle_rad,
                )
                print_status(state, args.max_angle_rad, "left")
            elif key in ("d", "right"):
                state.angle_rad = clamp(
                    state.angle_rad - args.step_rad,
                    -args.max_angle_rad,
                    args.max_angle_rad,
                )
                print_status(state, args.max_angle_rad, "right")
            elif key in ("s", "down", "2"):
                state.angle_rad = 0.0
                print_status(state, args.max_angle_rad, "center")
            elif key == "1":
                state.angle_rad = args.max_angle_rad
                print_status(state, args.max_angle_rad, "full L")
            elif key == "3":
                state.angle_rad = -args.max_angle_rad
                print_status(state, args.max_angle_rad, "full R")

            if now >= next_send:
                send_packet(sock, target, state, args.rate_rad_s)
                next_send = now + period_s

            if now - last_status >= args.status_every:
                print_status(state, args.max_angle_rad)
                last_status = now

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nInterrupted")
        return 130
    finally:
        print("Centering servo and sending emergency-center packets...")
        send_center_burst(sock, target, state, args.rate_rad_s, period_s)
        sock.close()

    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send keyboard servo commands to FRONT")
    parser.add_argument("--front-host", default=DEFAULT_FRONT_HOST)
    parser.add_argument("--front-port", type=int, default=DEFAULT_FRONT_PORT)
    parser.add_argument("--source-ip", default="", help="PC Ethernet IP, usually 192.168.10.1")
    parser.add_argument("--source-port", type=int, default=DEFAULT_SOURCE_PORT)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--step-rad", type=float, default=0.05)
    parser.add_argument("--max-angle-rad", type=float, default=0.50)
    parser.add_argument("--rate-rad-s", type=float, default=2.0)
    parser.add_argument("--status-every", type=float, default=1.0)
    parser.add_argument("--disarmed", action="store_true", help="Start with steering-valid output disabled")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(sys.argv[1:] if argv is None else argv))


if __name__ == "__main__":
    raise SystemExit(main())
