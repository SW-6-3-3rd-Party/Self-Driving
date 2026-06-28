#!/usr/bin/env python3
"""PC Ethernet bench test tool for the FRONT module.

Default mode listens for FRONT AEB sensor packets on UDP 5011.
Use the steer or interactive modes to send HPVC-style HPSC steering commands
to the FRONT TC375 on UDP 5100.
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
import sys
import time
import zlib
from dataclasses import dataclass

try:
    import msvcrt
except ImportError:  # pragma: no cover - Windows-only helper
    msvcrt = None

try:
    import select
except ImportError:  # pragma: no cover - Windows has msvcrt
    select = None


DEFAULT_FRONT_HOST = "192.168.10.11"
DEFAULT_PC_BIND = "0.0.0.0"
SENSOR_PORT = 5011
STEERING_PORT = 5100
STEERING_SOURCE_PORT = 5101

AEB1_SIZE = 22
AEB1_HEADER_FORMAT = "<4sBBHI"
AEB1_TAIL_FORMAT = "<IHHH"
LEGACY_V1_FORMAT = "<fffBBB"
LEGACY_V2_FORMAT = "<fffffBBBB"

AEB_VALID_LEFT = 1 << 0
AEB_VALID_RIGHT = 1 << 1
AEB_VALID_TOF = 1 << 2
AEB_VALID_STEERING_RX = 1 << 5
AEB_VALID_STEERING_COMMAND = 1 << 6
AEB_VALID_STEERING_CENTER = 1 << 7

TOF_FAULT_NAMES = {
    0x00: "TOF_NOT_TRIED",
    0x03: "TOF_OK",
    0xA1: "TOF_MODEL_READ_FAIL",
    0xA2: "TOF_UNEXPECTED_MODEL",
    0xA3: "TOF_INIT_FAIL",
    0xA4: "TOF_CALIBRATION_FAIL",
    0xB1: "TOF_MEASURE_START_FAIL",
    0xB2: "TOF_MEASURE_STATUS_FAIL",
    0xB3: "TOF_MEASURE_TIMEOUT",
    0xB4: "TOF_RANGE_READ_FAIL",
    0xB5: "TOF_RANGE_OUT_OF_LIMIT",
}

ULTRA_DIAG_NAMES = {
    0xF001: "NO_ECHO_RISE",
    0xF002: "NO_ECHO_FALL",
    0xF003: "RANGE_OUT_OF_LIMIT",
}

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


@dataclass
class SensorState:
    last_sequence: int | None = None
    last_alive: int | None = None
    last_time: float | None = None
    received: int = 0
    rows_since_header: int = 0


@dataclass
class SteeringState:
    angle_rad: float = 0.0
    sequence: int = 0
    alive: int = 0
    armed: bool = False


def cm(value_m: float) -> float:
    return value_m * 100.0


def cm_x10_to_m(value: int) -> float:
    return float(value) / 1000.0


def valid_text(mask: int) -> str:
    return "/".join(
        (
            "LEFT" if mask & AEB_VALID_LEFT else "left--",
            "RIGHT" if mask & AEB_VALID_RIGHT else "right--",
            "TOF" if mask & AEB_VALID_TOF else "tof--",
        )
    )


def valid_compact(mask: int) -> str:
    return "".join(
        (
            "L" if mask & AEB_VALID_LEFT else "-",
            "R" if mask & AEB_VALID_RIGHT else "-",
            "T" if mask & AEB_VALID_TOF else "-",
            "|",
            "S" if mask & AEB_VALID_STEERING_RX else "-",
            "C" if mask & AEB_VALID_STEERING_COMMAND else "-",
            "E" if mask & AEB_VALID_STEERING_CENTER else "-",
        )
    )


def fault_text(fault: int) -> str:
    return TOF_FAULT_NAMES.get(fault, f"FAULT_0x{fault:02X}")


def distance_text(packet: dict, key: str, valid_bit: int) -> str:
    value = packet.get(key, math.nan)
    if (packet["valid"] & valid_bit) == 0:
        raw_key = {
            "left_m": "left_raw_cm_x10",
            "right_m": "right_raw_cm_x10",
            "tof_m": "tof_raw_cm_x10",
        }.get(key)
        if raw_key is not None and packet.get(raw_key, 0) in ULTRA_DIAG_NAMES:
            return "  diag"
        if not math.isfinite(value) or abs(value) < 1.0e-6:
            return "    --"
        return f"!{cm(value):5.1f}"
    if not math.isfinite(value):
        return "    NaN"
    return f"{cm(value):6.1f}"


def packet_note(packet: dict) -> str:
    notes: list[str] = []
    if packet["version"].startswith("legacy"):
        notes.append("LEGACY_FW")
    missing = []
    if (packet["valid"] & AEB_VALID_LEFT) == 0:
        missing.append("L")
    if (packet["valid"] & AEB_VALID_RIGHT) == 0:
        missing.append("R")
    if (packet["valid"] & AEB_VALID_TOF) == 0:
        missing.append("ToF")
    if missing:
        notes.append("INVALID_" + ",".join(missing))
    if packet["valid"] & AEB_VALID_STEERING_COMMAND:
        notes.append("STEER_CMD")
    elif packet["valid"] & AEB_VALID_STEERING_RX:
        notes.append("STEER_RX_CENTER")
    elif packet["valid"] & AEB_VALID_STEERING_CENTER:
        notes.append("STEER_WATCHDOG_CENTER")
    for side, raw_key, valid_bit in (
        ("LEFT", "left_raw_cm_x10", AEB_VALID_LEFT),
        ("RIGHT", "right_raw_cm_x10", AEB_VALID_RIGHT),
    ):
        raw = packet.get(raw_key, 0)
        if (packet["valid"] & valid_bit) == 0 and raw in ULTRA_DIAG_NAMES:
            notes.append(f"{side}_{ULTRA_DIAG_NAMES[raw]}")
    if packet.get("fault", 0) not in (0, 0x03):
        notes.append(fault_text(packet["fault"]))
    return " ".join(notes) if notes else "OK"


def print_sensor_header() -> None:
    print(
        "time     source              fmt        id          dt_ms  valid "
        " L_cm   R_cm  ToF_cm  fault   diag   note"
    )
    print(
        "-------- ------------------- ---------- ---------- ------- ----- "
        "------ ------ ------- ------- ------ -------------------------"
    )
    print("        distance prefix ! means raw value exists but the valid bit is OFF")
    print("        valid bits are L/R/T sensors and S/C/E steering rx/command/center")


def decode_aeb1(data: bytes) -> dict:
    if len(data) != AEB1_SIZE:
        raise ValueError(f"AEB1 length must be {AEB1_SIZE}, got {len(data)}")

    magic, version, valid_mask, tof_diag, sequence = struct.unpack_from(
        AEB1_HEADER_FORMAT, data, 0
    )
    if magic != b"AEB1" or version != 1:
        raise ValueError("not an AEB1 v1 packet")

    timestamp_ms, tof_cm_x10, left_cm_x10, right_cm_x10 = struct.unpack_from(
        AEB1_TAIL_FORMAT, data, 12
    )
    return {
        "version": "AEB1",
        "valid": valid_mask,
        "fault": tof_diag & 0xFF,
        "diag": tof_diag,
        "model_id": (tof_diag >> 8) & 0xFF,
        "sequence": sequence,
        "timestamp_ms": timestamp_ms,
        "left_raw_cm_x10": left_cm_x10,
        "right_raw_cm_x10": right_cm_x10,
        "tof_raw_cm_x10": tof_cm_x10,
        "left_m": cm_x10_to_m(left_cm_x10),
        "right_m": cm_x10_to_m(right_cm_x10),
        "tof_m": cm_x10_to_m(tof_cm_x10),
        "filtered_tof_m": cm_x10_to_m(tof_cm_x10),
        "front_obstacle_m": cm_x10_to_m(tof_cm_x10),
    }


def decode_legacy(data: bytes) -> dict:
    if len(data) >= struct.calcsize(LEGACY_V2_FORMAT):
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
        ) = struct.unpack(LEGACY_V2_FORMAT, data[: struct.calcsize(LEGACY_V2_FORMAT)])
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

    if len(data) >= struct.calcsize(LEGACY_V1_FORMAT):
        left_m, right_m, tof_m, sensor_valid, sensor_fault, alive_count = struct.unpack(
            LEGACY_V1_FORMAT, data[: struct.calcsize(LEGACY_V1_FORMAT)]
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


def decode_sensor_packet(data: bytes) -> dict:
    if len(data) >= 4 and data[:4] == b"AEB1":
        return decode_aeb1(data)
    return decode_legacy(data)


def open_sensor_socket(bind_ip: str, port: int, timeout_s: float | None) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_ip, port))
    if timeout_s is None:
        sock.setblocking(False)
    else:
        sock.settimeout(timeout_s)
    return sock


def open_steering_socket(source_ip: str, source_port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((source_ip, source_port))
    return sock


def update_sequence_note(packet: dict, state: SensorState, now: float) -> tuple[str, float]:
    dt_ms = 0.0 if state.last_time is None else (now - state.last_time) * 1000.0
    state.last_time = now
    state.received += 1

    if "sequence" in packet:
        sequence = int(packet["sequence"])
        if state.last_sequence is None:
            note = f"seq={sequence}"
        else:
            expected = (state.last_sequence + 1) & 0xFFFFFFFF
            gap = (sequence - expected) & 0xFFFFFFFF
            note = f"seq={sequence}" if gap == 0 else f"seq={sequence} GAP+{gap}"
        state.last_sequence = sequence
        return note, dt_ms

    alive = packet.get("alive")
    if alive is not None:
        alive = int(alive)
        if state.last_alive is None:
            note = f"alive={alive:02d}"
        else:
            expected_alive = (state.last_alive + 1) & 0x0F
            note = f"alive={alive:02d}" if alive == expected_alive else f"alive={alive:02d} GAP"
        state.last_alive = alive
        return note, dt_ms

    return "seq=-", dt_ms


def print_sensor_packet(packet: dict, addr: tuple[str, int], state: SensorState, now: float) -> None:
    note, dt_ms = update_sequence_note(packet, state, now)
    if state.rows_since_header == 0:
        print_sensor_header()
    state.rows_since_header = (state.rows_since_header + 1) % 20

    print(
        f"{time.strftime('%H:%M:%S', time.localtime(now))} "
        f"{addr[0]}:{addr[1]:<5} "
        f"{packet['version']:<10} "
        f"{note:<10} "
        f"{dt_ms:7.1f} "
        f"{valid_compact(packet['valid']):>5} "
        f"{distance_text(packet, 'left_m', AEB_VALID_LEFT)} "
        f"{distance_text(packet, 'right_m', AEB_VALID_RIGHT)} "
        f"{distance_text(packet, 'tof_m', AEB_VALID_TOF)} "
        f"0x{packet['fault']:02X} "
        f"0x{packet.get('diag', 0):04X} "
        f"{packet_note(packet)}",
        flush=True,
    )


def csv_row(packet: dict, addr: tuple[str, int], now: float) -> dict:
    return {
        "pc_time_s": f"{now:.6f}",
        "source_ip": addr[0],
        "source_port": addr[1],
        "version": packet["version"],
        "sequence": packet.get("sequence", ""),
        "timestamp_ms": packet.get("timestamp_ms", ""),
        "valid_mask": packet["valid"],
        "tof_diag": packet.get("diag", ""),
        "left_m": packet["left_m"],
        "right_m": packet["right_m"],
        "tof_m": packet["tof_m"],
    }


def run_sensor(args: argparse.Namespace) -> int:
    state = SensorState()
    sock = open_sensor_socket(args.bind_ip, args.sensor_port, args.timeout)
    csv_file = None
    writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(
            csv_file,
            fieldnames=(
                "pc_time_s",
                "source_ip",
                "source_port",
                "version",
                "sequence",
                "timestamp_ms",
                "valid_mask",
                "tof_diag",
                "left_m",
                "right_m",
                "tof_m",
            ),
        )
        writer.writeheader()

    print(f"Listening FRONT sensors on UDP {args.bind_ip}:{args.sensor_port}")
    print("PC Ethernet should be on 192.168.10.1/24, FRONT TC375 on 192.168.10.11")
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                print("waiting... no FRONT sensor UDP packet")
                continue

            now = time.time()
            try:
                packet = decode_sensor_packet(data)
            except ValueError as error:
                print(f"rejected from {addr}: {error}; {len(data)} bytes {data.hex()}")
                continue

            print_sensor_packet(packet, addr, state, now)
            if writer is not None:
                writer.writerow(csv_row(packet, addr, now))
                csv_file.flush()
    except KeyboardInterrupt:
        print("\nStopped sensor monitor")
        return 0
    finally:
        sock.close()
        if csv_file is not None:
            csv_file.close()


def clamp_angle(angle_rad: float, max_abs_rad: float) -> float:
    if not math.isfinite(angle_rad):
        raise ValueError("angle must be finite")
    return max(-max_abs_rad, min(max_abs_rad, angle_rad))


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
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def send_steering_packet(
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


def run_steer(args: argparse.Namespace) -> int:
    if abs(args.angle_rad) > args.max_angle_rad:
        raise SystemExit(f"--angle-rad must be within +/-{args.max_angle_rad}")
    if args.hz <= 0:
        raise SystemExit("--hz must be positive")
    if not args.arm and abs(args.angle_rad) > 1.0e-6:
        raise SystemExit("Refusing to move servo without --arm")

    target = (args.front_host, args.front_port)
    state = SteeringState(
        angle_rad=clamp_angle(args.angle_rad, args.max_angle_rad),
        sequence=int(time.monotonic() * 1000) & 0xFFFFFFFF,
        armed=args.arm,
    )
    sock = open_steering_socket(args.source_ip, args.source_port)
    period_s = 1.0 / args.hz
    print(
        f"Sending HPSC to {target[0]}:{target[1]} from {args.source_ip or '0.0.0.0'}:{args.source_port} "
        f"angle={state.angle_rad:.3f}rad armed={int(state.armed)}"
    )
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            send_steering_packet(sock, target, state, args.rate_rad_s)
            print(f"STEER angle_rad={state.angle_rad:+.3f} seq={state.sequence}", flush=True)
            time.sleep(period_s)

        state.angle_rad = 0.0
        center_end = time.monotonic() + args.center_duration
        while time.monotonic() < center_end:
            send_steering_packet(sock, target, state, args.rate_rad_s)
            print(f"CENTER angle_rad=+0.000 seq={state.sequence}", flush=True)
            time.sleep(period_s)

        send_steering_packet(sock, target, state, args.rate_rad_s, emergency_center=True)
        print("Emergency-center packet sent")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted, sending emergency-center packets")
        for _ in range(5):
            send_steering_packet(sock, target, state, args.rate_rad_s, emergency_center=True)
            time.sleep(period_s)
        return 130
    finally:
        sock.close()


def read_key_nonblocking() -> str | None:
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
            }.get(second, None)
        return key.lower()

    if select is not None:
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if readable:
            return sys.stdin.read(1).lower()
    return None


def drain_sensor_socket(sock: socket.socket, state: SensorState, print_every_s: float) -> None:
    printed = getattr(drain_sensor_socket, "_last_print", 0.0)
    while True:
        try:
            data, addr = sock.recvfrom(2048)
        except BlockingIOError:
            break
        except OSError:
            break

        now = time.time()
        try:
            packet = decode_sensor_packet(data)
        except ValueError as error:
            if now - printed >= print_every_s:
                print(f"sensor reject from {addr}: {error}")
                setattr(drain_sensor_socket, "_last_print", now)
                printed = now
            continue

        if now - printed >= print_every_s:
            print_sensor_packet(packet, addr, state, now)
            setattr(drain_sensor_socket, "_last_print", now)
            printed = now


def run_interactive(args: argparse.Namespace) -> int:
    if args.hz <= 0:
        raise SystemExit("--hz must be positive")
    if args.angle_step_rad <= 0:
        raise SystemExit("--angle-step-rad must be positive")

    target = (args.front_host, args.front_port)
    steer_sock = open_steering_socket(args.source_ip, args.source_port)
    sensor_sock = None
    sensor_state = SensorState()
    if not args.no_sensor:
        sensor_sock = open_sensor_socket(args.bind_ip, args.sensor_port, None)

    steer_state = SteeringState(
        sequence=int(time.monotonic() * 1000) & 0xFFFFFFFF,
        armed=args.arm,
    )
    period_s = 1.0 / args.hz
    next_send = 0.0
    last_status = 0.0

    print("Interactive FRONT test")
    print("Keys: a/left = left, d/right = right, s = center, space = arm toggle, q = quit")
    print(f"Target FRONT: {target[0]}:{target[1]} | armed={int(steer_state.armed)}")
    try:
        while True:
            now = time.monotonic()
            if sensor_sock is not None:
                drain_sensor_socket(sensor_sock, sensor_state, args.sensor_print_every)

            key = read_key_nonblocking()
            if key in ("q", "\x03"):
                break
            if key in (" ",):
                steer_state.armed = not steer_state.armed
                print(f"armed={int(steer_state.armed)}")
            elif key in ("a", "left"):
                steer_state.angle_rad = clamp_angle(
                    steer_state.angle_rad + args.angle_step_rad, args.max_angle_rad
                )
                print(f"angle_rad={steer_state.angle_rad:+.3f}")
            elif key in ("d", "right"):
                steer_state.angle_rad = clamp_angle(
                    steer_state.angle_rad - args.angle_step_rad, args.max_angle_rad
                )
                print(f"angle_rad={steer_state.angle_rad:+.3f}")
            elif key in ("s", "down"):
                steer_state.angle_rad = 0.0
                print("angle_rad=+0.000")

            if now >= next_send:
                send_steering_packet(steer_sock, target, steer_state, args.rate_rad_s)
                next_send = now + period_s

            if now - last_status >= 1.0:
                print(
                    f"cmd angle={steer_state.angle_rad:+.3f}rad "
                    f"armed={int(steer_state.armed)} sent_seq={steer_state.sequence}"
                )
                last_status = now

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        print("Sending emergency-center packets before exit")
        steer_state.angle_rad = 0.0
        for _ in range(5):
            send_steering_packet(
                steer_sock,
                target,
                steer_state,
                args.rate_rad_s,
                emergency_center=True,
            )
            time.sleep(period_s)
        steer_sock.close()
        if sensor_sock is not None:
            sensor_sock.close()
    return 0


def add_network_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bind-ip", default=DEFAULT_PC_BIND, help="PC IP for sensor receive bind")
    parser.add_argument("--sensor-port", type=int, default=SENSOR_PORT)


def add_steering_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--front-host", default=DEFAULT_FRONT_HOST)
    parser.add_argument("--front-port", type=int, default=STEERING_PORT)
    parser.add_argument("--source-ip", default="", help="PC Ethernet IP, usually 192.168.10.1")
    parser.add_argument("--source-port", type=int, default=STEERING_SOURCE_PORT)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--rate-rad-s", type=float, default=1.0)
    parser.add_argument("--max-angle-rad", type=float, default=0.50)
    parser.add_argument("--arm", action="store_true", help="Actually send steering-valid motion commands")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="FRONT Ethernet sensor/servo bench test")
    subparsers = parser.add_subparsers(dest="command")

    sensor = subparsers.add_parser("sensor", help="Listen for FRONT AEB1 sensor packets")
    add_network_args(sensor)
    sensor.add_argument("--timeout", type=float, default=2.0)
    sensor.add_argument("--csv", default="", help="Optional CSV output path")
    sensor.set_defaults(func=run_sensor)

    steer = subparsers.add_parser("steer", help="Send one HPSC steering profile")
    add_steering_args(steer)
    steer.add_argument("--angle-rad", type=float, default=0.0)
    steer.add_argument("--duration", type=float, default=1.0)
    steer.add_argument("--center-duration", type=float, default=0.7)
    steer.set_defaults(func=run_steer)

    interactive = subparsers.add_parser("interactive", help="Monitor sensors and steer with keyboard")
    add_network_args(interactive)
    add_steering_args(interactive)
    interactive.add_argument("--angle-step-rad", type=float, default=0.03)
    interactive.add_argument("--sensor-print-every", type=float, default=0.25)
    interactive.add_argument("--no-sensor", action="store_true")
    interactive.set_defaults(func=run_interactive)

    args = parser.parse_args(argv)
    if args.command is None:
        args.command = "sensor"
        args.bind_ip = DEFAULT_PC_BIND
        args.sensor_port = SENSOR_PORT
        args.timeout = 2.0
        args.csv = ""
        args.func = run_sensor
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
