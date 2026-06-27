"""HPVC-side AEB runtime.

Receives:
  - MIDDLE perception packets on UDP 5005 (MID2 v2)
  - Front TC375 sensor packets on UDP 5011 (AEB1)

Produces:
  - AEB brake request packets on an optional UDP destination
  - A concise terminal status line for bench validation
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import IntEnum
import math
import select
import socket
import struct
import time
import zlib

from MIDDLE.protocol import (
    FLAG_PERSON_DETECTION_VALID,
    PerceptionPacket,
    PersonDetection,
    unpack_packet,
)


FRONT_MAGIC = b"AEB1"
FRONT_AEB1_SIZE = 22
FRONT_AEB1_HEADER_FORMAT = "<4sBBHI"
FRONT_AEB1_TAIL_FORMAT = "<IHHH"
FRONT_LEGACY_V1_FORMAT = "<fffBBB"
FRONT_LEGACY_V2_FORMAT = "<fffffBBBB"

VALID_LEFT = 1 << 0
VALID_RIGHT = 1 << 1
VALID_TOF = 1 << 2

BRAKE_MAGIC = b"HPAB"
BRAKE_VERSION = 1
BRAKE_BODY_FORMAT = "<4sBBBBIQffHH"
BRAKE_PACKET_SIZE = struct.calcsize(BRAKE_BODY_FORMAT) + 4


class AebState(IntEnum):
    CLEAR = 0
    FCW = 1
    PARTIAL_BRAKE = 2
    FULL_BRAKE = 3
    STOP_HOLD = 4
    DEGRADED = 5
    SENSOR_FAULT = 6


class ReasonCode(IntEnum):
    CLEAR = 0
    FRONT_TOF_CLOSE = 1
    FRONT_ULTRASONIC_CLOSE = 2
    PERSON_IN_PATH = 3
    PERSON_WITH_FRONT_RANGE = 4
    NO_FRESH_INPUT = 5
    FRONT_STALE = 6
    MIDDLE_STALE = 7


@dataclass(frozen=True)
class FrontSensorFrame:
    sequence: int
    timestamp_ms: int
    arrival_time: float
    valid_mask: int
    tof_diag: int
    tof_front_m: float
    ultrasonic_left_m: float
    ultrasonic_right_m: float

    @property
    def tof_valid(self) -> bool:
        return bool(self.valid_mask & VALID_TOF) and _finite_positive(self.tof_front_m)

    @property
    def left_valid(self) -> bool:
        return bool(self.valid_mask & VALID_LEFT) and _finite_positive(self.ultrasonic_left_m)

    @property
    def right_valid(self) -> bool:
        return bool(self.valid_mask & VALID_RIGHT) and _finite_positive(self.ultrasonic_right_m)


@dataclass(frozen=True)
class MiddleFrame:
    sequence: int
    arrival_time: float
    flags: int
    side_left_m: float
    side_right_m: float
    persons: tuple[PersonDetection, ...]

    @property
    def person_detection_valid(self) -> bool:
        return bool(self.flags & FLAG_PERSON_DETECTION_VALID)


@dataclass(frozen=True)
class AebDecision:
    sequence: int
    state: AebState
    brake_percent: int
    reason: ReasonCode
    reason_text: str
    threat_distance_m: float
    confidence: float
    front_fresh: bool
    middle_fresh: bool
    person_in_path: bool


@dataclass(frozen=True)
class AebConfig:
    front_timeout_s: float = 0.50
    middle_timeout_s: float = 0.50
    loop_period_s: float = 0.05
    stop_hold_s: float = 0.60
    person_confidence_min: float = 0.35
    person_center_gate_min: float = 0.32
    person_center_gate_max: float = 0.68
    front_fcw_m: float = 0.50
    front_partial_m: float = 0.30
    front_full_m: float = 0.18
    person_range_gain: float = 1.25
    side_fcw_m: float = 0.30
    side_partial_m: float = 0.18
    side_full_m: float = 0.10


class HpvcAebController:
    def __init__(self, config: AebConfig):
        self.config = config
        self._sequence = 0
        self._hold_until = 0.0

    def decide(
        self,
        front: FrontSensorFrame | None,
        middle: MiddleFrame | None,
        now: float | None = None,
    ) -> AebDecision:
        if now is None:
            now = time.monotonic()

        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        front_fresh = front is not None and now - front.arrival_time <= self.config.front_timeout_s
        middle_fresh = middle is not None and now - middle.arrival_time <= self.config.middle_timeout_s
        person_in_path, best_person_conf = self._person_in_path(middle) if middle_fresh else (False, 0.0)

        if not front_fresh and not middle_fresh:
            return self._decision(
                AebState.SENSOR_FAULT,
                0,
                ReasonCode.NO_FRESH_INPUT,
                "no fresh front or middle input",
                math.nan,
                0.0,
                front_fresh,
                middle_fresh,
                person_in_path,
            )

        if not front_fresh and person_in_path:
            return self._decision(
                AebState.DEGRADED,
                0,
                ReasonCode.FRONT_STALE,
                "person in path but front range is stale",
                math.nan,
                best_person_conf,
                front_fresh,
                middle_fresh,
                person_in_path,
            )

        candidates: list[tuple[float, ReasonCode, str, float, bool]] = []
        if front_fresh and front is not None:
            if front.tof_valid:
                candidates.append(
                    (
                        front.tof_front_m,
                        ReasonCode.PERSON_WITH_FRONT_RANGE if person_in_path else ReasonCode.FRONT_TOF_CLOSE,
                        "person/front ToF" if person_in_path else "front ToF",
                        max(0.65, best_person_conf),
                        True,
                    )
                )
            side_values = []
            if front.left_valid:
                side_values.append(front.ultrasonic_left_m)
            if front.right_valid:
                side_values.append(front.ultrasonic_right_m)
            if side_values:
                candidates.append(
                    (
                        min(side_values),
                        ReasonCode.FRONT_ULTRASONIC_CLOSE,
                        "front side ultrasonic",
                        0.60,
                        False,
                    )
                )

        best_state = AebState.CLEAR
        best_brake = 0
        best_reason = ReasonCode.CLEAR
        best_text = "clear"
        best_distance = math.nan
        best_confidence = best_person_conf if person_in_path else 1.0

        for distance_m, reason, text, confidence, is_front_center in candidates:
            if not _finite_positive(distance_m):
                continue
            state, brake = self._range_state(distance_m, is_front_center, person_in_path)
            if brake > best_brake or state > best_state:
                best_state = state
                best_brake = brake
                best_reason = reason
                best_text = f"{text} {distance_m:.3f} m"
                best_distance = distance_m
                best_confidence = confidence

        if best_state == AebState.CLEAR and person_in_path:
            best_state = AebState.FCW
            best_reason = ReasonCode.PERSON_IN_PATH
            best_text = "person in path without close range"
            best_confidence = best_person_conf

        if best_state == AebState.FULL_BRAKE:
            self._hold_until = max(self._hold_until, now + self.config.stop_hold_s)
        elif now < self._hold_until:
            best_state = AebState.STOP_HOLD
            best_brake = max(best_brake, 100)
            best_text = "stop hold"

        return self._decision(
            best_state,
            best_brake,
            best_reason,
            best_text,
            best_distance,
            best_confidence,
            front_fresh,
            middle_fresh,
            person_in_path,
        )

    def _range_state(
        self,
        distance_m: float,
        is_front_center: bool,
        person_in_path: bool,
    ) -> tuple[AebState, int]:
        if is_front_center:
            gain = self.config.person_range_gain if person_in_path else 1.0
            full_m = self.config.front_full_m * gain
            partial_m = self.config.front_partial_m * gain
            fcw_m = self.config.front_fcw_m * gain
        else:
            full_m = self.config.side_full_m
            partial_m = self.config.side_partial_m
            fcw_m = self.config.side_fcw_m

        if distance_m <= full_m:
            return AebState.FULL_BRAKE, 100
        if distance_m <= partial_m:
            return AebState.PARTIAL_BRAKE, 65
        if distance_m <= fcw_m:
            return AebState.FCW, 0
        return AebState.CLEAR, 0

    def _person_in_path(self, middle: MiddleFrame | None) -> tuple[bool, float]:
        if middle is None or not middle.person_detection_valid:
            return False, 0.0
        best_confidence = 0.0
        for person in middle.persons:
            if person.valid < 0.5:
                continue
            if person.confidence < self.config.person_confidence_min:
                continue
            if not (
                self.config.person_center_gate_min
                <= person.center_x_norm
                <= self.config.person_center_gate_max
            ):
                continue
            best_confidence = max(best_confidence, float(person.confidence))
        return best_confidence > 0.0, best_confidence

    def _decision(
        self,
        state: AebState,
        brake_percent: int,
        reason: ReasonCode,
        reason_text: str,
        threat_distance_m: float,
        confidence: float,
        front_fresh: bool,
        middle_fresh: bool,
        person_in_path: bool,
    ) -> AebDecision:
        return AebDecision(
            sequence=self._sequence,
            state=state,
            brake_percent=max(0, min(100, int(brake_percent))),
            reason=reason,
            reason_text=reason_text,
            threat_distance_m=threat_distance_m,
            confidence=max(0.0, min(1.0, float(confidence))),
            front_fresh=front_fresh,
            middle_fresh=middle_fresh,
            person_in_path=person_in_path,
        )


def decode_front_packet(data: bytes, arrival_time: float | None = None) -> FrontSensorFrame:
    if arrival_time is None:
        arrival_time = time.monotonic()
    if len(data) >= 4 and data[:4] == FRONT_MAGIC:
        return _decode_front_aeb1(data, arrival_time)
    return _decode_front_legacy(data, arrival_time)


def decode_middle_packet(data: bytes, arrival_time: float | None = None) -> MiddleFrame:
    if arrival_time is None:
        arrival_time = time.monotonic()
    packet = unpack_packet(data)
    return _middle_from_perception(packet, arrival_time)


def encode_brake_command(decision: AebDecision, timestamp_us: int | None = None) -> bytes:
    if timestamp_us is None:
        timestamp_us = time.monotonic_ns() // 1000
    flags = 0
    flags |= 1 if decision.front_fresh else 0
    flags |= 2 if decision.middle_fresh else 0
    flags |= 4 if decision.person_in_path else 0
    distance = decision.threat_distance_m
    if not math.isfinite(distance):
        distance = -1.0
    body = struct.pack(
        BRAKE_BODY_FORMAT,
        BRAKE_MAGIC,
        BRAKE_VERSION,
        int(decision.state) & 0xFF,
        decision.brake_percent & 0xFF,
        flags & 0xFF,
        decision.sequence & 0xFFFFFFFF,
        timestamp_us & 0xFFFFFFFFFFFFFFFF,
        float(distance),
        float(decision.confidence),
        int(decision.reason) & 0xFFFF,
        0,
    )
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def decode_brake_command(data: bytes) -> dict[str, object]:
    if len(data) != BRAKE_PACKET_SIZE:
        raise ValueError(f"expected {BRAKE_PACKET_SIZE} bytes, received {len(data)}")
    body = data[:-4]
    expected_crc = struct.unpack_from("<I", data, len(body))[0]
    actual_crc = zlib.crc32(body) & 0xFFFFFFFF
    if expected_crc != actual_crc:
        raise ValueError("CRC mismatch")
    magic, version, state, brake, flags, sequence, timestamp_us, distance, confidence, reason, _ = struct.unpack(
        BRAKE_BODY_FORMAT, body
    )
    if magic != BRAKE_MAGIC or version != BRAKE_VERSION:
        raise ValueError("invalid AEB brake command magic or version")
    return {
        "state": AebState(state),
        "brake_percent": brake,
        "flags": flags,
        "sequence": sequence,
        "timestamp_us": timestamp_us,
        "threat_distance_m": None if distance < 0.0 else distance,
        "confidence": confidence,
        "reason": ReasonCode(reason),
    }


def _decode_front_aeb1(data: bytes, arrival_time: float) -> FrontSensorFrame:
    if len(data) < FRONT_AEB1_SIZE:
        raise ValueError(f"short AEB1 front packet: {len(data)} bytes")
    magic, version, valid_mask, tof_diag, sequence = struct.unpack_from(
        FRONT_AEB1_HEADER_FORMAT, data, 0
    )
    if magic != FRONT_MAGIC or version != 1:
        raise ValueError("invalid AEB1 front packet")
    timestamp_ms, tof_cm_x10, left_cm_x10, right_cm_x10 = struct.unpack_from(
        FRONT_AEB1_TAIL_FORMAT, data, 12
    )
    return FrontSensorFrame(
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        arrival_time=arrival_time,
        valid_mask=valid_mask,
        tof_diag=tof_diag,
        tof_front_m=_cm_x10_to_m(tof_cm_x10),
        ultrasonic_left_m=_cm_x10_to_m(left_cm_x10),
        ultrasonic_right_m=_cm_x10_to_m(right_cm_x10),
    )


def _decode_front_legacy(data: bytes, arrival_time: float) -> FrontSensorFrame:
    if len(data) >= struct.calcsize(FRONT_LEGACY_V2_FORMAT):
        (
            left_m,
            right_m,
            tof_m,
            _filtered_tof_m,
            _front_obstacle_m,
            _distance_valid,
            sensor_valid,
            sensor_fault,
            alive_count,
        ) = struct.unpack_from(FRONT_LEGACY_V2_FORMAT, data, 0)
        return FrontSensorFrame(
            sequence=alive_count,
            timestamp_ms=0,
            arrival_time=arrival_time,
            valid_mask=sensor_valid,
            tof_diag=sensor_fault,
            tof_front_m=tof_m,
            ultrasonic_left_m=left_m,
            ultrasonic_right_m=right_m,
        )
    if len(data) >= struct.calcsize(FRONT_LEGACY_V1_FORMAT):
        left_m, right_m, tof_m, sensor_valid, sensor_fault, alive_count = struct.unpack_from(
            FRONT_LEGACY_V1_FORMAT, data, 0
        )
        return FrontSensorFrame(
            sequence=alive_count,
            timestamp_ms=0,
            arrival_time=arrival_time,
            valid_mask=sensor_valid,
            tof_diag=sensor_fault,
            tof_front_m=tof_m,
            ultrasonic_left_m=left_m,
            ultrasonic_right_m=right_m,
        )
    raise ValueError(f"short front packet: {len(data)} bytes")


def _middle_from_perception(packet: PerceptionPacket, arrival_time: float) -> MiddleFrame:
    return MiddleFrame(
        sequence=packet.sequence,
        arrival_time=arrival_time,
        flags=packet.flags,
        side_left_m=packet.side_left_distance_m,
        side_right_m=packet.side_right_distance_m,
        persons=packet.persons,
    )


def _cm_x10_to_m(value: int) -> float:
    return float(value) / 1000.0


def _finite_positive(value: float) -> bool:
    return math.isfinite(float(value)) and float(value) > 0.0


def _bind_udp(host: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.setblocking(False)
    return sock


def _status_line(decision: AebDecision) -> str:
    if math.isfinite(decision.threat_distance_m):
        distance = f"{decision.threat_distance_m:.3f}m"
    else:
        distance = "--"
    return (
        f"AEB {decision.state.name:<13} brake={decision.brake_percent:3d}% "
        f"dist={distance:<7} conf={decision.confidence:.2f} "
        f"front={int(decision.front_fresh)} middle={int(decision.middle_fresh)} "
        f"person={int(decision.person_in_path)} reason={decision.reason_text}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--middle-host", default="0.0.0.0")
    parser.add_argument("--middle-port", type=int, default=5005)
    parser.add_argument("--front-host", default="0.0.0.0")
    parser.add_argument("--front-port", type=int, default=5011)
    parser.add_argument("--brake-host", default=None, help="Optional brake ECU/actuator UDP host")
    parser.add_argument("--brake-port", type=int, default=5013)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--front-timeout", type=float, default=0.50)
    parser.add_argument("--middle-timeout", type=float, default=0.50)
    parser.add_argument("--front-full-m", type=float, default=0.18)
    parser.add_argument("--front-partial-m", type=float, default=0.30)
    parser.add_argument("--front-fcw-m", type=float, default=0.50)
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    front_full_m = max(0.01, args.front_full_m)
    front_partial_m = max(front_full_m, args.front_partial_m)
    front_fcw_m = max(front_partial_m, args.front_fcw_m)
    config = AebConfig(
        loop_period_s=max(0.01, args.period),
        front_timeout_s=max(0.05, args.front_timeout),
        middle_timeout_s=max(0.05, args.middle_timeout),
        front_full_m=front_full_m,
        front_partial_m=front_partial_m,
        front_fcw_m=front_fcw_m,
    )
    controller = HpvcAebController(config)
    middle_sock = _bind_udp(args.middle_host, args.middle_port)
    front_sock = _bind_udp(args.front_host, args.front_port)
    brake_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    brake_destination = (args.brake_host, args.brake_port) if args.brake_host else None

    latest_front: FrontSensorFrame | None = None
    latest_middle: MiddleFrame | None = None
    last_print = 0.0

    print(f"HPVC AEB listening: middle UDP {args.middle_port}, front UDP {args.front_port}")
    if brake_destination:
        print(f"AEB brake requests -> {brake_destination[0]}:{brake_destination[1]}")
    else:
        print("AEB brake output is dry-run. Pass --brake-host to transmit requests.")

    while True:
        loop_started = time.monotonic()
        readable, _, _ = select.select([middle_sock, front_sock], [], [], config.loop_period_s)
        now = time.monotonic()
        for sock in readable:
            while True:
                try:
                    data, _addr = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                try:
                    if sock is middle_sock:
                        latest_middle = decode_middle_packet(data, now)
                    else:
                        latest_front = decode_front_packet(data, now)
                except ValueError as error:
                    if not args.quiet:
                        print(f"Rejected UDP packet: {error}", flush=True)

        decision = controller.decide(latest_front, latest_middle, now)
        if brake_destination:
            brake_sock.sendto(encode_brake_command(decision), brake_destination)

        if not args.quiet and now - last_print >= 0.25:
            print(_status_line(decision), flush=True)
            last_print = now

        elapsed = time.monotonic() - loop_started
        if elapsed < config.loop_period_s:
            time.sleep(config.loop_period_s - elapsed)


if __name__ == "__main__":
    main()
