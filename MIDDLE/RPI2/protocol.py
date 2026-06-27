"""Binary UDP protocol shared by Raspberry Pi #2 and the HPVC."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import zlib


MAGIC = b"RP2L"
LEGACY_VERSION = 1
VERSION = 2
BASE_PAYLOAD_FLOAT_COUNT = 12
MAX_PERSON_DETECTIONS = 3
PERSON_DETECTION_FLOAT_COUNT = 6
PAYLOAD_FLOAT_COUNT = BASE_PAYLOAD_FLOAT_COUNT + 1 + (
    MAX_PERSON_DETECTIONS * PERSON_DETECTION_FLOAT_COUNT
)

FLAG_CAMERA_VALID = 1 << 0
FLAG_LANE_VALID = 1 << 1
FLAG_LEFT_ULTRASONIC_VALID = 1 << 2
FLAG_RIGHT_ULTRASONIC_VALID = 1 << 3
FLAG_PERSON_DETECTION_VALID = 1 << 4

# magic, version, flags, float count, sequence, frame timestamp, ultrasonic timestamp
HEADER_FORMAT = "<4sBBHIQQ"
LEGACY_PAYLOAD_FORMAT = f"<{BASE_PAYLOAD_FLOAT_COUNT}f"
PAYLOAD_FORMAT = f"<{PAYLOAD_FLOAT_COUNT}f"
CRC_FORMAT = "<I"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
LEGACY_PAYLOAD_SIZE = struct.calcsize(LEGACY_PAYLOAD_FORMAT)
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)
LEGACY_PACKET_SIZE = HEADER_SIZE + LEGACY_PAYLOAD_SIZE + struct.calcsize(CRC_FORMAT)
PACKET_SIZE = HEADER_SIZE + PAYLOAD_SIZE + struct.calcsize(CRC_FORMAT)

PAYLOAD_FIELDS = (
    "left_curvature_1pm",
    "left_curvature_derivative_1pm2",
    "left_heading_rad",
    "left_lateral_offset_m",
    "left_strength",
    "right_curvature_1pm",
    "right_curvature_derivative_1pm2",
    "right_heading_rad",
    "right_lateral_offset_m",
    "right_strength",
    "side_left_distance_m",
    "side_right_distance_m",
    "person_count",
    "person_0_valid",
    "person_0_confidence",
    "person_0_center_x_norm",
    "person_0_center_y_norm",
    "person_0_width_norm",
    "person_0_height_norm",
    "person_1_valid",
    "person_1_confidence",
    "person_1_center_x_norm",
    "person_1_center_y_norm",
    "person_1_width_norm",
    "person_1_height_norm",
    "person_2_valid",
    "person_2_confidence",
    "person_2_center_x_norm",
    "person_2_center_y_norm",
    "person_2_width_norm",
    "person_2_height_norm",
)


@dataclass(frozen=True)
class LaneBoundary:
    curvature_1pm: float = 0.0
    curvature_derivative_1pm2: float = 0.0
    heading_rad: float = 0.0
    lateral_offset_m: float = 0.0
    strength: float = 0.0

    def values(self) -> tuple[float, ...]:
        return (
            self.curvature_1pm,
            self.curvature_derivative_1pm2,
            self.heading_rad,
            self.lateral_offset_m,
            self.strength,
        )


@dataclass(frozen=True)
class PersonDetection:
    valid: float = 0.0
    confidence: float = 0.0
    center_x_norm: float = 0.0
    center_y_norm: float = 0.0
    width_norm: float = 0.0
    height_norm: float = 0.0

    def values(self) -> tuple[float, ...]:
        return (
            self.valid,
            self.confidence,
            self.center_x_norm,
            self.center_y_norm,
            self.width_norm,
            self.height_norm,
        )


@dataclass(frozen=True)
class PerceptionPacket:
    sequence: int
    frame_timestamp_us: int
    ultrasonic_timestamp_us: int
    flags: int
    left: LaneBoundary
    right: LaneBoundary
    side_left_distance_m: float
    side_right_distance_m: float
    person_count: int = 0
    persons: tuple[PersonDetection, ...] = ()


def pack_packet(packet: PerceptionPacket) -> bytes:
    payload = payload_values(packet)
    header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        VERSION,
        packet.flags & 0xFF,
        PAYLOAD_FLOAT_COUNT,
        packet.sequence & 0xFFFFFFFF,
        packet.frame_timestamp_us & 0xFFFFFFFFFFFFFFFF,
        packet.ultrasonic_timestamp_us & 0xFFFFFFFFFFFFFFFF,
    )
    body = header + struct.pack(PAYLOAD_FORMAT, *payload)
    return body + struct.pack(CRC_FORMAT, zlib.crc32(body) & 0xFFFFFFFF)


def payload_values(packet: PerceptionPacket) -> tuple[float, ...]:
    persons = tuple(packet.persons[:MAX_PERSON_DETECTIONS])
    detected_count = sum(1 for person in persons if person.valid >= 0.5)
    person_count = packet.person_count if packet.person_count else detected_count
    person_count = max(0, min(MAX_PERSON_DETECTIONS, int(person_count)))

    person_payload: list[float] = [float(person_count)]
    for index in range(MAX_PERSON_DETECTIONS):
        if index < len(persons):
            person_payload.extend(_sanitize_person(persons[index]).values())
        else:
            person_payload.extend(PersonDetection().values())

    return (
        *packet.left.values(),
        *packet.right.values(),
        packet.side_left_distance_m,
        packet.side_right_distance_m,
        *person_payload,
    )


def unpack_packet(data: bytes) -> PerceptionPacket:
    if len(data) not in (PACKET_SIZE, LEGACY_PACKET_SIZE):
        raise ValueError(
            f"Expected {PACKET_SIZE} bytes or legacy {LEGACY_PACKET_SIZE} bytes, "
            f"received {len(data)}"
        )

    body = data[:-4]
    received_crc = struct.unpack(CRC_FORMAT, data[-4:])[0]
    calculated_crc = zlib.crc32(body) & 0xFFFFFFFF
    if received_crc != calculated_crc:
        raise ValueError("CRC mismatch")

    magic, version, flags, count, sequence, frame_ts, ultrasonic_ts = struct.unpack(
        HEADER_FORMAT, body[:HEADER_SIZE]
    )
    if magic != MAGIC:
        raise ValueError(f"Unexpected magic: {magic!r}")

    if version == VERSION:
        payload_format = PAYLOAD_FORMAT
        expected_count = PAYLOAD_FLOAT_COUNT
        expected_size = PACKET_SIZE
    elif version == LEGACY_VERSION:
        payload_format = LEGACY_PAYLOAD_FORMAT
        expected_count = BASE_PAYLOAD_FLOAT_COUNT
        expected_size = LEGACY_PACKET_SIZE
    else:
        raise ValueError(f"Unsupported protocol version: {version}")

    if len(data) != expected_size:
        raise ValueError(f"Expected {expected_size} bytes for protocol v{version}, received {len(data)}")
    if count != expected_count:
        raise ValueError(f"Unexpected payload float count: {count}")

    values = struct.unpack(payload_format, body[HEADER_SIZE:])
    person_count = 0
    persons: tuple[PersonDetection, ...] = ()
    if version == VERSION:
        person_count = max(0, min(MAX_PERSON_DETECTIONS, int(values[BASE_PAYLOAD_FLOAT_COUNT])))
        persons = tuple(
            PersonDetection(
                *values[
                    BASE_PAYLOAD_FLOAT_COUNT
                    + 1
                    + (index * PERSON_DETECTION_FLOAT_COUNT) : BASE_PAYLOAD_FLOAT_COUNT
                    + 1
                    + ((index + 1) * PERSON_DETECTION_FLOAT_COUNT)
                ]
            )
            for index in range(MAX_PERSON_DETECTIONS)
        )

    return PerceptionPacket(
        sequence=sequence,
        frame_timestamp_us=frame_ts,
        ultrasonic_timestamp_us=ultrasonic_ts,
        flags=flags,
        left=LaneBoundary(*values[0:5]),
        right=LaneBoundary(*values[5:10]),
        side_left_distance_m=values[10],
        side_right_distance_m=values[11],
        person_count=person_count,
        persons=persons,
    )


def _sanitize_person(person: PersonDetection) -> PersonDetection:
    return PersonDetection(
        valid=1.0 if person.valid >= 0.5 else 0.0,
        confidence=_finite_clamp(person.confidence, 0.0, 1.0),
        center_x_norm=_finite_clamp(person.center_x_norm, 0.0, 1.0),
        center_y_norm=_finite_clamp(person.center_y_norm, 0.0, 1.0),
        width_norm=_finite_clamp(person.width_norm, 0.0, 1.0),
        height_norm=_finite_clamp(person.height_norm, 0.0, 1.0),
    )


def _finite_clamp(value: float, minimum: float, maximum: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        return minimum
    return max(minimum, min(maximum, value))
