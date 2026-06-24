import struct
import time

from comm.crc16 import crc16_ccitt_false


MAGIC = 0x4850
PROTOCOL_VERSION = 1

DEVICE_ID_HPVC = 0x01

MSG_ID_CENTER_SENSOR_SUMMARY = 0x10
MSG_ID_FRONT_SENSOR_DATA_V1 = 0x20
MSG_ID_FRONT_SENSOR_DATA_V2 = 0x21
MSG_ID_REAR_STATUS_DATA_V1 = 0x30
MSG_ID_REAR_STATUS_DATA_V2 = 0x31
MSG_ID_FRONT_ZONE_COMMAND = 0x41
MSG_ID_REAR_DRIVE_CONTROL_V1 = 0x42
MSG_ID_REAR_DRIVE_CONTROL_V2 = 0x43
MSG_ID_HEARTBEAT = 0x50
MSG_ID_UPDATE_STATUS = 0x51

HEADER_FORMAT = "<HBBHHIBBH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

_START_TIME = time.monotonic()


def now_ms() -> int:
    return int((time.monotonic() - _START_TIME) * 1000) & 0xFFFFFFFF


def build_packet(
    msg_id: int,
    payload: bytes,
    seq: int,
    device_id: int = DEVICE_ID_HPVC,
    flags: int = 0,
) -> bytes:
    payload_len = len(payload)
    timestamp_ms = now_ms()

    header_with_zero_crc = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        PROTOCOL_VERSION,
        msg_id,
        payload_len,
        seq & 0xFFFF,
        timestamp_ms,
        device_id,
        flags,
        0,
    )

    crc = crc16_ccitt_false(header_with_zero_crc + payload)

    header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        PROTOCOL_VERSION,
        msg_id,
        payload_len,
        seq & 0xFFFF,
        timestamp_ms,
        device_id,
        flags,
        crc,
    )

    return header + payload


def parse_header(packet: bytes) -> dict:
    if len(packet) < HEADER_SIZE:
        raise ValueError("packet too short")

    fields = struct.unpack(HEADER_FORMAT, packet[:HEADER_SIZE])

    return {
        "magic": fields[0],
        "protocol_version": fields[1],
        "msg_id": fields[2],
        "payload_len": fields[3],
        "seq": fields[4],
        "timestamp_ms": fields[5],
        "device_id": fields[6],
        "flags": fields[7],
        "crc16": fields[8],
    }


def validate_packet(packet: bytes) -> tuple[bool, str]:
    if len(packet) < HEADER_SIZE:
        return False, "packet too short"

    header = parse_header(packet)

    if header["magic"] != MAGIC:
        return False, "invalid magic"

    if header["protocol_version"] != PROTOCOL_VERSION:
        return False, "invalid protocol version"

    expected_len = HEADER_SIZE + header["payload_len"]

    if len(packet) < expected_len:
        return False, "payload length mismatch"

    packet_for_crc = bytearray(packet[:expected_len])
    packet_for_crc[14] = 0
    packet_for_crc[15] = 0

    calculated_crc = crc16_ccitt_false(bytes(packet_for_crc))

    if calculated_crc != header["crc16"]:
        return False, (
            f"crc mismatch: expected={header['crc16']}, "
            f"calculated={calculated_crc}"
        )

    return True, "OK"


def extract_payload(packet: bytes) -> bytes:
    header = parse_header(packet)
    start = HEADER_SIZE
    end = HEADER_SIZE + header["payload_len"]
    return packet[start:end]
