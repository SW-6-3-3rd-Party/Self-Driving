import struct


MSG_ID_CENTER_SENSOR_SUMMARY = 0x10
MSG_ID_FRONT_SENSOR_DATA_V1 = 0x20
MSG_ID_FRONT_SENSOR_DATA_V2 = 0x21
MSG_ID_REAR_STATUS_DATA_V1 = 0x30
MSG_ID_REAR_STATUS_DATA_V2 = 0x31
MSG_ID_HEARTBEAT = 0x50
MSG_ID_UPDATE_STATUS = 0x51


CENTER_SENSOR_SUMMARY_FORMAT = "<12fBBBBB"
FRONT_SENSOR_DATA_V1_FORMAT = "<fffBBB"
FRONT_SENSOR_DATA_V2_FORMAT = "<fffffBBBB"
REAR_STATUS_DATA_V1_FORMAT = "<ffBBB"
REAR_STATUS_DATA_V2_FORMAT = "<fffBBBBBB"


def _check_size(payload: bytes, fmt: str):
    expected = struct.calcsize(fmt)
    if len(payload) != expected:
        raise ValueError(f"payload size mismatch: expected={expected}, actual={len(payload)}")


def parse_center_sensor_summary(payload: bytes) -> dict:
    _check_size(payload, CENTER_SENSOR_SUMMARY_FORMAT)
    values = struct.unpack(CENTER_SENSOR_SUMMARY_FORMAT, payload)

    return {
        "type": "CenterSensorSummary",
        "lane_features": list(values[0:10]),
        "side_left_distance_m": values[10],
        "side_right_distance_m": values[11],
        "lane_valid": values[12],
        "side_ultrasonic_valid": values[13],
        "camera_fault": values[14],
        "sensor_fault": values[15],
        "alive_count": values[16],
    }


def parse_front_sensor_data_v1(payload: bytes) -> dict:
    _check_size(payload, FRONT_SENSOR_DATA_V1_FORMAT)
    values = struct.unpack(FRONT_SENSOR_DATA_V1_FORMAT, payload)

    return {
        "type": "FrontSensorData_v1",
        "front_left_diag_distance_m": values[0],
        "front_right_diag_distance_m": values[1],
        "tof_distance_m": values[2],
        "sensor_valid": values[3],
        "sensor_fault": values[4],
        "alive_count": values[5],
    }


def parse_front_sensor_data_v2(payload: bytes) -> dict:
    _check_size(payload, FRONT_SENSOR_DATA_V2_FORMAT)
    values = struct.unpack(FRONT_SENSOR_DATA_V2_FORMAT, payload)

    return {
        "type": "FrontSensorData_v2",
        "front_left_diag_distance_m": values[0],
        "front_right_diag_distance_m": values[1],
        "tof_distance_m": values[2],
        "filtered_tof_distance_m": values[3],
        "front_obstacle_distance_m": values[4],
        "distance_valid": values[5],
        "sensor_valid": values[6],
        "sensor_fault": values[7],
        "alive_count": values[8],
    }


def parse_rear_status_data_v1(payload: bytes) -> dict:
    _check_size(payload, REAR_STATUS_DATA_V1_FORMAT)
    values = struct.unpack(REAR_STATUS_DATA_V1_FORMAT, payload)

    return {
        "type": "RearStatusData_v1",
        "rear_left_diag_distance_m": values[0],
        "rear_right_diag_distance_m": values[1],
        "rear_sensor_valid": values[2],
        "motor_state": values[3],
        "alive_count": values[4],
    }


def parse_rear_status_data_v2(payload: bytes) -> dict:
    _check_size(payload, REAR_STATUS_DATA_V2_FORMAT)
    values = struct.unpack(REAR_STATUS_DATA_V2_FORMAT, payload)

    return {
        "type": "RearStatusData_v2",
        "rear_left_diag_distance_m": values[0],
        "rear_right_diag_distance_m": values[1],
        "vehicle_speed_mps": values[2],
        "motor_state": values[3],
        "emergency_stop_executed": values[4],
        "last_control_mode": values[5],
        "rear_sensor_valid": values[6],
        "motor_fault": values[7],
        "alive_count": values[8],
    }


def parse_payload(msg_id: int, payload: bytes) -> dict:
    if msg_id == MSG_ID_CENTER_SENSOR_SUMMARY:
        return parse_center_sensor_summary(payload)

    if msg_id == MSG_ID_FRONT_SENSOR_DATA_V1:
        return parse_front_sensor_data_v1(payload)

    if msg_id == MSG_ID_FRONT_SENSOR_DATA_V2:
        return parse_front_sensor_data_v2(payload)

    if msg_id == MSG_ID_REAR_STATUS_DATA_V1:
        return parse_rear_status_data_v1(payload)

    if msg_id == MSG_ID_REAR_STATUS_DATA_V2:
        return parse_rear_status_data_v2(payload)

    if msg_id == MSG_ID_HEARTBEAT:
        return {
            "type": "Heartbeat",
            "raw_payload_hex": payload.hex(),
        }

    if msg_id == MSG_ID_UPDATE_STATUS:
        return {
            "type": "UpdateStatus",
            "raw_payload_hex": payload.hex(),
        }

    return {
        "type": "Unknown",
        "msg_id": msg_id,
        "raw_payload_hex": payload.hex(),
    }
