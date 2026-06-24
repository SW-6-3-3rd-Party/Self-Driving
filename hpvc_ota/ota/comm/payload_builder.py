import struct

from control.command_types import (
    FrontZoneCommand,
    RearDriveControlV1,
    RearDriveControlV2,
)


# FrontZoneCommand payload
# steering_angle_rad float32
# steering_valid uint8
# emergency_center uint8
# turn_signal uint8
# command_valid uint8
# alive_count uint8
FRONT_ZONE_COMMAND_FORMAT = "<fBBBBB"


# RearDriveControl_v1 payload
# drive_direction uint8
# target_speed_mps float32
# normal_stop uint8
# turn_signal uint8
# command_valid uint8
# alive_count uint8
REAR_DRIVE_CONTROL_V1_FORMAT = "<BfBBBB"


# RearDriveControl_v2 payload
# control_mode uint8
# drive_direction uint8
# target_speed_mps float32
# acceleration_request_mps2 float32
# emergency_stop uint8
# turn_signal uint8
# command_valid uint8
# alive_count uint8
REAR_DRIVE_CONTROL_V2_FORMAT = "<BBffBBBB"


def build_front_zone_payload(command: FrontZoneCommand) -> bytes:
    return struct.pack(
        FRONT_ZONE_COMMAND_FORMAT,
        float(command.steering_angle_rad),
        int(command.steering_valid),
        int(command.emergency_center),
        int(command.turn_signal),
        int(command.command_valid),
        int(command.alive_count),
    )


def build_rear_drive_control_v1_payload(command: RearDriveControlV1) -> bytes:
    return struct.pack(
        REAR_DRIVE_CONTROL_V1_FORMAT,
        int(command.drive_direction),
        float(command.target_speed_mps),
        int(command.normal_stop),
        int(command.turn_signal),
        int(command.command_valid),
        int(command.alive_count),
    )


def build_rear_drive_control_v2_payload(command: RearDriveControlV2) -> bytes:
    return struct.pack(
        REAR_DRIVE_CONTROL_V2_FORMAT,
        int(command.control_mode),
        int(command.drive_direction),
        float(command.target_speed_mps),
        float(command.acceleration_request_mps2),
        int(command.emergency_stop),
        int(command.turn_signal),
        int(command.command_valid),
        int(command.alive_count),
    )
