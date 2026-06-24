from dataclasses import dataclass


@dataclass
class PcControlCommand:
    drive_command: str = "STOP"
    steering_command: str = "STRAIGHT"
    target_speed: float = 0.0
    turn_signal: str = "OFF"
    turn_request: int = 0


@dataclass
class FeatureControlRequest:
    # Lateral control
    lkas_active: bool = False
    lkas_steering_angle_rad: float = 0.0

    lca_active: bool = False
    lca_state: str = "IDLE"
    lca_lane_direction: int = 0
    lca_steering_angle_rad: float = 0.0

    # Longitudinal control
    acc_active: bool = False
    acc_target_speed_mps: float = 0.0
    acceleration_request_mps2: float = 0.0

    aeb_active: bool = False
    aeb_emergency_stop: bool = False


@dataclass
class FinalVehicleCommand:
    control_mode: str = "MANUAL"

    final_drive_command: str = "STOP"
    final_steering_command: str = "STRAIGHT"
    final_steering_angle_rad: float = 0.0
    final_target_speed: float = 0.0
    acceleration_request_mps2: float = 0.0

    emergency_stop: bool = False

    turn_signal: str = "OFF"
    turn_request: int = 0

    command_valid: bool = True
    reason: str = "OK"


@dataclass
class FrontZoneCommand:
    # HPVC -> FrontZoneCommand, Msg ID 0x41
    steering_angle_rad: float = 0.0
    steering_valid: int = 0
    emergency_center: int = 1
    turn_signal: int = 0
    command_valid: int = 0
    alive_count: int = 0


@dataclass
class RearDriveControlV1:
    # HPVC -> RearDriveControl_v1, Msg ID 0x42
    drive_direction: int = 0
    target_speed_mps: float = 0.0
    normal_stop: int = 1
    turn_signal: int = 0
    command_valid: int = 0
    alive_count: int = 0


@dataclass
class RearDriveControlV2:
    # HPVC -> RearDriveControl_v2, Msg ID 0x43
    control_mode: int = 0
    drive_direction: int = 0
    target_speed_mps: float = 0.0
    acceleration_request_mps2: float = 0.0
    emergency_stop: int = 0
    turn_signal: int = 0
    command_valid: int = 0
    alive_count: int = 0
