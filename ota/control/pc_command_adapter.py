from control.command_types import PcControlCommand


def build_pc_control_command(vehicle_status: dict) -> PcControlCommand:
    return PcControlCommand(
        drive_command=vehicle_status.get("drive_command", "STOP"),
        steering_command=vehicle_status.get("steering_command", "STRAIGHT"),
        target_speed=float(vehicle_status.get("target_speed", 0.0)),
        turn_signal=vehicle_status.get("turn_signal", "OFF"),
        turn_request=int(vehicle_status.get("turn_request", 0)),
    )
