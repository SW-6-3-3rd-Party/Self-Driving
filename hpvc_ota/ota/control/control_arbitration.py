from control.command_types import (
    PcControlCommand,
    FeatureControlRequest,
    FinalVehicleCommand,
)


STEERING_ANGLE_RAD = {
    "LEFT": 0.35,
    "STRAIGHT": 0.0,
    "RIGHT": -0.35,
}


class ControlArbitrator:
    def arbitrate(
        self,
        pc_command: PcControlCommand,
        feature_request: FeatureControlRequest,
    ) -> FinalVehicleCommand:

        # 1. Lateral control priority
        # LCA > LKAS > PC manual steering
        if feature_request.lca_active:
            final_steering_angle_rad = feature_request.lca_steering_angle_rad
            final_steering_command = "LCA"
            lateral_reason = "LCA"

        elif feature_request.lkas_active:
            final_steering_angle_rad = feature_request.lkas_steering_angle_rad
            final_steering_command = "LKAS"
            lateral_reason = "LKAS"

        else:
            final_steering_angle_rad = STEERING_ANGLE_RAD.get(
                pc_command.steering_command,
                0.0,
            )
            final_steering_command = pc_command.steering_command
            lateral_reason = "PC_MANUAL"

        # 2. Longitudinal control priority
        # AEB > ACC > PC manual speed
        if feature_request.aeb_active or feature_request.aeb_emergency_stop:
            control_mode = "AEB"
            final_drive_command = "STOP"
            final_target_speed = 0.0
            acceleration_request_mps2 = 0.0
            emergency_stop = True
            longitudinal_reason = "AEB"

        elif feature_request.acc_active:
            control_mode = "ACC"
            final_drive_command = "FORWARD"
            final_target_speed = feature_request.acc_target_speed_mps
            acceleration_request_mps2 = feature_request.acceleration_request_mps2
            emergency_stop = False
            longitudinal_reason = "ACC"

        else:
            control_mode = "MANUAL"
            final_drive_command = pc_command.drive_command
            final_target_speed = pc_command.target_speed
            acceleration_request_mps2 = 0.0
            emergency_stop = False
            longitudinal_reason = "PC_MANUAL"

        final_command = FinalVehicleCommand(
            control_mode=control_mode,
            final_drive_command=final_drive_command,
            final_steering_command=final_steering_command,
            final_steering_angle_rad=final_steering_angle_rad,
            final_target_speed=final_target_speed,
            acceleration_request_mps2=acceleration_request_mps2,
            emergency_stop=emergency_stop,
            turn_signal=pc_command.turn_signal,
            turn_request=pc_command.turn_request,
            command_valid=True,
            reason=f"lat={lateral_reason}, lon={longitudinal_reason}",
        )

        return final_command
