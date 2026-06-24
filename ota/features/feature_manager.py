from control.command_types import FeatureControlRequest

from features.feature_input_builder import FeatureInputBuilder
from features.feature_adapters import (
    LKASAdapter,
    LCAAdapter,
    ACCAdapter,
    AEBAdapter,
)


class FeatureManager:
    def __init__(self):
        self.input_builder = FeatureInputBuilder()

        self.lkas_adapter = LKASAdapter()
        self.lca_adapter = LCAAdapter()
        self.acc_adapter = ACCAdapter()
        self.aeb_adapter = AEBAdapter()

        self.last_feature_input = {}
        self.last_feature_outputs = {}

    def build_feature_request(
        self,
        comm_status: dict,
        vehicle_status: dict,
    ) -> FeatureControlRequest:
        feature_input = self.input_builder.build(
            comm_status=comm_status,
            vehicle_status=vehicle_status,
        )

        lkas_output = self.lkas_adapter.run(
            feature_input,
            enabled=bool(vehicle_status.get("lkas_active", False)),
        )

        lca_output = self.lca_adapter.run(
            feature_input,
            enabled=bool(vehicle_status.get("lca_active", False)),
        )

        acc_output = self.acc_adapter.run(
            feature_input,
            enabled=bool(vehicle_status.get("acc_active", False)),
        )

        aeb_output = self.aeb_adapter.run(
            feature_input,
            enabled=bool(vehicle_status.get("aeb_active", False)),
        )

        self.last_feature_input = feature_input
        self.last_feature_outputs = {
            "lkas": lkas_output,
            "lca": lca_output,
            "acc": acc_output,
            "aeb": aeb_output,
        }

        return FeatureControlRequest(
            lkas_active=bool(lkas_output.get("active", False)),
            lkas_steering_angle_rad=float(
                lkas_output.get("steering_angle_rad", 0.0)
            ),
            lca_active=bool(lca_output.get("active", False)),
            lca_state=str(lca_output.get("state", "IDLE")),
            lca_lane_direction=int(lca_output.get("lane_direction", 0)),
            lca_steering_angle_rad=float(
                lca_output.get("steering_angle_rad", 0.0)
            ),
            acc_active=bool(acc_output.get("active", False)),
            acc_target_speed_mps=float(
                acc_output.get("target_speed_mps", 0.0)
            ),
            acceleration_request_mps2=float(
                acc_output.get("acceleration_request_mps2", 0.0)
            ),
            aeb_active=bool(aeb_output.get("active", False)),
            aeb_emergency_stop=bool(aeb_output.get("emergency_stop", False)),
        )

    def get_debug_status(self) -> dict:
        return {
            "feature_input": self.last_feature_input,
            "feature_outputs": self.last_feature_outputs,
        }
