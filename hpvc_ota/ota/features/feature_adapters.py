def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class LKASAdapter:
    def run(self, feature_input: dict, enabled: bool) -> dict:
        if not enabled:
            return {
                "active": False,
                "steering_angle_rad": 0.0,
            }

        if not feature_input.get("lane_valid", False):
            return {
                "active": False,
                "steering_angle_rad": 0.0,
            }

        lane_features = feature_input.get("lane_features", [0.0] * 10)

        left_offset = float(lane_features[3]) if len(lane_features) > 3 else 0.0
        right_offset = float(lane_features[8]) if len(lane_features) > 8 else 0.0

        center_error = (left_offset + right_offset) / 2.0
        steering_angle_rad = clamp(-0.8 * center_error, -0.35, 0.35)

        return {
            "active": True,
            "steering_angle_rad": steering_angle_rad,
        }


class LCAAdapter:
    def run(self, feature_input: dict, enabled: bool) -> dict:
        turn_request = int(feature_input.get("turn_request", 0))

        if not enabled or turn_request == 0:
            return {
                "active": False,
                "state": "IDLE",
                "lane_direction": 0,
                "steering_angle_rad": 0.0,
            }

        if not feature_input.get("lca_data_valid", False):
            return {
                "active": False,
                "state": "BLOCKED_OR_INVALID",
                "lane_direction": turn_request,
                "steering_angle_rad": 0.0,
            }

        if turn_request == 1:
            steering_angle_rad = 0.25
        elif turn_request == 2:
            steering_angle_rad = -0.25
        else:
            steering_angle_rad = 0.0

        return {
            "active": True,
            "state": "ACTIVE",
            "lane_direction": turn_request,
            "steering_angle_rad": steering_angle_rad,
        }


class ACCAdapter:
    def run(self, feature_input: dict, enabled: bool) -> dict:
        if not enabled:
            return {
                "active": False,
                "target_speed_mps": 0.0,
                "acceleration_request_mps2": 0.0,
            }

        if not feature_input.get("distance_valid", False):
            return {
                "active": False,
                "target_speed_mps": 0.0,
                "acceleration_request_mps2": 0.0,
            }

        distance = float(feature_input.get("front_obstacle_distance_m", 99.0))
        current_speed = float(feature_input.get("vehicle_speed_mps", 0.0))
        pc_target_speed = float(feature_input.get("pc_target_speed_mps", 0.0))

        if distance < 0.8:
            target_speed = min(pc_target_speed, 0.15)
        elif distance < 1.2:
            target_speed = min(pc_target_speed, 0.25)
        else:
            target_speed = pc_target_speed

        acceleration_request = clamp(target_speed - current_speed, -0.5, 0.5)

        return {
            "active": True,
            "target_speed_mps": target_speed,
            "acceleration_request_mps2": acceleration_request,
        }


class AEBAdapter:
    def run(self, feature_input: dict, enabled: bool) -> dict:
        if not enabled:
            return {
                "active": False,
                "emergency_stop": False,
            }

        if not feature_input.get("distance_valid", False):
            return {
                "active": False,
                "emergency_stop": False,
            }

        distance = float(feature_input.get("front_obstacle_distance_m", 99.0))

        emergency_stop = distance <= 0.35

        return {
            "active": emergency_stop,
            "emergency_stop": emergency_stop,
        }
