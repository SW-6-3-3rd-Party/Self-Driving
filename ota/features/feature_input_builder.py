class FeatureInputBuilder:
    def build(self, comm_status: dict, vehicle_status: dict) -> dict:
        center = self._payload(comm_status.get("center"))
        front = self._payload(comm_status.get("front"))
        rear = self._payload(comm_status.get("rear"))
        link_status = comm_status.get("link_status", {})

        center_link = link_status.get("center", {})
        front_link = link_status.get("front", {})
        rear_link = link_status.get("rear", {})

        lane_features = center.get("lane_features", [0.0] * 10)
        lane_valid = bool(center.get("lane_valid", 0)) and not center_link.get("timeout", True)

        side_left = float(center.get("side_left_distance_m", 99.0))
        side_right = float(center.get("side_right_distance_m", 99.0))

        front_left = float(front.get("front_left_diag_distance_m", 99.0))
        front_right = float(front.get("front_right_diag_distance_m", 99.0))
        rear_left = float(rear.get("rear_left_diag_distance_m", 99.0))
        rear_right = float(rear.get("rear_right_diag_distance_m", 99.0))

        front_obstacle_distance_m = self._front_obstacle_distance(front)
        distance_valid = self._distance_valid(front) and not front_link.get("timeout", True)

        vehicle_speed_mps = float(rear.get("vehicle_speed_mps", 0.0))

        ultrasonic6 = [
            front_left,
            front_right,
            side_left,
            side_right,
            rear_left,
            rear_right,
        ]

        lca_data_valid = (
            lane_valid
            and not center_link.get("timeout", True)
            and not front_link.get("timeout", True)
            and not rear_link.get("timeout", True)
            and self._front_ultrasonic_valid(front)
            and self._side_ultrasonic_valid(center)
            and self._rear_ultrasonic_valid(rear)
        )

        return {
            "lane_features": lane_features,
            "lane_valid": lane_valid,
            "ultrasonic6": ultrasonic6,
            "lca_data_valid": lca_data_valid,
            "turn_request": int(vehicle_status.get("turn_request", 0)),
            "front_obstacle_distance_m": front_obstacle_distance_m,
            "distance_valid": distance_valid,
            "vehicle_speed_mps": vehicle_speed_mps,
            "pc_target_speed_mps": float(vehicle_status.get("target_speed", 0.0)),
            "link_status": link_status,
        }

    def _payload(self, item):
        if not item:
            return {}

        return item.get("payload", {}) or {}

    def _front_obstacle_distance(self, front: dict) -> float:
        if "front_obstacle_distance_m" in front:
            return float(front.get("front_obstacle_distance_m", 99.0))

        return float(front.get("tof_distance_m", 99.0))

    def _distance_valid(self, front: dict) -> bool:
        if "distance_valid" in front:
            return bool(front.get("distance_valid", 0))

        sensor_valid = int(front.get("sensor_valid", 0))
        return bool(sensor_valid & 0b100)

    def _front_ultrasonic_valid(self, front: dict) -> bool:
        sensor_valid = int(front.get("sensor_valid", 0))
        left_valid = bool(sensor_valid & 0b001)
        right_valid = bool(sensor_valid & 0b010)
        return left_valid and right_valid

    def _side_ultrasonic_valid(self, center: dict) -> bool:
        side_valid = int(center.get("side_ultrasonic_valid", 0))
        left_valid = bool(side_valid & 0b001)
        right_valid = bool(side_valid & 0b010)
        return left_valid and right_valid

    def _rear_ultrasonic_valid(self, rear: dict) -> bool:
        rear_valid = int(rear.get("rear_sensor_valid", 0))
        left_valid = bool(rear_valid & 0b001)
        right_valid = bool(rear_valid & 0b010)
        return left_valid and right_valid
