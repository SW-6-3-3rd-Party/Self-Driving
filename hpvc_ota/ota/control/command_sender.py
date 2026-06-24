import json
from dataclasses import asdict

from config_paths import CONFIG_DIR

from control.command_types import (
    FinalVehicleCommand,
    FrontZoneCommand,
    RearDriveControlV1,
    RearDriveControlV2,
)

from comm.packet import (
    MSG_ID_FRONT_ZONE_COMMAND,
    MSG_ID_REAR_DRIVE_CONTROL_V1,
    MSG_ID_REAR_DRIVE_CONTROL_V2,
    build_packet,
)

from comm.payload_builder import (
    build_front_zone_payload,
    build_rear_drive_control_v1_payload,
    build_rear_drive_control_v2_payload,
)

from comm.udp_socket_manager import UDPSocketManager


TURN_SIGNAL_ENUM = {
    "OFF": 0,
    "LEFT": 1,
    "RIGHT": 2,
}

DRIVE_DIRECTION_ENUM = {
    "STOP": 0,
    "FORWARD": 1,
    "REVERSE": 2,
}

CONTROL_MODE_ENUM = {
    "MANUAL": 0,
    "ACC": 1,
    "AEB": 2,
}


class CommandSender:
    def __init__(self):
        self.socket_manager = UDPSocketManager()

        self.front_alive_count = 0
        self.rear_alive_count = 0
        self.front_seq = 0
        self.rear_seq = 0

        self.send_count = 0
        self.last_log_key = None

        self.enable_udp = False
        self.front_ip = "127.0.0.1"
        self.front_port = 5100
        self.rear_ip = "127.0.0.1"
        self.rear_port = 5110

        self.load_endpoint_config()

    def load_endpoint_config(self):
        config_path = CONFIG_DIR / "zone_endpoints.json"

        if not config_path.exists():
            print(f"[COMMAND SENDER] endpoint config not found: {config_path}")
            return

        with open(config_path, "r", encoding="utf-8") as f:
            config = json.load(f)

        self.enable_udp = bool(config.get("enable_udp", False))

        front = config.get("front_zone", {})
        rear = config.get("rear_zone", {})

        self.front_ip = front.get("ip", "127.0.0.1")
        self.front_port = int(front.get("port", 5100))

        self.rear_ip = rear.get("ip", "127.0.0.1")
        self.rear_port = int(rear.get("port", 5110))

        print(
            "[COMMAND SENDER] "
            f"enable_udp={self.enable_udp}, "
            f"front={self.front_ip}:{self.front_port}, "
            f"rear={self.rear_ip}:{self.rear_port}"
        )

    def _next_front_alive(self):
        self.front_alive_count = (self.front_alive_count + 1) % 16
        return self.front_alive_count

    def _next_rear_alive(self):
        self.rear_alive_count = (self.rear_alive_count + 1) % 16
        return self.rear_alive_count

    def _next_front_seq(self):
        self.front_seq = (self.front_seq + 1) % 65536
        return self.front_seq

    def _next_rear_seq(self):
        self.rear_seq = (self.rear_seq + 1) % 65536
        return self.rear_seq

    def should_use_rear_v2(self) -> bool:
        version_path = CONFIG_DIR / "device_versions.json"

        if not version_path.exists():
            return False

        try:
            with open(version_path, "r", encoding="utf-8") as f:
                versions = json.load(f)

            rear_version = str(versions.get("REAR_ZONE", "1.0.0"))
            return rear_version.startswith("2.")

        except Exception:
            return False

    def build_front_zone_command(
        self,
        final_command: FinalVehicleCommand,
    ) -> FrontZoneCommand:
        command_valid = 1 if final_command.command_valid else 0

        if final_command.emergency_stop:
            steering_angle_rad = 0.0
            steering_valid = 0
            emergency_center = 1
        else:
            steering_angle_rad = final_command.final_steering_angle_rad
            steering_valid = 1
            emergency_center = 0

        return FrontZoneCommand(
            steering_angle_rad=steering_angle_rad,
            steering_valid=steering_valid,
            emergency_center=emergency_center,
            turn_signal=TURN_SIGNAL_ENUM.get(final_command.turn_signal, 0),
            command_valid=command_valid,
            alive_count=self._next_front_alive(),
        )

    def build_rear_drive_control_v1(
        self,
        final_command: FinalVehicleCommand,
    ) -> RearDriveControlV1:
        command_valid = 1 if final_command.command_valid else 0

        if final_command.emergency_stop:
            drive_direction = DRIVE_DIRECTION_ENUM["STOP"]
            target_speed_mps = 0.0
            normal_stop = 1
        else:
            drive_direction = DRIVE_DIRECTION_ENUM.get(
                final_command.final_drive_command,
                0,
            )
            target_speed_mps = final_command.final_target_speed
            normal_stop = 1 if final_command.final_drive_command == "STOP" else 0

        return RearDriveControlV1(
            drive_direction=drive_direction,
            target_speed_mps=target_speed_mps,
            normal_stop=normal_stop,
            turn_signal=TURN_SIGNAL_ENUM.get(final_command.turn_signal, 0),
            command_valid=command_valid,
            alive_count=self._next_rear_alive(),
        )

    def build_rear_drive_control_v2(
        self,
        final_command: FinalVehicleCommand,
    ) -> RearDriveControlV2:
        command_valid = 1 if final_command.command_valid else 0

        if final_command.emergency_stop:
            drive_direction = DRIVE_DIRECTION_ENUM["STOP"]
            target_speed_mps = 0.0
            emergency_stop = 1
        else:
            drive_direction = DRIVE_DIRECTION_ENUM.get(
                final_command.final_drive_command,
                0,
            )
            target_speed_mps = final_command.final_target_speed
            emergency_stop = 0

        return RearDriveControlV2(
            control_mode=CONTROL_MODE_ENUM.get(final_command.control_mode, 0),
            drive_direction=drive_direction,
            target_speed_mps=target_speed_mps,
            acceleration_request_mps2=final_command.acceleration_request_mps2,
            emergency_stop=emergency_stop,
            turn_signal=TURN_SIGNAL_ENUM.get(final_command.turn_signal, 0),
            command_valid=command_valid,
            alive_count=self._next_rear_alive(),
        )

    def send_command(self, final_command: FinalVehicleCommand):
        front_command = self.build_front_zone_command(final_command)

        if self.should_use_rear_v2():
            rear_command = self.build_rear_drive_control_v2(final_command)
            rear_msg_id = MSG_ID_REAR_DRIVE_CONTROL_V2
            rear_payload = build_rear_drive_control_v2_payload(rear_command)
            rear_type = "RearDriveControl_v2"
        else:
            rear_command = self.build_rear_drive_control_v1(final_command)
            rear_msg_id = MSG_ID_REAR_DRIVE_CONTROL_V1
            rear_payload = build_rear_drive_control_v1_payload(rear_command)
            rear_type = "RearDriveControl_v1"

        front_payload = build_front_zone_payload(front_command)

        front_packet = build_packet(
            msg_id=MSG_ID_FRONT_ZONE_COMMAND,
            payload=front_payload,
            seq=self._next_front_seq(),
        )

        rear_packet = build_packet(
            msg_id=rear_msg_id,
            payload=rear_payload,
            seq=self._next_rear_seq(),
        )

        if self.enable_udp:
            try:
                self.socket_manager.send(
                    front_packet,
                    self.front_ip,
                    self.front_port,
                )

                self.socket_manager.send(
                    rear_packet,
                    self.rear_ip,
                    self.rear_port,
                )

            except OSError as exc:
                print(f"[UDP SEND ERROR] {exc}")

        self._log_if_needed(final_command, front_command, rear_command, rear_type)

        return front_command, rear_command

    def _log_if_needed(
        self,
        final_command: FinalVehicleCommand,
        front_command: FrontZoneCommand,
        rear_command,
        rear_type: str,
    ):
        self.send_count += 1

        log_key = (
            final_command.control_mode,
            final_command.final_drive_command,
            round(final_command.final_target_speed, 3),
            round(final_command.final_steering_angle_rad, 3),
            final_command.emergency_stop,
            final_command.turn_signal,
            rear_type,
        )

        if log_key == self.last_log_key and self.send_count % 20 != 0:
            return

        self.last_log_key = log_key

        print(
            "[CONTROL TX] "
            f"mode={final_command.control_mode}, "
            f"drive={final_command.final_drive_command}, "
            f"speed={final_command.final_target_speed}, "
            f"steering_rad={final_command.final_steering_angle_rad}, "
            f"emergency_stop={final_command.emergency_stop}, "
            f"turn_signal={final_command.turn_signal}, "
            f"rear_type={rear_type}"
        )

        print(f"[CONTROL TX] Front={asdict(front_command)}")
        print(f"[CONTROL TX] Rear={asdict(rear_command)}")

    # 기존 ota_server.py와 호환용
    def send_mock(self, final_command: FinalVehicleCommand):
        return self.send_command(final_command)
