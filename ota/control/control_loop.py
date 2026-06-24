import threading
import time
from dataclasses import asdict

from control.pc_command_adapter import build_pc_control_command


class HPVCControlLoop:
    def __init__(
        self,
        vehicle_status: dict,
        status_lock,
        udp_receiver_manager,
        feature_manager,
        arbitrator,
        command_sender,
        period_sec: float = 0.05,
    ):
        self.vehicle_status = vehicle_status
        self.status_lock = status_lock
        self.udp_receiver_manager = udp_receiver_manager
        self.feature_manager = feature_manager
        self.arbitrator = arbitrator
        self.command_sender = command_sender
        self.period_sec = period_sec

        self.running = False
        self.thread = None
        self.loop_count = 0

    def start(self):
        if self.running:
            return

        self.running = True

        self.thread = threading.Thread(
            target=self._loop,
            daemon=True,
        )
        self.thread.start()

        print(f"[CONTROL LOOP] started period={self.period_sec}s")

    def stop(self):
        self.running = False

    def _loop(self):
        next_time = time.monotonic()

        while self.running:
            self.run_once()

            next_time += self.period_sec
            sleep_time = next_time - time.monotonic()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()

    def run_once(self):
        self.loop_count += 1

        with self.status_lock:
            local_status = dict(self.vehicle_status)

        self._apply_pc_command_timeout(local_status)

        pc_command = build_pc_control_command(local_status)

        comm_status = self.udp_receiver_manager.get_status()

        feature_request = self.feature_manager.build_feature_request(
            comm_status=comm_status,
            vehicle_status=local_status,
        )

        final_command = self.arbitrator.arbitrate(
            pc_command=pc_command,
            feature_request=feature_request,
        )

        front_command, rear_command = self.command_sender.send_command(
            final_command
        )

        with self.status_lock:
            self.vehicle_status["current_mode"] = final_command.control_mode
            self.vehicle_status["last_final_command"] = asdict(final_command)
            self.vehicle_status["last_front_zone_command"] = asdict(front_command)
            self.vehicle_status["last_rear_zone_command"] = asdict(rear_command)
            self.vehicle_status["last_feature_request"] = asdict(feature_request)
            self.vehicle_status["control_loop_count"] = self.loop_count

    def _apply_pc_command_timeout(self, status: dict):
        last_time = status.get("last_pc_command_time")

        if last_time is None:
            return

        age_sec = time.time() - float(last_time)

        if age_sec > 0.5:
            status["drive_command"] = "STOP"
            status["target_speed"] = 0.0
