"""HPVC runtime control API for web buttons and status polling.

This sidecar stores requested LKAS/LCA state, exposes the current control
snapshot over JSON, and also emits the Front TC375 steering UDP packet so the
runtime API can drive the bench setup directly.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
from dataclasses import dataclass, asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import struct
import threading
import time
from typing import Any
import zlib


TURN_SIGNAL_OFF = 0
TURN_SIGNAL_LEFT = 1
TURN_SIGNAL_RIGHT = 2

HPVC_VERSION_INFO = {
    "HPVC": "1.0.0",
    "FRONT_ZONE": "1.0.0",
    "REAR_ZONE": "1.0.0",
}

HPVC_OTA_STATUS = {
    "state": "IDLE",
    "progress": 0,
    "error_code": None,
    "running": False,
}

RUNTIME_CONTROL_HOST = "127.0.0.1"
RUNTIME_CONTROL_PORT = 5201
RUNTIME_CONTROL_SOURCE_PORT = 5203

FRONT_AEB_HOST = "0.0.0.0"
FRONT_AEB_PORT = 5011
FRONT_AEB_MAGIC = b"AEB1"
FRONT_AEB1_SIZE = 22
FRONT_AEB1_HEADER_FORMAT = "<4sBBHI"
FRONT_AEB1_TAIL_FORMAT = "<IHHH"
FRONT_AEB_V1_FORMAT = "<fffBBB"
FRONT_AEB_V2_FORMAT = "<fffffBBBB"

REAR_STATUS_HOST = "0.0.0.0"
REAR_STATUS_PORT = 5012
REAR_STATUS_V1_FORMAT = "<ffBBB"
REAR_STATUS_V2_FORMAT = "<fffBBBB"

FRONT_TC375_HOST = "192.168.10.11"
FRONT_TC375_PORT = 5100
FRONT_TC375_SOURCE_PORT = 5101
FRONT_TC375_LOCAL_BIND = "0.0.0.0"

REAR_TC375_HOST = "192.168.10.12"
REAR_TC375_PORT = 5110

HPSC_MAGIC = b"HPSC"
HPSC_VERSION = 1
HPSC_HEADER_SIZE = 32
HPSC_CONTROL_MODE_STEERING = 1
HPSC_BODY_FORMAT = "<4sBBBBIQffHHI"
HPSC_PACKET_SIZE = 40
HPSC_FLAG_STEERING_VALID = 1 << 0
HPSC_FLAG_EMERGENCY_CENTER = 1 << 1
HPSC_FLAG_UPSTREAM_VALID = 1 << 2

REAR_PROTOCOL_VERSION = 1
REAR_MSG_REAR_DRIVE_V2 = 0x43
REAR_HEADER_FORMAT = "<HBBHHIBBH"
REAR_BODY_FORMAT = "<BBffBBBBBB"
REAR_HEADER_SIZE = struct.calcsize(REAR_HEADER_FORMAT)
REAR_BODY_SIZE = struct.calcsize(REAR_BODY_FORMAT)
REAR_FLAG_UPSTREAM_VALID = 1 << 0


@dataclass
class RuntimeControlState:
    requested_lkas_enabled: bool = False
    requested_turn_signal: int = TURN_SIGNAL_OFF
    requested_lca_enabled: bool = False
    requested_acc_enabled: bool = False
    requested_aeb_enabled: bool = False
    requested_drive_command: str = "STOP"
    requested_steering_command: str = "STRAIGHT"
    requested_target_speed: float = 0.0
    effective_lkas_active: bool = False
    effective_lca_active: bool = False
    effective_acc_active: bool = False
    effective_aeb_active: bool = False
    lkas_reason: str = "boot default"
    lca_reason: str = "boot default"
    aeb_reason: str = "boot default"
    acc_reason: str = "boot default"
    acc_estimate_mps2: float | None = None
    updated_monotonic_s: float = 0.0
    front_aeb: dict[str, Any] | None = None
    rear_status: dict[str, Any] | None = None

    def snapshot(self) -> dict[str, Any]:
        data = asdict(self)
        data["turn_signal"] = self.turn_signal_name(self.requested_turn_signal)
        data["turn_request"] = self.requested_turn_signal
        data["lkas_active"] = self.effective_lkas_active
        data["lca_active"] = self.effective_lca_active
        data["acc_active"] = self.effective_acc_active
        data["aeb_active"] = self.effective_aeb_active
        data["feature_flags"] = self.feature_flags()
        data["features"] = self.feature_flags()
        if self.effective_aeb_active:
            data["current_mode"] = "AEB"
        elif self.effective_acc_active:
            data["current_mode"] = "ACC"
        elif self.effective_lkas_active:
            data["current_mode"] = "LKAS"
        else:
            data["current_mode"] = "MANUAL"
        data["drive_command"] = self.requested_drive_command
        data["steering_command"] = self.requested_steering_command
        data["target_speed"] = self.requested_target_speed
        data["aeb_reason"] = self.aeb_reason
        data["acc_reason"] = self.acc_reason
        data["acc_estimate_mps2"] = self.acc_estimate_mps2
        data["front_aeb"] = self.front_aeb
        data["rear_status"] = self.rear_status
        return data

    def feature_flags(self) -> dict[str, bool]:
        return {
            "lkas_active": bool(self.effective_lkas_active),
            "lca_active": bool(self.effective_lca_active),
            "acc_active": bool(self.effective_acc_active),
            "aeb_active": bool(self.effective_aeb_active),
        }

    @staticmethod
    def turn_signal_name(value: int) -> str:
        if value == TURN_SIGNAL_LEFT:
            return "LEFT"
        if value == TURN_SIGNAL_RIGHT:
            return "RIGHT"
        return "OFF"


class RuntimeControlStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = RuntimeControlState(updated_monotonic_s=time.monotonic())
        self._tc375_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tc375_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._tc375_socket.bind((FRONT_TC375_LOCAL_BIND, FRONT_TC375_SOURCE_PORT))
        self._front_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._front_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._front_socket.bind((FRONT_AEB_HOST, FRONT_AEB_PORT))
        self._rear_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rear_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rear_socket.bind((REAR_STATUS_HOST, REAR_STATUS_PORT))
        self._front_running = threading.Event()
        self._front_running.set()
        self._rear_running = threading.Event()
        self._rear_running.set()
        self._publish_running = threading.Event()
        self._publish_running.set()
        self._front_thread = threading.Thread(target=self._front_loop, name="front-aeb", daemon=True)
        self._rear_thread = threading.Thread(target=self._rear_loop, name="rear-status", daemon=True)
        self._publish_thread = threading.Thread(target=self._publish_loop, name="runtime-publish", daemon=True)
        self._front_thread.start()
        self._rear_thread.start()
        self._publish_thread.start()
        self._sequence = 0
        self._alive = 0

    def get(self) -> RuntimeControlState:
        with self._lock:
            return RuntimeControlState(**asdict(self._state))

    def update(self, **changes: Any) -> RuntimeControlState:
        with self._lock:
            for key, value in changes.items():
                if not hasattr(self._state, key):
                    raise KeyError(key)
                setattr(self._state, key, value)
            self._state.updated_monotonic_s = time.monotonic()
            self._recompute_effective_locked()
            self._publish_runtime_control_locked()
            return RuntimeControlState(**asdict(self._state))

    def publish_snapshot(self) -> None:
        with self._lock:
            self._publish_runtime_control_locked()

    def close(self) -> None:
        self._front_running.clear()
        self._rear_running.clear()
        self._publish_running.clear()
        with self._lock:
            try:
                self._tc375_socket.close()
            except OSError:
                pass
            try:
                self._front_socket.close()
            except OSError:
                pass
            try:
                self._rear_socket.close()
            except OSError:
                pass
        if self._front_thread.is_alive():
            self._front_thread.join(timeout=1.0)
        if self._rear_thread.is_alive():
            self._rear_thread.join(timeout=1.0)
        if self._publish_thread.is_alive():
            self._publish_thread.join(timeout=1.0)

    def _recompute_effective_locked(self) -> None:
        self._state.effective_lkas_active = bool(self._state.requested_lkas_enabled)
        self._state.effective_lca_active = bool(
            self._state.requested_lkas_enabled
            and self._state.requested_lca_enabled
            and self._state.requested_turn_signal in (TURN_SIGNAL_LEFT, TURN_SIGNAL_RIGHT)
        )
        if self._state.effective_lca_active:
            self._state.lca_reason = "turn signal and LKAS request accepted"
        elif self._state.requested_lca_enabled and not self._state.requested_lkas_enabled:
            self._state.lca_reason = "LKAS must be enabled first"
        elif self._state.requested_turn_signal == TURN_SIGNAL_OFF:
            self._state.lca_reason = "turn signal off"
        else:
            self._state.lca_reason = "waiting for lane-change preconditions"
        self._state.lkas_reason = (
            "operator enabled LKAS" if self._state.requested_lkas_enabled else "operator disabled LKAS"
        )
        self._state.effective_acc_active = bool(self._state.requested_acc_enabled)
        front = self._state.front_aeb
        rear = self._state.rear_status
        self._state.acc_estimate_mps2, self._state.acc_reason = self._estimate_acceleration_locked(front, rear)
        self._state.effective_aeb_active = bool(self._state.requested_aeb_enabled)
        if front is not None:
            tof = front.get("front_tof_m")
            if isinstance(tof, (int, float)) and math.isfinite(float(tof)) and float(tof) > 0.0:
                if float(tof) <= 0.18:
                    self._state.effective_aeb_active = True
                    self._state.aeb_reason = "front ToF emergency guard"
                elif float(tof) <= 0.30 and self._state.effective_acc_active:
                    self._state.aeb_reason = "front ToF caution"
                else:
                    self._state.aeb_reason = "operator enabled AEB" if self._state.requested_aeb_enabled else "front range nominal"
            else:
                self._state.aeb_reason = "front ToF invalid"
        else:
            self._state.aeb_reason = "waiting for front AEB packet"

    def _estimate_acceleration_locked(
        self,
        front: dict[str, Any] | None,
        rear: dict[str, Any] | None,
    ) -> tuple[float | None, str]:
        if front is None or rear is None:
            return None, "waiting for front/rear inputs"

        tof = front.get("front_tof_m")
        vehicle_speed = rear.get("rear_vehicle_speed_mps")
        if not isinstance(tof, (int, float)) or not math.isfinite(float(tof)) or float(tof) <= 0.0:
            return None, "front ToF invalid"
        if vehicle_speed is None or not isinstance(vehicle_speed, (int, float)) or not math.isfinite(float(vehicle_speed)):
            return None, "rear speed invalid"

        tof_m = float(tof)
        rear_speed = max(0.0, float(vehicle_speed))
        desired_gap = max(0.35, 0.45 + 0.90 * rear_speed)
        gap_error = tof_m - desired_gap
        acc = 1.40 * gap_error - 0.35 * rear_speed
        reason = "ACC disabled"

        if self._state.requested_aeb_enabled or tof_m <= 0.18:
            acc = min(acc, -1.50)
            reason = "AEB guard from front ToF"
        elif self._state.requested_acc_enabled:
            reason = "ACC gap control"

        acc = max(-2.50, min(1.50, acc))
        return acc, reason

    def _requested_steering_angle(self) -> float:
        steering = self._state.requested_steering_command.strip().upper()
        if steering == "LEFT":
            return 0.25
        if steering == "RIGHT":
            return -0.25
        return 0.0

    def _requested_drive_direction(self) -> int:
        drive = self._state.requested_drive_command.strip().upper()
        if drive == "FORWARD":
            return 1
        if drive == "REVERSE":
            return 2
        return 0

    def _rear_control_mode(self) -> int:
        if self._state.effective_aeb_active:
            return 2
        if self._state.effective_acc_active:
            return 1
        return 0

    @staticmethod
    def _crc16_ccitt(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def _build_rear_command_packet(self) -> bytes:
        control_mode = self._rear_control_mode()
        drive_direction = self._requested_drive_direction()
        target_speed = max(0.0, float(self._state.requested_target_speed))
        acceleration = self._state.acc_estimate_mps2
        if acceleration is None or not math.isfinite(float(acceleration)):
            acceleration = 0.0
        emergency_stop = 0
        brake_command = 1 if self._state.effective_aeb_active else 0
        turn_signal = int(self._state.requested_turn_signal)
        command_valid = 1
        alive = self._alive & 0xFF
        timestamp_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
        flags = REAR_FLAG_UPSTREAM_VALID

        header_wo_crc = struct.pack(
            "<HBBHHIBB",
            0x4850,
            REAR_PROTOCOL_VERSION,
            REAR_MSG_REAR_DRIVE_V2,
            REAR_BODY_SIZE,
            self._sequence & 0xFFFF,
            timestamp_ms,
            0x01,
            flags,
        )
        body = struct.pack(
            REAR_BODY_FORMAT,
            control_mode & 0xFF,
            drive_direction & 0xFF,
            float(target_speed),
            0.0 if brake_command else float(acceleration),
            emergency_stop & 0xFF,
            turn_signal & 0xFF,
            command_valid & 0xFF,
            alive,
            0,
            brake_command & 0xFF,
        )
        crc = self._crc16_ccitt(header_wo_crc + b"\x00\x00" + body)
        header = header_wo_crc + struct.pack("<H", crc)
        return header + body

    def _publish_runtime_control_locked(self) -> None:
        payload = bytes([1 if self._state.effective_lkas_active else 0])
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((RUNTIME_CONTROL_HOST, RUNTIME_CONTROL_SOURCE_PORT))
                sock.sendto(payload, (RUNTIME_CONTROL_HOST, RUNTIME_CONTROL_PORT))
        except OSError:
            pass

    def _rear_loop(self) -> None:
        self._rear_socket.settimeout(0.5)
        while self._rear_running.is_set():
            try:
                data, addr = self._rear_socket.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            snapshot = self._decode_rear_status(data)
            if snapshot is None:
                continue
            snapshot["source"] = f"{addr[0]}:{addr[1]}"
            snapshot["arrival_monotonic_s"] = time.monotonic()
            with self._lock:
                self._state.rear_status = snapshot
                self._recompute_effective_locked()

    def _front_loop(self) -> None:
        self._front_socket.settimeout(0.5)
        while self._front_running.is_set():
            try:
                data, addr = self._front_socket.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            snapshot = self._decode_front_aeb(data)
            if snapshot is None:
                continue
            snapshot["source"] = f"{addr[0]}:{addr[1]}"
            snapshot["arrival_monotonic_s"] = time.monotonic()
            with self._lock:
                self._state.front_aeb = snapshot
                self._recompute_effective_locked()

    @staticmethod
    def _decode_front_aeb(data: bytes) -> dict[str, Any] | None:
        if len(data) >= FRONT_AEB1_SIZE and data[:4] == FRONT_AEB_MAGIC:
            try:
                _, version, valid_mask, tof_diag, sequence = struct.unpack_from(
                    FRONT_AEB1_HEADER_FORMAT, data, 0
                )
                timestamp_ms, tof_cm_x10, left_cm_x10, right_cm_x10 = struct.unpack_from(
                    FRONT_AEB1_TAIL_FORMAT, data, 12
                )
            except struct.error:
                return None
            if version != 1:
                return None
            return {
                "front_left_diag_distance_m": float(left_cm_x10) / 1000.0,
                "front_right_diag_distance_m": float(right_cm_x10) / 1000.0,
                "front_tof_m": float(tof_cm_x10) / 1000.0,
                "front_sensor_valid": int(valid_mask),
                "front_sensor_fault": int(tof_diag),
                "front_alive_count": int(sequence & 0x0F),
                "front_sequence": int(sequence),
                "front_timestamp_ms": int(timestamp_ms),
                "front_speed_valid": bool(valid_mask & 0x04),
                "front_packet_size_bytes": len(data),
                "front_packet_format": "AEB1",
            }
        if len(data) >= struct.calcsize(FRONT_AEB_V2_FORMAT):
            try:
                left_m, right_m, tof_m, filtered_tof_m, front_obstacle_m, distance_valid, valid_mask, sensor_fault, alive = struct.unpack_from(
                    FRONT_AEB_V2_FORMAT, data, 0
                )
            except struct.error:
                return None
            return {
                "front_left_diag_distance_m": float(left_m),
                "front_right_diag_distance_m": float(right_m),
                "front_tof_m": float(tof_m),
                "front_filtered_tof_m": float(filtered_tof_m),
                "front_obstacle_distance_m": float(front_obstacle_m),
                "front_distance_valid": int(distance_valid),
                "front_sensor_valid": int(valid_mask),
                "front_sensor_fault": int(sensor_fault),
                "front_alive_count": int(alive),
                "front_speed_valid": bool(distance_valid),
                "front_packet_size_bytes": len(data),
                "front_packet_format": "legacy-v2",
            }
        if len(data) >= struct.calcsize(FRONT_AEB_V1_FORMAT):
            try:
                left_m, right_m, tof_m, valid_mask, sensor_fault, alive = struct.unpack_from(
                    FRONT_AEB_V1_FORMAT, data, 0
                )
            except struct.error:
                return None
            return {
                "front_left_diag_distance_m": float(left_m),
                "front_right_diag_distance_m": float(right_m),
                "front_tof_m": float(tof_m),
                "front_sensor_valid": int(valid_mask),
                "front_sensor_fault": int(sensor_fault),
                "front_alive_count": int(alive),
                "front_speed_valid": True,
                "front_packet_size_bytes": len(data),
                "front_packet_format": "legacy-v1",
            }
        return None

    @staticmethod
    def _decode_rear_status(data: bytes) -> dict[str, Any] | None:
        if len(data) >= struct.calcsize(REAR_STATUS_V2_FORMAT):
            try:
                left_m, right_m, vehicle_speed_mps, valid, motor_state, alive, reserved = struct.unpack_from(
                    REAR_STATUS_V2_FORMAT, data, 0
                )
            except struct.error:
                return None
            return {
                "rear_left_diag_distance_m": float(left_m),
                "rear_right_diag_distance_m": float(right_m),
                "rear_vehicle_speed_mps": float(vehicle_speed_mps),
                "rear_sensor_valid": int(valid),
                "rear_motor_state": int(motor_state),
                "rear_alive_count": int(alive),
                "rear_reserved": int(reserved),
                "rear_speed_valid": math.isfinite(float(vehicle_speed_mps)),
                "rear_packet_size_bytes": len(data),
                "rear_packet_format": "v2",
            }
        if len(data) >= struct.calcsize(REAR_STATUS_V1_FORMAT):
            try:
                left_m, right_m, valid, motor_state, alive = struct.unpack_from(REAR_STATUS_V1_FORMAT, data, 0)
            except struct.error:
                return None
            return {
                "rear_left_diag_distance_m": float(left_m),
                "rear_right_diag_distance_m": float(right_m),
                "rear_vehicle_speed_mps": None,
                "rear_sensor_valid": int(valid),
                "rear_motor_state": int(motor_state),
                "rear_alive_count": int(alive),
                "rear_speed_valid": False,
                "rear_packet_size_bytes": len(data),
                "rear_packet_format": "v1",
            }
        return None

    def _publish_runtime_control_locked(self) -> None:
        try:
            runtime_payload = bytes([1 if self._state.effective_lkas_active else 0])
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((RUNTIME_CONTROL_HOST, RUNTIME_CONTROL_SOURCE_PORT))
                sock.sendto(runtime_payload, (RUNTIME_CONTROL_HOST, RUNTIME_CONTROL_PORT))
        except OSError:
            pass

        try:
            steering_valid = bool(self._state.effective_lkas_active)
            emergency_center = not steering_valid
            flags = 0
            if steering_valid:
                flags |= HPSC_FLAG_STEERING_VALID
            if emergency_center:
                flags |= HPSC_FLAG_EMERGENCY_CENTER
            flags |= HPSC_FLAG_UPSTREAM_VALID
            angle = self._requested_steering_angle() if steering_valid else 0.0
            body = struct.pack(
                HPSC_BODY_FORMAT,
                HPSC_MAGIC,
                HPSC_VERSION,
                HPSC_CONTROL_MODE_STEERING,
                flags,
                HPSC_HEADER_SIZE,
                self._sequence & 0xFFFFFFFF,
                time.monotonic_ns() // 1000,
                float(angle),
                0.5,
                self._alive & 0xFFFF,
                0,
                0,
            )
            packet = body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
            self._tc375_socket.sendto(packet, (FRONT_TC375_HOST, FRONT_TC375_PORT))
            rear_packet = self._build_rear_command_packet()
            self._tc375_socket.sendto(rear_packet, (REAR_TC375_HOST, REAR_TC375_PORT))
            self._sequence = (self._sequence + 1) & 0xFFFFFFFF
            self._alive = (self._alive + 1) & 0xFFFF
        except OSError:
            pass

    def _publish_loop(self) -> None:
        interval_s = 0.05
        while self._publish_running.is_set():
            with self._lock:
                self._publish_runtime_control_locked()
            time.sleep(interval_s)


class _Handler(BaseHTTPRequestHandler):
    store: RuntimeControlStore

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/vehicle/status", "/control/status"):
            self._json_response(200, self.store.get().snapshot())
            return
        if self.path == "/version":
            self._json_response(200, HPVC_VERSION_INFO)
            return
        if self.path == "/ota/status":
            self._json_response(200, HPVC_OTA_STATUS)
            return
        if self.path == "/features/status":
            state = self.store.get().snapshot()
            self._json_response(
                200,
                {
                    "feature_flags": state["feature_flags"],
                    "features": state["features"],
                    "requested": {
                        "lkas_active": state["requested_lkas_enabled"],
                        "lca_active": state["requested_lca_enabled"],
                        "acc_active": state["requested_acc_enabled"],
                        "aeb_active": state["requested_aeb_enabled"],
                    },
                },
            )
            return
        if self.path == "/health":
            self._json_response(200, {"ok": True})
            return
        self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802
        payload = self._read_json()
        if self.path == "/features/enable":
            changes: dict[str, Any] = {}
            if "lkas_active" in payload:
                enabled = self._as_bool(payload.get("lkas_active"))
                if enabled is None:
                    self._json_response(400, {"error": "lkas_active must be boolean"})
                    return
                changes["requested_lkas_enabled"] = enabled
                changes["requested_lca_enabled"] = enabled
            if "lca_active" in payload:
                enabled = self._as_bool(payload.get("lca_active"))
                if enabled is None:
                    self._json_response(400, {"error": "lca_active must be boolean"})
                    return
                changes["requested_lca_enabled"] = enabled
            if "acc_active" in payload:
                enabled = self._as_bool(payload.get("acc_active"))
                if enabled is None:
                    self._json_response(400, {"error": "acc_active must be boolean"})
                    return
                changes["requested_acc_enabled"] = enabled
            if "aeb_active" in payload:
                enabled = self._as_bool(payload.get("aeb_active"))
                if enabled is None:
                    self._json_response(400, {"error": "aeb_active must be boolean"})
                    return
                changes["requested_aeb_enabled"] = enabled
            if not changes:
                self._json_response(400, {"error": "no supported feature fields provided"})
                return
            state = self.store.update(**changes)
            self._json_response(200, state.snapshot())
            return
        if self.path == "/control/lkas":
            enabled = self._as_bool(payload.get("enabled"))
            if enabled is None:
                self._json_response(400, {"error": "enabled must be boolean"})
                return
            self._json_response(200, self.store.update(requested_lkas_enabled=enabled).snapshot())
            return
        if self.path == "/control/manual":
            changes = {}
            drive_command = payload.get("drive_command")
            steering_command = payload.get("steering_command")
            target_speed = payload.get("target_speed")
            if isinstance(drive_command, str):
                changes["requested_drive_command"] = drive_command.strip().upper()
            if isinstance(steering_command, str):
                changes["requested_steering_command"] = steering_command.strip().upper()
            if isinstance(target_speed, (int, float)):
                changes["requested_target_speed"] = float(target_speed)
            if not changes:
                self._json_response(400, {"error": "no supported manual fields provided"})
                return
            self._json_response(200, self.store.update(**changes).snapshot())
            return
        if self.path == "/control/turn-signal":
            turn_signal = self._as_turn_signal(payload.get("turn_signal"))
            if turn_signal is None:
                self._json_response(400, {"error": "turn_signal must be OFF, LEFT, or RIGHT"})
                return
            self._json_response(200, self.store.update(requested_turn_signal=turn_signal).snapshot())
            return
        if self.path == "/control/lca":
            enabled = self._as_bool(payload.get("enabled"))
            if enabled is None:
                self._json_response(400, {"error": "enabled must be boolean"})
                return
            self._json_response(200, self.store.update(requested_lca_enabled=enabled).snapshot())
            return
        if self.path == "/control/reset":
            self._json_response(
                200,
                self.store.update(
                    requested_lkas_enabled=False,
                    requested_turn_signal=TURN_SIGNAL_OFF,
                    requested_lca_enabled=False,
                    requested_acc_enabled=False,
                    requested_aeb_enabled=False,
                    requested_drive_command="STOP",
                    requested_steering_command="STRAIGHT",
                    requested_target_speed=0.0,
                ).snapshot(),
            )
            return
        self.send_error(404)

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return {}
        if not isinstance(payload, dict):
            return {}
        return payload

    def _json_response(self, status: int, payload: dict[str, Any]) -> None:
        data = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    @staticmethod
    def _as_bool(value: Any) -> bool | None:
        if isinstance(value, bool):
            return value
        return None

    @staticmethod
    def _as_turn_signal(value: Any) -> int | None:
        if isinstance(value, str):
            normalized = value.strip().upper()
            if normalized == "OFF":
                return TURN_SIGNAL_OFF
            if normalized == "LEFT":
                return TURN_SIGNAL_LEFT
            if normalized == "RIGHT":
                return TURN_SIGNAL_RIGHT
        if isinstance(value, int) and value in (TURN_SIGNAL_OFF, TURN_SIGNAL_LEFT, TURN_SIGNAL_RIGHT):
            return value
        return None


def serve(host: str, port: int) -> None:
    store = RuntimeControlStore()
    handler = type("RuntimeControlHandler", (_Handler,), {"store": store})
    server = ThreadingHTTPServer((host, port), handler)
    stop_event = threading.Event()

    def publisher() -> None:
        while not stop_event.wait(0.05):
            store.publish_snapshot()

    publisher_thread = threading.Thread(target=publisher, name="runtime-publisher", daemon=True)
    publisher_thread.start()
    store.publish_snapshot()
    print(f"HPVC runtime API listening on http://{host}:{port}")
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        stop_event.set()
        store.close()
        server.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    serve(args.host, args.port)


if __name__ == "__main__":
    main()
