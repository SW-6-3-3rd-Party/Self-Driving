"""Flask remote controller for HPVC bench driving.

PC browser -> HPVC Flask -> FRONT steering UDP and REAR drive UDP.
Keyboard: W/S drive, A/D steer, Space stop.
"""

from __future__ import annotations

import argparse
import atexit
from dataclasses import asdict, dataclass
import socket
import struct
import threading
import time
from typing import Any
import zlib

try:
    from flask import Flask, Response, jsonify, request
except ImportError:  # pragma: no cover - handled in main()
    Flask = None  # type: ignore[assignment]
    Response = None  # type: ignore[assignment]
    jsonify = None  # type: ignore[assignment]
    request = None  # type: ignore[assignment]


DEFAULT_FRONT_HOST = "192.168.10.11"
DEFAULT_FRONT_PORT = 5100
DEFAULT_FRONT_SOURCE_IP = "0.0.0.0"
DEFAULT_FRONT_SOURCE_PORT = 5101
DEFAULT_REAR_HOST = "192.168.10.12"
DEFAULT_REAR_PORT = 5110

HPSC_MAGIC = b"HPSC"
HPSC_VERSION = 1
HPSC_HEADER_SIZE = 32
HPSC_CONTROL_DISABLED = 0
HPSC_CONTROL_STEERING_ANGLE = 1
HPSC_FLAG_STEERING_VALID = 1 << 0
HPSC_FLAG_EMERGENCY_CENTER = 1 << 1
HPSC_FLAG_UPSTREAM_VALID = 1 << 2
HPSC_BODY_FORMAT = "<4sBBBBIQffHHI"

DRIVE_STOP = "STOP"
DRIVE_FORWARD = "FORWARD"
DRIVE_REVERSE = "REVERSE"
STEER_STRAIGHT = "STRAIGHT"
STEER_LEFT = "LEFT"
STEER_RIGHT = "RIGHT"

DRIVE_DIRECTION = {
    DRIVE_STOP: 0,
    DRIVE_FORWARD: 1,
    DRIVE_REVERSE: 2,
}


@dataclass
class RemoteControlState:
    drive_command: str = DRIVE_STOP
    steering_command: str = STEER_STRAIGHT
    target_speed_mps: float = 0.30
    emergency_stop: bool = False
    sequence: int = 0
    alive_count: int = 0
    sent_front_packets: int = 0
    sent_rear_packets: int = 0
    last_error: str | None = None
    front_source: str = ""
    updated_monotonic_s: float = 0.0


class HpvcRemoteController:
    def __init__(
        self,
        *,
        front_host: str,
        front_port: int,
        front_source_ip: str,
        front_source_port: int,
        rear_host: str,
        rear_port: int,
        publish_hz: float,
        steer_rad: float,
        max_speed_mps: float,
    ) -> None:
        self.front_target = (front_host, front_port)
        self.rear_target = (rear_host, rear_port)
        self.publish_period_s = 1.0 / max(1.0, publish_hz)
        self.steer_rad = abs(float(steer_rad))
        self.max_speed_mps = max(0.01, float(max_speed_mps))
        self._lock = threading.RLock()
        self._state = RemoteControlState(updated_monotonic_s=time.monotonic())
        self._front_socket = self._open_front_socket(front_source_ip, front_source_port)
        self._rear_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = threading.Event()
        self._running.set()
        self._thread = threading.Thread(target=self._publish_loop, name="hpvc-remote-publisher", daemon=True)
        self._thread.start()

    def _open_front_socket(self, source_ip: str, source_port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((source_ip, source_port))
        except OSError as error:
            sock.bind((source_ip, 0))
            with self._lock:
                self._state.last_error = (
                    f"front source port {source_port} unavailable; using ephemeral port: {error}"
                )
        try:
            actual_ip, actual_port = sock.getsockname()
            with self._lock:
                self._state.front_source = f"{actual_ip}:{actual_port}"
        except OSError:
            pass
        return sock

    def close(self) -> None:
        self._running.clear()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        with self._lock:
            try:
                self._send_stop_locked()
            except OSError:
                pass
        self._front_socket.close()
        self._rear_socket.close()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            data = asdict(self._state)
            data["front_target"] = f"{self.front_target[0]}:{self.front_target[1]}"
            data["rear_target"] = f"{self.rear_target[0]}:{self.rear_target[1]}"
            data["publish_hz"] = round(1.0 / self.publish_period_s, 3)
            data["steer_rad"] = self._steering_angle_locked()
            data["rear_drive_direction"] = DRIVE_DIRECTION.get(self._state.drive_command, 0)
            return data

    def command(
        self,
        *,
        drive_command: str | None = None,
        steering_command: str | None = None,
        target_speed_mps: float | None = None,
        emergency_stop: bool | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if drive_command is not None:
                self._state.drive_command = self._normalize_drive(drive_command)
            if steering_command is not None:
                self._state.steering_command = self._normalize_steering(steering_command)
            if target_speed_mps is not None:
                self._state.target_speed_mps = self._clamp_speed(target_speed_mps)
            if emergency_stop is not None:
                self._state.emergency_stop = bool(emergency_stop)
                if self._state.emergency_stop:
                    self._state.drive_command = DRIVE_STOP
                    self._state.steering_command = STEER_STRAIGHT
            self._state.updated_monotonic_s = time.monotonic()
            self._publish_locked()
            return self.snapshot()

    def stop(self, *, clear_emergency: bool = True) -> dict[str, Any]:
        with self._lock:
            self._state.drive_command = DRIVE_STOP
            self._state.steering_command = STEER_STRAIGHT
            if clear_emergency:
                self._state.emergency_stop = False
            self._state.updated_monotonic_s = time.monotonic()
            self._publish_locked()
            return self.snapshot()

    @staticmethod
    def _normalize_drive(value: str) -> str:
        normalized = str(value).strip().upper()
        if normalized in ("FWD", "FORWARD", "W"):
            return DRIVE_FORWARD
        if normalized in ("REV", "REVERSE", "BACKWARD", "BACK", "S"):
            return DRIVE_REVERSE
        return DRIVE_STOP

    @staticmethod
    def _normalize_steering(value: str) -> str:
        normalized = str(value).strip().upper()
        if normalized in ("LEFT", "L", "A"):
            return STEER_LEFT
        if normalized in ("RIGHT", "R", "D"):
            return STEER_RIGHT
        return STEER_STRAIGHT

    def _clamp_speed(self, value: float) -> float:
        try:
            speed = float(value)
        except (TypeError, ValueError):
            speed = 0.0
        return max(0.0, min(self.max_speed_mps, speed))

    def _steering_angle_locked(self) -> float:
        if self._state.emergency_stop:
            return 0.0
        if self._state.steering_command == STEER_LEFT:
            return self.steer_rad
        if self._state.steering_command == STEER_RIGHT:
            return -self.steer_rad
        return 0.0

    def _pack_front_steering_locked(self, *, emergency_center: bool = False) -> bytes:
        if emergency_center:
            steering_valid = False
            control_mode = HPSC_CONTROL_DISABLED
            flags = HPSC_FLAG_EMERGENCY_CENTER
            angle_rad = 0.0
        else:
            steering_valid = not self._state.emergency_stop
            control_mode = HPSC_CONTROL_STEERING_ANGLE if steering_valid else HPSC_CONTROL_DISABLED
            flags = HPSC_FLAG_UPSTREAM_VALID
            if steering_valid:
                flags |= HPSC_FLAG_STEERING_VALID
            else:
                flags |= HPSC_FLAG_EMERGENCY_CENTER
            angle_rad = self._steering_angle_locked()

        body = struct.pack(
            HPSC_BODY_FORMAT,
            HPSC_MAGIC,
            HPSC_VERSION,
            control_mode,
            flags,
            HPSC_HEADER_SIZE,
            self._state.sequence & 0xFFFFFFFF,
            time.monotonic_ns() // 1000,
            float(angle_rad),
            2.0,
            self._state.alive_count & 0xFFFF,
            0,
            0,
        )
        return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)

    def _pack_rear_drive_locked(self) -> bytes:
        # Current REAR firmware reads these exact offsets:
        # buf[16]=control_mode, buf[17]=drive_direction, buf+20=target_speed,
        # buf+24=accel_cmd, buf[28]=emergency_stop.
        packet = bytearray(32)
        struct.pack_into("<H", packet, 0, 0x4850)
        packet[2] = 1
        packet[3] = 0x43
        struct.pack_into("<H", packet, 4, 12)
        struct.pack_into("<H", packet, 6, self._state.sequence & 0xFFFF)
        struct.pack_into("<I", packet, 8, int(time.monotonic() * 1000.0) & 0xFFFFFFFF)
        packet[16] = 0
        packet[17] = DRIVE_DIRECTION.get(self._state.drive_command, 0) & 0xFF
        speed = 0.0 if self._state.drive_command == DRIVE_STOP else self._state.target_speed_mps
        struct.pack_into("<f", packet, 20, float(speed))
        struct.pack_into("<f", packet, 24, 0.0)
        packet[28] = 1 if self._state.emergency_stop else 0
        return bytes(packet)

    def _publish_locked(self) -> None:
        try:
            self._front_socket.sendto(self._pack_front_steering_locked(), self.front_target)
            self._rear_socket.sendto(self._pack_rear_drive_locked(), self.rear_target)
            self._state.sent_front_packets += 1
            self._state.sent_rear_packets += 1
            self._state.sequence = (self._state.sequence + 1) & 0xFFFFFFFF
            self._state.alive_count = (self._state.alive_count + 1) & 0xFFFF
            self._state.last_error = None
        except OSError as error:
            self._state.last_error = str(error)

    def _send_stop_locked(self) -> None:
        self._state.drive_command = DRIVE_STOP
        self._state.steering_command = STEER_STRAIGHT
        self._rear_socket.sendto(self._pack_rear_drive_locked(), self.rear_target)
        self._front_socket.sendto(self._pack_front_steering_locked(emergency_center=True), self.front_target)

    def _publish_loop(self) -> None:
        while self._running.wait(self.publish_period_s):
            with self._lock:
                self._publish_locked()


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HPVC Remote Control</title>
<style>
:root { color-scheme: dark; font-family: Arial, sans-serif; background: #101418; color: #f5f7fa; }
body { margin: 0; min-height: 100vh; display: grid; place-items: center; }
main { width: min(920px, 94vw); display: grid; gap: 18px; }
h1 { font-size: 28px; margin: 0; }
.panel { background: #1a2129; border: 1px solid #2f3a45; border-radius: 8px; padding: 18px; }
.grid { display: grid; grid-template-columns: repeat(3, minmax(86px, 1fr)); gap: 12px; }
button { min-height: 76px; border: 0; border-radius: 8px; background: #293544; color: #fff; font-size: 18px; font-weight: 700; cursor: pointer; }
button:active, button.active { background: #3d7df0; }
button.stop { background: #b52d38; }
button.estop { background: #7f1d1d; }
.wide { grid-column: span 3; }
.row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
input[type=range] { width: min(420px, 80vw); }
pre { white-space: pre-wrap; word-break: break-word; margin: 0; color: #cbd5df; font-size: 13px; }
</style>
</head>
<body>
<main>
  <section class="panel row">
    <div>
      <h1>HPVC Remote Control</h1>
      <div>Keyboard: W/S drive, A/D steer, Space stop</div>
    </div>
  </section>
  <section class="panel">
    <div class="grid">
      <div></div><button data-drive="FORWARD">FORWARD</button><div></div>
      <button data-steer="LEFT">LEFT</button><button class="stop" id="stop">STOP</button><button data-steer="RIGHT">RIGHT</button>
      <div></div><button data-drive="REVERSE">REVERSE</button><div></div>
      <button class="estop wide" id="estop">EMERGENCY STOP</button>
    </div>
  </section>
  <section class="panel row">
    <label for="speed">Speed</label>
    <input id="speed" type="range" min="0" max="1" step="0.05" value="0.30">
    <strong id="speedText">0.30 m/s</strong>
  </section>
  <section class="panel"><pre id="status">loading...</pre></section>
</main>
<script>
const state = {drive_command: "STOP", steering_command: "STRAIGHT", target_speed_mps: 0.30};
const statusEl = document.querySelector("#status");
const speed = document.querySelector("#speed");
const speedText = document.querySelector("#speedText");

async function post(path, body) {
  const res = await fetch(path, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body || {})});
  const data = await res.json();
  statusEl.textContent = JSON.stringify(data, null, 2);
}
function update() {
  state.target_speed_mps = Number(speed.value);
  speedText.textContent = state.target_speed_mps.toFixed(2) + " m/s";
  post("/api/control", state);
}
function setDrive(value) { state.drive_command = value; update(); }
function setSteer(value) { state.steering_command = value; update(); }
function stopAll() { state.drive_command = "STOP"; state.steering_command = "STRAIGHT"; post("/api/stop", state); }

speed.addEventListener("input", update);
document.querySelectorAll("[data-drive]").forEach(btn => {
  const v = btn.dataset.drive;
  btn.addEventListener("pointerdown", () => setDrive(v));
  btn.addEventListener("pointerup", () => setDrive("STOP"));
  btn.addEventListener("pointerleave", () => setDrive("STOP"));
});
document.querySelectorAll("[data-steer]").forEach(btn => {
  const v = btn.dataset.steer;
  btn.addEventListener("pointerdown", () => setSteer(v));
  btn.addEventListener("pointerup", () => setSteer("STRAIGHT"));
  btn.addEventListener("pointerleave", () => setSteer("STRAIGHT"));
});
document.querySelector("#stop").addEventListener("click", stopAll);
document.querySelector("#estop").addEventListener("click", () => post("/api/estop", {}));

const keys = new Set();
window.addEventListener("keydown", e => {
  if (e.repeat) return;
  keys.add(e.key.toLowerCase());
  if (e.code === "Space") { e.preventDefault(); stopAll(); return; }
  if (keys.has("w")) state.drive_command = "FORWARD";
  else if (keys.has("s")) state.drive_command = "REVERSE";
  if (keys.has("a")) state.steering_command = "LEFT";
  else if (keys.has("d")) state.steering_command = "RIGHT";
  update();
});
window.addEventListener("keyup", e => {
  keys.delete(e.key.toLowerCase());
  if (!keys.has("w") && !keys.has("s")) state.drive_command = "STOP";
  if (!keys.has("a") && !keys.has("d")) state.steering_command = "STRAIGHT";
  update();
});
setInterval(async () => {
  const res = await fetch("/api/status");
  statusEl.textContent = JSON.stringify(await res.json(), null, 2);
}, 500);
update();
</script>
</body>
</html>
"""


def create_app(controller: HpvcRemoteController):
    if Flask is None:
        raise RuntimeError("Flask is not installed. Install it with: python3 -m pip install flask")

    app = Flask(__name__)

    @app.get("/")
    def index() -> Response:
        return Response(INDEX_HTML, mimetype="text/html")

    @app.get("/api/status")
    def status():
        return jsonify(controller.snapshot())

    @app.post("/api/control")
    def control():
        payload = request.get_json(silent=True) or {}
        return jsonify(
            controller.command(
                drive_command=payload.get("drive_command"),
                steering_command=payload.get("steering_command"),
                target_speed_mps=payload.get("target_speed_mps"),
                emergency_stop=False,
            )
        )

    @app.post("/api/stop")
    def stop():
        return jsonify(controller.stop(clear_emergency=True))

    @app.post("/api/estop")
    def estop():
        return jsonify(controller.command(emergency_stop=True))

    @app.post("/api/reset")
    def reset():
        return jsonify(controller.stop(clear_emergency=True))

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--front-host", default=DEFAULT_FRONT_HOST)
    parser.add_argument("--front-port", type=int, default=DEFAULT_FRONT_PORT)
    parser.add_argument("--front-source-ip", default=DEFAULT_FRONT_SOURCE_IP)
    parser.add_argument("--front-source-port", type=int, default=DEFAULT_FRONT_SOURCE_PORT)
    parser.add_argument("--rear-host", default=DEFAULT_REAR_HOST)
    parser.add_argument("--rear-port", type=int, default=DEFAULT_REAR_PORT)
    parser.add_argument("--publish-hz", type=float, default=20.0)
    parser.add_argument("--steer-rad", type=float, default=0.25)
    parser.add_argument("--max-speed-mps", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    controller = HpvcRemoteController(
        front_host=args.front_host,
        front_port=args.front_port,
        front_source_ip=args.front_source_ip,
        front_source_port=args.front_source_port,
        rear_host=args.rear_host,
        rear_port=args.rear_port,
        publish_hz=args.publish_hz,
        steer_rad=args.steer_rad,
        max_speed_mps=args.max_speed_mps,
    )
    atexit.register(controller.close)
    app = create_app(controller)
    print(f"HPVC Flask remote control: http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
