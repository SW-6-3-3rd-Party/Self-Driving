"""Raspberry Pi #2 camera, side ultrasonic, UDP, and web preview service."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import time

import cv2
import numpy as np

YOLO_IMPORT_ERROR = None
try:
    from ultralytics import YOLO
except Exception as error:  # Optional runtime dependency on the Raspberry Pi.
    YOLO = None
    YOLO_IMPORT_ERROR = error


DEFAULT_LEFT_TRIGGER_GPIO = 23
DEFAULT_LEFT_ECHO_GPIO = 24
DEFAULT_RIGHT_TRIGGER_GPIO = 17
DEFAULT_RIGHT_ECHO_GPIO = 27

try:
    from .lane_detector import LaneDetectionResult, LaneDetector, LaneDetectorConfig
    from .protocol import (
        FLAG_CAMERA_VALID,
        FLAG_LANE_VALID,
        FLAG_LEFT_ULTRASONIC_VALID,
        FLAG_PERSON_DETECTION_VALID,
        FLAG_RIGHT_ULTRASONIC_VALID,
        MAX_PERSON_DETECTIONS,
        PACKET_SIZE,
        PAYLOAD_FIELDS,
        PerceptionPacket,
        PersonDetection,
        VERSION,
        pack_packet,
        payload_values,
    )
    from .ultrasonic import (
        DisabledUltrasonicPair,
        GpioUltrasonicPair,
        MockUltrasonicPair,
        UltrasonicSampler,
    )
except ImportError:  # Allow `python3 app.py` from this directory.
    from lane_detector import LaneDetectionResult, LaneDetector, LaneDetectorConfig
    from protocol import (
        FLAG_CAMERA_VALID,
        FLAG_LANE_VALID,
        FLAG_LEFT_ULTRASONIC_VALID,
        FLAG_PERSON_DETECTION_VALID,
        FLAG_RIGHT_ULTRASONIC_VALID,
        MAX_PERSON_DETECTIONS,
        PACKET_SIZE,
        PAYLOAD_FIELDS,
        PerceptionPacket,
        PersonDetection,
        VERSION,
        pack_packet,
        payload_values,
    )
    from ultrasonic import (
        DisabledUltrasonicPair,
        GpioUltrasonicPair,
        MockUltrasonicPair,
        UltrasonicSampler,
    )


class SharedOutput:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.metrics: dict = {"camera_valid": False, "lane_valid": False}
        self.sequence = 0

    def update(self, jpeg: bytes, metrics: dict, sequence: int) -> None:
        with self.lock:
            self.jpeg = jpeg
            self.metrics = metrics
            self.sequence = sequence

    def snapshot(self) -> tuple[bytes | None, dict, int]:
        with self.lock:
            return self.jpeg, dict(self.metrics), self.sequence


class SyntheticCapture:
    """Generate a moving trapezoidal RC lane without a physical track."""

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.started = time.monotonic()

    def read(self):
        elapsed = time.monotonic() - self.started
        frame = np.full((self.height, self.width, 3), 35, dtype=np.uint8)
        y = np.linspace(0.58, 0.96, 120)
        progress = (y - 0.58) / (0.96 - 0.58)
        lateral_shift = 0.035 * math.sin(elapsed * 0.55)
        curve = 0.025 * math.sin(elapsed * 0.23)
        curve_shape = curve * (1.0 - progress) ** 2
        left_x = 0.43 + (0.26 - 0.43) * progress + lateral_shift + curve_shape
        right_x = 0.57 + (0.74 - 0.57) * progress + lateral_shift + curve_shape
        left_points = np.column_stack((left_x * self.width, y * self.height)).astype(np.int32)
        right_points = np.column_stack((right_x * self.width, y * self.height)).astype(np.int32)
        road = np.vstack((left_points, np.flipud(right_points)))
        cv2.fillPoly(frame, [road], (60, 60, 60))
        cv2.polylines(frame, [left_points], False, (255, 255, 255), 10)
        cv2.polylines(frame, [right_points], False, (255, 255, 255), 10)
        cv2.putText(
            frame,
            "SYNTHETIC CAMERA - NOT PHYSICAL SENSOR DATA",
            (15, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 220, 255),
            2,
        )
        return True, frame

    def release(self):
        return


class PersonDetector:
    """YOLOv5 person detector that runs sparsely and reuses fresh results."""

    def __init__(self, args):
        self.enabled = not args.disable_person_detection
        self.detect_every = max(1, args.person_detect_every)
        self.confidence = args.person_conf
        self.imgsz = args.person_imgsz
        self.max_det = args.person_max_det
        self.stale_sec = args.person_stale_sec
        self.device = args.person_device
        self.model = None
        self.last_detections: tuple[PersonDetection, ...] = ()
        self.last_inference_time = 0.0
        self.last_error_time = 0.0

        if not self.enabled:
            return
        if YOLO is None:
            print(f"YOLO person detector disabled: ultralytics import failed ({YOLO_IMPORT_ERROR})")
            self.enabled = False
            return
        try:
            self.model = YOLO(args.person_model)
            print(
                "YOLO person detector enabled: "
                f"model={args.person_model}, every={self.detect_every} frames, imgsz={self.imgsz}"
            )
        except Exception as error:
            print(f"YOLO person detector disabled: failed to load {args.person_model}: {error}")
            self.enabled = False

    def update(self, frame, sequence: int) -> tuple[PersonDetection, ...]:
        now = time.monotonic()
        if not self.enabled or self.model is None:
            return ()
        if sequence % self.detect_every != 0:
            return self._fresh_or_empty(now)

        try:
            kwargs = {
                "source": frame,
                "imgsz": self.imgsz,
                "conf": self.confidence,
                "classes": [0],
                "max_det": self.max_det,
                "verbose": False,
            }
            if self.device:
                kwargs["device"] = self.device
            results = self.model.predict(**kwargs)
            self.last_detections = self._parse_results(results, frame.shape)
            self.last_inference_time = now
            return self.last_detections
        except Exception as error:
            if now - self.last_error_time > 2.0:
                print(f"YOLO person detection failed: {error}", flush=True)
                self.last_error_time = now
            return self._fresh_or_empty(now)

    def _fresh_or_empty(self, now: float) -> tuple[PersonDetection, ...]:
        if now - self.last_inference_time <= self.stale_sec:
            return self.last_detections
        return ()

    def _parse_results(self, results, frame_shape) -> tuple[PersonDetection, ...]:
        if not results:
            return ()
        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return ()

        height, width = frame_shape[:2]
        xyxy = _to_numpy(boxes.xyxy)
        confidences = _to_numpy(boxes.conf)
        classes = _to_numpy(boxes.cls)
        detections = []
        for bbox, confidence, class_id in zip(xyxy, confidences, classes):
            if int(class_id) != 0:
                continue
            x1, y1, x2, y2 = [float(value) for value in bbox]
            x1, x2 = sorted((max(0.0, min(width, x1)), max(0.0, min(width, x2))))
            y1, y2 = sorted((max(0.0, min(height, y1)), max(0.0, min(height, y2))))
            box_width = x2 - x1
            box_height = y2 - y1
            if box_width < 1.0 or box_height < 1.0:
                continue
            detections.append(
                PersonDetection(
                    valid=1.0,
                    confidence=float(confidence),
                    center_x_norm=((x1 + x2) * 0.5) / width,
                    center_y_norm=((y1 + y2) * 0.5) / height,
                    width_norm=box_width / width,
                    height_norm=box_height / height,
                )
            )

        detections.sort(key=lambda detection: detection.confidence, reverse=True)
        return tuple(detections[: self.max_det])


class PerceptionService:
    def __init__(self, args):
        self.args = args
        self.detector = LaneDetector(
            LaneDetectorConfig(
                lane_width_m=args.lane_width_m,
                visible_length_m=args.visible_length_m,
            ),
            args.calibration,
        )
        self.ultrasonic = UltrasonicSampler(build_ultrasonic_pair(args), args.ultrasonic_hz)
        self.person_detector = PersonDetector(args)
        self.shared = SharedOutput()
        self.running = threading.Event()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_address = (args.udp_host, args.udp_port) if args.udp_host else None
        self.udp_source_port = None
        if self.udp_address:
            try:
                self.udp_socket.bind(("", args.udp_source_port))
            except OSError as error:
                self.udp_socket.close()
                raise RuntimeError(
                    f"UDP source port {args.udp_source_port} is already in use. "
                    "Stop the previous app.py process, or run with --udp-source-port 0 "
                    "to let Linux choose a free source port."
                ) from error
            self.udp_source_port = self.udp_socket.getsockname()[1]
        self.capture = None
        self._closed = False

    def run(self) -> None:
        self.capture = open_camera(self.args)
        self.ultrasonic.start()
        self.running.set()
        sequence = 0
        period_s = 1.0 / max(self.args.processing_fps, 1.0)
        print(f"Camera source: {self.args.camera} ({self.args.width}x{self.args.height})")
        if self.udp_address:
            print(f"UDP destination: {self.udp_address[0]}:{self.udp_address[1]}")
            print(f"UDP source port: {self.udp_source_port}")

        try:
            while self.running.is_set():
                started = time.monotonic()
                ok, frame = self.capture.read()
                frame_timestamp_us = time.monotonic_ns() // 1000
                if not ok:
                    print("Camera frame acquisition failed", flush=True)
                    time.sleep(0.05)
                    continue

                frame = cv2.resize(frame, (self.args.width, self.args.height))
                result = self.detector.process(frame)
                persons = self.person_detector.update(frame, sequence)
                ultrasonic = self.ultrasonic.snapshot()
                flags = build_flags(result, ultrasonic, persons)
                packet = PerceptionPacket(
                    sequence=sequence,
                    frame_timestamp_us=frame_timestamp_us,
                    ultrasonic_timestamp_us=ultrasonic.timestamp_us,
                    flags=flags,
                    left=result.left,
                    right=result.right,
                    side_left_distance_m=ultrasonic.left_distance_m,
                    side_right_distance_m=ultrasonic.right_distance_m,
                    person_count=len(persons),
                    persons=persons,
                )
                packet_bytes = pack_packet(packet)
                packet_sent = False
                packet_error = None
                if self.udp_address:
                    try:
                        self.udp_socket.sendto(packet_bytes, self.udp_address)
                        packet_sent = True
                    except OSError as error:
                        packet_error = str(error)
                        print(f"UDP send failed: {packet_error}", flush=True)

                tx_payload = dict(zip(PAYLOAD_FIELDS, payload_values(packet)))
                metrics = {
                    "camera_valid": result.camera_valid,
                    "lane_valid": result.lane_valid,
                    "center_offset_m": result.center_offset_m,
                    "heading_error_rad": result.heading_error_rad,
                    "confidence": result.confidence,
                    "left_lane_strength": result.left.strength,
                    "right_lane_strength": result.right.strength,
                    "left_lateral_offset_m": result.left.lateral_offset_m,
                    "right_lateral_offset_m": result.right.lateral_offset_m,
                    "left_ultrasonic_m": finite_or_none(ultrasonic.left_distance_m),
                    "right_ultrasonic_m": finite_or_none(ultrasonic.right_distance_m),
                    "left_ultrasonic_valid": ultrasonic.left_valid,
                    "right_ultrasonic_valid": ultrasonic.right_valid,
                    "frame_timestamp_us": frame_timestamp_us,
                    "person_count": len(persons),
                    "persons": [person_to_dict(person) for person in persons],
                    "person_detection_enabled": self.person_detector.enabled,
                    "person_detect_every_frames": self.person_detector.detect_every,
                    "udp_enabled": self.udp_address is not None,
                    "udp_destination": (
                        f"{self.udp_address[0]}:{self.udp_address[1]}" if self.udp_address else None
                    ),
                    "udp_source_port": self.udp_source_port if self.udp_address else None,
                    "packet_sent": packet_sent,
                    "packet_error": packet_error,
                    "packet_size_bytes": len(packet_bytes),
                    "expected_packet_size_bytes": PACKET_SIZE,
                    "protocol_version": VERSION,
                    "payload_float_count": len(tx_payload),
                    "flags": flags,
                    "flags_decoded": decode_flags(flags),
                    "tx_payload": tx_payload,
                }
                draw_person_detections(result.preview, persons)
                ok_jpeg, encoded = cv2.imencode(
                    ".jpg", result.preview, [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality]
                )
                if ok_jpeg:
                    self.shared.update(encoded.tobytes(), metrics, sequence)

                sequence = (sequence + 1) & 0xFFFFFFFF
                remaining = period_s - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.running.clear()
        try:
            self.ultrasonic.close()
        except Exception as error:
            print(f"Ultrasonic cleanup failed: {error}", flush=True)
        try:
            self.udp_socket.close()
        except OSError:
            pass
        if self.capture is not None:
            self.capture.release()


def build_flags(result, ultrasonic, persons=()) -> int:
    flags = 0
    if result.camera_valid:
        flags |= FLAG_CAMERA_VALID
    if result.lane_valid:
        flags |= FLAG_LANE_VALID
    if ultrasonic.left_valid:
        flags |= FLAG_LEFT_ULTRASONIC_VALID
    if ultrasonic.right_valid:
        flags |= FLAG_RIGHT_ULTRASONIC_VALID
    if persons:
        flags |= FLAG_PERSON_DETECTION_VALID
    return flags


def draw_person_detections(preview, persons: tuple[PersonDetection, ...]) -> None:
    height, width = preview.shape[:2]
    for person in persons:
        if person.valid < 0.5:
            continue
        box_width = person.width_norm * width
        box_height = person.height_norm * height
        center_x = person.center_x_norm * width
        center_y = person.center_y_norm * height
        x1 = int(max(0, center_x - box_width * 0.5))
        y1 = int(max(0, center_y - box_height * 0.5))
        x2 = int(min(width - 1, center_x + box_width * 0.5))
        y2 = int(min(height - 1, center_y + box_height * 0.5))
        cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 220, 80), 2)
        cv2.putText(
            preview,
            f"person {person.confidence:.2f}",
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 220, 80),
            2,
        )


def person_to_dict(person: PersonDetection) -> dict:
    return {
        "valid": bool(person.valid >= 0.5),
        "confidence": person.confidence,
        "center_x_norm": person.center_x_norm,
        "center_y_norm": person.center_y_norm,
        "width_norm": person.width_norm,
        "height_norm": person.height_norm,
    }


def decode_flags(flags: int) -> dict:
    return {
        "camera_valid": bool(flags & FLAG_CAMERA_VALID),
        "lane_valid": bool(flags & FLAG_LANE_VALID),
        "left_ultrasonic_valid": bool(flags & FLAG_LEFT_ULTRASONIC_VALID),
        "right_ultrasonic_valid": bool(flags & FLAG_RIGHT_ULTRASONIC_VALID),
        "person_detection_valid": bool(flags & FLAG_PERSON_DETECTION_VALID),
    }


def _to_numpy(value):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def finite_or_none(value: float):
    return float(value) if math.isfinite(value) else None


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return value


def build_ultrasonic_pair(args):
    if args.disable_ultrasonic:
        return DisabledUltrasonicPair()
    if args.mock_ultrasonic is not None:
        return MockUltrasonicPair(*args.mock_ultrasonic)
    pins = (
        args.left_trigger,
        args.left_echo,
        args.right_trigger,
        args.right_echo,
    )
    if all(pin is not None for pin in pins):
        try:
            return GpioUltrasonicPair(*pins)
        except RuntimeError as error:
            if args.require_ultrasonic:
                raise
            print(
                f"Ultrasonic disabled after GPIO setup failure: {error}",
                flush=True,
            )
            return DisabledUltrasonicPair()
    if any(pin is not None for pin in pins):
        raise ValueError("Specify all four ultrasonic GPIO pins or none of them")
    return DisabledUltrasonicPair()


def open_camera(args):
    if args.synthetic_camera:
        return SyntheticCapture(args.width, args.height)
    source = int(args.camera) if str(args.camera).isdigit() else args.camera
    backend = cv2.CAP_V4L2 if os.name == "posix" else cv2.CAP_ANY
    capture = cv2.VideoCapture(source, backend)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.camera_fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"Could not open camera source: {args.camera}")
    return capture


def start_web_server(shared: SharedOutput, host: str, port: int):
    page = b"""<!doctype html><html><head><meta charset='utf-8'>
<title>RPi2 Perception TX Monitor</title>
<style>
:root{color-scheme:dark}
body{margin:0;background:#101214;color:#e9edf1;font-family:Arial,Helvetica,sans-serif}
header{display:flex;align-items:center;justify-content:space-between;padding:14px 18px;border-bottom:1px solid #2a3036;background:#15191d}
h1{margin:0;font-size:19px;font-weight:700}
main{display:grid;grid-template-columns:minmax(320px,1.1fr) minmax(360px,.9fr);gap:14px;padding:14px}
section{background:#171b20;border:1px solid #2a3036;border-radius:8px;overflow:hidden}
.panel{padding:12px}
img{display:block;width:100%;height:auto;background:#050607}
h2{margin:0 0 10px;font-size:15px;color:#bfc8d2}
.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}
.item{background:#111519;border:1px solid #283039;border-radius:6px;padding:8px}
.label{font-size:11px;color:#8995a1;text-transform:uppercase;letter-spacing:.04em}
.value{margin-top:4px;font-size:17px;font-weight:700;word-break:break-word}
.pill{display:inline-block;border-radius:999px;padding:5px 9px;font-size:12px;font-weight:700}
.ok{background:#133b27;color:#7ff0af}.bad{background:#4a1f21;color:#ff999e}.idle{background:#29313a;color:#c8d2dd}
table{width:100%;border-collapse:collapse;font-size:12px}
td,th{padding:6px 7px;border-bottom:1px solid #29313a;text-align:left}
th{color:#aeb8c3;font-weight:700;background:#12161a;position:sticky;top:0}
pre{max-height:260px;overflow:auto;margin:0;background:#0d1013;border:1px solid #283039;border-radius:6px;padding:10px;color:#cfd7df}
.stack{display:grid;gap:14px}
@media(max-width:900px){main{grid-template-columns:1fr}}
</style></head>
<body>
<header><h1>RPi #2 Perception TX Monitor</h1><div id='status'></div></header>
<main>
  <section><img src='/stream.mjpg'></section>
  <div class='stack'>
    <section class='panel'><h2>UDP TX</h2><div id='tx' class='grid'></div></section>
    <section class='panel'><h2>Lane / Sensors</h2><div id='sensor' class='grid'></div></section>
    <section class='panel'><h2>YOLOv5 Person Detection</h2><div id='people'></div></section>
    <section class='panel'><h2>Payload Sent</h2><table><thead><tr><th>Field</th><th>Value</th></tr></thead><tbody id='payload'></tbody></table></section>
    <section class='panel'><h2>Raw JSON</h2><pre id='raw'></pre></section>
  </div>
</main>
<script>
const fmt=(v,d=3)=>typeof v==='number'?(Number.isFinite(v)?v.toFixed(d):String(v)):(v??'--');
const item=(label,value)=>`<div class="item"><div class="label">${label}</div><div class="value">${value}</div></div>`;
function renderBool(v){return v?'<span class="pill ok">TRUE</span>':'<span class="pill bad">FALSE</span>'}
function renderStatus(m){
  const cls=m.packet_sent?'ok':(m.udp_enabled?'bad':'idle');
  const text=m.packet_sent?'SENT':(m.udp_enabled?'TX ERROR':'UDP OFF');
  document.getElementById('status').innerHTML=`<span class="pill ${cls}">${text}</span>`;
}
function render(m){
  renderStatus(m);
  document.getElementById('tx').innerHTML=[
    item('destination',m.udp_destination||'--'),
    item('source port',m.udp_source_port??'--'),
    item('packet sent',renderBool(!!m.packet_sent)),
    item('packet bytes',`${m.packet_size_bytes}/${m.expected_packet_size_bytes}`),
    item('protocol',`v${m.protocol_version}`),
    item('sequence',m.sequence),
    item('flags',m.flags),
    item('error',m.packet_error||'none')
  ].join('');
  document.getElementById('sensor').innerHTML=[
    item('camera valid',renderBool(!!m.camera_valid)),
    item('lane valid',renderBool(!!m.lane_valid)),
    item('center offset m',fmt(m.center_offset_m)),
    item('heading rad',fmt(m.heading_error_rad)),
    item('left ultrasonic m',fmt(m.left_ultrasonic_m)),
    item('right ultrasonic m',fmt(m.right_ultrasonic_m)),
    item('left lane strength',fmt(m.left_lane_strength)),
    item('right lane strength',fmt(m.right_lane_strength))
  ].join('');
  const people=m.persons||[];
  document.getElementById('people').innerHTML=[
    `<div class="grid">${[
      item('enabled',renderBool(!!m.person_detection_enabled)),
      item('detect every',`${m.person_detect_every_frames} frames`),
      item('count',m.person_count??0),
      item('flag valid',renderBool(!!(m.flags_decoded&&m.flags_decoded.person_detection_valid)))
    ].join('')}</div>`,
    people.length?`<table style="margin-top:10px"><thead><tr><th>#</th><th>conf</th><th>cx</th><th>cy</th><th>w</th><th>h</th></tr></thead><tbody>${
      people.map((p,i)=>`<tr><td>${i}</td><td>${fmt(p.confidence,2)}</td><td>${fmt(p.center_x_norm,3)}</td><td>${fmt(p.center_y_norm,3)}</td><td>${fmt(p.width_norm,3)}</td><td>${fmt(p.height_norm,3)}</td></tr>`).join('')
    }</tbody></table>`:'<div class="item" style="margin-top:10px"><div class="value">No person detected</div></div>'
  ].join('');
  const payload=m.tx_payload||{};
  document.getElementById('payload').innerHTML=Object.entries(payload)
    .map(([k,v])=>`<tr><td>${k}</td><td>${fmt(v,5)}</td></tr>`).join('');
  document.getElementById('raw').textContent=JSON.stringify(m,null,2);
}
async function refresh(){
  try{render(await (await fetch('/metrics.json',{cache:'no-store'})).json())}
  catch(err){document.getElementById('status').innerHTML='<span class="pill bad">NO DATA</span>'}
}
setInterval(refresh,500);refresh();
</script></body></html>"""

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(page)
            elif path == "/metrics.json":
                _, metrics, sequence = shared.snapshot()
                data = json.dumps(
                    json_safe({"sequence": sequence, **metrics}),
                    allow_nan=False,
                ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif path == "/stream.mjpg":
                self._stream()
            else:
                self.send_error(404)

        def _stream(self):
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_sequence = -1
            while True:
                jpeg, _, sequence = shared.snapshot()
                if jpeg is None or sequence == last_sequence:
                    time.sleep(0.02)
                    continue
                last_sequence = sequence
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break

        def log_message(self, _format, *_args):
            return

    server = ThreadingHTTPServer((host, port), Handler)
    thread = threading.Thread(target=server.serve_forever, name="web", daemon=True)
    thread.start()
    return server


def parse_args():
    parser = argparse.ArgumentParser(description="Raspberry Pi #2 perception ECU")
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument(
        "--synthetic-camera",
        action="store_true",
        help="Generate moving lane images without a physical camera or track",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--processing-fps", type=float, default=15.0)
    parser.add_argument(
        "--disable-person-detection",
        action="store_true",
        help="Disable YOLOv5 person detection and send zero person objects",
    )
    parser.add_argument("--person-model", default="yolov5nu.pt")
    parser.add_argument("--person-detect-every", type=int, default=10)
    parser.add_argument("--person-imgsz", type=int, default=160)
    parser.add_argument("--person-conf", type=float, default=0.35)
    parser.add_argument("--person-max-det", type=int, default=MAX_PERSON_DETECTIONS)
    parser.add_argument("--person-stale-sec", type=float, default=1.0)
    parser.add_argument("--person-device", default="cpu")
    parser.add_argument("--lane-width-m", type=float, default=0.40)
    parser.add_argument("--visible-length-m", type=float, default=2.0)
    parser.add_argument("--calibration", default=None)
    parser.add_argument("--udp-host", default=None, help="RPi #1 HPVC IP")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--udp-source-port", type=int, default=5006)
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8000)
    parser.add_argument("--jpeg-quality", type=int, default=60)
    parser.add_argument("--ultrasonic-hz", type=float, default=8.0)
    parser.add_argument("--left-trigger", type=int, default=DEFAULT_LEFT_TRIGGER_GPIO)
    parser.add_argument("--left-echo", type=int, default=DEFAULT_LEFT_ECHO_GPIO)
    parser.add_argument("--right-trigger", type=int, default=DEFAULT_RIGHT_TRIGGER_GPIO)
    parser.add_argument("--right-echo", type=int, default=DEFAULT_RIGHT_ECHO_GPIO)
    parser.add_argument(
        "--disable-ultrasonic",
        action="store_true",
        help="Disable both side ultrasonic sensors",
    )
    parser.add_argument(
        "--require-ultrasonic",
        action="store_true",
        help="Stop app startup if the side ultrasonic GPIO pins cannot be opened",
    )
    parser.add_argument(
        "--mock-ultrasonic",
        type=float,
        nargs=2,
        metavar=("LEFT_M", "RIGHT_M"),
        help="Use fixed side distances without GPIO hardware",
    )
    args = parser.parse_args()
    args.jpeg_quality = max(1, min(100, args.jpeg_quality))
    args.person_detect_every = max(1, args.person_detect_every)
    args.person_imgsz = max(64, args.person_imgsz)
    args.person_conf = max(0.01, min(0.99, args.person_conf))
    args.person_max_det = max(1, min(MAX_PERSON_DETECTIONS, args.person_max_det))
    args.person_stale_sec = max(0.0, args.person_stale_sec)
    return args


def main():
    args = parse_args()
    service = PerceptionService(args)
    web_server = start_web_server(service.shared, args.web_host, args.web_port)
    print(f"Web preview: http://<rpi2-ip>:{args.web_port}")
    stopping = False

    def stop(_signum=None, _frame=None):
        nonlocal stopping
        if stopping:
            raise KeyboardInterrupt
        stopping = True
        print("Stopping RPi2 perception service...", flush=True)
        service.running.clear()
        web_server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        service.run()
    finally:
        web_server.shutdown()
        web_server.server_close()


if __name__ == "__main__":
    main()
