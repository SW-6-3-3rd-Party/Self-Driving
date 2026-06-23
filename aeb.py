#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Advanced Low-speed AEB for Raspberry Pi 4
Front camera + front ToF + left/right front-diagonal ultrasonic

Core logic:
- YOLO camera object detection
- Simple object tracking
- Monocular x/z position estimation
- Object vx/vz velocity estimation
- Front ToF median filtering and longitudinal distance fusion
- Left/right diagonal ultrasonic near-field fusion
- Ego stopping distance calculation
- Required deceleration calculation
- Longitudinal TTC calculation
- Crossing / cut-in time-to-path prediction
- Unknown close obstacle fallback from ToF/ultrasonic
- FCW / Brake Prefill / Partial Brake / Full Brake / Stop Hold state machine
- CSV logging
- Optional CAN brake request

SAFETY:
- Default DRY_RUN = True, so actuator output is disabled.
- Do not connect this directly to a real vehicle brake system.
- Use only on RC car / low-speed test bench.
- HC-SR04 Echo is 5V. Raspberry Pi GPIO is 3.3V only.
  Use a resistor divider or level shifter on every Echo pin.
"""

import csv
import math
import signal
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

# ============================================================
# User Config
# ============================================================

DRY_RUN = True

# BCM GPIO numbering. Ultrasonic sensors are mounted front-left and
# front-right, aimed diagonally toward the front path.
ULTRASONIC_PINS = {
    "LEFT_DIAGONAL": {"TRIG": 23, "ECHO": 24},
    "RIGHT_DIAGONAL": {"TRIG": 5, "ECHO": 6},
}

LEFT_DIAGONAL_NAME = "LEFT_DIAGONAL"
RIGHT_DIAGONAL_NAME = "RIGHT_DIAGONAL"

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SHOW_PREVIEW = True

# If distance estimation is inaccurate, tune these first.
CAMERA_HFOV_DEG = 62.0
CAMERA_VFOV_DEG = 48.0

USE_YOLO = True
YOLO_MODEL_PATH = "yolov8n.pt"
YOLO_CONF = 0.45
YOLO_IMGSZ = 320
YOLO_EVERY_N_FRAMES = 1  # Raise to 2 or 3 if Raspberry Pi 4 is too slow.

# Ego vehicle inputs. Replace get_ego_speed_mps() and get_steering_angle_deg()
# if you have encoder / CAN / servo command signals.
DEFAULT_EGO_SPEED_MPS = 0.7
DEFAULT_STEERING_ANGLE_DEG = 0.0

# RC car / test bench geometry.
VEHICLE_WIDTH_M = 0.32
PATH_MARGIN_M = 0.12
STEERING_PATH_GAIN = 0.65

# Brake performance. Must be tuned with actual stop tests.
AVAILABLE_DECEL_MPS2 = 3.0
REACTION_TIME_SEC = 0.25
SAFETY_MARGIN_M = 0.25
MAX_AEB_RANGE_M = 6.0

# Ultrasonic settings.
ULTRA_MIN_CM = 3
ULTRA_MAX_CM = 500
ULTRA_MEDIAN_SIZE = 5
ULTRA_TIMEOUT_SEC = 0.030
ULTRA_CROSSTALK_DELAY_SEC = 0.015
ULTRA_FUSION_MAX_M = 3.5
ULTRA_DIAGONAL_ANGLE_DEG = 35.0
ULTRA_STALE_SEC = 0.50
DIAGONAL_FCW_DISTANCE_M = 0.85
DIAGONAL_PARTIAL_BRAKE_DISTANCE_M = 0.55
DIAGONAL_FULL_BRAKE_DISTANCE_M = 0.35

# Front ToF settings. Default target is a common VL53L0X module; set
# TOF_SENSOR_MODEL to "VL53L1X" if your hardware uses that driver.
USE_TOF = True
TOF_SENSOR_MODEL = "AUTO"  # "AUTO", "VL53L0X", or "VL53L1X"
TOF_I2C_ADDRESS = 0x29
TOF_MIN_M = 0.03
TOF_MAX_M = 4.00
TOF_MEDIAN_SIZE = 7
TOF_FUSION_MAX_M = 4.00
TOF_STALE_SEC = 0.40
TOF_CENTER_GATE_RATIO = 0.36

UNKNOWN_STOP_DISTANCE_M = 0.50
STOP_HOLD_DISTANCE_M = 0.40
STOP_HOLD_SEC = 1.0

# Tracking / kinematics.
MIN_OBJECT_CONF = 0.55
MIN_TRACK_AGE_FOR_AEB = 3
TRACK_MATCH_DISTANCE_PX = 90
TRACK_MAX_MISSED_SEC = 0.7
POS_FILTER_ALPHA = 0.55
VEL_FILTER_ALPHA = 0.35

# Advanced threat thresholds.
FCW_TTC_SEC = 2.5
PARTIAL_TTC_SEC = 1.5
FULL_TTC_SEC = 0.8

FCW_REQ_DECEL = 1.2
PREFILL_REQ_DECEL = 1.8
PARTIAL_REQ_DECEL = 2.5
FULL_REQ_DECEL = 4.0

CROSSING_HORIZON_SEC = 3.0
CROSSING_FCW_TIME_GAP_SEC = 1.2
CROSSING_PARTIAL_TIME_GAP_SEC = 0.75
CROSSING_FULL_TIME_GAP_SEC = 0.45

CUTIN_HORIZON_SEC = 3.0
CUTIN_FCW_TIME_GAP_SEC = 1.0
CUTIN_PARTIAL_TIME_GAP_SEC = 0.65
CUTIN_FULL_TIME_GAP_SEC = 0.40

CLEAR_CONFIRM_COUNT = 5
LOOP_HZ = 10

# CAN output option.
USE_CAN = False
CAN_CHANNEL = "can0"
CAN_ID_AEB_BRAKE_REQUEST = 0x310

LOG_CSV_PATH = "advanced_aeb_log.csv"

# ============================================================
# Optional Imports
# ============================================================

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except Exception as e:
    GPIO_AVAILABLE = False
    print("[WARN] RPi.GPIO import failed:", e)

try:
    import cv2
    CV2_AVAILABLE = True
except Exception as e:
    CV2_AVAILABLE = False
    print("[WARN] OpenCV import failed:", e)

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception as e:
    YOLO_AVAILABLE = False
    print("[WARN] ultralytics import failed:", e)

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False

try:
    import board
    import busio
    I2C_AVAILABLE = True
except Exception as e:
    I2C_AVAILABLE = False
    print("[WARN] board/busio import failed:", e)

try:
    import adafruit_vl53l0x
    VL53L0X_AVAILABLE = True
except Exception as e:
    VL53L0X_AVAILABLE = False
    print("[WARN] adafruit_vl53l0x import failed:", e)

try:
    import adafruit_vl53l1x
    VL53L1X_AVAILABLE = True
except Exception as e:
    VL53L1X_AVAILABLE = False
    print("[WARN] adafruit_vl53l1x import failed:", e)

try:
    import can
    CAN_AVAILABLE = True
except Exception:
    CAN_AVAILABLE = False

# ============================================================
# Data Models
# ============================================================

class AEBState(Enum):
    OFF = 0
    STANDBY = 1
    MONITORING = 2
    FCW = 3
    BRAKE_PREFILL = 4
    PARTIAL_BRAKE = 5
    FULL_BRAKE = 6
    STOP_HOLD = 7
    RELEASE = 8
    DEGRADED = 9
    SENSOR_FAULT = 10


class ThreatType(Enum):
    NONE = 0
    LEAD_OBJECT = 1
    STATIONARY_OBSTACLE = 2
    CROSSING_VRU = 3
    CUT_IN = 4
    UNKNOWN_CLOSE = 5
    SENSOR_FAULT = 6


@dataclass
class UltrasonicReading:
    name: str
    raw_cm: Optional[float]
    filtered_cm: Optional[float]
    distance_m: Optional[float]
    forward_distance_m: Optional[float]
    approach_speed_mps: float
    ttc_sec: float
    valid: bool
    confidence: float
    timestamp: float


@dataclass
class ToFReading:
    name: str
    raw_mm: Optional[float]
    filtered_mm: Optional[float]
    distance_m: Optional[float]
    approach_speed_mps: float
    ttc_sec: float
    valid: bool
    confidence: float
    timestamp: float


@dataclass
class RawDetection:
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]
    center_x: float
    center_y: float
    bbox_w: float
    bbox_h: float
    area: float


@dataclass
class ObjectKinematics:
    track_id: int
    label: str
    confidence: float
    track_age: int
    bbox: Tuple[int, int, int, int]

    # Ego coordinate system:
    # x_m: right positive, left negative.
    # z_m: forward positive.
    x_m: float
    z_m: float
    vx_mps: float
    vz_mps: float

    width_m: float
    height_m: float
    in_path_now: bool
    time_seen: float

    selected_ultra_name: Optional[str]
    ultra_distance_m: Optional[float]
    ultra_approach_speed_mps: float
    tof_distance_m: Optional[float]
    tof_approach_speed_mps: float
    z_source: str


@dataclass
class AdvancedRisk:
    threat_type: ThreatType
    desired_state: AEBState
    brake_percent: int
    reason: str
    obj: Optional[ObjectKinematics]

    distance_m: Optional[float]
    ttc_long_sec: float
    required_decel_mps2: float
    stopping_distance_m: float
    time_to_path_sec: float
    ego_time_to_conflict_sec: float
    time_gap_sec: float
    collision_probability: float
    confidence_percent: int

# ============================================================
# Utility
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def low_pass(prev, new, alpha):
    if prev is None:
        return new
    return alpha * new + (1.0 - alpha) * prev


def fmt(value, digits=2):
    if value is None:
        return "None"
    if isinstance(value, float):
        if value > 900:
            return "INF"
        return f"{value:.{digits}f}"
    return str(value)


def calc_crc_xor(data):
    crc = 0
    for b in data:
        crc ^= int(b) & 0xFF
    return crc & 0xFF


def distance_confidence(distance_m, min_range_m, max_range_m):
    if distance_m is None:
        return 0.0
    usable = max(0.01, max_range_m - min_range_m)
    normalized = clamp((distance_m - min_range_m) / usable, 0.0, 1.0)
    return clamp(1.0 - normalized * 0.45, 0.15, 1.0)


def ultrasonic_forward_component(name, distance_m):
    if distance_m is None:
        return None
    if name in {LEFT_DIAGONAL_NAME, RIGHT_DIAGONAL_NAME}:
        return distance_m * math.cos(math.radians(ULTRA_DIAGONAL_ANGLE_DEG))
    return distance_m


def ultrasonic_forward_approach_component(name, approach_speed_mps):
    if name in {LEFT_DIAGONAL_NAME, RIGHT_DIAGONAL_NAME}:
        return approach_speed_mps * math.cos(math.radians(ULTRA_DIAGONAL_ANGLE_DEG))
    return approach_speed_mps


def focal_from_fov(size_px, fov_deg):
    return size_px / (2.0 * math.tan(math.radians(fov_deg) / 2.0))


FOCAL_X = focal_from_fov(FRAME_WIDTH, CAMERA_HFOV_DEG)
FOCAL_Y = focal_from_fov(FRAME_HEIGHT, CAMERA_VFOV_DEG)
IMAGE_CX = FRAME_WIDTH / 2.0
IMAGE_CY = FRAME_HEIGHT / 2.0

# Approximate COCO object dimensions. Tune for your test targets.
REAL_OBJECT_SIZE = {
    "person": {"height": 1.70, "width": 0.45},
    "bicycle": {"height": 1.35, "width": 1.60},
    "car": {"height": 1.50, "width": 1.80},
    "motorcycle": {"height": 1.30, "width": 0.80},
    "bus": {"height": 3.00, "width": 2.50},
    "truck": {"height": 3.00, "width": 2.50},
    "dog": {"height": 0.55, "width": 0.60},
    "cat": {"height": 0.30, "width": 0.40},
    "chair": {"height": 0.85, "width": 0.50},
    "bench": {"height": 0.80, "width": 1.20},
    "backpack": {"height": 0.50, "width": 0.35},
    "suitcase": {"height": 0.60, "width": 0.40},
    "sports ball": {"height": 0.22, "width": 0.22},
    "unknown": {"height": 0.60, "width": 0.50},
}


def object_type_code(label):
    if label is None:
        return 0
    if label == "person":
        return 2
    if label in {"bicycle", "motorcycle"}:
        return 3
    if label in {"car", "bus", "truck"}:
        return 1
    return 4


def is_vru(label):
    return label in {"person", "bicycle"}


def is_vehicle(label):
    return label in {"car", "bus", "truck", "motorcycle"}

# ============================================================
# Ego Vehicle Signals
# ============================================================

def get_ego_speed_mps():
    """
    Replace this with encoder / CAN speed / motor command based speed.
    """
    return DEFAULT_EGO_SPEED_MPS


def get_steering_angle_deg():
    """
    Replace this with steering angle sensor or servo command.
    Positive = right steering, negative = left steering.
    """
    return DEFAULT_STEERING_ANGLE_DEG


def get_driver_override():
    """
    Return True when manual override / emergency stop / driver braking is active.
    """
    return False

# ============================================================
# Ultrasonic Sensors
# ============================================================

class UltrasonicSensor:
    def __init__(self, name, trig_pin, echo_pin):
        self.name = name
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.values = deque(maxlen=ULTRA_MEDIAN_SIZE)
        self.prev_distance_m = None
        self.prev_time = None
        self.last_valid_time = None

        if GPIO_AVAILABLE:
            GPIO.setup(self.trig_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trig_pin, False)
            time.sleep(0.05)

    def read_raw_cm(self):
        if not GPIO_AVAILABLE:
            return None

        GPIO.output(self.trig_pin, False)
        time.sleep(0.000002)

        GPIO.output(self.trig_pin, True)
        time.sleep(0.000010)
        GPIO.output(self.trig_pin, False)

        wait_start = time.perf_counter()
        while GPIO.input(self.echo_pin) == 0:
            if time.perf_counter() - wait_start > ULTRA_TIMEOUT_SEC:
                return None

        pulse_start = time.perf_counter()
        while GPIO.input(self.echo_pin) == 1:
            if time.perf_counter() - pulse_start > ULTRA_TIMEOUT_SEC:
                return None

        pulse_end = time.perf_counter()
        pulse_duration = pulse_end - pulse_start
        distance_cm = pulse_duration * 34300.0 / 2.0

        if distance_cm < ULTRA_MIN_CM or distance_cm > ULTRA_MAX_CM:
            return None
        return distance_cm

    def update(self):
        now = time.perf_counter()
        raw_cm = self.read_raw_cm()

        if raw_cm is not None:
            self.values.append(raw_cm)
            self.last_valid_time = now

        if not self.values:
            return UltrasonicReading(
                name=self.name,
                raw_cm=raw_cm,
                filtered_cm=None,
                distance_m=None,
                forward_distance_m=None,
                approach_speed_mps=0.0,
                ttc_sec=999.0,
                valid=False,
                confidence=0.0,
                timestamp=now,
            )

        sorted_values = sorted(self.values)
        filtered_cm = sorted_values[len(sorted_values) // 2]
        distance_m = filtered_cm / 100.0
        forward_distance_m = ultrasonic_forward_component(self.name, distance_m)
        is_fresh = self.last_valid_time is not None and now - self.last_valid_time <= ULTRA_STALE_SEC

        approach_speed = 0.0
        ttc = 999.0

        if is_fresh and self.prev_distance_m is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                # Distance decreases -> object is approaching.
                approach_speed = max(0.0, (self.prev_distance_m - distance_m) / dt)
                if approach_speed > 0.03:
                    ttc = distance_m / approach_speed

        if is_fresh:
            self.prev_distance_m = distance_m
            self.prev_time = now

        return UltrasonicReading(
            name=self.name,
            raw_cm=raw_cm,
            filtered_cm=filtered_cm,
            distance_m=distance_m,
            forward_distance_m=forward_distance_m,
            approach_speed_mps=approach_speed,
            ttc_sec=ttc,
            valid=is_fresh,
            confidence=distance_confidence(distance_m, ULTRA_MIN_CM / 100.0, ULTRA_FUSION_MAX_M) if is_fresh else 0.0,
            timestamp=now,
        )


class UltrasonicManager:
    def __init__(self):
        self.sensors = {}

        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

        for name, pins in ULTRASONIC_PINS.items():
            self.sensors[name] = UltrasonicSensor(
                name=name,
                trig_pin=pins["TRIG"],
                echo_pin=pins["ECHO"],
            )

    def update_all(self):
        readings = {}
        for name, sensor in self.sensors.items():
            readings[name] = sensor.update()
            time.sleep(ULTRA_CROSSTALK_DELAY_SEC)
        return readings

# ============================================================
# Front ToF Sensor
# ============================================================

class ToFSensor:
    def __init__(self):
        self.name = "FRONT_TOF"
        self.values = deque(maxlen=TOF_MEDIAN_SIZE)
        self.prev_distance_m = None
        self.prev_time = None
        self.last_valid_time = None
        self.sensor = None
        self.sensor_model = None
        self.i2c = None

        if not USE_TOF:
            print("[INFO] Front ToF disabled by config.")
            return

        if not I2C_AVAILABLE:
            print("[WARN] I2C unavailable. Front ToF disabled.")
            return

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            print("[WARN] I2C init failed. Front ToF disabled:", e)
            self.i2c = None
            return

        requested = TOF_SENSOR_MODEL.upper()

        if requested in {"AUTO", "VL53L0X"} and VL53L0X_AVAILABLE:
            try:
                self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=TOF_I2C_ADDRESS)
                self.sensor_model = "VL53L0X"
                print("[INFO] Front ToF loaded: VL53L0X")
                return
            except Exception as e:
                print("[WARN] VL53L0X init failed:", e)

        if requested in {"AUTO", "VL53L1X"} and VL53L1X_AVAILABLE:
            try:
                self.sensor = adafruit_vl53l1x.VL53L1X(self.i2c, address=TOF_I2C_ADDRESS)
                self.sensor_model = "VL53L1X"
                try:
                    self.sensor.distance_mode = 1
                    self.sensor.timing_budget = 50
                    self.sensor.start_ranging()
                except Exception:
                    pass
                print("[INFO] Front ToF loaded: VL53L1X")
                return
            except Exception as e:
                print("[WARN] VL53L1X init failed:", e)

        print("[WARN] No supported front ToF driver available.")

    def read_raw_mm(self):
        if self.sensor is None:
            return None

        try:
            if self.sensor_model == "VL53L0X":
                mm = float(self.sensor.range)
            elif self.sensor_model == "VL53L1X":
                try:
                    data_ready = self.sensor.data_ready
                except Exception:
                    data_ready = True
                if not data_ready:
                    return None
                # Adafruit VL53L1X exposes distance in centimeters.
                mm = float(self.sensor.distance) * 10.0
                try:
                    self.sensor.clear_interrupt()
                except Exception:
                    pass
            else:
                return None
        except Exception as e:
            print("[WARN] Front ToF read failed:", e)
            return None

        distance_m = mm / 1000.0
        if distance_m < TOF_MIN_M or distance_m > TOF_MAX_M:
            return None
        return mm

    def update(self):
        now = time.perf_counter()
        raw_mm = self.read_raw_mm()

        if raw_mm is not None:
            self.values.append(raw_mm)
            self.last_valid_time = now

        if not self.values:
            return ToFReading(
                name=self.name,
                raw_mm=raw_mm,
                filtered_mm=None,
                distance_m=None,
                approach_speed_mps=0.0,
                ttc_sec=999.0,
                valid=False,
                confidence=0.0,
                timestamp=now,
            )

        sorted_values = sorted(self.values)
        filtered_mm = sorted_values[len(sorted_values) // 2]
        distance_m = filtered_mm / 1000.0
        is_fresh = self.last_valid_time is not None and now - self.last_valid_time <= TOF_STALE_SEC

        approach_speed = 0.0
        ttc = 999.0

        if is_fresh and self.prev_distance_m is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                approach_speed = max(0.0, (self.prev_distance_m - distance_m) / dt)
                if approach_speed > 0.03:
                    ttc = distance_m / approach_speed

        if is_fresh:
            self.prev_distance_m = distance_m
            self.prev_time = now

        return ToFReading(
            name=self.name,
            raw_mm=raw_mm,
            filtered_mm=filtered_mm,
            distance_m=distance_m,
            approach_speed_mps=approach_speed,
            ttc_sec=ttc,
            valid=is_fresh,
            confidence=distance_confidence(distance_m, TOF_MIN_M, TOF_MAX_M) if is_fresh else 0.0,
            timestamp=now,
        )

    def release(self):
        if self.sensor_model == "VL53L1X" and self.sensor is not None:
            try:
                self.sensor.stop_ranging()
            except Exception:
                pass


# ============================================================
# Camera + YOLO
# ============================================================

class CameraManager:
    def __init__(self):
        self.use_picamera2 = False
        self.cap = None
        self.picam2 = None

        if not CV2_AVAILABLE:
            print("[WARN] OpenCV unavailable. Camera disabled.")
            return

        if PICAMERA2_AVAILABLE:
            try:
                self.picam2 = Picamera2()
                config = self.picam2.create_preview_configuration(
                    main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
                )
                self.picam2.configure(config)
                self.picam2.start()
                self.use_picamera2 = True
                print("[INFO] Camera backend: Picamera2")
                time.sleep(1.0)
                return
            except Exception as e:
                print("[WARN] Picamera2 init failed:", e)
                self.picam2 = None

        try:
            self.cap = cv2.VideoCapture(CAMERA_INDEX)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

            if self.cap.isOpened():
                print("[INFO] Camera backend: OpenCV VideoCapture")
            else:
                print("[WARN] Camera open failed.")
                self.cap = None
        except Exception as e:
            print("[WARN] Camera init failed:", e)
            self.cap = None

    def read(self):
        if not CV2_AVAILABLE:
            return None

        if self.use_picamera2 and self.picam2 is not None:
            frame = self.picam2.capture_array()
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                return frame
        return None

    def release(self):
        if self.cap is not None:
            self.cap.release()
        if self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass


class ObjectDetector:
    def __init__(self):
        self.model = None
        self.enabled = False
        self.target_labels = {
            "person",
            "bicycle",
            "car",
            "motorcycle",
            "bus",
            "truck",
            "dog",
            "cat",
            "chair",
            "bench",
            "backpack",
            "suitcase",
            "sports ball",
        }

        if USE_YOLO and YOLO_AVAILABLE:
            try:
                self.model = YOLO(YOLO_MODEL_PATH)
                self.enabled = True
                print("[INFO] YOLO loaded:", YOLO_MODEL_PATH)
            except Exception as e:
                print("[WARN] YOLO load failed:", e)
                self.enabled = False

    def detect(self, frame):
        detections = []
        if frame is None or not self.enabled:
            return detections

        results = self.model.predict(
            source=frame,
            imgsz=YOLO_IMGSZ,
            conf=YOLO_CONF,
            verbose=False,
        )

        if not results:
            return detections

        result = results[0]
        names = result.names

        if result.boxes is None:
            return detections

        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            label = names.get(cls_id, str(cls_id))

            if label not in self.target_labels:
                continue
            if conf < MIN_OBJECT_CONF:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            w = max(1, x2 - x1)
            h = max(1, y2 - y1)
            cx = x1 + w / 2.0
            cy = y1 + h / 2.0

            detections.append(
                RawDetection(
                    label=label,
                    confidence=conf,
                    bbox=(x1, y1, x2, y2),
                    center_x=cx,
                    center_y=cy,
                    bbox_w=w,
                    bbox_h=h,
                    area=w * h,
                )
            )
        return detections

# ============================================================
# Kinematics Estimation
# ============================================================

def estimate_z_from_bbox(label, bbox_h):
    size = REAL_OBJECT_SIZE.get(label, REAL_OBJECT_SIZE["unknown"])
    real_h = size["height"]
    if bbox_h <= 1:
        return None
    z = FOCAL_Y * real_h / bbox_h
    if z <= 0 or z > 30:
        return None
    return z


def estimate_x_from_pixel(center_x, z_m):
    return (center_x - IMAGE_CX) * z_m / FOCAL_X


def estimate_width_from_bbox(bbox_w, z_m):
    return max(0.05, bbox_w * z_m / FOCAL_X)


def is_center_tof_gate(center_x):
    gate_half_width = FRAME_WIDTH * TOF_CENTER_GATE_RATIO / 2.0
    return abs(center_x - IMAGE_CX) <= gate_half_width


def select_ultra_name_by_pixel(center_x):
    if center_x < FRAME_WIDTH * 0.42:
        return LEFT_DIAGONAL_NAME
    if center_x > FRAME_WIDTH * 0.58:
        return RIGHT_DIAGONAL_NAME
    return None


def path_center_x_at_z(z_m, steering_angle_deg):
    # Positive steering angle means path bends to the right.
    steer_rad = math.radians(steering_angle_deg)
    return math.tan(steer_rad) * z_m * STEERING_PATH_GAIN


def path_half_width():
    return VEHICLE_WIDTH_M / 2.0 + PATH_MARGIN_M


def is_in_predicted_path(x_m, z_m, obj_width_m, steering_angle_deg):
    center = path_center_x_at_z(z_m, steering_angle_deg)
    bound = path_half_width() + obj_width_m / 2.0
    return abs(x_m - center) <= bound


class ObjectKinematicsTracker:
    def __init__(self):
        self.next_id = 1
        self.tracks = {}

    def _match_track(self, det):
        best_id = None
        best_dist = 1e9
        now = time.perf_counter()

        for track_id, tr in self.tracks.items():
            if now - tr["last_seen"] > TRACK_MAX_MISSED_SEC:
                continue
            if tr["label"] != det.label:
                continue

            dx = det.center_x - tr["center_x_px"]
            dy = det.center_y - tr["center_y_px"]
            dist = math.hypot(dx, dy)

            if dist < TRACK_MATCH_DISTANCE_PX and dist < best_dist:
                best_dist = dist
                best_id = track_id

        return best_id

    def _fuse_z_with_range(self, det, z_cam, ultrasonic_readings, tof_reading):
        ultra_name = select_ultra_name_by_pixel(det.center_x)
        ultra = ultrasonic_readings.get(ultra_name) if ultra_name else None

        ultra_forward_z = None
        ultra_distance = None
        ultra_approach = 0.0
        tof_z = None
        tof_approach = 0.0

        if ultra is not None and ultra.valid and ultra.distance_m is not None:
            ultra_forward_z = ultra.forward_distance_m
            ultra_distance = ultra.distance_m
            ultra_approach = ultrasonic_forward_approach_component(
                ultra.name,
                ultra.approach_speed_mps,
            )

        if tof_reading is not None and tof_reading.valid and tof_reading.distance_m is not None:
            tof_z = tof_reading.distance_m
            tof_approach = tof_reading.approach_speed_mps

        center_gate = is_center_tof_gate(det.center_x)

        if tof_z is not None and tof_z <= TOF_FUSION_MAX_M and (center_gate or z_cam is None):
            if z_cam is not None:
                diff = abs(z_cam - tof_z)
                allowed_diff = max(0.55, tof_z * 0.35)

                if diff <= allowed_diff:
                    z = 0.85 * tof_z + 0.15 * z_cam
                    return z, ultra_name, ultra_distance, ultra_approach, tof_z, tof_approach, "tof_fused"

                # If a central object and the front range sensor disagree at close
                # range, trust ToF for the longitudinal safety envelope.
                if center_gate and tof_z <= 2.5:
                    return tof_z, ultra_name, ultra_distance, ultra_approach, tof_z, tof_approach, "tof"

            return tof_z, ultra_name, ultra_distance, ultra_approach, tof_z, tof_approach, "tof"

        if ultra_forward_z is not None and ultra_forward_z <= ULTRA_FUSION_MAX_M:
            if z_cam is not None:
                diff = abs(z_cam - ultra_forward_z)
                allowed_diff = max(0.70, ultra_forward_z * 0.50)

                if diff <= allowed_diff:
                    z = 0.65 * ultra_forward_z + 0.35 * z_cam
                    return z, ultra_name, ultra_distance, ultra_approach, tof_z, tof_approach, "diag_ultra_fused"

                if ultra_forward_z <= 1.5:
                    return ultra_forward_z, ultra_name, ultra_distance, ultra_approach, tof_z, tof_approach, "diag_ultra"

            return ultra_forward_z, ultra_name, ultra_distance, ultra_approach, tof_z, tof_approach, "diag_ultra"

        if z_cam is not None:
            return z_cam, ultra_name, ultra_distance, ultra_approach, tof_z, tof_approach, "camera"

        return None, ultra_name, ultra_distance, ultra_approach, tof_z, tof_approach, "invalid"

    def update(self, detections, ultrasonic_readings, tof_reading, steering_angle_deg):
        now = time.perf_counter()
        objects = []

        for det in detections:
            z_cam = estimate_z_from_bbox(det.label, det.bbox_h)
            (
                z_m,
                ultra_name,
                ultra_distance_m,
                ultra_approach,
                tof_distance_m,
                tof_approach,
                z_source,
            ) = self._fuse_z_with_range(
                det, z_cam, ultrasonic_readings, tof_reading
            )

            if z_m is None:
                continue

            z_m = clamp(z_m, 0.05, 30.0)
            x_m = estimate_x_from_pixel(det.center_x, z_m)
            width_m = estimate_width_from_bbox(det.bbox_w, z_m)
            real_size = REAL_OBJECT_SIZE.get(det.label, REAL_OBJECT_SIZE["unknown"])
            height_m = real_size["height"]

            track_id = self._match_track(det)

            if track_id is None:
                track_id = self.next_id
                self.next_id += 1

                age = 1
                vx = 0.0
                vz = 0.0
                filt_x = x_m
                filt_z = z_m

            else:
                tr = self.tracks[track_id]
                age = tr["age"] + 1
                dt = max(0.001, now - tr["last_seen"])

                filt_x = low_pass(tr["x_m"], x_m, POS_FILTER_ALPHA)
                filt_z = low_pass(tr["z_m"], z_m, POS_FILTER_ALPHA)

                raw_vx = (filt_x - tr["x_m"]) / dt
                raw_vz = (filt_z - tr["z_m"]) / dt

                vx = low_pass(tr["vx_mps"], raw_vx, VEL_FILTER_ALPHA)
                vz = low_pass(tr["vz_mps"], raw_vz, VEL_FILTER_ALPHA)

            in_path_now = is_in_predicted_path(
                x_m=filt_x,
                z_m=filt_z,
                obj_width_m=width_m,
                steering_angle_deg=steering_angle_deg,
            )

            self.tracks[track_id] = {
                "label": det.label,
                "center_x_px": det.center_x,
                "center_y_px": det.center_y,
                "x_m": filt_x,
                "z_m": filt_z,
                "vx_mps": vx,
                "vz_mps": vz,
                "age": age,
                "last_seen": now,
            }

            objects.append(
                ObjectKinematics(
                    track_id=track_id,
                    label=det.label,
                    confidence=det.confidence,
                    track_age=age,
                    bbox=det.bbox,
                    x_m=filt_x,
                    z_m=filt_z,
                    vx_mps=vx,
                    vz_mps=vz,
                    width_m=width_m,
                    height_m=height_m,
                    in_path_now=in_path_now,
                    time_seen=now,
                    selected_ultra_name=ultra_name,
                    ultra_distance_m=ultra_distance_m,
                    ultra_approach_speed_mps=ultra_approach,
                    tof_distance_m=tof_distance_m,
                    tof_approach_speed_mps=tof_approach,
                    z_source=z_source,
                )
            )

        stale_ids = []
        for track_id, tr in self.tracks.items():
            if now - tr["last_seen"] > TRACK_MAX_MISSED_SEC:
                stale_ids.append(track_id)
        for track_id in stale_ids:
            del self.tracks[track_id]

        return objects

# ============================================================
# Advanced Threat Assessment
# ============================================================

class AdvancedThreatAssessment:
    def __init__(self):
        self.available_decel = AVAILABLE_DECEL_MPS2
        self.reaction_time = REACTION_TIME_SEC
        self.safety_margin = SAFETY_MARGIN_M

    def stopping_distance(self, v_ego):
        return (
            v_ego * self.reaction_time
            + (v_ego ** 2) / (2.0 * max(0.01, self.available_decel))
            + self.safety_margin
        )

    def required_decel(self, distance_m, closing_speed_mps):
        remain = max(0.01, distance_m - self.safety_margin)
        if closing_speed_mps <= 0:
            return 0.0
        return (closing_speed_mps ** 2) / (2.0 * remain)

    def ttc_longitudinal(self, distance_m, closing_speed_mps):
        if closing_speed_mps <= 0.03:
            return 999.0
        return distance_m / closing_speed_mps

    def time_to_path_entry(self, obj, steering_angle_deg):
        center = path_center_x_at_z(obj.z_m, steering_angle_deg)
        rel_x = obj.x_m - center
        bound = path_half_width() + obj.width_m / 2.0

        if abs(rel_x) <= bound:
            return 0.0

        # Object is on the left and moving right into path.
        if rel_x < -bound and obj.vx_mps > 0.03:
            return (-bound - rel_x) / obj.vx_mps

        # Object is on the right and moving left into path.
        if rel_x > bound and obj.vx_mps < -0.03:
            return (bound - rel_x) / obj.vx_mps

        return 999.0

    def ego_time_to_conflict_z(self, obj, v_ego):
        if v_ego <= 0.05:
            return 999.0
        return max(0.0, obj.z_m / v_ego)

    def closing_speed(self, obj, v_ego):
        # z decreasing means the object is approaching in ego coordinates.
        closing_from_camera = max(0.0, -obj.vz_mps)
        closing_from_tof = max(0.0, obj.tof_approach_speed_mps)
        closing_from_ultra = max(0.0, obj.ultra_approach_speed_mps)
        closing = max(closing_from_camera, closing_from_tof, closing_from_ultra)

        # Fallback for near in-path object when velocity estimate is unstable.
        if obj.in_path_now and obj.z_m < 2.0 and v_ego > 0.15 and closing < 0.05:
            closing = v_ego * 0.75

        return closing

    def classify_scenario(self, obj, steering_angle_deg):
        t_path = self.time_to_path_entry(obj, steering_angle_deg)

        if obj.in_path_now:
            if abs(obj.vz_mps) < 0.15 and abs(obj.vx_mps) < 0.15:
                return ThreatType.STATIONARY_OBSTACLE
            return ThreatType.LEAD_OBJECT

        if t_path < CROSSING_HORIZON_SEC:
            if is_vru(obj.label):
                return ThreatType.CROSSING_VRU
            if is_vehicle(obj.label):
                return ThreatType.CUT_IN
            return ThreatType.CROSSING_VRU

        return ThreatType.NONE

    def assess_object(self, obj, v_ego, steering_angle_deg):
        threat_type = self.classify_scenario(obj, steering_angle_deg)
        distance_m = max(0.01, obj.z_m)
        closing = self.closing_speed(obj, v_ego)

        ttc = self.ttc_longitudinal(distance_m, closing)
        req_decel = self.required_decel(distance_m, closing)
        stop_dist = self.stopping_distance(v_ego)

        t_path = self.time_to_path_entry(obj, steering_angle_deg)
        t_ego = self.ego_time_to_conflict_z(obj, v_ego)
        time_gap = abs(t_ego - t_path) if t_path < 900 and t_ego < 900 else 999.0

        p = 0.0

        if obj.in_path_now:
            if ttc < FCW_TTC_SEC:
                p += 0.20
            if ttc < PARTIAL_TTC_SEC:
                p += 0.20
            if req_decel > FCW_REQ_DECEL:
                p += 0.15
            if req_decel > PARTIAL_REQ_DECEL:
                p += 0.20
            if stop_dist >= distance_m - self.safety_margin:
                p += 0.25

        if threat_type in {ThreatType.CROSSING_VRU, ThreatType.CUT_IN}:
            if t_path < CROSSING_HORIZON_SEC:
                p += 0.20
            if t_ego < CROSSING_HORIZON_SEC:
                p += 0.20
            if time_gap < CROSSING_FCW_TIME_GAP_SEC:
                p += 0.25
            if time_gap < CROSSING_FULL_TIME_GAP_SEC:
                p += 0.25

        p *= clamp(obj.confidence, 0.0, 1.0)

        if obj.track_age < MIN_TRACK_AGE_FOR_AEB:
            p *= 0.25
        elif obj.track_age < 5:
            p *= 0.70

        p = clamp(p, 0.0, 1.0)

        if obj.z_m > MAX_AEB_RANGE_M:
            return AdvancedRisk(
                threat_type=ThreatType.NONE,
                desired_state=AEBState.MONITORING,
                brake_percent=0,
                reason=f"Object out of AEB range: z={obj.z_m:.2f}m",
                obj=obj,
                distance_m=distance_m,
                ttc_long_sec=ttc,
                required_decel_mps2=req_decel,
                stopping_distance_m=stop_dist,
                time_to_path_sec=t_path,
                ego_time_to_conflict_sec=t_ego,
                time_gap_sec=time_gap,
                collision_probability=p,
                confidence_percent=int(obj.confidence * 100),
            )

        if obj.track_age < MIN_TRACK_AGE_FOR_AEB:
            return AdvancedRisk(
                threat_type=threat_type,
                desired_state=AEBState.MONITORING,
                brake_percent=0,
                reason=f"Track not stable yet: age={obj.track_age}",
                obj=obj,
                distance_m=distance_m,
                ttc_long_sec=ttc,
                required_decel_mps2=req_decel,
                stopping_distance_m=stop_dist,
                time_to_path_sec=t_path,
                ego_time_to_conflict_sec=t_ego,
                time_gap_sec=time_gap,
                collision_probability=p,
                confidence_percent=int(obj.confidence * 100),
            )

        desired_state = AEBState.MONITORING
        brake = 0
        reason = "No critical threat"

        # Longitudinal AEB: object is currently in predicted path.
        if obj.in_path_now:
            if ttc <= FULL_TTC_SEC or req_decel >= FULL_REQ_DECEL or stop_dist >= distance_m:
                desired_state = AEBState.FULL_BRAKE
                brake = 100
                reason = (
                    f"Longitudinal FULL: TTC={ttc:.2f}s, reqDecel={req_decel:.2f}, "
                    f"stopDist={stop_dist:.2f}m, dist={distance_m:.2f}m"
                )
            elif ttc <= PARTIAL_TTC_SEC or req_decel >= PARTIAL_REQ_DECEL:
                desired_state = AEBState.PARTIAL_BRAKE
                brake = 60
                reason = f"Longitudinal PARTIAL: TTC={ttc:.2f}s, reqDecel={req_decel:.2f}, dist={distance_m:.2f}m"
            elif req_decel >= PREFILL_REQ_DECEL:
                desired_state = AEBState.BRAKE_PREFILL
                brake = 20
                reason = f"Longitudinal PREFILL: reqDecel={req_decel:.2f}, dist={distance_m:.2f}m"
            elif ttc <= FCW_TTC_SEC or req_decel >= FCW_REQ_DECEL or stop_dist >= distance_m * 0.75:
                desired_state = AEBState.FCW
                brake = 0
                reason = (
                    f"Longitudinal FCW: TTC={ttc:.2f}s, reqDecel={req_decel:.2f}, "
                    f"stopDist={stop_dist:.2f}m, dist={distance_m:.2f}m"
                )

        # Crossing pedestrian/cyclist/object.
        if threat_type == ThreatType.CROSSING_VRU:
            if t_ego < CROSSING_HORIZON_SEC and time_gap <= CROSSING_FULL_TIME_GAP_SEC:
                desired_state = AEBState.FULL_BRAKE
                brake = 100
                reason = f"Crossing VRU FULL: tEgo={t_ego:.2f}s, tPath={t_path:.2f}s, gap={time_gap:.2f}s"
            elif t_ego < CROSSING_HORIZON_SEC and time_gap <= CROSSING_PARTIAL_TIME_GAP_SEC:
                if desired_state.value < AEBState.PARTIAL_BRAKE.value:
                    desired_state = AEBState.PARTIAL_BRAKE
                    brake = 60
                reason = f"Crossing VRU PARTIAL: tEgo={t_ego:.2f}s, tPath={t_path:.2f}s, gap={time_gap:.2f}s"
            elif t_ego < CROSSING_HORIZON_SEC and time_gap <= CROSSING_FCW_TIME_GAP_SEC:
                if desired_state.value < AEBState.FCW.value:
                    desired_state = AEBState.FCW
                    brake = 0
                reason = f"Crossing VRU FCW: tEgo={t_ego:.2f}s, tPath={t_path:.2f}s, gap={time_gap:.2f}s"

        # Vehicle cut-in.
        if threat_type == ThreatType.CUT_IN:
            if t_ego < CUTIN_HORIZON_SEC and time_gap <= CUTIN_FULL_TIME_GAP_SEC:
                desired_state = AEBState.FULL_BRAKE
                brake = 100
                reason = f"Cut-in FULL: tEgo={t_ego:.2f}s, tPath={t_path:.2f}s, gap={time_gap:.2f}s"
            elif t_ego < CUTIN_HORIZON_SEC and time_gap <= CUTIN_PARTIAL_TIME_GAP_SEC:
                if desired_state.value < AEBState.PARTIAL_BRAKE.value:
                    desired_state = AEBState.PARTIAL_BRAKE
                    brake = 60
                reason = f"Cut-in PARTIAL: tEgo={t_ego:.2f}s, tPath={t_path:.2f}s, gap={time_gap:.2f}s"
            elif t_ego < CUTIN_HORIZON_SEC and time_gap <= CUTIN_FCW_TIME_GAP_SEC:
                if desired_state.value < AEBState.FCW.value:
                    desired_state = AEBState.FCW
                    brake = 0
                reason = f"Cut-in FCW: tEgo={t_ego:.2f}s, tPath={t_path:.2f}s, gap={time_gap:.2f}s"

        return AdvancedRisk(
            threat_type=threat_type,
            desired_state=desired_state,
            brake_percent=brake,
            reason=reason,
            obj=obj,
            distance_m=distance_m,
            ttc_long_sec=ttc,
            required_decel_mps2=req_decel,
            stopping_distance_m=stop_dist,
            time_to_path_sec=t_path,
            ego_time_to_conflict_sec=t_ego,
            time_gap_sec=time_gap,
            collision_probability=p,
            confidence_percent=int(obj.confidence * 100),
        )

    def assess_unknown_close(self, ultrasonic_readings, tof_reading, v_ego):
        valid_ultra = [r for r in ultrasonic_readings.values() if r.valid and r.distance_m is not None]
        front_valid = tof_reading is not None and tof_reading.valid and tof_reading.distance_m is not None
        stop_dist = self.stopping_distance(v_ego)

        if not front_valid and not valid_ultra:
            return AdvancedRisk(
                threat_type=ThreatType.SENSOR_FAULT,
                desired_state=AEBState.SENSOR_FAULT,
                brake_percent=0,
                reason="No valid front ToF or diagonal ultrasonic readings",
                obj=None,
                distance_m=None,
                ttc_long_sec=999.0,
                required_decel_mps2=0.0,
                stopping_distance_m=stop_dist,
                time_to_path_sec=999.0,
                ego_time_to_conflict_sec=999.0,
                time_gap_sec=999.0,
                collision_probability=0.0,
                confidence_percent=0,
            )

        if front_valid:
            distance_m = tof_reading.distance_m
            closing = max(tof_reading.approach_speed_mps, v_ego * 0.85)
            ttc = self.ttc_longitudinal(distance_m, closing)
            req_decel = self.required_decel(distance_m, closing)

            desired = AEBState.MONITORING
            brake = 0
            probability = 0.0
            reason = f"Front ToF clear: {distance_m:.2f}m"

            if (
                distance_m <= UNKNOWN_STOP_DISTANCE_M
                or ttc <= FULL_TTC_SEC
                or req_decel >= FULL_REQ_DECEL
                or stop_dist >= distance_m
            ):
                desired = AEBState.FULL_BRAKE
                brake = 100
                probability = 0.95
                reason = (
                    f"Front ToF unknown FULL: dist={distance_m:.2f}m, "
                    f"TTC={ttc:.2f}s, reqDecel={req_decel:.2f}"
                )
            elif ttc <= PARTIAL_TTC_SEC or req_decel >= PARTIAL_REQ_DECEL or stop_dist >= distance_m * 0.85:
                desired = AEBState.PARTIAL_BRAKE
                brake = 60
                probability = 0.70
                reason = (
                    f"Front ToF unknown PARTIAL: dist={distance_m:.2f}m, "
                    f"TTC={ttc:.2f}s, reqDecel={req_decel:.2f}"
                )
            elif ttc <= FCW_TTC_SEC or req_decel >= FCW_REQ_DECEL or stop_dist >= distance_m * 0.65:
                desired = AEBState.FCW
                brake = 0
                probability = 0.45
                reason = (
                    f"Front ToF unknown FCW: dist={distance_m:.2f}m, "
                    f"TTC={ttc:.2f}s, reqDecel={req_decel:.2f}"
                )

            if desired != AEBState.MONITORING:
                return AdvancedRisk(
                    threat_type=ThreatType.UNKNOWN_CLOSE,
                    desired_state=desired,
                    brake_percent=brake,
                    reason=reason,
                    obj=None,
                    distance_m=distance_m,
                    ttc_long_sec=ttc,
                    required_decel_mps2=req_decel,
                    stopping_distance_m=stop_dist,
                    time_to_path_sec=999.0,
                    ego_time_to_conflict_sec=999.0,
                    time_gap_sec=999.0,
                    collision_probability=probability,
                    confidence_percent=int(tof_reading.confidence * 100),
                )

        nearest = min(valid_ultra, key=lambda r: r.distance_m) if valid_ultra else None

        if nearest is not None:
            distance_m = nearest.forward_distance_m or nearest.distance_m
            ultra_closing = ultrasonic_forward_approach_component(
                nearest.name,
                nearest.approach_speed_mps,
            )
            closing = max(ultra_closing, v_ego * 0.60)
            ttc = self.ttc_longitudinal(distance_m, closing)
            req_decel = self.required_decel(distance_m, closing)

            desired = AEBState.MONITORING
            brake = 0
            probability = 0.0
            reason = f"Diagonal ultrasonic clear: {nearest.name} {nearest.distance_m:.2f}m"

            if nearest.distance_m <= DIAGONAL_FULL_BRAKE_DISTANCE_M or ttc <= FULL_TTC_SEC:
                desired = AEBState.FULL_BRAKE
                brake = 100
                probability = 0.88
                reason = (
                    f"Diagonal unknown FULL: {nearest.name} direct={nearest.distance_m:.2f}m, "
                    f"forward={distance_m:.2f}m, TTC={ttc:.2f}s"
                )
            elif nearest.distance_m <= DIAGONAL_PARTIAL_BRAKE_DISTANCE_M or ttc <= PARTIAL_TTC_SEC:
                desired = AEBState.PARTIAL_BRAKE
                brake = 60
                probability = 0.62
                reason = (
                    f"Diagonal unknown PARTIAL: {nearest.name} direct={nearest.distance_m:.2f}m, "
                    f"forward={distance_m:.2f}m, TTC={ttc:.2f}s"
                )
            elif nearest.distance_m <= DIAGONAL_FCW_DISTANCE_M or ttc <= FCW_TTC_SEC:
                desired = AEBState.FCW
                brake = 0
                probability = 0.38
                reason = (
                    f"Diagonal unknown FCW: {nearest.name} direct={nearest.distance_m:.2f}m, "
                    f"forward={distance_m:.2f}m, TTC={ttc:.2f}s"
                )

            if desired != AEBState.MONITORING:
                return AdvancedRisk(
                    threat_type=ThreatType.UNKNOWN_CLOSE,
                    desired_state=desired,
                    brake_percent=brake,
                    reason=reason,
                    obj=None,
                    distance_m=distance_m,
                    ttc_long_sec=ttc,
                    required_decel_mps2=req_decel,
                    stopping_distance_m=stop_dist,
                    time_to_path_sec=999.0,
                    ego_time_to_conflict_sec=999.0,
                    time_gap_sec=999.0,
                    collision_probability=probability,
                    confidence_percent=int(nearest.confidence * 100),
                )

        if not front_valid:
            nearest_distance = nearest.forward_distance_m if nearest is not None else None
            return AdvancedRisk(
                threat_type=ThreatType.SENSOR_FAULT,
                desired_state=AEBState.DEGRADED,
                brake_percent=0,
                reason="Front ToF unavailable; running camera + diagonal ultrasonic degraded mode",
                obj=None,
                distance_m=nearest_distance,
                ttc_long_sec=nearest.ttc_sec if nearest is not None else 999.0,
                required_decel_mps2=0.0,
                stopping_distance_m=stop_dist,
                time_to_path_sec=999.0,
                ego_time_to_conflict_sec=999.0,
                time_gap_sec=999.0,
                collision_probability=0.0,
                confidence_percent=0,
            )

        nearest_distance = tof_reading.distance_m
        nearest_ttc = tof_reading.ttc_sec

        return AdvancedRisk(
            threat_type=ThreatType.NONE,
            desired_state=AEBState.MONITORING,
            brake_percent=0,
            reason="No object threat",
            obj=None,
            distance_m=nearest_distance,
            ttc_long_sec=nearest_ttc,
            required_decel_mps2=0.0,
            stopping_distance_m=stop_dist,
            time_to_path_sec=999.0,
            ego_time_to_conflict_sec=999.0,
            time_gap_sec=999.0,
            collision_probability=0.0,
            confidence_percent=0,
        )

    def assess_all(self, objects, ultrasonic_readings, tof_reading, v_ego, steering_angle_deg):
        if not objects:
            return self.assess_unknown_close(ultrasonic_readings, tof_reading, v_ego)

        risks = [self.assess_object(obj, v_ego, steering_angle_deg) for obj in objects]
        unknown_risk = self.assess_unknown_close(ultrasonic_readings, tof_reading, v_ego)

        if unknown_risk.desired_state in {
            AEBState.FULL_BRAKE,
            AEBState.PARTIAL_BRAKE,
            AEBState.BRAKE_PREFILL,
            AEBState.FCW,
            AEBState.DEGRADED,
            AEBState.SENSOR_FAULT,
        }:
            risks.append(unknown_risk)

        def priority(r):
            state_score = {
                AEBState.FULL_BRAKE: 6,
                AEBState.STOP_HOLD: 6,
                AEBState.PARTIAL_BRAKE: 5,
                AEBState.BRAKE_PREFILL: 4,
                AEBState.FCW: 3,
                AEBState.DEGRADED: 2,
                AEBState.MONITORING: 1,
                AEBState.SENSOR_FAULT: 2,
            }.get(r.desired_state, 0)

            return (state_score, r.collision_probability, r.required_decel_mps2, -r.ttc_long_sec)

        risks.sort(key=priority, reverse=True)
        return risks[0]

# ============================================================
# State Machine
# ============================================================

class AEBStateMachine:
    def __init__(self):
        self.state = AEBState.STANDBY
        self.clear_count = 0
        self.hold_start_time = None

    def update(self, risk, driver_override=False):
        now = time.perf_counter()
        desired = risk.desired_state
        brake = risk.brake_percent

        if driver_override:
            self.state = AEBState.RELEASE
            self.clear_count = 0
            return self.state, 0, "Driver override"

        if desired == AEBState.SENSOR_FAULT:
            self.state = AEBState.SENSOR_FAULT
            self.clear_count = 0
            return self.state, 0, risk.reason

        if desired == AEBState.FULL_BRAKE:
            self.state = AEBState.FULL_BRAKE
            self.clear_count = 0

            if risk.distance_m is not None and risk.distance_m <= STOP_HOLD_DISTANCE_M:
                self.state = AEBState.STOP_HOLD
                self.hold_start_time = now
                return self.state, 100, "Stop hold entered"

            return self.state, 100, risk.reason

        if self.state == AEBState.STOP_HOLD:
            if self.hold_start_time is not None and now - self.hold_start_time < STOP_HOLD_SEC:
                return self.state, 100, "Holding brake"
            self.state = AEBState.RELEASE
            return self.state, 0, "Stop hold released"

        if desired in {AEBState.PARTIAL_BRAKE, AEBState.BRAKE_PREFILL, AEBState.FCW, AEBState.DEGRADED}:
            self.state = desired
            self.clear_count = 0
            return self.state, brake, risk.reason

        if desired == AEBState.MONITORING:
            if self.state in {
                AEBState.FCW,
                AEBState.BRAKE_PREFILL,
                AEBState.PARTIAL_BRAKE,
                AEBState.FULL_BRAKE,
                AEBState.RELEASE,
                AEBState.DEGRADED,
            }:
                self.clear_count += 1

                if self.clear_count >= CLEAR_CONFIRM_COUNT:
                    self.state = AEBState.MONITORING
                    self.clear_count = 0
                    return self.state, 0, "Risk cleared"

                if self.state == AEBState.PARTIAL_BRAKE:
                    return self.state, 30, "Clearing risk, reduced brake"
                if self.state == AEBState.BRAKE_PREFILL:
                    return self.state, 10, "Clearing risk, prefill release"
                return self.state, 0, "Clearing risk"

            self.state = AEBState.MONITORING
            return self.state, 0, risk.reason

        self.state = desired
        return self.state, brake, risk.reason

# ============================================================
# Brake Controller / Optional CAN
# ============================================================

class BrakeController:
    def __init__(self):
        self.last_state = None
        self.last_brake = -1
        self.alive_counter = 0
        self.can_bus = None

        if USE_CAN:
            if not CAN_AVAILABLE:
                print("[WARN] python-can unavailable. CAN disabled.")
            else:
                try:
                    self.can_bus = can.interface.Bus(channel=CAN_CHANNEL, interface="socketcan")
                    print("[INFO] CAN enabled:", CAN_CHANNEL)
                except Exception as e:
                    print("[WARN] CAN init failed:", e)
                    self.can_bus = None

    def apply(self, state, brake_percent, risk):
        brake_percent = int(clamp(brake_percent, 0, 100))

        if not DRY_RUN:
            self.apply_to_actuator(brake_percent, state, risk)

        if self.can_bus is not None:
            self.send_can(state, brake_percent, risk)

        important = state in {
            AEBState.FCW,
            AEBState.BRAKE_PREFILL,
            AEBState.PARTIAL_BRAKE,
            AEBState.FULL_BRAKE,
            AEBState.STOP_HOLD,
            AEBState.DEGRADED,
            AEBState.SENSOR_FAULT,
        }

        if important or brake_percent != self.last_brake or state != self.last_state:
            print(
                f"[AEB] {state.name:<14} "
                f"brake={brake_percent:3d}% "
                f"type={risk.threat_type.name:<18} "
                f"dist={fmt(risk.distance_m)}m "
                f"TTC={fmt(risk.ttc_long_sec)}s "
                f"reqDecel={fmt(risk.required_decel_mps2)} "
                f"stopDist={fmt(risk.stopping_distance_m)}m "
                f"reason={risk.reason}"
            )

        self.last_state = state
        self.last_brake = brake_percent

    def apply_to_actuator(self, brake_percent, state, risk):
        """
        Connect your RC car motor/brake control here.

        Example:
            if brake_percent >= 100:
                motor_stop()
                brake_full()
            elif brake_percent >= 60:
                motor_set_speed(20)
                brake_partial(brake_percent)
            elif brake_percent >= 20:
                motor_set_speed(40)
            else:
                brake_release()
        """
        if brake_percent >= 100:
            # TODO: motor_stop()
            # TODO: brake_full()
            pass
        elif brake_percent >= 60:
            # TODO: motor_set_speed(20)
            # TODO: brake_partial(brake_percent)
            pass
        elif brake_percent >= 20:
            # TODO: motor_set_speed(40)
            pass
        else:
            # TODO: brake_release()
            pass

    def send_can(self, state, brake_percent, risk):
        self.alive_counter = (self.alive_counter + 1) & 0x0F

        status = state.value & 0xFF
        brake = brake_percent & 0xFF

        dist_cm = 255
        if risk.distance_m is not None:
            dist_cm = int(clamp(risk.distance_m * 100.0, 0, 255))

        ttc_x10 = 255
        if risk.ttc_long_sec is not None:
            ttc_x10 = int(clamp(risk.ttc_long_sec * 10.0, 0, 255))

        obj_type = object_type_code(risk.obj.label if risk.obj else None)
        conf = int(clamp(risk.confidence_percent, 0, 100))
        alive = self.alive_counter

        data = [status, brake, dist_cm, ttc_x10, obj_type, conf, alive, 0]
        data[7] = calc_crc_xor(data[:7])

        try:
            msg = can.Message(arbitration_id=CAN_ID_AEB_BRAKE_REQUEST, data=data, is_extended_id=False)
            self.can_bus.send(msg)
        except Exception as e:
            print("[WARN] CAN send failed:", e)

# ============================================================
# CSV Logger
# ============================================================

class CSVLogger:
    def __init__(self, path):
        self.file = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "time",
            "state",
            "brake_percent",
            "threat_type",
            "reason",
            "obj_id",
            "label",
            "conf",
            "track_age",
            "x_m",
            "z_m",
            "vx_mps",
            "vz_mps",
            "in_path",
            "z_source",
            "selected_diagonal_ultra",
            "object_ultra_m",
            "object_tof_m",
            "front_tof_m",
            "left_diag_m",
            "left_diag_forward_m",
            "right_diag_m",
            "right_diag_forward_m",
            "distance_m",
            "ttc_long_sec",
            "required_decel_mps2",
            "stopping_distance_m",
            "time_to_path_sec",
            "ego_time_to_conflict_sec",
            "time_gap_sec",
            "collision_probability",
            "ego_speed_mps",
            "steering_angle_deg",
        ])

    def write(self, state, brake_percent, risk, ultrasonic_readings, tof_reading, ego_speed, steering_angle):
        obj = risk.obj

        def ultra_distance(name):
            r = ultrasonic_readings.get(name)
            if r and r.distance_m is not None:
                return f"{r.distance_m:.3f}"
            return ""

        def ultra_forward_distance(name):
            r = ultrasonic_readings.get(name)
            if r and r.forward_distance_m is not None:
                return f"{r.forward_distance_m:.3f}"
            return ""

        def tof_distance():
            if tof_reading and tof_reading.distance_m is not None:
                return f"{tof_reading.distance_m:.3f}"
            return ""

        row = [
            f"{time.time():.3f}",
            state.name,
            brake_percent,
            risk.threat_type.name,
            risk.reason,
            obj.track_id if obj else "",
            obj.label if obj else "",
            f"{obj.confidence:.3f}" if obj else "",
            obj.track_age if obj else "",
            f"{obj.x_m:.3f}" if obj else "",
            f"{obj.z_m:.3f}" if obj else "",
            f"{obj.vx_mps:.3f}" if obj else "",
            f"{obj.vz_mps:.3f}" if obj else "",
            obj.in_path_now if obj else "",
            obj.z_source if obj else "",
            obj.selected_ultra_name if obj else "",
            f"{obj.ultra_distance_m:.3f}" if obj and obj.ultra_distance_m is not None else "",
            f"{obj.tof_distance_m:.3f}" if obj and obj.tof_distance_m is not None else "",
            tof_distance(),
            ultra_distance(LEFT_DIAGONAL_NAME),
            ultra_forward_distance(LEFT_DIAGONAL_NAME),
            ultra_distance(RIGHT_DIAGONAL_NAME),
            ultra_forward_distance(RIGHT_DIAGONAL_NAME),
            f"{risk.distance_m:.3f}" if risk.distance_m is not None else "",
            f"{risk.ttc_long_sec:.3f}",
            f"{risk.required_decel_mps2:.3f}",
            f"{risk.stopping_distance_m:.3f}",
            f"{risk.time_to_path_sec:.3f}",
            f"{risk.ego_time_to_conflict_sec:.3f}",
            f"{risk.time_gap_sec:.3f}",
            f"{risk.collision_probability:.3f}",
            f"{ego_speed:.3f}",
            f"{steering_angle:.3f}",
        ]

        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        try:
            self.file.close()
        except Exception:
            pass

# ============================================================
# Visualization
# ============================================================

def draw_overlay(frame, objects, risk, ultrasonic_readings, tof_reading, state, brake_percent, steering_angle_deg):
    if frame is None or not CV2_AVAILABLE:
        return frame

    h, w = frame.shape[:2]

    # Simple visual path box. Real decision uses meter-based path model.
    path_left_px = int(w * 0.35)
    path_right_px = int(w * 0.65)
    cv2.rectangle(frame, (path_left_px, 0), (path_right_px, h), (0, 80, 255), 2)

    selected_id = risk.obj.track_id if risk.obj else None

    for obj in objects:
        x1, y1, x2, y2 = obj.bbox

        if obj.track_id == selected_id:
            color = (0, 0, 255)
            thickness = 3
        elif obj.in_path_now:
            color = (0, 255, 255)
            thickness = 2
        else:
            color = (150, 150, 150)
            thickness = 1

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

        text = (
            f"ID:{obj.track_id} {obj.label} "
            f"z:{obj.z_m:.2f} x:{obj.x_m:.2f} "
            f"vx:{obj.vx_mps:.2f} vz:{obj.vz_mps:.2f} "
            f"age:{obj.track_age} in:{int(obj.in_path_now)} src:{obj.z_source}"
        )
        cv2.putText(frame, text, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

    lines = [
        f"STATE: {state.name}  BRAKE: {brake_percent}%",
        f"Threat: {risk.threat_type.name}",
        f"Reason: {risk.reason}",
        (
            f"dist={fmt(risk.distance_m)}m "
            f"TTC={fmt(risk.ttc_long_sec)}s "
            f"reqDecel={fmt(risk.required_decel_mps2)} "
            f"stopDist={fmt(risk.stopping_distance_m)}m"
        ),
        (
            f"tPath={fmt(risk.time_to_path_sec)}s "
            f"tEgo={fmt(risk.ego_time_to_conflict_sec)}s "
            f"gap={fmt(risk.time_gap_sec)}s "
            f"P={risk.collision_probability:.2f}"
        ),
    ]

    if tof_reading:
        lines.append(
            f"FRONT_TOF: {fmt(tof_reading.distance_m)}m, "
            f"approach={tof_reading.approach_speed_mps:.2f}m/s, "
            f"TTC={fmt(tof_reading.ttc_sec)}s, valid={int(tof_reading.valid)}"
        )

    for name in [LEFT_DIAGONAL_NAME, RIGHT_DIAGONAL_NAME]:
        r = ultrasonic_readings.get(name)
        if r:
            lines.append(
                f"{name}: direct={fmt(r.distance_m)}m, forward={fmt(r.forward_distance_m)}m, "
                f"approach={r.approach_speed_mps:.2f}m/s, TTC={fmt(r.ttc_sec)}s, valid={int(r.valid)}"
            )

    y = 25
    for line in lines:
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        y += 23

    return frame

# ============================================================
# Main
# ============================================================

running = True


def signal_handler(sig, frame):
    global running
    running = False


def main():
    global running

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("============================================================")
    print(" Advanced Raspberry Pi 4 AEB")
    print(" Front camera + front ToF + left/right diagonal ultrasonic")
    print("============================================================")
    print(f"[CONFIG] DRY_RUN={DRY_RUN}")
    print(f"[CONFIG] USE_YOLO={USE_YOLO}, YOLO_AVAILABLE={YOLO_AVAILABLE}")
    print(f"[CONFIG] USE_TOF={USE_TOF}, TOF_SENSOR_MODEL={TOF_SENSOR_MODEL}")
    print(f"[CONFIG] FOCAL_X={FOCAL_X:.1f}, FOCAL_Y={FOCAL_Y:.1f}")
    print(f"[CONFIG] AVAILABLE_DECEL={AVAILABLE_DECEL_MPS2} m/s^2")
    print("============================================================")

    ultrasonic = UltrasonicManager()
    tof_sensor = ToFSensor()
    camera = CameraManager()
    detector = ObjectDetector()
    kin_tracker = ObjectKinematicsTracker()
    threat = AdvancedThreatAssessment()
    state_machine = AEBStateMachine()
    brake_controller = BrakeController()
    logger = CSVLogger(LOG_CSV_PATH)

    frame_count = 0
    last_objects = []
    yolo_interval = max(1, int(YOLO_EVERY_N_FRAMES))
    loop_period = 1.0 / max(1, LOOP_HZ)

    try:
        while running:
            loop_start = time.perf_counter()

            ego_speed = get_ego_speed_mps()
            steering_angle = get_steering_angle_deg()
            driver_override = get_driver_override()

            ultrasonic_readings = ultrasonic.update_all()
            tof_reading = tof_sensor.update()
            frame = camera.read()

            frame_count += 1
            objects = []

            if frame is not None and detector.enabled:
                if frame_count % yolo_interval == 0:
                    detections = detector.detect(frame)
                    objects = kin_tracker.update(
                        detections=detections,
                        ultrasonic_readings=ultrasonic_readings,
                        tof_reading=tof_reading,
                        steering_angle_deg=steering_angle,
                    )
                    last_objects = objects
                else:
                    objects = last_objects

            risk = threat.assess_all(
                objects=objects,
                ultrasonic_readings=ultrasonic_readings,
                tof_reading=tof_reading,
                v_ego=ego_speed,
                steering_angle_deg=steering_angle,
            )

            state, brake_percent, _state_reason = state_machine.update(
                risk=risk,
                driver_override=driver_override,
            )

            brake_controller.apply(state, brake_percent, risk)

            logger.write(
                state=state,
                brake_percent=brake_percent,
                risk=risk,
                ultrasonic_readings=ultrasonic_readings,
                tof_reading=tof_reading,
                ego_speed=ego_speed,
                steering_angle=steering_angle,
            )

            if SHOW_PREVIEW and frame is not None and CV2_AVAILABLE:
                overlay = draw_overlay(
                    frame=frame,
                    objects=objects,
                    risk=risk,
                    ultrasonic_readings=ultrasonic_readings,
                    tof_reading=tof_reading,
                    state=state,
                    brake_percent=brake_percent,
                    steering_angle_deg=steering_angle,
                )
                cv2.imshow("Advanced AEB - Camera + ToF + Diagonal Ultrasonic", overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    running = False

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, loop_period - elapsed))

    finally:
        print("[INFO] Cleaning up...")

        try:
            shutdown_risk = AdvancedRisk(
                threat_type=ThreatType.NONE,
                desired_state=AEBState.RELEASE,
                brake_percent=0,
                reason="System shutdown",
                obj=None,
                distance_m=None,
                ttc_long_sec=999.0,
                required_decel_mps2=0.0,
                stopping_distance_m=0.0,
                time_to_path_sec=999.0,
                ego_time_to_conflict_sec=999.0,
                time_gap_sec=999.0,
                collision_probability=0.0,
                confidence_percent=0,
            )
            brake_controller.apply(AEBState.RELEASE, 0, shutdown_risk)
        except Exception:
            pass

        logger.close()
        tof_sensor.release()
        camera.release()

        if CV2_AVAILABLE:
            cv2.destroyAllWindows()

        if GPIO_AVAILABLE:
            GPIO.cleanup()

        print("[INFO] AEB stopped.")
        print(f"[INFO] Log saved: {LOG_CSV_PATH}")


if __name__ == "__main__":
    main()
