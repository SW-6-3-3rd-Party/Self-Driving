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
import os
import signal
import sys
import threading
import time
import select

try:
    import termios
    import tty
    import fcntl
    TERMINAL_KEYBOARD_AVAILABLE = True
except Exception:
    termios = None
    tty = None
    fcntl = None
    TERMINAL_KEYBOARD_AVAILABLE = False
# ============================================================
# Runtime / terminal helpers
# ============================================================

LAST_RUNTIME_WARNING = ""
LAST_RUNTIME_WARNING_TIME = 0.0
_WARN_LAST_TIME = {}
DASHBOARD_HAS_DRAWN = False


def runtime_warn(key, message, interval_sec=2.0):
    """
    Throttled runtime warning.

    In dashboard mode, asynchronous print() calls from sensor / YOLO threads
    corrupt the redrawn terminal UI. So during dashboard operation we store the
    newest warning and show it inside the dashboard instead of printing a new line.
    """
    global LAST_RUNTIME_WARNING, LAST_RUNTIME_WARNING_TIME

    now = time.perf_counter()
    last = _WARN_LAST_TIME.get(key, 0.0)
    if now - last < interval_sec:
        return
    _WARN_LAST_TIME[key] = now

    text = str(message)
    LAST_RUNTIME_WARNING = text
    LAST_RUNTIME_WARNING_TIME = now

    dashboard_mode = (
        globals().get("PRINT_SENSOR_VALUES", False)
        and globals().get("PRETTY_TERMINAL_OUTPUT", False)
        and DASHBOARD_HAS_DRAWN
    )
    if not dashboard_mode:
        print(text)


def terminal_clear_and_home():
    return "\033[2J\033[H"


def terminal_hide_cursor():
    return "\033[?25l"


def terminal_show_cursor():
    return "\033[?25h"


def write_dashboard(lines):
    """
    Atomic-ish terminal dashboard redraw.
    Uses CRLF so output remains correct even if terminal input mode changes.
    """
    global DASHBOARD_HAS_DRAWN

    prefix = ""
    if globals().get("PRETTY_TERMINAL_OUTPUT", False) and globals().get("TERMINAL_CLEAR_EACH_PRINT", False):
        prefix = terminal_clear_and_home() + terminal_hide_cursor()

    text = prefix + "\r\n".join(lines) + "\r\n"
    sys.stdout.write(text)
    if globals().get("TERMINAL_FORCE_FLUSH", False):
        sys.stdout.flush()
    DASHBOARD_HAS_DRAWN = True
from collections import Counter, deque
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
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
SHOW_PREVIEW = False
AUTO_DISABLE_PREVIEW_WITHOUT_DISPLAY = True

# If distance estimation is inaccurate, tune these first.
CAMERA_HFOV_DEG = 62.0
CAMERA_VFOV_DEG = 48.0

USE_YOLO = True
YOLO_MODEL_PATH = "yolov5nu.pt"
YOLO_CONF = 0.40
YOLO_IMGSZ = 128
YOLO_EVERY_N_FRAMES = 10
YOLO_MAX_DET = 6
YOLO_TARGET_CLASS_IDS = [0, 1, 2, 3, 5, 7]  # person, bicycle, car, motorcycle, bus, truck

# Ego vehicle inputs. Replace get_ego_speed_mps() and get_steering_angle_deg()
# if you have encoder / CAN / servo command signals.
DEFAULT_EGO_SPEED_MPS = 0.0
DEFAULT_STEERING_ANGLE_DEG = 0.0

# Keyboard simulation control for test bench / Raspberry Pi terminal.
# Up    : speed +1 km/h
# Down  : speed -1 km/h, never below 0
# Left  : steer left
# Right : steer right
# S     : steer front
KEYBOARD_CONTROL_ENABLED = True
KEYBOARD_SPEED_STEP_KMH = 1.0
KEYBOARD_STEERING_ANGLE_DEG = 15.0
KEYBOARD_MAX_SPEED_KMH = 30.0

# Production-style AEB demonstration features for project presentation.
# These do not make this a real vehicle-grade safety system, but they mirror
# common 양산차 AEB concepts: readiness/degraded mode, sensor fusion confidence,
# speed-adaptive safety envelope, driver override, and decision rationale.
PRODUCTION_AEB_DEMO_FEATURES = True
SENSOR_FUSION_CAMERA_STALE_SEC = 1.5
SENSOR_FUSION_MIN_CONFIDENCE_OK = 55
SENSOR_FUSION_MIN_CONFIDENCE_DEGRADED = 25
AEB_SAFETY_ENVELOPE_GAIN = 1.00
AEB_WARNING_ENVELOPE_GAIN = 0.75
AEB_DRIVER_OVERRIDE_KEY = "O"

# Stricter engagement tuning.
# These make the dashboard Result turn ON only when the situation is clearly dangerous.
# FCW / PREFILL can still be shown as risk states, but Result means actual emergency braking.
STRICT_AEB_MODE = True
STRICT_FULL_STOP_DIST_GAIN = 1.00      # Production-like: brake when scaled stopping envelope reaches target.
STRICT_PARTIAL_STOP_DIST_GAIN = 0.82   # Partial brake before full stopping envelope is exhausted.
STRICT_FCW_STOP_DIST_GAIN = 0.62       # FCW before actual brake.
STRICT_EMERGENCY_RESULT_MIN_BRAKE = 60 # Result ON only for partial/full emergency braking.

# ============================================================
# Production-like 1/32 scale model tuning
# ============================================================
# Public standards/protocols define AEB purpose, operating speed ranges and test scenarios,
# but OEM internal trigger maps are proprietary. This code therefore uses a physics-based
# surrogate: real-equivalent TTC, required deceleration, and stopping envelope, then scales
# only distances by 1/32 for the RC car.
MODEL_SCALE_DENOMINATOR = 32.0
REAL_EQUIVALENT_SPEED_MODE = True
REAL_MIN_AEB_SPEED_KMH = 10.0       # NHTSA FMVSS 127 lower operating speed basis.
REAL_MAX_DECEL_MPS2 = 7.85          # ~0.8g dry-road emergency braking reference.
REAL_SYSTEM_DELAY_SEC = 0.35        # perception + decision + brake build-up surrogate.
REAL_SAFETY_MARGIN_M = 1.00         # final clearance margin in real-car scale.
REAL_FCW_TTC_SEC = 2.00
REAL_PARTIAL_TTC_SEC = 1.20
REAL_FULL_TTC_SEC = 0.60
REAL_FCW_REQ_DECEL_MPS2 = 2.00
REAL_PREFILL_REQ_DECEL_MPS2 = 3.00
REAL_PARTIAL_REQ_DECEL_MPS2 = 4.50
REAL_FULL_REQ_DECEL_MPS2 = 7.00
REAL_UNKNOWN_FULL_STOP_DISTANCE_M = 4.0   # scaled to 12.5cm on 1/32 RC.
REAL_DIAGONAL_FCW_DISTANCE_M = 2.0        # scaled to 6.25cm.
REAL_DIAGONAL_PARTIAL_DISTANCE_M = 1.2    # scaled to 3.75cm.
REAL_DIAGONAL_FULL_DISTANCE_M = 0.8       # scaled to 2.5cm.
EGO_MOVING_MIN_SPEED_MPS = (REAL_MIN_AEB_SPEED_KMH / 3.6) / MODEL_SCALE_DENOMINATOR

# RC car / test bench geometry.
VEHICLE_WIDTH_M = 0.32
PATH_MARGIN_M = 0.12
STEERING_PATH_GAIN = 0.65

# Brake performance. Must be tuned with actual stop tests.
AVAILABLE_DECEL_MPS2 = REAL_MAX_DECEL_MPS2 / MODEL_SCALE_DENOMINATOR
REACTION_TIME_SEC = REAL_SYSTEM_DELAY_SEC
SAFETY_MARGIN_M = REAL_SAFETY_MARGIN_M / MODEL_SCALE_DENOMINATOR
MAX_AEB_RANGE_M = 6.0

# Ultrasonic settings.
ULTRA_MIN_CM = 2
ULTRA_MAX_CM = 500
ULTRA_MEDIAN_SIZE = 3
ULTRA_TIMEOUT_SEC = 0.015
ULTRA_CROSSTALK_DELAY_SEC = 0.005
ULTRA_FUSION_MAX_M = 3.5
ULTRA_DIAGONAL_ANGLE_DEG = 35.0
ULTRA_STALE_SEC = 0.50
DIAGONAL_FCW_DISTANCE_M = REAL_DIAGONAL_FCW_DISTANCE_M / MODEL_SCALE_DENOMINATOR
DIAGONAL_PARTIAL_BRAKE_DISTANCE_M = REAL_DIAGONAL_PARTIAL_DISTANCE_M / MODEL_SCALE_DENOMINATOR
DIAGONAL_FULL_BRAKE_DISTANCE_M = REAL_DIAGONAL_FULL_DISTANCE_M / MODEL_SCALE_DENOMINATOR

# Front ToF settings.
# I2C mode uses Raspberry Pi GPIO2/GPIO3:
#   ToF SOA/SDA -> Pi GPIO2 SDA1
#   ToF SCL     -> Pi GPIO3 SCL1
# GPIO1 and XSHUT are optional and unused by this code.
USE_TOF = True
TOF_INTERFACE = "I2C"  # "I2C", "UART", or "CAN"
TOF_CAN_CHANNEL = "can0"
TOF_CAN_BITRATE = 500000
TOF_CAN_ID = None  # Set to sensor arbitration ID after checking raw frames.
TOF_CAN_TIMEOUT_SEC = 0.003
TOF_CAN_DISTANCE_OFFSET = 0
TOF_CAN_DISTANCE_LENGTH = 2
TOF_CAN_DISTANCE_ENDIAN = "little"  # "little" or "big"
TOF_CAN_DISTANCE_SCALE_MM = 1.0
TOF_CAN_DEBUG_FRAMES = True
TOF_UART_PORT = "/dev/serial0"
TOF_UART_PORT_CANDIDATES = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0", "/dev/ttyUSB0", "/dev/ttyACM0"]
TOF_UART_BAUD = 115200
TOF_UART_AUTO_DETECT_BAUD = True
TOF_UART_BAUD_CANDIDATES = [115200, 921600, 460800, 230400, 57600, 38400, 9600]
TOF_UART_AUTODETECT_SEC = 0.20
TOF_UART_TIMEOUT_SEC = 0.005
TOF_UART_PROTOCOL = "AUTO"  # "AUTO", "NOOPLOOP", "BENWAKE", or "ASCII"
TOF_UART_DEBUG_BYTES = True
TOF_SENSOR_MODEL = "AUTO"  # "AUTO", "VL53L0X", or "VL53L1X"
TOF_I2C_ADDRESS = 0x29
TOF_I2C_DEBUG_SCAN = True
TOF_DISTANCE_SCALE = 0.5
TOF_MIN_M = 0.03
TOF_MAX_M = 4.00
TOF_MEDIAN_SIZE = 3
TOF_FUSION_MAX_M = 4.00
TOF_STALE_SEC = 0.40
TOF_CENTER_GATE_RATIO = 0.36

UNKNOWN_STOP_DISTANCE_M = REAL_UNKNOWN_FULL_STOP_DISTANCE_M / MODEL_SCALE_DENOMINATOR
STOP_HOLD_DISTANCE_M = 3.0 / MODEL_SCALE_DENOMINATOR
STOP_HOLD_SEC = 1.0

# Tracking / kinematics.
MIN_OBJECT_CONF = 0.70  # production-like: reduce false positives
MIN_TRACK_AGE_FOR_AEB = 5  # stable person track before AEB
TRACK_MATCH_DISTANCE_PX = 90
TRACK_MAX_MISSED_SEC = 0.7
POS_FILTER_ALPHA = 0.55
VEL_FILTER_ALPHA = 0.35

# Advanced threat thresholds.
FCW_TTC_SEC = REAL_FCW_TTC_SEC
PARTIAL_TTC_SEC = REAL_PARTIAL_TTC_SEC
FULL_TTC_SEC = REAL_FULL_TTC_SEC

FCW_REQ_DECEL = REAL_FCW_REQ_DECEL_MPS2
PREFILL_REQ_DECEL = REAL_PREFILL_REQ_DECEL_MPS2
PARTIAL_REQ_DECEL = REAL_PARTIAL_REQ_DECEL_MPS2
FULL_REQ_DECEL = REAL_FULL_REQ_DECEL_MPS2

CROSSING_HORIZON_SEC = 2.0
CROSSING_FCW_TIME_GAP_SEC = 0.90
CROSSING_PARTIAL_TIME_GAP_SEC = 0.55
CROSSING_FULL_TIME_GAP_SEC = 0.35

CUTIN_HORIZON_SEC = 2.0
CUTIN_FCW_TIME_GAP_SEC = 0.75
CUTIN_PARTIAL_TIME_GAP_SEC = 0.50
CUTIN_FULL_TIME_GAP_SEC = 0.32

CLEAR_CONFIRM_COUNT = 5
LOOP_HZ = 25
PRINT_SENSOR_VALUES = True
# Terminal dashboard refresh. 0.05 sec = up to 20Hz terminal update.
# If the terminal itself becomes too heavy, raise to 0.08~0.10.
SENSOR_PRINT_INTERVAL_SEC = 0.05
PRETTY_TERMINAL_OUTPUT = True
TERMINAL_CLEAR_EACH_PRINT = True
TERMINAL_FORCE_FLUSH = True
AEB_EVENT_PRINT_INTERVAL_SEC = 1.0

# CAN output option.
USE_CAN = False
CAN_CHANNEL = "can0"
CAN_ID_AEB_BRAKE_REQUEST = 0x310

ENABLE_CSV_LOG = False
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
    try:
        # Keep OpenCV from spawning extra CPU threads on Raspberry Pi.
        cv2.setNumThreads(1)
    except Exception:
        pass
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

if USE_TOF and TOF_INTERFACE.upper() == "UART":
    try:
        import serial
        SERIAL_AVAILABLE = True
    except Exception as e:
        SERIAL_AVAILABLE = False
        print("[WARN] pyserial import failed:", e)
else:
    SERIAL_AVAILABLE = False

if USE_TOF and TOF_INTERFACE.upper() == "I2C":
    try:
        import board
        import busio
        I2C_AVAILABLE = True
        I2C_IMPORT_ERROR = ""
    except Exception as e:
        I2C_AVAILABLE = False
        I2C_IMPORT_ERROR = f"board/busio import failed: {e}"
        print("[WARN] board/busio import failed:", e)

    try:
        import adafruit_vl53l0x
        VL53L0X_AVAILABLE = True
        VL53L0X_IMPORT_ERROR = ""
    except Exception as e:
        VL53L0X_AVAILABLE = False
        VL53L0X_IMPORT_ERROR = f"adafruit_vl53l0x import failed: {e}"
        print("[WARN] adafruit_vl53l0x import failed:", e)

    try:
        import adafruit_vl53l1x
        VL53L1X_AVAILABLE = True
        VL53L1X_IMPORT_ERROR = ""
    except Exception as e:
        VL53L1X_AVAILABLE = False
        VL53L1X_IMPORT_ERROR = f"adafruit_vl53l1x import failed: {e}"
        print("[WARN] adafruit_vl53l1x import failed:", e)
else:
    I2C_AVAILABLE = False
    VL53L0X_AVAILABLE = False
    VL53L1X_AVAILABLE = False
    I2C_IMPORT_ERROR = ""
    VL53L0X_IMPORT_ERROR = ""
    VL53L1X_IMPORT_ERROR = ""

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
    sensor_ok: bool
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


def fmt_cell(value, unit="", digits=2, width=8):
    if value is None:
        text = "--"
    elif isinstance(value, float):
        if value > 900:
            text = "INF"
        elif digits == 0:
            text = f"{value:.0f}"
        else:
            text = f"{value:.{digits}f}"
    else:
        text = str(value)

    if unit and text not in {"--", "INF"}:
        text = f"{text}{unit}"
    return text.rjust(width)


def preview_display_available():
    if not SHOW_PREVIEW:
        return False
    if not AUTO_DISABLE_PREVIEW_WITHOUT_DISPLAY:
        return True
    if os.name != "posix":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


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

class KeyboardEgoController:
    """
    Robust non-blocking terminal keyboard controller for Raspberry Pi / Linux TTY.

    Uses cbreak-style input instead of raw mode. Raw mode disables terminal
    output processing on many Linux terminals, which can make the dashboard
    line breaks look broken. Cbreak keeps output normal while still allowing
    non-blocking key reads.

    Keys:
      Up arrow    : speed +1 km/h
      Down arrow  : speed -1 km/h, clamped at 0
      Left arrow  : steering left
      Right arrow : steering right
      S           : steering front
      O           : toggle driver override / AEB release

    Fallback:
      W/X : speed up/down
      A/D : steer left/right
      S   : steer front
      O   : override
      Q   : quit
    """

    def __init__(self):
        self.enabled = bool(KEYBOARD_CONTROL_ENABLED)
        self.speed_kmh = DEFAULT_EGO_SPEED_MPS * 3.6
        self.steering_angle_deg = DEFAULT_STEERING_ANGLE_DEG
        self.driver_override = False
        self._old_terminal_settings = None
        self._old_file_flags = None
        self._input_enabled = False
        self._fd = None
        self._key_buffer = bytearray()
        self.last_key_text = "--"

    def start(self):
        if not self.enabled:
            return
        if not TERMINAL_KEYBOARD_AVAILABLE or not sys.stdin.isatty():
            self.enabled = False
            print("[WARN] Keyboard control disabled: run from a real terminal/SSH TTY, not VSCode output panel.")
            return

        try:
            self._fd = sys.stdin.fileno()
            self._old_terminal_settings = termios.tcgetattr(self._fd)
            self._old_file_flags = fcntl.fcntl(self._fd, fcntl.F_GETFL)

            # cbreak-like mode: no line buffering, no echo, but keep output
            # processing so '\n' / ANSI redraws work correctly.
            new_settings = termios.tcgetattr(self._fd)
            new_settings[3] &= ~(termios.ICANON | termios.ECHO)
            new_settings[6][termios.VMIN] = 0
            new_settings[6][termios.VTIME] = 0
            termios.tcsetattr(self._fd, termios.TCSADRAIN, new_settings)
            fcntl.fcntl(self._fd, fcntl.F_SETFL, self._old_file_flags | os.O_NONBLOCK)

            self._input_enabled = True
            print("[INFO] Keyboard control enabled")
            print("       UP/DOWN speed, LEFT/RIGHT steering, S front, O override, Q quit")
            print("       fallback: W/X speed, A/D steering, S front, O override, Q quit")
        except Exception as e:
            self.enabled = False
            self._restore_terminal()
            print("[WARN] Keyboard control init failed:", e)

    def _restore_terminal(self):
        if self._fd is not None and self._old_file_flags is not None:
            try:
                fcntl.fcntl(self._fd, fcntl.F_SETFL, self._old_file_flags)
            except Exception:
                pass
        if self._fd is not None and self._old_terminal_settings is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_terminal_settings)
            except Exception:
                pass

    def stop(self):
        self._restore_terminal()
        self._input_enabled = False

    def _speed_up(self):
        self.speed_kmh = min(
            KEYBOARD_MAX_SPEED_KMH,
            self.speed_kmh + KEYBOARD_SPEED_STEP_KMH,
        )
        self.last_key_text = "Speed Up"

    def _speed_down(self):
        self.speed_kmh = max(
            0.0,
            self.speed_kmh - KEYBOARD_SPEED_STEP_KMH,
        )
        self.last_key_text = "Speed Down"

    def _steer_left(self):
        self.steering_angle_deg = -abs(KEYBOARD_STEERING_ANGLE_DEG)
        self.last_key_text = "Steer Left"

    def _steer_right(self):
        self.steering_angle_deg = abs(KEYBOARD_STEERING_ANGLE_DEG)
        self.last_key_text = "Steer Right"

    def _steer_front(self):
        self.steering_angle_deg = 0.0
        self.last_key_text = "Steer Front"

    def _toggle_driver_override(self):
        self.driver_override = not self.driver_override
        self.last_key_text = "Override ON" if self.driver_override else "Override OFF"

    def _request_quit(self):
        global running
        running = False
        self.last_key_text = "Quit"

    def _consume_one_key(self):
        if not self._key_buffer:
            return False

        # Arrow keys: ESC [ A/B/C/D or ESC O A/B/C/D.
        # Robustly handle partial/unknown ESC sequences so the input buffer
        # never gets stuck.
        if self._key_buffer[0] == 0x1B:
            if len(self._key_buffer) < 2:
                return False

            if self._key_buffer[1] not in (ord("["), ord("O")):
                del self._key_buffer[0]
                return True

            if len(self._key_buffer) < 3:
                return False

            seq = bytes(self._key_buffer[:3])
            if seq in (b"\x1b[A", b"\x1bOA"):
                del self._key_buffer[:3]
                self._speed_up()
                return True
            if seq in (b"\x1b[B", b"\x1bOB"):
                del self._key_buffer[:3]
                self._speed_down()
                return True
            if seq in (b"\x1b[D", b"\x1bOD"):
                del self._key_buffer[:3]
                self._steer_left()
                return True
            if seq in (b"\x1b[C", b"\x1bOC"):
                del self._key_buffer[:3]
                self._steer_right()
                return True

            del self._key_buffer[0]
            return True

        ch = chr(self._key_buffer[0])
        del self._key_buffer[0]

        if ch in {"w", "W"}:
            self._speed_up()
        elif ch in {"x", "X"}:
            self._speed_down()
        elif ch in {"a", "A"}:
            self._steer_left()
        elif ch in {"d", "D"}:
            self._steer_right()
        elif ch in {"s", "S"}:
            self._steer_front()
        elif ch in {"o", "O"}:
            self._toggle_driver_override()
        elif ch in {"q", "Q"}:
            self._request_quit()
        return True

    def poll(self):
        if not self.enabled or not self._input_enabled or self._fd is None:
            return

        while True:
            try:
                chunk = os.read(self._fd, 64)
            except BlockingIOError:
                break
            except Exception as e:
                runtime_warn("keyboard_read", f"[WARN] Keyboard read failed: {e}", 3.0)
                return

            if not chunk:
                break
            self._key_buffer.extend(chunk)

        guard = 0
        while self._key_buffer and guard < 64:
            before = len(self._key_buffer)
            consumed = self._consume_one_key()
            guard += 1
            if not consumed or len(self._key_buffer) == before:
                break

        if len(self._key_buffer) > 16:
            del self._key_buffer[:-16]

    def get_speed_mps(self):
        # speed_kmh is entered/displayed as real-car-equivalent speed.
        # The actual 1/32 RC physical speed is scaled down by 32.
        real_equiv_mps = max(0.0, self.speed_kmh) / 3.6
        if REAL_EQUIVALENT_SPEED_MODE:
            return real_equiv_mps / MODEL_SCALE_DENOMINATOR
        return real_equiv_mps

    def get_steering_angle_deg(self):
        return self.steering_angle_deg

    def get_driver_override(self):
        return bool(self.driver_override)


keyboard_ego = KeyboardEgoController()


def get_ego_speed_mps():
    """
    Keyboard-controlled simulated speed for test bench.
    Replace this with encoder / CAN speed / motor command based speed later.
    """
    if keyboard_ego.enabled:
        return keyboard_ego.get_speed_mps()
    return DEFAULT_EGO_SPEED_MPS


def get_steering_angle_deg():
    """
    Keyboard-controlled simulated steering.
    Positive = right steering, negative = left steering.
    Replace this with steering angle sensor or servo command later.
    """
    if keyboard_ego.enabled:
        return keyboard_ego.get_steering_angle_deg()
    return DEFAULT_STEERING_ANGLE_DEG


def get_driver_override():
    """
    Return True when manual override / emergency stop / driver braking is active.
    In the keyboard test bench, O toggles this value.
    """
    if keyboard_ego.enabled:
        return keyboard_ego.get_driver_override()
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
        self.serial_port = None
        self.can_bus = None
        self.last_can_frame_text = "--"
        self.last_uart_rx_text = "--"
        self.last_uart_parse_text = "--"
        self.last_uart_port_text = "--"
        self.last_i2c_scan_text = "--"
        self.last_i2c_error_text = "--"
        self.uart_buffer = bytearray()
        self.interface = TOF_INTERFACE.upper()

        if not USE_TOF:
            print("[INFO] Front ToF disabled by config.")
            return

        if self.interface == "CAN":
            self._init_can()
            return

        if self.interface == "UART":
            self._init_uart()
            return

        if self.interface != "I2C":
            print(f"[WARN] Unsupported ToF interface: {TOF_INTERFACE}")
            return

        self._init_i2c()

    def _init_can(self):
        if not CAN_AVAILABLE:
            print("[WARN] python-can unavailable. Front CAN ToF disabled.")
            return

        try:
            self.can_bus = can.interface.Bus(
                channel=TOF_CAN_CHANNEL,
                interface="socketcan",
                bitrate=TOF_CAN_BITRATE,
            )
            self.sensor_model = "CAN"
            print(f"[INFO] Front ToF loaded: CAN {TOF_CAN_CHANNEL} @ {TOF_CAN_BITRATE}")
        except Exception as e:
            print("[WARN] CAN ToF init failed:", e)
            self.can_bus = None

    def _init_uart(self):
        if not SERIAL_AVAILABLE:
            print("[WARN] pyserial unavailable. Front UART ToF disabled.")
            return

        baud_candidates = [TOF_UART_BAUD]
        if TOF_UART_AUTO_DETECT_BAUD:
            baud_candidates = []
            for baud in [TOF_UART_BAUD] + TOF_UART_BAUD_CANDIDATES:
                if baud not in baud_candidates:
                    baud_candidates.append(baud)

        port_candidates = []
        for port in [TOF_UART_PORT] + TOF_UART_PORT_CANDIDATES:
            if port not in port_candidates:
                port_candidates.append(port)

        last_error = None
        opened_without_bytes = []

        for port in port_candidates:
            for baud in baud_candidates:
                try:
                    candidate = serial.Serial(
                        port=port,
                        baudrate=baud,
                        timeout=TOF_UART_TIMEOUT_SEC,
                    )
                    candidate.reset_input_buffer()

                    if not TOF_UART_AUTO_DETECT_BAUD:
                        self.serial_port = candidate
                        self.sensor_model = "UART"
                        self.last_uart_port_text = f"{port} @ {baud}"
                        print(f"[INFO] Front ToF loaded: UART {self.last_uart_port_text}")
                        return

                    deadline = time.perf_counter() + TOF_UART_AUTODETECT_SEC
                    detected = bytearray()
                    while time.perf_counter() < deadline:
                        waiting = candidate.in_waiting
                        chunk = candidate.read(min(waiting, 64) if waiting else 16)
                        if chunk:
                            detected.extend(chunk)
                            break

                    if detected:
                        self.serial_port = candidate
                        self.sensor_model = "UART"
                        self.last_uart_port_text = f"{port} @ {baud}"
                        self.last_uart_rx_text = " ".join(f"{b:02X}" for b in detected[-32:])
                        print(f"[INFO] Front ToF loaded: UART {self.last_uart_port_text}")
                        print(f"[INFO] Front ToF first bytes: {self.last_uart_rx_text}")
                        return

                    candidate.close()
                    opened_without_bytes.append(f"{port}@{baud}")
                except Exception as e:
                    last_error = e

        if TOF_UART_AUTO_DETECT_BAUD:
            print(
                "[WARN] UART ToF received no bytes on scanned ports/baudrates: "
                + ", ".join(opened_without_bytes[:12])
                + (" ..." if len(opened_without_bytes) > 12 else "")
            )
            if last_error is not None:
                print("[WARN] Last UART ToF init error:", last_error)

        try:
            self.serial_port = serial.Serial(
                port=TOF_UART_PORT,
                baudrate=TOF_UART_BAUD,
                timeout=TOF_UART_TIMEOUT_SEC,
            )
            self.sensor_model = "UART"
            self.last_uart_port_text = f"{TOF_UART_PORT} @ {TOF_UART_BAUD}"
            print(f"[INFO] Front ToF loaded: UART {self.last_uart_port_text}")
        except Exception as e:
            print("[WARN] UART ToF init failed:", e)
            self.serial_port = None

    def _init_i2c(self):
        if not I2C_AVAILABLE:
            self.last_i2c_error_text = I2C_IMPORT_ERROR or "I2C unavailable"
            print("[WARN] I2C unavailable. Front ToF disabled.")
            return

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            self.last_i2c_error_text = f"I2C init failed: {e}"
            print("[WARN] I2C init failed. Front ToF disabled:", e)
            self.i2c = None
            return

        self._scan_i2c_bus()
        requested = TOF_SENSOR_MODEL.upper()

        if requested in {"AUTO", "VL53L0X"} and VL53L0X_AVAILABLE:
            try:
                self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=TOF_I2C_ADDRESS)
                self.sensor_model = "VL53L0X"
                print("[INFO] Front ToF loaded: VL53L0X")
                return
            except Exception as e:
                self.last_i2c_error_text = f"VL53L0X init failed: {e}"
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
                self.last_i2c_error_text = f"VL53L1X init failed: {e}"
                print("[WARN] VL53L1X init failed:", e)

        if not VL53L0X_AVAILABLE and not VL53L1X_AVAILABLE:
            self.last_i2c_error_text = " | ".join(
                msg for msg in [VL53L0X_IMPORT_ERROR, VL53L1X_IMPORT_ERROR] if msg
            ) or "VL53L0X/VL53L1X libraries unavailable"
        print("[WARN] No supported front ToF driver available.")

    def _scan_i2c_bus(self):
        if self.i2c is None:
            self.last_i2c_scan_text = "--"
            return

        locked = False
        try:
            deadline = time.perf_counter() + 1.0
            while not self.i2c.try_lock():
                if time.perf_counter() > deadline:
                    self.last_i2c_scan_text = "lock timeout"
                    return
                time.sleep(0.01)
            locked = True
            addresses = self.i2c.scan()
            if addresses:
                self.last_i2c_scan_text = " ".join(f"0x{addr:02X}" for addr in addresses)
            else:
                self.last_i2c_scan_text = "none"
            print(f"[INFO] I2C scan: {self.last_i2c_scan_text}")
        except Exception as e:
            self.last_i2c_scan_text = f"scan failed: {e}"
            self.last_i2c_error_text = self.last_i2c_scan_text
            print("[WARN] I2C scan failed:", e)
        finally:
            if locked:
                try:
                    self.i2c.unlock()
                except Exception:
                    pass

    def read_raw_mm(self):
        if self.sensor_model == "CAN":
            return self._read_can_mm()

        if self.sensor_model == "UART":
            return self._read_uart_mm()

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
            self.last_i2c_error_text = f"{self.sensor_model} read failed: {e}"
            runtime_warn("front_tof_read", f"[WARN] Front ToF read failed: {e}", 2.0)
            return None

        distance_m = mm / 1000.0
        if distance_m < TOF_MIN_M or distance_m > TOF_MAX_M:
            self.last_i2c_error_text = f"{self.sensor_model} no valid target: {distance_m:.2f}m"
            return None
        self.last_i2c_error_text = f"{self.sensor_model} OK"
        return mm

    def _read_can_mm(self):
        if self.can_bus is None:
            return None

        for _ in range(8):
            try:
                msg = self.can_bus.recv(timeout=TOF_CAN_TIMEOUT_SEC)
            except Exception as e:
                runtime_warn("can_tof_read", f"[WARN] CAN ToF read failed: {e}", 2.0)
                return None

            if msg is None:
                return None

            data = bytes(msg.data)
            self.last_can_frame_text = (
                f"id=0x{msg.arbitration_id:X} "
                f"dlc={msg.dlc} "
                f"data={' '.join(f'{b:02X}' for b in data)}"
            )

            if TOF_CAN_ID is not None and msg.arbitration_id != TOF_CAN_ID:
                continue

            start = TOF_CAN_DISTANCE_OFFSET
            end = start + TOF_CAN_DISTANCE_LENGTH
            if len(data) < end:
                continue

            raw = int.from_bytes(data[start:end], byteorder=TOF_CAN_DISTANCE_ENDIAN, signed=False)
            mm = float(raw) * TOF_CAN_DISTANCE_SCALE_MM
            distance_m = mm / 1000.0

            if TOF_MIN_M <= distance_m <= TOF_MAX_M:
                return mm

        return None

    def _read_uart_mm(self):
        if self.serial_port is None:
            return None

        try:
            waiting = self.serial_port.in_waiting
            if waiting:
                chunk = self.serial_port.read(min(waiting, 64))
            else:
                chunk = self.serial_port.read(16)
        except Exception as e:
            runtime_warn("uart_tof_read", f"[WARN] UART ToF read failed: {e}", 2.0)
            return None

        if chunk:
            self.uart_buffer.extend(chunk)
            shown = bytes(self.uart_buffer[-32:])
            self.last_uart_rx_text = " ".join(f"{b:02X}" for b in shown)

        protocol = TOF_UART_PROTOCOL.upper()
        mm = None

        if protocol in {"AUTO", "NOOPLOOP"}:
            mm = self._parse_nooploop_uart_mm()

        if mm is None and protocol in {"AUTO", "BENWAKE"}:
            mm = self._parse_benewake_uart_mm()

        if mm is None and protocol in {"AUTO", "ASCII"}:
            mm = self._parse_ascii_uart_mm()

        if len(self.uart_buffer) > 128:
            del self.uart_buffer[:-32]

        if mm is None:
            if self.uart_buffer:
                self.last_uart_parse_text = f"buffer={len(self.uart_buffer)}B, no valid distance frame"
            return None

        distance_m = mm / 1000.0
        if distance_m < TOF_MIN_M or distance_m > TOF_MAX_M:
            self.last_uart_parse_text = f"parsed {mm:.0f}mm out of range"
            return None
        self.last_uart_parse_text = f"distance={mm:.0f}mm"
        return mm

    def _parse_nooploop_uart_mm(self):
        # Nooploop TOFSense products use binary frames headed by 0x57 on many
        # firmwares. Distance is commonly a little-endian float in meters.
        frame_lengths = (16, 15, 17, 18)

        while len(self.uart_buffer) >= min(frame_lengths):
            if self.uart_buffer[0] != 0x57:
                del self.uart_buffer[0]
                continue

            for frame_len in frame_lengths:
                if len(self.uart_buffer) < frame_len:
                    continue

                frame = bytes(self.uart_buffer[:frame_len])
                checksum = sum(frame[:-1]) & 0xFF
                checksum_ok = checksum == frame[-1]

                for offset in range(3, min(frame_len - 4, 10)):
                    value = self._bytes_to_float_le(frame[offset:offset + 4])
                    if value is None:
                        continue
                    if TOF_MIN_M <= value <= TOF_MAX_M:
                        # Trust a valid checksum when present. If firmware uses a
                        # different length/checksum, still accept plausible 0x57 frames.
                        del self.uart_buffer[:frame_len]
                        self.last_uart_parse_text = (
                            f"nooploop offset={offset}, checksum={'OK' if checksum_ok else 'unchecked'}"
                        )
                        return value * 1000.0

            if len(self.uart_buffer) > max(frame_lengths):
                del self.uart_buffer[0]
            else:
                return None

        return None

    def _bytes_to_float_le(self, data):
        if len(data) != 4:
            return None
        try:
            import struct
            value = struct.unpack("<f", data)[0]
        except Exception:
            return None
        if math.isnan(value) or math.isinf(value):
            return None
        return value

    def _parse_benewake_uart_mm(self):
        while len(self.uart_buffer) >= 9:
            if self.uart_buffer[0] != 0x59 or self.uart_buffer[1] != 0x59:
                del self.uart_buffer[0]
                continue

            frame = self.uart_buffer[:9]
            checksum = sum(frame[:8]) & 0xFF
            if checksum != frame[8]:
                del self.uart_buffer[0]
                continue

            distance_cm = frame[2] | (frame[3] << 8)
            del self.uart_buffer[:9]
            return float(distance_cm) * 10.0

        return None

    def _parse_ascii_uart_mm(self):
        for newline in (10, 13):
            if newline in self.uart_buffer:
                idx = self.uart_buffer.index(newline)
                raw_line = bytes(self.uart_buffer[:idx]).decode("ascii", errors="ignore").strip()
                del self.uart_buffer[:idx + 1]

                token = ""
                for ch in raw_line:
                    if ch.isdigit() or ch == ".":
                        token += ch
                    elif token:
                        break

                if not token:
                    return None

                value = float(token)
                # Treat small decimal values as meters, medium values as cm,
                # and large values as mm so common UART ToF text formats work.
                if value <= 10.0:
                    return value * 1000.0
                if value <= 500.0:
                    return value * 10.0
                return value

        return None

    def update(self):
        now = time.perf_counter()
        sensor_ok = (
            self.sensor is not None
            or self.serial_port is not None
            or self.can_bus is not None
        )
        raw_mm = self.read_raw_mm()

        if raw_mm is not None:
            raw_mm *= TOF_DISTANCE_SCALE
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
                sensor_ok=sensor_ok,
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
            sensor_ok=sensor_ok,
            timestamp=now,
        )

    def release(self):
        if self.can_bus is not None:
            try:
                self.can_bus.shutdown()
            except Exception:
                pass

        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass

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
            try:
                # Avoid old camera frames piling up and causing apparent latency.
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
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

    def _normalize_frame(self, frame):
        if frame is None or not CV2_AVAILABLE:
            return frame
        h, w = frame.shape[:2]
        if w == FRAME_WIDTH and h == FRAME_HEIGHT:
            return frame
        return cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_AREA)

    def read(self):
        if not CV2_AVAILABLE:
            return None

        if self.use_picamera2 and self.picam2 is not None:
            frame = self.picam2.capture_array()
            return self._normalize_frame(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

        if self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                return self._normalize_frame(frame)
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
        # Fast mode: only detect people.
        self.target_labels = {"person"}

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
            classes=YOLO_TARGET_CLASS_IDS,
            max_det=YOLO_MAX_DET,
            device="cpu",
            half=False,
            augment=False,
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


class AsyncYOLOWorker:
    """
    Runs YOLO outside the real-time AEB loop.

    The safety loop must never wait for neural-network inference. This worker
    keeps only the newest frame, drops backlog, and exposes the latest finished
    detections to the main loop. If YOLO is slow, ToF/ultrasonic based braking
    still runs at LOOP_HZ.
    """

    def __init__(self, detector):
        self.detector = detector
        self.enabled = detector is not None and detector.enabled

        self.frame_lock = threading.Lock()
        self.result_lock = threading.Lock()

        self.latest_frame = None
        self.latest_frame_seq = 0

        self.latest_detections = []
        self.latest_result_seq = -1
        self.latest_result_time = 0.0

        self.running = False
        self.thread = None

    def start(self):
        if not self.enabled:
            return
        self.running = True
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()
        print("[INFO] Async YOLO worker started")

    def submit_frame(self, frame):
        if not self.enabled or frame is None:
            return

        # No queue: keep only the newest frame to avoid inference backlog.
        with self.frame_lock:
            self.latest_frame = frame.copy()
            self.latest_frame_seq += 1

    def get_result(self):
        if not self.enabled:
            return -1, [], 0.0
        with self.result_lock:
            return (
                self.latest_result_seq,
                list(self.latest_detections),
                self.latest_result_time,
            )

    def _worker_loop(self):
        last_processed_seq = -1

        while self.running:
            frame = None
            seq = -1

            with self.frame_lock:
                if self.latest_frame is not None and self.latest_frame_seq != last_processed_seq:
                    frame = self.latest_frame
                    seq = self.latest_frame_seq

            if frame is None:
                time.sleep(0.005)
                continue

            try:
                detections = self.detector.detect(frame)
            except Exception as e:
                runtime_warn("async_yolo_detect", f"[WARN] Async YOLO detect failed: {e}", 2.0)
                detections = []

            now = time.perf_counter()
            with self.result_lock:
                self.latest_detections = detections
                self.latest_result_seq = seq
                self.latest_result_time = now

            last_processed_seq = seq

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)


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

    def _to_real_distance(self, distance_m):
        return max(0.0, distance_m) * MODEL_SCALE_DENOMINATOR if REAL_EQUIVALENT_SPEED_MODE else max(0.0, distance_m)

    def _to_real_speed(self, speed_mps):
        return max(0.0, speed_mps) * MODEL_SCALE_DENOMINATOR if REAL_EQUIVALENT_SPEED_MODE else max(0.0, speed_mps)

    def _from_real_distance(self, distance_m):
        return max(0.0, distance_m) / MODEL_SCALE_DENOMINATOR if REAL_EQUIVALENT_SPEED_MODE else max(0.0, distance_m)

    def stopping_distance(self, v_ego):
        # Compute in real-car-equivalent domain, then scale distance back to RC.
        v_real = self._to_real_speed(v_ego)
        stop_real = (
            v_real * REAL_SYSTEM_DELAY_SEC
            + (v_real ** 2) / (2.0 * max(0.01, REAL_MAX_DECEL_MPS2))
            + REAL_SAFETY_MARGIN_M
        )
        return self._from_real_distance(stop_real)

    def required_decel(self, distance_m, closing_speed_mps):
        # Required deceleration is kept in real-car units, so thresholds such as
        # 4.5m/s^2 or 7.0m/s^2 remain meaningful and presentation-friendly.
        remain_real = max(0.05, self._to_real_distance(distance_m) - REAL_SAFETY_MARGIN_M)
        closing_real = self._to_real_speed(closing_speed_mps)
        if closing_real <= 0:
            return 0.0
        return (closing_real ** 2) / (2.0 * remain_real)

    def ttc_longitudinal(self, distance_m, closing_speed_mps):
        if closing_speed_mps <= 0.003:
            return 999.0
        # TTC is scale-invariant when both distance and speed are scaled by 1/32.
        return distance_m / closing_speed_mps

    def ego_is_moving(self, v_ego):
        return self._to_real_speed(v_ego) * 3.6 >= REAL_MIN_AEB_SPEED_KMH

    def controllable_closing_speed(self, measured_closing_mps, v_ego, fallback_speed_mps=0.0):
        if not self.ego_is_moving(v_ego):
            return 0.0

        # AEB braking can only reduce the ego vehicle's contribution to closing.
        # If an outside object moves toward a stopped car, measured closing exists
        # but ego-controllable closing is zero.
        measured_closing_mps = max(0.0, measured_closing_mps)
        fallback_speed_mps = max(0.0, fallback_speed_mps)
        ego_contribution = min(measured_closing_mps, v_ego)
        fallback_contribution = min(fallback_speed_mps, v_ego)
        return max(ego_contribution, fallback_contribution)

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
        measured_closing = max(closing_from_camera, closing_from_tof, closing_from_ultra)

        # Fallback for near in-path object when velocity estimate is unstable.
        fallback = 0.0
        if obj.in_path_now and obj.z_m < 2.0 and measured_closing < 0.05:
            fallback = v_ego * 0.50

        return self.controllable_closing_speed(measured_closing, v_ego, fallback)

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
        ego_moving = self.ego_is_moving(v_ego)

        t_path = self.time_to_path_entry(obj, steering_angle_deg)
        t_ego = self.ego_time_to_conflict_z(obj, v_ego)
        time_gap = abs(t_ego - t_path) if t_path < 900 and t_ego < 900 else 999.0

        p = 0.0

        if obj.in_path_now and ego_moving:
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

        if threat_type in {ThreatType.CROSSING_VRU, ThreatType.CUT_IN} and ego_moving:
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

        if not ego_moving:
            reason = (
                f"Ego stopped: vEgo={v_ego:.2f}m/s, "
                "no controllable AEB brake needed"
            )

        # Longitudinal AEB: object is currently in predicted path.
        # Strict mode prevents early/annoying engagement. AEB braking requires
        # either a very close target, tight TTC, high required decel, or a
        # speed-adaptive stopping envelope that clearly exceeds target range.
        if obj.in_path_now and ego_moving:
            full_by_distance = distance_m <= UNKNOWN_STOP_DISTANCE_M
            full_by_envelope = stop_dist >= distance_m * STRICT_FULL_STOP_DIST_GAIN
            partial_by_envelope = stop_dist >= distance_m * STRICT_PARTIAL_STOP_DIST_GAIN
            fcw_by_envelope = stop_dist >= distance_m * STRICT_FCW_STOP_DIST_GAIN

            if full_by_distance or ttc <= FULL_TTC_SEC or req_decel >= FULL_REQ_DECEL or full_by_envelope:
                desired_state = AEBState.FULL_BRAKE
                brake = 100
                reason = (
                    f"Longitudinal FULL(strict): TTC={ttc:.2f}s, reqDecel={req_decel:.2f}, "
                    f"stopDist={stop_dist:.2f}m, dist={distance_m:.2f}m"
                )
            elif ttc <= PARTIAL_TTC_SEC or req_decel >= PARTIAL_REQ_DECEL or partial_by_envelope:
                desired_state = AEBState.PARTIAL_BRAKE
                brake = 60
                reason = (
                    f"Longitudinal PARTIAL(strict): TTC={ttc:.2f}s, reqDecel={req_decel:.2f}, "
                    f"stopDist={stop_dist:.2f}m, dist={distance_m:.2f}m"
                )
            elif req_decel >= PREFILL_REQ_DECEL:
                desired_state = AEBState.BRAKE_PREFILL
                brake = 20
                reason = f"Longitudinal PREFILL(strict): reqDecel={req_decel:.2f}, dist={distance_m:.2f}m"
            elif ttc <= FCW_TTC_SEC or req_decel >= FCW_REQ_DECEL or fcw_by_envelope:
                desired_state = AEBState.FCW
                brake = 0
                reason = (
                    f"Longitudinal FCW(strict): TTC={ttc:.2f}s, reqDecel={req_decel:.2f}, "
                    f"stopDist={stop_dist:.2f}m, dist={distance_m:.2f}m"
                )

        # Crossing pedestrian/cyclist/object.
        if threat_type == ThreatType.CROSSING_VRU and ego_moving:
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
        if threat_type == ThreatType.CUT_IN and ego_moving:
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
        front_available = tof_reading is not None and tof_reading.sensor_ok
        front_valid = tof_reading is not None and tof_reading.valid and tof_reading.distance_m is not None
        stop_dist = self.stopping_distance(v_ego)
        ego_moving = self.ego_is_moving(v_ego)

        if not front_available and not valid_ultra:
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
            closing = self.controllable_closing_speed(
                tof_reading.approach_speed_mps,
                v_ego,
                v_ego * 0.55,
            )
            ttc = self.ttc_longitudinal(distance_m, closing)
            req_decel = self.required_decel(distance_m, closing)

            desired = AEBState.MONITORING
            brake = 0
            probability = 0.0
            reason = f"Front ToF clear: {distance_m:.2f}m"

            if (
                ego_moving
                and (
                    distance_m <= UNKNOWN_STOP_DISTANCE_M
                    or ttc <= FULL_TTC_SEC
                    or req_decel >= FULL_REQ_DECEL
                    or stop_dist >= distance_m * STRICT_FULL_STOP_DIST_GAIN
                )
            ):
                desired = AEBState.FULL_BRAKE
                brake = 100
                probability = 0.95
                reason = (
                    f"Front ToF unknown FULL: dist={distance_m:.2f}m, "
                    f"TTC={ttc:.2f}s, reqDecel={req_decel:.2f}"
                )
            elif ego_moving and (
                ttc <= PARTIAL_TTC_SEC
                or req_decel >= PARTIAL_REQ_DECEL
                or stop_dist >= distance_m * STRICT_PARTIAL_STOP_DIST_GAIN
            ):
                desired = AEBState.PARTIAL_BRAKE
                brake = 60
                probability = 0.70
                reason = (
                    f"Front ToF unknown PARTIAL: dist={distance_m:.2f}m, "
                    f"TTC={ttc:.2f}s, reqDecel={req_decel:.2f}"
                )
            elif ego_moving and (
                ttc <= FCW_TTC_SEC
                or req_decel >= FCW_REQ_DECEL
                or stop_dist >= distance_m * STRICT_FCW_STOP_DIST_GAIN
            ):
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
            closing = self.controllable_closing_speed(ultra_closing, v_ego, v_ego * 0.40)
            ttc = self.ttc_longitudinal(distance_m, closing)
            req_decel = self.required_decel(distance_m, closing)

            desired = AEBState.MONITORING
            brake = 0
            probability = 0.0
            reason = f"Diagonal ultrasonic clear: {nearest.name} {nearest.distance_m:.2f}m"

            if ego_moving and (
                nearest.distance_m <= DIAGONAL_FULL_BRAKE_DISTANCE_M
                or ttc <= FULL_TTC_SEC
            ):
                desired = AEBState.FULL_BRAKE
                brake = 100
                probability = 0.88
                reason = (
                    f"Diagonal unknown FULL: {nearest.name} direct={nearest.distance_m:.2f}m, "
                    f"forward={distance_m:.2f}m, TTC={ttc:.2f}s"
                )
            elif ego_moving and (
                nearest.distance_m <= DIAGONAL_PARTIAL_BRAKE_DISTANCE_M
                or ttc <= PARTIAL_TTC_SEC
            ):
                desired = AEBState.PARTIAL_BRAKE
                brake = 60
                probability = 0.62
                reason = (
                    f"Diagonal unknown PARTIAL: {nearest.name} direct={nearest.distance_m:.2f}m, "
                    f"forward={distance_m:.2f}m, TTC={ttc:.2f}s"
                )
            elif ego_moving and (
                nearest.distance_m <= DIAGONAL_FCW_DISTANCE_M
                or ttc <= FCW_TTC_SEC
            ):
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

        if not front_valid and not front_available:
            nearest_distance = (nearest.forward_distance_m or nearest.distance_m) if nearest is not None else None
            nearest_ttc = 999.0
            if nearest is not None:
                nearest_closing = self.controllable_closing_speed(
                    ultrasonic_forward_approach_component(
                        nearest.name,
                        nearest.approach_speed_mps,
                    ),
                    v_ego,
                )
                nearest_ttc = self.ttc_longitudinal(nearest_distance, nearest_closing)
            return AdvancedRisk(
                threat_type=ThreatType.SENSOR_FAULT,
                desired_state=AEBState.DEGRADED,
                brake_percent=0,
                reason="Front ToF unavailable; running camera + diagonal ultrasonic degraded mode",
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

        if not front_valid:
            nearest_distance = (nearest.forward_distance_m or nearest.distance_m) if nearest is not None else None
            nearest_ttc = 999.0
            if nearest is not None:
                nearest_closing = self.controllable_closing_speed(
                    ultrasonic_forward_approach_component(
                        nearest.name,
                        nearest.approach_speed_mps,
                    ),
                    v_ego,
                )
                nearest_ttc = self.ttc_longitudinal(nearest_distance, nearest_closing)
            return AdvancedRisk(
                threat_type=ThreatType.NONE,
                desired_state=AEBState.MONITORING,
                brake_percent=0,
                reason="Front ToF has no valid target in range",
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

        nearest_distance = tof_reading.distance_m
        nearest_closing = self.controllable_closing_speed(
            tof_reading.approach_speed_mps,
            v_ego,
        )
        nearest_ttc = self.ttc_longitudinal(nearest_distance, nearest_closing)
        reason = "No object threat"
        if not ego_moving:
            reason = (
                f"Ego stopped: vEgo={v_ego:.2f}m/s, "
                "no controllable AEB brake needed"
            )

        return AdvancedRisk(
            threat_type=ThreatType.NONE,
            desired_state=AEBState.MONITORING,
            brake_percent=0,
            reason=reason,
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
        self.last_event_print_time = 0.0
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
        now = time.perf_counter()
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

        changed = brake_percent != self.last_brake or state != self.last_state
        periodic_critical = (
            state in {AEBState.FULL_BRAKE, AEBState.STOP_HOLD}
            and now - self.last_event_print_time >= AEB_EVENT_PRINT_INTERVAL_SEC
        )
        dashboard_mode = PRINT_SENSOR_VALUES and PRETTY_TERMINAL_OUTPUT
        should_print = changed or periodic_critical

        if not dashboard_mode and (important or should_print):
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
            self.last_event_print_time = now
        elif dashboard_mode and periodic_critical:
            # In dashboard mode, avoid extra asynchronous lines that corrupt the redraw.
            self.last_event_print_time = now

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
            runtime_warn("can_send", f"[WARN] CAN send failed: {e}", 2.0)

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
        tof_cm = tof_reading.distance_m * 100.0 if tof_reading.distance_m is not None else None
        lines.append(
            f"FRONT_TOF: {fmt(tof_cm, 1)}cm, "
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



def cm_value_from_m(distance_m):
    if distance_m is None:
        return None
    if not math.isfinite(distance_m):
        return None
    return distance_m * 100.0


def calc_sensor_fusion_status(ultrasonic_readings, tof_reading, objects, yolo_result_age):
    """
    Production-style readiness indicator.

    Important fix:
    A front ToF sensor with "no target in range" is NOT a sensor fault.
    Sensor health is evaluated by sensor availability, while target distance is
    evaluated separately by the AEB risk logic.
    """
    front_alive = bool(tof_reading is not None and tof_reading.sensor_ok)
    front_target = bool(
        front_alive
        and tof_reading.valid
        and tof_reading.distance_m is not None
    )

    left = ultrasonic_readings.get(LEFT_DIAGONAL_NAME)
    right = ultrasonic_readings.get(RIGHT_DIAGONAL_NAME)
    left_ok = bool(left is not None and left.valid and left.distance_m is not None)
    right_ok = bool(right is not None and right.valid and right.distance_m is not None)

    camera_ok = bool(
        objects
        or (
            yolo_result_age is not None
            and math.isfinite(yolo_result_age)
            and yolo_result_age <= SENSOR_FUSION_CAMERA_STALE_SEC
        )
    )

    score = 0
    score += 55 if front_alive else 0
    score += 10 if front_target else 0
    score += 15 if left_ok else 0
    score += 15 if right_ok else 0
    score += 5 if camera_ok else 0
    score = int(clamp(score, 0, 100))

    if front_alive and score >= SENSOR_FUSION_MIN_CONFIDENCE_OK:
        status = "OK"
    elif score >= SENSOR_FUSION_MIN_CONFIDENCE_DEGRADED:
        status = "DEGRADED"
    else:
        status = "FAULT"

    details = []
    details.append("Front" if front_alive else "Front--")
    details.append("Target" if front_target else "Target--")
    details.append("Left" if left_ok else "Left--")
    details.append("Right" if right_ok else "Right--")
    details.append("Camera" if camera_ok else "Camera--")
    return status, score, "/".join(details)


def calc_aeb_mode(state, driver_override, sensor_status):
    if driver_override:
        return "OVERRIDE"
    if state == AEBState.SENSOR_FAULT or sensor_status == "FAULT":
        return "FAULT"
    if state == AEBState.DEGRADED or sensor_status == "DEGRADED":
        return "DEGRADED"
    return "NORMAL"


def calc_risk_level(state, brake_percent, risk, ego_speed=None):
    # If the car is not moving, do not label a close object as CRITICAL only
    # because the static safety margin is larger than the distance.
    if (
        ego_speed is not None
        and ego_speed <= EGO_MOVING_MIN_SPEED_MPS
        and state not in {AEBState.FULL_BRAKE, AEBState.STOP_HOLD}
        and brake_percent <= 0
    ):
        if state == AEBState.SENSOR_FAULT:
            return "FAULT"
        if state == AEBState.DEGRADED:
            return "DEGRADED"
        return "CLEAR"

    if state in {AEBState.FULL_BRAKE, AEBState.STOP_HOLD} or brake_percent >= 100:
        return "CRITICAL"
    if state == AEBState.PARTIAL_BRAKE or brake_percent >= 60:
        return "HIGH"
    if state == AEBState.BRAKE_PREFILL:
        return "LOW"
    if brake_percent > 0:
        return "MEDIUM"
    if state == AEBState.FCW:
        return "LOW"
    if risk.distance_m is not None and risk.stopping_distance_m > 0:
        ratio = risk.distance_m / max(0.01, risk.stopping_distance_m)
        if ratio <= 1.0:
            return "CRITICAL"
        if ratio <= 1.35:
            return "HIGH"
        if ratio <= 1.75:
            return "MEDIUM"
        if ratio <= 2.25:
            return "LOW"
    return "CLEAR"


def calc_decision_basis(risk):
    reason = risk.reason or ""
    if "stopDist" in reason:
        return "Stopping Distance"
    if "reqDecel" in reason or "req" in reason:
        return "Required Decel"
    if "TTC" in reason:
        return "TTC"
    if "ToF" in reason:
        return "Front Range"
    if "Diagonal" in reason or "ultrasonic" in reason:
        return "Side Range"
    if "Crossing" in reason:
        return "Crossing Prediction"
    if "Cut-in" in reason:
        return "Cut-in Prediction"
    if risk.threat_type == ThreatType.SENSOR_FAULT:
        return "Sensor Health"
    return risk.threat_type.name


def calc_safety_envelope_cm(risk):
    if risk.stopping_distance_m is None:
        return None
    return risk.stopping_distance_m * AEB_SAFETY_ENVELOPE_GAIN * 100.0

def print_sensor_values(
    ultrasonic_readings,
    tof_reading,
    tof_sensor,
    objects,
    risk,
    state,
    brake_percent,
    ego_speed,
    steering_angle,
    driver_override=False,
    yolo_result_age=999.0,
):
    """
    Stable terminal dashboard.
    - Title is exactly DASHBOARD.
    - Sensor names are Front / Left / Right.
    - Distances are in cm.
    - Detected-object display grouped by position and class.
    - Final result line shows AEB emergency braking ON/OFF.
    - Uses a single redraw call to prevent command-line corruption.
    """
    def distance_text(cm, width=7):
        if cm is None or not math.isfinite(cm) or cm > 9000:
            return f"{'---':>{width}} cm"
        return f"{cm:>{width}.0f} cm"

    def steering_text(angle_deg):
        if angle_deg <= -3.0:
            return "Left"
        if angle_deg >= 3.0:
            return "Right"
        return "Front"

    def object_position(obj):
        if obj.in_path_now or abs(obj.x_m) <= path_half_width():
            return "Front"
        if obj.x_m < 0:
            return "Left"
        return "Right"

    def object_count_text(position_key):
        labels = [
            obj.label
            for obj in objects
            if object_position(obj) == position_key
        ]
        if not labels:
            return "none"
        counts = Counter(labels)
        return ", ".join(f"{label} {count}" for label, count in sorted(counts.items()))

    def sensor_health_text(valid, sensor_ok=True):
        if not sensor_ok:
            return "FAULT"
        return "OK" if valid else "NO TARGET"

    front_cm = None
    front_valid = False
    front_sensor_ok = False
    if tof_reading is not None:
        front_sensor_ok = bool(tof_reading.sensor_ok)
        front_valid = bool(tof_reading.valid and tof_reading.filtered_mm is not None)
        if front_valid:
            front_cm = tof_reading.filtered_mm / 10.0

    left = ultrasonic_readings.get(LEFT_DIAGONAL_NAME)
    right = ultrasonic_readings.get(RIGHT_DIAGONAL_NAME)
    left_valid = bool(left is not None and left.valid and left.filtered_cm is not None)
    right_valid = bool(right is not None and right.valid and right.filtered_cm is not None)
    left_cm = left.filtered_cm if left_valid else None
    right_cm = right.filtered_cm if right_valid else None

    rc_speed_kmh = ego_speed * 3.6
    speed_kmh = rc_speed_kmh * MODEL_SCALE_DENOMINATOR if REAL_EQUIVALENT_SPEED_MODE else rc_speed_kmh
    threat_distance_cm = risk.distance_m * 100.0 if risk.distance_m is not None else None

    # Result means actual emergency braking, not early warning or prefill.
    # This avoids "AEB ON" appearing too easily during BRAKE_PREFILL.
    brake_on = (
        brake_percent >= STRICT_EMERGENCY_RESULT_MIN_BRAKE
        or state in {
            AEBState.PARTIAL_BRAKE,
            AEBState.FULL_BRAKE,
            AEBState.STOP_HOLD,
        }
    )
    if driver_override:
        brake_on = False

    result_text = "ON" if brake_on else "OFF"

    sensor_status, sensor_confidence, sensor_detail = calc_sensor_fusion_status(
        ultrasonic_readings=ultrasonic_readings,
        tof_reading=tof_reading,
        objects=objects,
        yolo_result_age=yolo_result_age,
    )
    aeb_mode = calc_aeb_mode(state, driver_override, sensor_status)
    risk_level = calc_risk_level(state, brake_percent, risk, ego_speed=ego_speed)
    decision_basis = calc_decision_basis(risk)
    safety_envelope_cm = calc_safety_envelope_cm(risk)

    warning_text = ""
    if LAST_RUNTIME_WARNING and time.perf_counter() - LAST_RUNTIME_WARNING_TIME <= 5.0:
        warning_text = LAST_RUNTIME_WARNING.replace("\n", " ")[:70]

    last_key = keyboard_ego.last_key_text if keyboard_ego.enabled else "OFF"
    override_text = "ON" if driver_override else "OFF"
    camera_text = "FRESH" if yolo_result_age <= SENSOR_FUSION_CAMERA_STALE_SEC else "STALE"

    lines = [
        "=" * 44,
        "DASHBOARD",
        "=" * 44,
        "Sensors (cm)",
        f"  Front : {distance_text(front_cm)}  {sensor_health_text(front_valid, front_sensor_ok)}",
        f"  Left  : {distance_text(left_cm)}  {'OK' if left_valid else '---'}",
        f"  Right : {distance_text(right_cm)}  {'OK' if right_valid else '---'}",
        "-" * 44,
        "Detected Object",
        f"  Front : {object_count_text('Front')}",
        f"  Left  : {object_count_text('Left')}",
        f"  Right : {object_count_text('Right')}",
        "-" * 44,
        "Vehicle",
        f"  Speed         : {speed_kmh:5.1f} km/h real-eq",
        f"  Steering      : {steering_text(steering_angle)}",
        f"  Risk Distance : {distance_text(threat_distance_cm)}",
        f"  Safe Envelope : {distance_text(safety_envelope_cm)}",
        f"  Scale         : 1/{MODEL_SCALE_DENOMINATOR:.0f} distance model",
        "-" * 44,
        "AEB",
        f"  Risk Level    : {risk_level}",
        f"  Mode          : {aeb_mode}",
        f"  Sensor Fusion : {sensor_status} ({sensor_confidence}%)",
        f"  Fusion Detail : {sensor_detail}",
        f"  Camera        : {camera_text}",
        f"  Decision      : {decision_basis}",
        f"  Override      : {override_text}",
        f"  Last Key      : {last_key}",
    ]

    if warning_text:
        lines.append(f"  Warning       : {warning_text}")

    lines.extend([
        "-" * 44,
        f"Result : AEB Emergency Braking {result_text}",
        "=" * 44,
        "Keys: UP/DOWN speed | LEFT/RIGHT steer | S front | O override | Q quit",
    ])

    write_dashboard(lines)

# ============================================================
# Main
# ============================================================

running = True


def signal_handler(sig, frame):
    global running
    running = False


def tof_config_summary():
    interface = TOF_INTERFACE.upper()
    if interface == "I2C":
        return (
            f"TOF_INTERFACE=I2C, TOF_SENSOR_MODEL={TOF_SENSOR_MODEL}, "
            f"I2C_ADDR=0x{TOF_I2C_ADDRESS:02X}, TOF_SCALE={TOF_DISTANCE_SCALE}, MODEL=1/{MODEL_SCALE_DENOMINATOR:.0f}"
        )
    if interface == "UART":
        return (
            f"TOF_INTERFACE=UART, TOF_UART_PORT={TOF_UART_PORT}, "
            f"TOF_UART_BAUD={TOF_UART_BAUD}, AUTO_BAUD={TOF_UART_AUTO_DETECT_BAUD}"
        )
    if interface == "CAN":
        return f"TOF_INTERFACE=CAN, TOF_CAN_CHANNEL={TOF_CAN_CHANNEL}, TOF_CAN_BITRATE={TOF_CAN_BITRATE}"
    return f"TOF_INTERFACE={TOF_INTERFACE}"


def main():
    global running

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    preview_enabled = preview_display_available()

    print("============================================================")
    print(" Advanced Raspberry Pi 4 AEB")
    print(" Front camera + front ToF + left/right diagonal ultrasonic")
    print("============================================================")
    print(f"[CONFIG] DRY_RUN={DRY_RUN}")
    print(f"[CONFIG] USE_YOLO={USE_YOLO}, YOLO_AVAILABLE={YOLO_AVAILABLE}")
    print(
        f"[CONFIG] CAMERA={FRAME_WIDTH}x{FRAME_HEIGHT}, "
        f"YOLO_MODEL={YOLO_MODEL_PATH}, IMGSZ={YOLO_IMGSZ}, "
        f"YOLO_EVERY_N_FRAMES={YOLO_EVERY_N_FRAMES}"
    )
    print(f"[CONFIG] USE_TOF={USE_TOF}, {tof_config_summary()}")
    print(f"[CONFIG] FOCAL_X={FOCAL_X:.1f}, FOCAL_Y={FOCAL_Y:.1f}")
    print(f"[CONFIG] AVAILABLE_DECEL={AVAILABLE_DECEL_MPS2} m/s^2")
    print(f"[CONFIG] ENABLE_CSV_LOG={ENABLE_CSV_LOG}")
    print(f"[CONFIG] PRINT_SENSOR_VALUES={PRINT_SENSOR_VALUES}")
    print(f"[CONFIG] PRETTY_TERMINAL_OUTPUT={PRETTY_TERMINAL_OUTPUT}")
    print(f"[CONFIG] KEYBOARD_CONTROL_ENABLED={KEYBOARD_CONTROL_ENABLED}")
    print(f"[CONFIG] PRODUCTION_AEB_DEMO_FEATURES={PRODUCTION_AEB_DEMO_FEATURES}")
    print(f"[CONFIG] SHOW_PREVIEW={SHOW_PREVIEW}, PREVIEW_ENABLED={preview_enabled}")
    if SHOW_PREVIEW and not preview_enabled:
        print("[WARN] Preview disabled because no DISPLAY/WAYLAND_DISPLAY was found.")
    print("============================================================")

    keyboard_ego.start()

    ultrasonic = UltrasonicManager()
    tof_sensor = ToFSensor()
    camera = CameraManager()
    detector = ObjectDetector()
    async_yolo = AsyncYOLOWorker(detector)
    async_yolo.start()
    kin_tracker = ObjectKinematicsTracker()
    threat = AdvancedThreatAssessment()
    state_machine = AEBStateMachine()
    brake_controller = BrakeController()
    logger = CSVLogger(LOG_CSV_PATH) if ENABLE_CSV_LOG else None

    frame_count = 0
    last_objects = []
    last_yolo_result_seq = -1
    YOLO_RESULT_STALE_SEC = 0.7
    CAMERA_OBJECT_STALE_SEC = 1.2
    last_sensor_print_time = 0.0
    yolo_interval = max(1, int(YOLO_EVERY_N_FRAMES))
    loop_period = 1.0 / max(1, LOOP_HZ)

    try:
        while running:
            loop_start = time.perf_counter()

            keyboard_ego.poll()
            ego_speed = get_ego_speed_mps()
            steering_angle = get_steering_angle_deg()
            driver_override = get_driver_override()

            ultrasonic_readings = ultrasonic.update_all()
            tof_reading = tof_sensor.update()
            frame = camera.read()

            frame_count += 1
            objects = last_objects

            # Submit frames to YOLO without blocking the AEB safety loop.
            # If YOLO inference takes 1 second, ToF/ultrasonic AEB still runs.
            if frame is not None and async_yolo.enabled:
                if frame_count % yolo_interval == 0:
                    async_yolo.submit_frame(frame)

            # Use a newly completed YOLO result only when it is fresh.
            result_seq, detections, result_time = async_yolo.get_result()
            result_age = time.perf_counter() - result_time if result_time > 0 else 999.0

            if result_seq != last_yolo_result_seq and result_age <= YOLO_RESULT_STALE_SEC:
                objects = kin_tracker.update(
                    detections=detections,
                    ultrasonic_readings=ultrasonic_readings,
                    tof_reading=tof_reading,
                    steering_angle_deg=steering_angle,
                )
                last_objects = objects
                last_yolo_result_seq = result_seq
            elif result_age > CAMERA_OBJECT_STALE_SEC:
                # Do not keep stale camera/YOLO detections forever. Range sensors
                # continue to protect the vehicle while camera inference is late.
                objects = []
                last_objects = []

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

            now = time.perf_counter()
            if PRINT_SENSOR_VALUES and now - last_sensor_print_time >= SENSOR_PRINT_INTERVAL_SEC:
                print_sensor_values(
                    ultrasonic_readings=ultrasonic_readings,
                    tof_reading=tof_reading,
                    tof_sensor=tof_sensor,
                    objects=objects,
                    risk=risk,
                    state=state,
                    brake_percent=brake_percent,
                    ego_speed=ego_speed,
                    steering_angle=steering_angle,
                    driver_override=driver_override,
                    yolo_result_age=result_age,
                )
                last_sensor_print_time = now

            brake_controller.apply(state, brake_percent, risk)

            if logger is not None:
                logger.write(
                    state=state,
                    brake_percent=brake_percent,
                    risk=risk,
                    ultrasonic_readings=ultrasonic_readings,
                    tof_reading=tof_reading,
                    ego_speed=ego_speed,
                    steering_angle=steering_angle,
                )

            if preview_enabled and frame is not None and CV2_AVAILABLE:
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
        try:
            sys.stdout.write(terminal_show_cursor())
            sys.stdout.flush()
        except Exception:
            pass

        try:
            keyboard_ego.stop()
        except Exception:
            pass

        print("[INFO] Cleaning up...")


        try:
            async_yolo.stop()
        except Exception:
            pass

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

        if logger is not None:
            logger.close()
        tof_sensor.release()
        camera.release()

        if CV2_AVAILABLE:
            cv2.destroyAllWindows()

        if GPIO_AVAILABLE:
            GPIO.cleanup()

        print("[INFO] AEB stopped.")
        if logger is not None:
            print(f"[INFO] Log saved: {LOG_CSV_PATH}")


if __name__ == "__main__":
    main()
