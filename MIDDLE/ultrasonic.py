"""Alternating left/right ultrasonic acquisition for MIDDLE."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time


@dataclass(frozen=True)
class UltrasonicSnapshot:
    left_distance_m: float = math.nan
    right_distance_m: float = math.nan
    left_valid: bool = False
    right_valid: bool = False
    timestamp_us: int = 0


class DisabledUltrasonicPair:
    def read_pair(self) -> UltrasonicSnapshot:
        return UltrasonicSnapshot(timestamp_us=time.monotonic_ns() // 1000)

    def close(self) -> None:
        return


class MockUltrasonicPair:
    def __init__(self, left_distance_m: float, right_distance_m: float):
        self.left_distance_m = left_distance_m
        self.right_distance_m = right_distance_m

    def read_pair(self) -> UltrasonicSnapshot:
        return UltrasonicSnapshot(
            left_distance_m=self.left_distance_m,
            right_distance_m=self.right_distance_m,
            left_valid=True,
            right_valid=True,
            timestamp_us=time.monotonic_ns() // 1000,
        )

    def close(self) -> None:
        return


class GpioUltrasonicPair:
    """HC-SR04-style sensors using BCM GPIO numbering.

    Install rpi-lgpio or a compatible RPi.GPIO implementation. Echo inputs
    must be level shifted to 3.3 V before connection to Raspberry Pi GPIO.
    """

    SPEED_OF_SOUND_MPS = 343.0

    def __init__(
        self,
        left_trigger: int,
        left_echo: int,
        right_trigger: int,
        right_echo: int,
        timeout_s: float = 0.03,
        inter_sensor_delay_s: float = 0.06,
    ):
        try:
            import RPi.GPIO as GPIO
        except ImportError as exc:
            raise RuntimeError(
                "RPi.GPIO is unavailable. Install rpi-lgpio on Raspberry Pi OS."
            ) from exc

        self.gpio = GPIO
        self.left = (left_trigger, left_echo)
        self.right = (right_trigger, right_echo)
        self.timeout_s = timeout_s
        self.inter_sensor_delay_s = inter_sensor_delay_s
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        pins = [self.left[0], self.left[1], self.right[0], self.right[1]]
        try:
            for name, pin, mode, initial in (
                ("left trigger", left_trigger, GPIO.OUT, GPIO.LOW),
                ("left echo", left_echo, GPIO.IN, None),
                ("right trigger", right_trigger, GPIO.OUT, GPIO.LOW),
                ("right echo", right_echo, GPIO.IN, None),
            ):
                try:
                    if initial is None:
                        GPIO.setup(pin, mode)
                    else:
                        GPIO.setup(pin, mode, initial=initial)
                except Exception as exc:
                    raise RuntimeError(f"{name} BCM GPIO{pin} is busy or unavailable") from exc
        except Exception as exc:
            try:
                GPIO.cleanup(pins)
            except Exception:
                pass
            raise RuntimeError(
                f"Ultrasonic GPIO setup failed: {exc}. Configured BCM GPIO pins: {pins}. "
                "Stop the previous app.py process or reboot the Raspberry Pi, then run app.py again."
            ) from exc
        time.sleep(0.1)

    def read_pair(self) -> UltrasonicSnapshot:
        left_distance, left_valid = self._measure(*self.left)
        time.sleep(self.inter_sensor_delay_s)
        right_distance, right_valid = self._measure(*self.right)
        return UltrasonicSnapshot(
            left_distance_m=left_distance,
            right_distance_m=right_distance,
            left_valid=left_valid,
            right_valid=right_valid,
            timestamp_us=time.monotonic_ns() // 1000,
        )

    def _measure(self, trigger: int, echo: int) -> tuple[float, bool]:
        gpio = self.gpio
        gpio.output(trigger, gpio.LOW)
        time.sleep(0.000002)
        gpio.output(trigger, gpio.HIGH)
        time.sleep(0.000010)
        gpio.output(trigger, gpio.LOW)

        deadline = time.perf_counter() + self.timeout_s
        while gpio.input(echo) == gpio.LOW:
            if time.perf_counter() >= deadline:
                return math.nan, False
        pulse_start = time.perf_counter()

        deadline = pulse_start + self.timeout_s
        while gpio.input(echo) == gpio.HIGH:
            if time.perf_counter() >= deadline:
                return math.nan, False
        pulse_end = time.perf_counter()
        distance = (pulse_end - pulse_start) * self.SPEED_OF_SOUND_MPS / 2.0
        valid = 0.02 <= distance <= 4.0
        return (distance if valid else math.nan), valid

    def close(self) -> None:
        self.gpio.cleanup(
            [self.left[0], self.left[1], self.right[0], self.right[1]]
        )


class UltrasonicSampler:
    def __init__(self, pair, sample_hz: float = 8.0):
        self.pair = pair
        self.period_s = 1.0 / max(sample_hz, 0.1)
        self._snapshot = UltrasonicSnapshot()
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="ultrasonic", daemon=True)
        self._thread.start()

    def snapshot(self) -> UltrasonicSnapshot:
        with self._lock:
            return self._snapshot

    def close(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.pair.close()

    def _loop(self) -> None:
        while self._running.is_set():
            started = time.monotonic()
            snapshot = self.pair.read_pair()
            with self._lock:
                self._snapshot = snapshot
            remaining = self.period_s - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
