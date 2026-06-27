"""Classical single-camera lane detector for a controlled RC-car track."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np

try:
    from .protocol import LaneBoundary
except ImportError:  # Allow direct execution from this directory.
    from protocol import LaneBoundary


@dataclass
class LaneDetectorConfig:
    lane_width_m: float = 0.40
    visible_length_m: float = 2.0
    history_size: int = 6
    min_fit_points: int = 35
    min_lane_width_ratio: float = 0.18
    max_lane_width_ratio: float = 0.75
    max_offset_m: float = 0.45
    roi: tuple[tuple[float, float], ...] = (
        (0.12, 1.00),
        (0.35, 0.56),
        (0.70, 0.56),
        (0.92, 1.00),
    )
    perspective_source: tuple[tuple[float, float], ...] = (
        (0.43, 0.58),
        (0.57, 0.58),
        (0.74, 0.96),
        (0.26, 0.96),
    )
    perspective_destination: tuple[tuple[float, float], ...] = (
        (0.35, 0.00),
        (0.65, 0.00),
        (0.65, 1.00),
        (0.35, 1.00),
    )


@dataclass
class LaneDetectionResult:
    camera_valid: bool
    lane_valid: bool
    left: LaneBoundary
    right: LaneBoundary
    center_offset_m: float
    heading_error_rad: float
    confidence: float
    preview: np.ndarray


def load_calibration(path: str | None) -> tuple[np.ndarray | None, np.ndarray | None]:
    if not path:
        return None, None
    calibration_path = Path(path)
    if not calibration_path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {path}")
    data = np.load(calibration_path)
    camera_matrix = data.get("camera_matrix")
    distortion = data.get("dist_coeffs")
    if camera_matrix is None or distortion is None:
        raise ValueError("Calibration .npz must contain camera_matrix and dist_coeffs")
    return camera_matrix, distortion


class LaneDetector:
    def __init__(self, config: LaneDetectorConfig, calibration_path: str | None = None):
        self.config = config
        self.camera_matrix, self.distortion = load_calibration(calibration_path)
        self.left_history: deque[np.ndarray] = deque(maxlen=config.history_size)
        self.right_history: deque[np.ndarray] = deque(maxlen=config.history_size)

    def process(self, frame: np.ndarray) -> LaneDetectionResult:
        if frame is None or frame.size == 0:
            raise ValueError("Empty camera frame")

        corrected = self._undistort(frame)
        binary, roi_polygon = self._binary_lane_image(corrected)
        warped, inverse = self._warp(binary)
        left_fit, right_fit, left_count, right_count = self._fit_sliding_windows(warped)

        current_pair_valid = left_fit is not None and right_fit is not None
        if current_pair_valid:
            self.left_history.append(left_fit)
            self.right_history.append(right_fit)

        if not self.left_history or not self.right_history:
            preview = self._draw_preview(corrected, warped, inverse, None, None, roi_polygon)
            return LaneDetectionResult(
                camera_valid=True,
                lane_valid=False,
                left=LaneBoundary(),
                right=LaneBoundary(),
                center_offset_m=0.0,
                heading_error_rad=0.0,
                confidence=0.0,
                preview=preview,
            )

        left_fit = np.mean(self.left_history, axis=0)
        right_fit = np.mean(self.right_history, axis=0)
        height, width = warped.shape[:2]
        left_bottom = float(np.polyval(left_fit, height - 1))
        right_bottom = float(np.polyval(right_fit, height - 1))
        lane_width_px = right_bottom - left_bottom
        lane_width_ratio = lane_width_px / max(width, 1)

        geometry_valid = (
            current_pair_valid
            and lane_width_px > 1.0
            and self.config.min_lane_width_ratio <= lane_width_ratio <= self.config.max_lane_width_ratio
        )
        if geometry_valid:
            xm_per_pix = self.config.lane_width_m / lane_width_px
            left_feature = self._boundary_feature(left_fit, xm_per_pix, width, height, left_count)
            right_feature = self._boundary_feature(right_fit, xm_per_pix, width, height, right_count)
            center_offset = 0.5 * (
                left_feature.lateral_offset_m + right_feature.lateral_offset_m
            )
            heading_error = 0.5 * (left_feature.heading_rad + right_feature.heading_rad)
            confidence = min(left_feature.strength, right_feature.strength)
            lane_valid = abs(center_offset) <= self.config.max_offset_m and confidence > 0.15
        else:
            left_feature = LaneBoundary()
            right_feature = LaneBoundary()
            center_offset = 0.0
            heading_error = 0.0
            confidence = 0.0
            lane_valid = False

        preview = self._draw_preview(
            corrected, warped, inverse, left_fit, right_fit, roi_polygon
        )
        status = (
            f"valid={int(lane_valid)} offset={center_offset:+.3f}m "
            f"heading={heading_error:+.3f}rad confidence={confidence:.2f}"
        )
        cv2.putText(preview, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        return LaneDetectionResult(
            camera_valid=True,
            lane_valid=lane_valid,
            left=left_feature,
            right=right_feature,
            center_offset_m=center_offset,
            heading_error_rad=heading_error,
            confidence=confidence,
            preview=preview,
        )

    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        if self.camera_matrix is None or self.distortion is None:
            return frame
        return cv2.undistort(frame, self.camera_matrix, self.distortion)

    def _binary_lane_image(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        height, width = frame.shape[:2]
        polygon = np.array(
            [[(int(width * x), int(height * y)) for x, y in self.config.roi]],
            dtype=np.int32,
        )
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(roi_mask, polygon, 255)
        roi = cv2.bitwise_and(frame, frame, mask=roi_mask)

        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        white = cv2.inRange(hls, (0, 170, 0), (180, 255, 255))
        yellow = cv2.inRange(hls, (15, 60, 60), (42, 255, 255))
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (7, 7), 0), 70, 170)
        binary = cv2.bitwise_or(cv2.bitwise_or(white, yellow), edges)
        binary = cv2.bitwise_and(binary, roi_mask)
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        return binary, polygon

    def _warp(self, binary: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        height, width = binary.shape[:2]
        source = np.float32([[width * x, height * y] for x, y in self.config.perspective_source])
        destination = np.float32(
            [[width * x, height * y] for x, y in self.config.perspective_destination]
        )
        matrix = cv2.getPerspectiveTransform(source, destination)
        inverse = cv2.getPerspectiveTransform(destination, source)
        return cv2.warpPerspective(binary, matrix, (width, height)), inverse

    def _fit_sliding_windows(
        self, binary: np.ndarray, windows: int = 9, margin: int = 50, min_pixels: int = 20
    ) -> tuple[np.ndarray | None, np.ndarray | None, int, int]:
        height, width = binary.shape[:2]
        histogram = np.sum(binary[height // 2 :, :], axis=0)
        midpoint = width // 2
        if histogram[:midpoint].max(initial=0) == 0 or histogram[midpoint:].max(initial=0) == 0:
            return None, None, 0, 0

        left_current = int(np.argmax(histogram[:midpoint]))
        right_current = int(np.argmax(histogram[midpoint:]) + midpoint)
        nonzero_y, nonzero_x = binary.nonzero()
        window_height = max(1, height // windows)
        left_indices: list[np.ndarray] = []
        right_indices: list[np.ndarray] = []

        for window in range(windows):
            y_low = height - (window + 1) * window_height
            y_high = height - window * window_height
            left = np.where(
                (nonzero_y >= y_low)
                & (nonzero_y < y_high)
                & (nonzero_x >= left_current - margin)
                & (nonzero_x < left_current + margin)
            )[0]
            right = np.where(
                (nonzero_y >= y_low)
                & (nonzero_y < y_high)
                & (nonzero_x >= right_current - margin)
                & (nonzero_x < right_current + margin)
            )[0]
            left_indices.append(left)
            right_indices.append(right)
            if left.size > min_pixels:
                left_current = int(np.mean(nonzero_x[left]))
            if right.size > min_pixels:
                right_current = int(np.mean(nonzero_x[right]))

        left_idx = np.concatenate(left_indices)
        right_idx = np.concatenate(right_indices)
        left_fit = (
            np.polyfit(nonzero_y[left_idx], nonzero_x[left_idx], 2)
            if left_idx.size >= self.config.min_fit_points
            else None
        )
        right_fit = (
            np.polyfit(nonzero_y[right_idx], nonzero_x[right_idx], 2)
            if right_idx.size >= self.config.min_fit_points
            else None
        )
        return left_fit, right_fit, int(left_idx.size), int(right_idx.size)

    def _boundary_feature(
        self, pixel_fit: np.ndarray, xm_per_pix: float, width: int, height: int, count: int
    ) -> LaneBoundary:
        image_y = np.linspace(0, height - 1, max(height, 20))
        forward_m = (height - 1 - image_y) * self.config.visible_length_m / max(height - 1, 1)
        # Vehicle coordinates: positive lateral position is to the left.
        lateral_m = (width / 2.0 - np.polyval(pixel_fit, image_y)) * xm_per_pix
        fit_m = np.polyfit(forward_m, lateral_m, 2)
        derivative = fit_m[1]
        curvature = (2.0 * fit_m[0]) / ((1.0 + derivative * derivative) ** 1.5)
        strength = float(np.clip(count / max(height * 1.5, 1), 0.0, 1.0))
        return LaneBoundary(
            curvature_1pm=float(curvature),
            curvature_derivative_1pm2=0.0,
            heading_rad=float(math.atan(derivative)),
            lateral_offset_m=float(fit_m[2]),
            strength=strength,
        )

    def _draw_preview(
        self,
        frame: np.ndarray,
        warped: np.ndarray,
        inverse: np.ndarray,
        left_fit: np.ndarray | None,
        right_fit: np.ndarray | None,
        roi_polygon: np.ndarray,
    ) -> np.ndarray:
        display = frame.copy()
        cv2.polylines(display, roi_polygon, True, (0, 255, 255), 2)
        if left_fit is not None and right_fit is not None:
            height, width = warped.shape[:2]
            y = np.linspace(0, height - 1, height)
            left_x = np.polyval(left_fit, y)
            right_x = np.polyval(right_fit, y)
            left_points = np.transpose(np.vstack((left_x, y)))
            right_points = np.flipud(np.transpose(np.vstack((right_x, y))))
            polygon = np.vstack((left_points, right_points)).astype(np.int32)
            overlay = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.fillPoly(overlay, [polygon], (0, 180, 0))
            overlay = cv2.warpPerspective(overlay, inverse, (width, height))
            display = cv2.addWeighted(display, 1.0, overlay, 0.45, 0.0)
        debug = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        return np.hstack((display, debug))
