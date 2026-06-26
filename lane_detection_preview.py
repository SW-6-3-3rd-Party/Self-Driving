import argparse
import glob
import os
import socket
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import time

import cv2
import numpy as np


LANE_HISTORY_SIZE = 6
SHARED_ROI_POINTS = (
    (0.18, 1.00),
    (0.40, 0.56),
    (0.60, 0.56),
    (0.82, 1.00),
)
SHARED_PERSPECTIVE_SOURCE = (
    (0.43, 0.58),
    (0.57, 0.58),
    (0.74, 0.96),
    (0.26, 0.96),
)
SHARED_PERSPECTIVE_DESTINATION = (
    (0.35, 0.00),
    (0.65, 0.00),
    (0.65, 1.00),
    (0.35, 1.00),
)
FUSED_CURRENT_LANE_SEARCH = (0.18, 0.50, 0.50, 0.82)
USB1_BOUNDARY_SEARCH = (0.30, 0.92)
USB2_BOUNDARY_SEARCH = (0.08, 0.70)
USB1_BOUNDARY_TARGET = 0.62
USB2_BOUNDARY_TARGET = 0.38
BOUNDARY_ERROR_SCALE_M = 3.7
MAX_FUSED_OFFSET_M = 0.65
MAX_OFFSET_JUMP_M = 0.25
MAX_BOUNDARY_DISAGREEMENT_M = 0.55
MIN_CURVATURE_M = 80.0
DUAL_OFFSET_SMOOTHING = 0.35
MIN_VALID_STREAK = 3


def existing_video_indices(max_index=20):
    indices = []
    for index in range(max_index + 1):
        if os.path.exists(f"/dev/video{index}"):
            indices.append(index)
    return indices


def camera_source_arg(value):
    if isinstance(value, int):
        return value
    if value.isdigit():
        return int(value)
    return value


def existing_video_paths():
    return sorted(glob.glob("/dev/video*"))


def existing_stable_video_links():
    links = []
    for pattern in ("/dev/v4l/by-id/*", "/dev/v4l/by-path/*"):
        for link in sorted(glob.glob(pattern)):
            try:
                target = os.path.realpath(link)
            except OSError:
                target = "unknown"
            links.append((link, target))
    return links


def auto_dual_camera_sources():
    sources = []
    seen_targets = set()
    for link in sorted(glob.glob("/dev/v4l/by-path/*video-index0")):
        if "usb-" not in os.path.basename(link):
            continue
        target = os.path.realpath(link)
        if target in seen_targets:
            continue
        seen_targets.add(target)
        sources.append(link)
    return sources[:2]


def probe_camera_indices(width, height, max_index=20):
    working = []
    for index in existing_video_indices(max_index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            ret, _ = cap.read()
            if ret:
                working.append(index)
        cap.release()
    return working


def probe_camera_sources(width, height):
    sources = []
    for path in existing_video_paths():
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            ret, frame = cap.read()
            if ret:
                height_actual, width_actual = frame.shape[:2]
                sources.append(f"{path} ({width_actual}x{height_actual})")
        cap.release()
    return sources


def format_camera_hint(width, height):
    existing = existing_video_paths()
    working = probe_camera_sources(width, height)
    links = existing_stable_video_links()

    if not existing:
        return "No /dev/video* devices were found."

    existing_text = ", ".join(existing)
    if not working:
        return f"Found {existing_text}, but none returned frames."

    working_text = ", ".join(working)
    hint = f"Found {existing_text}. Working camera sources: {working_text}."
    if links:
        links_text = ", ".join(f"{link}->{target}" for link, target in links)
        hint += f" Stable links: {links_text}."
    return hint


def get_preview_urls(host, port):
    if host not in ("", "0.0.0.0"):
        return [f"http://{host}:{port}"]

    urls = [f"http://localhost:{port}"]
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        address = probe.getsockname()[0]
        probe.close()
        if address and not address.startswith("127."):
            urls.append(f"http://{address}:{port}")
    except OSError:
        pass

    try:
        hostname = socket.gethostname()
        for _, _, _, _, sockaddr in socket.getaddrinfo(hostname, None):
            address = sockaddr[0]
            if ":" not in address and not address.startswith("127."):
                url = f"http://{address}:{port}"
                if url not in urls:
                    urls.append(url)
    except OSError:
        pass
    return urls


def create_udp_sender(host, port):
    if not host:
        return None, None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = (host, port)
    return sock, address


def send_lane_metrics_udp(sock, address, metrics):
    if sock is None or address is None:
        return

    packet = struct.pack(
        "<ffff",
        float(metrics["lane_detected"]),
        float(metrics["offset_m"]),
        float(metrics["curvature_m"]),
        float(metrics["camera_status"]),
    )
    sock.sendto(packet, address)


def default_lane_metrics(camera_status=1.0):
    return {
        "lane_detected": 0.0,
        "offset_m": 0.0,
        "curvature_m": 9999.0,
        "camera_status": float(camera_status),
    }


def load_calibration(calibration_path):
    if not calibration_path:
        return {}

    if not os.path.exists(calibration_path):
        raise FileNotFoundError(f"Calibration file not found: {calibration_path}")

    if calibration_path.endswith(".npz"):
        data = np.load(calibration_path)
        return {
            "camera_matrix": data.get("camera_matrix"),
            "dist_coeffs": data.get("dist_coeffs"),
            "perspective_matrix": data.get("perspective_matrix"),
        }

    storage = cv2.FileStorage(calibration_path, cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise RuntimeError(f"Could not open calibration file: {calibration_path}")

    calibration = {
        "camera_matrix": storage.getNode("camera_matrix").mat(),
        "dist_coeffs": storage.getNode("dist_coeffs").mat(),
        "perspective_matrix": storage.getNode("perspective_matrix").mat(),
    }
    storage.release()
    return calibration


def apply_camera_correction(frame, calibration):
    corrected = frame

    camera_matrix = calibration.get("camera_matrix")
    dist_coeffs = calibration.get("dist_coeffs")
    if camera_matrix is not None and dist_coeffs is not None:
        corrected = cv2.undistort(corrected, camera_matrix, dist_coeffs)

    perspective_matrix = calibration.get("perspective_matrix")
    if perspective_matrix is not None:
        height, width = corrected.shape[:2]
        corrected = cv2.warpPerspective(corrected, perspective_matrix, (width, height))

    return corrected


def build_roi_mask(frame):
    height, width = frame.shape[:2]

    polygon = np.array(
        [
            [
                (int(width * x_ratio), int(height * y_ratio))
                for x_ratio, y_ratio in SHARED_ROI_POINTS
            ]
        ],
        dtype=np.int32,
    )

    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, polygon, (255, 255, 255))
    return mask, polygon


class LaneTracker:
    def __init__(self, history_size=LANE_HISTORY_SIZE):
        self.history_size = history_size
        self.left_fits = []
        self.right_fits = []

    def has_lane(self):
        return bool(self.left_fits and self.right_fits)

    def current_fits(self):
        if not self.has_lane():
            return None, None
        return np.mean(self.left_fits, axis=0), np.mean(self.right_fits, axis=0)

    def update(self, left_fit, right_fit):
        if left_fit is not None:
            self.left_fits.append(left_fit)
            self.left_fits = self.left_fits[-self.history_size :]
        if right_fit is not None:
            self.right_fits.append(right_fit)
            self.right_fits = self.right_fits[-self.history_size :]


class BoundaryTracker:
    def __init__(self, history_size=LANE_HISTORY_SIZE):
        self.history_size = history_size
        self.fits = []

    def current_fit(self):
        if not self.fits:
            return None
        return np.mean(self.fits, axis=0)

    def update(self, fit):
        if fit is None:
            return
        self.fits.append(fit)
        self.fits = self.fits[-self.history_size :]


SINGLE_LANE_TRACKER = LaneTracker()
FUSED_LANE_TRACKER = LaneTracker()
USB1_BOUNDARY_TRACKER = BoundaryTracker()
USB2_BOUNDARY_TRACKER = BoundaryTracker()
DUAL_METRIC_STATE = {
    "offset_m": 0.0,
    "curvature_m": 9999.0,
    "valid_streak": 0,
}


def build_lane_binary(frame):
    roi_mask, roi_polygon = build_roi_mask(frame)
    roi_frame = cv2.bitwise_and(frame, roi_mask)

    hls = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HLS)
    white_mask = cv2.inRange(hls, (0, 175, 0), (180, 255, 255))
    yellow_mask = cv2.inRange(hls, (15, 70, 70), (40, 255, 255))
    color_mask = cv2.bitwise_or(white_mask, yellow_mask)

    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blurred, 80, 180)

    binary = cv2.bitwise_or(color_mask, edges)
    binary = cv2.bitwise_and(binary, cv2.cvtColor(roi_mask, cv2.COLOR_BGR2GRAY))
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return binary, roi_polygon


def perspective_points(width, height):
    source = np.float32(
        [[width * x_ratio, height * y_ratio] for x_ratio, y_ratio in SHARED_PERSPECTIVE_SOURCE]
    )
    destination = np.float32(
        [[width * x_ratio, height * y_ratio] for x_ratio, y_ratio in SHARED_PERSPECTIVE_DESTINATION]
    )
    return source, destination


def warp_lane_binary(binary):
    height, width = binary.shape[:2]
    source, destination = perspective_points(width, height)
    matrix = cv2.getPerspectiveTransform(source, destination)
    inverse = cv2.getPerspectiveTransform(destination, source)
    warped = cv2.warpPerspective(binary, matrix, (width, height))
    return warped, matrix, inverse, source


def lane_search_ranges(width, search_ranges=None):
    if search_ranges is None:
        return (0, width // 2), (width // 2, width)

    left_start, left_end, right_start, right_end = search_ranges
    left_range = (
        max(0, min(width - 1, int(width * left_start))),
        max(1, min(width, int(width * left_end))),
    )
    right_range = (
        max(0, min(width - 1, int(width * right_start))),
        max(1, min(width, int(width * right_end))),
    )
    return left_range, right_range


def fit_lane_from_windows(binary_warped, nwindows=9, margin=50, minpix=20, search_ranges=None):
    height, width = binary_warped.shape[:2]
    histogram = np.sum(binary_warped[height // 2 :, :], axis=0)
    left_range, right_range = lane_search_ranges(width, search_ranges)
    left_hist = histogram[left_range[0] : left_range[1]]
    right_hist = histogram[right_range[0] : right_range[1]]
    if left_hist.size == 0 or right_hist.size == 0:
        return None, None, (np.array([], dtype=int), np.array([], dtype=int)), (
            np.array([], dtype=int),
            np.array([], dtype=int),
        )
    if np.max(left_hist) <= 0 or np.max(right_hist) <= 0:
        return None, None, (np.array([], dtype=int), np.array([], dtype=int)), (
            np.array([], dtype=int),
            np.array([], dtype=int),
        )

    leftx_base = int(np.argmax(left_hist) + left_range[0])
    rightx_base = int(np.argmax(right_hist) + right_range[0])

    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    window_height = height // nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base
    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = height - (window + 1) * window_height
        win_y_high = height - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left_inds = (
            (nonzeroy >= win_y_low)
            & (nonzeroy < win_y_high)
            & (nonzerox >= win_xleft_low)
            & (nonzerox < win_xleft_high)
        ).nonzero()[0]
        good_right_inds = (
            (nonzeroy >= win_y_low)
            & (nonzeroy < win_y_high)
            & (nonzerox >= win_xright_low)
            & (nonzerox < win_xright_high)
        ).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else []
    right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else []
    return fit_lane_pixels(nonzerox, nonzeroy, left_lane_inds, right_lane_inds)


def fit_lane_near_previous(binary_warped, left_fit, right_fit, margin=45, search_ranges=None):
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    width = binary_warped.shape[1]
    left_range, right_range = lane_search_ranges(width, search_ranges)

    left_x = left_fit[0] * nonzeroy**2 + left_fit[1] * nonzeroy + left_fit[2]
    right_x = right_fit[0] * nonzeroy**2 + right_fit[1] * nonzeroy + right_fit[2]
    left_lane_inds = (
        (nonzerox > left_x - margin)
        & (nonzerox < left_x + margin)
        & (nonzerox >= left_range[0])
        & (nonzerox < left_range[1])
    ).nonzero()[0]
    right_lane_inds = (
        (nonzerox > right_x - margin) & (nonzerox < right_x + margin)
        & (nonzerox >= right_range[0])
        & (nonzerox < right_range[1])
    ).nonzero()[0]
    return fit_lane_pixels(nonzerox, nonzeroy, left_lane_inds, right_lane_inds)


def fit_lane_pixels(nonzerox, nonzeroy, left_lane_inds, right_lane_inds):
    min_points = 35
    left_fit = None
    right_fit = None
    left_pixels = (np.array([], dtype=int), np.array([], dtype=int))
    right_pixels = (np.array([], dtype=int), np.array([], dtype=int))

    if len(left_lane_inds) >= min_points:
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        left_fit = np.polyfit(lefty, leftx, 2)
        left_pixels = (leftx, lefty)

    if len(right_lane_inds) >= min_points:
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        right_fit = np.polyfit(righty, rightx, 2)
        right_pixels = (rightx, righty)

    return left_fit, right_fit, left_pixels, right_pixels


def fit_lane_from_search_bands(binary, search_ranges):
    height, width = binary.shape[:2]
    left_range, right_range = lane_search_ranges(width, search_ranges)
    nonzero = binary.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    min_y = int(height * 0.42)

    left_lane_inds = (
        (nonzeroy >= min_y)
        & (nonzerox >= left_range[0])
        & (nonzerox < left_range[1])
    ).nonzero()[0]
    right_lane_inds = (
        (nonzeroy >= min_y)
        & (nonzerox >= right_range[0])
        & (nonzerox < right_range[1])
    ).nonzero()[0]
    return fit_lane_pixels(nonzerox, nonzeroy, left_lane_inds, right_lane_inds)


def current_lane_search_ranges(args):
    return (
        getattr(args, "lane_left_start", FUSED_CURRENT_LANE_SEARCH[0]),
        getattr(args, "lane_left_end", FUSED_CURRENT_LANE_SEARCH[1]),
        getattr(args, "lane_right_start", FUSED_CURRENT_LANE_SEARCH[2]),
        getattr(args, "lane_right_end", FUSED_CURRENT_LANE_SEARCH[3]),
    )


def fit_lane_from_inner_edges(binary, search_ranges):
    height, width = binary.shape[:2]
    left_range, right_range = lane_search_ranges(width, search_ranges)
    nonzero = binary.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    min_y = int(height * 0.42)

    left_x = []
    left_y = []
    right_x = []
    right_y = []
    for y in range(min_y, height):
        row_left = nonzerox[
            (nonzeroy == y) & (nonzerox >= left_range[0]) & (nonzerox < left_range[1])
        ]
        row_right = nonzerox[
            (nonzeroy == y) & (nonzerox >= right_range[0]) & (nonzerox < right_range[1])
        ]
        if len(row_left) >= 2:
            left_x.append(np.percentile(row_left, 90))
            left_y.append(y)
        if len(row_right) >= 2:
            right_x.append(np.percentile(row_right, 10))
            right_y.append(y)

    min_points = 25
    left_fit = np.polyfit(left_y, left_x, 2) if len(left_y) >= min_points else None
    right_fit = np.polyfit(right_y, right_x, 2) if len(right_y) >= min_points else None
    return (
        left_fit,
        right_fit,
        (np.array(left_x, dtype=np.int32), np.array(left_y, dtype=np.int32)),
        (np.array(right_x, dtype=np.int32), np.array(right_y, dtype=np.int32)),
    )


def calculate_lane_metrics(left_fit, right_fit, width, height):
    ploty = np.linspace(0, height - 1, height)
    y_eval = height - 1
    left_bottom = np.polyval(left_fit, y_eval)
    right_bottom = np.polyval(right_fit, y_eval)
    lane_width_px = max(abs(right_bottom - left_bottom), 1.0)

    ym_per_pix = 30.0 / max(height, 1)
    xm_per_pix = 3.7 / lane_width_px
    left_fit_cr = np.polyfit(ploty * ym_per_pix, np.polyval(left_fit, ploty) * xm_per_pix, 2)
    right_fit_cr = np.polyfit(
        ploty * ym_per_pix, np.polyval(right_fit, ploty) * xm_per_pix, 2
    )

    y_eval_m = y_eval * ym_per_pix
    left_curverad = curvature_radius(left_fit_cr, y_eval_m)
    right_curverad = curvature_radius(right_fit_cr, y_eval_m)
    lane_center = (left_bottom + right_bottom) / 2.0
    center_offset_m = (width / 2.0 - lane_center) * xm_per_pix
    return (left_curverad + right_curverad) / 2.0, center_offset_m


def curvature_radius(fit_cr, y_eval_m):
    denominator = abs(2 * fit_cr[0])
    if denominator < 1e-6:
        return float("inf")
    return ((1 + (2 * fit_cr[0] * y_eval_m + fit_cr[1]) ** 2) ** 1.5) / denominator


def draw_tracked_lane(frame, binary_warped, inverse_matrix, left_fit, right_fit):
    height, width = binary_warped.shape[:2]
    ploty = np.linspace(0, height - 1, height)
    left_fitx = np.polyval(left_fit, ploty)
    right_fitx = np.polyval(right_fit, ploty)

    lane_overlay = np.zeros((height, width, 3), dtype=np.uint8)
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right)).astype(np.int32)
    cv2.fillPoly(lane_overlay, [pts], (0, 180, 0))
    cv2.polylines(lane_overlay, [pts_left.astype(np.int32)], False, (255, 0, 0), 8)
    cv2.polylines(lane_overlay, [pts_right.astype(np.int32)], False, (0, 0, 255), 8)

    unwarped = cv2.warpPerspective(lane_overlay, inverse_matrix, (width, height))
    return cv2.addWeighted(frame, 1.0, unwarped, 0.45, 0)


def draw_tracked_lane_direct(frame, left_fit, right_fit):
    height, width = frame.shape[:2]
    ploty = np.linspace(0, height - 1, height)
    left_fitx = np.polyval(left_fit, ploty)
    right_fitx = np.polyval(right_fit, ploty)

    lane_overlay = np.zeros_like(frame)
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right)).astype(np.int32)
    cv2.fillPoly(lane_overlay, [pts], (0, 180, 0))
    cv2.polylines(lane_overlay, [pts_left.astype(np.int32)], False, (255, 0, 0), 8)
    cv2.polylines(lane_overlay, [pts_right.astype(np.int32)], False, (0, 0, 255), 8)
    return cv2.addWeighted(frame, 1.0, lane_overlay, 0.45, 0)


def build_lane_debug(binary_warped, left_pixels, right_pixels, left_fit, right_fit):
    debug = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)
    leftx, lefty = left_pixels
    rightx, righty = right_pixels
    debug[lefty, leftx] = (255, 0, 0)
    debug[righty, rightx] = (0, 0, 255)

    height = binary_warped.shape[0]
    ploty = np.linspace(0, height - 1, height)
    for fit, color in ((left_fit, (255, 255, 0)), (right_fit, (0, 255, 255))):
        fitx = np.polyval(fit, ploty).astype(np.int32)
        valid = (fitx >= 0) & (fitx < binary_warped.shape[1])
        points = np.transpose(np.vstack([fitx[valid], ploty[valid]])).astype(np.int32)
        if len(points) > 1:
            cv2.polylines(debug, [points], False, color, 2)
    return debug


def draw_search_guides(frame, search_ranges):
    if search_ranges is None:
        return
    left_range, right_range = lane_search_ranges(frame.shape[1], search_ranges)
    for x_pos in (*left_range, *right_range):
        cv2.line(frame, (x_pos, 0), (x_pos, frame.shape[0]), (255, 160, 0), 1)


def lane_metrics_from_fits(left_fit, right_fit, width, height, camera_status=1.0):
    curvature_m, offset_m = calculate_lane_metrics(
        left_fit, right_fit, width, height
    )
    if np.isinf(curvature_m):
        curvature_m = 9999.0
    return {
        "lane_detected": 1.0,
        "offset_m": float(offset_m),
        "curvature_m": float(curvature_m),
        "camera_status": float(camera_status),
    }


def boundary_search_range(width, search_range):
    start, end = search_range
    return (
        max(0, min(width - 1, int(width * start))),
        max(1, min(width, int(width * end))),
    )


def fit_lane_boundary(binary, search_range, edge_side):
    height, width = binary.shape[:2]
    x_start, x_end = boundary_search_range(width, search_range)
    nonzero = binary.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    min_y = int(height * 0.42)

    boundary_x = []
    boundary_y = []
    percentile = 90 if edge_side == "right" else 10
    for y in range(min_y, height):
        row = nonzerox[
            (nonzeroy == y)
            & (nonzerox >= x_start)
            & (nonzerox < x_end)
        ]
        if len(row) >= 2:
            boundary_x.append(np.percentile(row, percentile))
            boundary_y.append(y)

    min_points = 25
    if len(boundary_y) < min_points:
        return None, (np.array([], dtype=np.int32), np.array([], dtype=np.int32)), 0.0

    fit = np.polyfit(boundary_y, boundary_x, 2)
    quality = min(1.0, len(boundary_y) / max(height - min_y, 1))
    return (
        fit,
        (np.array(boundary_x, dtype=np.int32), np.array(boundary_y, dtype=np.int32)),
        float(quality),
    )


def boundary_curvature_m(fit, width, height, error_scale_m):
    ploty = np.linspace(0, height - 1, height)
    ym_per_pix = 30.0 / max(height, 1)
    xm_per_pix = error_scale_m / max(width, 1)
    fit_cr = np.polyfit(ploty * ym_per_pix, np.polyval(fit, ploty) * xm_per_pix, 2)
    radius = curvature_radius(fit_cr, (height - 1) * ym_per_pix)
    return 9999.0 if np.isinf(radius) else float(radius)


def boundary_error_m(fit, width, height, target_ratio, side, error_scale_m, sign):
    bottom_x = float(np.polyval(fit, height - 1))
    x_ratio = bottom_x / max(width, 1)
    if side == "left":
        error_ratio = x_ratio - target_ratio
    else:
        error_ratio = target_ratio - x_ratio
    return float(sign * error_ratio * error_scale_m), x_ratio


def draw_boundary_detection(frame, binary, fit, pixels, search_range, target_ratio, label, color):
    display = frame.copy()
    height, width = display.shape[:2]
    x_start, x_end = boundary_search_range(width, search_range)
    cv2.rectangle(display, (x_start, int(height * 0.42)), (x_end, height - 1), (255, 160, 0), 1)
    target_x = int(width * target_ratio)
    cv2.line(display, (target_x, int(height * 0.42)), (target_x, height - 1), (0, 255, 255), 2)

    px, py = pixels
    valid_pixels = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    display[py[valid_pixels], px[valid_pixels]] = color

    if fit is not None:
        ploty = np.linspace(int(height * 0.42), height - 1, height - int(height * 0.42))
        fitx = np.polyval(fit, ploty).astype(np.int32)
        valid = (fitx >= 0) & (fitx < width)
        points = np.transpose(np.vstack([fitx[valid], ploty[valid]])).astype(np.int32)
        if len(points) > 1:
            cv2.polylines(display, [points], False, color, 4)
    else:
        cv2.putText(
            display,
            "Boundary not found",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
        )

    cv2.putText(
        display,
        label,
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )
    return display


def process_dual_boundaries(left_frame, right_frame, args):
    left_binary, _ = build_lane_binary(left_frame)
    right_binary, _ = build_lane_binary(right_frame)

    left_fit, left_pixels, left_quality = fit_lane_boundary(
        left_binary,
        (args.usb1_boundary_start, args.usb1_boundary_end),
        args.usb1_boundary_edge,
    )
    right_fit, right_pixels, right_quality = fit_lane_boundary(
        right_binary,
        (args.usb2_boundary_start, args.usb2_boundary_end),
        args.usb2_boundary_edge,
    )

    if left_fit is not None:
        USB1_BOUNDARY_TRACKER.update(left_fit)
        left_fit = USB1_BOUNDARY_TRACKER.current_fit()
    if right_fit is not None:
        USB2_BOUNDARY_TRACKER.update(right_fit)
        right_fit = USB2_BOUNDARY_TRACKER.current_fit()

    left_valid = left_fit is not None
    right_valid = right_fit is not None
    offset_values = []
    curvature_values = []
    left_error = 0.0
    right_error = 0.0
    left_ratio = 0.0
    right_ratio = 0.0

    if left_valid:
        left_error, left_ratio = boundary_error_m(
            left_fit,
            left_frame.shape[1],
            left_frame.shape[0],
            args.usb1_boundary_target,
            "left",
            args.boundary_error_scale_m,
            args.usb1_error_sign,
        )
        offset_values.append(left_error)
        curvature_values.append(
            boundary_curvature_m(left_fit, left_frame.shape[1], left_frame.shape[0], args.boundary_error_scale_m)
        )
    if right_valid:
        right_error, right_ratio = boundary_error_m(
            right_fit,
            right_frame.shape[1],
            right_frame.shape[0],
            args.usb2_boundary_target,
            "right",
            args.boundary_error_scale_m,
            args.usb2_error_sign,
        )
        offset_values.append(right_error)
        curvature_values.append(
            boundary_curvature_m(right_fit, right_frame.shape[1], right_frame.shape[0], args.boundary_error_scale_m)
        )

    both_valid = left_valid and right_valid
    lane_quality = min(left_quality, right_quality) if both_valid else 0.0
    raw_offset_m = float(np.mean(offset_values)) if offset_values else 0.0
    raw_curvature_m = float(np.mean(curvature_values)) if curvature_values else 9999.0
    boundary_disagreement_m = abs(left_error - right_error) if both_valid else 9999.0
    offset_jump_m = (
        abs(raw_offset_m - DUAL_METRIC_STATE["offset_m"])
        if DUAL_METRIC_STATE["valid_streak"] > 0
        else 0.0
    )
    candidate_valid = (
        both_valid
        and lane_quality >= args.min_lane_quality
        and abs(raw_offset_m) <= args.max_fused_offset_m
        and boundary_disagreement_m <= args.max_boundary_disagreement_m
        and raw_curvature_m >= args.min_curvature_m
        and offset_jump_m <= args.max_offset_jump_m
    )

    if candidate_valid:
        alpha = args.dual_offset_smoothing
        if DUAL_METRIC_STATE["valid_streak"] > 0:
            offset_m = (1.0 - alpha) * DUAL_METRIC_STATE["offset_m"] + alpha * raw_offset_m
            curvature_m = (1.0 - alpha) * DUAL_METRIC_STATE["curvature_m"] + alpha * raw_curvature_m
        else:
            offset_m = raw_offset_m
            curvature_m = raw_curvature_m
        DUAL_METRIC_STATE["offset_m"] = float(offset_m)
        DUAL_METRIC_STATE["curvature_m"] = float(curvature_m)
        DUAL_METRIC_STATE["valid_streak"] += 1
    else:
        offset_m = 0.0
        curvature_m = 9999.0
        DUAL_METRIC_STATE["valid_streak"] = 0

    lane_detected = 1.0 if DUAL_METRIC_STATE["valid_streak"] >= args.min_valid_streak else 0.0
    metrics = {
        "lane_detected": lane_detected,
        "offset_m": float(offset_m) if lane_detected > 0.5 else 0.0,
        "curvature_m": float(curvature_m) if lane_detected > 0.5 else 9999.0,
        "camera_status": 1.0,
    }

    left_display = draw_boundary_detection(
        left_frame,
        left_binary,
        left_fit,
        left_pixels,
        (args.usb1_boundary_start, args.usb1_boundary_end),
        args.usb1_boundary_target,
        "USB1 LEFT BOUNDARY",
        (255, 0, 0),
    )
    right_display = draw_boundary_detection(
        right_frame,
        right_binary,
        right_fit,
        right_pixels,
        (args.usb2_boundary_start, args.usb2_boundary_end),
        args.usb2_boundary_target,
        "USB2 RIGHT BOUNDARY",
        (0, 0, 255),
    )
    preview = np.hstack((left_display, right_display))
    cv2.putText(
        preview,
        f"Fused offset: {metrics['offset_m']:+.3f} m  L:{left_valid} {left_error:+.3f} R:{right_valid} {right_error:+.3f} Q:{lane_quality:.2f}",
        (20, preview.shape[0] - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        preview,
        f"Lx:{left_ratio:.2f} Rx:{right_ratio:.2f} dLR:{boundary_disagreement_m:.2f} jump:{offset_jump_m:.2f} streak:{DUAL_METRIC_STATE['valid_streak']}",
        (20, preview.shape[0] - 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    return preview, metrics


def annotate_lane_metrics(display_frame, metrics):
    curvature_m = metrics["curvature_m"]
    offset_m = metrics["offset_m"]
    curvature_text = "Radius: straight" if curvature_m >= 9999.0 else f"Radius: {curvature_m:6.1f} m"
    offset_direction = "left" if offset_m > 0 else "right"
    offset_text = f"Offset: {abs(offset_m):.2f} m {offset_direction}"
    cv2.putText(
        display_frame,
        curvature_text,
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        display_frame,
        offset_text,
        (20, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )


def process_frame(frame, tracker=None, camera_label=None, search_ranges=None, use_perspective=True):
    if tracker is None:
        tracker = SINGLE_LANE_TRACKER

    metrics = default_lane_metrics(camera_status=1.0)
    binary, roi_polygon = build_lane_binary(frame)
    if use_perspective:
        warped, _, inverse_matrix, source_points = warp_lane_binary(binary)
    else:
        warped = binary
        inverse_matrix = None
        source_points = None

    previous_left, previous_right = tracker.current_fits()
    if previous_left is not None and previous_right is not None:
        left_fit, right_fit, left_pixels, right_pixels = fit_lane_near_previous(
            warped, previous_left, previous_right, search_ranges=search_ranges
        )
        if left_fit is None or right_fit is None:
            if use_perspective:
                left_fit, right_fit, left_pixels, right_pixels = fit_lane_from_windows(
                    warped, search_ranges=search_ranges
                )
            else:
                left_fit, right_fit, left_pixels, right_pixels = fit_lane_from_inner_edges(
                    warped, search_ranges
                )
    else:
        if use_perspective:
            left_fit, right_fit, left_pixels, right_pixels = fit_lane_from_windows(
                warped, search_ranges=search_ranges
            )
        else:
            left_fit, right_fit, left_pixels, right_pixels = fit_lane_from_inner_edges(
                warped, search_ranges
            )

    display_frame = frame.copy()
    if camera_label:
        cv2.putText(
            display_frame,
            camera_label,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
    cv2.polylines(display_frame, roi_polygon, True, (0, 255, 255), 3)
    if source_points is not None:
        cv2.polylines(display_frame, [source_points.astype(np.int32)], True, (255, 255, 0), 2)
    draw_search_guides(display_frame, search_ranges)
    cv2.putText(
        display_frame,
        "CURRENT LANE ROI",
        tuple(roi_polygon[0][1]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )

    if left_fit is not None and right_fit is not None:
        tracker.update(left_fit, right_fit)
        left_fit, right_fit = tracker.current_fits()
        if use_perspective:
            display_frame = draw_tracked_lane(display_frame, warped, inverse_matrix, left_fit, right_fit)
        else:
            display_frame = draw_tracked_lane_direct(display_frame, left_fit, right_fit)
        metrics = lane_metrics_from_fits(left_fit, right_fit, frame.shape[1], frame.shape[0])
        annotate_lane_metrics(display_frame, metrics)
        debug = build_lane_debug(warped, left_pixels, right_pixels, left_fit, right_fit)
        draw_search_guides(debug, search_ranges)
    else:
        cv2.putText(
            display_frame,
            "Lane fit not found",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        debug = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        draw_search_guides(debug, search_ranges)

    combined = np.hstack((display_frame, debug))
    return combined, metrics


def draw_lane_lines(frame, lines):
    if lines is None:
        return

    width = frame.shape[1]
    center_x = width // 2

    for line in lines:
        x1, y1, x2, y2 = line[0]
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0:
            continue

        slope = dy / dx
        if abs(slope) < 0.35:
            continue

        mid_x = (x1 + x2) // 2
        if slope < 0 and mid_x < center_x:
            color = (255, 0, 0)
            label = "LEFT"
        elif slope > 0 and mid_x >= center_x:
            color = (0, 0, 255)
            label = "RIGHT"
        else:
            color = (0, 255, 0)
            label = None

        cv2.line(frame, (x1, y1), (x2, y2), color, 4)
        if label:
            cv2.putText(
                frame,
                label,
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )


def save_snapshot(preview, save_path, last_save_time, save_interval):
    if not save_path:
        return last_save_time

    now = time.monotonic()
    if now - last_save_time >= save_interval:
        cv2.imwrite(save_path, preview)
        print(f"saved {save_path}")
        return now

    return last_save_time


def create_capture(camera_source, width, height, label):
    camera_source = camera_source_arg(camera_source)
    cap = cv2.VideoCapture(camera_source, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        cap.release()
        raise RuntimeError(
            f"Could not open {label} camera source {camera_source}. "
            f"{format_camera_hint(width, height)}"
        )

    return cap


class CameraReader:
    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self.ok = False
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        while self.running:
            ok, frame = self.cap.read()
            with self.lock:
                self.ok = ok
                if ok:
                    self.frame = frame
                    self.ready.set()
            if not ok:
                time.sleep(0.02)

    def read(self, timeout=1.0):
        self.ready.wait(timeout)
        with self.lock:
            if not self.ok or self.frame is None:
                return False, None
            return True, self.frame.copy()

    def release(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


def capture_info(cap):
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_text = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4)).strip()
    return {
        "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "fps": cap.get(cv2.CAP_PROP_FPS),
        "fourcc": fourcc_text or str(fourcc),
    }


def print_capture_info(source):
    if source["mode"] == "single":
        info = capture_info(source["reader"].cap)
        print(
            "single camera actual mode: "
            f"{info['width']}x{info['height']} @ {info['fps']:.1f}fps {info['fourcc']}"
        )
        return

    left = capture_info(source["left_reader"].cap)
    right = capture_info(source["right_reader"].cap)
    print(
        "USB1 front-left actual mode: "
        f"{left['width']}x{left['height']} @ {left['fps']:.1f}fps {left['fourcc']}"
    )
    print(
        "USB2 front-right actual mode: "
        f"{right['width']}x{right['height']} @ {right['fps']:.1f}fps {right['fourcc']}"
    )


def build_camera_source(args):
    if args.left_camera is None and args.right_camera is None:
        cap = create_capture(args.camera, args.width, args.height, "single")
        return {
            "mode": "single",
            "reader": CameraReader(cap),
            "calibration": load_calibration(args.calibration),
        }

    if args.left_camera is None or args.right_camera is None:
        raise RuntimeError("Use both --left-camera and --right-camera for dual mode.")

    left_cap = create_capture(args.left_camera, args.width, args.height, "left")
    right_cap = create_capture(args.right_camera, args.width, args.height, "right")
    return {
        "mode": "dual",
        "left_reader": CameraReader(left_cap),
        "right_reader": CameraReader(right_cap),
        "left_calibration": load_calibration(args.left_calibration),
        "right_calibration": load_calibration(args.right_calibration),
    }


def read_camera_frame(source, args):
    if source["mode"] == "single":
        ret, frame = source["reader"].read()
        if not ret:
            return False, None
        frame = apply_camera_correction(frame, source["calibration"])
        return True, frame

    left_ret, left_frame = source["left_reader"].read()
    if not left_ret:
        return False, None

    left_frame = apply_camera_correction(left_frame, source["left_calibration"])

    left_frame = cv2.resize(left_frame, (args.width, args.height))
    cv2.putText(
        left_frame,
        "USB1 FRONT / LANE SOURCE",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )
    return True, left_frame


def build_processed_preview(source, args):
    if source["mode"] == "single":
        ret, frame = read_camera_frame(source, args)
        if not ret:
            return False, None, default_lane_metrics(camera_status=0.0)
        preview, metrics = process_frame(
            frame,
            SINGLE_LANE_TRACKER,
            "USB1 FRONT / LANE SOURCE",
        )
        return True, preview, metrics

    left_ret, left_frame = source["left_reader"].read()
    right_ret, right_frame = source["right_reader"].read()
    if not left_ret or not right_ret:
        return False, None, default_lane_metrics(camera_status=0.0)

    left_frame = apply_camera_correction(left_frame, source["left_calibration"])
    right_frame = apply_camera_correction(right_frame, source["right_calibration"])
    left_frame = cv2.resize(left_frame, (args.width, args.height))
    right_frame = cv2.resize(right_frame, (args.width, args.height))

    if args.lane_source == "dual":
        preview, metrics = process_dual_boundaries(left_frame, right_frame, args)
        return True, preview, metrics

    if args.lane_source == "usb1":
        preview, metrics = process_frame(
            left_frame,
            SINGLE_LANE_TRACKER,
            "USB1 FRONT LEFT / LANE SOURCE",
        )
        return True, preview, metrics

    if args.lane_source == "usb2":
        preview, metrics = process_frame(
            right_frame,
            SINGLE_LANE_TRACKER,
            "USB2 FRONT RIGHT / LANE SOURCE",
        )
        return True, preview, metrics

    fused_frame = np.hstack((left_frame, right_frame))
    preview, metrics = process_frame(
        fused_frame,
        FUSED_LANE_TRACKER,
        "USB1 LEFT + USB2 RIGHT / FUSED PREVIEW",
        search_ranges=current_lane_search_ranges(args),
        use_perspective=False,
    )
    return True, preview, metrics


def read_live_stream_frame(source, args, camera_name):
    if source["mode"] == "single":
        if camera_name not in ("usb1", "single"):
            return False, None
        ret, frame = source["reader"].read()
        if not ret:
            return False, None
        frame = apply_camera_correction(frame, source["calibration"])
        return True, cv2.resize(frame, (args.width, args.height))

    if camera_name == "usb1":
        reader = source["left_reader"]
        calibration = source["left_calibration"]
        label = "USB1 FRONT LEFT"
    elif camera_name in ("usb2", "usb"):
        reader = source["right_reader"]
        calibration = source["right_calibration"]
        label = "USB2 FRONT RIGHT"
    else:
        return False, None

    ret, frame = reader.read()
    if not ret:
        return False, None

    frame = apply_camera_correction(frame, calibration)
    frame = cv2.resize(frame, (args.width, args.height))
    cv2.putText(
        frame,
        label,
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )
    return True, frame


def release_camera_source(source):
    if source["mode"] == "single":
        source["reader"].release()
        return

    source["left_reader"].release()
    source["right_reader"].release()


def start_udp_metrics_publisher(source, args):
    if not args.udp_host:
        return None

    args.udp_running = True

    def loop():
        interval = 1.0 / max(args.udp_fps, 1.0)

        while args.udp_running:
            started_at = time.monotonic()

            try:
                ret, _, metrics = build_processed_preview(source, args)
                if ret:
                    send_lane_metrics_udp(args.udp_sock, args.udp_address, metrics)
                else:
                    send_lane_metrics_udp(
                        args.udp_sock,
                        args.udp_address,
                        default_lane_metrics(camera_status=0.0),
                    )
            except Exception as exc:
                print(f"UDP metrics publisher error: {exc}", flush=True)

            elapsed = time.monotonic() - started_at
            if elapsed < interval:
                time.sleep(interval - elapsed)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    return thread


def run_window_preview(source, args):
    print("Showing realtime lane detection preview. Press 'q' to quit.")
    if args.save_path:
        print(f"Saving snapshots to {args.save_path} every {args.save_interval} seconds.")

    last_save_time = 0.0

    while True:
        ret, preview, _ = build_processed_preview(source, args)
        if not ret:
            print("Failed to read frame from camera.")
            break

        cv2.imshow("Lane Detection Preview | left: tracked lane, right: bird-eye binary", preview)
        last_save_time = save_snapshot(
            preview, args.save_path, last_save_time, args.save_interval
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


def run_web_preview(source, args):
    processed_html = b"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Processed Camera Stream</title>
  <style>
    * { box-sizing: border-box; }
    html, body { margin: 0; min-height: 100%; background: #111; }
    body {
      display: flex;
      justify-content: center;
      align-items: flex-start;
      padding: 8px;
    }
    img {
      width: min(1400px, calc(100vw - 16px));
      height: auto;
      border: 2px solid #333;
      border-radius: 4px;
      background: #000;
    }
  </style>
</head>
<body>
  <img src="/stream.mjpg" alt="Lane detection preview">
</body>
</html>"""

    class PreviewHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html", "/processed"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(processed_html)
                return

            live_streams = {
                "/stream/usb1": "usb1",
                "/stream/usb2": "usb2",
                "/stream/usb": "usb",
            }
            if path in live_streams:
                self.stream_frames(live_streams[path], processed=False)
                return

            if path == "/stream.mjpg":
                self.stream_frames("processed", processed=True)
                return

            if path != "/stream.mjpg":
                self.send_error(404)
                return

        def stream_frames(self, camera_name, processed):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()

            last_save_time = 0.0
            frame_interval = 1.0 / max(args.stream_fps, 1.0)
            while True:
                started_at = time.monotonic()
                try:
                    if processed:
                        ret, output, _ = build_processed_preview(source, args)
                    else:
                        ret, output = read_live_stream_frame(source, args, camera_name)
                except Exception as exc:
                    print(f"Stream error ({camera_name}): {exc}", flush=True)
                    break
                if not ret:
                    print(f"Stream stopped: no frame for {camera_name}", flush=True)
                    break

                if processed:
                    last_save_time = save_snapshot(
                        output, args.save_path, last_save_time, args.save_interval
                    )
                ok, encoded = cv2.imencode(
                    ".jpg",
                    output,
                    [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                )
                if not ok:
                    print(f"Stream skipped: JPEG encode failed for {camera_name}", flush=True)
                    continue

                data = encoded.tobytes()
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break

                elapsed = time.monotonic() - started_at
                if elapsed < frame_interval:
                    time.sleep(frame_interval - elapsed)

        def log_message(self, format, *args):
            return

    server = ThreadingHTTPServer((args.host, args.port), PreviewHandler)
    print(f"Serving realtime lane detection preview on {args.host}:{args.port}")
    print("Open one of these URLs in a browser for the processed preview:")
    for url in get_preview_urls(args.host, args.port):
        print(f"  {url}")
    print("Raw stream paths remain available: /stream/usb1, /stream/usb2, /stream/usb")
    print("Press Ctrl+C here to quit.")
    if args.save_path:
        print(f"Saving snapshots to {args.save_path} every {args.save_interval} seconds.")
    server.serve_forever()


def main():
    parser = argparse.ArgumentParser(
        description="USB camera preview with lane tracking and curvature output."
    )
    parser.add_argument(
        "--camera",
        default="0",
        help="single USB camera index or device path, for example 0 or /dev/video0",
    )
    parser.add_argument(
        "--left-camera",
        default=None,
        help="left USB camera index or device path for dual-camera mode",
    )
    parser.add_argument(
        "--right-camera",
        default=None,
        help="right USB camera index or device path for dual-camera mode",
    )
    parser.add_argument("--width", type=int, default=320, help="camera frame width")
    parser.add_argument("--height", type=int, default=240, help="camera frame height")
    parser.add_argument(
        "--lane-left-start",
        type=float,
        default=FUSED_CURRENT_LANE_SEARCH[0],
        help="fused-frame ratio where current left lane search starts",
    )
    parser.add_argument(
        "--lane-left-end",
        type=float,
        default=FUSED_CURRENT_LANE_SEARCH[1],
        help="fused-frame ratio where current left lane search ends",
    )
    parser.add_argument(
        "--lane-right-start",
        type=float,
        default=FUSED_CURRENT_LANE_SEARCH[2],
        help="fused-frame ratio where current right lane search starts",
    )
    parser.add_argument(
        "--lane-right-end",
        type=float,
        default=FUSED_CURRENT_LANE_SEARCH[3],
        help="fused-frame ratio where current right lane search ends",
    )
    parser.add_argument(
        "--lane-source",
        choices=("usb1", "usb2", "fused", "dual"),
        default="dual",
        help="camera source for lane metrics sent to Simulink",
    )
    parser.add_argument(
        "--usb1-boundary-start",
        type=float,
        default=USB1_BOUNDARY_SEARCH[0],
        help="USB1 frame ratio where left-boundary search starts",
    )
    parser.add_argument(
        "--usb1-boundary-end",
        type=float,
        default=USB1_BOUNDARY_SEARCH[1],
        help="USB1 frame ratio where left-boundary search ends",
    )
    parser.add_argument(
        "--usb2-boundary-start",
        type=float,
        default=USB2_BOUNDARY_SEARCH[0],
        help="USB2 frame ratio where right-boundary search starts",
    )
    parser.add_argument(
        "--usb2-boundary-end",
        type=float,
        default=USB2_BOUNDARY_SEARCH[1],
        help="USB2 frame ratio where right-boundary search ends",
    )
    parser.add_argument(
        "--usb1-boundary-target",
        type=float,
        default=USB1_BOUNDARY_TARGET,
        help="expected USB1 left-boundary x ratio when centered in the lane",
    )
    parser.add_argument(
        "--usb2-boundary-target",
        type=float,
        default=USB2_BOUNDARY_TARGET,
        help="expected USB2 right-boundary x ratio when centered in the lane",
    )
    parser.add_argument(
        "--usb1-boundary-edge",
        choices=("left", "right"),
        default="right",
        help="edge of the detected stripe to track in USB1",
    )
    parser.add_argument(
        "--usb2-boundary-edge",
        choices=("left", "right"),
        default="left",
        help="edge of the detected stripe to track in USB2",
    )
    parser.add_argument(
        "--usb1-error-sign",
        type=float,
        default=1.0,
        help="set to -1 if USB1 error sign is reversed",
    )
    parser.add_argument(
        "--usb2-error-sign",
        type=float,
        default=1.0,
        help="set to -1 if USB2 error sign is reversed",
    )
    parser.add_argument(
        "--boundary-error-scale-m",
        type=float,
        default=BOUNDARY_ERROR_SCALE_M,
        help="meters represented by a full-frame boundary error",
    )
    parser.add_argument(
        "--min-lane-quality",
        type=float,
        default=0.20,
        help="minimum dual-boundary quality required to set lane_detected=1",
    )
    parser.add_argument(
        "--max-fused-offset-m",
        type=float,
        default=MAX_FUSED_OFFSET_M,
        help="maximum accepted fused lane offset before lane_detected is cleared",
    )
    parser.add_argument(
        "--max-offset-jump-m",
        type=float,
        default=MAX_OFFSET_JUMP_M,
        help="maximum accepted frame-to-frame fused offset jump",
    )
    parser.add_argument(
        "--max-boundary-disagreement-m",
        type=float,
        default=MAX_BOUNDARY_DISAGREEMENT_M,
        help="maximum accepted disagreement between USB1 and USB2 boundary errors",
    )
    parser.add_argument(
        "--min-curvature-m",
        type=float,
        default=MIN_CURVATURE_M,
        help="minimum accepted curvature radius for dual-boundary metrics",
    )
    parser.add_argument(
        "--dual-offset-smoothing",
        type=float,
        default=DUAL_OFFSET_SMOOTHING,
        help="low-pass filter alpha for accepted dual-boundary offset",
    )
    parser.add_argument(
        "--min-valid-streak",
        type=int,
        default=MIN_VALID_STREAK,
        help="consecutive valid dual-boundary frames required before lane_detected=1",
    )
    parser.add_argument(
        "--udp-host",
        default=None,
        help="Simulink/MATLAB PC IP address for UDP lane metrics",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=5005,
        help="UDP port for Simulink lane metrics",
    )
    parser.add_argument(
        "--udp-fps",
        type=float,
        default=10.0,
        help="UDP publishing FPS for Simulink lane metrics",
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="show detected /dev/video* devices and usable camera indices, then exit",
    )
    parser.add_argument(
        "--calibration",
        default=None,
        help="optional .npz/.yml calibration file for single-camera mode",
    )
    parser.add_argument(
        "--left-calibration",
        default=None,
        help="optional .npz/.yml calibration file for the left camera",
    )
    parser.add_argument(
        "--right-calibration",
        default=None,
        help="optional .npz/.yml calibration file for the right camera",
    )
    parser.add_argument(
        "--view",
        choices=("auto", "window", "web"),
        default="web",
        help="preview mode: browser stream, OpenCV window, or auto-detect",
    )
    parser.add_argument("--host", default="0.0.0.0", help="web preview host")
    parser.add_argument("--port", type=int, default=8000, help="web preview port")
    parser.add_argument(
        "--stream-fps",
        type=float,
        default=8.0,
        help="maximum MJPEG stream FPS per browser connection",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=55,
        help="MJPEG JPEG quality from 1 to 100",
    )
    parser.add_argument(
        "--save-path",
        default=None,
        help="optional output image path for saving preview snapshots",
    )
    parser.add_argument(
        "--save-interval",
        type=float,
        default=1.0,
        help="seconds between preview snapshot saves when --save-path is set",
    )
    args = parser.parse_args()
    args.jpeg_quality = max(1, min(100, args.jpeg_quality))
    args.stream_fps = max(1.0, args.stream_fps)
    args.udp_fps = max(1.0, args.udp_fps)
    args.lane_left_start = max(0.0, min(1.0, args.lane_left_start))
    args.lane_left_end = max(args.lane_left_start, min(1.0, args.lane_left_end))
    args.lane_right_start = max(0.0, min(1.0, args.lane_right_start))
    args.lane_right_end = max(args.lane_right_start, min(1.0, args.lane_right_end))
    args.usb1_boundary_start = max(0.0, min(1.0, args.usb1_boundary_start))
    args.usb1_boundary_end = max(args.usb1_boundary_start, min(1.0, args.usb1_boundary_end))
    args.usb2_boundary_start = max(0.0, min(1.0, args.usb2_boundary_start))
    args.usb2_boundary_end = max(args.usb2_boundary_start, min(1.0, args.usb2_boundary_end))
    args.usb1_boundary_target = max(0.0, min(1.0, args.usb1_boundary_target))
    args.usb2_boundary_target = max(0.0, min(1.0, args.usb2_boundary_target))
    args.boundary_error_scale_m = max(0.01, args.boundary_error_scale_m)
    args.min_lane_quality = max(0.0, min(1.0, args.min_lane_quality))
    args.max_fused_offset_m = max(0.01, args.max_fused_offset_m)
    args.max_offset_jump_m = max(0.01, args.max_offset_jump_m)
    args.max_boundary_disagreement_m = max(0.01, args.max_boundary_disagreement_m)
    args.min_curvature_m = max(0.0, args.min_curvature_m)
    args.dual_offset_smoothing = max(0.0, min(1.0, args.dual_offset_smoothing))
    args.min_valid_streak = max(1, args.min_valid_streak)

    if args.list_cameras:
        print(format_camera_hint(args.width, args.height))
        return

    if args.left_camera is None and args.right_camera is None:
        auto_sources = auto_dual_camera_sources()
        if len(auto_sources) >= 2:
            args.left_camera, args.right_camera = auto_sources[:2]
            print(f"Auto-selected USB1 front-left camera: {args.left_camera}")
            print(f"Auto-selected USB2 front-right camera: {args.right_camera}")

    source = build_camera_source(args)
    print_capture_info(source)
    args.udp_sock, args.udp_address = create_udp_sender(args.udp_host, args.udp_port)
    udp_thread = start_udp_metrics_publisher(source, args)
    if args.udp_host:
        print(f"Sending lane metrics UDP to {args.udp_host}:{args.udp_port}")

    try:
        has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        view_mode = "window" if args.view == "auto" and has_display else args.view
        if view_mode == "auto":
            view_mode = "web"

        if view_mode == "window":
            run_window_preview(source, args)
        else:
            run_web_preview(source, args)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if hasattr(args, "udp_running"):
            args.udp_running = False
        if udp_thread is not None:
            udp_thread.join(timeout=1.0)
        if args.udp_sock is not None:
            args.udp_sock.close()
        release_camera_source(source)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
