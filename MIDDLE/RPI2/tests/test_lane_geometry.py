import math
import unittest

import numpy as np

from RPI2.lane_detector import LaneDetector, LaneDetectorConfig


class LaneGeometryTest(unittest.TestCase):
    def test_straight_boundaries_use_rc_lane_width(self):
        detector = LaneDetector(
            LaneDetectorConfig(lane_width_m=0.40, visible_length_m=2.0)
        )
        width = 640
        height = 360
        left_x = 224.0
        right_x = 416.0
        xm_per_pix = 0.40 / (right_x - left_x)
        left = detector._boundary_feature(
            np.array([0.0, 0.0, left_x]), xm_per_pix, width, height, 500
        )
        right = detector._boundary_feature(
            np.array([0.0, 0.0, right_x]), xm_per_pix, width, height, 500
        )
        self.assertTrue(math.isclose(left.lateral_offset_m, 0.20, abs_tol=1e-6))
        self.assertTrue(math.isclose(right.lateral_offset_m, -0.20, abs_tol=1e-6))
        self.assertTrue(math.isclose(left.heading_rad, 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(right.heading_rad, 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(left.curvature_1pm, 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(right.curvature_1pm, 0.0, abs_tol=1e-6))


if __name__ == "__main__":
    unittest.main()

