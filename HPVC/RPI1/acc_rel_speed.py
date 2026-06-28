"""Relative speed (v_rel) estimator for the ACC upper controller.

The ACC model (`acc_upper_controller`) expects ``v_rel`` as an input but nothing
currently produces it. This derives it from the forward-gap distance stream
(front ToF) by differentiation:

    v_rel = d(d_actual)/dt

Sign convention (matches the model's a_dist = Kp_dist*e_d + Kd_vel*v_rel with a
positive Kd_vel, and physical v_rel = v_lead - v_ego):

    positive  -> gap opening (lead pulling away)
    negative  -> gap closing (approaching)

Raw differentiation of ToF is noisy, so a first-order low-pass ("dirty
derivative") is applied. Invalid/missing distance and sensor dropouts are gated
so the estimate never emits a derivative spike.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class RelativeSpeedEstimator:
    filter_tau_s: float = 0.20   # low-pass time constant
    min_dt_s: float = 1e-3       # ignore samples closer in time than this
    max_dt_s: float = 0.50       # longer gaps treated as dropout -> reseed
    max_abs_v_mps: float = 5.0   # clamp to reject residual spikes

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._last_d: float | None = None
        self._last_t: float | None = None
        self._v_filt: float = 0.0

    def update(self, d_actual: float | None, t_now: float) -> tuple[float, bool]:
        """Feed one distance sample taken at monotonic time ``t_now`` (s).

        Returns ``(v_rel_mps, valid)``. When the distance is invalid the
        estimator resets and reports ``valid=False`` with v_rel 0.
        """
        if d_actual is None or not math.isfinite(d_actual) or d_actual <= 0.0:
            self.reset()
            return 0.0, False

        if self._last_d is None:
            # First valid sample: seed history, no velocity yet.
            self._last_d = d_actual
            self._last_t = t_now
            self._v_filt = 0.0
            return 0.0, True

        dt = t_now - self._last_t
        if dt < self.min_dt_s:
            # Duplicate / too-close timestamp: hold the previous estimate.
            return self._v_filt, True
        if dt > self.max_dt_s:
            # Sensor dropout: reseed so we don't differentiate across the gap.
            self._last_d = d_actual
            self._last_t = t_now
            self._v_filt = 0.0
            return 0.0, True

        v_raw = (d_actual - self._last_d) / dt
        alpha = self.filter_tau_s / (self.filter_tau_s + dt)
        self._v_filt = alpha * self._v_filt + (1.0 - alpha) * v_raw
        self._v_filt = max(-self.max_abs_v_mps, min(self.max_abs_v_mps, self._v_filt))
        self._last_d = d_actual
        self._last_t = t_now
        return self._v_filt, True
