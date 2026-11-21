"""Sensor correction for gyro/accelerometer measurements."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


@dataclass
class SensorCorrectionConfig:
    # initial biases (rad/s, m/s^2) estimated in preprocessing
    use_initial_gyro_bias: bool = True
    use_initial_accel_bias: bool = False  # usually not observable; off by default
    # online gyro bias adaptation (very conservative, only when near-stationary)
    online_gyro_bias_adaptation: bool = True
    gyro_bias_lr: float = 0.002
    stationary_gyro_norm_max: float = 0.08
    stationary_acc_rel_tol: float = 0.08
    # simple low-pass filtering (IIR). alpha=1 means no filtering.
    gyro_lpf_alpha: float = 1.0
    accel_lpf_alpha: float = 1.0
    # scale correction factor (multiplicative around 1.0). Estimation not generally observable; keep identity by default.
    gyro_scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    accel_scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    estimate_scale: bool = False


class SensorCorrector:
    """Apply bias, optional filtering, and simple robustness corrections.

    Limitations: unknown time-varying biases and scale factors cannot always be recovered from IMU-only data.
    This module applies justified, conservative corrections and exposes configuration for tuning.
    """

    def __init__(self, config: SensorCorrectionConfig = None,
                 gyro_bias_init: np.ndarray = None, accel_bias_init: np.ndarray = None,
                 gravity_magnitude: float = 9.80665):
        self.config = config or SensorCorrectionConfig()
        self.gyro_bias = np.asarray(gyro_bias_init if gyro_bias_init is not None else np.zeros(3), dtype=np.float64).copy()
        self.accel_bias = np.asarray(accel_bias_init if accel_bias_init is not None else np.zeros(3), dtype=np.float64).copy()
        self.gravity_magnitude = float(gravity_magnitude)
        self._gyro_lpf_state: Optional[np.ndarray] = None
        self._accel_lpf_state: Optional[np.ndarray] = None
        self.stats: Dict[str, float] = {"gyro_bias_updates": 0}

    def _lpf(self, prev: Optional[np.ndarray], x: np.ndarray, alpha: float) -> np.ndarray:
        a = float(np.clip(alpha, 0.0, 1.0))
        if prev is None or a >= 1.0:
            return x.copy()
        return (a * x + (1.0 - a) * prev).astype(np.float64)

    def correct(self, gyro_raw: np.ndarray, accel_raw: np.ndarray, dt: float = 0.0,
                sample_valid: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        g = np.asarray(gyro_raw, dtype=np.float64).reshape(3).copy()
        a = np.asarray(accel_raw, dtype=np.float64).reshape(3).copy()

        if not sample_valid or not np.all(np.isfinite(g)) or not np.all(np.isfinite(a)):
            # return zeros to keep pipeline stable; caller/failure detector should handle
            g = np.nan_to_num(g, nan=0.0, posinf=0.0, neginf=0.0)
            a = np.nan_to_num(a, nan=0.0, posinf=0.0, neginf=0.0)

        # scale (if provided/estimated externally)
        g = g * np.asarray(self.config.gyro_scale, dtype=np.float64)
        a = a * np.asarray(self.config.accel_scale, dtype=np.float64)

        # bias correction
        if self.config.use_initial_gyro_bias or self.config.online_gyro_bias_adaptation:
            g_corr = g - self.gyro_bias
        else:
            g_corr = g
        if self.config.use_initial_accel_bias:
            a_corr = a - self.accel_bias
        else:
            a_corr = a

        # conservative online gyro bias adaptation when sensor appears stationary
        if self.config.online_gyro_bias_adaptation and sample_valid:
            gyro_norm = float(np.linalg.norm(g_corr))
            acc_norm = float(np.linalg.norm(a_corr))
            gref = self.gravity_magnitude if self.gravity_magnitude > 0 else 9.80665
            acc_rel = abs(acc_norm - gref) / gref
            if gyro_norm <= self.config.stationary_gyro_norm_max and acc_rel <= self.config.stationary_acc_rel_tol:
                # residual gyro likely dominated by bias
                self.gyro_bias = self.gyro_bias + self.config.gyro_bias_lr * (g_corr)
                self.stats["gyro_bias_updates"] += 1
                g_corr = g - self.gyro_bias

        # filtering
        g_f = self._lpf(self._gyro_lpf_state, g_corr, self.config.gyro_lpf_alpha)
        a_f = self._lpf(self._accel_lpf_state, a_corr, self.config.accel_lpf_alpha)
        self._gyro_lpf_state = g_f
        self._accel_lpf_state = a_f

        return g_f.astype(np.float64), a_f.astype(np.float64)

    def get_state(self) -> Dict[str, object]:
        return {
            "gyro_bias": self.gyro_bias.tolist(),
            "accel_bias": self.accel_bias.tolist(),
            "gyro_scale": list(self.config.gyro_scale),
            "accel_scale": list(self.config.accel_scale),
            "gyro_bias_updates": int(self.stats.get("gyro_bias_updates", 0)),
        }
