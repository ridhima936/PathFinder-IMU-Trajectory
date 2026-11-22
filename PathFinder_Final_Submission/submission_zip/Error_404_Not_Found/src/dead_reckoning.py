"""Post-failure dead reckoning / trajectory prediction."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from orientation import q_from_rotvec, q_multiply, q_normalize


@dataclass
class DeadReckoningConfig:
    horizon_s: float = 5.0
    interval_s: Optional[float] = None  # if None, use median dt or fallback
    fallback_dt_s: float = 0.005
    history_size: int = 200
    # motion model
    use_constant_acceleration: bool = True
    accel_decay_tau_s: float = 1.5  # exponential decay of acceleration influence to avoid unstable extrapolation
    max_speed_m_s: float = 30.0
    max_accel_m_s2: float = 20.0
    max_gyro_rad_s: float = 10.0
    min_speed_for_direction_m_s: float = 0.05


class MotionHistory:
    """Stores recent motion states for prediction."""

    def __init__(self, maxlen: int = 200):
        self.maxlen = maxlen
        self.t_ns: Deque[int] = deque(maxlen=maxlen)
        self.p: Deque[np.ndarray] = deque(maxlen=maxlen)
        self.v: Deque[np.ndarray] = deque(maxlen=maxlen)
        self.a_world: Deque[np.ndarray] = deque(maxlen=maxlen)
        self.q: Deque[np.ndarray] = deque(maxlen=maxlen)
        self.gyro: Deque[np.ndarray] = deque(maxlen=maxlen)

    def append(self, t_ns: int, p: np.ndarray, v: np.ndarray, a_world: np.ndarray, q: np.ndarray, gyro: np.ndarray = None):
        self.t_ns.append(int(t_ns))
        self.p.append(np.asarray(p, dtype=np.float64).copy())
        self.v.append(np.asarray(v, dtype=np.float64).copy())
        self.a_world.append(np.asarray(a_world, dtype=np.float64).copy())
        self.q.append(np.asarray(q, dtype=np.float64).copy())
        if gyro is None:
            gyro = np.zeros(3, dtype=np.float64)
        self.gyro.append(np.asarray(gyro, dtype=np.float64).copy())

    def __len__(self) -> int:
        return len(self.t_ns)

    def last(self):
        if not self.t_ns:
            return None
        return (self.t_ns[-1], self.p[-1], self.v[-1], self.a_world[-1], self.q[-1], self.gyro[-1])


def _robust_mean(arrs: List[np.ndarray], default: np.ndarray) -> np.ndarray:
    if not arrs:
        return default.copy()
    m = np.stack(arrs, axis=0)
    return np.median(m, axis=0)


def predict_future_trajectory(
    history: MotionHistory,
    config: DeadReckoningConfig = None,
    dt_s: Optional[float] = None,
) -> List[Dict[str, object]]:
    """Predict future poses after complete IMU failure.

    Returns list of dicts with keys: t_ns, p (3,), q (4,wxyz).
    Does not use ground truth. Uses a damped constant-acceleration/velocity model for stability.
    """
    cfg = config or DeadReckoningConfig()
    if len(history) == 0:
        return []

    t0, p0, v0, a0, q0, g0 = history.last()
    interval = dt_s if dt_s and dt_s > 0 else (cfg.interval_s if cfg.interval_s and cfg.interval_s > 0 else cfg.fallback_dt_s)
    interval = float(np.clip(interval, 1e-4, 0.1))
    steps = max(1, int(round(cfg.horizon_s / interval)))

    # derive recent trends
    k = min(len(history), 20)
    recent_v = list(history.v)[-k:]
    recent_a = list(history.a_world)[-k:]
    recent_g = list(history.gyro)[-k:]
    v_est = recent_v[-1].copy() if recent_v else v0.copy()
    a_est = _robust_mean(recent_a, np.zeros(3)) if cfg.use_constant_acceleration else np.zeros(3)
    gyro_est = _robust_mean(recent_g, np.zeros(3))

    # clamp estimates to avoid unstable extrapolation
    a_est = np.clip(a_est, -cfg.max_accel_m_s2, cfg.max_accel_m_s2)
    gyro_est = np.clip(gyro_est, -cfg.max_gyro_rad_s, cfg.max_gyro_rad_s)
    speed = float(np.linalg.norm(v_est))
    if speed > cfg.max_speed_m_s and speed > 1e-9:
        v_est = v_est * (cfg.max_speed_m_s / speed)

    preds: List[Dict[str, object]] = []
    p = p0.astype(np.float64).copy()
    v = v_est.astype(np.float64).copy()
    q = q_normalize(q0.astype(np.float64).copy())

    # if motion history is insufficient/degenerate, fall back to constant velocity (or rest)
    insufficient = (len(history) < 3) or (not np.all(np.isfinite(v))) or (not np.all(np.isfinite(p)))
    if insufficient:
        a_est = np.zeros(3, dtype=np.float64)
        if not np.all(np.isfinite(v)):
            v = np.zeros(3, dtype=np.float64)

    for i in range(1, steps + 1):
        t_i = (i - 1) * interval  # relative time since start of prediction step 1
        tau = max(cfg.accel_decay_tau_s, 1e-3)
        decay = float(np.exp(-t_i / tau)) if cfg.use_constant_acceleration else 0.0
        a_i = a_est * decay
        # integrate
        v = v + a_i * interval
        speed = float(np.linalg.norm(v))
        if speed > cfg.max_speed_m_s and speed > 1e-9:
            v = v * (cfg.max_speed_m_s / speed)
        p = p + v * interval
        # orientation propagation (damped/constant gyro)
        q = q_normalize(q_multiply(q, q_from_rotvec(gyro_est * interval * decay if cfg.use_constant_acceleration else gyro_est * interval * 0.5)))

        t_ns = int(t0 + round(i * interval * 1e9))
        preds.append({"t_ns": t_ns, "p": p.copy(), "q": q.copy()})

    return preds


def predict_fixed_horizon(
    last_t_ns: int,
    last_p: np.ndarray,
    last_v: np.ndarray,
    last_q: np.ndarray,
    last_a_world: np.ndarray = None,
    last_gyro: np.ndarray = None,
    horizon_s: float = 5.0,
    interval_s: float = 0.005,
) -> List[Dict[str, object]]:
    """Convenience wrapper for simple callers."""
    hist = MotionHistory(maxlen=10)
    a = last_a_world if last_a_world is not None else np.zeros(3)
    g = last_gyro if last_gyro is not None else np.zeros(3)
    hist.append(last_t_ns, last_p, last_v, a, last_q, g)
    cfg = DeadReckoningConfig(horizon_s=horizon_s, interval_s=interval_s)
    return predict_future_trajectory(hist, cfg, dt_s=interval_s)
