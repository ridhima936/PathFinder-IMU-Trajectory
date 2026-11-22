"""Trajectory (position/velocity/orientation) estimation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from orientation import OrientationConfig, OrientationEstimator, align_gravity, rotate_vector


@dataclass
class TrajectoryConfig:
    gravity_magnitude: float = 9.80665
    # motion constraints / drift reduction (conservative)
    enable_stationary_zupt: bool = True
    zupt_gyro_norm_max: float = 0.12  # rad/s
    zupt_acc_rel_tol: float = 0.12
    zupt_min_duration_s: float = 0.4
    zupt_velocity_damping: float = 0.15  # blend toward zero when stationary detected
    # general damping to reduce runaway integration (not forced zeroing)
    velocity_damping: float = 0.0005
    max_dt_s: float = 0.2
    min_dt_s: float = 1e-4


@dataclass
class TrajectoryState:
    t_ns: Optional[int] = None
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64))
    p: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    v: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    a_world_prev: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    gyro_prev: Optional[np.ndarray] = None
    stationary_duration_s: float = 0.0


class TrajectoryEstimator:
    """IMU strapdown trajectory estimator.

    Coordinate-frame assumptions:
    - Input gyro/accel are in the sensor/body frame (EuRoC/TUM-VI convention as loaded).
    - Quaternion q represents rotation from body frame to world frame.
    - World frame is gravity-aligned with +Z upward and gravity vector [0,0,-g].
    - Accelerometer measures specific force; world linear acceleration is R*q*a_body + [0,0,-g].
    - Initial yaw is not observable from IMU alone and is set by gravity alignment (effectively arbitrary yaw).

    Drift reduction is conservative (optional ZUPT-like damping when confidently stationary). Velocity/position
    are never hard-reset to zero arbitrarily.
    """

    def __init__(self, config: TrajectoryConfig = None, orientation_config: OrientationConfig = None):
        self.config = config or TrajectoryConfig()
        self.orientation = OrientationEstimator(config=orientation_config)
        self.state = TrajectoryState()
        self.initialized = False
        self.gravity_world = np.array([0.0, 0.0, -abs(self.config.gravity_magnitude)], dtype=np.float64)

    def initialize(self, t_ns: int, accel_body_for_alignment: np.ndarray = None, q_init: np.ndarray = None):
        if q_init is not None:
            self.state.q = q_init.astype(np.float64).copy()
        elif accel_body_for_alignment is not None:
            # align measured specific force direction to world +Z
            self.state.q = align_gravity(np.asarray(accel_body_for_alignment, dtype=np.float64), g_world=np.array([0.0, 0.0, 1.0]))
        else:
            self.state.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.orientation.reset(self.state.q)
        self.state.t_ns = int(t_ns)
        self.state.p[:] = 0.0
        self.state.v[:] = 0.0
        self.state.a_world_prev[:] = 0.0
        self.state.gyro_prev = None
        self.state.stationary_duration_s = 0.0
        self.initialized = True

    def _is_stationary(self, gyro: np.ndarray, accel: np.ndarray) -> bool:
        gnorm = float(np.linalg.norm(gyro))
        anorm = float(np.linalg.norm(accel))
        gref = abs(self.config.gravity_magnitude)
        rel = abs(anorm - gref) / gref if gref > 0 else 1.0
        return gnorm <= self.config.zupt_gyro_norm_max and rel <= self.config.zupt_acc_rel_tol

    def step(self, t_ns: int, gyro: np.ndarray, accel: np.ndarray, dt: float,
             sample_usable: bool = True) -> TrajectoryState:
        if not self.initialized:
            self.initialize(t_ns, accel_body_for_alignment=accel)
            return self.state

        dt = float(np.clip(dt, self.config.min_dt_s, self.config.max_dt_s))
        g = np.asarray(gyro, dtype=np.float64).reshape(3)
        a = np.asarray(accel, dtype=np.float64).reshape(3)

        if not sample_usable or not np.all(np.isfinite(g)) or not np.all(np.isfinite(a)):
            # propagate with constant velocity, keep orientation
            self.state.p = self.state.p + self.state.v * dt
            self.state.a_world_prev = np.zeros(3, dtype=np.float64)
            self.state.t_ns = int(t_ns)
            self.state.stationary_duration_s = 0.0
            return self.state

        # orientation update
        self.state.q = self.orientation.update(g, a, dt, g_magnitude=abs(self.config.gravity_magnitude), gyro_prev=self.state.gyro_prev)

        # gravity compensation: rotate specific force to world and subtract gravity
        f_world = rotate_vector(self.state.q, a)
        a_world = f_world + self.gravity_world  # gravity_world is negative Z

        # integrate velocity and position (trapezoidal)
        v_new = self.state.v + 0.5 * (self.state.a_world_prev + a_world) * dt
        p_new = self.state.p + 0.5 * (self.state.v + v_new) * dt

        # motion constraint / drift mitigation (conservative)
        if self.config.enable_stationary_zupt and self._is_stationary(g, a):
            self.state.stationary_duration_s += dt
            if self.state.stationary_duration_s >= self.config.zupt_min_duration_s:
                # gentle damping toward zero velocity (justified by sustained stationarity evidence)
                damp = float(np.clip(self.config.zupt_velocity_damping, 0.0, 1.0))
                v_new = (1.0 - damp) * v_new
        else:
            self.state.stationary_duration_s = 0.0

        if self.config.velocity_damping > 0:
            v_new = v_new * (1.0 - float(self.config.velocity_damping) * dt)

        self.state.v = v_new
        self.state.p = p_new
        self.state.a_world_prev = a_world
        self.state.gyro_prev = g.copy()
        self.state.t_ns = int(t_ns)
        return self.state

    def get_pose(self) -> Dict[str, object]:
        return {
            "t_ns": self.state.t_ns,
            "q_wxyz": self.state.q.copy(),
            "p": self.state.p.copy(),
            "v": self.state.v.copy(),
        }
