"""Quaternion utilities and orientation estimation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np


# Quaternion convention: (w, x, y, z), representing rotation from body/sensor frame to world frame.


def q_identity() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def q_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n == 0.0:
        return q_identity()
    return q / n


def q_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


def q_conjugate(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=np.float64)


def q_from_rotvec(rv: np.ndarray) -> np.ndarray:
    rv = np.asarray(rv, dtype=np.float64)
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        # small-angle approximation
        return q_normalize(np.array([1.0, 0.5 * rv[0], 0.5 * rv[1], 0.5 * rv[2]], dtype=np.float64))
    axis = rv / theta
    half = 0.5 * theta
    s = math.sin(half)
    return np.array([math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s], dtype=np.float64)


def q_to_rotmat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v from body to world frame using quaternion q (w,x,y,z)."""
    return q_to_rotmat(q) @ np.asarray(v, dtype=np.float64)


def q_from_two_vectors(v_from: np.ndarray, v_to: np.ndarray) -> np.ndarray:
    """Quaternion rotating v_from to v_to."""
    a = np.asarray(v_from, dtype=np.float64)
    b = np.asarray(v_to, dtype=np.float64)
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-12 or nb < 1e-12:
        return q_identity()
    a = a / na
    b = b / nb
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if dot < -0.999999:
        # 180 degree rotation: choose arbitrary orthogonal axis
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis = axis / np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]], dtype=np.float64)
    c = np.cross(a, b)
    q = np.array([1.0 + dot, c[0], c[1], c[2]], dtype=np.float64)
    return q_normalize(q)


def align_gravity(accel_body_mean: np.ndarray, g_world: np.ndarray = None) -> np.ndarray:
    """Compute initial orientation such that measured specific force aligns with world up/gravity frame.

    Assumes accel_body_mean approximates specific force at rest: f ≈ -R^T g_w in world-up convention,
    equivalently rotated body acceleration points opposite gravity. For a stationary sensor, accelerometer
    measures upward reaction; rotating body 'up' vector to world 'up' gives initial attitude (yaw unconstrained).
    """
    if g_world is None:
        g_world = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    f = np.asarray(accel_body_mean, dtype=np.float64)
    nf = np.linalg.norm(f)
    if nf < 1e-9:
        return q_identity()
    return q_from_two_vectors(f / nf, g_world)


def quat_to_tum(q_wxyz: np.ndarray) -> Tuple[float, float, float, float]:
    w, x, y, z = q_normalize(q_wxyz)
    return (float(x), float(y), float(z), float(w))


def quat_from_tum(q_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = [float(v) for v in q_xyzw]
    return q_normalize(np.array([w, x, y, z], dtype=np.float64))


@dataclass
class OrientationConfig:
    acc_alpha: float = 0.02  # complementary filter gain for accelerometer tilt correction
    acc_g_tolerance: float = 0.35  # relative tolerance around gravity magnitude to trust accel for tilt
    acc_min_norm: float = 1e-6
    use_acc_correction: bool = True


class OrientationEstimator:
    """Lightweight quaternion orientation estimator with optional accelerometer tilt correction."""

    def __init__(self, q_init: np.ndarray = None, config: OrientationConfig = None):
        self.q = q_normalize(q_init) if q_init is not None else q_identity()
        self.config = config or OrientationConfig()

    def reset(self, q_init: np.ndarray = None):
        self.q = q_normalize(q_init) if q_init is not None else q_identity()

    def integrate_gyro(self, gyro_rad_s: np.ndarray, dt: float, gyro_prev: np.ndarray = None) -> np.ndarray:
        g = np.asarray(gyro_rad_s, dtype=np.float64)
        if gyro_prev is not None:
            g = 0.5 * (np.asarray(gyro_prev, dtype=np.float64) + g)
        dq = q_from_rotvec(g * float(dt))
        self.q = q_normalize(q_multiply(self.q, dq))
        return self.q.copy()

    def apply_acc_correction(self, accel_body: np.ndarray, g_magnitude: float) -> np.ndarray:
        if not self.config.use_acc_correction:
            return self.q.copy()
        a = np.asarray(accel_body, dtype=np.float64)
        norm_a = float(np.linalg.norm(a))
        if norm_a < self.config.acc_min_norm or g_magnitude <= 0:
            return self.q.copy()
        # trust accelerometer mostly when magnitude is close to gravity (low linear acceleration)
        rel = abs(norm_a - g_magnitude) / g_magnitude
        if rel > self.config.acc_g_tolerance:
            return self.q.copy()

        # Expected world gravity direction (upward specific force reference)
        # Stationary: accel_body ≈ R^T * [0,0,g], so rotate accel to world and nudge toward [0,0,g]
        g_world = np.array([0.0, 0.0, g_magnitude], dtype=np.float64)
        a_world_est = rotate_vector(self.q, a)
        na = np.linalg.norm(a_world_est)
        if na < 1e-9:
            return self.q.copy()
        a_world_est_n = a_world_est / na
        g_world_n = g_world / np.linalg.norm(g_world)

        # correction quaternion from estimated gravity direction to expected
        dq = q_from_two_vectors(a_world_est_n, g_world_n)
        # scale correction by alpha (slerp-like small-angle blend via nlerp approximation)
        alpha = float(np.clip(self.config.acc_alpha, 0.0, 1.0))
        # blend identity with dq: q_corr = (1-alpha)*I + alpha*dq (in axis-angle small form) -> use nlerp
        # convert dq to small step: interpolate
        # if dot < 0 flip for shortest path
        if np.dot([1, 0, 0, 0], dq) < 0:
            dq = -dq
        q_corr = q_normalize(np.array([1.0 - alpha, 0.0, 0.0, 0.0]) + alpha * dq)
        self.q = q_normalize(q_multiply(q_corr, self.q))
        return self.q.copy()

    def update(self, gyro_rad_s: np.ndarray, accel_body: np.ndarray, dt: float,
               g_magnitude: float, gyro_prev: np.ndarray = None) -> np.ndarray:
        self.integrate_gyro(gyro_rad_s, dt, gyro_prev=gyro_prev)
        self.apply_acc_correction(accel_body, g_magnitude)
        return self.q.copy()
