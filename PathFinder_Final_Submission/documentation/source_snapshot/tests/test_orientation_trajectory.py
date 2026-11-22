import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from orientation import OrientationConfig, OrientationEstimator, align_gravity, q_from_rotvec, q_multiply, q_normalize, q_to_rotmat, rotate_vector
from trajectory import TrajectoryConfig, TrajectoryEstimator


class TestOrientationAndTrajectory(unittest.TestCase):
    def test_quaternion_normalization_and_multiplication(self):
        q = np.array([2.0, 0.0, 0.0, 0.0])
        qn = q_normalize(q)
        self.assertAlmostEqual(float(np.linalg.norm(qn)), 1.0, places=12)
        ident = np.array([1.0, 0.0, 0.0, 0.0])
        self.assertTrue(np.allclose(q_multiply(ident, qn), qn))
        self.assertTrue(np.allclose(q_multiply(qn, ident), qn))

    def test_gyro_integration(self):
        est = OrientationEstimator(config=OrientationConfig(use_acc_correction=False))
        dt = 0.01
        gyro = np.array([0.0, 0.0, 1.0])  # rad/s yaw
        for _ in range(100):  # 1 second
            est.integrate_gyro(gyro, dt)
        R = q_to_rotmat(est.q)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        self.assertAlmostEqual(yaw, 1.0, delta=0.05)

    def test_gravity_alignment_and_compensation_stationary(self):
        g = 9.81
        q0 = align_gravity(np.array([0.0, 0.0, g]))
        # body accel [0,0,g] should map to world +Z
        aw = rotate_vector(q0, np.array([0.0, 0.0, g]))
        self.assertAlmostEqual(aw[0], 0.0, delta=1e-6)
        self.assertAlmostEqual(aw[1], 0.0, delta=1e-6)
        self.assertAlmostEqual(aw[2], g, delta=1e-6)

        cfg = TrajectoryConfig(gravity_magnitude=g, enable_stationary_zupt=False, velocity_damping=0.0)
        traj = TrajectoryEstimator(cfg, orientation_config=OrientationConfig(use_acc_correction=False))
        t0 = 1_000_000_000
        dt = 0.005
        traj.initialize(t0, accel_body_for_alignment=np.array([0.0, 0.0, g]))
        for i in range(1, 400):
            traj.step(t0 + int(i * dt * 1e9), np.zeros(3), np.array([0.0, 0.0, g]), dt, sample_usable=True)
        self.assertTrue(np.linalg.norm(traj.state.p) < 0.2, msg=f"position drift too large: {traj.state.p}")
        self.assertTrue(np.linalg.norm(traj.state.v) < 0.5, msg=f"velocity drift too large: {traj.state.v}")

    def test_constant_acceleration_integration(self):
        cfg = TrajectoryConfig(gravity_magnitude=9.81, enable_stationary_zupt=False, velocity_damping=0.0)
        cfg.max_dt_s = 0.01
        traj = TrajectoryEstimator(cfg, orientation_config=OrientationConfig(use_acc_correction=False))
        t0 = 1_000_000_000
        dt = 0.005
        # identity orientation, apply body accel equivalent to +1 m/s^2 in world X plus gravity support
        traj.initialize(t0, accel_body_for_alignment=np.array([0.0, 0.0, 9.81]))
        traj.state.q = np.array([1.0, 0.0, 0.0, 0.0])
        traj.orientation.reset(traj.state.q)
        steps = 200  # 1 second
        for i in range(1, steps + 1):
            traj.step(t0 + int(i * dt * 1e9), np.zeros(3), np.array([1.0, 0.0, 9.81]), dt, sample_usable=True)
        # expected x ~ 0.5*a*t^2 = 0.5 m, v_x ~ 1 m/s (approx with trapezoidal integration)
        self.assertAlmostEqual(traj.state.p[0], 0.5, delta=0.1)
        self.assertAlmostEqual(traj.state.v[0], 1.0, delta=0.1)
        self.assertAlmostEqual(traj.state.p[1], 0.0, delta=0.05)
        self.assertAlmostEqual(traj.state.p[2], 0.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()
