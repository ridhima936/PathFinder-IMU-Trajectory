import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from evaluation import evaluate_trajectory
from preprocessing import PreprocessConfig, estimate_initial_bias_and_gravity, preprocess_imu

EUROC_HEADER = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"


def write_tum(path, t, p, q_xyzw):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.9f} {p[i,0]:.9f} {p[i,1]:.9f} {p[i,2]:.9f} {q_xyzw[i,0]:.9f} {q_xyzw[i,1]:.9f} {q_xyzw[i,2]:.9f} {q_xyzw[i,3]:.9f}\n")


class TestPreprocessingAndEvaluation(unittest.TestCase):
    def test_bias_and_gravity_estimation(self):
        n = 400
        ts = (np.arange(n) * 5_000_000).astype(np.int64)
        true_bias = np.array([0.01, -0.02, 0.005])
        gyro = np.tile(true_bias, (n, 1)) + 0.001 * np.random.default_rng(0).standard_normal((n, 3))
        accel = np.tile(np.array([0.0, 0.0, 9.81]), (n, 1)) + 0.01 * np.random.default_rng(1).standard_normal((n, 3))
        cfg = PreprocessConfig(initial_bias_window_s=1.0)
        gb, ab, gmag, info = estimate_initial_bias_and_gravity(ts, gyro, accel, cfg)
        self.assertTrue(np.all(np.abs(gb - true_bias) < 0.01), msg=f"gyro bias est {gb} vs {true_bias}")
        self.assertTrue(abs(gmag - 9.81) < 0.1)

    def test_preprocessing_handles_missing_frames_via_dt_and_spikes(self):
        ts = np.array([0, 5_000_000, 25_000_000, 30_000_000], dtype=np.int64)  # gap in middle
        gyro = np.array([[0, 0, 0], [0, 0, 0], [10.0, 0, 0], [0, 0, 0]], dtype=np.float64)  # spike
        accel = np.tile([0.0, 0.0, 9.81], (4, 1))
        res = preprocess_imu(ts, gyro, accel, PreprocessConfig(spike_z_thresh=3.0))
        self.assertEqual(len(res.dt_s), 4)
        self.assertTrue(np.all(np.isfinite(res.dt_s)))
        self.assertGreater(res.dt_s[2], res.dt_s[1])  # gap reflected

    def test_evaluation_ate_perfect_and_offset(self):
        with tempfile.TemporaryDirectory() as td:
            t = np.linspace(0, 1, 101)
            p = np.stack([t, np.zeros_like(t), np.zeros_like(t)], axis=1)
            q = np.tile([0, 0, 0, 1.0], (len(t), 1))
            gt = os.path.join(td, "gt.txt")
            est = os.path.join(td, "est.txt")
            write_tum(gt, t, p, q)
            write_tum(est, t, p + np.array([1.0, 2.0, 0.0]), q)
            metrics = evaluate_trajectory(gt, est, align=True)
            self.assertLess(metrics["overall"]["ate_rmse_m"], 1e-6)
            metrics_no_align = evaluate_trajectory(gt, est, align=False)
            self.assertGreater(metrics_no_align["overall"]["ate_rmse_m"], 1.0)


if __name__ == "__main__":
    unittest.main()
