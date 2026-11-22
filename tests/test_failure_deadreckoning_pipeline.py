import json
import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from dead_reckoning import DeadReckoningConfig, MotionHistory, predict_future_trajectory
from failure_detection import FailureConfig, FailureDetector, FailureState
from main import run_pipeline

EUROC_HEADER = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"


def write_euroc_csv(path, ts, gyro, accel):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(EUROC_HEADER)
        for i in range(len(ts)):
            row = [int(ts[i]), gyro[i, 0], gyro[i, 1], gyro[i, 2], accel[i, 0], accel[i, 1], accel[i, 2]]
            f.write(",".join(f"{x:.9f}" if isinstance(x, float) else str(x) for x in row) + "\n")


def make_stationary_then_failure_dataset(path, total_s=10.0, rate=200.0, failure_s=5.0):
    n = int(total_s * rate) + 1
    ts0 = 1_000_000_000
    dt_ns = int(round(1e9 / rate))
    ts = ts0 + np.arange(n, dtype=np.int64) * dt_ns
    gyro = np.zeros((n, 3), dtype=np.float64)
    accel = np.zeros((n, 3), dtype=np.float64)
    accel[:, 2] = 9.81
    fail_start = int(round((total_s - failure_s) * rate))
    gyro[fail_start:, :] = 0.0
    accel[fail_start:, :] = 0.0
    write_euroc_csv(os.path.join(path, "imu0", "data.csv"), ts, gyro, accel)
    return fail_start, ts


class TestFailureDeadReckoningAndPipeline(unittest.TestCase):
    def test_failure_detection_states(self):
        det = FailureDetector(FailureConfig(complete_failure_consecutive=3, stuck_window=3))
        g = np.array([0.0, 0.0, 0.0])
        a = np.array([0.0, 0.0, 9.81])
        # valid small/noisy motion not failure
        info = det.observe(0, 1000, g, a)
        self.assertNotEqual(info["state"], FailureState.COMPLETE_FAILURE.value)
        # sustained zeros -> complete failure
        states = []
        for i in range(1, 6):
            info = det.observe(i, 1000 + i * 5_000_000, np.zeros(3), np.zeros(3))
            states.append(info["state"])
        self.assertEqual(states[-1], FailureState.COMPLETE_FAILURE.value)

    def test_five_second_dead_reckoning_prediction(self):
        hist = MotionHistory(maxlen=50)
        t0 = 1_000_000_000
        q = np.array([1.0, 0.0, 0.0, 0.0])
        p = np.array([10.0, 0.0, 0.0])
        v = np.array([1.0, 0.0, 0.0])
        a = np.array([0.0, 0.0, 0.0])
        for i in range(20):
            hist.append(t0 + i * 5_000_000, p + v * (i * 0.005), v, a, q, np.zeros(3))
        preds = predict_future_trajectory(hist, DeadReckoningConfig(horizon_s=5.0, interval_s=0.1), dt_s=0.1)
        self.assertGreaterEqual(len(preds), 49)
        dt_total = (preds[-1]["t_ns"] - preds[0]["t_ns"]) * 1e-9
        self.assertGreaterEqual(dt_total, 4.7)
        self.assertTrue(np.all(np.isfinite(preds[-1]["p"])))
        # should move forward roughly along +x
        self.assertGreater(preds[-1]["p"][0], preds[0]["p"][0])

    def test_end_to_end_pipeline_output_and_failure_prediction(self):
        with tempfile.TemporaryDirectory() as td:
            ds = os.path.join(td, "data")
            fail_start, ts = make_stationary_then_failure_dataset(ds, total_s=12.0, rate=100.0, failure_s=5.0)
            out_traj = os.path.join(td, "out", "estimated_trajectory.txt")
            out_meta = os.path.join(td, "out", "metadata.json")
            meta = run_pipeline(ds, out_traj, metadata_path=out_meta, prediction_horizon_s=5.0)
            self.assertTrue(os.path.exists(out_traj))
            self.assertTrue(os.path.exists(out_meta))
            with open(out_meta, "r", encoding="utf-8") as f:
                m = json.load(f)
            self.assertEqual(m["num_input_samples"], meta["num_input_samples"])
            self.assertEqual(m["num_output_samples"], m["num_input_samples"], "Official contract: one pose per input IMU frame")
            self.assertEqual(m["prediction_appended_samples"], 0)
            self.assertTrue(m["failure_detected"])
            self.assertTrue(m["prediction_generated"])
            # output format: comment + one pose per output sample
            with open(out_traj, "r", encoding="utf-8") as f:
                lines = [ln.strip() for ln in f if ln.strip()]
            self.assertTrue(lines[0].startswith("#"))
            pose_lines = [ln for ln in lines[1:] if not ln.startswith("#")]
            self.assertEqual(len(pose_lines), m["num_output_samples"])
            for ln in pose_lines:
                parts = ln.split()
                self.assertEqual(len(parts), 8)
                vals = [float(x) for x in parts]
                self.assertTrue(all(np.isfinite(vals)))
                q = np.array(vals[4:8], dtype=float)
                self.assertTrue(abs(np.linalg.norm(q) - 1.0) < 1e-3, msg=f"quat norm {np.linalg.norm(q)}")


if __name__ == "__main__":
    unittest.main()
