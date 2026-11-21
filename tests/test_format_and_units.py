import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from imu_reader import read_imu
from main import run_pipeline

EUROC_HEADER = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"


class TestFormatAndUnits(unittest.TestCase):
    def test_timestamp_conversion_ns_to_seconds_output(self):
        with tempfile.TemporaryDirectory() as td:
            csv_path = os.path.join(td, "imu0", "data.csv")
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            t0 = 1_400_000_000_000_000_000  # ns
            rows = []
            for i in range(20):
                t = t0 + i * 5_000_000
                rows.append([t, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81])
            with open(csv_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(EUROC_HEADER)
                for r in rows:
                    f.write(",".join(str(x) for x in r) + "\n")

            out_traj = os.path.join(td, "estimated_trajectory.txt")
            meta = os.path.join(td, "metadata.json")
            run_pipeline(td, out_traj, metadata_path=meta, prediction_horizon_s=1.0)

            with open(out_traj, "r", encoding="utf-8") as f:
                lines = [ln.strip() for ln in f if ln.strip() and not ln.startswith("#")]
            self.assertGreaterEqual(len(lines), 20)
            first_ts = float(lines[0].split()[0])
            self.assertAlmostEqual(first_ts, t0 * 1e-9, places=6)
            # all rows must have 8 columns
            for ln in lines:
                parts = ln.split()
                self.assertEqual(len(parts), 8)
                q = np.array([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])], dtype=float)
                self.assertTrue(abs(np.linalg.norm(q) - 1.0) < 1e-3)

    def test_euroc_unit_and_column_order(self):
        with tempfile.TemporaryDirectory() as td:
            csv_path = os.path.join(td, "imu0", "data.csv")
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            # EuRoC: timestamp, gyro_x,y,z, accel_x,y,z
            with open(csv_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(EUROC_HEADER)
                f.write("1000000000,0.1,0.2,0.3,1.0,2.0,9.81\n")
                f.write("1005000000,0.1,0.2,0.3,1.0,2.0,9.81\n")
            data = read_imu(csv_path)
            self.assertTrue(np.allclose(data.gyro[0], [0.1, 0.2, 0.3]))
            self.assertTrue(np.allclose(data.accel[0], [1.0, 2.0, 9.81]))
            self.assertEqual(int(data.timestamps_ns[0]), 1000000000)


if __name__ == "__main__":
    unittest.main()
