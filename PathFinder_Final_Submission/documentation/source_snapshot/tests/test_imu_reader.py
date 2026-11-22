import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from imu_reader import read_imu, validate_imu_data


EUROC_HEADER = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"


def write_euroc_csv(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(EUROC_HEADER)
        for r in rows:
            f.write(",".join(str(x) for x in r) + "\n")


class TestIMUReader(unittest.TestCase):
    def test_load_euroc_csv_and_order(self):
        with tempfile.TemporaryDirectory() as td:
            csv_path = os.path.join(td, "imu0", "data.csv")
            rows = [
                [1000, 0.1, 0.2, 0.3, 0.0, 0.0, 9.81],
                [1000 + 5_000_000, 0.11, 0.21, 0.31, 0.1, 0.0, 9.80],
            ]
            write_euroc_csv(csv_path, rows)
            data = read_imu(td)
            self.assertEqual(data.n_samples, 2)
            self.assertTrue(np.allclose(data.gyro[0], [0.1, 0.2, 0.3]))
            self.assertTrue(np.allclose(data.accel[0], [0.0, 0.0, 9.81]))
            report = validate_imu_data(data)
            self.assertTrue(report["ok"], msg=report)

    def test_timestamp_validation_and_stats(self):
        with tempfile.TemporaryDirectory() as td:
            csv_path = os.path.join(td, "imu0", "data.csv")
            rows = [
                [1000, 0, 0, 0, 0, 0, 9.81],
                [1000, 0, 0, 0, 0, 0, 9.81],  # duplicate
                [900, 0, 0, 0, 0, 0, 9.81],   # non-monotonic
                [2000, 0, 0, 0, 0, 0, 9.81],
            ]
            write_euroc_csv(csv_path, rows)
            data = read_imu(td)
            self.assertEqual(data.n_samples, 4)
            self.assertGreaterEqual(int(data.meta.get("duplicate_timestamps", 0)), 1)
            self.assertGreaterEqual(int(data.meta.get("non_monotonic_timestamps", 0)), 1)
            report = validate_imu_data(data)
            self.assertFalse(report["ok"])

    def test_missing_and_malformed_values(self):
        with tempfile.TemporaryDirectory() as td:
            csv_path = os.path.join(td, "imu0", "data.csv")
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            with open(csv_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(EUROC_HEADER)
                f.write("1000,0,0,0,0,0,9.81\n")
                f.write("bad,line,not,a,timestamp\n")
                f.write("2000,nan,0,0,0,0,9.81\n")
            data = read_imu(csv_path, strict=False)
            self.assertEqual(data.n_samples, 2)
            self.assertGreaterEqual(int(data.meta.get("malformed_rows", 0)) + int(data.meta.get("skipped_lines", 0)), 1)
            self.assertGreaterEqual(int(data.meta.get("invalid_value_rows", 0)), 1)

    def test_tumvi_adapter_layout(self):
        # TUM-VI EuRoC-style imu0/data.csv should be read the same way
        with tempfile.TemporaryDirectory() as td:
            csv_path = os.path.join(td, "mav0", "imu0", "data.csv")
            rows = [
                [100, 0.01, 0.02, 0.03, 0.0, 0.1, 9.7],
                [100 + 5_000_000, 0.02, 0.03, 0.04, 0.0, 0.1, 9.7],
            ]
            write_euroc_csv(csv_path, rows)
            data = read_imu(td, fmt="tumvi")
            self.assertEqual(data.n_samples, 2)
            self.assertTrue(np.allclose(data.gyro[1], [0.02, 0.03, 0.04]))
            self.assertTrue(np.allclose(data.accel[1], [0.0, 0.1, 9.7]))


if __name__ == "__main__":
    unittest.main()
