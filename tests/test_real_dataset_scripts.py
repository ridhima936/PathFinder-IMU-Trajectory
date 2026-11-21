import io
import os
import sys
import tarfile
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from fetch_tumvi_subset import extract_subset  # noqa: E402
from validate_real_sequence import column_order_check, crop_recording, make_degraded, make_dropouts, sha256  # noqa: E402

EUROC_HEADER = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"


def write_euroc_csv(path, ts, gyro, accel):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(EUROC_HEADER)
        for i in range(len(ts)):
            row = [int(ts[i]), *(float(x) for x in gyro[i]), *(float(x) for x in accel[i])]
            f.write(",".join(f"{x:.9f}" if j else str(x) for j, x in enumerate(row)) + "\n")


class TestRealDatasetScripts(unittest.TestCase):
    def test_crop_recording(self):
        with tempfile.TemporaryDirectory() as td:
            src = os.path.join(td, "in.csv")
            with open(src, "w") as f:
                f.write("#header\n")
                for i in range(100):
                    f.write(f"{1_000_000_000 + i * 5_000_000},0,0,0,0,0,9.81\n")
            dst = crop_recording(src, os.path.join(td, "crop.csv"), 0.25)
            with open(dst) as f:
                lines = f.readlines()
            self.assertEqual(lines[0].strip(), "#header")
            self.assertEqual(len(lines), 1 + 51)

    def test_tar_subset_extraction(self):
        with tempfile.TemporaryDirectory() as td:
            archive = os.path.join(td, "sequence.tar")
            imu_payload = b"#imu\n1,2,3\n"
            gt_payload = b"#gt\n4,5,6\n"
            with tarfile.open(archive, "w") as tf:
                for name, payload in [
                    ("dataset/mav0/cam0/data/frame0.png", b"x" * 4096),
                    ("dataset/mav0/imu0/data.csv", imu_payload),
                    ("dataset/mav0/state_groundtruth_estimate0/data.csv", gt_payload),
                ]:
                    info = tarfile.TarInfo(name)
                    info.size = len(payload)
                    tf.addfile(info, io.BytesIO(payload))
            out_dir = os.path.join(td, "dataset")
            paths = extract_subset(archive, out_dir)
            with open(paths["imu"], "rb") as f:
                self.assertEqual(f.read(), imu_payload)
            with open(paths["gt"], "rb") as f:
                self.assertEqual(f.read(), gt_payload)

    def test_degraded_and_dropout_preparation(self):
        with tempfile.TemporaryDirectory() as td:
            n = 2401  # 12 s at 200 Hz: long enough for the 5 s failure window
            ts = 1_400_000_000_000_000_000 + np.arange(n, dtype=np.int64) * 5_000_000
            gyro = np.zeros((n, 3), dtype=np.float64)
            accel = np.zeros((n, 3), dtype=np.float64)
            accel[:, 2] = 9.81
            imu_csv = os.path.join(td, "clean", "mav0", "imu0", "data.csv")
            write_euroc_csv(imu_csv, ts, gyro, accel)

            degraded = make_degraded(imu_csv, os.path.join(td, "degraded"), seed=42)
            self.assertGreaterEqual(degraded["stats"]["failure_frames"], 900)
            self.assertEqual(degraded["stats"]["rows_in"], n)
            deg_ts = np.loadtxt(degraded["input"], delimiter=",", comments="#", usecols=0, dtype=np.int64)
            self.assertGreater(len(deg_ts), 0)
            check = column_order_check(imu_csv, degraded["input"])
            self.assertEqual(check["degraded_timestamps_present_in_clean_fraction"], 1.0)
            self.assertLess(check["degraded_initial_gyro_norm_rad_s"], 0.1)

            dropouts = make_dropouts(imu_csv, os.path.join(td, "dropouts"), 0.05, 0.0, seed=7)
            self.assertEqual(dropouts["stats"]["n_in"], n)
            self.assertLess(dropouts["stats"]["kept"], n)
            self.assertEqual(sha256(imu_csv), sha256(imu_csv))


if __name__ == "__main__":
    unittest.main()
