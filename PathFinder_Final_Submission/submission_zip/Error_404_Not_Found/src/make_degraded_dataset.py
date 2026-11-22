"""
make_degraded_dataset.py
EuRoC IMU folder lo -> organiser ki degrade_imu.py se degrade karo
-> wapas EuRoC format (imu0/data.csv) mein likho.

Usage (repo root se):
    py src/make_degraded_dataset.py data/synthetic/mav0 data/synthetic_degraded
    py src/make_degraded_dataset.py data/MH_01_easy/mav0 data/MH_01_easy_degraded
"""
import argparse
import json
import os

import numpy as np
import pandas as pd

from degrade_imu import degrade, CONFIG, SEED  # degrade_imu.py isi folder (src/) mein rakhna

EUROC_HEADER = (
    "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
    "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
    "a_RS_S_z [m s^-2]\n"
)


def load_euroc_imu(csv_path):
    """EuRoC imu0/data.csv padho. Return: ts (int64 ns), gyro (N,3), accel (N,3)."""
    # timestamp alag padhte hain int64 mein, warna float mein precision chali jaati hai
    ts = np.loadtxt(csv_path, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    vals = np.loadtxt(csv_path, delimiter=",", comments="#",
                      usecols=range(1, 7), dtype=np.float64)
    gyro = vals[:, 0:3]    # EuRoC mein pehle gyro
    accel = vals[:, 3:6]   # phir accel
    return ts, gyro, accel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="folder jisme imu0/data.csv hai (e.g. .../mav0)")
    ap.add_argument("dst", help="output folder (isme imu0/data.csv banega)")
    ap.add_argument("--max-level", type=float, default=None)
    ap.add_argument("--gamma", type=float, default=None)
    args = ap.parse_args()

    ts, gyro, accel = load_euroc_imu(os.path.join(args.src, "imu0", "data.csv"))

    # degrade_imu.py ko order chahiye: timestamp, accel, gyro (EuRoC se ulta!)
    df = pd.DataFrame({
        "timestamp": ts,
        "ax": accel[:, 0], "ay": accel[:, 1], "az": accel[:, 2],
        "gx": gyro[:, 0], "gy": gyro[:, 1], "gz": gyro[:, 2],
    })

    cfg = dict(CONFIG)
    if args.max_level is not None:
        cfg["max_level"] = args.max_level
    if args.gamma is not None:
        cfg["gamma"] = args.gamma

    # units hum jaante hain, isliye auto-detect ki jagah force kar rahe hain
    out, stats = degrade(df, cfg, SEED, ts_unit="ns", accel_unit="ms2", gyro_deg=False)

    # wapas EuRoC order: timestamp, gyro, accel
    out = out[["timestamp", "gx", "gy", "gz", "ax", "ay", "az"]]

    out_dir = os.path.join(args.dst, "imu0")
    os.makedirs(out_dir, exist_ok=True)
    out_csv = os.path.join(out_dir, "data.csv")
    with open(out_csv, "w", newline="") as f:
        f.write(EUROC_HEADER)
        out.to_csv(f, header=False, index=False, float_format="%.6f", lineterminator="\n")

    with open(os.path.join(args.dst, "degradation_stats.json"), "w") as f:
        json.dump(stats, f, indent=2)

    print(f"rows_in={stats['rows_in']}  rows_out={stats['rows_out']}")
    print(f"dropped_total={stats['dropped_total']}  duplicated={stats['duplicated_rows']}")
    print(f"failure_frames={stats['failure_frames']} (last ~5 s zero honge)")
    print(f"written: {out_csv}")


if __name__ == "__main__":
    main()
