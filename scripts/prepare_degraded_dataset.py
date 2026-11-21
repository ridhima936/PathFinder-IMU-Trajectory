"""Prepare a degraded EuRoC-format IMU dataset with explicit column-order conversion.

Preserves original `src/degrade_imu.py` and uses it as the degradation engine.
EuRoC IMU input order:  timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
degrade_imu.py expects: timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

This script converts explicitly and never swaps columns implicitly.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from degrade_imu import CONFIG, SEED, degrade  # noqa: E402

EUROC_HEADER = (
    "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
    "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
    "a_RS_S_z [m s^-2]\n"
)


def load_euroc_imu(csv_path: str):
    ts = np.loadtxt(csv_path, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    vals = np.loadtxt(csv_path, delimiter=",", comments="#", usecols=range(1, 7), dtype=np.float64)
    gyro = vals[:, 0:3]
    accel = vals[:, 3:6]
    return ts, gyro, accel


def main():
    ap = argparse.ArgumentParser(description="Create degraded EuRoC IMU dataset with reproducible stats")
    ap.add_argument("src", help="Source dataset folder containing imu0/data.csv (e.g. .../mav0)")
    ap.add_argument("dst", help="Destination dataset folder (will contain imu0/data.csv and degradation_stats.json)")
    ap.add_argument("--max-level", type=float, default=None)
    ap.add_argument("--gamma", type=float, default=None)
    ap.add_argument("--seed", type=int, default=SEED, help="Random seed (default matches degrade_imu.SEED)")
    ap.add_argument("--in-csv", default=None, help="Override input IMU CSV path")
    ap.add_argument("--out-csv", default=None, help="Override output IMU CSV path")
    ap.add_argument("--meta", default=None, help="Override output stats JSON path")
    args = ap.parse_args()

    in_csv = args.in_csv or os.path.join(args.src, "imu0", "data.csv")
    out_csv = args.out_csv or os.path.join(args.dst, "imu0", "data.csv")
    meta_path = args.meta or os.path.join(args.dst, "degradation_stats.json")

    ts, gyro, accel = load_euroc_imu(in_csv)

    # Explicit conversion: EuRoC (ts, gyro, accel) -> degrade_imu (ts, accel, gyro)
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

    out, stats = degrade(df, cfg, args.seed, ts_unit="ns", accel_unit="ms2", gyro_deg=False)

    # Explicit conversion back: degrade_imu columns (timestamp, accel..., gyro...) -> EuRoC (timestamp, gyro, accel)
    out_euroc = out[["timestamp", "gx", "gy", "gz", "ax", "ay", "az"]]

    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        f.write(EUROC_HEADER)
        out_euroc.to_csv(f, header=False, index=False, float_format="%.6f", lineterminator="\n")

    os.makedirs(os.path.dirname(meta_path), exist_ok=True)
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=2)

    print(f"input:  {in_csv}")
    print(f"output: {out_csv}")
    print(f"stats:  {meta_path}")
    print(f"rows_in={stats['rows_in']} rows_out={stats['rows_out']} dropped_total={stats['dropped_total']} duplicated={stats['duplicated_rows']} failure_frames={stats['failure_frames']}")
    print(f"seed={stats['seed']} severity_start={stats['severity_start']:.3f} severity_end={stats['severity_end']:.3f}")


if __name__ == "__main__":
    main()
