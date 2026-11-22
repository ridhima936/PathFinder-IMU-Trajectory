"""Tier 3 evaluation: dropout and corruption resilience."""

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

from evaluation import evaluate_trajectory, save_evaluation_summary, try_plot_position_error, try_plot_trajectory
from main import run_pipeline

EUROC_HEADER = (
    "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
    "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
    "a_RS_S_z [m s^-2]\n"
)


def load_euroc_csv(path: str):
    ts = np.loadtxt(path, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    vals = np.loadtxt(path, delimiter=",", comments="#", usecols=range(1, 7), dtype=np.float64)
    return ts, vals


def write_euroc_csv(path: str, ts: np.ndarray, vals: np.ndarray):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    df = pd.DataFrame({
        "timestamp": ts.astype(np.int64),
        "gx": vals[:, 0], "gy": vals[:, 1], "gz": vals[:, 2],
        "ax": vals[:, 3], "ay": vals[:, 4], "az": vals[:, 5],
    })
    with open(path, "w", newline="") as f:
        f.write(EUROC_HEADER)
        df.to_csv(f, header=False, index=False, float_format="%.9f", lineterminator="\n")


def inject_dropouts(ts: np.ndarray, vals: np.ndarray, drop_prob: float, burst_prob: float, seed: int = 123):
    rng = np.random.default_rng(seed)
    n = len(ts)
    keep = np.ones(n, dtype=bool)
    single = rng.random(n) < drop_prob
    keep &= ~single
    burst_starts = np.flatnonzero(rng.random(n) < burst_prob)
    for i in burst_starts:
        L = int(rng.integers(3, 30))
        keep[i:min(n, i + L)] = False
    keep[0] = True
    keep[-1] = True
    return ts[keep], vals[keep], {"dropped": int(n - np.sum(keep)), "kept": int(np.sum(keep)), "n_in": int(n)}


def main():
    ap = argparse.ArgumentParser(description="Tier 3: dropout and corruption resilience evaluation")
    ap.add_argument("--input", required=True, help="Clean or baseline IMU input dataset folder/csv")
    ap.add_argument("--gt", default=None, help="Ground truth for evaluation")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "outputs", "tier3"))
    ap.add_argument("--drop-prob", type=float, default=0.03)
    ap.add_argument("--burst-prob", type=float, default=0.001)
    ap.add_argument("--seed", type=int, default=123)
    ap.add_argument("--prediction-horizon", type=float, default=5.0)
    ap.add_argument("--use-existing-dropout-file", default=None, help="Use an already corrupted/dropout IMU CSV instead of injecting")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    if args.use_existing_dropout_file:
        run_input = args.use_existing_dropout_file
        dropout_info = {"mode": "existing_file", "path": os.path.abspath(run_input)}
    else:
        # resolve input csv
        in_path = args.input
        if os.path.isdir(in_path):
            cand = os.path.join(in_path, "imu0", "data.csv")
            if not os.path.exists(cand):
                cand = os.path.join(in_path, "mav0", "imu0", "data.csv")
            in_path = cand
        ts, vals = load_euroc_csv(in_path)
        ts2, vals2, info = inject_dropouts(ts, vals, args.drop_prob, args.burst_prob, seed=args.seed)
        dropout_csv = os.path.join(args.out_dir, "imu0_dropout", "data.csv")
        write_euroc_csv(dropout_csv, ts2, vals2)
        run_input = os.path.join(args.out_dir, "imu0_dropout")
        dropout_info = {"mode": "injected", **info, "drop_prob": args.drop_prob, "burst_prob": args.burst_prob, "seed": args.seed, "generated_csv": dropout_csv}

    est_path = os.path.join(args.out_dir, "estimated_trajectory.txt")
    meta_path = os.path.join(args.out_dir, "metadata.json")
    run_meta = run_pipeline(run_input, est_path, metadata_path=meta_path, prediction_horizon_s=args.prediction_horizon)

    summary = {
        "tier": "tier3_dropout_resilience",
        "input": os.path.abspath(args.input),
        "run_input": os.path.abspath(run_input),
        "ground_truth": os.path.abspath(args.gt) if args.gt else None,
        "dropout": dropout_info,
        "run": {k: run_meta[k] for k in ["num_input_samples", "num_output_samples", "processing_time_s", "peak_memory_mb", "failure_detected", "prediction_generated"]},
    }

    if args.gt and os.path.exists(args.gt):
        metrics = evaluate_trajectory(args.gt, est_path, align=True, blind_horizon_s=args.prediction_horizon)
        summary["metrics"] = metrics
        summary["plots"] = {
            "trajectory_3d": try_plot_trajectory(args.gt, est_path, os.path.join(args.out_dir, "trajectory_3d.png"), title="Tier 3 Dropout Resilience"),
            "position_error": try_plot_position_error(metrics, args.gt, est_path, os.path.join(args.out_dir, "position_error.png")),
        }
    else:
        summary["metrics"] = None
        summary["note"] = "Ground truth not provided; resilience run completed without ATE/RPE comparison."

    paths = save_evaluation_summary(summary, args.out_dir, basename="tier3_summary")
    print(json.dumps({"summary_json": paths["json"], "summary_csv": paths["csv"], "trajectory": est_path}, indent=2))


if __name__ == "__main__":
    main()
