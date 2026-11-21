"""Tier 2 evaluation: blind 5-second future trajectory prediction after IMU failure."""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from evaluation import evaluate_trajectory, save_evaluation_summary, try_plot_position_error, try_plot_trajectory
from main import run_pipeline


def main():
    ap = argparse.ArgumentParser(description="Tier 2: blind 5s prediction after IMU failure")
    ap.add_argument("--input", required=True, help="Degraded IMU input containing terminal failure (e.g. last 5s zeros)")
    ap.add_argument("--gt", default=None, help="Ground truth for post-failure/blind-window evaluation")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "outputs", "tier2"))
    ap.add_argument("--prediction-horizon", type=float, default=5.0)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    est_path = os.path.join(args.out_dir, "estimated_trajectory.txt")
    meta_path = os.path.join(args.out_dir, "metadata.json")
    run_meta = run_pipeline(args.input, est_path, metadata_path=meta_path, prediction_horizon_s=args.prediction_horizon)

    summary = {
        "tier": "tier2_blind_prediction",
        "input": os.path.abspath(args.input),
        "ground_truth": os.path.abspath(args.gt) if args.gt else None,
        "run": {k: run_meta[k] for k in ["num_input_samples", "num_output_samples", "processing_time_s", "peak_memory_mb", "failure_detected", "failure_start_t_ns", "prediction_generated", "prediction_appended_samples"]},
    }

    if args.gt and os.path.exists(args.gt):
        metrics = evaluate_trajectory(args.gt, est_path, align=True, blind_horizon_s=args.prediction_horizon)
        summary["metrics"] = metrics
        summary["blind_window_focus"] = metrics.get("blind_window")
        summary["plots"] = {
            "trajectory_3d": try_plot_trajectory(args.gt, est_path, os.path.join(args.out_dir, "trajectory_3d.png"), title="Tier 2 Blind Prediction"),
            "position_error": try_plot_position_error(metrics, args.gt, est_path, os.path.join(args.out_dir, "position_error.png")),
        }
    else:
        summary["metrics"] = None
        summary["note"] = "Ground truth not provided; prediction output generated but blind-window accuracy not measured."

    paths = save_evaluation_summary(summary, args.out_dir, basename="tier2_summary")
    print(json.dumps({"summary_json": paths["json"], "summary_csv": paths["csv"], "trajectory": est_path}, indent=2))


if __name__ == "__main__":
    main()
