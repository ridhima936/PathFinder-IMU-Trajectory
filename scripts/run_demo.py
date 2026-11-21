"""Reproducible demonstration workflow for PathFinder.

Generates:
1. Clean synthetic IMU input
2. Progressively degraded IMU input (with terminal failure)
3. Estimated trajectories for clean and degraded cases
4. 5-second predicted trajectory segment (from degraded/failure case)
5. Ground-truth comparison plots where available
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")
PY = sys.executable

if SRC not in sys.path:
    sys.path.insert(0, SRC)

from evaluation import evaluate_trajectory, save_evaluation_summary, try_plot_position_error, try_plot_trajectory
from main import run_pipeline


def run(cmd, cwd=None):
    print("+", " ".join(cmd))
    r = subprocess.run(cmd, cwd=cwd or ROOT, capture_output=True, text=True)
    if r.stdout:
        print(r.stdout.strip())
    if r.stderr:
        print(r.stderr.strip())
    if r.returncode != 0:
        raise RuntimeError(f"Command failed ({r.returncode}): {' '.join(cmd)}")
    return r


def main():
    demo_dir = os.path.join(ROOT, "outputs", "demo")
    clean_dir = os.path.join(demo_dir, "clean_data")
    degraded_dir = os.path.join(demo_dir, "degraded_data")
    os.makedirs(demo_dir, exist_ok=True)

    # 1) Clean synthetic input with rest/ramp and ground truth
    run([PY, os.path.join(SRC, "make_synthetic_imu.py"), "--duration", "30", "--rate", "200", "--rest", "2", "--ramp", "2", "--out", clean_dir])

    # 2) Progressively degraded IMU input (uses original degrade_imu.py via adapter script)
    run([PY, os.path.join(ROOT, "scripts", "prepare_degraded_dataset.py"), os.path.join(clean_dir, "mav0"), os.path.join(degraded_dir, "mav0")])

    clean_input = os.path.join(clean_dir, "mav0")
    degraded_input = os.path.join(degraded_dir, "mav0")
    gt_path = os.path.join(clean_dir, "mav0", "groundtruth_tum.txt")

    # 3) Estimate clean trajectory
    clean_est = os.path.join(demo_dir, "estimated_clean.txt")
    clean_meta = os.path.join(demo_dir, "metadata_clean.json")
    run_meta_clean = run_pipeline(clean_input, clean_est, metadata_path=clean_meta, prediction_horizon_s=5.0)

    # 4) Estimate degraded trajectory (includes failure handling and 5s prediction)
    deg_est = os.path.join(demo_dir, "estimated_degraded.txt")
    deg_meta = os.path.join(demo_dir, "metadata_degraded.json")
    run_meta_deg = run_pipeline(degraded_input, deg_est, metadata_path=deg_meta, prediction_horizon_s=5.0)

    summary = {
        "demo": "PathFinder reproducible demonstration",
        "inputs": {
            "clean_imu": os.path.abspath(clean_input),
            "degraded_imu": os.path.abspath(degraded_input),
            "ground_truth": os.path.abspath(gt_path),
        },
        "outputs": {
            "clean_estimated": os.path.abspath(clean_est),
            "degraded_estimated": os.path.abspath(deg_est),
            "clean_metadata": os.path.abspath(clean_meta),
            "degraded_metadata": os.path.abspath(deg_meta),
        },
        "runs": {
            "clean": {k: run_meta_clean[k] for k in ["num_input_samples", "num_output_samples", "processing_time_s", "peak_memory_mb", "failure_detected", "prediction_generated"]},
            "degraded": {k: run_meta_deg[k] for k in ["num_input_samples", "num_output_samples", "processing_time_s", "peak_memory_mb", "failure_detected", "failure_start_t_ns", "prediction_generated", "prediction_appended_samples"]},
        },
        "plots": {},
        "metrics": {},
    }

    if os.path.exists(gt_path):
        m_clean = evaluate_trajectory(gt_path, clean_est, align=True, blind_horizon_s=5.0)
        m_deg = evaluate_trajectory(gt_path, deg_est, align=True, blind_horizon_s=5.0)
        summary["metrics"]["clean_vs_gt"] = m_clean
        summary["metrics"]["degraded_vs_gt"] = m_deg
        summary["plots"]["clean_trajectory_3d"] = try_plot_trajectory(gt_path, clean_est, os.path.join(demo_dir, "clean_trajectory_3d.png"), "Clean IMU Trajectory")
        summary["plots"]["degraded_trajectory_3d"] = try_plot_trajectory(gt_path, deg_est, os.path.join(demo_dir, "degraded_trajectory_3d.png"), "Degraded IMU + Failure + 5s Prediction")
        summary["plots"]["clean_position_error"] = try_plot_position_error(m_clean, gt_path, clean_est, os.path.join(demo_dir, "clean_position_error.png"))
        summary["plots"]["degraded_position_error"] = try_plot_position_error(m_deg, gt_path, deg_est, os.path.join(demo_dir, "degraded_position_error.png"))
    else:
        summary["note"] = "Ground truth not found; plots/metrics limited."

    paths = save_evaluation_summary(summary, demo_dir, basename="demo_summary")
    print(json.dumps({"demo_summary": paths, "demo_dir": os.path.abspath(demo_dir)}, indent=2))


if __name__ == "__main__":
    main()
