"""Real-dataset validation workflow for PathFinder (TUM-VI / EuRoC layout).

Runs three cases on one real recording and writes a single auditable summary:

  clean       - unmodified IMU, trajectory evaluated against recorded poses
  degraded    - reproducible degradation + 5 s terminal failure, blind-window
                prediction evaluated against the same poses
  tier3       - injected dropouts on the clean IMU, resilience evaluation

The estimator receives only IMU data.  Ground truth is passed exclusively to
evaluation functions in ``evaluation.py`` and is never read by ``run_pipeline``.

Example:
    python scripts/validate_real_sequence.py \
        --dataset data/real/tumvi_room4 \
        --sequence tumvi_room4 \
        --out-dir outputs/real/tumvi_room4
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "src")
for p in (SRC, SCRIPTS):
    if p not in sys.path:
        sys.path.insert(0, p)

from degrade_imu import CONFIG, SEED, degrade  # noqa: E402
from evaluate_tier3 import inject_dropouts, load_euroc_csv, write_euroc_csv  # noqa: E402
from evaluation import (  # noqa: E402
    evaluate_trajectory,
    save_evaluation_summary,
    save_metrics_json,
    try_plot_position_error,
    try_plot_trajectory,
)
from main import run_pipeline  # noqa: E402
from prepare_degraded_dataset import EUROC_HEADER, load_euroc_imu  # noqa: E402

RUN_KEYS = [
    "num_input_samples",
    "num_output_samples",
    "processing_time_s",
    "peak_memory_mb",
    "failure_detected",
    "failure_start_t_ns",
    "prediction_generated",
    "prediction_appended_samples",
    "median_dt_s",
]


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def crop_recording(src: str, dst: str, duration_s: float) -> str:
    """Keep the first ``duration_s`` seconds of a timestamped IMU/GT CSV."""
    os.makedirs(os.path.dirname(os.path.abspath(dst)) or ".", exist_ok=True)
    is_ns = src.lower().endswith(".csv")
    t0 = None
    kept = []
    with open(src, "r", encoding="utf-8-sig") as f:
        for line in f:
            if not line.strip() or line.lstrip().startswith("#"):
                kept.append(line)
                continue
            token = line.split(",")[0].split()[0]
            t = float(token) if is_ns else float(token) * 1e9
            if t0 is None:
                t0 = t
            if t - t0 > duration_s * 1e9:
                break
            kept.append(line)
    with open(dst, "w", encoding="utf-8", newline="\n") as f:
        f.writelines(kept)
    return dst


def source_info(args) -> dict:
    imu = os.path.abspath(args.imu_csv or os.path.join(args.dataset, "mav0", "imu0", "data.csv"))
    default_gt_candidates = [
        os.path.join(args.dataset, "mav0", "mocap0", "data.csv"),  # TUM-VI EuroC export
        os.path.join(args.dataset, "mav0", "state_groundtruth_estimate0", "data.csv"),  # EuRoC
        os.path.join(args.dataset, "mav0", "groundtruth_tum.txt"),  # synthetic/demo layout
    ]
    gt = os.path.abspath(args.gt_csv or next((p for p in default_gt_candidates if os.path.isfile(p)), default_gt_candidates[0]))
    for label, path in (("IMU", imu), ("ground truth", gt)):
        if not os.path.isfile(path):
            raise FileNotFoundError(f"{label} file not found: {path}")
    return {
        "dataset": os.path.abspath(args.dataset),
        "sequence": args.sequence,
        "source_url": args.source_url,
        "imu_csv": imu,
        "ground_truth_csv": gt,
        "imu_sha256": sha256(imu),
        "ground_truth_sha256": sha256(gt),
        "ground_truth_usage": "evaluation only; never provided to run_pipeline()",
    }


def make_degraded(imu_csv: str, dst_dir: str, seed: int, cfg: dict | None = None) -> dict:
    ts, gyro, accel = load_euroc_imu(imu_csv)
    df = pd.DataFrame(
        {
            "timestamp": ts,
            "ax": accel[:, 0], "ay": accel[:, 1], "az": accel[:, 2],
            "gx": gyro[:, 0], "gy": gyro[:, 1], "gz": gyro[:, 2],
        }
    )
    out, stats = degrade(df, cfg or dict(CONFIG), seed, ts_unit="ns", accel_unit="ms2", gyro_deg=False)
    out_euroc = out[["timestamp", "gx", "gy", "gz", "ax", "ay", "az"]]
    out_csv = os.path.join(dst_dir, "imu0", "data.csv")
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        f.write(EUROC_HEADER)
        out_euroc.to_csv(f, header=False, index=False, float_format="%.6f", lineterminator="\n")
    with open(os.path.join(dst_dir, "degradation_stats.json"), "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=2)
    return {"input": out_csv, "stats": stats}


def column_order_check(clean_csv: str, degraded_csv: str) -> dict:
    """Evidence that the EuRoC->degrade->EuRoC adapters preserved columns/timestamps."""
    ts0, gyro0, accel0 = load_euroc_imu(clean_csv)
    ts1, gyro1, accel1 = load_euroc_imu(degraded_csv)
    finite0 = np.isfinite(accel0).all(axis=1)
    finite1 = np.isfinite(accel1).all(axis=1)
    return {
        "clean_rows": int(len(ts0)),
        "degraded_rows": int(len(ts1)),
        "degraded_timestamps_present_in_clean_fraction": float(np.isin(ts1, ts0).mean()),
        "clean_initial_accel_norm_m_s2": float(np.linalg.norm(accel0[finite0][0])) if finite0.any() else None,
        "degraded_initial_accel_norm_m_s2": float(np.linalg.norm(accel1[finite1][0])) if finite1.any() else None,
        "degraded_initial_gyro_norm_rad_s": float(np.linalg.norm(gyro1[finite1][0])) if finite1.any() else None,
        "note": "EuRoC order is timestamp, gyro(3), accel(3); a swap would show ~9.81 in the gyro norm.",
    }


def make_dropouts(imu_csv: str, dst_dir: str, drop_prob: float, burst_prob: float, seed: int) -> dict:
    ts, vals = load_euroc_csv(imu_csv)
    ts2, vals2, info = inject_dropouts(ts, vals, drop_prob, burst_prob, seed=seed)
    out_csv = os.path.join(dst_dir, "imu0", "data.csv")
    write_euroc_csv(out_csv, ts2, vals2)
    return {"input": out_csv, "stats": {"drop_prob": drop_prob, "burst_prob": burst_prob, "seed": seed, **info}}


def run_case(name: str, imu_input: str, gt_csv: str, out_dir: str, assoc_tol: float, failure_time_s=None) -> dict:
    case_dir = os.path.join(out_dir, name)
    os.makedirs(case_dir, exist_ok=True)
    est_path = os.path.join(case_dir, "estimated_trajectory.txt")
    meta_path = os.path.join(case_dir, "metadata.json")
    run_meta = run_pipeline(imu_input, est_path, metadata_path=meta_path, prediction_horizon_s=5.0)

    metrics = evaluate_trajectory(
        gt_csv,
        est_path,
        align=True,
        max_assoc_diff_s=assoc_tol,
        failure_time_s=failure_time_s,
        blind_horizon_s=5.0,
    )
    save_metrics_json(metrics, os.path.join(case_dir, "metrics.json"))
    paths = save_evaluation_summary(
        {"case": name, "metrics": metrics}, case_dir, basename=f"{name}_summary"
    )
    plots = {
        "trajectory_3d": try_plot_trajectory(
            gt_csv, est_path, os.path.join(case_dir, "trajectory_3d.png"), title=f"Real data - {name}"
        ),
        "position_error": try_plot_position_error(
            metrics, gt_csv, est_path, os.path.join(case_dir, "position_error.png")
        ),
    }
    return {
        "case": name,
        "input": os.path.abspath(imu_input),
        "run": {k: run_meta.get(k) for k in RUN_KEYS},
        "metrics": metrics,
        "summary_files": paths,
        "plots": plots,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", required=True, help="Dataset folder containing mav0/imu0/data.csv")
    ap.add_argument("--sequence", default="real_sequence", help="Dataset identifier for the report")
    ap.add_argument("--gt-csv", default=None, help="Override ground-truth CSV path")
    ap.add_argument("--imu-csv", default=None, help="Override IMU CSV path")
    ap.add_argument("--source-url", default=None, help="Provenance URL recorded in the summary")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "outputs", "real_validation"))
    ap.add_argument("--assoc-tol", type=float, default=0.005,
                    help="Timestamp association tolerance in seconds (TUM-VI GT is 120 Hz)")
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--drop-prob", type=float, default=0.03)
    ap.add_argument("--burst-prob", type=float, default=0.001)
    ap.add_argument("--only", nargs="*", choices=["clean", "degraded", "tier3"], default=None)
    ap.add_argument("--duration-s", type=float, default=None,
                    help="Use only the first N seconds of the recording (useful for short-window analysis)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    info = source_info(args)
    imu_csv = info["imu_csv"]
    gt_csv = info["ground_truth_csv"]
    if args.duration_s is not None:
        crop_dir = os.path.join(args.out_dir, "cropped")
        imu_csv = crop_recording(imu_csv, os.path.join(crop_dir, "imu0", "data.csv"), args.duration_s)
        gt_csv = crop_recording(gt_csv, os.path.join(crop_dir, "groundtruth.csv"), args.duration_s)
        info["cropped_duration_s"] = args.duration_s
        info["cropped_imu_csv"] = imu_csv
        info["cropped_ground_truth_csv"] = gt_csv
    selected = set(args.only or ["clean", "degraded", "tier3"])
    results = {}

    if "clean" in selected:
        print("running clean case ...", file=sys.stderr)
        results["clean"] = run_case("clean", imu_csv, gt_csv, args.out_dir, args.assoc_tol)
    if "degraded" in selected:
        print("preparing degraded dataset ...", file=sys.stderr)
        degraded_dir = os.path.join(args.out_dir, "degraded_dataset")
        degraded = make_degraded(imu_csv, degraded_dir, args.seed)
        meta_stats = degraded["stats"]
        failure_ns = None
        # determine failure timestamp from the generated CSV's last failure window start
        ts, _ = load_euroc_csv(degraded["input"])
        rate = 1.0 / float(np.median(np.diff(ts)) * 1e-9) if len(ts) > 2 else 200.0
        failure_ns = int(ts[-1]) - int(round(float(meta_stats.get("failure_frames", 0)) / rate * 1e9))
        print("running degraded case ...", file=sys.stderr)
        results["degraded"] = run_case(
            "degraded",
            degraded_dir,
            gt_csv,
            args.out_dir,
            args.assoc_tol,
            failure_time_s=failure_ns * 1e-9 if failure_ns is not None else None,
        )
        results["degraded"]["degradation_stats"] = meta_stats
        results["degraded"]["column_order_check"] = column_order_check(imu_csv, degraded["input"])
        results["degraded"]["failure_time_from_degradation_s"] = failure_ns * 1e-9 if failure_ns else None
    if "tier3" in selected:
        print("preparing dropout dataset ...", file=sys.stderr)
        dropout_dir = os.path.join(args.out_dir, "dropout_dataset")
        dropouts = make_dropouts(imu_csv, dropout_dir, args.drop_prob, args.burst_prob, args.seed)
        print("running tier3 case ...", file=sys.stderr)
        results["tier3"] = run_case(
            "tier3", dropout_dir, gt_csv, args.out_dir, args.assoc_tol
        )
        results["tier3"]["dropout_stats"] = dropouts["stats"]

    summary = {
        "workflow": "real_dataset_validation",
        "source": info,
        "configuration": {
            "prediction_horizon_s": 5.0,
            "association_tolerance_s": args.assoc_tol,
            "degradation_seed": args.seed,
            "drop_prob": args.drop_prob,
            "burst_prob": args.burst_prob,
            "degradation_config": {k: v for k, v in CONFIG.items() if k in {"max_level", "gamma", "failure_seconds"}},
            "duration_s": args.duration_s,
        },
        "cases": results,
    }
    out_json = os.path.join(args.out_dir, "real_validation_summary.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps({"summary": out_json, "cases": sorted(results)}, indent=2))


if __name__ == "__main__":
    main()
