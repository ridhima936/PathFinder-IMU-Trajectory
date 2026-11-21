"""
PathFinder - IMU trajectory estimation under sensor degradation.

Pipeline:
    Raw IMU CSV/folder/zip -> validation -> preprocessing -> sensor correction
    -> orientation -> gravity compensation -> velocity/position -> failure detection
    -> 5-second dead reckoning -> trajectory output
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import tracemalloc
from dataclasses import asdict
from typing import Dict, List, Optional, Tuple

import numpy as np

# Allow running as `python src/main.py` or from repo root
_SRC_DIR = os.path.dirname(os.path.abspath(__file__))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from dead_reckoning import DeadReckoningConfig, MotionHistory, predict_future_trajectory
from failure_detection import FailureConfig, FailureDetector, FailureState
from imu_reader import read_imu, validate_imu_data
from orientation import OrientationConfig, quat_to_tum
from preprocessing import PreprocessConfig, preprocess_imu
from sensor_correction import SensorCorrectionConfig, SensorCorrector
from trajectory import TrajectoryConfig, TrajectoryEstimator


def _ns_to_tum_ts(t_ns: int) -> str:
    return f"{int(t_ns) // 1_000_000_000}.{int(t_ns) % 1_000_000_000:09d}"


def _write_tum_pose_line(f, t_ns: int, p: np.ndarray, q_wxyz: np.ndarray):
    qx, qy, qz, qw = quat_to_tum(q_wxyz)
    f.write(
        f"{_ns_to_tum_ts(int(t_ns))} "
        f"{float(p[0]):.9f} {float(p[1]):.9f} {float(p[2]):.9f} "
        f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
    )


def _estimate_median_dt(timestamps_ns: np.ndarray) -> float:
    if len(timestamps_ns) < 2:
        return 0.005
    dt = np.diff(timestamps_ns.astype(np.int64)).astype(np.float64) * 1e-9
    pos = dt[dt > 0]
    if pos.size == 0:
        return 0.005
    return float(np.median(pos))


def run_pipeline(
    input_path: str,
    output_path: str,
    metadata_path: Optional[str] = None,
    imu_format: str = "auto",
    prediction_horizon_s: float = 5.0,
    prediction_interval_s: Optional[float] = None,
    config: Optional[Dict[str, object]] = None,
) -> Dict[str, object]:
    cfg = config or {}
    t_wall_start = time.perf_counter()
    tracemalloc.start()

    # 1) Read & validate
    data = read_imu(input_path, fmt=imu_format, strict=False)
    validation = validate_imu_data(data)
    median_dt = _estimate_median_dt(data.timestamps_ns)

    # 2) Preprocess
    pre_cfg = PreprocessConfig()
    # allow reasonable dt cap
    if median_dt > 0:
        pre_cfg.max_dt_s = max(0.05, 10.0 * median_dt)
    pre = preprocess_imu(data.timestamps_ns, data.gyro, data.accel, pre_cfg)

    # 3) Correction, orientation, trajectory, failure detection
    orient_cfg = OrientationConfig()
    traj_cfg = TrajectoryConfig(gravity_magnitude=float(pre.gravity_magnitude))
    traj_cfg.max_dt_s = pre_cfg.max_dt_s or traj_cfg.max_dt_s
    corr_cfg = SensorCorrectionConfig()
    failure_cfg = FailureConfig()

    corrector = SensorCorrector(corr_cfg, gyro_bias_init=pre.gyro_bias_init, accel_bias_init=pre.accel_bias_init,
                                gravity_magnitude=pre.gravity_magnitude)
    estimator = TrajectoryEstimator(traj_cfg, orientation_config=orient_cfg)
    detector = FailureDetector(failure_cfg, median_dt_s=median_dt)
    history = MotionHistory(maxlen=400)

    # 4) Initialize using a short window to reduce startup sensitivity
    init_n = min(len(pre.timestamps_ns), max(5, int(round(max(0.2, pre_cfg.initial_bias_window_s) / max(median_dt, 1e-6)))))
    init_n = max(1, init_n)
    # select finite samples for alignment
    init_acc = pre.accel[:init_n]
    finite_init = np.all(np.isfinite(init_acc), axis=1)
    acc_align = np.mean(init_acc[finite_init], axis=0) if np.any(finite_init) else (init_acc[0] if len(init_acc) else np.array([0, 0, 9.81]))
    estimator.initialize(int(pre.timestamps_ns[0]), accel_body_for_alignment=acc_align)
    estimator.config.gravity_magnitude = float(pre.gravity_magnitude)
    estimator.gravity_world = np.array([0.0, 0.0, -abs(float(pre.gravity_magnitude))], dtype=np.float64)

    n = len(pre.timestamps_ns)
    out_dir = os.path.dirname(os.path.abspath(output_path)) if os.path.dirname(output_path) else os.getcwd()
    os.makedirs(out_dir, exist_ok=True)

    rows_out = 0
    last_emitted_t_ns: Optional[int] = None
    prediction_generated = False
    prediction_start_t_ns: Optional[int] = None
    prediction_count = 0
    # to ensure we do not duplicate timestamps and we cover 5s prediction
    output_t_ns_set = set()

    # For dead samples during/after complete failure, propagate with dead reckoning timing logic
    # but we must emit one pose per input sample timestamp.
    with open(output_path, "w", encoding="utf-8", newline="\n") as out:
        out.write("# timestamp tx ty tz qx qy qz qw\n")

        for i in range(n):
            t_ns = int(pre.timestamps_ns[i])
            gyro_raw = pre.gyro[i]
            accel_raw = pre.accel[i]
            dt = float(pre.dt_s[i]) if i < len(pre.dt_s) else median_dt
            if dt <= 0 or not math.isfinite(dt):
                dt = median_dt

            # correction (applies bias/filter; uses raw validity)
            sample_valid = bool(pre.valid_mask[i]) if i < len(pre.valid_mask) else True
            gyro_c, accel_c = corrector.correct(gyro_raw, accel_raw, dt=dt, sample_valid=sample_valid)

            # failure detection on corrected/raw values? use raw+validity characteristics
            det = detector.observe(i, t_ns, gyro_raw, accel_raw, gap_s=dt)
            state = det["state"]
            dead_sample = bool(det["dead_sample"])

            usable = sample_valid and (not dead_sample) and np.all(np.isfinite(gyro_c)) and np.all(np.isfinite(accel_c))
            if state == FailureState.COMPLETE_FAILURE.value:
                usable = False

            if usable:
                est = estimator.step(t_ns, gyro_c, accel_c, dt, sample_usable=True)
                p = est.p.copy()
                q = est.q.copy()
                # update motion history with world-frame linear acceleration estimate
                # recompute a_world consistent with trajectory step
                from orientation import rotate_vector as _rv
                f_world = _rv(q, accel_c)
                a_world = f_world + np.array([0.0, 0.0, -abs(estimator.config.gravity_magnitude)])
                history.append(t_ns, p, estimator.state.v, a_world, q, gyro_c)
            else:
                # If complete failure started, propagate via dead-reckoning motion model
                if detector.failure_started and len(history) > 0:
                    if prediction_start_t_ns is None:
                        prediction_start_t_ns = t_ns
                    dr_cfg = DeadReckoningConfig(
                        horizon_s=max(prediction_horizon_s, 1.0),
                        interval_s=prediction_interval_s or median_dt,
                        fallback_dt_s=median_dt if median_dt > 0 else 0.005,
                    )
                    k = min(len(history), 20)
                    recent_a = list(history.a_world)[-k:]
                    recent_g = list(history.gyro)[-k:]
                    a_est = np.median(np.stack(recent_a, axis=0), axis=0) if recent_a else np.zeros(3)
                    gyro_est = np.median(np.stack(recent_g, axis=0), axis=0) if recent_g else np.zeros(3)
                    a_est = np.clip(a_est, -dr_cfg.max_accel_m_s2, dr_cfg.max_accel_m_s2)
                    gyro_est = np.clip(gyro_est, -dr_cfg.max_gyro_rad_s, dr_cfg.max_gyro_rad_s)
                    fail_t = detector.failure_start_t_ns if detector.failure_start_t_ns is not None else (history.last()[0] if history.last() else t_ns)
                    t_rel = max(0.0, (t_ns - int(fail_t)) * 1e-9)
                    decay = float(np.exp(-t_rel / max(dr_cfg.accel_decay_tau_s, 1e-3))) if dr_cfg.use_constant_acceleration else 0.0
                    v_base = estimator.state.v.copy()
                    if not np.all(np.isfinite(v_base)):
                        v_base = np.array(list(history.v)[-1], dtype=np.float64) if len(history.v) else np.zeros(3)
                    v_step = v_base + a_est * decay * dt
                    speed = float(np.linalg.norm(v_step))
                    if speed > dr_cfg.max_speed_m_s and speed > 1e-9:
                        v_step = v_step * (dr_cfg.max_speed_m_s / speed)
                    p = estimator.state.p + v_step * dt
                    # orientation propagation
                    from orientation import q_from_rotvec, q_multiply, q_normalize
                    gyro_step = gyro_est * (decay if dr_cfg.use_constant_acceleration else 0.5)
                    q = q_normalize(q_multiply(estimator.state.q, q_from_rotvec(gyro_step * dt)))
                    estimator.state.p = p.copy()
                    estimator.state.v = v_step.copy()
                    estimator.state.q = q.copy()
                    estimator.state.t_ns = t_ns
                    estimator.state.a_world_prev = a_est * decay
                    estimator.state.gyro_prev = gyro_step.copy()
                    # do not append dead/failure samples as valid motion history except state progression
                else:
                    # temporary dropout / unusable sample before confirmed complete failure: propagate conservatively
                    est = estimator.step(t_ns, gyro_c, accel_c, dt, sample_usable=False)
                    p = est.p.copy()
                    q = est.q.copy()

            _write_tum_pose_line(out, t_ns, p, q)
            output_t_ns_set.add(t_ns)
            rows_out += 1
            last_emitted_t_ns = t_ns

        # 5) Ensure 5-second prediction after complete failure
        failure_started = detector.failure_started
        prediction_debug = {"branch": "none", "history_len": len(history)}
        if failure_started:
            failure_t_ns = int(detector.failure_start_t_ns) if detector.failure_start_t_ns is not None else (int(pre.timestamps_ns[-1]) if n else None)
            if prediction_start_t_ns is None:
                prediction_start_t_ns = failure_t_ns
            # compute how much predicted horizon is already covered by emitted samples after failure
            if failure_t_ns is not None and last_emitted_t_ns is not None:
                covered_s = max(0.0, (last_emitted_t_ns - failure_t_ns) * 1e-9)
            else:
                covered_s = 0.0
            remaining = max(0.0, float(prediction_horizon_s) - covered_s)
            prediction_debug.update({"branch": "failure_started", "covered_s": covered_s, "remaining": remaining, "history_len": len(history)})
            # Build prediction anchor from last emitted estimator state to extend beyond already-emitted samples
            pred_history = MotionHistory(maxlen=400)
            if len(history) > 0:
                # seed with recent valid history for trend estimation context
                k_seed = min(len(history), 50)
                for j in range(len(history) - k_seed, len(history)):
                    pred_history.append(list(history.t_ns)[j], list(history.p)[j], list(history.v)[j],
                                        list(history.a_world)[j], list(history.q)[j], list(history.gyro)[j])
            # ensure anchor is at last emitted state/time
            anchor_t = int(last_emitted_t_ns if last_emitted_t_ns is not None else (estimator.state.t_ns or 0))
            pred_history.append(anchor_t, estimator.state.p, estimator.state.v,
                                getattr(estimator.state, "a_world_prev", np.zeros(3)),
                                estimator.state.q, estimator.state.gyro_prev if estimator.state.gyro_prev is not None else np.zeros(3))

            if remaining > 1e-6:
                dr_cfg = DeadReckoningConfig(
                    horizon_s=remaining,
                    interval_s=prediction_interval_s or median_dt,
                    fallback_dt_s=median_dt if median_dt > 0 else 0.005,
                )
                preds = predict_future_trajectory(pred_history, dr_cfg, dt_s=dr_cfg.interval_s)
                prediction_debug.update({"preds_len": len(preds), "interval_s": dr_cfg.interval_s, "anchor_t_ns": anchor_t})
                for pred in preds:
                    t_ns = int(pred["t_ns"])
                    if t_ns in output_t_ns_set:
                        continue
                    # enforce strictly increasing timestamps
                    if last_emitted_t_ns is not None and t_ns <= last_emitted_t_ns:
                        step_ns = int(round((dr_cfg.interval_s or median_dt) * 1e9))
                        t_ns = last_emitted_t_ns + max(step_ns, 1)
                    _write_tum_pose_line(out, t_ns, np.asarray(pred["p"]), np.asarray(pred["q"]))
                    output_t_ns_set.add(t_ns)
                    rows_out += 1
                    prediction_count += 1
                    prediction_generated = True
                    last_emitted_t_ns = t_ns
            else:
                prediction_generated = True  # horizon already covered by dead samples in input
                prediction_debug.update({"preds_len": 0, "covered": True, "anchor_t_ns": anchor_t})

    current_mem, peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    wall_s = time.perf_counter() - t_wall_start

    metadata = {
        "input_dataset": os.path.abspath(input_path),
        "resolved_input": data.meta.get("resolved_path"),
        "format": data.meta.get("format"),
        "num_input_samples": int(n),
        "num_output_samples": int(rows_out),
        "processing_time_s": float(wall_s),
        "peak_memory_bytes": int(peak_mem),
        "peak_memory_mb": float(peak_mem) / (1024.0 * 1024.0),
        "failure_detection": detector.summary(),
        "failure_detected": bool(detector.failure_started),
        "failure_start_t_ns": detector.failure_start_t_ns,
        "prediction_horizon_s": float(prediction_horizon_s),
        "prediction_interval_s": float(prediction_interval_s or median_dt),
        "prediction_generated": bool(prediction_generated),
        "prediction_appended_samples": int(prediction_count),
        "prediction_debug": prediction_debug,
        "median_dt_s": float(median_dt),
        "validation": validation,
        "preprocessing": pre.stats,
        "input_meta": {k: v for k, v in data.meta.items() if k not in {"input_path", "resolved_path"}},
        "algorithm_configuration": {
            "orientation": {
                "acc_alpha": orient_cfg.acc_alpha,
                "acc_g_tolerance": orient_cfg.acc_g_tolerance,
                "use_acc_correction": orient_cfg.use_acc_correction,
            },
            "trajectory": {
                "gravity_magnitude": float(estimator.config.gravity_magnitude),
                "enable_stationary_zupt": traj_cfg.enable_stationary_zupt,
                "velocity_damping": traj_cfg.velocity_damping,
            },
            "sensor_correction": corrector.get_state(),
            "failure_detection": {
                "complete_failure_consecutive": failure_cfg.complete_failure_consecutive,
                "zero_eps": failure_cfg.zero_eps,
            },
            "dead_reckoning": {
                "use_constant_acceleration": True,
                "accel_decay_tau_s": 1.5,
            },
        },
    }

    if metadata_path is None:
        base, _ = os.path.splitext(output_path)
        metadata_path = base + "_metadata.json" if os.path.basename(base) not in {"estimated_trajectory"} else os.path.join(out_dir, "metadata.json")
        # prefer metadata.json alongside output
        metadata_path = os.path.join(out_dir, "metadata.json")

    with open(metadata_path, "w", encoding="utf-8") as mf:
        json.dump(metadata, mf, indent=2)

    metadata["output_path"] = os.path.abspath(output_path)
    metadata["metadata_path"] = os.path.abspath(metadata_path)
    return metadata


def main(argv: Optional[List[str]] = None):
    ap = argparse.ArgumentParser(description="PathFinder IMU trajectory estimation pipeline")
    ap.add_argument("input", help="IMU CSV file, dataset folder (EuRoC/TUM-VI), or zip archive")
    ap.add_argument("-o", "--output", default="estimated_trajectory.txt", help="Output trajectory file (TUM format)")
    ap.add_argument("--metadata", default=None, help="Output metadata JSON path (default: metadata.json next to output)")
    ap.add_argument("--format", dest="imu_format", default="auto", choices=["auto", "euroc", "tumvi"], help="Input IMU format")
    ap.add_argument("--prediction-horizon", type=float, default=5.0, help="Dead-reckoning horizon in seconds after complete failure")
    ap.add_argument("--prediction-interval", type=float, default=None, help="Prediction sample interval in seconds (default: median input dt)")
    ap.add_argument("--print-summary", action="store_true", help="Print summary to stdout")
    args = ap.parse_args(argv)

    meta = run_pipeline(
        input_path=args.input,
        output_path=args.output,
        metadata_path=args.metadata,
        imu_format=args.imu_format,
        prediction_horizon_s=args.prediction_horizon,
        prediction_interval_s=args.prediction_interval,
    )

    # concise stdout summary
    print(f"input_samples={meta['num_input_samples']} output_samples={meta['num_output_samples']}")
    print(f"failure_detected={meta['failure_detected']} prediction_generated={meta['prediction_generated']} horizon_s={meta['prediction_horizon_s']}")
    print(f"processing_time_s={meta['processing_time_s']:.6f} peak_memory_mb={meta['peak_memory_mb']:.3f}")
    print(f"trajectory: {meta['output_path']}")
    print(f"metadata:   {meta['metadata_path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
