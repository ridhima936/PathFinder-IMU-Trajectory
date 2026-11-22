"""Evaluation utilities for IMU trajectory estimation.

Ground truth may be used ONLY within this module/workflows that explicitly perform evaluation.
"""

from __future__ import annotations

import csv
import json
import math
import os
import time
import tracemalloc
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Trajectory:
    t: np.ndarray  # seconds (float)
    p: np.ndarray  # (N,3)
    q: np.ndarray  # (N,4) as x,y,z,w (TUM order)


def load_tum(path: str) -> Trajectory:
    arr = np.loadtxt(path, comments="#")
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.shape[1] != 8:
        raise ValueError(f"TUM trajectory must have 8 columns, got {arr.shape[1]} in {path}")
    return Trajectory(t=arr[:, 0].astype(np.float64), p=arr[:, 1:4].astype(np.float64), q=arr[:, 4:8].astype(np.float64))


def load_euroc_groundtruth_csv(path: str) -> Trajectory:
    arr = np.loadtxt(path, delimiter=",", comments="#")
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.shape[1] < 8:
        raise ValueError(f"EuRoC groundtruth CSV must have >=8 columns, got {arr.shape[1]}")
    t = arr[:, 0].astype(np.float64) * 1e-9
    p = arr[:, 1:4].astype(np.float64)
    # EuRoC state groundtruth is typically: timestamp, p, q(w,x,y,z), ...
    q_wxyz = arr[:, 4:8].astype(np.float64)
    q_xyzw = q_wxyz[:, [1, 2, 3, 0]]
    return Trajectory(t=t, p=p, q=q_xyzw)


def load_groundtruth(path: str) -> Trajectory:
    if path.lower().endswith(".csv"):
        return load_euroc_groundtruth_csv(path)
    return load_tum(path)


def normalize_quat_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def quat_xyzw_to_rot(q: np.ndarray) -> np.ndarray:
    q = normalize_quat_xyzw(q)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3), dtype=np.float64)
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def associate(t_est: np.ndarray, t_gt: np.ndarray, max_diff: float = 0.0025) -> Tuple[np.ndarray, np.ndarray]:
    order = np.argsort(t_gt)
    tg = t_gt[order]
    j = np.clip(np.searchsorted(tg, t_est), 1, len(tg) - 1)
    left, right = tg[j - 1], tg[j]
    pick = np.where(np.abs(t_est - left) <= np.abs(t_est - right), j - 1, j)
    ok = np.abs(tg[pick] - t_est) <= max_diff
    return np.flatnonzero(ok), order[pick[ok]]


def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool = False) -> Tuple[np.ndarray, np.ndarray, float]:
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    X = src - mu_s
    Y = dst - mu_d
    cov = (Y.T @ X) / src.shape[0]
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    if with_scale:
        var_s = (X ** 2).sum() / src.shape[0]
        scale = float(np.trace(np.diag(D) @ S) / var_s) if var_s > 1e-12 else 1.0
    else:
        scale = 1.0
    t = mu_d - scale * (R @ mu_s)
    return R, t, scale


def apply_alignment(p: np.ndarray, R: np.ndarray, t: np.ndarray, scale: float = 1.0) -> np.ndarray:
    return (scale * (R @ p.T)).T + t


def rotation_error_deg(R_est: np.ndarray, R_gt: np.ndarray) -> np.ndarray:
    dR = np.einsum("nji,njk->nik", R_gt, R_est)
    trace = np.trace(dR, axis1=1, axis2=2)
    cosang = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(cosang))


def compute_ate(est_aligned: np.ndarray, gt: np.ndarray) -> Dict[str, float]:
    err = np.linalg.norm(est_aligned - gt, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(err ** 2))) if len(err) else float("nan"),
        "mean": float(np.mean(err)) if len(err) else float("nan"),
        "median": float(np.median(err)) if len(err) else float("nan"),
        "std": float(np.std(err)) if len(err) else float("nan"),
        "max": float(np.max(err)) if len(err) else float("nan"),
        "final": float(err[-1]) if len(err) else float("nan"),
    }


def compute_rpe(est_p: np.ndarray, gt_p: np.ndarray, delta: int = 1) -> Dict[str, float]:
    if len(est_p) <= delta:
        return {"rmse": float("nan"), "mean": float("nan"), "median": float("nan"), "std": float("nan"), "max": float("nan"), "count": 0}
    est_rel = est_p[delta:] - est_p[:-delta]
    gt_rel = gt_p[delta:] - gt_p[:-delta]
    err = np.linalg.norm(est_rel - gt_rel, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(err ** 2))),
        "mean": float(np.mean(err)),
        "median": float(np.median(err)),
        "std": float(np.std(err)),
        "max": float(np.max(err)),
        "count": int(len(err)),
    }


def evaluate_trajectory(
    gt_path: str,
    est_path: str,
    align: bool = True,
    max_assoc_diff_s: float = 0.0025,
    failure_time_s: Optional[float] = None,
    blind_horizon_s: float = 5.0,
) -> Dict[str, object]:
    gt = load_groundtruth(gt_path)
    est = load_tum(est_path)

    ie, ig = associate(est.t, gt.t, max_diff=max_assoc_diff_s)
    if len(ie) < 5:
        raise ValueError(f"Too few associated poses ({len(ie)}). Check timestamps/units.")

    t = est.t[ie]
    pe = est.p[ie]
    pg = gt.p[ig]
    qe = est.q[ie]
    qg = gt.q[ig]

    if align:
        R_al, t_al, s_al = umeyama_alignment(pe, pg, with_scale=False)
        pe_eval = apply_alignment(pe, R_al, t_al, s_al)
        Re = quat_xyzw_to_rot(qe)
        Re = np.einsum("ij,njk->nik", R_al, Re)
    else:
        R_al = np.eye(3)
        t_al = np.zeros(3)
        s_al = 1.0
        pe_eval = pe.copy()
        Re = quat_xyzw_to_rot(qe)
    Rg = quat_xyzw_to_rot(qg)

    pos_err = np.linalg.norm(pe_eval - pg, axis=1)
    rot_err = rotation_error_deg(Re, Rg)

    # split masks
    if failure_time_s is None:
        # heuristic: last blind_horizon_s seconds of estimated sequence
        failure_time_s = float(est.t[-1] - blind_horizon_s)
    pre_mask = t <= failure_time_s
    post_mask = t > failure_time_s
    blind_mask = t > (float(est.t[-1]) - blind_horizon_s)

    def stats(mask: np.ndarray) -> Dict[str, float]:
        if not np.any(mask):
            return {"count": 0}
        epos = pos_err[mask]
        erot = rot_err[mask]
        return {
            "count": int(np.count_nonzero(mask)),
            "ate_rmse_m": float(np.sqrt(np.mean(epos ** 2))),
            "position_rmse_m": float(np.sqrt(np.mean(epos ** 2))),
            "position_mean_m": float(np.mean(epos)),
            "position_median_m": float(np.median(epos)),
            "position_max_m": float(np.max(epos)),
            "position_final_m": float(epos[-1]),
            "orientation_mean_deg": float(np.mean(erot)),
            "orientation_median_deg": float(np.median(erot)),
            "orientation_rmse_deg": float(np.sqrt(np.mean(erot ** 2))),
            "orientation_max_deg": float(np.max(erot)),
        }

    rpe = compute_rpe(pe_eval, pg, delta=1)
    rpe10 = compute_rpe(pe_eval, pg, delta=min(10, max(1, len(pe_eval) - 1)))

    return {
        "matched_poses": int(len(ie)),
        "estimated_rows": int(len(est.t)),
        "groundtruth_rows": int(len(gt.t)),
        "alignment": {
            "enabled": bool(align),
            "R": R_al.tolist(),
            "t": t_al.tolist(),
            "scale": float(s_al),
        },
        "overall": stats(np.ones_like(t, dtype=bool)),
        "pre_failure": stats(pre_mask),
        "post_failure": stats(post_mask),
        "blind_window": stats(blind_mask),
        "ate": compute_ate(pe_eval, pg),
        "rpe_delta1": rpe,
        "rpe_delta10": rpe10,
        "failure_time_s_used": float(failure_time_s),
        "blind_horizon_s": float(blind_horizon_s),
    }


def save_metrics_json(metrics: Dict[str, object], path: str):
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)


def save_metrics_csv(metrics: Dict[str, object], path: str):
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    rows = []
    for section in ["overall", "pre_failure", "post_failure", "blind_window"]:
        data = metrics.get(section, {})
        if isinstance(data, dict):
            row = {"section": section}
            for k, v in data.items():
                if isinstance(v, (int, float, str)) or v is None:
                    row[k] = v
            rows.append(row)
    if not rows:
        return
    keys = sorted({k for r in rows for k in r.keys()}, key=lambda x: (x != "section", x))
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def try_plot_trajectory(gt_path: str, est_path: str, out_path: str, title: str = "Trajectory comparison") -> Optional[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None
    gt = load_groundtruth(gt_path)
    est = load_tum(est_path)
    ie, ig = associate(est.t, gt.t)
    if len(ie) < 2:
        return None
    pe = est.p[ie]
    pg = gt.p[ig]
    R_al, t_al, s_al = umeyama_alignment(pe, pg, with_scale=False)
    pe = apply_alignment(pe, R_al, t_al, s_al)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(pg[:, 0], pg[:, 1], pg[:, 2], label="ground truth", linewidth=1.5)
    ax.plot(pe[:, 0], pe[:, 1], pe[:, 2], label="estimated (aligned)", linewidth=1.2)
    ax.set_title(title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def try_plot_position_error(metrics: Dict[str, object], gt_path: str, est_path: str, out_path: str) -> Optional[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None
    gt = load_groundtruth(gt_path)
    est = load_tum(est_path)
    ie, ig = associate(est.t, gt.t)
    if len(ie) < 2:
        return None
    t = est.t[ie]
    pe = est.p[ie]
    pg = gt.p[ig]
    R_al, t_al, s_al = umeyama_alignment(pe, pg, with_scale=False)
    pe = apply_alignment(pe, R_al, t_al, s_al)
    err = np.linalg.norm(pe - pg, axis=1)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t - t[0], err)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position error [m]")
    ax.set_title("Position error over time (aligned)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def export_evo_tum_copy(src_tum_path: str, dst_path: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(dst_path)) or ".", exist_ok=True)
    with open(src_tum_path, "r", encoding="utf-8") as fsrc, open(dst_path, "w", encoding="utf-8", newline="\n") as fdst:
        for line in fsrc:
            fdst.write(line)
    return dst_path


def benchmark_pipeline(run_fn, *args, repeats: int = 1, **kwargs) -> Dict[str, object]:
    """Measure runtime and peak memory (tracemalloc, Python allocations) for a pipeline callable."""
    results = []
    for _ in range(max(1, repeats)):
        tracemalloc.start()
        t0 = time.perf_counter()
        ret = run_fn(*args, **kwargs)
        t1 = time.perf_counter()
        cur, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        results.append({"wall_s": t1 - t0, "peak_bytes": peak, "current_bytes": cur})
    wall = [r["wall_s"] for r in results]
    peak = [r["peak_bytes"] for r in results]
    return {
        "repeats": len(results),
        "wall_s_mean": float(np.mean(wall)),
        "wall_s_min": float(np.min(wall)),
        "peak_memory_bytes_mean": float(np.mean(peak)),
        "peak_memory_mb_mean": float(np.mean(peak)) / (1024 * 1024),
        "peak_memory_mb_max": float(np.max(peak)) / (1024 * 1024),
        "last_result": results[-1],
    }


def save_evaluation_summary(summary: Dict[str, object], out_dir: str, basename: str = "evaluation_summary"):
    os.makedirs(out_dir, exist_ok=True)
    json_path = os.path.join(out_dir, f"{basename}.json")
    csv_path = os.path.join(out_dir, f"{basename}.csv")
    save_metrics_json(summary, json_path)
    save_metrics_csv(summary, csv_path)
    return {"json": json_path, "csv": csv_path}
