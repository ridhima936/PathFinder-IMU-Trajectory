"""IMU preprocessing and validation utilities."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


@dataclass
class PreprocessConfig:
    # timestamp handling
    enforce_monotonic: bool = True
    max_dt_s: Optional[float] = None  # cap integration dt (seconds)
    # outlier/spike handling (robust z-score via MAD on magnitude deviation)
    spike_z_thresh: float = 8.0
    spike_clip: bool = True
    # stuck handling
    stuck_window: int = 6
    stuck_tol: float = 1e-12
    # initial bias estimation (optional heuristic)
    initial_bias_window_s: float = 1.0
    stationary_gyro_max: float = 0.25  # rad/s norm threshold for candidate stationary samples
    stationary_acc_tolerance: float = 0.8  # m/s^2 tolerance around gravity magnitude
    min_stationary_samples: int = 10
    # gravity
    default_gravity: float = 9.80665


@dataclass
class PreprocessResult:
    timestamps_ns: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray
    dt_s: np.ndarray
    valid_mask: np.ndarray
    gyro_bias_init: np.ndarray
    accel_bias_init: np.ndarray
    gravity_magnitude: float
    stats: Dict[str, object]


def _robust_spike_fix(values: np.ndarray, z_thresh: float) -> Tuple[np.ndarray, int]:
    """Fix spikes per-axis using median/MAD and clip to median +/- z*MAD."""
    out = values.copy()
    fixed = 0
    for ax in range(out.shape[1]):
        col = out[:, ax]
        finite = np.isfinite(col)
        if np.count_nonzero(finite) < 5:
            continue
        med = np.median(col[finite])
        mad = np.median(np.abs(col[finite] - med))
        if mad < 1e-12:
            continue
        z = np.abs(col - med) / (1.4826 * mad)
        mask = finite & (z > z_thresh)
        if np.any(mask):
            limit = z_thresh * 1.4826 * mad
            out[mask, ax] = np.clip(col[mask], med - limit, med + limit)
            fixed += int(np.sum(mask))
    return out, fixed


def compute_dt_s(timestamps_ns: np.ndarray, max_dt_s: Optional[float] = None) -> np.ndarray:
    ts = np.asarray(timestamps_ns, dtype=np.int64)
    n = len(ts)
    dt = np.zeros(n, dtype=np.float64)
    if n < 2:
        return dt
    diffs = np.diff(ts).astype(np.float64) * 1e-9
    pos = diffs[diffs > 0]
    median_dt = float(np.median(pos)) if pos.size else 0.005
    # first sample dt unknown; use median
    dt[0] = median_dt
    dt[1:] = diffs
    # handle non-positive dt
    for i in range(1, n):
        if not np.isfinite(dt[i]) or dt[i] <= 0:
            dt[i] = median_dt
    if max_dt_s is not None:
        dt = np.minimum(dt, float(max_dt_s))
    dt = np.clip(dt, 1e-6, None)
    return dt


def detect_stuck(values: np.ndarray, window: int, tol: float) -> np.ndarray:
    """Return boolean mask where samples are part of a stuck run."""
    n = len(values)
    stuck = np.zeros(n, dtype=bool)
    if n == 0 or window <= 1:
        return stuck
    run = 1
    for i in range(1, n):
        if np.all(np.isfinite(values[i])) and np.all(np.isfinite(values[i - 1])) and np.all(np.abs(values[i] - values[i - 1]) <= tol):
            run += 1
        else:
            if run >= window:
                stuck[i - run:i] = True
            run = 1
    if run >= window:
        stuck[n - run:n] = True
    return stuck


def estimate_initial_bias_and_gravity(
    timestamps_ns: np.ndarray,
    gyro: np.ndarray,
    accel: np.ndarray,
    config: PreprocessConfig,
) -> Tuple[np.ndarray, np.ndarray, float, Dict[str, object]]:
    """Estimate initial gyro bias and gravity magnitude.

    Does not assume perfect stillness: selects low-motion samples within the initial window when available,
    otherwise falls back to robust medians over the initial window.
    """
    n = len(timestamps_ns)
    if n == 0:
        g0 = config.default_gravity
        return np.zeros(3), np.zeros(3), g0, {"method": "empty"}

    t0 = int(timestamps_ns[0])
    window_ns = int(max(0.0, config.initial_bias_window_s) * 1e9)
    idx_end = n
    if window_ns > 0:
        idx_end = int(np.searchsorted(timestamps_ns, t0 + window_ns, side="right"))
        idx_end = max(idx_end, min(n, 5))
    idx_end = min(n, max(idx_end, 1))

    g_win = gyro[:idx_end]
    a_win = accel[:idx_end]
    finite = np.all(np.isfinite(g_win), axis=1) & np.all(np.isfinite(a_win), axis=1)
    g_win = g_win[finite]
    a_win = a_win[finite]
    info: Dict[str, object] = {"window_samples": int(idx_end), "finite_samples": int(len(g_win))}

    if len(g_win) == 0:
        g0 = config.default_gravity
        return np.zeros(3), np.zeros(3), g0, {**info, "method": "default_no_finite"}

    gyro_norm = np.linalg.norm(g_win, axis=1)
    accel_norm = np.linalg.norm(a_win, axis=1)

    # Try to estimate gravity magnitude robustly from accel norms
    med_acc_norm = float(np.median(accel_norm))
    g_mag = med_acc_norm if 5.0 < med_acc_norm < 15.0 else config.default_gravity

    stationary = (gyro_norm <= config.stationary_gyro_max) & (np.abs(accel_norm - g_mag) <= config.stationary_acc_tolerance)
    if np.count_nonzero(stationary) >= config.min_stationary_samples:
        sel_g = g_win[stationary]
        sel_a = a_win[stationary]
        method = "low_motion_subset"
    else:
        # fallback: use lower-motion half of window to reduce motion contamination
        k = max(config.min_stationary_samples, len(g_win) // 3)
        order = np.argsort(gyro_norm + 0.05 * np.abs(accel_norm - g_mag))
        take = order[: min(len(order), max(k, 5))]
        sel_g = g_win[take]
        sel_a = a_win[take]
        method = "robust_low_motion_fallback"

    gyro_bias = np.median(sel_g, axis=0)
    # Accelerometer bias is not reliably observable without known reference; keep zero and document limitation.
    accel_bias = np.zeros(3, dtype=np.float64)
    g_mag_refined = float(np.median(np.linalg.norm(sel_a, axis=1))) if len(sel_a) else g_mag
    if not (5.0 < g_mag_refined < 15.0):
        g_mag_refined = g_mag

    info.update({
        "method": method,
        "selected_samples": int(len(sel_g)),
        "gravity_magnitude": g_mag_refined,
        "gyro_bias_init": gyro_bias.tolist(),
    })
    return gyro_bias.astype(np.float64), accel_bias, g_mag_refined, info


def preprocess_imu(
    timestamps_ns: np.ndarray,
    gyro: np.ndarray,
    accel: np.ndarray,
    config: PreprocessConfig = None,
) -> PreprocessResult:
    cfg = config or PreprocessConfig()
    ts = np.asarray(timestamps_ns, dtype=np.int64).copy()
    g = np.asarray(gyro, dtype=np.float64).copy()
    a = np.asarray(accel, dtype=np.float64).copy()
    if ts.shape[0] != g.shape[0] or ts.shape[0] != a.shape[0]:
        raise ValueError("timestamps, gyro, accel length mismatch")

    stats: Dict[str, object] = {"n_input": int(len(ts))}
    n = len(ts)

    # handle NaN/inf by marking invalid; replace with 0 for downstream stability (will be treated via valid_mask/failure logic)
    finite_mask = np.all(np.isfinite(g), axis=1) & np.all(np.isfinite(a), axis=1)
    stats["invalid_value_samples"] = int(n - np.count_nonzero(finite_mask))
    g[~finite_mask] = 0.0
    a[~finite_mask] = 0.0

    # Preserve explicit all-zero failure rows (used by degradation/failure model) so they are not altered by outlier clipping
    exact_zero_rows = np.all(np.abs(g) == 0.0, axis=1) & np.all(np.abs(a) == 0.0, axis=1)
    stats["exact_zero_rows_preserved"] = int(np.count_nonzero(exact_zero_rows))

    # spike/outlier handling on finite samples (excluding exact-zero failure rows)
    if cfg.spike_clip and n >= 5:
        g_fix, fg = _robust_spike_fix(g, cfg.spike_z_thresh)
        a_fix, fa = _robust_spike_fix(a, cfg.spike_z_thresh)
        if np.any(exact_zero_rows):
            g_fix[exact_zero_rows] = g[exact_zero_rows]
            a_fix[exact_zero_rows] = a[exact_zero_rows]
        g, a = g_fix, a_fix
        stats["gyro_spikes_clipped"] = int(fg)
        stats["accel_spikes_clipped"] = int(fa)
    else:
        stats["gyro_spikes_clipped"] = 0
        stats["accel_spikes_clipped"] = 0

    stuck_g = detect_stuck(g, cfg.stuck_window, cfg.stuck_tol)
    stuck_a = detect_stuck(a, cfg.stuck_window, cfg.stuck_tol)
    stuck = stuck_g | stuck_a
    stats["stuck_samples_detected"] = int(np.count_nonzero(stuck))

    dt_s = compute_dt_s(ts, max_dt_s=cfg.max_dt_s)

    gyro_bias_init, accel_bias_init, g_mag, bias_info = estimate_initial_bias_and_gravity(ts, g, a, cfg)
    stats["bias_estimation"] = bias_info

    # valid mask: finite original values and not exact-all-zero placeholders except keep info (zeros can be real failure handled later)
    valid_mask = finite_mask.copy()
    stats["n_output"] = int(n)

    return PreprocessResult(
        timestamps_ns=ts,
        gyro=g,
        accel=a,
        dt_s=dt_s,
        valid_mask=valid_mask,
        gyro_bias_init=gyro_bias_init,
        accel_bias_init=accel_bias_init,
        gravity_magnitude=g_mag,
        stats=stats,
    )
