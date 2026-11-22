"""IMU data loading, validation, and format adapters.

Supports EuRoC MAV IMU CSV and common TUM-VI IMU text layouts.
All returned measurements use EuRoC sensor ordering semantics:
    timestamp [ns], gyro_x, gyro_y, gyro_z [rad/s], accel_x, accel_y, accel_z [m/s^2]
"""

from __future__ import annotations

import csv
import io
import math
import os
import zipfile
from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class IMUData:
    timestamps_ns: np.ndarray  # int64
    gyro: np.ndarray  # (N,3) rad/s
    accel: np.ndarray  # (N,3) m/s^2
    meta: Dict[str, object] = field(default_factory=dict)

    @property
    def n_samples(self) -> int:
        return int(self.timestamps_ns.shape[0])

    def iter_rows(self) -> Iterator[Tuple[int, float, float, float, float, float, float]]:
        ts = self.timestamps_ns
        g = self.gyro
        a = self.accel
        for i in range(len(ts)):
            yield (
                int(ts[i]),
                float(g[i, 0]), float(g[i, 1]), float(g[i, 2]),
                float(a[i, 0]), float(a[i, 1]), float(a[i, 2]),
            )


def _is_zip(path: str) -> bool:
    return path.lower().endswith(".zip")


def _open_text(path: str):
    if _is_zip(path):
        zf = zipfile.ZipFile(path)
        names = [n for n in zf.namelist() if not n.endswith("/")]
        # Prefer EuRoC/TUM-VI imu0/data.csv, else imu.txt, else first csv/txt
        def score(name: str) -> Tuple[int, int]:
            n = name.replace("\\", "/").lower()
            if n.endswith("imu0/data.csv"):
                return (0, len(name))
            if n.endswith("/imu0/data.csv") or n == "imu0/data.csv":
                return (0, len(name))
            if n.endswith("imu.txt"):
                return (1, len(name))
            if n.endswith(".csv") or n.endswith(".txt"):
                return (2, len(name))
            return (9, len(name))
        names_sorted = sorted(names, key=score)
        if not names_sorted:
            zf.close()
            raise ValueError(f"No files found in zip: {path}")
        chosen = names_sorted[0]
        return io.TextIOWrapper(zf.open(chosen), encoding="utf-8-sig"), zf, chosen
    return open(path, "r", encoding="utf-8-sig", newline=""), None, path


def resolve_imu_path(path: str) -> str:
    """Resolve a dataset path to an IMU data file path or zip path."""
    if os.path.isfile(path):
        return path
    if not os.path.isdir(path):
        raise FileNotFoundError(f"Input path not found: {path}")

    candidates = [
        os.path.join(path, "imu0", "data.csv"),
        os.path.join(path, "mav0", "imu0", "data.csv"),
        os.path.join(path, "mav0", "imu0", "data.txt"),
        os.path.join(path, "imu", "data.csv"),
        os.path.join(path, "dso", "imu.txt"),
        os.path.join(path, "imu.txt"),
        os.path.join(path, "data.csv"),
    ]
    for cand in candidates:
        if os.path.isfile(cand):
            return cand

    # recursive search fallback
    for root, _, files in os.walk(path):
        for name in files:
            low = name.lower()
            rel = os.path.join(root, name).replace("\\", "/").lower()
            if rel.endswith("imu0/data.csv") or rel.endswith("imu0/data.txt") or low == "imu.txt":
                return os.path.join(root, name)
    raise FileNotFoundError(f"Could not locate IMU data under: {path}")


def _parse_float(token: str) -> float:
    t = token.strip()
    if t == "" or t.lower() in {"nan", "+nan", "-nan"}:
        return float("nan")
    if t.lower() in {"inf", "+inf"}:
        return float("inf")
    if t.lower() == "-inf":
        return float("-inf")
    return float(t)


def _parse_timestamp_ns(token: str) -> int:
    t = token.strip()
    # integer nanoseconds (EuRoC)
    if t.lstrip("+-").isdigit():
        return int(t)
    # numeric string (possibly float/scientific, possibly seconds)
    val = float(t)
    if not math.isfinite(val):
        raise ValueError("non-finite timestamp")
    # Heuristic: >=1e14 -> already ns, >=1e11 -> us/ms-ish unlikely for epoch but treat by magnitude
    av = abs(val)
    if av >= 1e14:
        return int(round(val))
    if av >= 1e11:
        # microseconds
        return int(round(val * 1e3))
    if av >= 1e9:
        # could be ns-as-float or seconds in some systems; EuRoC ns are ~1e18
        return int(round(val))
    # likely seconds
    return int(round(val * 1e9))


def _detect_format(path_hint: str, header_line: Optional[str], delimiter: str) -> str:
    hint = path_hint.replace("\\", "/").lower()
    if "tum" in hint:
        return "tumvi"
    if header_line:
        h = header_line.lower()
        if "w_rs_s" in h or "a_rs_s" in h:
            # EuRoC/TUM-VI EuRoC-style export
            return "euroc"
        if "gyro" in h and "acc" in h:
            # generic header; keep order detection later
            return "generic"
    if hint.endswith("imu.txt") and "mav0" not in hint:
        # Many TUM-VI raw logs use space-separated imu.txt; treat as tumvi-compatible generic
        return "tumvi"
    return "euroc"


def _infer_columns(values: Sequence[str], fmt: str) -> Tuple[str, List[int]]:
    """Return (layout, indices) where layout in {'g_a','a_g'} for 6 sensor cols."""
    # Default EuRoC/TUM-VI EuRoC-style: ts, gx,gy,gz, ax,ay,az
    if fmt in ("euroc", "tumvi", "generic"):
        return "g_a", [0, 1, 2, 3, 4, 5, 6]
    return "g_a", [0, 1, 2, 3, 4, 5, 6]


def read_imu(path: str, fmt: str = "auto", strict: bool = False) -> IMUData:
    """Read IMU data from file/folder/zip.

    Args:
        path: CSV/TXT file, dataset folder, or zip archive.
        fmt: 'auto', 'euroc', or 'tumvi'.
        strict: if True, raise on malformed rows; otherwise skip malformed rows but record stats.
    """
    resolved = resolve_imu_path(path)
    f, zf, chosen_name = _open_text(resolved)
    timestamps: List[int] = []
    gyro_rows: List[Tuple[float, float, float]] = []
    accel_rows: List[Tuple[float, float, float]] = []
    stats: Dict[str, object] = {
        "input_path": path,
        "resolved_path": resolved,
        "source_member": chosen_name,
        "skipped_lines": 0,
        "malformed_rows": 0,
        "non_monotonic_timestamps": 0,
        "duplicate_timestamps": 0,
        "nonpositive_dt": 0,
        "invalid_value_rows": 0,
        "total_lines": 0,
    }

    try:
        sample = f.read(4096)
        f.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=", \t;")
            delimiter = dialect.delimiter
        except Exception:
            # fallback: comma if present else whitespace
            first_line = sample.splitlines()[0] if sample.splitlines() else ""
            if "," in first_line:
                delimiter = ","
            elif "\t" in first_line:
                delimiter = "\t"
            elif ";" in first_line:
                delimiter = ";"
            else:
                delimiter = None
        if delimiter is None:
            delimiter = ","

        header_line = None
        first_data_checked = False
        layout = "g_a"
        detected_fmt = fmt if fmt != "auto" else "euroc"

        for raw in f:
            stats["total_lines"] = int(stats["total_lines"]) + 1
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                header_line = line
                continue

            # split
            if delimiter == "," or "," in line:
                parts = [p.strip() for p in line.split(",")]
            else:
                parts = line.split()
            if len(parts) < 7:
                # try alternate split
                parts_alt = [p.strip() for p in line.replace(";", ",").split(",")] if "," not in line else parts
                if len(parts_alt) >= 7:
                    parts = parts_alt
                else:
                    if not first_data_checked:
                        # could be header without '#'
                        maybe_header = any(c.isalpha() for c in line)
                        if maybe_header:
                            header_line = line
                            continue
                    stats["skipped_lines"] = int(stats["skipped_lines"]) + 1
                    if strict:
                        raise ValueError(f"Malformed IMU row (expected >=7 columns): {line[:120]}")
                    continue

            if not first_data_checked:
                detected_fmt = fmt if fmt != "auto" else _detect_format(chosen_name, header_line, delimiter)
                # if header explicitly indicates accel before gyro, respect it
                if header_line:
                    h = header_line.lower().replace(" ", "")
                    if ("accel" in h and "gyro" in h and h.find("accel") < h.find("gyro")):
                        layout = "a_g"
                    else:
                        layout = "g_a"
                first_data_checked = True

            try:
                t_ns = _parse_timestamp_ns(parts[0])
                vals = [_parse_float(parts[i]) for i in range(1, 7)]
            except Exception:
                # header-like row without '#'
                if any(c.isalpha() for c in line):
                    header_line = line
                    continue
                stats["malformed_rows"] = int(stats["malformed_rows"]) + 1
                if strict:
                    raise
                continue

            if layout == "a_g":
                ax, ay, az, gx, gy, gz = vals
            else:
                gx, gy, gz, ax, ay, az = vals

            if not all(math.isfinite(v) for v in vals):
                stats["invalid_value_rows"] = int(stats["invalid_value_rows"]) + 1

            timestamps.append(t_ns)
            gyro_rows.append((gx, gy, gz))
            accel_rows.append((ax, ay, az))

        if not timestamps:
            raise ValueError(f"No valid IMU samples found in: {resolved}")

        ts_arr = np.asarray(timestamps, dtype=np.int64)
        gyro_arr = np.asarray(gyro_rows, dtype=np.float64)
        accel_arr = np.asarray(accel_rows, dtype=np.float64)

        if ts_arr.size > 1:
            dt = np.diff(ts_arr)
            stats["non_monotonic_timestamps"] = int(np.sum(dt < 0))
            stats["duplicate_timestamps"] = int(np.sum(dt == 0))
            stats["nonpositive_dt"] = int(np.sum(dt <= 0))

        stats.update({
            "format": detected_fmt,
            "layout_detected": layout,
            "n_samples": int(ts_arr.size),
            "t_start_ns": int(ts_arr[0]),
            "t_end_ns": int(ts_arr[-1]),
            "duration_s": float((int(ts_arr[-1]) - int(ts_arr[0])) * 1e-9),
        })

        # sampling interval stats (on positive diffs)
        if ts_arr.size > 1:
            pos_dt = dt[dt > 0]
            if pos_dt.size:
                stats["dt_median_s"] = float(np.median(pos_dt) * 1e-9)
                stats["dt_min_s"] = float(np.min(pos_dt) * 1e-9)
                stats["dt_max_s"] = float(np.max(pos_dt) * 1e-9)

        return IMUData(timestamps_ns=ts_arr, gyro=gyro_arr, accel=accel_arr, meta=stats)
    finally:
        f.close()
        if zf is not None:
            zf.close()


def validate_imu_data(data: IMUData) -> Dict[str, object]:
    """Return validation summary; does not modify data."""
    ts = data.timestamps_ns
    report: Dict[str, object] = {"n_samples": data.n_samples, "ok": True, "issues": []}
    issues: List[str] = report["issues"]  # type: ignore
    if data.n_samples == 0:
        issues.append("empty dataset")
    if data.gyro.shape != (data.n_samples, 3) or data.accel.shape != (data.n_samples, 3):
        issues.append("shape mismatch")
    if data.n_samples > 1:
        dt = np.diff(ts)
        if np.any(dt < 0):
            issues.append("non-monotonic timestamps present")
        if np.any(dt == 0):
            issues.append("duplicate timestamps present")
        if np.any(dt <= 0):
            issues.append("non-positive timestamp intervals present")
    if not np.all(np.isfinite(data.gyro)):
        issues.append("non-finite gyro values present")
    if not np.all(np.isfinite(data.accel)):
        issues.append("non-finite accel values present")
    report["ok"] = len(issues) == 0
    return report
