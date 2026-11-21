# REAL_DATA_VALIDATION.md — PathFinder on real IMU recordings

This file records reproducible validation of the estimator on **real**, not
synthetic, IMU data. Ground truth is used **only** by `evaluation.py` through
the validation workflow; `run_pipeline()` receives IMU data only.

## 1. Dataset provenance

| Item | Value |
|---|---|
| Dataset | TUM Visual-Inertial Benchmark (TUM-VI) |
| Sequence | `room4` (motion-capture room, ground truth for the whole trajectory) |
| Export used | `exported/euroc/512_16`, EuRoC-style IMU + ground-truth CSVs |
| Official URL | `https://cvg.cit.tum.de/tumvi/exported/euroc/512_16/dataset-room4_512_16.tar` |
| License / citation | See `https://vision.in.tum.de/data/datasets/visual-inertial-dataset` (Schubert et al., IROS 2018) |
| Files fetched | `mav0/imu0/data.csv`, `mav0/mocap0/data.csv` (TUM-VI ground truth) |
| Fetch helper | `scripts/fetch_tumvi_subset.py` (streams/caches the archive, extracts only these two CSVs) |

The archive is not stored in the repository. The helper records SHA-256 hashes
of both extracted CSVs in `outputs/real/<sequence>/real_validation_summary.json`
so a run can be tied to the exact input files. Machine-readable copies of the
consolidated summaries are committed as
`evidence/real/tumvi_room4/full_recording.json` and
`evidence/real/tumvi_room4/short_window.json`; file paths inside them reflect the
validation machine. Full trajectories and plots remain in gitignored `outputs/`.

## 2. Preparation (network step)

```bash
python scripts/fetch_tumvi_subset.py --sequence room4 \
  --tar-cache "$TEMP/dataset-room4_512_16.tar" \
  --out-dir data/real/tumvi_room4
```

## 3. Validation workflow (offline estimation)

```bash
python scripts/validate_real_sequence.py \
  --dataset data/real/tumvi_room4 \
  --sequence tumvi_room4 \
  --source-url "https://cvg.cit.tum.de/tumvi/exported/euroc/512_16/dataset-room4_512_16.tar" \
  --out-dir outputs/real/tumvi_room4
```

The workflow runs three cases and writes a consolidated summary:

1. **clean** — original IMU; ATE/RPE against recorded poses.
2. **degraded** — reproducible degradation (`SEED=42`, time-ramped noise,
   bias/scale drift, spikes, stuck samples, dropouts, 5 s terminal failure)
   followed by 5 s blind prediction; post-failure window evaluated separately.
3. **tier3** — injected dropouts on the clean IMU (resilience check).

Outputs per case: `estimated_trajectory.txt`, `metadata.json`, `metrics.json`,
summary JSON/CSV and plots. The consolidated report is
`outputs/real/tumvi_room4/real_validation_summary.json`.

Ground truth is passed only to `evaluate_trajectory()` / plotting helpers in
`evaluation.py`. It is never an input to `run_pipeline()`, the degradation
engine, or the dead-reckoning predictor.

## 4. Results (measured)

Environment: Windows, Python 3.14.6, NumPy 2.5.1, Pandas 3.0.3. Single-process,
offline estimation. Metrics use rigid (no-scale) alignment and 5 ms timestamp
association. Memory is `tracemalloc` peak (Python allocations only).

### 4.1 Full `room4` recording (111 s, 200 Hz)

| Case | IMU in / out | Wall (s) | Peak MB | Failure / 5 s prediction | Matched poses | ATE RMSE (m) | RPE δ1 (m) | Mean orient. err (°) | Blind-window RMSE (m) |
|---|---:|---:|---:|:---:|---:|---:|---:|---:|---:|
| clean | 22212 / 22212 | 39.96 | 9.43 | no / no | 21737 | 98.46 | 0.0248 | 101.1 | 205.20 (heuristic last 5 s) |
| degraded | 21607 / 21612 | 37.87 | 8.99 | yes / yes (5 appended) | 21145 | 97.38 | 0.0261 | 89.6 | 204.36 (998 poses after detected failure) |
| tier3 dropouts | 21273 / 21273 | 37.22 | 8.85 | no / no | 20833 | 115.08 | 0.0302 | 144.7 | 234.97 (heuristic last 5 s) |

Degradation stats (`SEED=42`): 997 failure frames (5 s), 631 dropped samples,
22 spikes, 242 stuck samples. Failure was detected at
`t_ns=1520531230579905567`, about 25 ms after the degradation window's
computed start; the detector reported 999 dead samples. Column-order check:
100% of degraded timestamps exist in the clean recording, initial accelerometer
norm is 10.46 m/s² in both files and initial gyroscope norm is 0.36 rad/s,
confirming the EuRoC `timestamp, gyro(3), accel(3)` order was preserved.

### 4.2 First 30 s of the same recording (`--duration-s 30`)

| Case | IMU in / out | Wall (s) | Peak MB | Failure / 5 s prediction | Matched poses | ATE RMSE (m) | RPE δ1 (m) | Mean orient. err (°) | Blind-window RMSE (m) |
|---|---:|---:|---:|:---:|---:|---:|---:|---:|---:|
| clean | 5982 / 5982 | 18.86 | 2.65 | no / no | 5978 | 13.37 | 0.0093 | 82.2 | 23.10 |
| degraded | 5885 / 5890 | 11.34 | 2.45 | yes / yes (5 appended) | 5886 | 15.54 | 0.0116 | 107.0 | 26.54 (1008 poses after detected failure) |
| tier3 dropouts | 5751 / 5751 | 10.76 | 2.39 | no / no | 5747 | 13.68 | 0.0101 | 76.7 | 23.39 |

Reproduce with the same command as Section 3 plus `--duration-s 30` and an
output directory such as `outputs/real/tumvi_room4_short`.

### 4.3 Interpretation

- Short-horizon local motion consistency is reasonable: RPE δ1 is 0.009–0.012 m
  on real IMU data. Absolute ATE grows quickly (98 m over 111 s, 13–16 m over
  30 s) because pure IMU integration has no absolute position or yaw
  observability and no aiding sensor is permitted.
- Orientation error remains large (77–145°) on real data. This is a concrete
  finding for the reserved orientation/frame review in `TEAM_FINAL_TASKS.md`;
  it is reported as measured, not claimed as solved.
- The terminal-failure detector fires correctly on the real degraded sequence
  and the estimator appends the requested 5 s of poses. Predictive accuracy is
  limited by the drift already accumulated before failure (204 m blind-window
  RMSE on the full recording, 26.5 m on the 30 s window).
- Throughput is ~0.34–0.63 s of wall time per 1 s of IMU, far from the 20 ms
  target. Peak `tracemalloc` allocation exceeds 8 MB on the full recording
  (8.85–9.43 MB), so neither resource target is claimed as met. Process RSS was
  not measured.

Artifacts: `outputs/real/tumvi_room4/**` and
`outputs/real/tumvi_room4_short/**`, including per-case `metadata.json`,
`metrics.json`, trajectory files, plots and the consolidated
`real_validation_summary.json`.

## 5. Scope and limitations

- One real TUM-VI sequence is validated here. Additional EuRoC/Vicon sequences
  and TUM-VI corridor/magistrate sequences remain part of the reserved team
  verification in `TEAM_FINAL_TASKS.md`.
- IMU-only integration has no absolute position or yaw observability; ATE is
  reported after rigid (no-scale) alignment, and absolute errors are expected
  to be large compared with visual-inertial SLAM.
- The timestamp association tolerance is set to 5 ms because TUM-VI ground
  truth is recorded at 120 Hz while the IMU runs at 200 Hz.
- Runtime and memory numbers are from this development environment and are
  not a claim of compliance with the 8 MB / 20 ms per 1 s IMU targets.
