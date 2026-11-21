# EXPERIMENTS.md — PathFinder

## 1. Scope and data availability
This document records **actual measurements** obtained in the current development environment using both the repository's synthetic/demo workflows and a real TUM-VI recording. The real-data protocol, provenance, hashes and detailed tables are in [`REAL_DATA_VALIDATION.md`](REAL_DATA_VALIDATION.md); results are summarized in Section 3 below.

For the real run, only IMU and ground-truth CSVs were extracted from the official TUM-VI archive. Ground truth was used exclusively by `evaluation.py`; `run_pipeline()` received IMU data only. No GPS, camera images, external positioning, or future ground truth was used in estimation.

**Environment (measured runs):** Windows, Python 3.14.6, NumPy 2.5.1, Pandas 3.0.3. Single-process, offline execution. Plotting/evo are optional and were not required to produce the metrics below.

## 2. Reproducible demonstration (synthetic)

**Workflow:** `py -3 scripts/run_demo.py`  
**Data generation:** 30 s synthetic IMU at 200 Hz with rest/ramp (`make_synthetic_imu.py`), degraded copy via `scripts/prepare_degraded_dataset.py` (preserved `degrade_imu.py`, `SEED=42`, `failure_seconds=5.0`, explicit EuRoC column conversion).

### 2.1 Degradation statistics (actual)
From `outputs/demo/degraded_data/mav0/degradation_stats.json`:
- `rows_in=6001`, `rows_out=5914`, `duration_s=30.0`
- `dropped_total=96` (single=79, burst gaps=2), `duplicated_rows=9`, `spikes=1`, `stuck_samples=32`
- `failure_frames=1000` (terminal failure window), `severity_start=0.02`, `severity_end=1.0`

### 2.2 Pipeline runtime and memory (actual, `tracemalloc` peak)
| Run | Input samples | Output samples | Processing time (s) | Peak memory (MB, tracemalloc) | Failure detected | 5 s prediction |
|---|---:|---:|---:|---:|:---:|:---:|
| Clean synthetic (`estimated_clean.txt`) | 6001 | 6001 | 14.389 | 3.042 | No | No |
| Degraded + terminal failure (`estimated_degraded.txt`) | 5914 | 5919 | 8.448 | 2.487 | Yes (`t_ns=1403636604783555392`) | Yes (appended 5 samples) |

**Derived throughput (measured):** ~480 ms per 1 s IMU (clean, 30 s sequence) and ~282 ms per 1 s IMU (degraded). These do **not** meet the 20 ms per 1 s IMU target; optimization/profiling is required (see Section 6).

### 2.3 Trajectory accuracy vs synthetic ground truth (actual, aligned)
Ground truth: `outputs/demo/clean_data/mav0/groundtruth_tum.txt`. Metrics from `outputs/demo/demo_summary.json`.

| Case | Matched poses | ATE RMSE (m) | Pos. RMSE overall (m) | Pre-failure RMSE (m) | Post-failure/blind 5 s RMSE (m) | Final pos. err (m) | Orient. mean (deg, overall) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Clean vs GT | 6001 | 4.679 | 4.679 | 4.702 | 4.558 | 5.017 | 120.05 |
| Degraded+failure vs GT | 5914 | 5.130 | 5.130 | 5.536 | 2.224 | 3.067 | 115.12 |

**Notes:** absolute position/yaw are weakly observable in IMU-only integration; ATE is reported with rigid alignment (no scale). The degraded run demonstrates correct terminal-failure detection and continued 5 s prediction coverage; blind-window error depends strongly on pre-failure state quality and motion during failure.

## 3. Real-dataset validation (TUM-VI `room4`)

Official source: `https://cvg.cit.tum.de/tumvi/exported/euroc/512_16/dataset-room4_512_16.tar`. The helper `scripts/fetch_tumvi_subset.py` extracted only `mav0/imu0/data.csv` (2,232,296 bytes) and `mav0/mocap0/data.csv` (1,481,244 bytes). SHA-256 hashes and exact commands are recorded in `REAL_DATA_VALIDATION.md` and in `outputs/real/tumvi_room4/real_validation_summary.json`.

Reproduce:

```bash
python scripts/fetch_tumvi_subset.py --sequence room4 \
  --tar-cache "$TEMP/dataset-room4_512_16.tar" \
  --out-dir data/real/tumvi_room4

python scripts/validate_real_sequence.py \
  --dataset data/real/tumvi_room4 \
  --sequence tumvi_room4 \
  --source-url "https://cvg.cit.tum.de/tumvi/exported/euroc/512_16/dataset-room4_512_16.tar" \
  --out-dir outputs/real/tumvi_room4
```

The degraded recording contains the terminal 5 s failure window; those samples are propagated by the dead-reckoning branch. From detected failure onset (`1520531230.579905567`) to the final emitted pose (`1520531235.580477567`) the horizon is 5.0006 s. Only 5 residual samples were appended past the end of the input because the input's dead samples already covered ~4.975 s.

### 3.1 Full recording (111 s, 200 Hz)

| Case | IMU in / out | Wall (s) | Peak MB | Failure / 5 s prediction | Matched poses | ATE RMSE (m) | RPE δ1 (m) | Orient. mean (°) | Blind-window RMSE (m) |
|---|---:|---:|---:|:---:|---:|---:|---:|---:|---:|
| clean | 22212 / 22212 | 41.22 | 9.43 | no / no | 21737 | 98.46 | 0.0248 | 101.1 | 205.20* |
| degraded | 21607 / 21612 | 38.82 | 8.99 | yes / yes (5 residual appended) | 21145 | 97.38 | 0.0261 | 89.6 | 204.36† |
| tier3 dropouts | 21273 / 21273 | 37.84 | 8.85 | no / no | 20833 | 115.08 | 0.0302 | 144.7 | 234.97* |

\*Heuristic last-5-s window because no terminal failure was detected.  
†998 poses after the detected failure time.

Degradation stats (`SEED=42`): 997 failure frames (5 s), 631 dropped samples, 22 spikes, 242 stuck samples. The column-order audit confirmed 100% of degraded timestamps existed in the clean recording, initial accelerometer norm was 10.46 m/s² in both files, and initial gyroscope norm was 0.36 rad/s, so no accel/gyro swap occurred.

### 3.2 First 30 s (`--duration-s 30`)

| Case | IMU in / out | Wall (s) | Peak MB | Failure / 5 s prediction | Matched poses | ATE RMSE (m) | RPE δ1 (m) | Orient. mean (°) | Blind-window RMSE (m) |
|---|---:|---:|---:|:---:|---:|---:|---:|---:|---:|
| clean | 5982 / 5982 | 18.86 | 2.65 | no / no | 5978 | 13.37 | 0.0093 | 82.2 | 23.10* |
| degraded | 5885 / 5890 | 11.34 | 2.45 | yes / yes (5 residual appended) | 5886 | 15.54 | 0.0116 | 107.0 | 26.54† |
| tier3 dropouts | 5751 / 5751 | 10.76 | 2.39 | no / no | 5747 | 13.68 | 0.0101 | 76.7 | 23.39* |

\*Heuristic last-5-s window; †1008 poses after the detected failure time.

### 3.3 Findings
- Local consistency is better than absolute consistency: RPE δ1 is 0.009–0.012 m on the 30 s window, while ATE grows to 13–16 m over 30 s and ~98–115 m over the full recording. This is expected for pure IMU integration with no aiding sensor.
- Orientation error remains large (77–145°). It is reported as measured, not claimed as solved; it is concrete input to the reserved orientation/frame review in `TEAM_FINAL_TASKS.md`.
- The terminal-failure detector fires on the real degraded sequence; the dead-reckoning branch covers the 5 s window from failure onset, with 5 residual samples appended to complete the horizon. Prediction accuracy is limited by drift accumulated before failure.
- The ~98 m ATE was investigated rather than re-scored: the same rigid, scale-fixed alignment gives a 68.70 m ground-truth path length but a 362.83 m estimated path length before alignment, and the end error is 225.90 m. Local RPE δ1 remains 0.0018–0.0336 m per window, so the ATE is cumulative integration drift, not an association/RPE artifact.

### 3.4 Runtime and memory: Python allocations vs process RSS
`metadata.json` reports `tracemalloc` peak, which covers Python heap allocations inside `run_pipeline()` only. Peak process RSS was sampled every 20 ms with `Get-Process.PeakWorkingSet64` around isolated runs; it includes the interpreter, NumPy/Pandas and native allocations.

| Case | Pipeline processing (s) | Wrapper wall (s) | Peak `tracemalloc` (MB) | Peak process RSS (MB) |
|---|---:|---:|---:|---:|
| clean | 43.71 | 45.46 | 9.43 | 104.97 |
| degraded | 40.87 | 43.21 | 9.13 | 106.18 |
| tier3 dropouts | 40.70 | 42.85 | 9.00 | 104.43 |

Neither memory figure meets the 8 MB target; `tracemalloc` understates total process memory by roughly 11x.

## 4. Tier evaluation workflows (synthetic/demo)

All tiers executed via provided scripts with actual outputs in `outputs/tier1..3`.

### Tier 1 — Progressive degradation (`scripts/evaluate_tier1.py`)
- **Input:** degraded demo dataset; **GT:** synthetic ground truth.
- **Run:** 5914 in / 5919 out, 10.175 s, 2.487 MB peak, failure detected, prediction generated (5 appended).
- **Metrics:** ATE RMSE 5.130 m (overall), pre-failure 5.536 m, post-failure/blind 2.224 m (998 poses), RPE δ1 RMSE 0.00976 m.
- **Artifacts:** `outputs/tier1/tier1_summary.{json,csv}`, `estimated_trajectory.txt`, `metadata.json`, plots.

### Tier 2 — Blind 5 s prediction after IMU failure (`scripts/evaluate_tier2.py`)
- **Input:** same degraded terminal-failure dataset; **GT:** synthetic ground truth.
- **Run:** 5914 in / 5919 out, 10.200 s, 3.027 MB peak, failure detected at `1403636604783555392`, prediction generated (5 appended).
- **Blind-window focus (actual):** 998 poses, ATE/position RMSE 2.224 m, mean 2.177 m, median 2.093 m, max 3.097 m, final 3.067 m, orientation mean 155.31°.
- **Artifacts:** `outputs/tier2/tier2_summary.{json,csv}`, plots (`trajectory_3d.png`, `position_error.png`).

### Tier 3 — Dropout/corruption resilience (`scripts/evaluate_tier3.py`)
- **Input:** clean synthetic dataset with injected dropouts (`drop_prob=0.03`, `burst_prob=0.001`, `seed=123`); **GT:** synthetic ground truth.
- **Dropout stats:** `n_in=6001`, `kept=5712`, `dropped=289`.
- **Run:** 5712 in / 5712 out, 17.348 s, 2.547 MB peak, no terminal failure detected.
- **Metrics:** ATE RMSE 4.686 m (overall), pre-failure 4.708 m, post-failure/blind (heuristic window) 4.571 m, RPE δ1 RMSE 0.01195 m.
- **Artifacts:** `outputs/tier3/tier3_summary.{json,csv}`, generated dropout IMU, plots.

## 5. Automated tests (actual execution)
Command: `py -3 -m unittest discover -s tests -v`  
**Result:** 17 tests, **OK** (0 failures, 0 errors). Coverage includes IMU loading/timestamps/malformed values/TUM-VI layout handling, quaternion normalization/integration, gravity alignment/compensation, trajectory integration, preprocessing bias/gravity estimation and spike/gap handling, failure detection states, 5 s dead-reckoning prediction, ATE evaluation sanity, end-to-end pipeline output/metadata/failure+prediction behavior, plus the new tar-subset extractor, recording cropper, degradation/dropout preparation and column-order audit.

## 6. Resource-constraint assessment (actual vs targets)

| Constraint (challenge target) | Measured/observed | Status |
|---|---|---|
| Peak RAM <= 8 MB | ~2.5–3.0 MB Python allocations on demo/tier runs; 9.00–9.43 MB `tracemalloc` and 104.43–106.18 MB process RSS on real TUM-VI runs | **Not met**: `tracemalloc` exceeds 8 MB on the long real run and total process RSS is ~11x higher |
| <= 20 ms per 1 s IMU, single CPU thread | ~280–480 ms per 1 s IMU in demo runs; ~340–630 ms per 1 s IMU on real TUM-VI runs | **Not met** in current implementation/environment |
| Offline execution | Yes (no network/external APIs during estimation) | Met by design in measured runs; the dataset fetch is a separate preparation step |

Bottlenecks are primarily Python-level per-sample overhead in the modular pipeline; the preserved `baseline.py` demonstrates a streaming/pure-Python approach that may be more memory-frugal in some cases but would require further engineering/measurement to meet targets.

## 7. Known limitations observed
- IMU-only drift and yaw unobservability lead to large absolute errors without aiding; results depend on alignment and motion profile.
- Throughput is far from the 20 ms/s target in this environment.
- Demo/tier accuracy numbers are synthetic; real validation currently covers one TUM-VI `room4` sequence, not the full set of EuRoC/TUM-VI sequences.
- Orientation error metrics are high in absolute terms on both synthetic and real data (sensitive to alignment/frame conventions and drift); teammate review (Team Member 2) is required.

## 8. Remaining verification
- Run and record Tier 1/2/3 on at least two real EuRoC sequences and additional TUM-VI sequences.
- Capture process-level memory/runtime benchmarks on target hardware/settings.
- Re-tune preprocessing/correction/orientation gains for real sensor noise characteristics.
