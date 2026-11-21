# EXPERIMENTS.md — PathFinder

## 1. Scope and data availability
This document records **actual measurements** obtained in the current development environment using the repository's synthetic/demo workflows. **Real EuRoC/TUM-VI dataset validation is pending** because those datasets are not bundled in this workspace; Tier 1/2/3 scripts and the main pipeline support them, but results on real recordings must be produced and reported separately by the team (see `TEAM_FINAL_TASKS.md`).

**Environment (measured runs):** Windows, Python 3.14.10, NumPy 2.5.5, SciPy 1.18.1, Pandas 3.1.1, Matplotlib 3.11.2 (for plots). Single-process, offline execution.

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

## 3. Tier evaluation workflows (synthetic/demo)

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

## 4. Automated tests (actual execution)
Command: `py -3 -m unittest discover -s tests -v`  
**Result:** 14 tests, **OK** (0 failures, 0 errors). Coverage includes IMU loading/timestamps/malformed values/TUM-VI layout handling, quaternion normalization/integration, gravity alignment/compensation, trajectory integration, preprocessing bias/gravity estimation and spike/gap handling, failure detection states, 5 s dead-reckoning prediction, ATE evaluation sanity, and end-to-end pipeline output/metadata/failure+prediction behavior.

## 5. Resource-constraint assessment (actual vs targets)

| Constraint (challenge target) | Measured/observed | Status |
|---|---|---|
| Peak RAM <= 8 MB | ~2.5–3.0 MB peak **Python allocations** via `tracemalloc` on demo/tier runs | **Not claimed compliant** (process RSS/total footprint not measured; `tracemalloc` under-reports non-Python/native allocations) |
| <= 20 ms per 1 s IMU, single CPU thread | ~280–480 ms per 1 s IMU in demo runs (Python/NumPy modular pipeline) | **Not met** in current implementation/environment |
| Offline execution | Yes (no network/external APIs) | Met by design in measured runs |

Bottlenecks are primarily Python-level per-sample overhead in the modular pipeline; the preserved `baseline.py` demonstrates a streaming/pure-Python approach that may be more memory-frugal in some cases but would require further engineering/measurement to meet targets.

## 6. Known limitations observed
- IMU-only drift and yaw unobservability lead to large absolute errors without aiding; results depend on alignment and motion profile.
- Throughput is far from the 20 ms/s target in this environment.
- All reported accuracy numbers above are on **synthetic** data; performance on real EuRoC/TUM-VI sequences under the official degraded protocol must be measured independently.
- Orientation error metrics in these synthetic comparisons are high in absolute terms (sensitive to alignment/frame conventions and drift); teammate review (Team Member 2) is required.

## 7. Pending real-dataset validation
- Run and record Tier 1/2/3 on actual EuRoC and TUM-VI recordings.
- Capture process-level memory/runtime benchmarks on target hardware/settings.
- Re-tune preprocessing/correction/orientation gains for real sensor noise characteristics.
