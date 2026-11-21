# PROJECT_AUDIT.md — PathFinder IMU Trajectory

**Date:** 2026-11-20  
**Auditor:** AI/ML engineering agent (automated repository audit)  
**Repository:** `PathFinder-IMU-Trajectory` (cloned from https://github.com/ridhima936/PathFinder-IMU-Trajectory)

## 1. Audit scope and inputs

Inspected all repository files (excluding `.git` internals): `.gitignore`, `requirements.txt`, and all modules under `src/` (`baseline.py`, `degrade_imu.py`, `main.py`, `make_degraded_dataset.py`, `make_synthetic_imu.py`, `quick_score.py`). Also compared the supplied `degrade_imu.py` (provided as attachment) with `src/degrade_imu.py` — **contents are identical** (same logic, `SEED=42`, same degradation model and CLI).

Note: attached challenge PDFs (`XR_Hackathon_2026_Consolidated_Slides_V3-2.pdf`, `Submission_Final_forGithub.pdf`) could not be parsed in this environment; audit/implementation decisions are based on the problem statement provided in the task and on the repository code.

## 2. Existing implementation

| Component | Status | Notes |
|---|---|---|
| `src/baseline.py` | Implemented (baseline) | Streaming pure-Python dead-reckoning estimator. Reads EuRoC IMU CSV/folder/zip, performs initial-window gyro-bias and gravity alignment (assumes initially stationary), quaternion gyro integration (trapezoid), world-frame accel with gravity removal, trapezoid velocity/position integration, handles NaN/invalid and exact-zero rows as "dead" by freezing orientation and constant-velocity propagation. Writes TUM pose per input row. No explicit 5s future-frame extrapolation beyond available rows, limited robustness to spikes/stuck/bias drift/scale, and no modular separation. |
| `src/main.py` | Minimal wrapper | Delegates entirely to `baseline.main()`. No configurable pipeline, no modular stages, no metadata/failure-detection outputs as required. |
| `src/degrade_imu.py` | Implemented (organizer-style utility) | Generates progressively degraded IMU copy with time-ramped severity: white noise, accel/gyro bias random walk, scale random walk, spikes, stuck samples, single/burst dropouts, duplicates, and terminal failure (zeros for last `failure_seconds`). Input/CLI expects columns `timestamp, accel_x,y,z, gyro_x,y,z`; auto-detects/overrides units; fixed `SEED=42`; optional JSON stats via `--meta`. Preserves header/column order of input. |
| `src/make_degraded_dataset.py` | Implemented adapter workflow | Converts EuRoC (`timestamp, gyro, accel`) to degrade format (`timestamp, accel, gyro`), runs `degrade(...)` with forced `ns/ms2/rad/s`, converts back to EuRoC order, writes `imu0/data.csv` and `degradation_stats.json`. Correct column handling; uses fixed `SEED`. |
| `src/make_synthetic_imu.py` | Implemented | Generates synthetic circular/vertical motion EuRoC IMU and TUM ground truth with optional rest/ramp, useful for controlled tests/demos without external datasets. |
| `src/quick_score.py` | Implemented (numpy) | Timestamp association, Umeyama rigid alignment (R,t no scale), ATE RMSE overall/pre-fail/blind-window (last N seconds) and orientation error. Lightweight evaluation utility (not the full requested framework). |
| `requirements.txt` | Partial | Lists `numpy`, `pandas`, `scipy`, `evo`. No pinning; no test tooling declared. |
| `.gitignore` | Minimal | Ignores `data/`, `__pycache__/`, `*.pyc`, `.venv/`. |

## 3. Missing components (relative to hackathon requirements)

**Required modules not present:** `src/imu_reader.py`, `src/preprocessing.py`, `src/sensor_correction.py`, `src/orientation.py`, `src/trajectory.py`, `src/dead_reckoning.py`, `src/failure_detection.py`, `src/evaluation.py`.

**Integration and I/O gaps:**
- No unified modular pipeline (`data validation → preprocessing → correction → orientation → gravity compensation → velocity/position → failure detection → 5s dead reckoning → output`).
- No dedicated failure detection distinguishing temporary dropout vs severe degradation vs complete failure (baseline uses simple row-level heuristics).
- No explicit 5-second future trajectory generation (timestamped future positions/orientations at configurable intervals) after complete failure.
- No TUM-VI adapter in reader layer.
- No `estimated_trajectory.txt`/`metadata.json` generation contract with required fields in a single reproducible run (current baseline writes `estimated_trajectory.txt`-like TUM poses but no metadata, limited CLI).
- No three-tier evaluation workflow (Tier 1 progressive, Tier 2 blind 5s post-failure prediction, Tier 3 dropout/corruption resilience).

**Evaluation/benchmarking gaps:** no ATE/RPE implementation as a module with CSV summaries/plots/trajectory visualizations; no runtime-per-1s-IMU and peak-memory measurement/reporting; no post-failure prediction evaluation workflow; no `evo`-compatible export/comparison integration in `evaluation.py`.

**Testing/documentation gaps:** no `tests/` suite, no `README.md`, `ARCHITECTURE.md`, `ALGORITHM_EXPLANATION.md`, `EXPERIMENTS.md`, `TEAM_FINAL_TASKS.md`, no reproducible demo workflow, no `scripts/` or organized `outputs/`/`docs/` content.

## 4. Bugs, risks, and technical concerns

- **Column-order risk (critical):** EuRoC is `ts, gyro, accel` while `degrade_imu.py` expects `ts, accel, gyro`. `make_degraded_dataset.py` handles this correctly, but any new pipeline/CLI must never pass raw EuRoC columns directly to the degrader without explicit conversion.
- **Stationarity assumption:** `baseline.py` assumes initial 0.5 s stationary for bias/gravity; warns but proceeds. Required preprocessing must not assume perfect initial stillness and must make behavior configurable/robust.
- **Drift and dynamics:** pure double integration without strong motion constraints will drift rapidly under bias/scale/noise; dead rows only freeze attitude and hold velocity (no modeled deceleration/turning), which is a simplistic post-failure model.
- **Failure semantics:** baseline classifies NaN or all-zero rows as dead but does not separate temporary dropouts, stuck/invalid-but-nonzero corruption, or sustained inactivity with explicit state transitions.
- **Resource constraints:** baseline is streaming/pure-Python (RAM-friendly) but slow for numeric work at scale; numpy/pandas/scipy modular code may increase memory/overhead. The 8 MB peak RAM / 20 ms per 1 s IMU (single thread, offline) targets must be **measured**, not assumed.
- **Dependency footprint:** `evo` is listed but may be unavailable/heavy; evaluation must degrade gracefully and report actual measurements only.
- **Data availability:** real EuRoC/TUM-VI datasets are not bundled in-repo; validation on them cannot be claimed without files present.
- **`main.py` contract:** currently prints stats and accepts limited args; replacing it risks breaking existing usage unless backward-compatible behavior/flags are preserved.

## 5. Proposed improvements

1. **Preserve and build around existing assets:** keep `baseline.py`, `quick_score.py`, `degrade_imu.py`, `make_*` scripts functional; refactor `main.py` into a configurable entry point that can run the new modular pipeline (and optionally legacy baseline) without source edits.
2. **Implement modular core:** add `imu_reader` (EuRoC + TUM-VI adapter, validation), `preprocessing` (timestamp normalization, dt/gap handling, outlier/missing handling, optional initial bias/gravity estimation), `sensor_correction` (bias/noise/spike/stuck/scale handling with documented limits), `orientation` (quaternion ops, gyro integration, accel gravity correction/complementary filter), `trajectory` (body→world, gravity removal, v/p integration, drift-mitigation constraints), `failure_detection` (temporary/severe/complete states), `dead_reckoning` (history-based 5s prediction with safe fallback).
3. **Integrate degradation reproducibly:** retain `degrade_imu.py`; add an explicit EuRoC/TUM-VI-safe preparation workflow and always persist degradation stats and seed/config.
4. **Evaluation and tiers:** implement `evaluation.py` (ATE, RPE, position RMSE, orientation error, runtime/memory, dropout resilience, post-failure metrics) and scripts/workflows for Tier 1/2/3 with CSV/plots/visualizations; export evo-compatible trajectories when possible.
5. **Testing and benchmarking:** create comprehensive `tests/` with synthetic and pipeline-level cases; measure actual runtime and peak memory and record results honestly.
6. **Documentation and demo:** produce README/architecture/algorithm/experiments docs, reproducible demo, metadata/trajectory outputs, and `TEAM_FINAL_TASKS.md` reserving final verification for teammates.

## 6. Implementation priorities

1. **P0 — Correctness & required pipeline:** `imu_reader`, `preprocessing`, `sensor_correction`, `orientation`, `trajectory`, `failure_detection`, `dead_reckoning`, and integrated `main.py` producing required trajectory (+ 5s prediction) and `metadata.json`.
2. **P1 — Data & failure handling:** robust validation, gap/duplicate/stuck/spike handling, degradation workflow integration, and clear failure state logic.
3. **P2 — Evaluation & tiers:** `evaluation.py`, Tier 1/2/3 workflows, metrics outputs, plots/visualizations, runtime/memory measurement.
4. **P3 — Resource/performance:** profile and optimize toward 8 MB / 20 ms-per-1s targets; document actual measurements and bottlenecks if unmet.
5. **P4 — Tests, demo, docs:** full test suite, reproducible demo, README/ARCHITECTURE/ALGORITHM_EXPLANATION/EXPERIMENTS, and team verification tasks.

## 7. Preservation notes

- No existing source files will be deleted without justification.
- Existing working scripts (`baseline.py`, `quick_score.py`, degradation utilities) will be preserved and, where adapted, kept compatible or documented.
- No datasets/user data will be overwritten; generated artifacts will go to new/explicit output locations.
