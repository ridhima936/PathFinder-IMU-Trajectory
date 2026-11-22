# PathFinder — IMU Trajectory Estimation Under Sensor Degradation

PathFinder is a modular Python pipeline for reconstructing **6-DoF IMU trajectories** (position + orientation) under progressive sensor degradation and **complete IMU failure**, developed for **Samsung R&D Institute India XR Hackathon 2026, Track 3 (Path Finder: Trajectory Estimation Under Sensor Degradation)**.

The estimator uses **IMU-only data** and, after complete failure, predicts **5 seconds** of future trajectory from motion history. **No GPS, camera, external positioning, or future ground-truth information is used in estimation.** Ground truth is used only in evaluation.

## 1. Problem statement (Track 3)
Reconstruct a 6-DoF trajectory from degraded IMU telemetry while the sensor experiences increasing noise, accelerometer/gyroscope bias drift, scale errors, spikes, stuck readings, single/burst dropouts, and complete failure. The system must:
- maintain continuous position and orientation estimates during the mission,
- detect failure conditions, and
- predict the next **5 seconds** of trajectory after complete IMU failure.

## 2. Technical approach
- **Modular strapdown INS pipeline:** IMU reading/validation → preprocessing → sensor correction → quaternion orientation estimation → gravity compensation and trajectory integration → failure detection → dead-reckoning prediction.
- **Robustness to degradation:** timestamp/validity checks, spike/outlier mitigation (while preserving explicit zero-failure samples), stuck/dropout awareness, conservative bias handling, and gated accelerometer tilt correction.
- **Failure-aware prediction:** state-based detection (temporary dropout / severe degradation / complete failure) and damped motion-history extrapolation to generate timestamped future poses for a configurable horizon (default 5 s).
- **Reproducible workflows:** preserved degradation engine with explicit EuRoC/TUM-VI column-order adapters, tiered evaluation scripts, and a demo pipeline producing trajectories, metadata, metrics, and plots.

## 3. System architecture
See [`ARCHITECTURE.md`](ARCHITECTURE.md) for module responsibilities, data flow, formats, and coordinate-frame conventions.

## 4. Algorithms and mathematical foundations
See [`ALGORITHM_EXPLANATION.md`](ALGORITHM_EXPLANATION.md) for sensor models, quaternion propagation, gravity alignment/compensation, integration, drift-mitigation constraints, failure logic, dead-reckoning model, and evaluation metrics (ATE/RPE).

## 5. Repository structure
```
PathFinder-IMU-Trajectory/
├── src/                    # core modules and CLI (main.py), preserved baseline/utilities
```
PathFinder-IMU-Trajectory/
??? src/                    # core modules and CLI (main.py), preserved baseline/utilities
??? scripts/                # degradation prep, tier1/2/3 evaluation, demo, dataset/evo/real-data helpers
??? tests/                  # unittest suite (synthetic/controlled + real-data helpers)
??? data/{sample,processed,external,real}/  # local datasets (gitignored)
??? outputs/                # generated trajectories, metadata, metrics, plots (gitignored)
??? results/                # evaluation/submission-ready summaries and artifacts
??? evidence/real/          # committed machine-readable real-data summaries
??? docs/
??? README.md
??? ARCHITECTURE.md
??? ALGORITHM_EXPLANATION.md
??? EXPERIMENTS.md
??? REAL_DATA_VALIDATION.md
??? PROJECT_AUDIT.md
??? TEAM_FINAL_TASKS.md
??? requirements.txt
??? .gitignore
```
├── docs/
├── README.md
├── ARCHITECTURE.md
├── ALGORITHM_EXPLANATION.md
├── EXPERIMENTS.md
├── REAL_DATA_VALIDATION.md
├── PROJECT_AUDIT.md
├── TEAM_FINAL_TASKS.md
├── requirements.txt
└── .gitignore
```

**Submission bundle (per hackathon guidance):** include at minimum `estimated_trajectory.txt`, `metadata.json`, `src/` (with `main.py` and `src/README.md`), and project `README.md`; adapt to the official evaluator’s exact naming/format requirements.

## 6. Installation
**Requirements:** Python 3.10+ recommended (tested with Python 3.14), Windows/Linux/macOS.

```bash
# from repository root
python -m venv .venv
# activate venv (platform-specific), then:
pip install -r requirements.txt
```

Notes: `evo` is optional for extended trajectory comparison; plotting requires `matplotlib` (used by evaluation/demo scripts when available). On Windows, `py -3` may be used instead of `python`.

## 7. Dataset preparation

### EuRoC MAV
Provide a dataset folder containing `imu0/data.csv` (EuRoC format) or pass the CSV path directly:
- Columns: `timestamp [ns], w_RS_S_x/y/z [rad/s], a_RS_S_x/y/z [m/s^2]` (gyro before accel).

### TUM-VI
Supported via TUM-VI-compatible/EuRoC-style IMU layouts and adapters in `src/imu_reader.py` (e.g., dataset folders containing `mav0/imu0/data.csv` or other common IMU text locations resolved automatically). Use `--format tumvi` or `--format auto`.

### Download helper (public datasets)
```bash
# List known sources and manual instructions
python scripts/download_datasets.py --list

# Attempt download (may be large; network/mirror dependent)
python scripts/download_datasets.py --euroc MH_01_easy --extract
```
If automatic download fails, follow the official manual instructions printed by the script. Store datasets under `data/external/` (gitignored) or outside the repo. **Do not commit datasets.**

To fetch only the IMU and ground-truth CSVs from a TUM-VI EuroC-export archive (without downloading/keeping camera frames into the dataset folder):

```bash
python scripts/fetch_tumvi_subset.py --sequence room4 \
  --tar-cache "$TEMP/dataset-room4_512_16.tar" \
  --out-dir data/real/tumvi_room4
```

TUM-VI stores ground truth as `mav0/mocap0/data.csv` (EuRoC uses `mav0/state_groundtruth_estimate0/data.csv`). Ground truth is used only by evaluation scripts.

### Synthetic data (no external dataset required)
```bash
python src/make_synthetic_imu.py --duration 30 --rate 200 --rest 2 --ramp 2 --out data/synthetic
```

## 8. Degradation simulation (reproducible)
The original degradation engine is preserved at `src/degrade_imu.py` (fixed seed/config semantics as supplied). **Important:** `degrade_imu.py` expects column order `timestamp, accel_x,y,z, gyro_x,y,z`, while EuRoC is `timestamp, gyro_x,y,z, accel_x,y,z`.

Use the explicit adapter workflow (never swaps columns implicitly):
```bash
python scripts/prepare_degraded_dataset.py data/synthetic/mav0 data/processed/degraded_mav0
# outputs: data/processed/degraded_mav0/imu0/data.csv and degradation_stats.json
```
(Equivalent legacy utility: `python src/make_degraded_dataset.py <src_mav0> <dst>`.)

## 9. Running the estimator
```bash
python src/main.py <input> -o estimated_trajectory.txt --metadata metadata.json
```
- `<input>`: IMU CSV, dataset folder (EuRoC/TUM-VI), or `.zip` archive.
- Options: `--format {auto,euroc,tumvi}`, `--prediction-horizon 5.0`, `--prediction-interval <seconds>`.

**Outputs**
- `estimated_trajectory.txt` — TUM format: `timestamp tx ty tz qx qy qz qw` (includes 5 s prediction coverage after complete failure when applicable).
- `metadata.json` — input/output counts, processing time, peak memory (tracemalloc), failure detection info, prediction horizon/config, and algorithm configuration.

## 10. Evaluation instructions
Evaluation uses ground truth **only** in evaluation scripts/modules.

### Tier workflows
```bash
# Tier 1: progressive degradation
python scripts/evaluate_tier1.py --input <degraded_dataset> --gt <groundtruth_tum_or_euroc_csv> --out-dir outputs/tier1

# Tier 2: blind 5 s prediction after IMU failure
python scripts/evaluate_tier2.py --input <degraded_dataset_with_failure> --gt <groundtruth> --out-dir outputs/tier2

# Tier 3: dropout/corruption resilience
python scripts/evaluate_tier3.py --input <clean_dataset> --gt <groundtruth> --out-dir outputs/tier3
```
Each tier produces `estimated_trajectory.txt`, `metadata.json`, summary JSON/CSV, and plots when `matplotlib` is available.

### Optional: evo ecosystem
If `evo` is installed, you can compare TUM trajectories with:
```bash
python scripts/evo_evaluate.py <groundtruth.tum> <estimated_trajectory.txt> --align --rpe
# or directly:
evo_ape tum <groundtruth.tum> <estimated_trajectory.txt> -va --align
```

### Reproducible demo
```bash
python scripts/run_demo.py
```
Generates clean + degraded synthetic data, runs estimation (including failure/5 s prediction), and writes metrics/plots under `outputs/demo/`.

## 11. Results (actual measured metrics only)
Measurements below include **synthetic/demo runs** and a **real TUM-VI `room4` validation** (Python 3.14.6, Windows). Full commands, provenance, hashes, metrics and interpretation are in [`EXPERIMENTS.md`](EXPERIMENTS.md) and [`REAL_DATA_VALIDATION.md`](REAL_DATA_VALIDATION.md).

### Synthetic/demo (30 s)
| Case | Samples (in/out) | Time (s) | Peak mem MB (tracemalloc) | Failure detected | 5 s prediction | ATE RMSE (m, aligned) | Blind/post-failure RMSE (m) |
|---|---:|---:|---:|:---:|:---:|---:|---:|
| Clean synthetic (30 s, 200 Hz) | 6001 / 6001 | 10.120 | 3.030 | No | No | 4.679 | 4.558* |
| Degraded + terminal failure | 5914 / 5919 | 8.542 | 2.488 | Yes | Yes (5 appended) | 5.130 | 2.224 |
| Tier 3 dropout resilience (289 dropped / 6001) | 5712 / 5712 | 17.348 | 2.547 | No | No | 4.686 | 4.571* |

### Real TUM-VI `room4` (first 30 s)
| Case | Samples (in/out) | Time (s) | Peak mem MB (tracemalloc) | Failure detected | 5 s prediction | ATE RMSE (m, aligned) | Blind/post-failure RMSE (m) |
|---|---:|---:|---:|:---:|:---:|---:|---:|
| Clean IMU | 5982 / 5982 | 18.86 | 2.65 | No | No | 13.37 | 23.10* |
| Degraded + terminal failure | 5885 / 5890 | 11.34 | 2.45 | Yes | Yes (5 residual appended) | 15.54 | 26.54 |
| Tier 3 dropout resilience | 5751 / 5751 | 10.76 | 2.39 | No | No | 13.68 | 23.39* |

Full-recording results (111 s, re-verified on `data/real/tumvi_room4_run2`) are also recorded in collaborator evidence: ATE RMSE 98.46 m (clean), 97.38 m (degraded) and 115.08 m (dropouts); RPE ?1 0.0248 / 0.0261 / 0.0302 m; pipeline processing 52.82 / 50.19 / 49.71 s; peak `tracemalloc` 8.85?9.43 MB. See [`REAL_DATA_VALIDATION.md`](REAL_DATA_VALIDATION.md).

**Resource targets:** measured throughput is ~280?480 ms per 1 s IMU on demo runs and 446?516 ms per 1 s IMU on real runs (above the 20 ms target). Fresh synthetic degraded demo (this environment): 8.542 s for 30 s IMU, `tracemalloc` peak 2.488 MB, **peak process RSS 41.55 MB** (`windows_psapi`). Real-run peak process RSS was 104.93?106.27 MB (collaborator, separately sampled). `tracemalloc` counts Python allocations only; RSS includes interpreter and native libraries. Compliance with 8 MB peak RAM / 20 ms per 1 s IMU is **not met**.

## 12. Known limitations
- IMU-only integration is drift-prone; absolute position and yaw are not fully observable without aiding sensors.
- Bias/scale errors under severe degradation are only partially compensable; dead-reckoning accuracy degrades over the 5 s horizon and cannot anticipate unobserved maneuvers.
- Current Python modular pipeline does not meet the 20 ms per 1 s IMU throughput target in measured runs (see `EXPERIMENTS.md`).
- Real validation currently covers one TUM-VI `room4` recording; additional EuRoC/TUM-VI sequences and process-level memory compliance remain to be verified.

## 13. Team contribution section
- **Lead (current implementation):** repository audit, modular pipeline implementation/integration, degradation workflow integration, evaluation/tier/demo scripts, automated tests, and documentation with measured synthetic/demo results.
- **Collaborator (real-dataset validation):** TUM-VI subset fetch helper, real-sequence validation workflow, column-order/timestamp audit, and measured clean/degraded/dropout results on `room4`; see [`REAL_DATA_VALIDATION.md`](REAL_DATA_VALIDATION.md).
- **Reserved final verification (10%, not claimed complete):** see [`TEAM_FINAL_TASKS.md`](TEAM_FINAL_TASKS.md) for Team Member 1 (preprocessing/timestamps on real EuRoC), Team Member 2 (orientation/gravity/frames review), and Team Member 3 (ATE/RPE, dropout resilience, 5 s prediction evaluation) with required evidence.

## Additional references
- [`PROJECT_AUDIT.md`](PROJECT_AUDIT.md) — initial audit, gaps, risks, and priorities
- [`EXPERIMENTS.md`](EXPERIMENTS.md) — detailed measured results and commands
- [`REAL_DATA_VALIDATION.md`](REAL_DATA_VALIDATION.md) — real TUM-VI `room4` protocol, provenance and results
- [`scripts/`](scripts/) — reproducible workflows
