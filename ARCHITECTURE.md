# ARCHITECTURE.md — PathFinder IMU Trajectory Estimation

## 1. Purpose and scope
PathFinder reconstructs a 6-DoF trajectory (position + orientation) from IMU-only telemetry under progressive degradation and complete sensor failure. The system is designed as a modular, offline, single-threaded pipeline with reproducible data preparation and evaluation workflows.

**Strict constraint:** the estimator does not use GPS, camera images, external positioning systems, or future ground-truth information. Ground truth is used only in evaluation modules/scripts.

## 2. High-level pipeline

```
Raw IMU input (CSV / EuRoC folder / TUM-VI layout / zip)
        │
        ▼
Data validation & format adaptation (imu_reader)
        │  - EuRoC/TUM-VI parsing, column/order detection
        │  - timestamp normalization to ns, value/timestamp checks
        ▼
Preprocessing (preprocessing)
        │  - dt computation, invalid handling
        │  - robust spike/outlier mitigation (preserving exact-zero failure rows)
        │  - stuck-sample detection, initial gyro-bias & gravity estimation
        ▼
Sensor correction (sensor_correction)
        │  - gyro/accel bias application, conservative online gyro-bias adaptation
        │  - optional filtering/scaling hooks
        ▼
Orientation estimation (orientation)
        │  - quaternion integration from gyro
        │  - accelerometer-based gravity/tilt correction (complementary, gated)
        ▼
Trajectory estimation (trajectory)
        │  - body→world specific-force rotation
        │  - gravity compensation and linear acceleration
        │  - velocity/position integration (trapezoidal), drift-mitigation constraints
        ▼
Failure detection (failure_detection)
        │  - temporary dropout / severe degradation / complete failure states
        ▼
Dead reckoning after complete failure (dead_reckoning)
        │  - motion-history-based damped prediction
        │  - 5 s future horizon at configurable interval
        ▼
Outputs: estimated_trajectory.txt (TUM), metadata.json
```

## 3. Repository structure

```
PathFinder-IMU-Trajectory/
├── src/
│   ├── baseline.py              # preserved streaming baseline estimator
│   ├── degrade_imu.py           # preserved degradation engine (supplied/original behavior)
│   ├── make_degraded_dataset.py # preserved EuRoC<->degrader adapter utility
│   ├── make_synthetic_imu.py    # synthetic IMU + ground-truth generator
│   ├── quick_score.py           # lightweight ATE/orientation scoring utility
│   ├── imu_reader.py            # IMU loading, validation, EuRoC/TUM-VI adapters
│   ├── preprocessing.py         # timestamp/dt handling, outliers, bias/gravity init
│   ├── sensor_correction.py     # bias/filter/scale correction hooks
│   ├── orientation.py           # quaternion math and orientation estimator
│   ├── trajectory.py            # strapdown integration and gravity compensation
│   ├── dead_reckoning.py        # post-failure prediction (5 s horizon)
│   ├── failure_detection.py     # dropout/degradation/failure state machine
│   ├── evaluation.py            # ATE/RPE/RMSE/orientation/runtime/memory utilities
│   └── main.py                  # integrated CLI pipeline entry point
├── scripts/
│   ├── prepare_degraded_dataset.py  # reproducible EuRoC degradation workflow
│   ├── evaluate_tier1.py            # Tier 1: progressive degradation
│   ├── evaluate_tier2.py            # Tier 2: blind 5 s prediction after failure
│   ├── evaluate_tier3.py            # Tier 3: dropout/corruption resilience
│   └── run_demo.py                  # reproducible end-to-end demonstration
├── tests/                       # unittest suite (synthetic/controlled cases)
├── data/{sample,processed}/     # data locations (datasets not bundled)
├── outputs/                     # generated trajectories, metrics, plots
├── docs/                        # additional documentation artifacts
├── PROJECT_AUDIT.md
├── TEAM_FINAL_TASKS.md
├── README.md
├── ARCHITECTURE.md
├── ALGORITHM_EXPLANATION.md
├── EXPERIMENTS.md
├── requirements.txt
└── .gitignore
```

## 4. Module responsibilities and interfaces

- **`imu_reader.py`**: `read_imu(path, fmt)` resolves files/folders/zips, detects EuRoC/TUM-VI-compatible layouts, normalizes timestamps to `int64 ns`, and returns arrays in canonical sensor order `(timestamp, gyro_xyz, accel_xyz)`. Provides validation/report metadata.
- **`preprocessing.py`**: `preprocess_imu(...)` computes `dt`, handles invalid values and outliers, preserves explicit all-zero failure rows, detects stuck samples, and estimates initial gyro bias and gravity magnitude without assuming perfect stillness.
- **`sensor_correction.py`**: `SensorCorrector.correct(...)` applies initial/conservative online gyro-bias correction and optional filtering/scaling; exposes corrected measurements and estimator state/limitations.
- **`orientation.py`**: quaternion primitives (`normalize/multiply/rotvec`, vector rotation) and `OrientationEstimator` combining gyro integration with gated accelerometer tilt correction for gravity alignment.
- **`trajectory.py`**: `TrajectoryEstimator` performs strapdown propagation (body→world, gravity removal, trapezoidal integration) with configurable, conservative drift-mitigation (stationarity-based damping) and no arbitrary hard zeroing.
- **`failure_detection.py`**: `FailureDetector` tracks dead/invalid/zero/stuck/gap evidence and outputs `ok`, `temporary_dropout`, `severe_degradation`, or `complete_failure` with failure timing.
- **`dead_reckoning.py`**: maintains motion history and predicts future poses for a configurable horizon/interval using a damped motion model with clamping to avoid unstable extrapolation.
- **`main.py`**: orchestrates the full pipeline, produces per-sample TUM poses plus required post-failure prediction coverage, and writes `estimated_trajectory.txt` and `metadata.json`.
- **`evaluation.py`**: ground-truth-only evaluation (timestamp association, Umeyama alignment, ATE/RPE/position/orientation metrics, optional plots/exports, runtime/memory helpers).
- **`scripts/*`**: reproducible degradation preparation, tiered evaluation, and demo workflows.

## 5. Data formats and conventions

- **Canonical internal IMU order:** `timestamp_ns, gyro_x, gyro_y, gyro_z (rad/s), accel_x, accel_y, accel_z (m/s^2)`.
- **EuRoC IMU CSV:** `#timestamp [ns], w_RS_S_x/y/z, a_RS_S_x/y/z` (gyro before accel). TUM-VI EuRoC-style exports are supported via format detection/adapters.
- **Degradation utility (`degrade_imu.py`) column order:** `timestamp, accel_x,y,z, gyro_x,y,z` (accel before gyro). All workflows perform explicit conversion and never pass raw EuRoC columns in the wrong order.
- **Trajectory output (TUM):** `timestamp tx ty tz qx qy qz qw` with normalized quaternions and monotonic timestamps. Default timestamp string is seconds with nanosecond precision.
- **Coordinate frame:** quaternion represents body→world rotation; world frame is gravity-aligned with `+Z` upward (`g_world = [0,0,-g]`). Initial yaw is unobservable from IMU alone.

## 6. Failure handling and prediction strategy

- **Detection:** complete failure is inferred from sustained dead evidence (exact-zero/invalid samples) with state transitions distinguishing temporary dropouts and severe degradation from complete failure; naturally small but valid stationary readings are not classified as failure.
- **During failure tail:** when failure is active, the estimator propagates state using a dead-reckoning model driven by recent pre-failure motion history (with decay/clamping) to produce continuous poses at input timestamps.
- **Horizon guarantee:** after failure, the pipeline ensures at least the configured prediction horizon (default 5 s) is covered, appending future timestamped poses when needed.
- **Fallback:** if motion history is insufficient, prediction degrades safely to conservative constant-velocity/rest behavior to limit instability.

## 7. Resource and dependency considerations

- Single-threaded, offline Python implementation using NumPy/SciPy/Pandas as needed; plotting/evo support is optional and used only in evaluation workflows when available.
- Runtime and peak memory are **measured and reported** (see `EXPERIMENTS.md`); challenge targets (<= 8 MB peak RAM, <= 20 ms per 1 s IMU) are not claimed met unless supported by measurements on the target environment/data.

## 8. Extensibility

The modular boundaries allow swapping/adding readers (dataset adapters), correction/constraint strategies, alternative orientation filters, and evaluation backends without changing the CLI contract or output formats.
