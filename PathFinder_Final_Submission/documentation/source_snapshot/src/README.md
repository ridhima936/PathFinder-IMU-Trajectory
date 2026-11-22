# src/ — PathFinder modules

This directory contains the core PathFinder implementation.

## Main entry point
- `main.py` — integrated CLI pipeline. Runs validation, preprocessing, sensor correction, orientation estimation, gravity compensation, velocity/position integration, failure detection, and 5-second dead reckoning. Produces `estimated_trajectory.txt` (TUM format) and `metadata.json`.

## Core modules
- `imu_reader.py` — EuRoC/TUM-VI IMU loading, validation, and format adapters.
- `preprocessing.py` — timestamp normalization, dt handling, missing/invalid/outlier handling, initial bias/gravity estimation.
- `sensor_correction.py` — gyro/accel bias correction, conservative online adaptation, filtering/scaling hooks.
- `orientation.py` — quaternion math and orientation estimator (gyro integration + gated accelerometer correction).
- `trajectory.py` — body→world transform, gravity compensation, velocity/position integration, conservative drift mitigation.
- `dead_reckoning.py` — post-failure motion-history prediction for configurable horizon (default 5 s).
- `failure_detection.py` — temporary dropout / severe degradation / complete failure state detection.
- `evaluation.py` — ATE/RPE/RMSE/orientation/runtime/memory evaluation (ground truth used only here).

## Preserved utilities
- `baseline.py` — original streaming baseline estimator.
- `degrade_imu.py` — preserved degradation engine (supplied behavior; fixed seed).
- `make_degraded_dataset.py` — EuRoC↔degrader column-order adapter.
- `make_synthetic_imu.py` — synthetic IMU + ground-truth generator.
- `quick_score.py` — lightweight ATE/orientation scorer.

## Example
```bash
python src/main.py <input_csv_or_dataset_folder_or_zip> -o estimated_trajectory.txt --metadata metadata.json
```