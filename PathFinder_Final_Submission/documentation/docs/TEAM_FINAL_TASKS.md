# TEAM_FINAL_TASKS.md — Final 10% Verification Work (Reserved)

**Project:** PathFinder – IMU Trajectory Estimation Under Sensor Degradation  
**Status:** Core engineering implementation and synthetic/demo validation completed by lead engineer. The tasks below are **reserved for teammates** and are **not claimed as completed**.

**Related preliminary evidence (not a substitute for the reserved work):** real TUM-VI `room4` validation has been added by a collaborator with reproducible scripts, input hashes, consolidated metric summaries in `evidence/real/tumvi_room4/`, and a written report in [`REAL_DATA_VALIDATION.md`](REAL_DATA_VALIDATION.md). Full trajectories and plots are generated locally under gitignored `outputs/real/`. This provides partial evidence for Team Member 3's scope; independent EuRoC runs and teammate review remain required.

Each team member must perform independent verification/experimentation, document observations, and provide concrete evidence (commands run, logs/plots/CSVs, dataset identifiers, and a short technical note on findings and discrepancies).

---

## Team Member 1 — IMU preprocessing and timestamp handling (EuRoC)

### Scope
Independently verify IMU ingestion, preprocessing, and timestamp handling on **actual EuRoC recordings** (e.g., MH/V1/V2 sequences from the EuRoC MAV dataset), not only synthetic data.

### Required work (actual verification/experimentation)
- Run the pipeline on at least **2 real EuRoC sequences** (include one longer/dynamic sequence if available).
- Validate correct parsing of EuRoC IMU format: `timestamp [ns], gyro_x,y,z [rad/s], accel_x,y,z [m/s^2]` (no accel/gyro swap).
- Inspect timestamp handling: monotonicity, duplicates, non-positive intervals, sampling interval estimation, and behavior under irregular/missing samples (including degraded variants).
- Exercise preprocessing controls/effects: invalid-value handling, spike/outlier handling, initial bias/gravity estimation behavior when the start is not perfectly stationary, and configurable parameters.
- Generate degraded EuRoC inputs using the project workflow and confirm conversion adapters preserve column order and timestamps.

### Evidence to provide
- Exact commands used, dataset names/paths (local), and environment details (OS, Python, package versions).
- Produced artifacts: `estimated_trajectory.txt`, `metadata.json`, and any logs/screenshots showing timestamp/preprocessing stats.
- A short written review (>= 1 page or equivalent markdown section) describing: correctness findings, any preprocessing/timestamp issues observed on real data, and recommended parameter/config adjustments.

---

## Team Member 2 — Quaternion orientation, gravity compensation, and coordinate frames

### Scope
Independently review and experimentally validate orientation estimation, quaternion math, gravity alignment/compensation, and coordinate-frame consistency.

### Required work (actual verification/experimentation)
- Review implementation in `src/orientation.py` and `src/trajectory.py` (quaternion convention, normalization, multiplication/integration, body→world rotation, gravity model/removal).
- Run targeted experiments/tests and add/execute reproducible checks (e.g., stationary gravity-only cases, known-rotation gyro integration cases, axis/sign consistency) using real and/or controlled synthetic sequences.
- Verify numerical stability (quaternion normalization over long runs) and assess accelerometer-based tilt/gravity correction behavior under dynamics and degradation.
- Confirm coordinate-frame assumptions are consistent from input through output (TUM pose `tx,ty,tz,qx,qy,qz,qw`), and document yaw observability/initial alignment limitations.

### Evidence to provide
- Commands/tests executed and results (including any additional scripts/notebooks created for validation).
- Plots or numeric summaries demonstrating orientation/gravity behavior (e.g., gravity residuals in stationary segments, attitude sanity checks, drift observations).
- A technical review note documenting: confirmed-correct aspects, identified risks/bugs (if any), sign/frame conventions verified, and suggested fixes or tuning.

---

## Team Member 3 — Trajectory quality, ATE/RPE, dropout resilience, and 5-second prediction

### Scope
Independently evaluate end-to-end trajectory quality and robustness, including dropout/corruption resilience and post-failure 5-second dead-reckoning prediction, using **actual EuRoC/TUM-VI data where available** and the provided tier workflows.

### Required work (actual verification/experimentation)
- Execute Tier 1/2/3 evaluation workflows (`scripts/evaluate_tier1.py`, `scripts/evaluate_tier2.py`, `scripts/evaluate_tier3.py`) on appropriate datasets (real datasets preferred; synthetic only if real data unavailable, clearly stated).
- Compute and interpret ATE/RPE, position RMSE, and orientation error; compare pre-failure vs blind/post-failure windows and analyze failure-detection/prediction behavior.
- Perform dropout-resilience experiments (vary drop/burst rates or use corrupted inputs) and report sensitivity/failure modes.
- Measure runtime and peak memory for representative runs and compare against challenge targets (<= 8 MB peak RAM, <= 20 ms per 1 s IMU, single-thread, offline), reporting **actual measurements**.

### Evidence to provide
- Commands run, dataset identifiers, and generated outputs: `estimated_trajectory.txt`, `metadata.json`, tier summary JSON/CSV, and plots/visualizations.
- A results table/report with ATE/RPE/orientation/runtime/memory metrics (clearly labeled by dataset and tier) and an analysis of 5-second prediction quality and limitations.
- A short review documenting: observed strengths/weaknesses, reproducibility notes, and concrete recommendations for improvement.

---

## Completion criteria for reserved work
- Each member’s evidence must be independently reproducible from repository scripts/commands.
- No fabricated metrics or claims of real-dataset validation without provided artifacts.
- Findings should be consolidated into project notes/issues or an appendix update after review.
