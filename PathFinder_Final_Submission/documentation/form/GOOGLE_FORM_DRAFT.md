# Google Form Draft Answers ? Samsung PRISM XR Hackathon 2026 (Track 03)

**Form:** https://forms.gle/YtxP2aDwZcEtqu8QA  
**Status:** DRAFT ONLY. **Do not submit without explicit user review/approval.**  
**Agent has not submitted this form. Video file was generated locally; manual upload to YouTube/Drive is still required.**

Fill all `[CONFIRM: ...]` / `[MANUAL UPLOAD]` / `[USER-PROVIDED]` fields before submission. Do not invent personal/team identity details.

## A) Team / identity fields (user-provided)
| Field | Draft value |
|---|---|
| Team name | `Error 404 Not Found` [CONFIRM: replace with official registered team name if different] |
| College / Organization | [USER-PROVIDED: COLLEGE_NAME] |
| Theme / Track | `Track 03 ? Path Finder: Trajectory Estimation Under Sensor Degradation` |
| Project title | `PathFinder ? Trajectory Estimation Under Sensor Degradation` |
| Team member names | [USER-PROVIDED] |
| Emails | [USER-PROVIDED] |
| Phone numbers | [USER-PROVIDED] |
| Year / Branch | [USER-PROVIDED] |

## B) Project description fields

### Problem statement
Track 03 requires reconstructing 6-DoF trajectory (position and orientation) from degraded IMU telemetry. The sensor may suffer noise, bias drift, scale errors, spikes, stuck samples, dropouts, and complete failure. After complete IMU failure, the system must predict the next 5 seconds of trajectory. The solution must be standalone Python, offline, single CPU thread, with no GPS/camera/future ground truth in inference, and must produce the required TUM trajectory format.

### Proposed solution / technical approach
PathFinder is a modular offline IMU-only Python pipeline: IMU reader/adapter (EuRoC/TUM-VI), preprocessing and sensor correction, quaternion orientation estimation, gravity compensation, velocity/position integration, failure detection, and 5-second dead-reckoning prediction. It outputs `estimated_trajectory.txt` in TUM format (`# timestamp tx ty tz qx qy qz qw`) with exactly one pose per input IMU frame, plus `metadata.json`. Ground truth is used only for evaluation.

### Innovation / highlights
- Contract-aware 1:1 output-per-input-frame implementation with 5s horizon metadata.
- Failure-signature-preserving preprocessing (exact-zero failure rows preserved).
- Separate reporting of tracemalloc (Python allocations) and process RSS.
- Reproducible test/demo scripts and integrated real-data validation workflow/evidence.

### Results (actual, labeled)
**Independently verified in this workspace (synthetic degraded demo, after 1:1 contract fix):**
- Tests: 19 passed (`py -3 -m unittest discover -s tests -v`).
- Degraded run: 5914 input / 5914 output, 8 columns, normalized quaternions, finite values.
- Failure detected: true; 5s prediction generated: true; appended extra rows: 0.
- ATE RMSE (aligned): 5.130 m; blind-window RMSE (5s): 2.229 m; RPE ?1: 0.00975 m.
- Runtime: 8.542 s for 30 s IMU (~285 ms/s). tracemalloc peak: 2.488 MB. Process RSS peak: 41.55 MB (Windows psapi).

**Collaborator-reported real TUM-VI room4 evidence (integrated, NOT independently rerun in this workspace):**
- ATE RMSE: clean 98.46 m, degraded 97.38 m, dropout 115.08 m.
- RPE ?1: 0.0248 / 0.0261 / 0.0302 m.
- Runtime: 446?516 ms/s IMU. tracemalloc: 8.85?9.43 MB. Process RSS: 104.93?106.27 MB.
- Evidence files: `REAL_DATA_VALIDATION.md`, `evidence/real/tumvi_room4/*.json`.

### Limitations (must disclose)
- **Runtime NOT MET:** approximately **285 ms/s IMU** against a **20 ms/s** target (demo); collaborator real run 446?516 ms/s.
- **Process RAM NOT MET:** approximately **41.86 MB process RSS** against an **8 MB** target (demo); collaborator real RSS 104.93?106.27 MB. tracemalloc is lower but measures Python allocations only, not process RSS.
- IMU-only drift and yaw unobservability cause large absolute ATE on long real sequences.
- 5-second prediction is motion extrapolation (no future GT); residual horizon beyond last input cannot be appended without violating 1:1 output contract and is recorded in metadata.

## C) Links / files / uploads

| Form item | Draft answer / file | Status |
|---|---|---|
| GitHub link | https://github.com/ridhima936/PathFinder-IMU-Trajectory | VERIFIED |
| Main commit / branch | `main` @ `1cea58f` (integration `5ad871f`, feature fix `50f67f1`) | VERIFIED |
| Submission package / ZIP | `PathFinder_Final_Submission/submission_zip/Error_404_Not_Found.zip` | READY (confirm official team name before upload) |
| Source code | Included in repo and ZIP (`Error_404_Not_Found/src/`, full source in `PathFinder_Final_Submission/documentation/source_snapshot/`) | VERIFIED |
| Presentation (PPT/PDF) | `PathFinder_Final_Submission/presentation/Final_Presentation.pptx` and `.pdf` | DRAFT - replace college/member placeholders |
| Video (2?3 min) | `PathFinder_Final_Submission/video/PathFinder_Final_Presentation.mp4` (narrated slide-based video; ~3:37). Upload to YouTube/Drive and paste accessible link. | GENERATED - MANUAL UPLOAD REQUIRED |
| AI disclosure | `PathFinder_Final_Submission/documentation/AI_DISCLOSURE.md` | COMPLETE |
| README | `README.md` in repo; also `PathFinder_Final_Submission/documentation/docs/README.md` and `PathFinder_Final_Submission/README.md` | VERIFIED |
| APK / SDK | N/A ? Track 03 standalone Python submission (no Android APK/SDK deliverable in this package) | N/A |
| Other / evidence | `REAL_DATA_VALIDATION.md`, `evidence/real/tumvi_room4/*.json`, `documentation/SUBMISSION_STATUS.md`, test/demo outputs | VERIFIED |

## D) Pre-submit checklist (user action)
- [ ] Replace all `[USER-PROVIDED]` / `[CONFIRM: ...]` placeholders.
- [ ] Confirm official registered team name; rename ZIP/folder if different.
- [ ] Confirm presentation has final team/college/member details.
- [ ] Record and upload video; ensure link access is correct.
- [ ] Verify ZIP contents from clean extraction.
- [ ] Confirm all claims match evidence; keep runtime/RAM non-compliance and collaborator-reported real-data labels.
- [ ] Manually submit Google Form. **Agent must not submit.**
