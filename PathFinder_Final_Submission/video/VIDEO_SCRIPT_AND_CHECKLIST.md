# PathFinder ? 2?3 Minute Demo Video Script & Recording Checklist

**Target length:** 2:00?3:00  
**Track:** Samsung PRISM XR Hackathon 2026 Track 03  
**Status:** Script/checklist prepared. **Video has NOT been recorded or uploaded by the agent.**

## Recording checklist
- [ ] Python 3.10+ installed; deps: `pip install -r requirements.txt`
- [ ] Clean terminal, readable font, notifications off, mic checked
- [ ] Repo at known commit (show `git rev-parse --short HEAD` or state `1cea58f`)
- [ ] Screen recorder ready (OBS / Windows Game Bar / equivalent)
- [ ] Use actual commands/outputs; do not edit in fake results
- [ ] Do not show secrets/personal data
- [ ] Upload to YouTube/Drive with accessible sharing; add link to form draft

## Exact commands to run
```bash
cd PathFinder-IMU-Trajectory
git rev-parse --short HEAD
py -3 -m unittest discover -s tests -v
py -3 scripts/run_demo.py
py -3 -c "import json; print(json.dumps(json.load(open('outputs/demo/demo_summary.json'))['runs']['degraded'], indent=2))"
```
Optional (if dataset available, clearly label): run real-data validation script; otherwise do not claim independent real rerun.

## Scene-by-scene script (timed)

### 0:00?0:20 ? Title & problem
**On screen:** README title / slide 1.  
**Narration:** "Hi, we are PathFinder_Team. This is PathFinder for Samsung PRISM XR Hackathon 2026, Track 03: Trajectory Estimation Under Sensor Degradation. The task is to reconstruct 6-DoF IMU trajectory under degradation, detect complete sensor failure, and predict 5 seconds of future motion. The solution must be standalone Python, offline, single-threaded, with no GPS, camera, or future ground truth in inference."

### 0:20?0:45 ? Architecture
**On screen:** `ARCHITECTURE.md` or architecture slide.  
**Narration:** "Our pipeline reads EuRoC/TUM-VI IMU data, validates timestamps and units, preprocesses and corrects sensor errors, estimates orientation with quaternions, compensates gravity, integrates velocity and position, detects failure states, and generates 5-second dead reckoning. Ground truth is used only for offline evaluation."

### 0:45?1:25 ? Live tests and demo
**On screen:** terminal running commands above.  
**Narration:** "We run the unit test suite and the reproducible synthetic demo. The tests verify format, units, preprocessing, orientation, integration, failure detection, 5-second prediction, and the one-pose-per-input-frame output contract. The demo generates clean and degraded data, runs the estimator, and writes TUM trajectory outputs and metadata."

### 1:25?1:55 ? Output contract, failure, 5s prediction
**On screen:** `estimated_degraded.txt` header/rows, `metadata_degraded.json`, demo summary.  
**Narration:** "The output follows the required TUM format: timestamp, tx, ty, tz, qx, qy, qz, qw ? 8 columns, Unix seconds, normalized quaternions. We enforce exactly one pose per input IMU frame. After failure detection, the 5-second prediction is represented over the available input horizon; any residual beyond the last input timestamp is recorded in metadata, not appended as extra rows."

### 1:55?2:35 ? Results and honest limitations
**On screen:** results slide/table.  
**Narration:** "In our independently verified synthetic degraded demo, 19 tests passed, output was 5914 in and 5914 out, failure was detected, prediction was generated, ATE RMSE was 5.130 meters, blind-window RMSE was 2.229 meters, and RPE delta-1 was 0.00975 meters. We also disclose limitations: runtime is about 285 milliseconds per second of IMU data versus a 20 millisecond target, and process RSS is about 41.86 MB versus an 8 MB target. Real TUM-VI room4 results in the repo are collaborator-reported evidence and were not independently rerun in this workspace."

### 2:35?2:55 ? Closing
**On screen:** GitHub repo, submission package, limitations.  
**Narration:** "PathFinder is reproducible from the repository with documented commands, tests, and submission artifacts. Future work includes performance optimization and stronger real-dataset validation. Thank you."

## Post-recording checklist
- [ ] Confirm audio/video clarity and visible command outputs
- [ ] Confirm no fabricated claims or private data
- [ ] Upload video (YouTube unlisted/public or Drive with access)
- [ ] Paste link into `documentation/form/GOOGLE_FORM_DRAFT.md`
- [ ] Do not claim submission complete until form is manually submitted
