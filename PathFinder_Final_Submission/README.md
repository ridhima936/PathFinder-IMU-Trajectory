# PathFinder_Final_Submission

Final submission package for **Samsung PRISM XR Hackathon 2026 ? Track 03: Path Finder (Trajectory Estimation Under Sensor Degradation)**.

## Folder structure
```text
PathFinder_Final_Submission/
  presentation/
    Final_Presentation.pptx
    Final_Presentation.pdf
  submission_zip/
    Error_404_Not_Found.zip                  # Official Track 03 ZIP
    Error_404_Not_Found/                      # Extracted bundle (same content)
  documentation/
    README.md                             # Repo/project README snapshot
    AI_DISCLOSURE.md
    SUBMISSION_STATUS.md
    docs/                                 # Architecture/algorithm/experiments/audit/validation
    results/                              # estimated_trajectory.txt, metadata.json, demo_summary.json
    form/GOOGLE_FORM_DRAFT.md             # Draft only - not submitted
    source_snapshot/                      # Full source/tests/scripts snapshot
  video/
    VIDEO_SCRIPT_AND_CHECKLIST.md
    VIDEO_STATUS.md                       # No video recorded/uploaded by agent
  evidence/
    real/tumvi_room4/*.json               # Collaborator-reported real-data evidence
  README.md
```

## Key disclosures (must remain visible)
- **Runtime NOT MET:** ~285 ms/s IMU vs 20 ms/s target (independently verified demo); 446?516 ms/s collaborator real run.
- **Process RAM NOT MET:** ~41.86 MB process RSS vs 8 MB target (demo); 104.93?106.27 MB collaborator real. `tracemalloc` is Python allocations only, not process RSS.
- **Real TUM-VI results:** collaborator-reported evidence, not independently rerun in this workspace.
- **No fabrication:** no invented team identity/personal info, no live demo fabricated, no Google Form submitted. Video is narrated slide-based (not live software demo).

## Official Track 03 ZIP
`submission_zip/Error_404_Not_Found.zip` contains the required structure:
`Error_404_Not_Found/{estimated_trajectory.txt, metadata.json, src/main.py, src/*required modules*, src/README.md}`.  
ZIP integrity verified (`testzip()` None) and clean extraction verified.

## Reproducibility
```bash
py -3 -m unittest discover -s tests -v
py -3 scripts/run_demo.py
py -3 src/main.py --input <imu_csv_or_folder> --output estimated_trajectory.txt --metadata metadata.json
```
