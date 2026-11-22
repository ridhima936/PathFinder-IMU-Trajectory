# results/

Store evaluation outputs, benchmark summaries, and submission-ready artifacts generated during validation.

Suggested contents (generated, not hand-fabricated):
- `estimated_trajectory.txt` — TUM trajectory output for a run.
- `metadata.json` — run metadata (input, counts, processing time, peak memory, failure/prediction config).
- Tier summaries/plots/CSV metrics from `scripts/evaluate_tier*.py`.
- Real-dataset validation notes with exact commands and measured metrics.

Do not place large raw datasets here. Keep raw datasets in `data/` (gitignored) or outside the repository.