"""Optional helper: evaluate TUM trajectories with evo (if installed).

Ground truth is used only for evaluation, never for inference.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def run(cmd):
    print("+", " ".join(cmd))
    p = subprocess.run(cmd, text=True)
    return p.returncode


def main():
    ap = argparse.ArgumentParser(description="Run evo_ape/evo_rpe on estimated TUM trajectory")
    ap.add_argument("gt", help="Ground truth trajectory (TUM .txt or supported by evo)")
    ap.add_argument("est", help="Estimated trajectory (TUM .txt)")
    ap.add_argument("--align", action="store_true", help="Use Umeyama alignment (SE3/Sim3 depending on evo flags)")
    ap.add_argument("--rpe", action="store_true", help="Also run evo_rpe")
    ap.add_argument("--plot", default=None, help="Optional plot output path (e.g. results/evo_ape.png)")
    args = ap.parse_args()

    if not shutil.which("evo_ape"):
        print("evo_ape not found. Install with: pip install evo")
        print("You can still use: python src/evaluation.py workflows or scripts/evaluate_tier*.py")
        return 2

    cmd = ["evo_ape", "tum", args.gt, args.est, "-va"]
    if args.align:
        cmd.append("--align")
    if args.plot:
        cmd += ["--plot", args.plot, "--save_plot", args.plot]
    rc = run(cmd)
    if args.rpe and shutil.which("evo_rpe"):
        rc2 = run(["evo_rpe", "tum", args.gt, args.est, "-va"] + (["--align"] if args.align else []))
        rc = rc or rc2
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
