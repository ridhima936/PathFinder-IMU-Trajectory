"""Download helper for small public IMU validation datasets.

This script attempts to download small/public EuRoC and TUM-VI files for validation.
It keeps datasets outside source code (default: data/external/).
Do NOT commit downloaded datasets to GitHub.

If automatic download fails (network restrictions, mirrors, size), follow the printed
manual instructions.
"""

from __future__ import annotations

import argparse
import os
import sys
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "data" / "external"

# Official/public sources (URLs may change; script reports failures clearly).
EUROC_ASL_BASE = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
TUMVI_BASE = "https://vision.in.tum.de/tumvi/exported/euroc/512_16"

EUROC_SMALL = {
    # Full sequence zips are large; we still provide them as official sources.
    "MH_01_easy": f"{EUROC_ASL_BASE}/machine_hall/MH_01_easy/MH_01_easy.zip",
    "V1_01_easy": f"{EUROC_ASL_BASE}/vicon_room1/V1_01_easy/V1_01_easy.zip",
}

TUMVI_SMALL = {
    # TUM-VI exported Euroc-format tar.gz files are large; provided for completeness.
    "dataset-room1_512_16": f"{TUMVI_BASE}/dataset-room1_512_16.tar.gz",
}


def download(url: str, dest: Path) -> bool:
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    try:
        print(f"Downloading: {url}")
        req = urllib.request.Request(url, headers={"User-Agent": "PathFinder-dataset-downloader/1.0"})
        with urllib.request.urlopen(req, timeout=60) as r, open(tmp, "wb") as f:
            total = r.headers.get("Content-Length")
            total = int(total) if total and total.isdigit() else None
            read = 0
            while True:
                chunk = r.read(1024 * 1024)
                if not chunk:
                    break
                f.write(chunk)
                read += len(chunk)
                if total:
                    pct = read * 100 / total
                    print(f"  {read/1e6:.1f} MB / {total/1e6:.1f} MB ({pct:.1f}%)", end="\r")
        print()
        tmp.replace(dest)
        print(f"Saved: {dest}")
        return True
    except Exception as e:
        print(f"FAILED: {url}\n  Reason: {e}")
        try:
            if tmp.exists():
                tmp.unlink()
        except Exception:
            pass
        return False


def maybe_extract(path: Path) -> None:
    if path.suffix.lower() == ".zip":
        out_dir = path.with_suffix("")
        print(f"Extracting zip to: {out_dir}")
        with zipfile.ZipFile(path, "r") as zf:
            zf.extractall(out_dir)
        print("Extraction complete.")


def manual_instructions() -> str:
    return f"""
Manual dataset instructions (if automatic download fails):

EuRoC MAV (official):
  - Dataset page: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
  - Example small/easy sequences: MH_01_easy, V1_01_easy
  - Download the zip, extract it, and use the IMU file:
      <sequence>/mav0/imu0/data.csv
  - Ground truth (for evaluation only): <sequence>/mav0/state_groundtruth_estimate0/data.csv

TUM-VI (official):
  - Dataset page: https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
  - Download Euroc/512_16 exported sequences (e.g., dataset-room1_512_16).
  - Extract and locate the IMU data (commonly mav0/imu0/data.csv in Euroc exports).

Store datasets under: {DEFAULT_OUT}
Do NOT commit datasets to git. `data/` is gitignored.
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="Output directory for downloaded datasets")
    ap.add_argument("--euroc", nargs="*", default=[], help="EuRoC sequences to download (e.g. MH_01_easy)")
    ap.add_argument("--tumvi", nargs="*", default=[], help="TUM-VI exports to download")
    ap.add_argument("--all-small", action="store_true", help="Attempt known small/easy entries (may be large downloads)")
    ap.add_argument("--extract", action="store_true", help="Extract zip files after download")
    ap.add_argument("--list", action="store_true", help="List known dataset keys and URLs")
    args = ap.parse_args()

    if args.list:
        print("EuRoC:")
        for k, v in EUROC_SMALL.items():
            print(f"  {k}: {v}")
        print("TUM-VI:")
        for k, v in TUMVI_SMALL.items():
            print(f"  {k}: {v}")
        print(manual_instructions())
        return 0

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    euroc_keys = list(args.euroc)
    tumvi_keys = list(args.tumvi)
    if args.all_small:
        euroc_keys = list(EUROC_SMALL.keys())
        tumvi_keys = list(TUMVI_SMALL.keys())

    if not euroc_keys and not tumvi_keys:
        print("No datasets selected. Use --list, --euroc MH_01_easy, --tumvi dataset-room1_512_16, or --all-small.")
        print(manual_instructions())
        return 2

    ok = True
    for k in euroc_keys:
        url = EUROC_SMALL.get(k)
        if not url:
            print(f"Unknown EuRoC key: {k}. Known: {', '.join(EUROC_SMALL)}")
            ok = False
            continue
        dest = out / "euroc" / f"{k}.zip"
        if download(url, dest) and args.extract:
            maybe_extract(dest)
        else:
            ok = ok and dest.exists()

    for k in tumvi_keys:
        url = TUMVI_SMALL.get(k)
        if not url:
            print(f"Unknown TUM-VI key: {k}. Known: {', '.join(TUMVI_SMALL)}")
            ok = False
            continue
        dest = out / "tumvi" / Path(url).name
        if not download(url, dest):
            ok = False

    if not ok:
        print(manual_instructions())
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
