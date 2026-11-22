"""Fetch only IMU + ground-truth CSVs from a TUM-VI EuroC-export tar archive.

TUM-VI sequences are published as large (1-3 GB) uncompressed tar files.  This
script downloads the archive once (optionally caching it outside the repo) and
extracts only the two text files needed by PathFinder, so validation does not
require keeping camera images or unpacking the full sequence.

Ground truth is downloaded for evaluation only.  The estimator never reads it.

Example:
    python scripts/fetch_tumvi_subset.py --sequence room4 \
        --tar-cache "$TEMP/dataset-room4_512_16.tar" \
        --out-dir data/real/tumvi_room4
    python scripts/validate_real_sequence.py --dataset data/real/tumvi_room4 \
        --sequence tumvi_room4 --out-dir outputs/real/tumvi_room4

The preparation step needs network access; the estimator itself is offline.
"""

from __future__ import annotations

import argparse
import os
import sys
import tarfile
import urllib.request

BASE_URL = "https://cvg.cit.tum.de/tumvi/exported/euroc/512_16/"
ARCHIVE = "dataset-{sequence}_512_16.tar"
CHUNK = 1024 * 1024


def download(url: str, dst_path: str, timeout: float = 120.0) -> str:
    """Download an archive to dst_path with progress reporting."""
    os.makedirs(os.path.dirname(os.path.abspath(dst_path)) or ".", exist_ok=True)
    tmp = dst_path + ".part"
    with urllib.request.urlopen(urllib.request.Request(url), timeout=timeout) as resp, open(tmp, "wb") as f:
        total = int(resp.headers.get("Content-Length") or 0)
        done = 0
        while True:
            chunk = resp.read(CHUNK)
            if not chunk:
                break
            f.write(chunk)
            done += len(chunk)
            suffix = f"/{total / 1e6:.1f}" if total else ""
            print(f"\r  downloaded {done / 1e6:.1f}{suffix} MB", end="", file=sys.stderr, flush=True)
    print(file=sys.stderr)
    os.replace(tmp, dst_path)
    return dst_path


def extract_from_fileobj(fileobj, out_dir: str, imu_name: str, gt_name: str) -> dict:
    """Extract only IMU and ground-truth CSVs from a tar stream."""
    imu_dst = os.path.join(out_dir, "mav0", imu_name.replace("/", os.sep))
    gt_dst = os.path.join(out_dir, "mav0", gt_name.replace("/", os.sep))
    os.makedirs(os.path.dirname(imu_dst), exist_ok=True)
    os.makedirs(os.path.dirname(gt_dst), exist_ok=True)

    targets = {
        "imu": (imu_name, imu_dst),
        "gt": (gt_name, gt_dst),
    }
    found = set()
    with tarfile.open(fileobj=fileobj, mode="r|") as tf:
        for member in tf:
            if not member.isfile():
                continue
            key = next((k for k, (suffix, _) in targets.items() if member.name.endswith("/" + suffix)), None)
            if key is None:
                continue
            src = tf.extractfile(member)
            if src is None:
                continue
            dst_path = targets[key][1]
            with src, open(dst_path, "wb") as dst:
                while True:
                    chunk = src.read(CHUNK)
                    if not chunk:
                        break
                    dst.write(chunk)
            found.add(key)
            label = "ground truth (evaluation only)" if key == "gt" else "IMU"
            print(f"{label}: {dst_path} ({member.size} bytes)", file=sys.stderr)
            if len(found) == len(targets):
                break
    if len(found) != len(targets):
        raise RuntimeError(f"Archive did not contain {imu_name} and {gt_name}")
    return {"imu": imu_dst, "gt": gt_dst}


def extract_subset(
    archive: str,
    out_dir: str,
    imu_name: str = "imu0/data.csv",
    gt_name: str = "state_groundtruth_estimate0/data.csv",
) -> dict:
    """Extract the two CSVs from a local tar archive."""
    with open(archive, "rb") as f:
        return extract_from_fileobj(f, out_dir, imu_name, gt_name)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sequence", help="TUM-VI sequence, e.g. room4 or room5")
    ap.add_argument("--url", default=None, help="Override archive URL")
    ap.add_argument("--out-dir", required=True, help="Destination dataset folder")
    ap.add_argument("--tar-cache", default=None, help="Local path for caching the downloaded archive")
    ap.add_argument("--imu-name", default="imu0/data.csv")
    ap.add_argument("--gt-name", default="mocap0/data.csv",
                    help="TUM-VI uses mocap0/data.csv; EuRoC uses state_groundtruth_estimate0/data.csv")
    args = ap.parse_args()

    if not args.url and not args.sequence:
        ap.error("provide --sequence or --url")
    url = args.url or (BASE_URL + ARCHIVE.format(sequence=args.sequence))

    if args.tar_cache:
        if not os.path.exists(args.tar_cache):
            print(f"downloading {url} -> {args.tar_cache}", file=sys.stderr)
            download(url, args.tar_cache)
        else:
            print(f"using cached archive {args.tar_cache}", file=sys.stderr)
        paths = extract_subset(args.tar_cache, args.out_dir, args.imu_name, args.gt_name)
    else:
        print(f"streaming {url}", file=sys.stderr)
        with urllib.request.urlopen(urllib.request.Request(url), timeout=120) as resp:
            paths = extract_from_fileobj(resp, args.out_dir, args.imu_name, args.gt_name)
    print(paths)


if __name__ == "__main__":
    main()
