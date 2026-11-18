"""
main.py - Track 3 (Path Finder) ka entry point.

    py src/main.py <input> [-o estimated_trajectory.txt]

<input> = imu0/data.csv, ya folder jisme imu0/data.csv hai, ya .zip.
Output  = estimated_trajectory.txt (TUM format, har input row ke liye ek pose).

Abhi yeh sirf baseline.py ko chalata hai. Jaise jaise filters.py / predict.py ready hote jaayenge,
yahan jodte jaayenge. (Aakhir mein sab ek hi standalone main.py mein merge karenge -- FAQ ke hisaab se.)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from baseline import main  # noqa: E402

if __name__ == "__main__":
    main()
