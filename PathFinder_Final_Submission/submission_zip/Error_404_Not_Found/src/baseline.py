"""
baseline.py  -  Track 3 (Path Finder) ka pehla dead-reckoning baseline.

Input : EuRoC imu0/data.csv  ->  timestamp[ns], wx, wy, wz [rad/s], ax, ay, az [m/s^2]
Output: estimated_trajectory.txt (TUM format) ->  timestamp[s] tx ty tz qx qy qz qw
        Input ki HAR row ke liye ek pose (deck ka rule: one pose per IMU frame).

Design (deck ki limits ke hisaab se: <= 8 MB RAM, <= 20 ms per 1 s IMU, single thread):
  * STREAMING: file ek-ek line padhte hain, ek-ek pose likhte hain. Poori file memory mein nahi.
  * PURE PYTHON: numpy/pandas import nahi (unki import hi kaafi RAM kha jaati hai).

Algorithm (baseline):
  1. Shuru ke INIT_SECONDS (drone khada maanke):
       - gyro ka average  = gyro bias (isse har gyro reading se minus karenge)
       - accel ka average = gravity ki direction -> isse starting roll/pitch milta hai
         (yaw pata nahi hota, isliye 0 maante hain; "unknown gravity alignment" yahi hai)
  2. Har sample par:
       - gyro integrate  -> orientation (quaternion)
       - accel ko world frame mein ghumao, gravity minus karo
       - accel integrate -> velocity, velocity integrate -> position
  3. Agar IMU "mar" gaya (saari 6 values exactly 0): orientation freeze,
     velocity constant (constant-velocity dead reckoning). Yeh T2 (blind 5 s) ka sabse simple version hai.

Filters (spike hatana, bias estimate) is file mein NAHI hain - woh src/filters.py mein jaayenge.
Plug-in tareeka: rows ek iterator hai, filter bas us iterator ko wrap karega.

Usage:
    py src/baseline.py data/synthetic/mav0 -o outputs/estimated_trajectory.txt
    py src/baseline.py path/to/imu0/data.csv -o estimated_trajectory.txt
"""
import argparse
import io
import math
import os
import sys
import zipfile

INIT_SECONDS = 0.5          # shuru ka itna time "khada" maanke calibrate karte hain
ZERO_EPS = 0.0              # failure = saari values exactly 0.0 (organiser script aisa hi karti hai)


# ---------------------------------------------------------------------------
# Chhote quaternion helpers. Quaternion = (w, x, y, z), body -> world rotation.
# ---------------------------------------------------------------------------
def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_from_rotvec(x, y, z):
    """Rotation vector (axis * angle, radians) -> quaternion."""
    th = math.sqrt(x * x + y * y + z * z)
    if th < 1e-12:
        return (1.0, 0.5 * x, 0.5 * y, 0.5 * z)
    s = math.sin(0.5 * th) / th
    return (math.cos(0.5 * th), x * s, y * s, z * s)


def q_rotate(q, v):
    """Vector v ko quaternion q se ghumao (body -> world)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def q_align_to_up(f):
    """
    Body-frame accel direction f (khade drone par = 'upar' ki direction) ko
    world ke (0,0,1) par laane wala quaternion. Yaw = 0 rakhte hain.
    """
    n = math.sqrt(f[0] ** 2 + f[1] ** 2 + f[2] ** 2)
    ax, ay, az = f[0] / n, f[1] / n, f[2] / n
    dot = az                                  # a . (0,0,1)
    if dot < -1.0 + 1e-9:                     # bilkul ulta (upside down): x-axis par 180 deg
        return (0.0, 1.0, 0.0, 0.0)
    # a x (0,0,1) = (ay, -ax, 0)
    return q_normalize((1.0 + dot, ay, -ax, 0.0))


# ---------------------------------------------------------------------------
# CSV padhna (streaming)
# ---------------------------------------------------------------------------
NAN = float("nan")


def open_imu_text(path):
    """
    (text file object, zip ya None) wapas karta hai.
    path .zip ho toh uske andar se imu0/data.csv khulti hai (extract kiye bina).
    """
    if path.lower().endswith(".zip"):
        zf = zipfile.ZipFile(path)
        names = [n for n in zf.namelist() if n.replace("\\", "/").endswith("imu0/data.csv")]
        if not names:
            zf.close()
            sys.exit(f"zip ke andar imu0/data.csv nahi mili: {path}")
        name = sorted(names, key=len)[0]
        return io.TextIOWrapper(zf.open(name), encoding="utf-8-sig"), zf
    return open(path, "r", encoding="utf-8-sig"), None


def read_rows(f, stats):
    """
    Yield (t_ns, wx, wy, wz, ax, ay, az). '#' wali lines aur header skip.
    Agar timestamp theek hai par values kharab/khaali/'nan' hain (FAQ: blind window mein sensor values
    'unusable/invalid' hongi), toh row SKIP NAHI karte -- values NaN bana ke de dete hain, taaki
    us row ka pose bhi nikle (rule: exactly one pose per IMU frame).
    """
    for line in f:
        line = line.strip()
        if not line or line[0] == "#":
            continue
        p = line.split(",")
        try:
            t = int(p[0])
        except ValueError:
            stats["skipped_lines"] += 1       # header ya bekaar line (timestamp hi nahi)
            continue
        vals = []
        for k in range(1, 7):
            try:
                vals.append(float(p[k]))
            except (ValueError, IndexError):
                vals.append(NAN)
        yield (t, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])


def resolve_imu_csv(path):
    """Folder ya seedha csv, dono chalenge."""
    if os.path.isdir(path):
        for cand in (os.path.join(path, "imu0", "data.csv"),
                     os.path.join(path, "mav0", "imu0", "data.csv"),
                     os.path.join(path, "data.csv")):
            if os.path.isfile(cand):
                return cand
        sys.exit(f"imu0/data.csv nahi mili is folder mein: {path}")
    return path


def fmt_pose(t_ns, p, q):
    """TUM line. Timestamp integer ns se seedha string bana rahe hain (precision nahi jaati)."""
    w, x, y, z = q
    return (f"{t_ns // 1000000000}.{t_ns % 1000000000:09d} "
            f"{p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {x:.9f} {y:.9f} {z:.9f} {w:.9f}\n")


# ---------------------------------------------------------------------------
# Main estimator
# ---------------------------------------------------------------------------
def estimate(rows, out, stats):
    """
    rows : iterator of (t_ns, wx, wy, wz, ax, ay, az)
    out  : file object jisme TUM lines likhni hain
    """
    # ---- 1. Init window buffer karo (sirf ~100 rows, memory ki chinta nahi) ----
    init = []
    t_first = None
    for row in rows:
        if t_first is None:
            t_first = row[0]
        init.append(row)
        if (row[0] - t_first) * 1e-9 >= INIT_SECONDS:
            break
    if len(init) < 5:
        sys.exit("Bahut kam rows hain, calibrate nahi ho sakta.")

    good = [r for r in init if math.isfinite(r[1] + r[2] + r[3] + r[4] + r[5] + r[6])]
    if len(good) < 5:
        sys.exit("Shuru ki rows mein valid IMU data hi nahi hai, calibrate nahi ho sakta.")
    n = float(len(good))
    gb = [sum(r[1 + k] for r in good) / n for k in range(3)]     # gyro bias
    fm = [sum(r[4 + k] for r in good) / n for k in range(3)]     # accel mean (khade drone par)
    g_est = math.sqrt(fm[0] ** 2 + fm[1] ** 2 + fm[2] ** 2)      # gravity magnitude (self-calibrated)

    # "khada tha ya nahi" ka sanity check (sirf warning, kaam rukta nahi)
    gstd = max(math.sqrt(sum((r[1 + k] - gb[k]) ** 2 for r in good) / n) for k in range(3))
    astd = max(math.sqrt(sum((r[4 + k] - fm[k]) ** 2 for r in good) / n) for k in range(3))
    stats["init_gyro_bias"] = gb
    stats["init_g"] = g_est
    stats["init_gyro_std"] = gstd
    stats["init_accel_std"] = astd
    gmag = math.sqrt(gb[0] ** 2 + gb[1] ** 2 + gb[2] ** 2)
    if gstd > 0.05 or astd > 0.5 or gmag > 0.15 or abs(g_est - 9.81) > 0.7:
        print("WARNING: shuru ke 0.5 s mein drone khada nahi lag raha, isliye result bahut kharab aa sakta hai!\n"
              f"  gyro average = {gmag:.3f} rad/s (khade drone par bias aam taur par < 0.05 hota hai)\n"
              f"  accel size   = {g_est:.2f} m/s^2 (khade drone par ~9.81 hona chahiye)\n"
              f"  gyro std = {gstd:.3f}, accel std = {astd:.3f}",
              file=sys.stderr)

    q = q_align_to_up(fm)
    p = [0.0, 0.0, 0.0]
    v = [0.0, 0.0, 0.0]
    a_prev = (0.0, 0.0, 0.0)          # khade drone par world accel = 0 (g_est ne cancel kar diya)
    w_prev = (0.0, 0.0, 0.0)
    prev_ns = init[-1][0]
    alive_prev = True

    # init wali rows ke liye pose: khada hua (p=0, q=q0)
    for r in init:
        out.write(fmt_pose(r[0], p, q))
        stats["rows_out"] += 1
    stats["rows_in"] += len(init)

    # ---- 2. Baaki rows: ek-ek karke ----
    for row in rows:
        stats["rows_in"] += 1
        t_ns, gx, gy, gz, ax, ay, az = row
        dt = (t_ns - prev_ns) * 1e-9

        if dt <= 0.0:
            # duplicate ya ulta timestamp: state mat chhedo, bas pose dobara likh do
            stats["nonpositive_dt"] += 1
            out.write(fmt_pose(t_ns, p, q))
            stats["rows_out"] += 1
            continue

        # IMU "dead" = values kharab (NaN/inf/khaali) YA saari 6 values exactly 0
        dead = (not math.isfinite(gx + gy + gz + ax + ay + az)) or (
                abs(gx) <= ZERO_EPS and abs(gy) <= ZERO_EPS and abs(gz) <= ZERO_EPS and
                abs(ax) <= ZERO_EPS and abs(ay) <= ZERO_EPS and abs(az) <= ZERO_EPS)

        if dead:
            # IMU fail: orientation freeze, velocity constant (a = 0)
            stats["dead_rows"] += 1
            p[0] += v[0] * dt
            p[1] += v[1] * dt
            p[2] += v[2] * dt
            a_prev = (0.0, 0.0, 0.0)
            alive_prev = False
        else:
            wx, wy, wz = gx - gb[0], gy - gb[1], gz - gb[2]
            if not alive_prev:
                w_prev = (wx, wy, wz)
                alive_prev = True
            # orientation: trapezoid (pichhle aur abhi ke gyro ka average)
            mx = 0.5 * (w_prev[0] + wx)
            my = 0.5 * (w_prev[1] + wy)
            mz = 0.5 * (w_prev[2] + wz)
            q = q_normalize(q_mul(q, q_from_rotvec(mx * dt, my * dt, mz * dt)))
            # accel -> world frame, gravity minus
            rf = q_rotate(q, (ax, ay, az))
            a_cur = (rf[0], rf[1], rf[2] - g_est)
            # velocity aur position: trapezoid
            vx = v[0] + 0.5 * (a_prev[0] + a_cur[0]) * dt
            vy = v[1] + 0.5 * (a_prev[1] + a_cur[1]) * dt
            vz = v[2] + 0.5 * (a_prev[2] + a_cur[2]) * dt
            p[0] += 0.5 * (v[0] + vx) * dt
            p[1] += 0.5 * (v[1] + vy) * dt
            p[2] += 0.5 * (v[2] + vz) * dt
            v[0], v[1], v[2] = vx, vy, vz
            a_prev = a_cur
            w_prev = (wx, wy, wz)

        prev_ns = t_ns
        out.write(fmt_pose(t_ns, p, q))
        stats["rows_out"] += 1


def run(imu_path, out_path):
    stats = {"rows_in": 0, "rows_out": 0, "dead_rows": 0, "nonpositive_dt": 0, "skipped_lines": 0}
    csv_path = resolve_imu_csv(imu_path)
    out_dir = os.path.dirname(os.path.abspath(out_path))
    os.makedirs(out_dir, exist_ok=True)
    f, zf = open_imu_text(csv_path)
    try:
        with open(out_path, "w", newline="\n") as out:
            out.write("# timestamp tx ty tz qx qy qz qw\n")
            estimate(read_rows(f, stats), out, stats)
    finally:
        f.close()
        if zf is not None:
            zf.close()
    return stats


def main():
    ap = argparse.ArgumentParser(description="IMU dead-reckoning baseline (Track 3)")
    ap.add_argument("input", help="imu0/data.csv, ya folder jisme imu0/data.csv hai, ya .zip")
    ap.add_argument("-o", "--output", default="estimated_trajectory.txt")
    args = ap.parse_args()

    stats = run(args.input, args.output)
    print(f"rows_in={stats['rows_in']}  rows_out={stats['rows_out']}  "
          f"dead_rows={stats['dead_rows']}  duplicate/ulte_timestamps={stats['nonpositive_dt']}")
    if stats["skipped_lines"]:
        print(f"WARNING: {stats['skipped_lines']} lines parse nahi hui (skip ki)", file=sys.stderr)
    print(f"written: {args.output}")


if __name__ == "__main__":
    main()
