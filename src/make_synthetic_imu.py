"""
make_synthetic_imu.py
Ek nakli (synthetic) EuRoC-format IMU dataset banata hai, jiska sahi jawab
(ground truth) humein pehle se pata hota hai.

Trajectory: drone circle mein udta hai + upar-neeche hota hai + thoda tilt (roll/pitch).

Output (repo root se chalao):
    data/synthetic/mav0/imu0/data.csv      <- EuRoC format (gyro pehle, accel baad mein)
    data/synthetic/mav0/groundtruth_tum.txt <- TUM format: t tx ty tz qx qy qz qw

Usage:
    py src/make_synthetic_imu.py
    py src/make_synthetic_imu.py --duration 120 --out data/synthetic
"""
import argparse
import os

import numpy as np

G = 9.81  # gravity (m/s^2)

# Traj parameters
RADIUS = 5.0   # circle ka radius (m)
OMEGA = 0.4    # circle ki angular speed (rad/s)
Z_AMP = 1.0    # upar-neeche amplitude (m)
Z_FREQ = 0.6   # upar-neeche frequency (rad/s)


def position(t):
    return np.stack([RADIUS * np.cos(OMEGA * t),
                     RADIUS * np.sin(OMEGA * t),
                     Z_AMP * np.sin(Z_FREQ * t)], axis=1)


def accel_world(t):
    """Position ka second derivative (analytic)."""
    return np.stack([-RADIUS * OMEGA ** 2 * np.cos(OMEGA * t),
                     -RADIUS * OMEGA ** 2 * np.sin(OMEGA * t),
                     -Z_AMP * Z_FREQ ** 2 * np.sin(Z_FREQ * t)], axis=1)


def velocity(t):
    return np.stack([-RADIUS * OMEGA * np.sin(OMEGA * t),
                     RADIUS * OMEGA * np.cos(OMEGA * t),
                     Z_AMP * Z_FREQ * np.cos(Z_FREQ * t)], axis=1)


def rotation(t):
    """Body -> World rotation matrices (N,3,3). Yaw = chalne ki direction."""
    yaw = OMEGA * t + np.pi / 2
    roll = 0.15 * np.sin(0.8 * t)
    pitch = 0.10 * np.sin(0.5 * t + 1.0)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    R = np.zeros((len(t), 3, 3))
    R[:, 0, 0] = cy * cp
    R[:, 0, 1] = cy * sp * sr - sy * cr
    R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp
    R[:, 1, 1] = sy * sp * sr + cy * cr
    R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp
    R[:, 2, 1] = cp * sr
    R[:, 2, 2] = cp * cr
    return R


def rot_to_quat(R):
    """Rotation matrix -> quaternion (qx, qy, qz, qw), qw >= 0."""
    out = np.zeros((len(R), 4))
    for i, m in enumerate(R):
        tr = m[0, 0] + m[1, 1] + m[2, 2]
        if tr > 0:
            s = 2.0 * np.sqrt(tr + 1.0)
            w = 0.25 * s
            x = (m[2, 1] - m[1, 2]) / s
            y = (m[0, 2] - m[2, 0]) / s
            z = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        q = np.array([x, y, z, w])
        if q[3] < 0:
            q = -q
        out[i] = q / np.linalg.norm(q)
    return out


def gyro_body(t, h=1e-5):
    """Body frame mein angular velocity: vee(R^T * dR/dt)."""
    R = rotation(t)
    dR = (rotation(t + h) - rotation(t - h)) / (2 * h)
    M = np.einsum("nji,njk->nik", R, dR)  # R^T @ dR
    return np.stack([M[:, 2, 1], M[:, 0, 2], M[:, 1, 0]], axis=1)


def ns_to_sec_str(ns):
    return f"{ns // 10**9}.{ns % 10**9:09d}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0, help="seconds")
    ap.add_argument("--rate", type=float, default=200.0, help="IMU Hz")
    ap.add_argument("--out", default="data/synthetic")
    args = ap.parse_args()

    n = int(args.duration * args.rate) + 1
    t0_ns = 1403636579758555392                 # EuRoC jaisa bada nanosecond timestamp
    step_ns = int(round(1e9 / args.rate))
    ts_ns = t0_ns + np.arange(n, dtype=np.int64) * step_ns
    t = np.arange(n) * (step_ns * 1e-9)         # 0 se shuru hone wala time (sec)

    R = rotation(t)
    gyro = gyro_body(t)
    # accelerometer "specific force" naapta hai: body frame mein (a_world + g_up)
    a_w = accel_world(t) + np.array([0.0, 0.0, G])
    accel = np.einsum("nji,nj->ni", R, a_w)     # R^T @ a_w

    pos = position(t)
    quat = rot_to_quat(R)

    imu_dir = os.path.join(args.out, "mav0", "imu0")
    os.makedirs(imu_dir, exist_ok=True)

    header = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
              "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
              "a_RS_S_z [m s^-2]\n")
    with open(os.path.join(imu_dir, "data.csv"), "w", newline="") as f:
        f.write(header)
        for i in range(n):
            f.write(f"{ts_ns[i]},"
                    + ",".join(f"{v:.9f}" for v in gyro[i]) + ","
                    + ",".join(f"{v:.9f}" for v in accel[i]) + "\n")

    gt_path = os.path.join(args.out, "mav0", "groundtruth_tum.txt")
    with open(gt_path, "w", newline="") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(n):
            f.write(ns_to_sec_str(int(ts_ns[i])) + " "
                    + " ".join(f"{v:.9f}" for v in pos[i]) + " "
                    + " ".join(f"{v:.9f}" for v in quat[i]) + "\n")

    print(f"rows={n}  duration={args.duration}s  rate={args.rate}Hz")
    print(f"written: {os.path.join(imu_dir, 'data.csv')}")
    print(f"written: {gt_path}")
    print(f"start pos={pos[0].round(3)}  start vel={velocity(t[:1])[0].round(3)}")


if __name__ == "__main__":
    main()
