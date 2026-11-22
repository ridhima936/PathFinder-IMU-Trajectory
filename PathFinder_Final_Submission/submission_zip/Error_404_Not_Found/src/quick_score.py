"""
quick_score.py  -  estimated trajectory ka ATE score (numpy-only, evo ki zaroorat nahi).

Kya karta hai:
  1. estimated_trajectory.txt aur ground truth ko timestamp se match karta hai
  2. Umeyama se est ko GT par rigid align karta hai (rotation + translation, scale nahi)
     -> yeh wahi hai jo `evo_ape tum gt est --align` karta hai
  3. ATE RMSE (position error, meters) aur orientation error (degrees) batata hai
     - poore path ka, failure se pehle ka, aur aakhri 5 s (blind) ka alag

GT formats (extension se pehchanta hai):
  *.txt  -> TUM:   t tx ty tz qx qy qz qw
  *.csv  -> EuRoC: #timestamp[ns], px, py, pz, qw, qx, qy, qz, ...

Usage:
    py src/quick_score.py data/synthetic/mav0/groundtruth_tum.txt outputs/estimated_trajectory.txt
"""
import argparse
import sys

import numpy as np


def load_gt(path):
    if path.lower().endswith(".csv"):
        a = np.loadtxt(path, delimiter=",", comments="#", usecols=range(0, 8))
        t = a[:, 0] * 1e-9
        pos = a[:, 1:4]
        q = a[:, [5, 6, 7, 4]]            # EuRoC: qw qx qy qz  ->  qx qy qz qw
        return t, pos, q
    a = np.loadtxt(path, comments="#")
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def quat_to_R(q):
    """q: (N,4) as (x,y,z,w) -> (N,3,3)."""
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def associate(t_est, t_gt, max_diff):
    """Har est timestamp ke liye sabse paas wala GT index. (est_idx, gt_idx) wapas."""
    order = np.argsort(t_gt)
    tg = t_gt[order]
    j = np.clip(np.searchsorted(tg, t_est), 1, len(tg) - 1)
    left, right = tg[j - 1], tg[j]
    pick = np.where(np.abs(t_est - left) <= np.abs(t_est - right), j - 1, j)
    ok = np.abs(tg[pick] - t_est) <= max_diff
    return np.flatnonzero(ok), order[pick[ok]]


def umeyama(src, dst):
    """dst ~= R @ src + t (rigid, no scale)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = mu_d - R @ mu_s
    return R, t


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def score(gt_path, est_path, fail_seconds=5.0, max_diff=0.0025):
    t_gt, p_gt, q_gt = load_gt(gt_path)
    est = np.loadtxt(est_path, comments="#")
    if est.ndim != 2 or est.shape[1] != 8:
        raise ValueError(f"estimated file mein exactly 8 columns chahiye, mile: {est.shape}")
    t_est, p_est, q_est = est[:, 0], est[:, 1:4], est[:, 4:8]

    ie, ig = associate(t_est, t_gt, max_diff)
    if len(ie) < 10:
        raise ValueError("GT aur estimate ke timestamps match nahi ho rahe (10 se kam pairs).")
    t, pe, pg = t_est[ie], p_est[ie], p_gt[ig]
    Re, Rg = quat_to_R(q_est[ie]), quat_to_R(q_gt[ig])

    R_al, t_al = umeyama(pe, pg)
    pe_al = pe @ R_al.T + t_al
    err = np.linalg.norm(pe_al - pg, axis=1)

    Re_al = np.einsum("ij,njk->nik", R_al, Re)
    tr = np.einsum("nji,njk->nik", Rg, Re_al)          # Rg^T @ Re_al
    cosang = np.clip((np.trace(tr, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    rot_err = np.degrees(np.arccos(cosang))

    blind = t > (t_est[-1] - fail_seconds)
    pre = ~blind
    return {
        "matched": int(len(ie)),
        "est_rows": int(len(t_est)),
        "ate_all": rms(err),
        "ate_pre": rms(err[pre]),
        "ate_blind": rms(err[blind]),
        "final_pos_err": float(err[-1]),
        "rot_mean": float(rot_err.mean()),
        "rot_max": float(rot_err.max()),
        "rot_pre_mean": float(rot_err[pre].mean()) if pre.any() else float("nan"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gt")
    ap.add_argument("est")
    ap.add_argument("--fail-seconds", type=float, default=5.0, help="aakhri itne sec ko 'blind' maano")
    args = ap.parse_args()
    try:
        s = score(args.gt, args.est, args.fail_seconds)
    except (ValueError, OSError) as e:
        sys.exit(f"ERROR: {e}")
    print(f"matched poses      : {s['matched']} / {s['est_rows']}")
    print(f"ATE RMSE (all)     : {s['ate_all']:.4f} m")
    print(f"ATE RMSE (pre-fail): {s['ate_pre']:.4f} m")
    print(f"ATE RMSE (blind {args.fail_seconds:g}s): {s['ate_blind']:.4f} m")
    print(f"final position err : {s['final_pos_err']:.4f} m")
    print(f"orientation err    : mean {s['rot_mean']:.3f} deg, max {s['rot_max']:.3f} deg "
          f"(pre-fail mean {s['rot_pre_mean']:.3f} deg)")


if __name__ == "__main__":
    main()
