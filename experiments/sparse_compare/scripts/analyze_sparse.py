#!/usr/bin/env python3
"""
Sparse ON vs OFF quantitative analysis for an arbitrary EuRoC sequence.

Identical alignment to plot_compare.py: flip_xy + Horn SE(3) + nearest resample.
Metrics: ATE, RPE (per-segment), drift, trajectory stats.

Usage:
  python3 analyze_sparse.py --seq MH04
  python3 analyze_sparse.py --seq MH05 --gt path/to/gt.csv --on ... --off ...
"""
import numpy as np
import os

ROOT = '/home/lxy/asr_sdm_robo'

DEFAULT_GT  = lambda seq: os.path.join(ROOT, f'experiments/sparse_compare/results/{seq.lower()}_gt.csv')
DEFAULT_ON  = lambda seq: os.path.join(ROOT, f'output/{seq}/sparse_on/vins_sparse_on.csv')
DEFAULT_OFF = lambda seq: os.path.join(ROOT, f'output/{seq}/sparse_off/vins_sparse_off.csv')


def load_csv(path):
    rows = []
    with open(path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) >= 4:
                try:
                    rows.append([float(x) for x in p[:4]])
                except:
                    pass
    return np.array(rows)


def load_vins(path):
    a = load_csv(path)
    if a[0, 0] > 1e15:
        a[:, 0] /= 1e9
    _, inv = np.unique(a[:, 0], return_inverse=True)
    out = np.zeros((inv.max() + 1, 4))
    last = {}
    for i, k in enumerate(inv):
        last[k] = i
    for k, i in last.items():
        out[k] = a[i]
    return out


def flip_xy(arr):
    out = arr.copy()
    out[:, 1] = -out[:, 1]
    out[:, 2] = -out[:, 2]
    return out


def first_peak_z(arr, thresh=0.3):
    z0 = arr[0, 3]
    for i in range(len(arr)):
        if abs(arr[i, 3] - z0) > thresh:
            return arr[i, 0]
    return arr[0, 0]


def resample(est_t, est_xyz, ref_t):
    """Nearest-neighbor resample — identical to plot_compare.py."""
    idx = np.searchsorted(est_t, ref_t)
    idx = np.clip(idx, 1, len(est_t) - 1)
    left = idx - 1
    choose_left = (ref_t - est_t[left]) <= (est_t[idx] - ref_t)
    return est_xyz[np.where(choose_left, left, idx)]


def se3_horn(src, dst):
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    p = src - src_mean
    q = dst - dst_mean
    H = p.T @ q
    U, S, Vh = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vh.T @ U.T))
    R = Vh.T @ np.diag([1, 1, d]) @ U.T
    t = dst_mean - R @ src_mean
    return R, t


def align_to(arr, R, t):
    return (R @ arr.T).T + t


def ate(a, b):
    return np.sqrt(np.mean(np.sum((a - b)**2, axis=1)))


def ate_aligned(gt_c, R, t, est_resampled):
    """ATE between aligned GT and resampled VINS — same as plot_compare.py."""
    gt_ali = align_to(gt_c, R, t)
    return ate(gt_ali, est_resampled)


def rpe(a, b, intervals=None):
    """Relative Pose Error at multiple segment lengths."""
    if intervals is None:
        intervals = [10, 20, 50, 100, 200]
    n = len(a)
    results = {}
    for seg in intervals:
        if seg >= n:
            continue
        step = max(1, (n - seg) // 500)
        errors = []
        for i in range(0, n - seg, step):
            da = a[i + seg] - a[i]
            db = b[i + seg] - b[i]
            errors.append(np.linalg.norm(da - db))
        results[seg] = {
            'mean': np.mean(errors),
            'median': np.median(errors),
            'std': np.std(errors),
            'max': np.max(errors),
        }
    return results


def main(dt=1.6, thresh=0.3, seq='MH03', gt_path=None, on_path=None, off_path=None):
    gt_path = gt_path or DEFAULT_GT(seq)
    on_path = on_path or DEFAULT_ON(seq)
    off_path = off_path or DEFAULT_OFF(seq)
    print("=" * 62)
    print(f"{seq} Sparse ON vs OFF — Quantitative Analysis")
    print(f"Alignment: flip_xy + nearest-resample + Horn SE(3)")
    print(f"Parameters: dt={dt}s, thresh={thresh}")
    print(f"GT  : {gt_path}")
    print(f"ON  : {on_path}")
    print(f"OFF : {off_path}")
    print("=" * 62)

    gt    = load_csv(gt_path)
    on_l  = load_vins(on_path)
    off_l = load_vins(off_path)

    gt_f  = flip_xy(gt)
    on_f  = flip_xy(on_l)
    off_f = flip_xy(off_l)

    v_t0 = first_peak_z(on_l, thresh) - dt
    g_t0 = first_peak_z(gt, thresh)

    gt_t   = gt_f[:, 0]  - g_t0
    on_t   = on_f[:, 0]  - v_t0
    off_t  = off_f[:, 0] - v_t0

    t_lo = max(gt_t[0], on_t[0])
    t_hi = min(gt_t[-1], on_t[-1])

    mask_gt   = (gt_t  >= t_lo) & (gt_t  <= t_hi)
    mask_on   = (on_t  >= t_lo) & (on_t  <= t_hi)
    mask_off  = (off_t >= t_lo) & (off_t <= t_hi)

    gt_tc  = gt_t[mask_gt];   gt_c   = gt_f[mask_gt,  1:4]
    on_tc  = on_t[mask_on];   on_c   = on_f[mask_on,  1:4]
    off_tc = off_t[mask_off]; off_c  = off_f[mask_off, 1:4]

    on_g  = resample(on_tc,  on_c,  gt_tc)
    off_g = resample(off_tc, off_c, gt_tc)

    R_on,  t_on  = se3_horn(gt_c, on_g)
    R_off, t_off = se3_horn(gt_c, off_g)

    ate_on  = ate_aligned(gt_c, R_on,  t_on,  on_g)
    ate_off = ate_aligned(gt_c, R_off, t_off, off_g)

    gt_ali_on  = align_to(gt_c, R_on,  t_on)
    gt_ali_off = align_to(gt_c, R_off, t_off)

    print(f"\n## 1. Absolute Trajectory Error (ATE)")
    print(f"  Sparse ON : {ate_on:.4f} m")
    print(f"  Sparse OFF: {ate_off:.4f} m")
    print(f"  Ratio OFF/ON: {ate_off/ate_on:.3f}x")

    # Per-axis errors
    err_on  = gt_ali_on  - on_g
    err_off = gt_ali_off - off_g

    print(f"\n## 2. Per-Axis RMSE [m]")
    for i, name in enumerate(['X', 'Y', 'Z']):
        rmse_on  = np.sqrt(np.mean(err_on[:,  i]**2))
        rmse_off = np.sqrt(np.mean(err_off[:, i]**2))
        ratio = rmse_off / rmse_on
        print(f"  {name}: ON={rmse_on:.4f}  OFF={rmse_off:.4f}  (OFF/ON={ratio:.3f}x)")

    print(f"\n## 3. Per-Axis Mean Abs Error [m]")
    for i, name in enumerate(['X', 'Y', 'Z']):
        mae_on  = np.mean(np.abs(err_on[:,  i]))
        mae_off = np.mean(np.abs(err_off[:, i]))
        ratio = mae_off / mae_on
        print(f"  {name}: ON={mae_on:.4f}  OFF={mae_off:.4f}  (OFF/ON={ratio:.3f}x)")

    print(f"\n## 4. Relative Pose Error (RPE) [m]")
    print(f"  {'seg':>6} | {'ON mean':>8} {'ON med':>8} {'ON max':>8} | {'OFF mean':>8} {'OFF med':>8} {'OFF max':>8}")
    print(f"  {'-'*6}-+-{'-'*26}-+-{'-'*26}")
    rpe_on  = rpe(gt_ali_on,  on_g)
    rpe_off = rpe(gt_ali_off, off_g)
    for seg in sorted(rpe_on.keys()):
        ron  = rpe_on[seg]
        roff = rpe_off[seg]
        print(f"  {seg:>6} | {ron['mean']:>8.4f} {ron['median']:>8.4f} {ron['max']:>8.4f} | {roff['mean']:>8.4f} {roff['median']:>8.4f} {roff['max']:>8.4f}")

    print(f"\n## 5. Drift (window=100 frames)")
    print(f"  Sparse ON : max={np.max([ate(gt_ali_on[i:i+100], on_g[i:i+100]) for i in range(0,len(gt_ali_on)-100,50)]):.4f}m")
    print(f"  Sparse OFF: max={np.max([ate(gt_ali_off[i:i+100], off_g[i:i+100]) for i in range(0,len(gt_ali_off)-100,50)]):.4f}m")

    print(f"\n## 6. Trajectory Statistics")
    print(f"  GT  : {len(gt_c):>5} pts,  duration={gt_tc[-1]-gt_tc[0]:.2f}s")
    print(f"  ON  : {len(on_c):>5} pts,  duration={on_tc[-1]-on_tc[0]:.2f}s")
    print(f"  OFF : {len(off_c):>5} pts,  duration={off_tc[-1]-off_tc[0]:.2f}s")

    print(f"\n## 7. Start/End Closed-Loop Gap [m]")
    for label, ali, est in [('ON', gt_ali_on, on_g), ('OFF', gt_ali_off, off_g)]:
        s = np.linalg.norm(ali[0]  - est[0])
        e = np.linalg.norm(ali[-1] - est[-1])
        print(f"  Sparse {label}: start_gap={s:.4f}  end_gap={e:.4f}")

    print(f"\n" + "=" * 62)
    print("Summary")
    print("=" * 62)
    winner = "ON" if ate_on < ate_off else "OFF"
    print(f"  ATE  : Sparse {winner} wins  ({min(ate_on, ate_off):.4f}m vs {max(ate_on, ate_off):.4f}m)")
    print(f"  ATE improvement (ON vs OFF): {(1 - ate_on/ate_off)*100:+.1f}%")
    print()


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--seq', type=str, default='MH03',
                   help='Sequence tag for default paths (e.g. MH03/MH04/MH05).')
    p.add_argument('--gt',  type=str, default=None, help='Override GT csv path.')
    p.add_argument('--on',  type=str, default=None, help='Override sparse-ON csv.')
    p.add_argument('--off', type=str, default=None, help='Override sparse-OFF csv.')
    p.add_argument('--dt', type=float, default=1.6)
    p.add_argument('--thresh', type=float, default=0.3)
    args = p.parse_args()
    main(dt=args.dt, thresh=args.thresh,
         seq=args.seq, gt_path=args.gt, on_path=args.on, off_path=args.off)
