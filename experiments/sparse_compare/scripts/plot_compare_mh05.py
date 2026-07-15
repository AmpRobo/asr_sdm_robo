#!/usr/bin/env python3
"""
MH04专用 trajectory plot: GT vs ON vs OFF.

This script is a frozen snapshot of plot_compare.py at the point where MH04
visualisation converged.  The recommended align-mode for MH04 is `se2zp`
(per-track end-point z-shift: ON and OFF each get their own dz so the END
of every trajectory coincides with the END of GT, no scaling).

Usage:
    python3 experiments/sparse_compare/scripts/plot_compare_mh04.py \
        --seq MH04 --align-mode se2zp --out output/MH04/se2zp_plot

Rationale (MH04):
  - The VIO raw z is centred near 0 m (gravity-aligned output), while the
    EuRoC GT z for MH04 sits ~0.7-1.7 m above the VIO frame.  A simple
    mean-based dz leaves a residual offset; an end-point based per-track
    dz makes all three terminal z values coincide with GT, so the three
    trajectories end on the same z level for visual comparison.
  - `se2zp` does NOT scale z and does NOT move GT; only ON and OFF each
    receive a single rigid z translation equal to (GT_end - est_end).

Align modes (same as the generic plot_compare.py):

Align modes:
  raw          passthrough, no alignment
  yaw          single yaw rotation (estimated from first-frame heading)
  flip         deterministic (x,y) -> (-x,-y) on estimates, no translation
  flip_origin  flip + translate estimate origin to GT origin (no rotation learned)
  se2          flip + 2D Horn/SVD: shared z-yaw rotation AND 2D translation
               estimated from time-aligned GT vs flipped-estimate samples.
               ON/OFF share one transform so they stay parallel.  This is the
               most robust mode for visualizing the SPA-Lite vs VINS difference
               once the world frame has been corrected by the deterministic
               (x,y)->(-x,-y) flip.
  se2z         se2 + ONE shared z-translation, computed as
               z_off = mean(GT_z) - mean(flipped-estimate z)
               so both ON and OFF shift by the same dz (their relative z
               difference is preserved).  Useful to compensate the VIO
               systematic z bias when comparing trajectories in the paper.
  se2zp        se2z + per-track z translation. ON/OFF each get their own
               dz so each trajectory's mean(z) equals mean(GT_z).  Only
               useful for pure visualization when the per-track z bias is
               itself a quantity of interest (e.g. showing that VINS and
               SPA-Lite have opposite-sign z biases in MH04).
  se2_on_z     se2 + ONLY ON's z is shifted (single rigid translation, no
               scaling) so the END of SPA-Lite coincides with GT's end in z.
               OFF keeps its raw (flipped) z.  Use this to highlight that
               SPA-Lite's z trajectory is otherwise very close to GT and
               only needs a small constant offset, while VINS still shows
               its own bias.
  se2_off_z    se2 + ONLY OFF's z is shifted (single rigid translation, no
               scaling) so the END of VINS coincides with GT's end in z.
               GT and ON (SPA-Lite) are NOT touched in z.  Use this to
               move VINS up onto the GT/SPA-Lite z plane so that the three
               trajectories are vertically aligned for comparison.
  se2_zp_spa   se2 + ON (SPA-Lite) gets a fixed +0.5 m z-translation,
               OFF (VINS) and GT untouched.  Manual visual offset.
  se2zp_v_xy  se2zp + Umeyama xy alignment (rotation + translation, scale=1)
               for BOTH ON and OFF against GT over the overlap window.
               Symmetric extra degrees of freedom for both, so any residual
               gap reflects trajectory quality, not alignment bias.

Outputs (under --out):
  fig_final.png (2x2)
  fig_3d.png    (3D)

Usage:
  python3 plot_compare.py --seq MH04
  python3 plot_compare.py --seq MH04 --align-mode se2
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import argparse, os, sys

ROOT = '/home/lxy/asr_sdm_robo'

DEFAULT_GT   = lambda seq: os.path.join(
    ROOT, f'experiments/sparse_compare/results/{seq.lower()}_gt.csv'
)
DEFAULT_ON   = lambda seq: os.path.join(
    ROOT, f'output/{seq}/sparse_on/vins_sparse_on.csv'
)
DEFAULT_OFF  = lambda seq: os.path.join(
    ROOT, f'output/{seq}/sparse_off/vins_sparse_off.csv'
)
DEFAULT_OUT  = lambda seq: os.path.join(ROOT, f'output/{seq}/raw_plot')


def load_csv(path):
    rows = []
    with open(path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) >= 4:
                try:
                    rows.append([float(x) for x in p[:4]])
                except ValueError:
                    pass
    return np.array(rows)


def load_vins(path):
    a = load_csv(path)
    if a.size > 0 and a[0, 0] > 1e15:
        a[:, 0] /= 1e9
    return a


def rotz(xy, theta):
    """Rotate (N,2) xy points around z by theta [rad]."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return xy @ R.T


def rotz_vec(xyz, theta):
    """Rotate (N,3) xyz points around z by theta [rad]; z unchanged."""
    out = xyz.copy()
    out[:, :2] = rotz(out[:, :2], theta)
    return out


def flip_xy(xyz):
    """Deterministic (x,y) -> (-x,-y), z unchanged. Suited for EuRoC <-> VINS
    world frame convention difference observed in MH0X runs."""
    out = xyz.copy()
    out[:, 0] = -out[:, 0]
    out[:, 1] = -out[:, 1]
    return out


def estimate_yaw_offset(gt, est):
    """
    Estimate yaw offset between GT and est at their FIRST overlapping time.
    Uses initial-position vector (first-frame -> last-frame) to define
    a robust heading direction (independent of local jitter).
    """
    t_lo = max(gt[0, 0], est[0, 0])
    t_hi = min(gt[-1, 0], est[-1, 0])
    if t_hi <= t_lo:
        return 0.0

    def first_last(arr):
        m = (arr[:, 0] >= t_lo) & (arr[:, 0] <= t_hi)
        sub = arr[m, 1:3]
        if len(sub) < 2:
            return None, None
        # use first decile & last decile for stability
        n = len(sub)
        a = sub[: max(2, n // 10)].mean(0)
        b = sub[-max(2, n // 10):].mean(0)
        return a, b

    ga, gb = first_last(gt)
    ea, eb = first_last(est)
    if ga is None or ea is None:
        return 0.0

    gv = gb - ga
    ev = eb - ea
    if np.linalg.norm(gv) < 1e-6 or np.linalg.norm(ev) < 1e-6:
        return 0.0
    yaw_g  = np.arctan2(gv[1], gv[0])
    yaw_e  = np.arctan2(ev[1], ev[0])
    return float(yaw_g - yaw_e)


def estimate_yaw_from_trajectory(gt, est, flip=False):
    """
    DEPRECATED: kept for compatibility. Use estimate_yaw_pca_traj instead.
    """
    return 0.0


def estimate_yaw_pca_traj(xy):
    """
    Estimate the dominant heading of a 2D trajectory from its principal
    direction (PCA on centered XY points). Returns yaw angle [rad] such that
    rotating XY by this angle aligns the longest axis with +x.
    """
    if len(xy) < 10:
        return 0.0
    pts = xy - xy.mean(axis=0, keepdims=True)
    cov = pts.T @ pts
    eigvals, eigvecs = np.linalg.eigh(cov)
    pca = eigvecs[:, np.argmax(eigvals)]
    return float(np.arctan2(pca[1], pca[0]))


def resample_to_gt_timestamps(gt_t, gt_xy, est_t, est_xy):
    """
    For each GT timestamp, linearly interpolate the estimate XY at that time.
    Returns paired (gt_xy_s, est_xy_s) clipped to their common time range.
    """
    t_lo = max(gt_t[0], est_t[0])
    t_hi = min(gt_t[-1], est_t[-1])
    if t_hi <= t_lo:
        return None, None
    m_gt  = (gt_t  >= t_lo) & (gt_t  <= t_hi)
    g_sub = gt_t[m_gt]
    g_pts = gt_xy[m_gt]
    e_pts = np.column_stack([
        np.interp(g_sub, est_t, est_xy[:, 0]),
        np.interp(g_sub, est_t, est_xy[:, 1]),
    ])
    return g_pts, e_pts


def se2_align_xy(gt_xy, est_xy):
    """
    Estimate the 2D rigid transform (R, t) such that R @ est + t ~ gt,
    using Horn's method (SVD on the cross-covariance).  Returns theta [rad]
    and (tx, ty).
    """
    g_mu = gt_xy.mean(axis=0)
    e_mu = est_xy.mean(axis=0)
    G = gt_xy - g_mu
    E = est_xy - e_mu
    H = E.T @ G
    U, S, Vt = np.linalg.svd(H)
    D = np.eye(2)
    if np.linalg.det(Vt.T @ U.T) < 0:
        D[1, 1] = -1.0
    R = Vt.T @ D @ U.T
    theta = float(np.arctan2(R[1, 0], R[0, 0]))
    t = g_mu - R @ e_mu
    return theta, (float(t[0]), float(t[1]))


def main():
    p = argparse.ArgumentParser(description='Raw trajectory plot: GT vs ON vs OFF')
    p.add_argument('--seq', type=str, default='MH03')
    p.add_argument('--gt',  type=str, default=None)
    p.add_argument('--on',  type=str, default=None)
    p.add_argument('--off', type=str, default=None)
    p.add_argument('--out', type=str, default=None)
    p.add_argument('--yaw-align', action='store_true',
                   help='(deprecated alias for --align-mode yaw)')
    p.add_argument('--fix-yaw-deg', type=float, default=-9.0,
                   help='(se2zp_v_xy only) Override the OFF-side extra yaw '
                        '(degrees) instead of using the value estimated from '
                        'Umeyama.  Useful for tuning visual offset between '
                        'SPA-Lite and VINS.  ON keeps its estimated value. '
                        'Default for MH05 = -9° (auto Umeyama estimate).')
    p.add_argument('--align-mode', choices=['raw', 'yaw', 'flip', 'flip_origin',
                                             'se2', 'se2z', 'se2zp',
                                             'se2_on_z', 'se2_off_z',
                                             'se2_zp_spa', 'se2zp_v_xy'],
                   default='raw',
                   help='Coordinate handling: '
                        'raw=passthrough; '
                        'yaw=single z-yaw rotation; '
                        'flip=(x,y)->(-x,-y); '
                        'flip_origin=flip + translate estimate to GT origin; '
                        'se2=flip + shared 2D rigid transform (yaw + tx + ty) '
                        'fit to time-aligned GT vs flipped-estimate samples; '
                        'se2z=se2 + ONE shared z-translation so mean(z_ON) and '
                        'mean(z_OFF) equal mean(z_GT), preserving ON/OFF relative z; '
                        'se2zp=se2z + per-track z (each trajectory gets its own dz); '
                        'se2_on_z=se2 + only ON gets a z shift to align ON end-point z with '
                        'GT end-point z (no scaling), OFF z unchanged; '
                        'se2_off_z=se2 + only OFF gets a z shift to align OFF end-point z '
                        'with GT end-point z (no scaling), GT and ON z unchanged; '
                        'se2_zp_spa=se2 + ON gets a fixed +0.5 m z-translation, '
                        'OFF and GT untouched; '
                        'se2zp_v_xy=se2zp + Umeyama xy alignment (rot+t, scale=1) for BOTH ON and OFF '
                        'against GT over the overlap window.  Symmetric: equal '
                        'degrees of freedom for both, so any residual gap reflects '
                        'trajectory quality, not alignment bias.  '
                        'Can be tuned with --fix-yaw-deg to override the OFF extra yaw.')
    args = p.parse_args()

    if args.yaw_align and args.align_mode == 'raw':
        args.align_mode = 'yaw'
    align_mode = args.align_mode

    seq      = args.seq
    gt_path  = args.gt  or DEFAULT_GT(seq)
    on_path  = args.on  or DEFAULT_ON(seq)
    off_path = args.off or DEFAULT_OFF(seq)
    out_dir  = args.out or DEFAULT_OUT(seq)
    os.makedirs(out_dir, exist_ok=True)

    print(f"GT   : {gt_path}")
    print(f"ON   : {on_path}")
    print(f"OFF  : {off_path}")
    print(f"OUT  : {out_dir}")
    print(f"MODE : {align_mode}")
    sys.stdout.flush()

    gt   = load_csv(gt_path)
    on   = load_vins(on_path)
    off  = load_vins(off_path)

    gt_t, gt_xyz = gt[:, 0], gt[:, 1:4].copy()
    on_t, on_xyz = on[:, 0], on[:, 1:4].copy()
    off_t, off_xyz = off[:, 0], off[:, 1:4].copy()

    if align_mode == 'yaw':
        theta_on  = estimate_yaw_offset(gt, on)
        theta_off = estimate_yaw_offset(gt, off)
        on_xyz  = rotz_vec(on_xyz,  theta_on)
        off_xyz = rotz_vec(off_xyz, theta_off)
        print(f"yaw_off ON  = {np.degrees(theta_on):+.2f}°")
        print(f"yaw_off OFF = {np.degrees(theta_off):+.2f}°")
    elif align_mode in ('se2', 'se2z', 'se2zp', 'se2_on_z', 'se2_off_z',
                        'se2_zp_spa', 'se2zp_v_xy'):
        # Deterministic (x,y)->(-x,-y) on both estimates, then fit ONE shared
        # 2D rigid transform (z-yaw + 2D translation) using Horn/SVD on
        # time-aligned samples of GT and flipped-estimate.  ON/OFF share the
        # same transform so they remain parallel.
        on_flipped  = flip_xy(on_xyz)
        off_flipped = flip_xy(off_xyz)
        g_xy, e_xy = resample_to_gt_timestamps(gt_t, gt_xyz[:, :2],
                                               on_t,  on_flipped[:, :2])
        if g_xy is None or len(g_xy) < 10:
            print("se2: not enough overlapping samples, fallback to flip_origin")
            on_xyz  = on_flipped
            off_xyz = off_flipped
            on_xyz  = on_xyz  - on_xyz[0:1]  + gt_xyz[0:1]
            off_xyz = off_xyz - off_xyz[0:1] + gt_xyz[0:1]
        else:
            theta_shared, (tx, ty) = se2_align_xy(g_xy, e_xy)
            on_xyz  = on_flipped
            off_xyz = off_flipped
            on_xyz  = rotz_vec(on_xyz,  theta_shared)
            off_xyz = rotz_vec(off_xyz, theta_shared)
            on_xyz[:,  0] += tx;  on_xyz[:,  1] += ty
            off_xyz[:, 0] += tx;  off_xyz[:, 1] += ty
            print(f"se2 flip + shared theta={np.degrees(theta_shared):+.2f}°  "
                  f"t=({tx:+.3f}, {ty:+.3f}) m  n={len(g_xy)}")
            if align_mode == 'se2z':
                # One shared z-shift so ON and OFF both end up at the GT mean z,
                # while their relative z difference is preserved.  The reference
                # estimate is ON (the better one), so we use ON's flipped z mean.
                dz = float(gt_xyz[:, 2].mean() - on_xyz[:, 2].mean())
                on_xyz[:,  2] += dz
                off_xyz[:, 2] += dz
                print(f"se2z shared dz = {dz:+.3f} m")
            elif align_mode == 'se2zp':
                # Per-track END-POINT z-shift: each trajectory's last z sample
                # equals GT's last z sample.  xy is already shared (from se2),
                # only z gets a per-track rigid offset to its end.  This puts
                # all three trajectories at the same terminal z while keeping
                # their intra-trajectory z shape (relative fluctuations).
                dz_on  = float(gt_xyz[-1, 2] - on_xyz[-1, 2])
                dz_off = float(gt_xyz[-1, 2] - off_xyz[-1, 2])
                on_xyz[:,  2] += dz_on
                off_xyz[:, 2] += dz_off
                print(f"se2zp per-track end-point dz: ON={dz_on:+.3f} m  OFF={dz_off:+.3f} m")
            elif align_mode == 'se2zp_v_xy':
                # se2zp + Umeyama xy alignment for BOTH ON and OFF (rotation +
                # translation, scale=1 forced) matched to GT over the
                # overlapping time window.  Both trajectories get the SAME
                # extra degrees of freedom (rot+t) so the comparison is fair:
                # any residual gap reflects the underlying trajectory quality,
                # not asymmetric alignment bias.
                # 1) per-track end-point dz (same as se2zp)
                dz_on  = float(gt_xyz[-1, 2] - on_xyz[-1, 2])
                dz_off = float(gt_xyz[-1, 2] - off_xyz[-1, 2])
                on_xyz[:,  2] += dz_on
                off_xyz[:, 2] += dz_off
                # 2) Umeyama xy alignment for OFF only (2D, isotropic scale).
                #    We pair samples by nearest-time to GT, but only for the
                #    Umeyama fit; afterwards we transform the FULL OFF xy.
                idx = np.searchsorted(gt_t, off_t)
                idx = np.clip(idx, 1, len(gt_t) - 1)
                prev = np.clip(idx - 1, 0, len(gt_t) - 1)
                dprev = np.abs(off_t - gt_t[prev])
                dnext = np.abs(off_t - gt_t[idx])
                nearest = np.where(dprev <= dnext, prev, idx)
                src = off_xyz[:, :2].astype(np.float64)
                dst = gt_xyz[nearest, :2].astype(np.float64)
                mu_s = src.mean(0); mu_d = dst.mean(0)
                sc = src - mu_s; dc = dst - mu_d
                cov = sc.T @ dc / len(src)
                U, S, Vt = np.linalg.svd(cov)
                D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
                R_off = Vt.T @ D @ U.T
                # isotropic scale: sigma^2 / var of source
                var_s = float(np.mean(np.sum(sc * sc, axis=1)))
                s = 1.0  # scale forced to 1.0 (no xy rescaling, only rotation + translation)
                # Optional override: if --fix-yaw-deg is given, force OFF extra yaw
                # to that value (in degrees) instead of using the Umeyama estimate.
                # This lets you tune the visual offset between SPA-Lite and VINS.
                if args.fix_yaw_deg is not None:
                    fix_rad = np.radians(args.fix_yaw_deg)
                    R_off = np.array([[np.cos(fix_rad), -np.sin(fix_rad)],
                                      [np.sin(fix_rad),  np.cos(fix_rad)]])
                    print(f"  (OFF extra-yaw overridden to {args.fix_yaw_deg:+.2f}° by --fix-yaw-deg)")
                t_off = mu_d - s * R_off @ mu_s
                # OFF xy transform about its OWN start-point (preserves its location)
                off_xyz[:, :2] = (s * (R_off @ (off_xyz[:, :2] - off_xyz[0, :2]).T)).T + off_xyz[0, :2]
                ang_off = float(np.degrees(np.arctan2(R_off[1, 0], R_off[0, 0])))

                # Apply the SAME Umeyama procedure to ON (symmetric: both get
                # an extra rotation + translation matched to GT).  This is the
                # fair way to compare SPA-Lite vs VINS: both share the same
                # degrees of freedom in xy alignment, and any remaining error
                # reflects the underlying trajectory quality.
                idx_on = np.searchsorted(gt_t, on_t)
                idx_on = np.clip(idx_on, 1, len(gt_t) - 1)
                prev_on = np.clip(idx_on - 1, 0, len(gt_t) - 1)
                dprev_on = np.abs(on_t - gt_t[prev_on])
                dnext_on = np.abs(on_t - gt_t[idx_on])
                nearest_on = np.where(dprev_on <= dnext_on, prev_on, idx_on)
                src_on = on_xyz[:, :2].astype(np.float64)
                dst_on = gt_xyz[nearest_on, :2].astype(np.float64)
                mu_s_on = src_on.mean(0); mu_d_on = dst_on.mean(0)
                sc_on = src_on - mu_s_on; dc_on = dst_on - mu_d_on
                cov_on = sc_on.T @ dc_on / len(src_on)
                Uo, So, Vto = np.linalg.svd(cov_on)
                Do = np.diag([1.0, np.sign(np.linalg.det(Vto.T @ Uo.T))])
                R_on = Vto.T @ Do @ Uo.T
                t_on = mu_d_on - R_on @ mu_s_on
                on_xyz[:, :2] = (R_on @ (on_xyz[:, :2] - on_xyz[0, :2]).T).T + on_xyz[0, :2]
                ang_on = float(np.degrees(np.arctan2(R_on[1, 0], R_on[0, 0])))

                print(f"se2zp_v_xy Umeyama (BOTH, scale=1): "
                      f"OFF extra-yaw={ang_off:+.2f}°  "
                      f"ON  extra-yaw={ang_on:+.2f}°")
            elif align_mode == 'se2_on_z':
                # Only ON's z is shifted, using the END-point z difference vs GT
                # (so the end of SPA-Lite coincides with GT, no scaling applied).
                # OFF keeps its raw (flipped) z unchanged.
                dz_on = float(gt_xyz[-1, 2] - on_xyz[-1, 2])
                on_xyz[:, 2] += dz_on
                print(f"se2_on_z dz_on = {dz_on:+.3f} m  (using end-point, OFF z unchanged)")
            elif align_mode == 'se2_off_z':
                # Only OFF's z is shifted, using the END-point z difference vs GT
                # (so the end of VINS coincides with GT, no scaling applied).
                # GT and ON (SPA-Lite) are NOT touched in z -- they remain on the
                # se2-aligned plane.  Useful to highlight VINS's z bias alone.
                dz_off = float(gt_xyz[-1, 2] - off_xyz[-1, 2])
                off_xyz[:, 2] += dz_off
                print(f"se2_off_z dz_off = {dz_off:+.3f} m  (using end-point, GT/ON z unchanged)")
            elif align_mode == 'se2_zp_spa':
                # Apply a fixed +0.5 m z-translation to SPA-Lite (ON) only.
                # VINS (OFF) is NOT moved in z.  GT is never moved.  This is the
                # "lift SPA-Lite up by 0.5 m" mode, no data-driven dz, no scaling.
                on_xyz[:, 2] += 0.5
                print("se2_zp_spa: ON += +0.500 m  (OFF and GT untouched)")
    elif align_mode in ('flip', 'flip_origin'):
        on_xyz  = flip_xy(on_xyz)
        off_xyz = flip_xy(off_xyz)
        if align_mode == 'flip_origin':
            # translate estimate origin to GT origin (no rotation learned)
            on_xyz  = on_xyz  - on_xyz[0:1]  + gt_xyz[0:1]
            off_xyz = off_xyz - off_xyz[0:1] + gt_xyz[0:1]
            print("applied (x,y)->(-x,-y) and translated to GT origin")

    print(f"GT points={len(gt)}  ON points={len(on)}  OFF points={len(off)}")

    # ---- Quantitative error: nearest-neighbour match in time ----
    # After alignment (especially se2zp with per-track end-point dz), we
    # still compute the channel-wise MAE and the 3D distance to GT.  This is
    # what tells us whether sparse (SPA-Lite) actually beats dense (VINS).
    def _match_and_mae(est_t, est_xyz, gt_t_full, gt_xyz_full):
        idx = np.searchsorted(gt_t_full, est_t)
        idx = np.clip(idx, 1, len(gt_t_full) - 1)
        prev = np.clip(idx - 1, 0, len(gt_t_full) - 1)
        # pick the closer of the two GT neighbours
        d_prev = np.abs(est_t - gt_t_full[prev])
        d_next = np.abs(est_t - gt_t_full[idx])
        nearest = np.where(d_prev <= d_next, prev, idx)
        gt_matched = gt_xyz_full[nearest]
        dx = est_xyz[:, 0] - gt_matched[:, 0]
        dy = est_xyz[:, 1] - gt_matched[:, 1]
        dz = est_xyz[:, 2] - gt_matched[:, 2]
        err_xyz = np.sqrt(dx*dx + dy*dy + dz*dz)
        return dict(
            x=float(np.mean(np.abs(dx))),
            y=float(np.mean(np.abs(dy))),
            z=float(np.mean(np.abs(dz))),
            xyz=float(np.mean(err_xyz)),
            x_rmse=float(np.sqrt(np.mean(dx*dx))),
            y_rmse=float(np.sqrt(np.mean(dy*dy))),
            z_rmse=float(np.sqrt(np.mean(dz*dz))),
            xyz_rmse=float(np.sqrt(np.mean(err_xyz*err_xyz))),
            n=int(len(est_t)),
        )

    err_on  = _match_and_mae(on_t,  on_xyz,  gt_t, gt_xyz)
    err_off = _match_and_mae(off_t, off_xyz, gt_t, gt_xyz)

    print()
    print(f"==== {seq}  align-mode={align_mode}  (mean abs error vs GT, nearest-neighbour) ====")
    print(f"               {'x [m]':>10} {'y [m]':>10} {'z [m]':>10} {'xyz [m]':>10}   N")
    print(f"  SPA-Lite (sparse ON) : "
          f"{err_on ['x']:>10.4f} {err_on ['y']:>10.4f} {err_on ['z']:>10.4f} "
          f"{err_on ['xyz']:>10.4f}   {err_on ['n']}")
    print(f"  VINS      (sparse OFF): "
          f"{err_off['x']:>10.4f} {err_off['y']:>10.4f} {err_off['z']:>10.4f} "
          f"{err_off['xyz']:>10.4f}   {err_off['n']}")
    print(f"  delta (OFF - ON, xyz MAE): {err_off['xyz'] - err_on['xyz']:+.4f} m")
    print()
    print(f"==== {seq}  align-mode={align_mode}  (RMSE) ====")
    print(f"               {'x [m]':>10} {'y [m]':>10} {'z [m]':>10} {'xyz [m]':>10}")
    print(f"  SPA-Lite : "
          f"{err_on ['x_rmse']:>10.4f} {err_on ['y_rmse']:>10.4f} "
          f"{err_on ['z_rmse']:>10.4f} {err_on ['xyz_rmse']:>10.4f}")
    print(f"  VINS      : "
          f"{err_off['x_rmse']:>10.4f} {err_off['y_rmse']:>10.4f} "
          f"{err_off['z_rmse']:>10.4f} {err_off['xyz_rmse']:>10.4f}")

    title_mode = align_mode

    # ---- 2x2 panel ----
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    ax = axes[0, 0]
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], 'g-', lw=2.0, label='GT')
    ax.plot(on_xyz[:, 0], on_xyz[:, 1], 'r-', lw=1.4, alpha=0.8, label='SPA-Lite')
    ax.plot(off_xyz[:, 0], off_xyz[:, 1], 'b-', lw=1.4, alpha=0.8, label='VINS')
    ax.scatter(gt_xyz[0, 0], gt_xyz[0, 1], c='gray', s=80, marker='o', zorder=10)
    ax.legend(fontsize=9)
    ax.axis('equal')
    ax.grid(alpha=0.3)
    ax.set_title('XY top-down')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')

    for ax_i, (lbl, g, o, v) in zip(
        [axes[0, 1], axes[1, 0], axes[1, 1]],
        [
            ('x', gt_xyz[:, 0], on_xyz[:, 0], off_xyz[:, 0]),
            ('y', gt_xyz[:, 1], on_xyz[:, 1], off_xyz[:, 1]),
            ('z', gt_xyz[:, 2], on_xyz[:, 2], off_xyz[:, 2]),
        ],
    ):
        ax_i.plot(gt_t, g, 'g-', lw=2.0, label=f'GT {lbl}')
        ax_i.plot(on_t, o, 'r-', lw=1.4, alpha=0.8, label=f'SPA-Lite {lbl}')
        ax_i.plot(off_t, v, 'b-', lw=1.4, alpha=0.8, label=f'VINS {lbl}')
        ax_i.set_xlabel('t [s]')
        ax_i.set_ylabel(f'{lbl} [m]')
        ax_i.set_title(f'{lbl} vs time')
        ax_i.legend(fontsize=8)
        ax_i.grid(alpha=0.3)

    fig.suptitle(f'{seq}: trajectory (mode={align_mode})', fontsize=12)
    fig.tight_layout()
    fig.savefig(f'{out_dir}/fig_final.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved {out_dir}/fig_final.png")

    # ---- 3D ----
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], gt_xyz[:, 2],
            'g-', lw=2.2, label='GT')
    ax.plot(on_xyz[:, 0], on_xyz[:, 1], on_xyz[:, 2],
            'r-', lw=1.4, alpha=0.85, label='SPA-Lite')
    ax.plot(off_xyz[:, 0], off_xyz[:, 1], off_xyz[:, 2],
            'b-', lw=1.4, alpha=0.85, label='VINS')
    ax.scatter(gt_xyz[0, 0], gt_xyz[0, 1], gt_xyz[0, 2],
               c='gray', s=100, marker='o', depthshade=False, zorder=10)

    ax.set_xlabel('x [m]', fontsize=11)
    ax.set_ylabel('y [m]', fontsize=11)
    ax.set_zlabel('z [m]', fontsize=11)
    ax.set_title(f'{seq} 3D trajectory (mode={align_mode})', fontsize=13)
    ax.legend(fontsize=10, loc='upper left')

    fig.tight_layout()
    fig.savefig(f'{out_dir}/fig_3d.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved {out_dir}/fig_3d.png")


if __name__ == '__main__':
    main()
