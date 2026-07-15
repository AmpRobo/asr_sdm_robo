"""Sparse ON vs OFF comparison: focus on shape and drift.

Inputs:
  GT:                experiments/sparse_compare/results/mh03_gt.csv
  sparse_off_path:   experiments/sparse_compare/results/mh03_sparse_off_path.csv
  sparse_on_path:    experiments/sparse_compare/results/mh03_sparse_on_path.csv

Alignment: 'first-umeyama' — similarity transform (s*R, t) anchored at GT[0],
so all three curves start from the same point. Remaining gaps are pure
local drift.

NOTE: The two `path` files are the raw /vins/path topic output, which is
emitted BEFORE loop-closure correction in the VINS pipeline. So this
comparison reflects the VIO drift + sparse-align effect, NOT post-loop
correction.
"""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_path_csv(p):
    with open(p, "r") as f:
        first = f.readline()
    has_header = not any(ch.isdigit() for ch in first.split(",")[0].strip())
    skip = 1 if has_header else 0
    try:
        arr = np.loadtxt(p, delimiter=",", skiprows=skip, usecols=(0, 1, 2, 3))
    except Exception:
        arr = np.loadtxt(p, delimiter=None, skiprows=skip, usecols=(0, 1, 2, 3))
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    arr = arr[:, :4]
    if arr.shape[0] > 0 and abs(arr[0, 0]) > 1e15:
        arr[:, 0] = arr[:, 0] / 1e9
    # Dedup keeping LAST write per timestamp, and sort by time ASC
    _, inv = np.unique(arr[:, 0], return_inverse=True)
    out = np.zeros((inv.max() + 1, 4))
    last_idx = {}
    for i, k in enumerate(inv):
        last_idx[k] = i
    for k, i in last_idx.items():
        out[k] = arr[i]
    out = out[np.argsort(out[:, 0])]
    return out


def umeyama(src, dst):
    n = len(src)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sigma = (dst - mu_d).T @ (src - mu_s) / n
    U, D, Vt = np.linalg.svd(sigma)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var = ((src - mu_s) ** 2).sum() / n
    s = (D * np.diag(S)).sum() / var
    t = mu_d - s * R @ mu_s
    return s, R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--off", required=True)
    ap.add_argument("--on",  required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---- Load ----
    gt = np.loadtxt(args.gt, delimiter=None, skiprows=1)
    if gt.ndim == 1: gt = gt.reshape(1, -1)
    gt_t = gt[:, 0] - gt[0, 0]
    gt_xyz = gt[:, 1:4]

    off = load_path_csv(args.off)
    on  = load_path_csv(args.on)

    # Rebase VINS to GT origin (bag-start sync)
    off[:, 0] = (off[:, 0] - off[0, 0]) + gt_t[0]
    on [:, 0] = (on [:, 0] - on [0, 0]) + gt_t[0]

    # Common time window
    t_lo = max(gt_t[0], off[0, 0], on[0, 0])
    t_hi = min(gt_t[-1], off[-1, 0], on[-1, 0])
    gt_t_c  = gt_t[(gt_t >= t_lo) & (gt_t <= t_hi)] - t_lo
    gt_c    = gt_xyz[(gt_t >= t_lo) & (gt_t <= t_hi)]
    def crop(a, lo, hi):
        m = (a[:, 0] >= lo) & (a[:, 0] <= hi)
        return a[m]
    off_c = crop(off, t_lo, t_hi); off_c[:, 0] -= t_lo
    on_c  = crop(on , t_lo, t_hi); on_c [:, 0] -= t_lo
    print(f"common window: [{t_lo:.1f}, {t_hi:.1f}]s = {t_hi-t_lo:.1f}s")

    # ---- First-umeyama alignment (R + scale, t anchored at GT[0]) ----
    def align(e):
        # nearest GT sample for each est sample, used for Umeyama
        idx = np.searchsorted(gt_t_c, e[:, 0])
        idx = np.clip(idx, 1, len(gt_t_c) - 1)
        left = idx - 1
        choose_left = (gt_t_c[left] - e[:, 0]) <= (e[:, 0] - gt_t_c[idx])
        nearest = np.where(choose_left, left, idx)
        s, R, t_vec = umeyama(e[:, 1:4], gt_c[nearest])
        e_rot = (s * (R @ e[:, 1:4].T)).T
        t_anchor = gt_c[0] - e_rot[0]
        return e_rot + t_anchor, s

    off_a, s_off = align(off_c)
    on_a , s_on  = align(on_c)

    # ---- Metrics ----
    err_off = np.linalg.norm(off_a - gt_c[:len(off_a)], axis=1) if len(off_a) < len(gt_c) \
              else np.linalg.norm(off_a[:len(gt_c)] - gt_c, axis=1)
    # resample to GT timestamps for fair RMSE
    def resample(a, gt_t):
        idx = np.searchsorted(a[:, 0], gt_t)
        idx = np.clip(idx, 1, len(a) - 1)
        left = idx - 1
        choose_left = (gt_t - a[left, 0]) <= (a[idx, 0] - gt_t)
        nearest = np.where(choose_left, left, idx)
        return a[nearest, 1:4]
    off_g = resample(off_c, gt_t_c)
    on_g  = resample(on_c , gt_t_c)
    rmse_off = np.sqrt(((off_g - gt_c) ** 2).sum(axis=1).mean())
    rmse_on  = np.sqrt(((on_g  - gt_c) ** 2).sum(axis=1).mean())
    # back-to-origin in aligned frame (each ends where its data ends)
    d_gt    = np.linalg.norm(gt_c[-1]    - gt_c[0])
    d_off   = np.linalg.norm(off_a[-1]   - off_a[0])
    d_on    = np.linalg.norm(on_a [-1]   - on_a [0])

    print(f"\nAlignment: first-umeyama")
    print(f"  scale: sparse_OFF={s_off:.4f}  sparse_ON={s_on:.4f}")
    print(f"  back-to-origin (aligned frame):")
    print(f"    GT           {d_gt:.3f}m")
    print(f"    sparse OFF   {d_off:.3f}m")
    print(f"    sparse ON    {d_on :.3f}m")
    print(f"  RMSE vs GT:")
    print(f"    sparse OFF   {rmse_off:.3f}m")
    print(f"    sparse ON    {rmse_on :.3f}m")

    # ===== x-t, y-t, z-t =====
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for ax, k, name in zip(axes, [0, 1, 2], ["x", "y", "z"]):
        ax.plot(gt_t_c, gt_c[:, k], "k-", lw=2.0,
                label=f"Ground truth  back={d_gt:.3f}m")
        ax.plot(off_c[:, 0], off_a[:, k], color="#2ca02c", lw=1.0, alpha=0.8,
                label=f"sparse OFF  back={d_off:.3f}m  RMSE={rmse_off:.3f}m")
        ax.plot(on_c [:, 0], on_a [:, k], color="#d62728", lw=1.0, alpha=0.8,
                label=f"sparse ON   back={d_on :.3f}m  RMSE={rmse_on :.3f}m")
        ax.set_ylabel(f"{name} [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=9)
    axes[-1].set_xlabel("t [s]")
    axes[0].set_title(
        f"MH_03: sparse_align ON vs OFF  (first-umeyama aligned to GT)\n"
        f"back-to-origin  GT={d_gt:.3f}m  off={d_off:.3f}m  on={d_on:.3f}m  |  "
        f"RMSE  off={rmse_off:.3f}m  on={rmse_on:.3f}m",
        fontsize=11)
    fig.tight_layout()
    fig.savefig(out_dir / "fig_xt_yt_zt.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out_dir/'fig_xt_yt_zt.png'}")

    # ===== 2D top-down =====
    fig, ax = plt.subplots(figsize=(11, 9))
    ax.plot(gt_c[:, 0], gt_c[:, 1], "k-", lw=2.0,
            label=f"Ground truth  back={d_gt:.3f}m")
    ax.plot(off_a[:, 0], off_a[:, 1], color="#2ca02c", lw=1.4,
            label=f"sparse OFF (no loop fix)  back={d_off:.3f}m  RMSE={rmse_off:.3f}m")
    ax.plot(on_a [:, 0], on_a [:, 1], color="#d62728", lw=1.4, alpha=0.85,
            label=f"sparse ON  (no loop fix)  back={d_on :.3f}m  RMSE={rmse_on :.3f}m")

    # Start (GT[0])
    ax.scatter(gt_c[0, 0], gt_c[0, 1], c="gray", s=80, marker="o",
               zorder=10, label="start (GT[0])")
    # End points
    ax.scatter(gt_c[-1, 0],   gt_c[-1, 1],     c="k", s=200, marker="*", zorder=10)
    ax.scatter(off_a[-1, 0],  off_a[-1, 1],    c="#2ca02c", s=200, marker="*", zorder=10)
    ax.scatter(on_a [-1, 0],  on_a [-1, 1],    c="#d62728", s=200, marker="*", zorder=10)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        f"MH_03: 2D top-down  (first-umeyama aligned to GT)\n"
        f"back-to-origin  GT={d_gt:.3f}m  off={d_off:.3f}m  on={d_on:.3f}m  |  "
        f"RMSE  off={rmse_off:.3f}m  on={rmse_on:.3f}m",
        fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    ax.axis("equal")
    fig.tight_layout()
    fig.savefig(out_dir / "fig_xy_topdown.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out_dir/'fig_xy_topdown.png'}")

    # ===== 3D =====
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(gt_c[:, 0], gt_c[:, 1], gt_c[:, 2], "k-", lw=2.0,
            label=f"Ground truth  back={d_gt:.3f}m")
    ax.plot(off_a[:, 0], off_a[:, 1], off_a[:, 2], color="#2ca02c", lw=1.2,
            label=f"sparse OFF  back={d_off:.3f}m")
    ax.plot(on_a [:, 0], on_a [:, 1], on_a [:, 2], color="#d62728", lw=1.0, alpha=0.85,
            label=f"sparse ON   back={d_on :.3f}m")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(f"MH_03: 3D trajectory  (first-umeyama aligned to GT)")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_dir / "fig_xyz.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out_dir/'fig_xyz.png'}")


if __name__ == "__main__":
    main()