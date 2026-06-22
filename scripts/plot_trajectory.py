#!/usr/bin/env python3
"""
W2.6 — 出图 (XY 轨迹对比 + per-frame speed)。

用法：
  python3 scripts/plot_trajectory.py \
      --baseline /home/lxy/output/baseline/vins_result_no_loop.csv \
      --d2       /home/lxy/output/d2/vins_result_no_loop.csv \
      --out      results/week2_trajectory.png
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from eval_odometry import load_vins_csv


def plot_xy(b, d2, out_path: str) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))

    # --- left: XY trajectory ---
    ax = axes[0]
    ax.plot(b.p[:, 0], b.p[:, 1], "-", color="#1f77b4", lw=1.0,
            label=f"VINS-orig (path={np.sum(np.linalg.norm(np.diff(b.p, axis=0), axis=1)):.1f} m)")
    ax.plot(d2.p[:, 0], d2.p[:, 1], "-", color="#d62728", lw=1.0, alpha=0.9,
            label=f"D2 (path={np.sum(np.linalg.norm(np.diff(d2.p, axis=0), axis=1)):.1f} m)")
    ax.scatter([b.p[0, 0]], [b.p[0, 1]], c="k", marker="o", s=40, zorder=5,
               label="start")
    ax.scatter([b.p[-1, 0]], [b.p[-1, 1]], c="b", marker="x", s=60, zorder=5,
               label="VINS end")
    ax.scatter([d2.p[-1, 0]], [d2.p[-1, 1]], c="r", marker="x", s=60, zorder=5,
               label="D2 end")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("D435i slow bag — XY trajectory")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")

    # --- right: per-frame speed ---
    ax2 = axes[1]

    def speeds(traj):
        dp = np.diff(traj.p, axis=0)
        if traj.t[-1] > traj.t[0] and (traj.t[-1] - traj.t[0]) > 1.0:
            dt = float((traj.t[-1] - traj.t[0]) / (len(traj.p) - 1))
        else:
            dt = 0.1
        return np.linalg.norm(dp, axis=1) / max(dt, 1e-6)

    sp_b = speeds(b)
    sp_d = speeds(d2)
    ax2.plot(sp_b, color="#1f77b4", lw=0.6, alpha=0.85, label="VINS-orig")
    ax2.plot(sp_d, color="#d62728", lw=0.6, alpha=0.85, label="D2")
    ax2.set_xlabel("frame index")
    ax2.set_ylabel("speed [m/s]")
    ax2.set_title("instantaneous speed")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="best", fontsize=9)

    fig.suptitle("W2 — D435i slow bag, baseline vs D2 (sparse align)", y=1.02)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", required=True)
    ap.add_argument("--d2", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    b = load_vins_csv(args.baseline)
    d = load_vins_csv(args.d2)
    plot_xy(b, d, args.out)


if __name__ == "__main__":
    main()
