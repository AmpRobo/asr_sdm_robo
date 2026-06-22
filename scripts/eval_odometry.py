#!/usr/bin/env python3
"""
W2.5 — 评估 baseline vs D2 在 D435i 慢 bag 上的差异。

指标：
  1) rel_err     : 相对位姿误差 (RPE) per 1m 运动单位
  2) reboots     : 速度突增帧数（> max_trans_per_frame）
  3) cpu_ms      : front-end 单帧耗时（从 log 的 KLT_STATS 抽）
  4) sparse_%    : sparse 对齐成功率（从 log 的 SPARSE_STATS 抽）

CSV 格式 (VINS 内部产物)：
  vins_result_no_loop.csv :
    timestamp_sec, x, y, z, qw, qx, qy, qz, vx, vy, vz
  vins_result_loop.csv    :
    timestamp_ns, x, y, z, qw, qx, qy, qz

输入：
  --baseline-dir DIR       含 vins_result_no_loop.csv + .log
  --d2-dir       DIR       同上
  --max-trans    FLOAT     reboot 阈值 (m/s)，默认 2.0
  --out-csv      PATH      输出表格路径
"""

import argparse
import csv
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Trajectory:
    t: np.ndarray            # (N,)  seconds
    p: np.ndarray            # (N, 3) meters
    q: np.ndarray            # (N, 4)  qw, qx, qy, qz


def load_vins_csv(path: str) -> Trajectory:
    """Load a VINS vins_result_no_loop.csv (sec stamp) or _loop.csv (ns stamp).

    The vins_result_no_loop.csv in this codebase is written with
    `precision(0)` for the timestamp field, so the sec column is actually
    integer seconds (e.g. "1780099576").  Adjacent rows therefore have dt=0
    and instantaneous speeds blow up to infinity.  We detect this and fall
    back to the known VINS output rate (~10 Hz) for dt purposes.
    """
    rows = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 8:
                continue
            try:
                ts = float(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                qw, qx, qy, qz = (
                    float(parts[4]), float(parts[5]),
                    float(parts[6]), float(parts[7]),
                )
            except ValueError:
                continue
            rows.append((ts, x, y, z, qw, qx, qy, qz))
    if not rows:
        raise RuntimeError(f"empty / unparseable: {path}")
    arr = np.array(rows, dtype=np.float64)
    t_raw = arr[:, 0]
    if t_raw.max() > 1e12:
        t = t_raw * 1e-9
    else:
        t = t_raw
    # Heuristic: if too many duplicate timestamps, treat as constant-rate 10 Hz
    dt = np.diff(t)
    n_dup = int(np.sum(dt == 0))
    if n_dup > 0.5 * len(dt):
        # Rebuild time as 0.1 s spacing from first recorded timestamp
        t = np.arange(len(t)) * 0.1 + (t[0] if len(t) else 0.0)
    return Trajectory(
        t=t,
        p=arr[:, 1:4],
        q=arr[:, 4:8],
    )


def quat_to_R(q: np.ndarray) -> np.ndarray:
    """(4,) qw,qx,qy,qz -> (3,3) R."""
    qw, qx, qy, qz = q
    n = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5
    if n < 1e-9:
        return np.eye(3)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def rpe_per_meter(traj: Trajectory, step_m: float = 1.0) -> float:
    """Per-meter heading drift of the trajectory (no-GT proxy for RPE).

    Walks the trajectory in `step_m` increments along path length, and at
    each sample compares the segment direction with the *start* direction.
    A perfectly straight trajectory gives 0; a curve gives a positive
    drift in m/m (averaged).

    For the D435i slow bag (no GT available), this is the only meaningful
    RPE-style metric.  An "ideal" estimate yields rpe == 0; an estimator
    that frequently resets produces large rpe.
    """
    if len(traj.p) < 2:
        return float("nan")
    dp = np.diff(traj.p, axis=0)
    seg_len = np.linalg.norm(dp, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum[-1]
    if total < step_m * 2:
        return float("nan")
    dir_start = dp[0] / max(seg_len[0], 1e-9)
    samples = np.arange(step_m, total, step_m)
    j = 0
    acc = 0.0
    errs = []
    for s in samples:
        # advance j so cum[j] >= s
        target = s
        while j < len(cum) - 1 and cum[j + 1] < target:
            j += 1
        if j + 1 >= len(traj.p):
            break
        seg = traj.p[j + 1] - traj.p[0]
        seg_d = np.linalg.norm(seg)
        if seg_d < 1e-6:
            continue
        dir_seg = seg / seg_d
        # per-meter drift over the first `s` meters
        errs.append(np.linalg.norm(dir_seg - dir_start) * step_m)
    if not errs:
        return float("nan")
    arr = np.array(errs, dtype=np.float64)
    return float(np.mean(arr))


def total_drift(traj: Trajectory) -> float:
    """Return (total_path_length, net_displacement, drift_ratio) of the trajectory."""
    if len(traj.p) < 2:
        return (0.0, 0.0, 0.0)
    dp = np.diff(traj.p, axis=0)
    seg_len = np.linalg.norm(dp, axis=1)
    total_path = float(np.sum(seg_len))
    net_disp = float(np.linalg.norm(traj.p[-1] - traj.p[0]))
    drift = (total_path - net_disp) / max(total_path, 1e-9)
    return (total_path, net_disp, drift)


def reboot_count(traj: Trajectory, max_speed: float = 2.0) -> int:
    """Number of frames whose instantaneous speed > max_speed (m/s).

    The D435i slow bag (~0.4 m/s peak) is well below 2 m/s, so any speed spike
    above 2 m/s in a slow recording is a VINS estimator reset / pose jump.

    NOTE: vins_result_no_loop.csv writes timestamps with precision(0), so all
    timestamps are integer seconds and the per-frame dt is unreliable.
    We use a fixed `dt` derived from the bag duration / frame count instead.
    """
    if len(traj.p) < 2 or len(traj.t) < 2:
        return 0
    if traj.t[-1] > traj.t[0] and (traj.t[-1] - traj.t[0]) > 1.0:
        dt = float((traj.t[-1] - traj.t[0]) / (len(traj.p) - 1))
    else:
        dt = 0.1  # 10 Hz fallback
    dp = np.diff(traj.p, axis=0)
    speed = np.linalg.norm(dp, axis=1) / max(dt, 1e-6)
    return int(np.sum(speed > max_speed))


def max_speed(traj: Trajectory) -> float:
    if len(traj.p) < 2:
        return 0.0
    if traj.t[-1] > traj.t[0] and (traj.t[-1] - traj.t[0]) > 1.0:
        dt = float((traj.t[-1] - traj.t[0]) / (len(traj.p) - 1))
    else:
        dt = 0.1
    dp = np.diff(traj.p, axis=0)
    return float(np.max(np.linalg.norm(dp, axis=1) / max(dt, 1e-6)))


def parse_klt_stats(log_path: str) -> Optional[float]:
    """Last [KLT_STATS] line — return mean_cost in ms."""
    last = None
    pat = re.compile(r"\[KLT_STATS\]\s*frames=\d+\s+mean_cost=([\d.]+)ms")
    with open(log_path, "r") as f:
        for line in f:
            m = pat.search(line)
            if m:
                last = float(m.group(1))
    return last


def parse_sparse_stats(log_path: str) -> Tuple[Optional[float], Optional[float]]:
    """Return (last mean_chi2, last success_rate %) from [SPARSE_STATS]."""
    last_chi2 = None
    last_succ = None
    pat = re.compile(
        r"\[SPARSE_STATS\].*success_rate=([\d.]+)%.*mean_chi2=([\d.]+)"
    )
    with open(log_path, "r") as f:
        for line in f:
            m = pat.search(line)
            if m:
                last_succ = float(m.group(1))
                last_chi2 = float(m.group(2))
    return last_chi2, last_succ


def parse_d2_stat_angle(log_path: str) -> Optional[float]:
    """Mean of [D2_STAT] sparse_vs_IMU_angle values (deg)."""
    vals = []
    pat = re.compile(r"\[D2_STAT\][^\n]*sparse_vs_IMU_angle=([\d.]+)")
    with open(log_path, "r") as f:
        for line in f:
            m = pat.search(line)
            if m:
                vals.append(float(m.group(1)))
    if not vals:
        return None
    return float(np.mean(vals))


def find_log(out_dir: str) -> Optional[str]:
    """Look for any .log file in out_dir (ros2 launch redirect)."""
    p = Path(out_dir)
    for cand in p.glob("*.log"):
        return str(cand)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline-dir", required=True)
    ap.add_argument("--d2-dir", required=True)
    ap.add_argument("--max-trans", type=float, default=5.0)
    ap.add_argument("--bag-name", default="d435i_01")
    ap.add_argument("--out-csv", required=True)
    args = ap.parse_args()

    results = []
    for label, d in [("VINS-orig", args.baseline_dir), ("D2", args.d2_dir)]:
        no_loop = os.path.join(d, "vins_result_no_loop.csv")
        if not os.path.isfile(no_loop):
            print(f"[WARN] missing {no_loop}", file=sys.stderr)
            continue
        traj = load_vins_csv(no_loop)
        rel_err = rpe_per_meter(traj, step_m=1.0)
        reboots = reboot_count(traj, max_speed=args.max_trans)
        peak_v = max_speed(traj)
        path_len, net_disp, drift = total_drift(traj)
        cpu_ms = sparse_succ = sparse_chi2 = sparse_imu_angle = None
        log_path = find_log(d)
        if log_path:
            cpu_ms = parse_klt_stats(log_path)
            sparse_chi2, sparse_succ = parse_sparse_stats(log_path)
            sparse_imu_angle = parse_d2_stat_angle(log_path)
        results.append({
            "bag": args.bag_name,
            "method": label,
            "frames": len(traj.p),
            "duration_s": float(traj.t[-1] - traj.t[0]) if len(traj.t) > 1 else 0.0,
            "path_length_m": path_len,
            "net_displacement_m": net_disp,
            "drift_ratio": drift,
            "rpe_mpm": rel_err,
            "reboots": reboots,
            "max_speed_mps": peak_v,
            "klt_cpu_ms": cpu_ms if cpu_ms is not None else "",
            "sparse_succ_pct": sparse_succ if sparse_succ is not None else "",
            "sparse_mean_chi2": sparse_chi2 if sparse_chi2 is not None else "",
            "sparse_vs_imu_angle_deg": (
                sparse_imu_angle if sparse_imu_angle is not None else ""
            ),
        })

    if not results:
        print("[ERROR] no results to write", file=sys.stderr)
        sys.exit(1)

    fieldnames = list(results[0].keys())
    os.makedirs(os.path.dirname(os.path.abspath(args.out_csv)), exist_ok=True)
    with open(args.out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(results)
    print(f"wrote {args.out_csv}  ({len(results)} rows)")

    # Stdout summary
    print("\n=== W2 summary ===")
    for r in results:
        print(
            f"  {r['method']:8s}  frames={r['frames']:5d}  "
            f"duration={r['duration_s']:6.1f}s  "
            f"path={r['path_length_m']:6.2f}m  "
            f"net={r['net_displacement_m']:5.2f}m  "
            f"drift={r['drift_ratio']:.3f}  "
            f"rpe={r['rpe_mpm']!r}  reboots={r['reboots']}  "
            f"peak_v={r['max_speed_mps']:.2f}m/s  "
            f"klt={r['klt_cpu_ms']}ms  sparse_succ={r['sparse_succ_pct']}"
        )


if __name__ == "__main__":
    main()
