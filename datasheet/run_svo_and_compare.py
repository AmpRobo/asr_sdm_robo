#!/usr/bin/env python3
"""
SVO 视觉里程计姿态 vs 真值 (Rig) 对比分析。

关键：录制时使用 --clock，使 /Rig 和 /svo/pose 在同一时间域：
  - /Rig: 原始 bag 时间 (2013年)
  - /svo/pose: 相对时间 (bag播放开始 = t=0)
  通过 /Rig 的 bag_time - wc_time 计算时间偏移

Usage:
    source /opt/ros/jazzy/setup.bash
    source ~/asr_sdm_robo/install/setup.bash
    python3 run_svo_and_compare.py \
        --bag /path/to/bag \
        --recording /path/to/recording \
        --output ./output
"""
import argparse
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R_scipy
import json

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
import rclpy.serialization


# ─── 数据读取 ──────────────────────────────────────────────────

def read_recording_with_offset(recording_path):
    """
    读取录制 bag (/Rig + /svo/pose)，
    通过 /Rig 的 bag time 计算时间偏移，
    返回 SVO 数据 (转换到 bag time) 和 Rig 数据 (原始 bag time)。
    """
    storage_opts = StorageOptions(uri=recording_path, storage_id="")
    converter_opts = ConverterOptions("cdr", "cdr")
    reader = SequentialReader()
    reader.open(storage_opts, converter_opts)

    PoseStamped_cls = get_message("geometry_msgs/msg/PoseStamped")
    PoseWC_cls = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")

    rig_raw = []   # (wc_ts, bag_ts, pos, quat)
    svo_raw = []   # (wc_ts, rel_ts, pos, quat)

    while reader.has_next():
        topic, data, t = reader.read_next()
        wc = t * 1e-9
        if topic == "/Rig":
            msg = rclpy.serialization.deserialize_message(data, PoseStamped_cls)
            bag = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rig_raw.append((wc, bag,
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                [msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w]))
        elif topic == "/svo/pose":
            msg = rclpy.serialization.deserialize_message(data, PoseWC_cls)
            rel = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            svo_raw.append((wc, rel,
                [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))

    if not rig_raw:
        return None, None, None

    # 计算时间偏移
    # bag_time = wc_time - offset, 其中 offset = first_rig.wc - first_rig.bag
    rig_raw.sort(key=lambda x: x[0])
    svo_raw.sort(key=lambda x: x[0])
    first_rig = rig_raw[0]
    offset = first_rig[0] - first_rig[1]   # wc - bag = 415444865.141

    print(f"  时间偏移 offset (wc-bag): {offset:.3f} s")
    print(f"  第一个 /Rig: wc={first_rig[0]:.3f}s, bag={first_rig[1]:.3f}s")
    print(f"  第一个 /svo: rel={svo_raw[0][1]:.3f}s, wc={svo_raw[0][0]:.3f}s")
    print(f"  验证: wc-offset = {svo_raw[0][0]-offset:.3f}s (应接近 first_rig.bag={first_rig[1]:.3f}s)")

    # 转换 SVO 时间到 bag 时间: bag_time = wc_time - offset
    rig_data = [{"stamp": r[1], "position": r[2], "orientation": r[3]} for r in rig_raw]
    svo_data = [{"stamp": r[0] - offset, "position": r[2], "orientation": r[3]} for r in svo_raw]

    return rig_data, svo_data, offset


def quaternion_to_euler(q):
    x, y, z, w = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp = np.clip(2*(w*y - z*x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


# ─── 对齐与对比 ──────────────────────────────────────────────

def align_and_compare(rig_data, svo_data):
    if not rig_data or not svo_data:
        print("  数据不足"); return None

    rig_ts = np.array([m["stamp"] for m in rig_data])
    svo_ts = np.array([m["stamp"] for m in svo_data])

    t0 = max(rig_ts[0], svo_ts[0])
    t1 = min(rig_ts[-1], svo_ts[-1])
    overlap = t1 - t0
    print(f"\n  SVO: {len(svo_data)} poses, {svo_ts[0]:.2f} ~ {svo_ts[-1]:.2f} s (bag time)")
    print(f"  Rig: {len(rig_data)} poses, {rig_ts[0]:.2f} ~ {rig_ts[-1]:.2f} s (bag time)")
    print(f"  重叠时长: {overlap:.1f} s ({overlap/60:.1f} min)")

    n_pts = min(800, max(50, int(overlap * 5)))
    t = np.linspace(t0, t1, n_pts)

    # 插值 Rig
    rig_pos = np.array([m["position"] for m in rig_data])
    rig_eul = np.rad2deg(np.array([quaternion_to_euler(m["orientation"]) for m in rig_data]))
    rig_pos_i = np.stack([interp1d(rig_ts, rig_pos[:,j], "linear", fill_value="extrapolate")(t) for j in range(3)], axis=1)
    rig_eul_i = np.stack([interp1d(rig_ts, rig_eul[:,j], "linear", fill_value="extrapolate")(t) for j in range(3)], axis=1)

    # 插值 SVO
    svo_pos = np.array([m["position"] for m in svo_data])
    svo_eul = np.rad2deg(np.array([quaternion_to_euler(m["orientation"]) for m in svo_data]))
    svo_pos_i = np.stack([interp1d(svo_ts, svo_pos[:,j], "linear", fill_value="extrapolate")(t) for j in range(3)], axis=1)
    svo_eul_i = np.stack([interp1d(svo_ts, svo_eul[:,j], "linear", fill_value="extrapolate")(t) for j in range(3)], axis=1)

    # Sim(3) Umeyama 对齐
    try:
        svo_c = svo_pos_i - svo_pos_i.mean(axis=0)
        rig_c = rig_pos_i - rig_pos_i.mean(axis=0)
        s = np.clip(np.sqrt(np.sum(rig_c**2) / (np.sum(svo_c**2) + 1e-12)), 0.05, 20.0)
        H = svo_c.T @ rig_c
        U, _, Vt = np.linalg.svd(H)
        R_opt = Vt.T @ U.T
        if np.linalg.det(R_opt) < 0:
            Vt[-1] *= -1; R_opt = Vt.T @ U.T
        t_opt = rig_pos_i.mean(axis=0) - s * (R_opt @ svo_pos_i.mean(axis=0))
        svo_pos_aligned = s * (R_opt @ svo_pos_i.T).T + t_opt
        rot_eul = np.rad2deg(R_scipy.from_matrix(R_opt).as_euler("xyz"))
    except Exception as e:
        print(f"  Umeyama 失败: {e}")
        s, R_opt = 1.0, np.eye(3)
        rot_eul = np.zeros(3)
        t_opt = rig_pos_i[0] - svo_pos_i[0]
        svo_pos_aligned = svo_pos_i + t_opt

    pos_error = np.linalg.norm(svo_pos_aligned - rig_pos_i, axis=1)
    eul_error = np.minimum(np.abs(svo_eul_i - rig_eul_i), 360 - np.abs(svo_eul_i - rig_eul_i))

    print(f"\n  === 对齐结果 ===")
    print(f"  尺度因子: {s:.4f}")
    print(f"  旋转 (R_opt): rx={rot_eul[0]:+.2f}°, ry={rot_eul[1]:+.2f}°, rz={rot_eul[2]:+.2f}°")
    print(f"  位置 ATE: 均值={pos_error.mean():.3f}m, RMS={np.sqrt((pos_error**2).mean()):.3f}m, "
          f"最大={pos_error.max():.3f}m")
    print(f"  Roll:  均值={eul_error[:,0].mean():+.2f}° ± {eul_error[:,0].std():.2f}°")
    print(f"  Pitch: 均值={eul_error[:,1].mean():+.2f}° ± {eul_error[:,1].std():.2f}°")
    print(f"  Yaw:   均值={eul_error[:,2].mean():+.2f}° ± {eul_error[:,2].std():.2f}°")

    return {
        "t": t, "t_rel": t - t[0],
        "rig_pos": rig_pos_i, "rig_eul": rig_eul_i,
        "svo_pos": svo_pos_i, "svo_eul": svo_eul_i,
        "svo_pos_aligned": svo_pos_aligned,
        "pos_error": pos_error, "eul_error": eul_error,
        "scale": s, "R_opt": R_opt, "rot_eul": rot_eul,
    }


# ─── 可视化 ──────────────────────────────────────────────────

def plot_comparison(data, output_dir):
    t_rel = data["t_rel"]
    R, G, B, P, O = "#e74c3c", "#27ae60", "#2980b9", "#8e44ad", "#e67e22"

    # 2D 轨迹
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("SVO vs Ground Truth — 2D Projection", fontsize=13, fontweight="bold")
    for ax, xi, yi, xl, yl in [(axes[0],0,1,"X","Y"),(axes[1],0,2,"X","Z"),(axes[2],1,2,"Y","Z")]:
        ax.plot(data["rig_pos"][:,xi], data["rig_pos"][:,yi], color=G, lw=1.5, label="GT", alpha=0.9)
        ax.plot(data["svo_pos_aligned"][:,xi], data["svo_pos_aligned"][:,yi], color=R, lw=1.2, ls="--", label="SVO", alpha=0.7)
        ax.scatter(data["rig_pos"][0,xi], data["rig_pos"][0,yi], c=G, s=80, zorder=5, marker="o", label="Start")
        ax.set_xlabel(f"{xl} (m)"); ax.set_ylabel(f"{yl} (m)")
        ax.set_title(f"{xl}{yl}"); ax.legend(fontsize=7); ax.grid(True, alpha=0.2); ax.axis("equal")
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, "07_svo_trajectory_2d.png"), dpi=150); plt.close(fig)
    print(f"  saved: 07_svo_trajectory_2d.png")

    # 位置时序
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig.suptitle("SVO Position vs Ground Truth", fontsize=13, fontweight="bold")
    for j, (lbl, col) in enumerate([("X",R),("Y",G),("Z",B)]):
        axes[j].plot(t_rel, data["rig_pos"][:,j], color=col, lw=1.5, label=f"GT {lbl}")
        axes[j].plot(t_rel, data["svo_pos_aligned"][:,j], color=col, lw=1.2, ls="--", alpha=0.7, label=f"SVO {lbl}")
        err = data["rig_pos"][:,j] - data["svo_pos_aligned"][:,j]
        axes[j].fill_between(t_rel, 0, err, color=col, alpha=0.12)
        axes[j].set_ylabel(f"{lbl} (m)"); axes[j].legend(fontsize=8); axes[j].grid(True, alpha=0.2)
    axes[-1].set_xlabel("Time (s) relative")
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, "08_svo_position_timeseries.png"), dpi=150); plt.close(fig)
    print(f"  saved: 08_svo_position_timeseries.png")

    # 欧拉角
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig.suptitle("SVO vs Ground Truth — Euler Angles", fontsize=13, fontweight="bold")
    for j, (lbl, col) in enumerate([("Roll",R),("Pitch",G),("Yaw",B)]):
        axes[j].plot(t_rel, data["rig_eul"][:,j], color=col, lw=1.5, label=f"GT {lbl}")
        axes[j].plot(t_rel, data["svo_eul"][:,j], color=col, lw=1.2, ls="--", alpha=0.7, label=f"SVO {lbl}")
        axes[j].set_ylabel(f"{lbl} (deg)"); axes[j].legend(fontsize=8); axes[j].grid(True, alpha=0.2)
    axes[-1].set_xlabel("Time (s) relative")
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, "09_svo_orientation_timeseries.png"), dpi=150); plt.close(fig)
    print(f"  saved: 09_svo_orientation_timeseries.png")

    # 位置误差
    fig, axes = plt.subplots(2, 1, figsize=(14, 7))
    fig.suptitle("SVO Position Error", fontsize=13, fontweight="bold")
    axes[0].plot(t_rel, data["pos_error"], color=P, lw=1.2, label="ATE")
    axes[0].axhline(data["pos_error"].mean(), color=O, lw=2, ls="--", label=f"Mean={data['pos_error'].mean():.3f}m")
    axes[0].set_ylabel("Error (m)"); axes[0].legend(); axes[0].grid(True, alpha=0.2)
    for j, (lbl, col) in enumerate([("X",R),("Y",G),("Z",B)]):
        axes[1].plot(t_rel, data["rig_pos"][:,j]-data["svo_pos_aligned"][:,j], color=col, lw=1.0, label=f"{lbl} err", alpha=0.8)
    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].set_ylabel("Error (m)"); axes[1].set_xlabel("Time (s)"); axes[1].legend(); axes[1].grid(True, alpha=0.2)
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, "10_svo_position_error.png"), dpi=150); plt.close(fig)
    print(f"  saved: 10_svo_position_error.png")

    # 3D
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    step = max(1, len(data["rig_pos"])//300)
    ax.plot(data["rig_pos"][::step,0], data["rig_pos"][::step,1], data["rig_pos"][::step,2], color=G, lw=1.5, label="GT", alpha=0.9)
    ax.plot(data["svo_pos_aligned"][::step,0], data["svo_pos_aligned"][::step,1], data["svo_pos_aligned"][::step,2], color=R, lw=1.2, ls="--", label="SVO", alpha=0.7)
    ax.scatter(data["rig_pos"][0,0], data["rig_pos"][0,1], data["rig_pos"][0,2], c=G, s=120, marker="o", label="Start", zorder=5)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("SVO vs GT — 3D Trajectory")
    ax.legend()
    fig.savefig(os.path.join(output_dir, "11_svo_trajectory_3d.png"), dpi=150); plt.close(fig)
    print(f"  saved: 11_svo_trajectory_3d.png")


def generate_report(data, offset, output_dir):
    report = {
        "dataset": "airground_rig_s3_ros2",
        "time_offset_s": round(offset, 3),
        "scale_vs_ground_truth": round(float(data["scale"]), 4),
        "umeyama_rotation_deg": {
            "rx": round(float(data["rot_eul"][0]), 2),
            "ry": round(float(data["rot_eul"][1]), 2),
            "rz": round(float(data["rot_eul"][2]), 2),
        },
        "position_error_ate_m": {
            "mean": round(float(data["pos_error"].mean()), 4),
            "rms": round(float(np.sqrt((data["pos_error"]**2).mean())), 4),
            "max": round(float(data["pos_error"].max()), 4),
        },
        "orientation_error_deg": {
            "roll":  {"mean": round(float(data["eul_error"][:,0].mean()), 2),
                      "std": round(float(data["eul_error"][:,0].std()), 2)},
            "pitch": {"mean": round(float(data["eul_error"][:,1].mean()), 2),
                      "std": round(float(data["eul_error"][:,1].std()), 2)},
            "yaw":   {"mean": round(float(data["eul_error"][:,2].mean()), 2),
                      "std": round(float(data["eul_error"][:,2].std()), 2)},
        },
    }
    path = os.path.join(output_dir, "svo_comparison_report.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"  saved: svo_comparison_report.json")


# ─── 主函数 ──────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="SVO vs 真值对比")
    parser.add_argument("--bag", type=str,
        default="/home/lxy/asr_sdm_robo/datasheet/airground_rig_s3_ros2")
    parser.add_argument("--recording", type=str,
        default="/home/lxy/asr_sdm_robo/datasheet/svo_recording_clock/svo_bag")
    parser.add_argument("--output", type=str,
        default="/home/lxy/asr_sdm_robo/datasheet/svo_analysis_output")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    print(f"\n{'='*65}")
    print(f"  读取数据: 录制={args.recording}")
    print(f"{'='*65}")

    rig_data, svo_data, offset = read_recording_with_offset(args.recording)
    if rig_data is None:
        print("读取失败"); sys.exit(1)

    print(f"  Rig: {len(rig_data)} poses, SVO: {len(svo_data)} poses")

    print(f"\n{'='*65}")
    print(f"  姿态对齐与对比")
    print(f"{'='*65}")

    comparison = align_and_compare(rig_data, svo_data)
    if comparison is None:
        sys.exit(1)

    print(f"\n{'='*65}")
    print(f"  生成图表")
    print(f"{'='*65}")
    plot_comparison(comparison, args.output)
    generate_report(comparison, offset, args.output)

    print(f"\n{'='*65}")
    print(f"  完成！输出: {args.output}")
    print(f"{'='*65}")


if __name__ == "__main__":
    main()
