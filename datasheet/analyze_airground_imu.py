#!/usr/bin/env python3
"""
分析 airground_rig_s3_ros2 数据包中的 IMU 坐标系、IMU 测量值与真值，
通过四元数积分求解 IMU 姿态并与真值进行对比。

依赖: pip install numpy matplotlib scipy
运行环境: ROS2 Jazzy (rosbag2_py, sensor_msgs, geometry_msgs)
Usage:
    source /opt/ros/jazzy/setup.bash
    python3 analyze_airground_imu.py [--bag /path/to/bag] [--output ./output]
"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from scipy.interpolate import interp1d
import json
import warnings
warnings.filterwarnings("ignore")

# ── ROS2 bag 读取 ──────────────────────────────────────────────
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
import rclpy.serialization

# ROS2 消息类型 (通过 get_message 动态获取，这里仅用于类型注解)


def get_msg_class(type_str: str):
    """从 'pkg/msg/Type' 字符串获取 Python 消息类"""
    # get_message expects 'pkg/msg/Type'
    return get_message(type_str)


def read_bag(bag_path: str):
    """
    使用 rosbag2_py + rclpy.serialization 读取 bag，
    正确解析 CDR 编码的 ROS2 消息。
    """
    storage_opts = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_opts = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_opts, converter_opts)

    topic_types = reader.get_all_topics_and_types()
    topic_map = {t.name: t.type for t in topic_types}
    print(f"[Bag] Topics: {list(topic_map.keys())}")

    imu_data = []
    rig_data = []
    mag_data = []

    # 预加载消息类型
    Imu_cls = get_msg_class("sensor_msgs/msg/Imu")
    PoseStamped_cls = get_msg_class("geometry_msgs/msg/PoseStamped")
    Vector3Stamped_cls = get_msg_class("geometry_msgs/msg/Vector3Stamped")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        timestamp = t * 1e-9  # 纳秒 → 秒

        if topic == "imu/data":
            msg = rclpy.serialization.deserialize_message(data, Imu_cls)
            imu_data.append({
                "stamp": timestamp,
                "orientation": [msg.orientation.x, msg.orientation.y,
                               msg.orientation.z, msg.orientation.w],
                "angular_velocity": [msg.angular_velocity.x, msg.angular_velocity.y,
                                   msg.angular_velocity.z],
                "linear_acceleration": [msg.linear_acceleration.x,
                                       msg.linear_acceleration.y,
                                       msg.linear_acceleration.z],
            })
        elif topic == "Rig":
            msg = rclpy.serialization.deserialize_message(data, PoseStamped_cls)
            rig_data.append({
                "stamp": timestamp,
                "position": [msg.pose.position.x, msg.pose.position.y,
                            msg.pose.position.z],
                "orientation": [msg.pose.orientation.x, msg.pose.orientation.y,
                               msg.pose.orientation.z, msg.pose.orientation.w],
            })
        elif topic == "/imu/magnetometer":
            msg = rclpy.serialization.deserialize_message(data, Vector3Stamped_cls)
            mag_data.append({
                "stamp": timestamp,
                "vector": [msg.vector.x, msg.vector.y, msg.vector.z],
            })

    print(f"[Bag] imu/data: {len(imu_data)}, Rig: {len(rig_data)}, magnetometer: {len(mag_data)}")
    return imu_data, rig_data, mag_data


# ── 四元数工具 ──────────────────────────────────────────────

def quaternion_normalize(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def quaternion_multiply(a, b):
    """四元数乘法 (格式: [x,y,z,w])"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quaternion_to_euler(q):
    """四元数 [x,y,z,w] → [roll, pitch, yaw] (rad)"""
    x, y, z, w = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp = 2*(w*y - z*x)
    pitch = np.clip(arcsin_smart(sinp), -np.pi/2, np.pi/2)
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def arcsin_smart(x):
    """安全的 arcsin，避免浮点误差"""
    return np.arcsin(np.clip(x, -1.0, 1.0))


def quaternion_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quaternion_error(q_est, q_true):
    """计算两个四元数之间的误差角（度）"""
    q_conj = quaternion_conjugate(q_true)
    q_err = quaternion_multiply(q_est, q_conj)
    # 误差角 = 2 * acos(|qw|)
    qw = np.abs(q_err[3])
    qw = min(qw, 1.0)
    return np.degrees(2 * np.arccos(qw))


# ── IMU 姿态积分 ──────────────────────────────────────────────

def integrate_gyro_quaternion(imu_data):
    """
    用角速度积分四元数。
    初始姿态: 单位四元数（假设 IMU 启动时与世界坐标系对齐）
    返回: (时间戳, 四元数组[N,4])
    """
    if not imu_data:
        return np.array([]), np.zeros((0, 4))

    stamps = np.array([m["stamp"] for m in imu_data])
    gyro = np.array([m["angular_velocity"] for m in imu_data])
    dt = np.clip(np.diff(stamps), 0, 0.1)

    quats = np.zeros((len(imu_data), 4))
    q = np.array([0.0, 0.0, 0.0, 1.0])

    for i in range(len(imu_data)):
        quats[i] = q
        if i < len(imu_data) - 1:
            wx, wy, wz = gyro[i]
            omega_q = np.array([wx, wy, wz, 0.0])
            q_dot = 0.5 * quaternion_multiply(omega_q, q)
            q = quaternion_normalize(q + q_dot * dt[i])

    return stamps, quats


def integrate_gyro_euler(imu_data, method="zxy"):
    """
    用角速度直接积分欧拉角（每帧单独积分，再转四元数）
    """
    if not imu_data:
        return np.array([]), np.zeros((0, 4))

    stamps = np.array([m["stamp"] for m in imu_data])
    gyro = np.array([m["angular_velocity"] for m in imu_data])
    dt = np.clip(np.diff(stamps), 0, 0.1)

    # 初始欧拉角（从 IMU 内置四元数取均值）
    init_q = quaternion_normalize(
        np.mean([m["orientation"] for m in imu_data[:30]
                 if np.linalg.norm(m["orientation"]) > 0.1], axis=0)
        if any(np.linalg.norm(m["orientation"]) > 0.1 for m in imu_data[:30]) else
        [0.0, 0.0, 0.0, 1.0]
    )
    euler = quaternion_to_euler(init_q)

    quats = np.zeros((len(imu_data), 4))
    quats[0] = init_q

    for i in range(len(imu_data) - 1):
        wx, wy, wz = gyro[i]
        euler_dot = np.array([wx, wy, wz])
        euler = euler + euler_dot * dt[i]
        quats[i+1] = quaternion_normalize(
            np.array([np.sin(euler[0]/2)*np.cos(euler[1]/2)*np.cos(euler[2]/2)
                      + np.cos(euler[0]/2)*np.sin(euler[1]/2)*np.sin(euler[2]/2),
                      np.cos(euler[0]/2)*np.sin(euler[1]/2)*np.cos(euler[2]/2)
                      - np.sin(euler[0]/2)*np.cos(euler[1]/2)*np.sin(euler[2]/2),
                      np.cos(euler[0]/2)*np.cos(euler[1]/2)*np.sin(euler[2]/2)
                      + np.sin(euler[0]/2)*np.sin(euler[1]/2)*np.cos(euler[2]/2),
                      np.cos(euler[0]/2)*np.cos(euler[1]/2)*np.cos(euler[2]/2)
                      - np.sin(euler[0]/2)*np.sin(euler[1]/2)*np.sin(euler[2]/2)])
        )

    return stamps, quats


# ── 分析函数 ──────────────────────────────────────────────

def analyze_imu_coordinate_frame(imu_data, mag_data):
    """分析 IMU 坐标系（通过静止段的重力方向和磁场方向）"""
    print("\n" + "=" * 65)
    print("  1. IMU 坐标系分析")
    print("=" * 65)

    if not imu_data:
        print("  无 IMU 数据")
        return None

    # 静止段（前 5 秒）
    t0 = imu_data[0]["stamp"]
    static = [m for m in imu_data if m["stamp"] - t0 < 5.0]

    acc = np.array([m["linear_acceleration"] for m in static])
    gyro = np.array([m["angular_velocity"] for m in static])
    acc_norm = np.linalg.norm(acc, axis=1)

    acc_mean = acc.mean(axis=0)
    gyro_mean = gyro.mean(axis=0)

    print(f"\n  [静止段] {len(static)} 帧 (前 5 秒)")
    print(f"\n  加速度 (m/s²):")
    print(f"    X: {acc_mean[0]:+.5f} ± {acc[:,0].std():.5f}")
    print(f"    Y: {acc_mean[1]:+.5f} ± {acc[:,1].std():.5f}")
    print(f"    Z: {acc_mean[2]:+.5f} ± {acc[:,2].std():.5f}")
    print(f"    |a| = {acc_norm.mean():.4f} m/s²  (重力 ≈ 9.81)")

    print(f"\n  角速度 (rad/s):")
    print(f"    X: {gyro_mean[0]:+.6f} ± {gyro[:,0].std():.6f}")
    print(f"    Y: {gyro_mean[1]:+.6f} ± {gyro[:,1].std():.6f}")
    print(f"    Z: {gyro_mean[2]:+.6f} ± {gyro[:,2].std():.6f}")

    # 判断重力主轴
    axis_names = ["X", "Y", "Z"]
    dominant = np.argmax(np.abs(acc_mean))
    dominant_axis = axis_names[dominant]
    gravity_sign = "+" if acc_mean[dominant] > 0 else "-"
    print(f"\n  坐标系判断:")
    if abs(acc_mean[2]) > max(abs(acc_mean[0]), abs(acc_mean[1])):
        print(f"    重力主要在 Z 轴 → IMU Z 轴{'正向' if acc_mean[2]>0 else '负向'}指向天空")
        print(f"    → 坐标系: 符合 ENU (East-North-Up) 惯例")
    elif abs(acc_mean[1]) > max(abs(acc_mean[0]), abs(acc_mean[2])):
        print(f"    重力主要在 Y 轴 → IMU Y 轴{'正向' if acc_mean[1]>0 else '负向'}指向天空")
    else:
        print(f"    重力主要在 X 轴 → IMU X 轴{'正向' if acc_mean[0]>0 else '负向'}指向天空")

    # IMU 内置四元数
    orient_arr = np.array([m["orientation"] for m in imu_data])
    orient_valid = orient_arr[~np.all(orient_arr == 0, axis=1)]
    print(f"\n  IMU 内置四元数: {len(orient_valid)}/{len(imu_data)} 帧有效")
    if len(orient_valid) > 10:
        euls_builtin = np.array([quaternion_to_euler(q) for q in orient_valid])
        eul_mean = np.degrees(euls_builtin.mean(axis=0))
        eul_std = np.degrees(euls_builtin.std(axis=0))
        print(f"    Roll : {eul_mean[0]:+.2f}° ± {eul_std[0]:.2f}°")
        print(f"    Pitch: {eul_mean[1]:+.2f}° ± {eul_std[1]:.2f}°")
        print(f"    Yaw  : {eul_mean[2]:+.2f}° ± {eul_std[2]:.2f}°")

    # 磁力计数据
    if mag_data:
        static_mag = [m for m in mag_data if m["stamp"] - t0 < 5.0]
        if static_mag:
            mag = np.array([m["vector"] for m in static_mag])
            mag_mean = mag.mean(axis=0)
            print(f"\n  磁力计均值 (前5秒静止段):")
            print(f"    X: {mag_mean[0]:+.3f}  Y: {mag_mean[1]:+.3f}  Z: {mag_mean[2]:+.3f}")
            mag_norm = np.linalg.norm(mag_mean)
            print(f"    |mag| = {mag_norm:.3f} (参考: 25-65 µT)")
            if abs(mag_mean[1]) > abs(mag_mean[0]) and abs(mag_mean[1]) > abs(mag_mean[2]):
                print(f"    → Y 轴磁场最强，Y 轴可能指向地磁北")

    return {
        "acc_mean": acc_mean,
        "gyro_mean": gyro_mean,
        "acc_norm_mean": acc_norm.mean(),
        "dominant_axis": dominant_axis,
        "gravity_sign": gravity_sign,
    }


def analyze_ground_truth(rig_data):
    """分析真值轨迹"""
    print("\n" + "=" * 65)
    print("  2. 真值 (Rig) 轨迹分析")
    print("=" * 65)

    if not rig_data:
        print("  无真值数据")
        return None

    stamps = np.array([m["stamp"] for m in rig_data])
    pos = np.array([m["position"] for m in rig_data])
    quats = np.array([m["orientation"] for m in rig_data])
    euls = np.array([quaternion_to_euler(q) for q in quats])

    duration = stamps[-1] - stamps[0]
    freq = len(rig_data) / duration

    print(f"\n  总帧数: {len(rig_data)}")
    print(f"  时长: {duration:.2f} 秒")
    print(f"  频率: {freq:.1f} Hz")
    print(f"\n  位置:")
    print(f"    起始: [{pos[0,0]:+.4f}, {pos[0,1]:+.4f}, {pos[0,2]:+.4f}] m")
    print(f"    终止: [{pos[-1,0]:+.4f}, {pos[-1,1]:+.4f}, {pos[-1,2]:+.4f}] m")
    print(f"    范围: X[{pos[:,0].min():.2f} ~ {pos[:,0].max():.2f}], "
          f"Y[{pos[:,1].min():.2f} ~ {pos[:,1].max():.2f}], "
          f"Z[{pos[:,2].min():.2f} ~ {pos[:,2].max():.2f}]")

    print(f"\n  欧拉角 (起始 → 终止):")
    print(f"    Roll : {np.degrees(euls[0,0]):+.2f}° → {np.degrees(euls[-1,0]):+.2f}°")
    print(f"    Pitch: {np.degrees(euls[0,1]):+.2f}° → {np.degrees(euls[-1,1]):+.2f}°")
    print(f"    Yaw  : {np.degrees(euls[0,2]):+.2f}° → {np.degrees(euls[-1,2]):+.2f}°")

    dt = np.diff(stamps)
    vel = np.diff(pos, axis=0) / dt[:, None]
    speed = np.linalg.norm(vel, axis=1)
    print(f"\n  速度:")
    print(f"    最大: {speed.max():.3f} m/s, 平均: {speed.mean():.3f} m/s")

    return {
        "stamps": stamps, "position": pos,
        "quaternions": quats, "euler": euls,
        "speed": speed,
    }


def compare_attitude(imu_data, rig_data):
    """IMU 积分姿态与真值对比"""
    print("\n" + "=" * 65)
    print("  3. IMU 积分姿态 vs 真值姿态对比")
    print("=" * 65)

    if not imu_data or not rig_data:
        print("  数据不足，无法对比")
        return None

    # IMU 角速度积分
    imu_ts, imu_quats = integrate_gyro_quaternion(imu_data)

    # 真值
    rig_ts = np.array([m["stamp"] for m in rig_data])
    rig_quats = np.array([m["orientation"] for m in rig_data])

    # 时间对齐（取公共区间）
    t_start = max(imu_ts[0], rig_ts[0])
    t_end = min(imu_ts[-1], rig_ts[-1])
    n_points = 600
    t_compare = np.linspace(t_start, t_end, n_points)

    # 插值
    imu_q_interp = []
    for j in range(4):
        f = interp1d(imu_ts, imu_quats[:, j], kind="linear", fill_value="extrapolate")
        imu_q_interp.append(f(t_compare))
    imu_q_interp = np.stack(imu_q_interp, axis=1)

    rig_q_interp = []
    for j in range(4):
        f = interp1d(rig_ts, rig_quats[:, j], kind="linear", fill_value="extrapolate")
        rig_q_interp.append(f(t_compare))
    rig_q_interp = np.stack(rig_q_interp, axis=1)

    # 误差角
    errors = np.array([quaternion_error(imu_q_interp[i], rig_q_interp[i])
                       for i in range(n_points)])

    print(f"\n  时间区间: {t_compare[0]:.3f} ~ {t_compare[-1]:.3f} s (相对起始 {t_start - t_compare[0]:.2f} s)")
    print(f"\n  整体姿态误差角 (IMU积分 vs 真值):")
    print(f"    均值:   {errors.mean():+.2f}°")
    print(f"    标准差: {errors.std():.2f}°")
    print(f"    最大值: {errors.max():+.2f}°")
    print(f"    最小值: {errors.min():+.2f}°")

    # 分段误差
    segs = [(0, 10), (10, 30), (30, float(t_compare[-1] - t_compare[0]))]
    print(f"\n  分段误差:")
    for t0, t1 in segs:
        mask = (t_compare - t_compare[0] >= t0) & (t_compare - t_compare[0] <= t1)
        if mask.sum() > 0:
            print(f"    [{t0:3.0f}~{t1:3.0f}s]: 均值={errors[mask].mean():+.2f}°, 最大={errors[mask].max():+.2f}°")

    # 欧拉角对比
    imu_euls = np.rad2deg(np.array([quaternion_to_euler(q) for q in imu_q_interp]))
    rig_euls = np.rad2deg(np.array([quaternion_to_euler(q) for q in rig_q_interp]))
    t_rel = t_compare - t_compare[0]

    print(f"\n  欧拉角误差 (IMU积分 - 真值):")
    labels = ["Roll ", "Pitch", "Yaw  "]
    for j in range(3):
        err_e = imu_euls[:, j] - rig_euls[:, j]
        print(f"    {labels[j]}: 均值={err_e.mean():+.2f}°, σ={err_e.std():.2f}°, "
              f"最大={err_e.max():+.2f}°")

    print(f"\n  说明: IMU 仅通过角速度积分，无 GPS/视觉修正，")
    print(f"        yaw 角漂移最严重（零偏累积），这是正常现象。")

    return {
        "t_compare": t_compare,
        "t_rel": t_rel,
        "imu_quats": imu_q_interp,
        "rig_quats": rig_q_interp,
        "imu_euls": imu_euls,
        "rig_euls": rig_euls,
        "errors": errors,
    }


def analyze_imu_measurement_stats(imu_data):
    """IMU 测量值统计分析"""
    print("\n" + "=" * 65)
    print("  4. IMU 测量值统计分析 (全时序)")
    print("=" * 65)

    if not imu_data:
        return

    stamps = np.array([m["stamp"] for m in imu_data])
    acc = np.array([m["linear_acceleration"] for m in imu_data])
    gyro = np.array([m["angular_velocity"] for m in imu_data])
    acc_norm = np.linalg.norm(acc, axis=1)

    print(f"\n  数据点数: {len(imu_data)}")
    print(f"  时长: {stamps[-1] - stamps[0]:.2f} s")
    print(f"  频率: {len(imu_data)/(stamps[-1]-stamps[0]):.1f} Hz")
    print(f"\n  加速度 (m/s²):")
    print(f"    X: 均值={acc[:,0].mean():+.4f}, std={acc[:,0].std():.4f}, "
          f"min={acc[:,0].min():.4f}, max={acc[:,0].max():.4f}")
    print(f"    Y: 均值={acc[:,1].mean():+.4f}, std={acc[:,1].std():.4f}, "
          f"min={acc[:,1].min():.4f}, max={acc[:,1].max():.4f}")
    print(f"    Z: 均值={acc[:,2].mean():+.4f}, std={acc[:,2].std():.4f}, "
          f"min={acc[:,2].min():.4f}, max={acc[:,2].max():.4f}")
    print(f"    |a|: 均值={acc_norm.mean():.4f}, std={acc_norm.std():.4f}, "
          f"min={acc_norm.min():.4f}, max={acc_norm.max():.4f}")

    print(f"\n  角速度 (rad/s):")
    print(f"    X: 均值={gyro[:,0].mean():+.6f}, std={gyro[:,0].std():.6f}")
    print(f"    Y: 均值={gyro[:,1].mean():+.6f}, std={gyro[:,1].std():.6f}")
    print(f"    Z: 均值={gyro[:,2].mean():+.6f}, std={gyro[:,2].std():.6f}")

    # Allan 方差（简化的）
    tau = 1.0  # 1秒平均
    n = min(len(imu_data), int(tau * 100))  # 100Hz
    gyro_n = gyro[:, 2]  # z轴角速度
    alland = []
    for i in range(0, len(gyro_n) - n, n):
        seg = gyro_n[i:i+n]
        alland.append(seg.std())
    if alland:
        print(f"\n  角速度 Allan 偏差 (τ=1s, Z轴): {np.mean(alland):.6f} rad/s")


# ── 可视化 ──────────────────────────────────────────────

def plot_all(imu_data, rig_data, mag_data, coord_info, comparison, output_dir):
    """生成所有图表"""
    os.makedirs(output_dir, exist_ok=True)

    # 配色
    RED = "#e74c3c"; GREEN = "#27ae60"; BLUE = "#2980b9"
    PURPLE = "#8e44ad"; ORANGE = "#e67e22"; TEAL = "#16a085"
    GRAY = "#7f8c8d"

    stamps = np.array([m["stamp"] for m in imu_data]) if imu_data else np.array([])
    t0 = stamps[0] if len(stamps) > 0 else 0.0
    t_rel_imu = stamps - t0

    # ═══ 图1: IMU 原始测量时序 ════════════════════════════════
    fig1, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    fig1.suptitle("IMU Raw Measurements — airground_rig_s3_ros2",
                  fontsize=14, fontweight="bold")

    if imu_data:
        acc = np.array([m["linear_acceleration"] for m in imu_data])
        gyro = np.array([m["angular_velocity"] for m in imu_data])

        axes[0].plot(t_rel_imu, acc[:, 0], label="Acc X", color=RED, alpha=0.7, lw=0.5)
        axes[0].plot(t_rel_imu, acc[:, 1], label="Acc Y", color=GREEN, alpha=0.7, lw=0.5)
        axes[0].plot(t_rel_imu, acc[:, 2], label="Acc Z", color=BLUE, alpha=0.7, lw=0.5)
        axes[0].axhline(0, color="k", lw=0.3)
        axes[0].axhline(9.81, color=GRAY, lw=1, ls="--", label="g=9.81")
        axes[0].set_ylabel("Acceleration (m/s²)")
        axes[0].legend(loc="upper right")
        axes[0].grid(True, alpha=0.25)

        axes[1].plot(t_rel_imu, gyro[:, 0], label="ωx", color=RED, alpha=0.7, lw=0.5)
        axes[1].plot(t_rel_imu, gyro[:, 1], label="ωy", color=GREEN, alpha=0.7, lw=0.5)
        axes[1].plot(t_rel_imu, gyro[:, 2], label="ωz", color=BLUE, alpha=0.7, lw=0.5)
        axes[1].axhline(0, color="k", lw=0.3)
        axes[1].set_ylabel("Angular Velocity (rad/s)")
        axes[1].legend(loc="upper right")
        axes[1].grid(True, alpha=0.25)

        orient_arr = np.array([m["orientation"] for m in imu_data])
        valid_mask = ~np.all(orient_arr == 0, axis=1)
        if valid_mask.sum() > 0:
            euls = np.rad2deg(np.array([quaternion_to_euler(q) for q in orient_arr]))
            axes[2].plot(t_rel_imu[valid_mask], euls[valid_mask, 0],
                         label="Roll", color=RED, alpha=0.7, lw=0.5)
            axes[2].plot(t_rel_imu[valid_mask], euls[valid_mask, 1],
                         label="Pitch", color=GREEN, alpha=0.7, lw=0.5)
            axes[2].plot(t_rel_imu[valid_mask], euls[valid_mask, 2],
                         label="Yaw", color=BLUE, alpha=0.7, lw=0.5)
            axes[2].set_ylabel("Euler Angles (deg)")
            axes[2].legend(loc="upper right")
        else:
            axes[2].text(0.5, 0.5, "No built-in orientation in imu/data",
                         transform=axes[2].transAxes, ha="center", va="center")
        axes[2].set_xlabel("Time (s)")
        axes[2].grid(True, alpha=0.25)

    plt.tight_layout()
    fig1.savefig(os.path.join(output_dir, "01_imu_raw_measurements.png"), dpi=150)
    plt.close(fig1)
    print(f"  saved: 01_imu_raw_measurements.png")

    # ═══ 图2: 真值轨迹 ═══════════════════════════════════════
    if rig_data:
        pos = np.array([m["position"] for m in rig_data])
        rig_t = np.array([m["stamp"] for m in rig_data]) - rig_data[0]["stamp"]
        quats = np.array([m["orientation"] for m in rig_data])
        euls = np.rad2deg(np.array([quaternion_to_euler(q) for q in quats]))

        fig2, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig2.suptitle("Ground Truth Trajectory (Rig)", fontsize=14, fontweight="bold")

        axes[0, 0].plot(pos[:, 0], pos[:, 1], "b-", lw=0.8)
        axes[0, 0].scatter(pos[0, 0], pos[0, 1], c=GREEN, s=100, zorder=5, label="Start")
        axes[0, 0].scatter(pos[-1, 0], pos[-1, 1], c=RED, s=100, zorder=5, label="End")
        axes[0, 0].set_xlabel("X (m)"); axes[0, 0].set_ylabel("Y (m)")
        axes[0, 0].set_title("XY Trajectory"); axes[0, 0].legend(); axes[0, 0].grid(True, alpha=0.25)
        axes[0, 0].axis("equal")

        axes[0, 1].plot(pos[:, 0], pos[:, 2], "b-", lw=0.8)
        axes[0, 1].set_xlabel("X (m)"); axes[0, 1].set_ylabel("Z (m)")
        axes[0, 1].set_title("XZ Trajectory"); axes[0, 1].grid(True, alpha=0.25)

        axes[1, 0].plot(rig_t, pos[:, 0], label="X", color=RED, lw=0.8)
        axes[1, 0].plot(rig_t, pos[:, 1], label="Y", color=GREEN, lw=0.8)
        axes[1, 0].plot(rig_t, pos[:, 2], label="Z", color=BLUE, lw=0.8)
        axes[1, 0].set_xlabel("Time (s)"); axes[1, 0].set_ylabel("Position (m)")
        axes[1, 0].set_title("Position vs Time"); axes[1, 0].legend(); axes[1, 0].grid(True, alpha=0.25)

        axes[1, 1].plot(rig_t, euls[:, 0], label="Roll", color=RED, lw=0.8)
        axes[1, 1].plot(rig_t, euls[:, 1], label="Pitch", color=GREEN, lw=0.8)
        axes[1, 1].plot(rig_t, euls[:, 2], label="Yaw", color=BLUE, lw=0.8)
        axes[1, 1].set_xlabel("Time (s)"); axes[1, 1].set_ylabel("Euler (deg)")
        axes[1, 1].set_title("Ground Truth Orientation"); axes[1, 1].legend(); axes[1, 1].grid(True, alpha=0.25)

        plt.tight_layout()
        fig2.savefig(os.path.join(output_dir, "02_ground_truth_trajectory.png"), dpi=150)
        plt.close(fig2)
        print(f"  saved: 02_ground_truth_trajectory.png")

    # ═══ 图3: 姿态对比 ════════════════════════════════════════
    if comparison is not None:
        t_rel = comparison["t_rel"]
        imu_e = comparison["imu_euls"]
        rig_e = comparison["rig_euls"]

        fig3, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
        fig3.suptitle("IMU Gyro Integration vs Ground Truth — Attitude Comparison",
                      fontsize=14, fontweight="bold")

        for j, (lbl, col) in enumerate([("Roll", RED), ("Pitch", GREEN), ("Yaw", BLUE)]):
            axes[j].plot(t_rel, rig_e[:, j], color=col, lw=1.8, label=f"Ground Truth {lbl}")
            axes[j].plot(t_rel, imu_e[:, j], color=col, lw=1.5, ls="--",
                         alpha=0.7, label=f"IMU Int. {lbl}")
            err = imu_e[:, j] - rig_e[:, j]
            axes[j].fill_between(t_rel, 0, err, color=col, alpha=0.12, label="Error")
            axes[j].set_ylabel(f"{lbl} (deg)")
            axes[j].legend(loc="upper right")
            axes[j].grid(True, alpha=0.25)

        axes[-1].set_xlabel("Time (s) relative to first overlap")
        plt.tight_layout()
        fig3.savefig(os.path.join(output_dir, "03_attitude_comparison.png"), dpi=150)
        plt.close(fig3)
        print(f"  saved: 03_attitude_comparison.png")

        # ═══ 图4: 误差角时序 ═════════════════════════════════════
        fig4, axes = plt.subplots(2, 1, figsize=(14, 7))
        fig4.suptitle("Attitude Error Analysis", fontsize=14, fontweight="bold")

        axes[0].plot(t_rel, comparison["errors"], color=PURPLE, lw=1.2, label="Error angle")
        axes[0].axhline(comparison["errors"].mean(), color=ORANGE, lw=2, ls="--",
                        label=f"Mean={comparison['errors'].mean():.1f}°")
        axes[0].set_ylabel("Error angle (deg)")
        axes[0].legend(); axes[0].grid(True, alpha=0.25)
        axes[0].set_title("Quaternion Error Angle: IMU Int. vs Ground Truth")

        # 误差分解
        err_r = imu_e[:, 0] - rig_e[:, 0]
        err_p = imu_e[:, 1] - rig_e[:, 1]
        err_y = imu_e[:, 2] - rig_e[:, 2]
        axes[1].plot(t_rel, err_r, color=RED, lw=1.2, label="Roll error")
        axes[1].plot(t_rel, err_p, color=GREEN, lw=1.2, label="Pitch error")
        axes[1].plot(t_rel, err_y, color=BLUE, lw=1.2, label="Yaw error")
        axes[1].axhline(0, color="k", lw=0.5)
        axes[1].set_ylabel("Euler error (deg)")
        axes[1].set_xlabel("Time (s)")
        axes[1].legend(); axes[1].grid(True, alpha=0.25)
        axes[1].set_title("Euler Angle Error: IMU Int. - Ground Truth")

        plt.tight_layout()
        fig4.savefig(os.path.join(output_dir, "04_attitude_error.png"), dpi=150)
        plt.close(fig4)
        print(f"  saved: 04_attitude_error.png")

    # ═══ 图5: 坐标轴与加速度分析 ══════════════════════════════
    if imu_data and coord_info:
        fig5, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig5.suptitle("IMU Coordinate Frame & Motion Analysis",
                      fontsize=14, fontweight="bold")

        # 静止段均值加速度条形图
        ax = axes[0]
        acc_mean = coord_info["acc_mean"]
        bars = ax.bar(["X", "Y", "Z"], acc_mean, color=[RED, GREEN, BLUE], alpha=0.85)
        ax.axhline(9.81, color="k", ls="--", lw=1.2, label="g=9.81")
        ax.axhline(-9.81, color="k", ls="--", lw=1.2)
        ax.set_ylabel("Acceleration (m/s²)")
        ax.set_title("Mean Accel (first 5s, static)")
        ax.legend()
        for bar, v in zip(bars, acc_mean):
            ax.text(bar.get_x() + bar.get_width()/2, v + 0.15,
                    f"{v:.2f}", ha="center", va="bottom", fontsize=9)

        # 重力向量 2D 示意图
        ax = axes[1]
        x, y, z = acc_mean / (np.linalg.norm(acc_mean) + 1e-12)
        # 在 XY 平面投影
        ax.arrow(0, 0, x * 0.6, y * 0.6, head_width=0.05, head_length=0.03,
                 fc=PURPLE, ec=PURPLE, label="Gravity (XY proj)")
        ax.arrow(0, 0, 0, 0.8, head_width=0.05, head_length=0.03,
                 fc=BLUE, ec=BLUE, label="World Z")
        ax.arrow(0, 0, 0.8, 0, head_width=0.05, head_length=0.03,
                 fc=GREEN, ec=GREEN, label="World Y")
        ax.set_xlim([-1.1, 1.1]); ax.set_ylim([-1.1, 1.1])
        ax.set_xlabel("X"); ax.set_ylabel("Y")
        ax.set_title("Gravity Vector Projection (XY plane)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.2)
        ax.set_aspect("equal")
        # 标注 Z 分量
        ax.text(x * 0.65, y * 0.65 + 0.05,
                f"z={z:.3f}", fontsize=8, ha="center",
                color=PURPLE)

        # 加速度模长
        ax = axes[2]
        acc = np.array([m["linear_acceleration"] for m in imu_data])
        acc_n = np.linalg.norm(acc, axis=1)
        ax.plot(t_rel_imu, acc_n, color=PURPLE, lw=0.5, alpha=0.8, label="|acc|")
        ax.axhline(9.81, color="k", ls="--", lw=1.2, label="g=9.81")
        ax.axhline(acc_n.mean(), color=ORANGE, lw=1.5, label=f"mean={acc_n.mean():.2f}")
        ax.set_xlabel("Time (s)"); ax.set_ylabel("|acc| (m/s²)")
        ax.set_title("Acceleration Magnitude")
        ax.legend(); ax.grid(True, alpha=0.25)

        plt.tight_layout()
        fig5.savefig(os.path.join(output_dir, "05_coordinate_frame_analysis.png"), dpi=150)
        plt.close(fig5)
        print(f"  saved: 05_coordinate_frame_analysis.png")

    # ═══ 图6: 3D 轨迹 ════════════════════════════════════════
    if rig_data:
        from mpl_toolkits.mplot3d import Axes3D
        pos = np.array([m["position"] for m in rig_data])
        rig_t = np.array([m["stamp"] for m in rig_data]) - rig_data[0]["stamp"]

        fig6 = plt.figure(figsize=(10, 8))
        ax = fig6.add_subplot(111, projection="3d")
        # 颜色按时间渐变
        sc = ax.scatter(pos[::5, 0], pos[::5, 1], pos[::5, 2],
                        c=rig_t[::5], cmap="viridis", s=2, alpha=0.8)
        ax.scatter(pos[0, 0], pos[0, 1], pos[0, 2],
                   c=GREEN, s=150, marker="o", label="Start", zorder=5)
        ax.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2],
                   c=RED, s=150, marker="s", label="End", zorder=5)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
        ax.set_title("3D Ground Truth Trajectory")
        plt.colorbar(sc, label="Time (s)", shrink=0.6)
        ax.legend()
        fig6.savefig(os.path.join(output_dir, "06_trajectory_3d.png"), dpi=150)
        plt.close(fig6)
        print(f"  saved: 06_trajectory_3d.png")


# ── 报告生成 ──────────────────────────────────────────────

def generate_report(imu_data, rig_data, coord_info, comparison, output_dir):
    """生成 JSON 分析报告"""
    report = {"dataset": "airground_rig_s3_ros2"}

    if imu_data:
        ts = np.array([m["stamp"] for m in imu_data])
        acc = np.array([m["linear_acceleration"] for m in imu_data])
        gyro = np.array([m["angular_velocity"] for m in imu_data])
        report["imu"] = {
            "messages": len(imu_data),
            "duration_s": round(ts[-1] - ts[0], 3),
            "rate_hz": round(len(imu_data) / (ts[-1] - ts[0]), 2),
            "acc_mean": [round(float(acc[:, i].mean()), 5) for i in range(3)],
            "acc_std": [round(float(acc[:, i].std()), 5) for i in range(3)],
            "gyro_mean": [round(float(gyro[:, i].mean()), 6) for i in range(3)],
            "gyro_std": [round(float(gyro[:, i].std()), 6) for i in range(3)],
            "acc_norm_mean": round(float(np.linalg.norm(acc, axis=1).mean()), 5),
        }

    if rig_data:
        ts = np.array([m["stamp"] for m in rig_data])
        pos = np.array([m["position"] for m in rig_data])
        report["ground_truth"] = {
            "messages": len(rig_data),
            "duration_s": round(ts[-1] - ts[0], 3),
            "rate_hz": round(len(rig_data) / (ts[-1] - ts[0]), 2),
            "position_range": {
                "x": [round(float(pos[:, 0].min()), 4), round(float(pos[:, 0].max()), 4)],
                "y": [round(float(pos[:, 1].min()), 4), round(float(pos[:, 1].max()), 4)],
                "z": [round(float(pos[:, 2].min()), 4), round(float(pos[:, 2].max()), 4)],
            }
        }

    if coord_info:
        report["imu_coordinate_frame"] = {
            "gravity_dominant_axis": coord_info["dominant_axis"],
            "gravity_sign": coord_info["gravity_sign"],
            "acc_mean_xyz": [round(float(v), 5) for v in coord_info["acc_mean"]],
            "acc_norm_mean": round(float(coord_info["acc_norm_mean"]), 5),
            "interpretation": (
                f"IMU {coord_info['dominant_axis']} axis "
                f"{'positive' if coord_info['gravity_sign']=='+' else 'negative'} "
                "direction points to sky (gravity along -Z in world frame)"
                if coord_info["dominant_axis"] == "Z" else
                f"IMU gravity dominant on {coord_info['dominant_axis']} axis"
            ),
        }

    if comparison is not None:
        labels = ["roll", "pitch", "yaw"]
        err_stats = {}
        for j, lbl in enumerate(labels):
            err = comparison["imu_euls"][:, j] - comparison["rig_euls"][:, j]
            err_stats[lbl] = {
                "mean_deg": round(float(err.mean()), 2),
                "std_deg": round(float(err.std()), 2),
                "max_deg": round(float(err.max()), 2),
                "min_deg": round(float(err.min()), 2),
            }
        report["attitude_comparison"] = {
            "quaternion_error_deg": {
                "mean": round(float(comparison["errors"].mean()), 2),
                "std": round(float(comparison["errors"].std()), 2),
                "max": round(float(comparison["errors"].max()), 2),
                "min": round(float(comparison["errors"].min()), 2),
            },
            "euler_error_deg": err_stats,
            "note": "IMU gyro integration has no external correction, yaw drift is expected",
        }

    path = os.path.join(output_dir, "analysis_report.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"  saved: analysis_report.json")
    return report


# ── 主函数 ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="分析 airground_rig_s3_ros2 数据包 IMU 与真值")
    parser.add_argument("--bag", type=str, default=None,
                        help="Bag 目录/文件路径 (默认: metadata.yaml 同目录)")
    parser.add_argument("--output", type=str, default="./imu_analysis_output",
                        help="输出目录 (默认: ./imu_analysis_output)")
    args = parser.parse_args()

    # 解析路径（支持目录或 .db3 文件）
    meta_dir = "/home/lxy/asr_sdm_robo/datasheet/airground_rig_s3_ros2"
    if args.bag:
        bag_arg = os.path.expanduser(args.bag)
        if os.path.isdir(bag_arg):
            bag_path = bag_arg
        elif os.path.isfile(bag_arg):
            bag_path = os.path.dirname(bag_arg) or os.getcwd()
        else:
            print(f"Error: {bag_arg} not found"); return
    else:
        bag_path = meta_dir

    print(f"\n{'='*65}")
    print(f"  正在读取: {bag_path}")
    print(f"{'='*65}")

    try:
        imu_data, rig_data, mag_data = read_bag(bag_path)
    except Exception as e:
        print(f"\n读取失败: {e}")
        print("请确保已 source ROS2: source /opt/ros/jazzy/setup.bash")
        sys.exit(1)

    if not imu_data and not rig_data:
        print("未读取到任何数据！")
        sys.exit(1)

    output_dir = args.output
    os.makedirs(output_dir, exist_ok=True)

    # 分析
    coord_info = analyze_imu_coordinate_frame(imu_data, mag_data)
    gt_info = analyze_ground_truth(rig_data)
    analyze_imu_measurement_stats(imu_data)
    comparison = compare_attitude(imu_data, rig_data)

    # 可视化 + 报告
    print(f"\n{'='*65}")
    print(f"  生成图表中...")
    print(f"{'='*65}")
    plot_all(imu_data, rig_data, mag_data, coord_info, comparison, output_dir)
    report = generate_report(imu_data, rig_data, coord_info, comparison, output_dir)

    print(f"\n{'='*65}")
    print(f"  完成！输出目录: {output_dir}")
    print(f"{'='*65}")


if __name__ == "__main__":
    main()
