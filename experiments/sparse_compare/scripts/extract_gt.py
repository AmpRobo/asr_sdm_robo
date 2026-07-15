#!/usr/bin/env python3
"""
extract_gt.py — 从 rosbag2 (.db3) 中提取地面真值（GT）轨迹。

当前数据集 (EuRoC MAV, MH_03_medium_ros2) 的 GT 通过 /leica/position 发布，
为 geometry_msgs/PointStamped (位置 + ROS 时间戳，无姿态)。

输出 CSV 列 (以 'sparse_compare' 实验约定):
    timestamp_s, x, y, z

注意:
    1. 仅含 3D 位置，不含姿态. 如果需要 6-DoF GT，请改用 EuRoC `state_groundtruth_estimate0.csv`.
    2. 时间戳为浮点秒 (stamp.sec + stamp.nanosec*1e-9). 帧间间隔约 0.05 s (20 Hz 视场记录),
       对齐 ASR-SDM 输出时通常按最近邻或线性插值。
    3. `frame_id` 一般为空字符串 (本数据集)，位置坐标系为 Vicon/Leica 世界系 [m]。

用法:
    python extract_gt.py --bag datasheet/MH_03_medium_ros2/MH_03_medium_ros2.db3
    python extract_gt.py --bag <path.db3> --out gt.csv --topic /leica/position
"""

import argparse
import csv
import sqlite3
import struct
from pathlib import Path
from typing import Iterator, Tuple


# CDR (Common Data Representation) 解码: ROS2 默认小端序.
# PointStamped 内存布局 (去掉 4 字节封装头 00 01 00 00):
#   std_msgs/Header   : builtin_interfaces/Time stamp (int32 sec, uint32 nanosec) + string frame_id
#   geometry_msgs/Point: float64 x, y, z
# 整个消息 = 44 bytes total, 其中 frame_id 为空字符串 (length=1 仅有 '\0'),
# 故 Point 数据从 offset 20 开始 (4B header + 8B stamp + 4B str_len + 4B str_content = 20).
HDR = 4   # 封装头 (CDR little-endian)
STAMP = 8
LEN_F = 4
CONTENT_F = 4  # 空字符串占 1 字节，但用 4 字节对齐槽
POINT_OFF = HDR + STAMP + LEN_F + CONTENT_F  # = 20
POINT_SIZE = 24  # 3 * float64


def parse_point_stamped(buf: bytes) -> Tuple[float, float, float, float]:
    """解析 geometry_msgs/PointStamped 的 CDR 字节流.

    返回 (timestamp_sec, x, y, z).  若消息字节流过短或布局不符，抛 ValueError.
    """
    if len(buf) < POINT_OFF + POINT_SIZE:
        raise ValueError(f"buffer too short: {len(buf)} bytes (need {POINT_OFF + POINT_SIZE})")
    sec, nsec = struct.unpack_from("<iI", buf, HDR)
    t = float(sec) + nsec * 1e-9
    x, y, z = struct.unpack_from("<3d", buf, POINT_OFF)
    return t, x, y, z


def iter_messages(db3_path: Path, topic: str) -> Iterator[Tuple[int, bytes]]:
    """流式迭代 sqlite3 rosbag2 数据库中指定话题的消息 (按时间戳顺序)."""
    conn = sqlite3.connect(str(db3_path))
    try:
        cur = conn.cursor()
        row = cur.execute(
            "SELECT id FROM topics WHERE name = ?", (topic,)
        ).fetchone()
        if row is None:
            raise ValueError(
                f"话题 '{topic}' 不在该 bag 内. "
                f"可用话题: {[r[0] for r in cur.execute('SELECT name FROM topics').fetchall()]}"
            )
        topic_id = row[0]
        yield from cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,),
        )
    finally:
        conn.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--bag", required=True, type=Path,
        help="rosbag2 .db3 路径 (含 metadata 的目录或单文件)",
    )
    parser.add_argument(
        "--out", type=Path, default=None,
        help="输出 CSV 路径 (默认: <bag_stem>_gt.csv 与 bag 同目录)",
    )
    parser.add_argument(
        "--topic", default="/leica/position",
        help="GT 话题名 (默认 /leica/position，对应 EuRoC Leica 位置真值)",
    )
    parser.add_argument(
        "--limit", type=int, default=None,
        help="仅解析前 N 条 (调试用)",
    )
    args = parser.parse_args()

    bag_path: Path = args.bag
    if not bag_path.is_file():
        parser.error(f"找不到 bag 文件: {bag_path}")

    out_path: Path = args.out or bag_path.with_name(f"{bag_path.stem}_gt.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    n_ok = 0
    n_bad = 0
    t0 = t1 = None
    x_min = y_min = z_min = float("inf")
    x_max = y_max = z_max = float("-inf")

    with out_path.open("w", newline="") as fcsv:
        writer = csv.writer(fcsv)
        writer.writerow(["timestamp_s", "x", "y", "z"])
        for ts_db, data in iter_messages(bag_path, args.topic):
            try:
                t, x, y, z = parse_point_stamped(bytes(data))
            except (struct.error, ValueError) as e:
                n_bad += 1
                if n_bad <= 3:
                    print(f"[WARN] 跳过一条消息 (ts={ts_db}): {e}")
                continue
            writer.writerow([f"{t:.9f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])
            n_ok += 1
            if t0 is None:
                t0 = t
            t1 = t
            x_min, x_max = min(x_min, x), max(x_max, x)
            y_min, y_max = min(y_min, y), max(y_max, y)
            z_min, z_max = min(z_min, z), max(z_max, z)
            if args.limit and n_ok >= args.limit:
                break

    if n_ok == 0:
        print(f"[ERROR] 话题 {args.topic} 在 {bag_path} 内没有可解析消息 (bad={n_bad})")
        return 1

    duration = (t1 or 0) - (t0 or 0)
    rate = n_ok / duration if duration > 0 else 0.0
    print(f"[OK] 写出 {n_ok} 条 GT (跳过 {n_bad} 条) → {out_path}")
    print(f"     时间: [{t0:.3f}, {t1:.3f}] s  duration={duration:.2f} s  avg_rate={rate:.1f} Hz")
    print(f"     x: [{x_min:.3f}, {x_max:.3f}]")
    print(f"     y: [{y_min:.3f}, {y_max:.3f}]")
    print(f"     z: [{z_min:.3f}, {z_max:.3f}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
