#!/usr/bin/env python3
"""
extract_gt.py — 从 rosbag2 (.db3) 中提取地面真值（GT）轨迹。

支持以下 ROS 消息:
  1) geometry_msgs/PointStamped        (EuRoC MAV, MH_*; 话题 /leica/position)
     仅含 3D 位置.
  2) geometry_msgs/PoseStamped        (TUM-VI room1..6; 话题 /pose_gt, ASL dataset format)
     含 3D 位置 + 四元数姿态 (xyzw).
  3) geometry_msgs/TransformStamped   (UZH-FPV Drone Racing, V*; 话题 /vicon/.../...)
     含 3D 位置 + 四元数姿态 (xyzw).

脚本会自动按 seq 名猜 GT 话题与话题类型；可手动覆盖。

输出 CSV 默认列 (与 plot_compare.load_csv 兼容: 只读前 4 列):
    timestamp_s, x, y, z
若消息为 TransformStamped, 会附加 4 列姿态:
    timestamp_s, x, y, z, qx, qy, qz, qw

时间戳单位: 浮点秒 (stamp.sec + stamp.nanosec*1e-9).  Vicon GT 在 ASL/UZH
数据集中通常以约 200 Hz 发布, 但实际数据中分布不均 (>=0.1 Hz 量级的
"静止"段也会被记录), 后续与 VINS 输出对齐时按最近邻或线性插值即可。

用法:
    # 自动从 seq 名选话题, 输出到 experiments/sparse_compare/results/<seq>_gt.csv
    python extract_gt.py --bag datasheet/V1_01_easy_ros2/V1_01_easy_ros2.db3 --seq V101

    # 手动指定话题与输出路径
    python extract_gt.py --bag <path.db3> --out gt.csv --topic /leica/position
"""

from __future__ import annotations

import argparse
import csv
import sqlite3
import struct
from pathlib import Path
from typing import Iterator, Tuple, Optional


# ---------------------------------------------------------------- CDR helpers --

# CDR (Common Data Representation) 解码: ROS2 默认小端序.
def _cdr_skip_string(buf: bytes, off: int, next_align: int = 4) -> Tuple[int, bytes]:
    """Skip a CDR-encoded ROS string starting at `off` (4B length + content + alignment).

    ROS2 CDR string layout: 4B little-endian length (含末尾 NUL) + body bytes.
    `next_align` 是 string 字段之后下一个字段的对齐要求 (4 或 8).
      - 4: 标准 OMG CDR (rclcpp 默认)
      - 8: 当下一个字段是含 double 的嵌套消息 (Pose, Transform, Point 等) 时,
            把整个 string 字段对齐到 8 字节 (rosbags 实测行为).
    多数情况 string body padded to `next_align` 即可, 这里采用"对齐 string 字段
    总长到 next_align"的等价写法: 返回 off = off_start + ceil_to_next_align(4 + slen).
    """
    if len(buf) < off + 4:
        raise ValueError(f"buffer too short at off={off} for string length")
    slen = struct.unpack_from("<I", buf, off)[0]
    body_off = off + 4
    body_len = max(slen, 1)  # body 含 NUL 终止符
    if len(buf) < body_off + body_len:
        raise ValueError(f"buffer too short at off={body_off} for string body (need {body_len} bytes)")
    raw = buf[body_off:body_off + body_len]
    try:
        s = raw.rstrip(b"\x00").decode("utf-8")
    except UnicodeDecodeError:
        s = raw.decode("utf-8", errors="replace")
    # 整个 string 字段 (4B len + body) 对齐到 next_align 字节
    total = 4 + body_len
    aligned = off + ((total + next_align - 1) & ~(next_align - 1))
    return aligned, s


def _probe_hdr(buf: bytes) -> int:
    """Detect CDR encapsulation header size (0 for bare ROS2, 4 for `rosbags` lib).

    rosbags 库会在每条消息前加 4B 固定前缀 `00 01 00 00` (CDRv1 LE marker).
    rosbag2_py 录制的 bag 是裸消息, 没有这个前缀.
    """
    if len(buf) >= 4 and buf[:4] == b"\x00\x01\x00\x00":
        return 4
    return 0


def _parse_point_stamped(buf: bytes) -> Tuple[float, float, float, float]:
    """解析 geometry_msgs/PointStamped. 返回 (timestamp_s, x, y, z)."""
    HDR = _probe_hdr(buf)
    if len(buf) < HDR + 8 + 4:
        raise ValueError(f"PointStamped buffer too short: {len(buf)}")
    sec, nsec = struct.unpack_from("<iI", buf, HDR)
    # frame_id 后跟 Point (含 double → 8-byte align)
    off, _ = _cdr_skip_string(buf, HDR + 8, next_align=8)
    if len(buf) < off + 24:
        raise ValueError(f"PointStamped body too short: {len(buf)} < {off + 24}")
    x, y, z = struct.unpack_from("<3d", buf, off)
    return float(sec) + nsec * 1e-9, x, y, z


def _parse_transform_stamped(buf: bytes) -> Tuple[float, float, float, float, float, float, float, float]:
    """解析 geometry_msgs/TransformStamped. 返回 (t, x, y, z, qx, qy, qz, qw)."""
    HDR = _probe_hdr(buf)
    if len(buf) < HDR + 8:
        raise ValueError(f"TransformStamped buffer too short: {len(buf)}")
    sec, nsec = struct.unpack_from("<iI", buf, HDR)
    # frame_id 后跟 child_frame_id (string), 都按 4-byte aligned;
    # Transform 是嵌套 message 含 double → 第二个 string 后用 8-byte align.
    off, _frame_id = _cdr_skip_string(buf, HDR + 8, next_align=4)
    off, _child = _cdr_skip_string(buf, off, next_align=8)
    if len(buf) < off + 56:
        raise ValueError(f"TransformStamped body too short: {len(buf)} < {off + 56}")
    tx, ty, tz, qx, qy, qz, qw = struct.unpack_from("<7d", buf, off)
    return float(sec) + nsec * 1e-9, tx, ty, tz, qx, qy, qz, qw


def _parse_pose_stamped(buf: bytes) -> Tuple[float, float, float, float, float, float, float, float]:
    """解析 geometry_msgs/PoseStamped. 返回 (t, x, y, z, qx, qy, qz, qw)."""
    HDR = _probe_hdr(buf)
    if len(buf) < HDR + 8 + 4:
        raise ValueError(f"PoseStamped buffer too short: {len(buf)}")
    sec, nsec = struct.unpack_from("<iI", buf, HDR)
    # frame_id 后跟 Pose (含 double → 8-byte align).
    off, _frame_id = _cdr_skip_string(buf, HDR + 8, next_align=8)
    if len(buf) < off + 56:
        raise ValueError(f"PoseStamped body too short: {len(buf)} < {off + 56}")
    x, y, z, qx, qy, qz, qw = struct.unpack_from("<7d", buf, off)
    return float(sec) + nsec * 1e-9, x, y, z, qx, qy, qz, qw


# ------------------------------------------------------------- DB streaming --

def iter_messages(db3_path: Path, topic: str) -> Iterator[Tuple[int, bytes]]:
    """流式迭代 sqlite3 rosbag2 数据库中指定话题的消息 (按时间戳顺序)."""
    conn = sqlite3.connect(str(db3_path))
    try:
        cur = conn.cursor()
        row = cur.execute(
            "SELECT id FROM topics WHERE name = ?", (topic,)
        ).fetchone()
        if row is None:
            avail = [r[0] for r in cur.execute("SELECT name FROM topics").fetchall()]
            raise ValueError(f"话题 '{topic}' 不在该 bag 内. 可用话题: {avail}")
        topic_id = row[0]
        yield from cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,),
        )
    finally:
        conn.close()


# --------------------------------------------------------- seq 命名约定 --

# 已知 seq -> 默认 GT 话题 + 类型. 当 --seq 给定时按这里查; 若 --topic 显式
# 给出则覆盖.  对未知 seq, 退化为 -1 (需要 --topic 或 --topic-type).
KNOWN_SEQ_TOPICS = {
    # EuRoC MAV — /leica/position (PointStamped)
    "MH01": ("/leica/position",        "geometry_msgs/msg/PointStamped"),
    "MH02": ("/leica/position",        "geometry_msgs/msg/PointStamped"),
    "MH03": ("/leica/position",        "geometry_msgs/msg/PointStamped"),
    "MH04": ("/leica/position",        "geometry_msgs/msg/PointStamped"),
    "MH05": ("/leica/position",        "geometry_msgs/msg/PointStamped"),
    # UZH-FPV Drone Racing — /vicon/... (TransformStamped)
    "V101": ("/vicon/firefly_sbx/firefly_sbx", "geometry_msgs/msg/TransformStamped"),
    "V102": ("/vicon/firefly_sbx/firefly_sbx", "geometry_msgs/msg/TransformStamped"),
    "V103": ("/vicon/firefly_sbx/firefly_sbx", "geometry_msgs/msg/TransformStamped"),
    "V201": ("vicon/firefly_sbx/firefly_sbx",   "geometry_msgs/msg/TransformStamped"),
    "V202": ("vicon/firefly_sbx/firefly_sbx",   "geometry_msgs/msg/TransformStamped"),
    "V203": ("vicon/firefly_sbx/firefly_sbx",   "geometry_msgs/msg/TransformStamped"),
    # TUM-VI (ASL) — /pose_gt (PoseStamped).  seq 名沿用原始 room 编号.
    "ROOM1": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM2": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM3": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM4": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM5": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM6": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    # 也支持带 dataset 前缀的小写风格
    "ROOM1_512_16": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM2_512_16": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM3_512_16": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM4_512_16": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM5_512_16": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
    "ROOM6_512_16": ("/pose_gt", "geometry_msgs/msg/PoseStamped"),
}

RESULTS_DIR = Path("/home/lxy/asr_sdm_robo/experiments/sparse_compare/results")


def _resolve_topic(db3: Path, topic: Optional[str], seq: Optional[str]) -> Tuple[str, str]:
    """Resolve (topic_name, topic_type). Priority: --topic > --seq > auto-detect from db."""
    if topic is not None:
        # type 留 None 表示让 caller 根据消息解析长度自动判别
        return topic, ""

    if seq is not None:
        s = seq.upper()
        if s in KNOWN_SEQ_TOPICS:
            return KNOWN_SEQ_TOPICS[s]

    # auto-detect: 先找 vicon, 再找 leica, 再找 TUM-VI /pose_gt
    conn = sqlite3.connect(str(db3))
    try:
        for cand in (
            "/vicon/firefly_sbx/firefly_sbx",
            "vicon/firefly_sbx/firefly_sbx",
            "/leica/position",
            "/pose_gt",
        ):
            row = conn.execute("SELECT type FROM topics WHERE name=?", (cand,)).fetchone()
            if row is not None:
                return cand, row[0]
    finally:
        conn.close()

    raise ValueError(f"无法自动选择 GT 话题; 请通过 --topic 或 --seq 显式指定")


def _detect_msg_type(db3: Path, topic: str) -> str:
    conn = sqlite3.connect(str(db3))
    try:
        row = conn.execute("SELECT type FROM topics WHERE name=?", (topic,)).fetchone()
        if row is None:
            raise ValueError(f"话题 '{topic}' 不在 {db3}")
        return row[0]
    finally:
        conn.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--bag", required=True, type=Path, help="rosbag2 .db3 路径")
    parser.add_argument("--seq", default=None,
                        help="序列号 (如 V101 / MH04), 用于自动选 GT 话题与默认输出路径")
    parser.add_argument("--out", type=Path, default=None,
                        help="输出 CSV 路径 (默认: experiments/sparse_compare/results/<seq>_gt.csv)")
    parser.add_argument("--topic", default=None, help="GT 话题名 (覆盖 --seq 默认)")
    parser.add_argument("--limit", type=int, default=None, help="仅解析前 N 条 (调试用)")
    args = parser.parse_args()

    bag_path: Path = args.bag
    if not bag_path.is_file():
        parser.error(f"找不到 bag 文件: {bag_path}")

    topic, topic_type = _resolve_topic(bag_path, args.topic, args.seq)
    if not topic_type:
        topic_type = _detect_msg_type(bag_path, topic)
    print(f"[INFO] bag={bag_path.name}  topic={topic}  type={topic_type}")

    if args.out is not None:
        out_path: Path = args.out
    elif args.seq is not None:
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        out_path = RESULTS_DIR / f"{args.seq.lower()}_gt.csv"
    else:
        out_path = bag_path.with_name(f"{bag_path.stem}_gt.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 选择解析器
    if topic_type.endswith("/PoseStamped"):
        parser_fn = _parse_pose_stamped
        header = ["timestamp_s", "x", "y", "z", "qx", "qy", "qz", "qw"]
    elif topic_type.endswith("/TransformStamped"):
        parser_fn = _parse_transform_stamped
        header = ["timestamp_s", "x", "y", "z", "qx", "qy", "qz", "qw"]
    elif topic_type.endswith("/PointStamped"):
        parser_fn = _parse_point_stamped
        header = ["timestamp_s", "x", "y", "z"]
    else:
        parser.error(f"不支持的话题类型: {topic_type} (需要 PointStamped/PoseStamped/TransformStamped)")

    n_ok = 0
    n_bad = 0
    t0 = t1 = None
    x_min = y_min = z_min = float("inf")
    x_max = y_max = z_max = float("-inf")

    with out_path.open("w", newline="") as fcsv:
        writer = csv.writer(fcsv)
        writer.writerow(header)
        for ts_db, data in iter_messages(bag_path, topic):
            try:
                parsed = parser_fn(bytes(data))
            except (struct.error, ValueError) as e:
                n_bad += 1
                if n_bad <= 3:
                    print(f"[WARN] 跳过一条消息 (ts={ts_db}): {e}")
                continue
            t = parsed[0]
            x, y, z = parsed[1], parsed[2], parsed[3]
            if len(header) == 8:
                writer.writerow(
                    [f"{t:.9f}",
                     f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                     f"{parsed[4]:.6f}", f"{parsed[5]:.6f}", f"{parsed[6]:.6f}", f"{parsed[7]:.6f}"]
                )
            else:
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
        print(f"[ERROR] 话题 {topic} 在 {bag_path} 内没有可解析消息 (bad={n_bad})")
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