#!/usr/bin/env python3
"""
EuRoC MAV dataset to ROS2 rosbag2 (SQLite3 .db3) converter.

Usage:
    python euroc_to_ros2bag.py \\
        --input /path/to/dataset-room1_512_16 \\
        --output /path/to/output.db3

Dependencies (use the existing venv):
    cd src/asr_sdm_tools/bag_to_db3
    source .venv/bin/activate
    pip install rosbags pyyaml

Topics in output bag:
    /imu0                        - sensor_msgs/Imu
    /cam0/image_raw/compressed   - sensor_msgs/CompressedImage
    /cam1/image_raw/compressed   - sensor_msgs/CompressedImage
    /pose_gt                     - geometry_msgs/PoseStamped  (mocap ground truth)
"""

import argparse
import csv
import os
import shutil
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description="Convert EuRoC MAV dataset to ROS2 rosbag2 (.db3)")
    parser.add_argument("--input", "-i", required=True, help="Path to EuRoC dataset root (contains mav0/, dso/)")
    parser.add_argument("--output", "-o", required=True, help="Output ROS2 bag directory (will be created, e.g. room1_512_16.db3/)")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output if it exists")
    return parser.parse_args()


def read_csv(csv_path):
    """Read CSV with #header and return list of dicts."""
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        if header[0].startswith("#"):
            header[0] = header[0].lstrip("#")
        header = [h.strip() for h in header]
        for row in reader:
            if len(row) < len(header):
                continue
            rows.append(dict(zip(header, [v.strip() for v in row])))
    return rows


def ns_to_sec(ns_str):
    return float(ns_str) * 1e-9


def ts_ns_to_ros2_header(ns_str):
    """Return (sec, nanosec) for ROS2 Header stamp from nanosecond string."""
    ns = int(ns_str)
    return ns // 1_000_000_000, ns % 1_000_000_000


def main():
    args = parse_args()
    dataset_root = Path(args.input)
    output_path = Path(args.output)

    mav0 = dataset_root / "mav0"
    dso = dataset_root / "dso"

    if not mav0.exists():
        print(f"Error: mav0 not found at {mav0}")
        return 1
    if output_path.exists() and not args.overwrite:
        print(f"Error: output exists (use --overwrite to overwrite): {output_path}")
        return 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        shutil.rmtree(output_path) if output_path.is_dir() else output_path.unlink()

    # --- Load data ---
    print("Loading dataset...")
    imu_rows = read_csv(mav0 / "imu0" / "data.csv")
    cam0_rows = read_csv(mav0 / "cam0" / "data.csv")
    cam1_rows = read_csv(mav0 / "cam1" / "data.csv")
    mocap_rows = read_csv(mav0 / "mocap0" / "data.csv") if (mav0 / "mocap0" / "data.csv").exists() else []
    print(f"  IMU samples : {len(imu_rows)}")
    print(f"  cam0 images : {len(cam0_rows)}")
    print(f"  cam1 images : {len(cam1_rows)}")
    print(f"  mocap samples: {len(mocap_rows)}")

    # --- Import rosbags ---
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import get_typestore, Stores

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Message classes
    Imu_cls = typestore.get_msgdef("sensor_msgs/msg/Imu").cls
    CompressedImage_cls = typestore.get_msgdef("sensor_msgs/msg/CompressedImage").cls
    PoseStamped_cls = typestore.get_msgdef("geometry_msgs/msg/PoseStamped").cls
    Header_cls = typestore.get_msgdef("std_msgs/msg/Header").cls
    Quaternion_cls = typestore.get_msgdef("geometry_msgs/msg/Quaternion").cls
    Vector3_cls = typestore.get_msgdef("geometry_msgs/msg/Vector3").cls
    Pose_cls = typestore.get_msgdef("geometry_msgs/msg/Pose").cls
    Time_cls = typestore.get_msgdef("builtin_interfaces/msg/Time").cls

    msgdef_Imu = typestore.get_msgdef("sensor_msgs/msg/Imu")
    msgdef_CompressedImage = typestore.get_msgdef("sensor_msgs/msg/CompressedImage")
    msgdef_PoseStamped = typestore.get_msgdef("geometry_msgs/msg/PoseStamped")

    # --- Create bag writer ---
    print(f"\nWriting ROS2 bag to {output_path}...")
    writer = Writer(output_path, version=9)
    writer.open()

    conn_imu = writer.add_connection("/imu0", "sensor_msgs/msg/Imu", typestore=typestore)
    conn_cam0 = writer.add_connection("/cam0/image_raw/compressed", "sensor_msgs/msg/CompressedImage", typestore=typestore)
    conn_cam1 = writer.add_connection("/cam1/image_raw/compressed", "sensor_msgs/msg/CompressedImage", typestore=typestore)
    conn_mocap = None
    if mocap_rows:
        conn_mocap = writer.add_connection("/pose_gt", "geometry_msgs/msg/PoseStamped", typestore=typestore)

    total = len(imu_rows) + len(cam0_rows) + len(cam1_rows) + len(mocap_rows)
    prog = 0

    def progress(label, inc):
        nonlocal prog
        prog += inc
        pct = 100 * prog // total
        print(f"\r  [{pct:3d}%] {label}", end="", flush=True)

    # --- IMU ---
    print()
    for row in imu_rows:
        ts_ns = int(row["timestamp [ns]"])
        sec, nsec = ts_ns // 1_000_000_000, ts_ns % 1_000_000_000

        msg = Imu_cls(
            header=Header_cls(stamp=ts_ns, frame_id="imu0"),
            orientation=Quaternion_cls(x=0.0, y=0.0, z=0.0, w=0.0),
            orientation_covariance=[0.0] * 9,
            angular_velocity=Vector3_cls(
                x=float(row["w_RS_S_x [rad s^-1]"]),
                y=float(row["w_RS_S_y [rad s^-1]"]),
                z=float(row["w_RS_S_z [rad s^-1]"]),
            ),
            angular_velocity_covariance=[0.0] * 9,
            linear_acceleration=Vector3_cls(
                x=float(row["a_RS_S_x [m s^-2]"]),
                y=float(row["a_RS_S_y [m s^-2]"]),
                z=float(row["a_RS_S_z [m s^-2]"]),
            ),
            linear_acceleration_covariance=[0.0] * 9,
        )
        data = msgdef_Imu.serialize_ros1(msg)
        writer.write(conn_imu, ts_ns, data)
        progress(f"IMU ({prog}/{total})", 1)

    # --- cam0 ---
    print()
    cam0_dir = mav0 / "cam0" / "data"
    for row in cam0_rows:
        ts_ns = int(row["timestamp [ns]"])

        img_path = cam0_dir / row["filename"]
        if not img_path.exists():
            print(f"\n  Warning: {img_path} not found, skipping")
            progress(f"cam0 ({prog}/{total})", 1)
            continue

        with open(img_path, "rb") as f:
            img_bytes = f.read()

        msg = CompressedImage_cls(
            header=Header_cls(stamp=ts_ns, frame_id="cam0"),
            format="png",
            data=np.frombuffer(img_bytes, dtype=np.uint8),
        )
        data = msgdef_CompressedImage.serialize_ros1(msg)
        writer.write(conn_cam0, ts_ns, data)
        progress(f"cam0 ({prog}/{total})", 1)

    # --- cam1 ---
    print()
    cam1_dir = mav0 / "cam1" / "data"
    for row in cam1_rows:
        ts_ns = int(row["timestamp [ns]"])

        img_path = cam1_dir / row["filename"]
        if not img_path.exists():
            print(f"\n  Warning: {img_path} not found, skipping")
            progress(f"cam1 ({prog}/{total})", 1)
            continue

        with open(img_path, "rb") as f:
            img_bytes = f.read()

        msg = CompressedImage_cls(
            header=Header_cls(stamp=ts_ns, frame_id="cam1"),
            format="png",
            data=np.frombuffer(img_bytes, dtype=np.uint8),
        )
        data = msgdef_CompressedImage.serialize_ros1(msg)
        writer.write(conn_cam1, ts_ns, data)
        progress(f"cam1 ({prog}/{total})", 1)

    # --- mocap ---
    if mocap_rows:
        print()
        for row in mocap_rows:
            ts_ns = int(row["timestamp [ns]"])

            msg = PoseStamped_cls(
                header=Header_cls(stamp=ts_ns, frame_id="world"),
                pose=Pose_cls(
                    position=Vector3_cls(
                        x=float(row["p_RS_R_x [m]"]),
                        y=float(row["p_RS_R_y [m]"]),
                        z=float(row["p_RS_R_z [m]"]),
                    ),
                    orientation=Quaternion_cls(
                        x=float(row["q_RS_x []"]),
                        y=float(row["q_RS_y []"]),
                        z=float(row["q_RS_z []"]),
                        w=float(row["q_RS_w []"]),
                    ),
                ),
            )
            data = msgdef_PoseStamped.serialize_ros1(msg)
            writer.write(conn_mocap, ts_ns, data)
            progress(f"mocap ({prog}/{total})", 1)

    writer.close()

    size_mb = os.path.getsize(output_path) / (1024 ** 2)
    print(f"\n\nDone! ROS2 bag written to: {output_path}")
    print(f"Bag size: {size_mb:.1f} MB")
    print(f"\nTopics:")
    print(f"  /imu0                        → sensor_msgs/Imu")
    print(f"  /cam0/image_raw/compressed   → sensor_msgs/CompressedImage")
    print(f"  /cam1/image_raw/compressed   → sensor_msgs/CompressedImage")
    print(f"  /pose_gt                     → geometry_msgs/PoseStamped  (mocap GT)")

    return 0


if __name__ == "__main__":
    exit(main())
