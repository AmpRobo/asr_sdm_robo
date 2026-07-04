#!/usr/bin/env python3
"""
Convert an MP4 video to an MCAP file at a fixed output rate.

Each MCAP message is a ROS 2 `sensor_msgs/msg/CompressedImage` (JPEG).

Prerequisites:
  This script uses ROS 2 Python packages (`rclpy`, `rosbag2_py`, `sensor_msgs`)
  and OpenCV. Source your ROS 2 environment before running:

  source /opt/ros/<distro>/setup.bash
  source /path/to/your/workspace/install/setup.bash   # if needed

  Install OpenCV for Python if it is not already available:

  sudo apt install python3-opencv                        # Debian/Ubuntu
  pip install opencv-python                              # alternative

Usage:
  python mp4_to_mcap.py [options]

Options:
  --input PATH         Input MP4 file (default: cv_1000.mp4)
  --output PATH        Output MCAP file or rosbag2 directory (default: video_enhancement_origin.mcap)
  --hz RATE            Output message rate in Hz (default: 20)
  --topic NAME         MCAP topic name (default: /video_enhancement/origin/compressed)
  --jpeg-quality N     JPEG quality in [0, 100] (default: 95)
  --width N            Output frame width in pixels (default: 640)
  --height N           Output frame height in pixels (default: 480)
  --frame-id NAME      ROS header frame_id (default: camera)

Examples:
  # Use defaults: cv_1000.mp4 -> video_enhancement_origin.mcap at 20 Hz
  python mp4_to_mcap.py

  # Specify input and output
  python mp4_to_mcap.py --input water.mp4 --output water.mcap

  # Custom rate, resolution, and topic
  python mp4_to_mcap.py \\
      --input cv_1000.mp4 \\
      --output /tmp/video_enhancement_origin.mcap \\
      --hz 10 \\
      --width 1280 \\
      --height 720 \\
      --topic /camera/compressed

Output:
  If --output ends with .mcap, the script writes a single MCAP file.
  Otherwise it writes a rosbag2 directory backed by MCAP storage.

  Inspect or replay the result with:

  ros2 bag info <output_path>
  ros2 bag play <output_path>
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import cv2
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import CompressedImage


def convert_mp4_to_mcap(
    input_path: Path,
    output_path: Path,
    output_hz: float,
    topic: str,
    quality: int,
    width: int,
    height: int,
    frame_id: str,
) -> None:
    if output_hz <= 0:
        raise ValueError("output_hz must be > 0")
    if not input_path.exists():
        raise FileNotFoundError(f"Input video not found: {input_path}")
    if not (0 <= quality <= 100):
        raise ValueError("quality must be in range [0, 100]")
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be > 0")

    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open input video: {input_path}")

    src_fps = cap.get(cv2.CAP_PROP_FPS)
    if not src_fps or src_fps <= 0:
        src_fps = 30.0

    src_dt = 1.0 / src_fps
    out_dt = 1.0 / output_hz

    output_path.parent.mkdir(parents=True, exist_ok=True)

    # rosbag2 writes to a bag URI (directory). If user asks for a single .mcap
    # file, write to a temporary bag dir first, then copy out the produced mcap.
    export_single_mcap = output_path.suffix.lower() == ".mcap"
    bag_uri = output_path.with_suffix("") if export_single_mcap else output_path
    if bag_uri.exists() and not bag_uri.is_dir():
        raise RuntimeError(f"Bag URI is not a directory path: {bag_uri}")
    if bag_uri.exists() and bag_uri.is_dir():
        shutil.rmtree(bag_uri)
    if export_single_mcap and output_path.exists() and output_path.is_file():
        output_path.unlink()

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(bag_uri), storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    writer.create_topic(
        TopicMetadata(
            id=0,
            name=topic,
            type="sensor_msgs/msg/CompressedImage",
            serialization_format="cdr",
        )
    )

    src_t = 0.0
    next_out_t = 0.0
    written = 0
    read_frames = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        read_frames += 1
        src_t = (read_frames - 1) * src_dt

        # Write this frame when we pass the next 20 Hz output tick.
        if src_t + 1e-12 >= next_out_t:
            resized = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            ok_enc, buf = cv2.imencode(
                ".jpg",
                resized,
                [cv2.IMWRITE_JPEG_QUALITY, quality],
            )
            if not ok_enc:
                raise RuntimeError("Failed to JPEG-encode frame")

            log_ns = int(next_out_t * 1e9)
            stamp_sec = log_ns // 1_000_000_000
            stamp_nsec = log_ns % 1_000_000_000

            msg = CompressedImage()
            msg.header.stamp = Time(sec=stamp_sec, nanosec=stamp_nsec)
            msg.header.frame_id = frame_id
            msg.format = "jpeg"
            msg.data = buf.tobytes()

            writer.write(topic, serialize_message(msg), log_ns)
            written += 1
            next_out_t += out_dt

    cap.release()

    if export_single_mcap:
        produced = sorted(bag_uri.glob("*.mcap"))
        if not produced:
            raise RuntimeError(f"No MCAP file generated under bag URI: {bag_uri}")
        shutil.copy2(produced[0], output_path)

    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")
    print(f"Source FPS: {src_fps:.6f}")
    print(f"Output rate: {output_hz:.6f} Hz")
    print(f"Output size: {width}x{height}")
    print(f"Topic: {topic} (sensor_msgs/msg/CompressedImage)")
    print(f"Frames read: {read_frames}")
    print(f"Messages written: {written}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert MP4 video to MCAP at a fixed output rate.",
        epilog=(
            "Examples:\n"
            "  %(prog)s\n"
            "  %(prog)s --input water.mp4 --output water.mcap\n"
            "  %(prog)s --input cv_1000.mp4 --output out.mcap --hz 10 --width 1280 --height 720"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("cv_1000.mp4"),
        help="Input MP4 path (default: cv_1000.mp4)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("video_enhancement_origin.mcap"),
        help="Output MCAP path (default: video_enhancement_origin.mcap)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=20.0,
        help="Output message rate in Hz (default: 20)",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/video_enhancement/origin/compressed",
        help="MCAP topic name",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG quality [0-100] (default: 95)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Output frame width (default: 640)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Output frame height (default: 480)",
    )
    parser.add_argument(
        "--frame-id",
        type=str,
        default="camera",
        help="ROS header frame_id (default: camera)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    convert_mp4_to_mcap(
        input_path=args.input,
        output_path=args.output,
        output_hz=args.hz,
        topic=args.topic,
        quality=args.jpeg_quality,
        width=args.width,
        height=args.height,
        frame_id=args.frame_id,
    )


if __name__ == "__main__":
    main()
