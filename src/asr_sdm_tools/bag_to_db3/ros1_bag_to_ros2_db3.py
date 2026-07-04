#!/usr/bin/env python3
"""
Convert a ROS1 .bag into a ROS2 rosbag2 (SQLite3 .db3) bag directory.

This script is a thin wrapper around the `rosbags` converter:
  python -m rosbags.convert ...

Why this approach:
  - Pure Python (no ROS1/ROS2 install required)
  - Handles ROS1 serialization -> ROS2 CDR serialization correctly
  - Produces a standard ROS2 bag directory containing metadata.yaml and *.db3

Dependencies:
  Create and activate a Python virtual environment, then install rosbags:

  # Create a virtual environment (once)
  python3 -m venv .venv

  # Activate the environment
  source .venv/bin/activate          # Linux / macOS (bash/zsh)
  .venv\\Scripts\\activate            # Windows (cmd)
  .venv\\Scripts\\Activate.ps1         # Windows (PowerShell)

  # Install dependency (inside the activated environment)
  pip install rosbags

  # Deactivate when finished
  deactivate

Usage:
  python ros1_bag_to_ros2_db3.py <input.bag> [output_dir] [options]

Arguments:
  input       Path to the input ROS1 .bag file (required)
  output_dir  Output ROS2 bag directory (optional; defaults to <name>_ros2/ next to the input)

Options:
  --overwrite              Delete the output directory if it already exists before converting
  --dst-typestore NAME     Destination ROS2 distro typestore (e.g. ros2_humble, ros2_iron, ros2_jazzy)
  --include-topic TOPIC    Convert only these topics (exact match; repeatable)
  --exclude-topic TOPIC    Exclude these topics (exact match; takes precedence over --include-topic)

Examples:
  # Activate the environment first, then run the converter
  source .venv/bin/activate

  # Specify input and output
  python ros1_bag_to_ros2_db3.py MH_01_easy.bag MH_01_easy_ros2

  # Input only; output defaults to MH_01_easy_ros2/
  python ros1_bag_to_ros2_db3.py MH_01_easy.bag

  # Overwrite an existing output directory
  python ros1_bag_to_ros2_db3.py input.bag output_ros2 --overwrite

  # Convert a subset of topics
  python ros1_bag_to_ros2_db3.py input.bag output_ros2 \\
      --include-topic /cam0/image_raw /imu0

  # Target a specific ROS2 distro
  python ros1_bag_to_ros2_db3.py input.bag output_ros2 --dst-typestore ros2_humble

Output:
  After conversion, output_dir contains metadata.yaml and *.db3 files,
  ready for playback with ros2 bag play.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def _default_dst_from_src(src: Path) -> Path:
    # ROS2 bags are directories. Keep it obvious and non-destructive.
    # e.g. foo.bag -> foo_ros2
    stem = src.name
    if stem.endswith(".bag"):
        stem = stem[: -len(".bag")]
    return src.with_name(f"{stem}_ros2")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Convert a ROS1 .bag into a ROS2 rosbag2 SQLite3 (.db3) bag directory (via rosbags).",
        epilog=(
            "Examples:\n"
            "  %(prog)s input.bag output_ros2\n"
            "  %(prog)s input.bag                    # writes to input_ros2/"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "input",
        type=Path,
        help="Path to input ROS1 .bag file.",
    )
    p.add_argument(
        "output",
        type=Path,
        nargs="?",
        default=None,
        help="Output ROS2 bag directory. If omitted, uses <input>_ros2 next to the source.",
    )
    p.add_argument(
        "--overwrite",
        action="store_true",
        help="If set, delete the output directory if it already exists.",
    )
    p.add_argument(
        "--dst-typestore",
        default=None,
        help=(
            "Optional rosbags typestore for the destination (e.g. ros2_humble, ros2_iron, ros2_jazzy). "
            "If omitted, rosbags will copy message definitions from the source."
        ),
    )
    p.add_argument(
        "--include-topic",
        nargs="*",
        default=None,
        help="Only include these topics (exact match).",
    )
    p.add_argument(
        "--exclude-topic",
        nargs="*",
        default=None,
        help="Exclude these topics (exact match). Exclusions take precedence.",
    )
    return p.parse_args()


def main() -> int:
    args = _parse_args()

    src: Path = args.input.expanduser().resolve()
    if not src.exists():
        print(f"ERROR: input does not exist: {src}", file=sys.stderr)
        return 2
    if src.suffix != ".bag":
        print(f"WARNING: input does not end with .bag: {src}", file=sys.stderr)

    dst: Path = (
        args.output if args.output is not None else _default_dst_from_src(src)
    ).expanduser().resolve()
    if dst.exists():
        if not args.overwrite:
            print(f"ERROR: output already exists (use --overwrite): {dst}", file=sys.stderr)
            return 2
        if dst.is_dir():
            shutil.rmtree(dst)
        else:
            dst.unlink()

    # Prefer the python module entrypoint. It is present whenever `rosbags` is installed.
    cmd = [
        sys.executable,
        "-m",
        "rosbags.convert",
        "--src",
        str(src),
        "--dst",
        str(dst),
        "--dst-storage",
        "sqlite3",
    ]

    if args.dst_typestore:
        cmd += ["--dst-typestore", args.dst_typestore]
    if args.include_topic:
        cmd += ["--include-topic", *args.include_topic]
    if args.exclude_topic:
        cmd += ["--exclude-topic", *args.exclude_topic]

    print("Running:")
    print("  " + " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        print("ERROR: Python executable not found when launching subprocess.", file=sys.stderr)
        return 3
    except subprocess.CalledProcessError as e:
        print(f"ERROR: conversion failed with exit code {e.returncode}", file=sys.stderr)
        return e.returncode

    db3_files = sorted(dst.glob("*.db3"))
    print(f"\nDone. ROS2 bag directory:\n  {dst}")
    if db3_files:
        print("DB3 file(s):")
        for p in db3_files:
            print(f"  {p}")
    else:
        print("NOTE: No *.db3 files found in destination; check conversion output above.", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


