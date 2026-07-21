#!/usr/bin/env python3
"""
EuRoC / TUM-VI dataset to ROS2 rosbag2 (.db3 folder) converter.

Supports BOTH layouts:

  Standard EuRoC / TUM-VI  (root/mav0/imu0/data.csv, ...)
        \
         root/mav0/cam0/data.csv    [header: "#timestamp [ns],filename"]
         root/mav0/cam1/data.csv
         root/mav0/imu0/data.csv     [header: "timestamp [ns] w_RS_S_x [...] ..."]
         root/mav0/state_groundtruth_estimate0/data.csv
                                  OR  root/mav0/mocap0/data.csv
                                  OR  root/mav0/groundtruth/data.csv

  "origin" / dso-export layout  (root/dso/...)
        \
         root/dso/imu.txt           [header: "# timestamp[ns] w.x w.y w.z a.x a.y a.z",
                                    space-separated]
         root/dso/gt_imu.csv        [header: "# timestamp[ns],tx,ty,tz,qw,qx,qy,qz",
                                    qw-first order]
         root/mav0/cam0/data.csv + root/mav0/cam0/data/<ns>.png
         root/mav0/cam1/data.csv + root/mav0/cam1/data/<ns>.png

Usage:
    python tumvi_to_ros2bag.py \\
        --input /path/to/dataset-room1_512_16_origin \\
        --output /path/to/output.db3 \\
        --overwrite

Dependencies:
    pip install rosbags numpy

Topics in output bag (default):
    /imu0                        sensor_msgs/msg/Imu
    /cam0/image_raw/compressed   sensor_msgs/msg/CompressedImage
    /cam1/image_raw/compressed   sensor_msgs/msg/CompressedImage
    /pose_gt                     geometry_msgs/msg/PoseStamped (mocap / ground truth)

With `--raw-image`, image topics become:
    /cam0/image_raw              sensor_msgs/msg/Image   (decoded from PNG)
    /cam1/image_raw              sensor_msgs/msg/Image   (decoded from PNG)

TUM-VI PNGs are 16-bit grayscale (`I;16`); when `--raw-image` is set
we convert them to 8-bit mono (`mono8`) by right-shifting 8 bits, which
matches what VINS-Mono feature_tracker expects.  This option requires
the Pillow package (`pip install pillow`).
"""

import argparse
import csv
import shutil
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).parent.resolve()


def parse_args():
    # Default output: <datasheet>/<dataset_name>.db3/
    default_output = None

    p = argparse.ArgumentParser(description="Convert EuRoC/TUM-VI dataset to ROS2 rosbag2")
    p.add_argument("--input", "-i", required=True,
                   help="Dataset root (contains mav0/ and/or dso/)")
    p.add_argument("--output", "-o", default=default_output,
                   help="Output rosbag2 FOLDER  [default: <datasheet>/<dataset-name>.db3/]")
    p.add_argument("--overwrite", action="store_true")
    p.add_argument("--raw-image", dest="raw_image", action="store_true",
                   help="Write camera topics as sensor_msgs/msg/Image "
                        "(mono8) on /camN/image_raw instead of "
                        "sensor_msgs/msg/CompressedImage.  Required for "
                        "VINS feature_tracker which only subscribes to "
                        "raw Image.  Requires Pillow.")
    parsed = p.parse_args()

    # Auto-generate output path if not given: <repo>/datasheet/<dataset-name>.db3
    if parsed.output is None:
        dataset_name = Path(parsed.input).resolve().name
        parsed.output = str(Path(__file__).parents[3] / "datasheet" / dataset_name)

    return parsed


# -----------------------------------------------------------------------------
# CSV readers
# -----------------------------------------------------------------------------
def read_csv(path: Path, delimiter: str = ","):
    """Read a delimited file.

    Header may start with '#' (which is stripped).  Returns list of dicts.
    """
    rows = []
    if not path.is_file():
        return rows
    with open(path, "r", encoding="utf-8") as f:
        # Header is the first non-blank line; it may or may not start with '#'.
        # We use it directly after stripping the leading '#' from the first
        # field.  This works for:
        #   - EuRoC cam CSVs: "#timestamp [ns],filename"
        #   - EuRoC/TUM-VI IMU/GT CSVs without leading '#'
        #   - TUM-VI origin/dso files where the entire header is one "# ..." line
        all_lines = f.readlines()
        header_idx = None
        for i, line in enumerate(all_lines):
            if line.strip():
                header_idx = i
                break
        if header_idx is None:
            return rows
        raw_header = all_lines[header_idx].strip()
        if raw_header.startswith("#"):
            raw_header = raw_header.lstrip("#").strip()
        header = [h.strip() for h in raw_header.split(delimiter)]
        for line in all_lines[header_idx + 1:]:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(delimiter)]
            if len(parts) != len(header):
                continue
            rows.append(dict(zip(header, parts)))
    return rows


def read_csv_with_stripped_hash_header(path: Path, delimiter: str = ","):
    """Backward-compatible reader: keeps the original '#' header parsing logic
    so that older EuRoC CSVs that look like ``#timestamp [ns],filename`` still
    load.  Use this for cam CSVs and GT in the standard layout.
    """
    rows = []
    if not path.is_file():
        return rows
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        raw_header = next(reader)
        header = [h.strip().lstrip("#") for h in raw_header]
        for row in reader:
            if len(row) != len(header):
                continue
            rows.append(dict(zip(header, [v.strip() for v in row])))
    return rows


def to_ns(value: str) -> int:
    """Convert a timestamp field to nanoseconds.

    Supports:
      - "1520530308181901469"                       (already ns)
      - "1520530308.1819017"                       (seconds.fraction -> ns)
      - "1520530308,181901469" or "... 181901469"   (seconds + ns suffix)
      - "Wed May 10 ..."                           (best effort: not supported, raises)
    """
    s = value.strip()
    if not s:
        raise ValueError("empty timestamp")

    # Already an integer-looking string in ns (>=1e15) or a clean integer
    try:
        n = int(s)
        # Heuristic: ns timestamps are > 1e15 (post-2015); s are < 1e11
        if n > 10**15:
            return n
        return int(n * 1e9)
    except ValueError:
        pass

    # ISO-like: split at '.' for seconds.fraction (or ',' / ' ')
    for sep in (".", ",", " "):
        if sep in s:
            head, _, frac = s.partition(sep)
            try:
                sec = int(head)
                pad = (frac + "0" * 9)[:9]
                return sec * 1_000_000_000 + int(pad)
            except ValueError:
                continue
    raise ValueError(f"Cannot parse timestamp: {value!r}")


# -----------------------------------------------------------------------------
# Column-name detection
# -----------------------------------------------------------------------------
def detect_imu_keys(row: dict) -> dict:
    """Detect column names for IMU data (angle rate + linear acceleration).

    Recognised layouts:
      1. EuRoC / TUM-VI standard:
         "timestamp [ns],w_x [rad s^-1],w_y [...],w_z [...],a_x [m s^-2],...,a_z [...]"
         (the *original* VIO column names also use w_RS_S_x / a_RS_S_x;
          we accept any "w_x"/"w_y"/"w_z"/"a_x"/"a_y"/"a_z" suffix)
      2. "origin" / dso export:
         "w.x","w.y","w.z","a.x","a.y","a.z"  (timestamp handled separately)
      3. ROS bag export:
         "angular_velocity.x","angular_velocity.y","angular_velocity.z",
         "linear_acceleration.x",...
    """
    keys = {}

    # Layout 1: official EuRoC/TUM-VI names (with or without the RS_S prefix)
    for k in row:
        kl = k.lower()
        if kl.startswith("w_rs_s_x") or kl.startswith("w_x") and "w_x" not in keys:
            if "w_x" not in keys:
                keys["wx"] = k
        elif kl.startswith("w_rs_s_y") or (kl.startswith("w_y") and "w_y" not in keys):
            if "w_y" not in keys:
                keys["wy"] = k
        elif kl.startswith("w_rs_s_z") or (kl.startswith("w_z") and "w_z" not in keys):
            if "w_z" not in keys:
                keys["wz"] = k
        elif kl.startswith("a_rs_s_x") or (kl.startswith("a_x") and "a_x" not in keys):
            if "a_x" not in keys:
                keys["ax"] = k
        elif kl.startswith("a_rs_s_y") or (kl.startswith("a_y") and "a_y" not in keys):
            if "a_y" not in keys:
                keys["ay"] = k
        elif kl.startswith("a_rs_s_z") or (kl.startswith("a_z") and "a_z" not in keys):
            if "a_z" not in keys:
                keys["az"] = k
    if all(v in keys for v in ("wx", "wy", "wz", "ax", "ay", "az")):
        return keys
    keys = {}

    # Layout 2: "w.x", "w.y", "w.z", "a.x", "a.y", "a.z"  (dso export)
    dot_names = {
        "w.x": "wx", "w.y": "wy", "w.z": "wz",
        "a.x": "ax", "a.y": "ay", "a.z": "az",
    }
    for k in row:
        kl = k.lower().strip()
        if kl in dot_names and dot_names[kl] not in keys:
            keys[dot_names[kl]] = k
    if all(v in keys for v in ("wx", "wy", "wz", "ax", "ay", "az")):
        return keys

    # Layout 3: ROS bag export
    for k in row:
        kl = k.lower()
        if kl == "angular_velocity.x": keys["wx"] = k
        elif kl == "angular_velocity.y": keys["wy"] = k
        elif kl == "angular_velocity.z": keys["wz"] = k
        elif kl == "linear_acceleration.x": keys["ax"] = k
        elif kl == "linear_acceleration.y": keys["ay"] = k
        elif kl == "linear_acceleration.z": keys["az"] = k
    return keys


def detect_gt_keys(row: dict) -> dict:
    """Detect column names for ground-truth pose data.

    Recognised layouts:
      1. EuRoC / TUM-VI standard: "p_RS_R_x [m]", "q_RS_R_x []", "q_RS_R_w []"
         (or the non-prefixed variant "p_RS_x", etc.)
      2. "origin" / dso export: "tx", "ty", "tz", "qw", "qx", "qy", "qz"
         (note: quat order is qw-first)
      3. ROS bag export: "transform.translation.x", "transform.rotation.x", ...
    Returns dict with keys: px, py, pz, qx, qy, qz, qw
                            (always ROS convention x,y,z,w).
    """
    keys = {}

    # 1. EuRoC / TUM-VI official
    suffixes = (
        ("p_RS_R_x", "px"), ("p_RS_R_y", "py"), ("p_RS_R_z", "pz"),
        ("q_RS_R_x", "qx"), ("q_RS_R_y", "qy"), ("q_RS_R_z", "qz"), ("q_RS_R_w", "qw"),
    )
    no_prefix = (
        ("p_RS_x", "px"), ("p_RS_y", "py"), ("p_RS_z", "pz"),
        ("q_RS_x", "qx"), ("q_RS_y", "qy"), ("q_RS_z", "qz"), ("q_RS_w", "qw"),
    )
    for prefix, key in no_prefix + suffixes:
        for col in row:
            if col.startswith(prefix) and col[len(prefix):].strip() in (" [m]", " []", "[m]", "[]"):
                keys[key] = col
                break
    if len(keys) == 7:
        return keys
    keys = {}

    # 2. "origin" / dso export: tx, ty, tz, qw, qx, qy, qz  (qw before qx/qy/qz)
    simple = (
        ("tx", "px"), ("ty", "py"), ("tz", "pz"),
        ("qx", "qx"), ("qy", "qy"), ("qz", "qz"), ("qw", "qw"),
    )
    for name, key in simple:
        for col in row:
            if col.lower().strip() == name:
                keys[key] = col
                break
    if len(keys) == 7:
        return keys
    keys = {}

    # 3. nested rosbag2 export
    nested = (
        ("transform.translation.x", "px"), ("transform.translation.y", "py"), ("transform.translation.z", "pz"),
        ("transform.rotation.x", "qx"), ("transform.rotation.y", "qy"), ("transform.rotation.z", "qz"),
        ("transform.rotation.w", "qw"),
    )
    for prefix, key in nested:
        if prefix in row:
            keys[key] = prefix
    return keys


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main() -> int:
    args = parse_args()
    dataset_root = Path(args.input).resolve()
    bag_out_dir = Path(args.output).resolve()

    # Strip trailing ".db3" so the internal sqlite file ends up as "<name>.db3"
    # (rosbag2.Writer appends ".db3" to its output dir name internally).
    if bag_out_dir.name.endswith(".db3"):
        bag_out_dir = bag_out_dir.with_name(bag_out_dir.name[:-4])

    mav0_root = dataset_root / "mav0"

    if not mav0_root.exists():
        print(f"[ERROR] mav0 folder not found: {mav0_root}")
        return 1

    if bag_out_dir.exists():
        if args.overwrite:
            if bag_out_dir.is_dir():
                shutil.rmtree(bag_out_dir)
            else:
                bag_out_dir.unlink()
        else:
            print(f"[ERROR] Output exists, use --overwrite: {bag_out_dir}")
            return 1
    # NOTE: do NOT mkdir here -- Writer requires the target path NOT to exist.

    # ---- Dataset paths (standard layout) ---------------------------------
    imu_csv_std  = mav0_root / "imu0" / "data.csv"
    cam0_csv_std = mav0_root / "cam0" / "data.csv"
    cam1_csv_std = mav0_root / "cam1" / "data.csv"
    cam0_dir_std = mav0_root / "cam0" / "data"
    cam1_dir_std = mav0_root / "cam1" / "data"
    gt_csv_std = None
    for cand in [
        mav0_root / "mocap0" / "data.csv",                             # EuRoC
        mav0_root / "state_groundtruth_estimate0" / "data.csv",        # TUM-VI
        mav0_root / "groundtruth" / "data.csv",
    ]:
        if cand.exists():
            gt_csv_std = cand
            break

    # ---- Dataset paths ("origin" / dso layout) ---------------------------
    dso_root = dataset_root / "dso"
    imu_dso  = dso_root / "imu.txt"
    gt_dso   = dso_root / "gt_imu.csv"

    # Pick IMU source
    if imu_dso.is_file():
        imu_path = imu_dso
        imu_delim = " "   # space-separated
        use_dso_imu = True
    elif imu_csv_std.is_file():
        imu_path = imu_csv_std
        imu_delim = ","
        use_dso_imu = False
    else:
        print(f"[ERROR] No IMU data found (looked for {imu_dso} and {imu_csv_std})")
        return 1

    # Pick GT source
    if gt_dso.is_file():
        gt_path = gt_dso
        gt_delim = ","
        use_dso_gt = True
    elif gt_csv_std is not None:
        gt_path = gt_csv_std
        gt_delim = ","
        use_dso_gt = False
    else:
        gt_path = None
        gt_delim = ","
        use_dso_gt = False

    # Cameras: only supported under mav0/
    if not cam0_csv_std.is_file() or not cam1_csv_std.is_file():
        print(f"[ERROR] cam0/cam1 data.csv not found under {mav0_root}")
        return 1

    print("[LOAD] Reading CSVs...")
    if use_dso_imu:
        imu_rows = read_csv(imu_path, delimiter=imu_delim)
    else:
        imu_rows = read_csv(imu_path, delimiter=imu_delim)
    cam0_rows = read_csv_with_stripped_hash_header(cam0_csv_std, delimiter=",")
    cam1_rows = read_csv_with_stripped_hash_header(cam1_csv_std, delimiter=",")
    gt_rows   = read_csv(gt_path, delimiter=gt_delim) if gt_path else []

    print(f"  IMU source : {imu_path}  ({'dso' if use_dso_imu else 'standard'})")
    print(f"  GT source  : {gt_path or '<none>'}  ({'dso' if use_dso_gt else 'standard' if gt_path else ''})")
    print(f"  IMU samples  : {len(imu_rows)}")
    print(f"  cam0 images  : {len(cam0_rows)}")
    print(f"  cam1 images  : {len(cam1_rows)}")
    print(f"  GT poses     : {len(gt_rows)}")

    if not imu_rows:
        print("[ERROR] No IMU data found")
        return 1

    # Detect IMU & GT column naming from first row
    imu_keys = detect_imu_keys(imu_rows[0])
    if not imu_keys:
        print(f"[ERROR] IMU columns not recognised: {list(imu_rows[0].keys())}")
        return 1
    print(f"  IMU keys     : {imu_keys}")

    gt_keys = {}
    if gt_rows:
        gt_keys = detect_gt_keys(gt_rows[0])
        if not gt_keys:
            print(f"[WARN] GT columns not recognised: {list(gt_rows[0].keys())}")
            gt_rows = []
        else:
            print(f"  GT keys      : {gt_keys}")

    if args.raw_image:
        try:
            from PIL import Image as _PILImage  # noqa: F401
        except ImportError:
            print("[ERROR] --raw-image requires Pillow.  pip install pillow")
            return 1
        print("  image mode   : raw sensor_msgs/msg/Image (mono8, /camN/image_raw)")

    # ROS2 message classes
    sys.path.insert(0, str(SCRIPT_DIR / ".venv" / "lib" / "python3.12" / "site-packages"))
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import get_typestore, Stores

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    TimeCls        = typestore.get_msgdef("builtin_interfaces/msg/Time").cls
    HeaderCls      = typestore.get_msgdef("std_msgs/msg/Header").cls
    Vector3Cls     = typestore.get_msgdef("geometry_msgs/msg/Vector3").cls
    QuatCls        = typestore.get_msgdef("geometry_msgs/msg/Quaternion").cls
    PoseCls        = typestore.get_msgdef("geometry_msgs/msg/Pose").cls
    ImuCls         = typestore.get_msgdef("sensor_msgs/msg/Imu").cls
    CompImgCls     = typestore.get_msgdef("sensor_msgs/msg/CompressedImage").cls
    ImageCls       = typestore.get_msgdef("sensor_msgs/msg/Image").cls
    PoseStampedCls = typestore.get_msgdef("geometry_msgs/msg/PoseStamped").cls

    def make_header(ns: int, frame_id: str):
        return HeaderCls(
            stamp=TimeCls(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000),
            frame_id=frame_id,
        )

    def make_imu_msg(ns, wx, wy, wz, ax, ay, az):
        return ImuCls(
            header=make_header(ns, "imu0"),
            orientation=QuatCls(x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=np.zeros(9, dtype=np.float64),
            angular_velocity=Vector3Cls(x=wx, y=wy, z=wz),
            angular_velocity_covariance=np.zeros(9, dtype=np.float64),
            linear_acceleration=Vector3Cls(x=ax, y=ay, z=az),
            linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
        )

    def make_comp_img_msg(ns, frame_id, img_bytes, fmt):
        return CompImgCls(
            header=make_header(ns, frame_id),
            format=fmt,
            data=np.frombuffer(img_bytes, dtype=np.uint8),
        )

    def make_raw_img_msg(ns, frame_id, png_bytes):
        """Decode a 16-bit grayscale PNG and return a sensor_msgs/msg/Image
        (mono8, right-shifted by 8).  Matches VINS feature_tracker expectations."""
        from PIL import Image as _PILImage
        import io as _io
        with _PILImage.open(_io.BytesIO(png_bytes)) as im:
            im.load()
            if im.mode in ("I", "I;16"):
                # TUM-VI stores 16-bit grayscale PNGs (mode I or I;16).
                # Right-shift by 8 to get 0-255 mono8.
                arr = np.asarray(im, dtype=np.uint16) >> 8
                height, width = arr.shape
                data = arr.astype(np.uint8).tobytes()
                encoding = "mono8"
            elif im.mode in ("L", "8bitL"):
                arr = np.asarray(im, dtype=np.uint8)
                height, width = arr.shape
                data = arr.tobytes()
                encoding = "mono8"
            elif im.mode == "RGB":
                arr = np.asarray(im, dtype=np.uint8)
                height, width, _ = arr.shape
                data = arr.tobytes()
                encoding = "rgb8"
            else:
                arr = np.asarray(im.convert("L"), dtype=np.uint8)
                height, width = arr.shape
                data = arr.tobytes()
                encoding = "mono8"
        return ImageCls(
            header=make_header(ns, frame_id),
            height=int(height),
            width=int(width),
            encoding=encoding,
            is_bigendian=0,
            step=int(width * (1 if encoding == "mono8" else 3)),
            data=np.frombuffer(data, dtype=np.uint8),
        )

    def make_pose_msg(ns, px, py, pz, qx, qy, qz, qw):
        return PoseStampedCls(
            header=make_header(ns, "world"),
            pose=PoseCls(
                position=Vector3Cls(x=px, y=py, z=pz),
                orientation=QuatCls(x=qx, y=qy, z=qz, w=qw),
            ),
        )

    # Open rosbag2 writer
    print(f"\n[BAG] Writing rosbag2 to {bag_out_dir}")
    writer = Writer(bag_out_dir, version=9)
    writer.open()

    if args.raw_image:
        cam0_topic, cam0_type = "/cam0/image_raw", "sensor_msgs/msg/Image"
        cam1_topic, cam1_type = "/cam1/image_raw", "sensor_msgs/msg/Image"
    else:
        cam0_topic, cam0_type = "/cam0/image_raw/compressed", "sensor_msgs/msg/CompressedImage"
        cam1_topic, cam1_type = "/cam1/image_raw/compressed", "sensor_msgs/msg/CompressedImage"

    conn_imu  = writer.add_connection("/imu0", "sensor_msgs/msg/Imu", typestore=typestore)
    conn_cam0 = writer.add_connection(cam0_topic, cam0_type, typestore=typestore)
    conn_cam1 = writer.add_connection(cam1_topic, cam1_type, typestore=typestore)
    conn_gt   = writer.add_connection("/pose_gt", "geometry_msgs/msg/PoseStamped",
                                      typestore=typestore) if gt_rows else None

    topic_counts = {"/imu0": 0, cam0_topic: 0, cam1_topic: 0, "/pose_gt": 0}
    total = len(imu_rows) + len(cam0_rows) + len(cam1_rows) + len(gt_rows)
    done = 0

    def report(label):
        pct = int(100 * done / total) if total else 0
        print(f"\r  [{pct:3d}%] {label} | {done}/{total}", end="", flush=True)

    def write_msg(conn, msg, typename, ts_ns, topic):
        nonlocal done
        data = typestore.serialize_cdr(msg, typename)
        writer.write(conn, ts_ns, data.tobytes() if hasattr(data, "tobytes") else bytes(data))
        topic_counts[topic] += 1
        done += 1

    # ---- IMU ----
    print("\n[WRITE] IMU ...")
    wxk = imu_keys["wx"]; wyk = imu_keys["wy"]; wzk = imu_keys["wz"]
    axk = imu_keys["ax"]; ayk = imu_keys["ay"]; azk = imu_keys["az"]
    # dso files use '# timestamp[ns]' which is the *first* column in the dict;
    # standard files use 'timestamp [ns]'.  Handle both.
    ts_col_imu = next((c for c in imu_rows[0]
                       if c.lower().replace(" ", "").startswith("timestamp")), None)
    if ts_col_imu is None:
        # fallback: first column
        ts_col_imu = next(iter(imu_rows[0].keys()))
    for row in imu_rows:
        ts_ns = to_ns(row[ts_col_imu])
        msg = make_imu_msg(
            ts_ns,
            float(row[wxk]), float(row[wyk]), float(row[wzk]),
            float(row[axk]), float(row[ayk]), float(row[azk]),
        )
        write_msg(conn_imu, msg, "sensor_msgs/msg/Imu", ts_ns, "/imu0")
        report("IMU")

    # ---- cam0 ----
    print("\n[WRITE] cam0 ...")
    cam0_dir = cam0_dir_std
    for row in cam0_rows:
        ts_ns = to_ns(row["timestamp [ns]"])
        img_path = cam0_dir / row["filename"]
        if not img_path.is_file():
            done += 1
            report("cam0 (skip)")
            continue
        with open(img_path, "rb") as f:
            raw = f.read()
        if args.raw_image:
            msg = make_raw_img_msg(ts_ns, "cam0", raw)
            typename = "sensor_msgs/msg/Image"
        else:
            msg = make_comp_img_msg(ts_ns, "cam0", raw, img_path.suffix.lower().lstrip("."))
            typename = "sensor_msgs/msg/CompressedImage"
        write_msg(conn_cam0, msg, typename, ts_ns, cam0_topic)
        report("cam0")

    # ---- cam1 ----
    print("\n[WRITE] cam1 ...")
    cam1_dir = cam1_dir_std
    for row in cam1_rows:
        ts_ns = to_ns(row["timestamp [ns]"])
        img_path = cam1_dir / row["filename"]
        if not img_path.is_file():
            done += 1
            report("cam1 (skip)")
            continue
        with open(img_path, "rb") as f:
            raw = f.read()
        if args.raw_image:
            msg = make_raw_img_msg(ts_ns, "cam1", raw)
            typename = "sensor_msgs/msg/Image"
        else:
            msg = make_comp_img_msg(ts_ns, "cam1", raw, img_path.suffix.lower().lstrip("."))
            typename = "sensor_msgs/msg/CompressedImage"
        write_msg(conn_cam1, msg, typename, ts_ns, cam1_topic)
        report("cam1")

    # ---- GT ----
    if gt_rows and gt_keys:
        print("\n[WRITE] GT ...")
        pkx = gt_keys["px"]; pky = gt_keys["py"]; pkz = gt_keys["pz"]
        qkx = gt_keys["qx"]; qky = gt_keys["qy"]; qkz = gt_keys["qz"]; qkw = gt_keys["qw"]
        ts_col_gt = next((c for c in gt_rows[0]
                          if c.lower().replace(" ", "").startswith("timestamp")), None)
        if ts_col_gt is None:
            ts_col_gt = next(iter(gt_rows[0].keys()))
        for row in gt_rows:
            ts_ns = to_ns(row[ts_col_gt])
            msg = make_pose_msg(
                ts_ns,
                float(row[pkx]), float(row[pky]), float(row[pkz]),
                float(row[qkx]), float(row[qky]), float(row[qkz]), float(row[qkw]),
            )
            write_msg(conn_gt, msg, "geometry_msgs/msg/PoseStamped", ts_ns, "/pose_gt")
            report("GT")

    writer.close()
    print(f"\n\n[DONE] Wrote {done} messages")

    total_size = sum(f.stat().st_size for f in bag_out_dir.rglob("*") if f.is_file())
    print(f"\nOutput bag: {bag_out_dir}")
    print(f"Size: {total_size / 1024**2:.1f} MB")
    print("Topics:")
    for topic, count in topic_counts.items():
        if count > 0:
            print(f"  {topic}: {count} msgs")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
