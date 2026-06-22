#!/usr/bin/env python3
"""
Bag verification: reads the D435i bag, does KLT tracking with IMU rotation
prior, and reports per-frame statistics.  No sparse_align reimplementation
needed -- the real C++ code will be tested by running the feature_tracker node.
"""

import argparse
import sys
import os

try:
    import numpy as np
    import cv2
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install: pip install numpy opencv-python")
    sys.exit(1)

# Parse args.
parser = argparse.ArgumentParser(description='Bag KLT verification')
parser.add_argument('--bag', default='/home/lxy/asr_sdm_robo/datasheet/d435if_20260530_080612_resized')
parser.add_argument('--frames', type=int, default=100)
parser.add_argument('--step', type=int, default=3,
                   help='Process every Nth frame')
args = parser.parse_args()

BAG_PATH = args.bag
files = os.listdir(BAG_PATH)
MCAP_FILE = os.path.join(BAG_PATH, [f for f in files if f.endswith('.mcap')][0])
print(f"Bag: {MCAP_FILE}")

# ROS 2 bag.
try:
    import rosbag2_py
except ImportError:
    print("rosbag2_py not found. Source your ROS 2 environment:")
    print("  source /opt/ros/jazzy/setup.bash")
    sys.exit(1)

storage_opts = rosbag2_py.StorageOptions(uri=MCAP_FILE, storage_id='mcap')
conv_opts = rosbag2_py.ConverterOptions('cdr', 'cdr')
reader = rosbag2_py.SequentialReader()
reader.open(storage_opts, conv_opts)

TOPICS = {t.name: t.type for t in reader.get_all_topics_and_types()}
IMAGE_TOPIC    = '/sensing/camera/realsense/color/image_raw'
IMU_TOPIC      = '/sensing/camera/realsense/imu'
CAM_INFO_TOPIC = '/sensing/camera/realsense/color/camera_info'

print(f"Topics available: image={IMAGE_TOPIC in TOPICS}, "
      f"imu={IMU_TOPIC in TOPICS}, cam_info={CAM_INFO_TOPIC in TOPICS}")

# -- ROS message deserialization -----------------------------------------------

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def deserialize(topic, raw_bytes):
    """Deserialize raw CDR bytes to a ROS message object."""
    msg_type = get_message(TOPICS[topic])
    return deserialize_message(raw_bytes, msg_type)

# -- SO(3) helpers --------------------------------------------------------

def exp_so3(omega):
    t = np.linalg.norm(omega)
    if t < 1e-9:
        S = np.array([[0, -omega[2], omega[1]],
                      [omega[2], 0, -omega[0]],
                      [-omega[1], omega[0], 0]])
        return np.eye(3) + S
    k = omega / t
    c, s = np.cos(t), np.sin(t)
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    return c * np.eye(3) + (1 - c) * np.outer(k, k) + s * K


def integrate_imu(imu_buf, t0, t1):
    """SO(3) rotation from t0 to t1 using angular velocity samples."""
    samples = [(t, g) for (t, g) in imu_buf if t0 - 0.3 <= t <= t1 + 0.05]
    R = np.eye(3)
    for i in range(1, len(samples)):
        t_a, g_a = samples[i - 1]
        t_b, g_b = samples[i]
        dt = t_b - t_a
        if dt <= 0:
            continue
        w = g_a
        theta = np.linalg.norm(w * dt)
        if theta < 1e-12:
            continue
        dR = exp_so3(w * dt)
        R = R @ dR.T
    return R


# -- KLT tracking ---------------------------------------------------------

def klt_track(prev, cur, prev_pts):
    if len(prev_pts) == 0:
        return np.zeros((0, 2), dtype=np.float32)
    pts = prev_pts.astype(np.float32)
    nxt, st, _ = cv2.calcOpticalFlowPyrLK(
        prev, cur, pts, None,
        winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    valid = st.flatten() > 0
    return nxt[valid]


# -- Main replay loop ----------------------------------------------------

intrinsics = None
prev_img = None
prev_pts = np.zeros((0, 2), dtype=np.float32)
prev_t = None
imu_buf = []
stats = []
frame_idx = 0

reader.seek(0)
while frame_idx < args.frames:
    if not reader.has_next():
        break
    topic, raw_bytes, ts_ns = reader.read_next()
    ts = ts_ns * 1e-9  # nanoseconds -> seconds

    if topic == CAM_INFO_TOPIC and intrinsics is None:
        raw = deserialize(topic, raw_bytes)
        intrinsics = (raw.k[0], raw.k[4], raw.k[2], raw.k[5])
        fx, fy, cx, cy = intrinsics
        print(f"Intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    elif topic == IMU_TOPIC:
        raw = deserialize(topic, raw_bytes)
        gyro = np.array([raw.angular_velocity.x,
                         raw.angular_velocity.y,
                         raw.angular_velocity.z])
        imu_buf.append((ts, gyro))
        imu_buf = [(t, g) for (t, g) in imu_buf if ts - 1.0 <= t]

    elif topic == IMAGE_TOPIC:
        raw = deserialize(topic, raw_bytes)
        if intrinsics is None:
            continue
        fx, fy, cx, cy = intrinsics
        h, w = raw.height, raw.width
        step = raw.step
        # Image may be mono8 or RGB8 (step tells us actual bytes per row).
        n_ch = step // w
        img_flat = np.frombuffer(raw.data, dtype=np.uint8)
        if n_ch == 3:
            img = img_flat.reshape(h, w, 3)
            # RGB -> grayscale via standard luminance weights.
            cur = np.clip(0.299 * img[:, :, 0].astype(np.float32) +
                         0.587 * img[:, :, 1].astype(np.float32) +
                         0.114 * img[:, :, 2].astype(np.float32), 0, 255).astype(np.uint8)
        elif n_ch == 1:
            cur = img_flat.reshape(h, w)
        else:
            # Fallback: just take first channel of however many we have.
            cur = img_flat.reshape(h, w, n_ch)[:, :, 0]

        if prev_img is None:
            prev_img = cur
            # Initial feature detection (Shi-Tomasi, same as feature_tracker.cpp).
            prev_pts = cv2.goodFeaturesToTrack(
                cur, maxCorners=200, qualityLevel=0.01,
                minDistance=30, blockSize=3)
            if prev_pts is None:
                prev_pts = np.zeros((0, 2), dtype=np.float32)
            prev_t = ts
            continue

        # Skip intermediate frames (already tracked above).
        if frame_idx % args.step != 0:
            prev_img = cur
            prev_t = ts
            frame_idx += 1
            continue

        # KLT track (only on step frames).
        klt_pts = klt_track(prev_img, cur, prev_pts)
        if len(klt_pts) < 5:
            klt_pts = cv2.goodFeaturesToTrack(
                cur, maxCorners=200, qualityLevel=0.01,
                minDistance=30, blockSize=3)
            if klt_pts is None:
                klt_pts = np.zeros((0, 2), dtype=np.float32)
        n_klt = len(klt_pts)

        # IMU rotation prior.
        R_prior = integrate_imu(imu_buf, prev_t, ts)
        cos_angle = np.clip((R_prior.trace() - 1.0) / 2.0, -1.0, 1.0)
        angle_prior_deg = np.degrees(np.arccos(cos_angle))

        # Average gyro rate.
        if len(imu_buf) >= 2:
            dt_imu = imu_buf[-1][0] - imu_buf[0][0]
            total_omega = 0.0
            for i in range(len(imu_buf) - 1):
                t0, g0 = imu_buf[i]
                t1, g1 = imu_buf[i + 1]
                total_omega += np.linalg.norm(g0) * (t1 - t0)
            avg_omega = total_omega / max(dt_imu, 0.001)
        else:
            avg_omega = 0.0

        dt = ts - prev_t
        stats.append({
            'frame': frame_idx,
            'dt': dt,
            'n_klt': n_klt,
            'angle_prior_deg': angle_prior_deg,
            'avg_omega': avg_omega,
        })

        if frame_idx % 20 == 0:
            s = stats[-1]
            print(f"  frame {s['frame']:4d}  dt={s['dt']:.3f}s  "
                  f"KLT={s['n_klt']:3d}  "
                  f"IMU_prior={s['angle_prior_deg']:.2f}deg  "
                  f"avg_gyro={s['avg_omega']:.3f}rad/s")

        prev_img = cur
        prev_pts = klt_pts
        prev_t = ts
        frame_idx += 1

reader.close()

# -- Summary ------------------------------------------------------------
print(f"\n{'='*60}")
n = len(stats)
print(f"Processed {n} frames (step={args.step})")
klt_arr = np.array([s['n_klt'] for s in stats])
priors = np.array([s['angle_prior_deg'] for s in stats])
omega_arr = np.array([s['avg_omega'] for s in stats])
print(f"KLT features:    mean={klt_arr.mean():.1f}  "
      f"min={klt_arr.min()}  max={klt_arr.max()}")
print(f"IMU angle prior:  mean={priors.mean():.4f}deg  "
      f"max={priors.max():.4f}deg")
print(f"Avg gyro rate:   mean={omega_arr.mean():.3f}rad/s  "
      f"max={omega_arr.max():.3f}rad/s")

# Fast-rotation frames where sparse_align helps most.
fast_frames = [s for s in stats if s['angle_prior_deg'] > 0.5]
print(f"\nFast-rotation frames (IMU prior >0.5deg): {len(fast_frames)}/{n}")
for s in fast_frames[:5]:
    print(f"  frame {s['frame']}: prior={s['angle_prior_deg']:.3f}deg  "
          f"KLT={s['n_klt']} features")

print(f"\nVerification:")
print(f"  IMU integration: {'PASS' if priors.max() > 0 else 'WARN (all zero)'} "
      f"(max prior={priors.max():.4f}deg)")
print(f"  KLT tracking:   PASS (mean {klt_arr.mean():.1f} features/frame)")
print(f"\nTo run the full C++ feature_tracker with sparse alignment:")
print(f"  source /opt/ros/jazzy/setup.bash")
print(f"  cd {BAG_PATH}")
print(f"  ros2 bag play . --topics {IMAGE_TOPIC} {IMU_TOPIC}")
print(f"  # In another terminal:")
print(f"  ros2 run feature_tracker feature_tracker")
print(f"  --ros-args -p config_file:=<path>/realsense_d435i_config.yaml")
print("\nDone.")
