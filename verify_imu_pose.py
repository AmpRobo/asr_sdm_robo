#!/usr/bin/env python3
"""
Verify IMU rotation vs SVO pose consistency.

Steps:
1. Run SVO with the bag (ros2 bag play + ros2 launch svo_ros)
2. Capture /svo/pose to CSV: ros2 topic echo /svo/pose --csv > svo_poses.csv
3. This script: parses IMU from bag, compares IMU-integrated rotation vs SVO pose rotation

Key question: Does IMU-predicted rotation match SVO-computed rotation?
"""

import sqlite3
import struct
import math
import numpy as np


def parse_ros1_imu(blob):
    """Parse ROS1 sensor_msgs/Imu binary. Returns (wx, wy, wz) in rad/s."""
    offset = 20  # skip seq(4) + stamp(8) + frame_id_len(4) + frame_id(4)
    wx = struct.unpack_from('<d', blob, offset)[0]
    wy = struct.unpack_from('<d', blob, offset + 8)[0]
    wz = struct.unpack_from('<d', blob, offset + 16)[0]
    return wx, wy, wz


def load_imu(bag_path):
    """Load IMU as (ts_s, wx, wy, wz) numpy arrays."""
    conn = sqlite3.connect(bag_path)
    cur = conn.cursor()
    cur.execute("""
        SELECT m.timestamp, m.data FROM messages m
        JOIN topics t ON m.topic_id = t.id
        WHERE t.name = 'imu/data'
        ORDER BY m.timestamp
    """)
    rows = []
    for ts, data in cur.fetchall():
        wx, wy, wz = parse_ros1_imu(data)
        rows.append((ts / 1e9, wx, wy, wz))
    conn.close()
    return np.array(rows, dtype=np.float64)


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quaternion_to_rpy(q):
    w, x, y, z = q
    sinr = 2*(w*x + y*z); cosr = 1 - 2*(x*x + y*y); roll = math.atan2(sinr, cosr)
    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1: pitch = math.copysign(math.pi/2, sinp)
    else: pitch = math.asin(sinp)
    siny = 2*(w*z + x*y); cosy = 1 - 2*(y*y + z*z); yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def integrate_imu(imu, t_start, t_end):
    """
    Integrate IMU gyroscope from t_start to t_end.
    Returns: quaternion [w,x,y,z] representing rotation from start to end,
            and dict of per-axis total angle.
    """
    mask = (imu[:, 0] >= t_start) & (imu[:, 0] <= t_end)
    chunk = imu[mask]
    if len(chunk) < 2:
        return None

    q = np.array([1.0, 0.0, 0.0, 0.0])
    total_roll = 0.0
    total_pitch = 0.0
    total_yaw = 0.0

    for i in range(len(chunk) - 1):
        t1, wx, wy, wz = chunk[i]
        t2 = chunk[i + 1][0]
        dt = t2 - t1
        if dt <= 0 or dt > 0.05:
            continue

        omega = np.array([wx, wy, wz])
        theta = np.linalg.norm(omega) * dt
        if theta < 1e-10:
            continue

        axis = omega / np.linalg.norm(omega)
        sin_half = math.sin(theta / 2)
        cos_half = math.cos(theta / 2)
        dq = np.array([cos_half, sin_half*axis[0], sin_half*axis[1], sin_half*axis[2]])
        q = quaternion_multiply(q, dq)

        # Accumulate angle per axis
        total_roll  += wx * dt
        total_pitch += wy * dt
        total_yaw   += wz * dt

    return q, {'roll': total_roll, 'pitch': total_pitch, 'yaw': total_yaw}


def parse_svo_poses(csv_path):
    """Parse SVO pose CSV from ros2 topic echo --csv.

    Format: stamp_sec, stamp_nsec, frame_id, x, y, z, qw, qx, qy, qz, [...covariance...]
    """
    poses = []
    with open(csv_path) as f:
        for line in f:
            if not line.strip() or line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if len(parts) < 10:
                continue
            try:
                sec = int(parts[0])
                nsec = int(parts[1])
                t = sec + nsec * 1e-9
                qw = float(parts[6])
                qx = float(parts[7])
                qy = float(parts[8])
                qz = float(parts[9])
                r, p, y = quaternion_to_rpy(np.array([qw, qx, qy, qz]))
                poses.append({'t': t, 'roll': r, 'pitch': p, 'yaw': y,
                              'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
                              'x': float(parts[3]), 'y': float(parts[4]), 'z': float(parts[5])})
            except (ValueError, IndexError):
                continue
    return poses


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path')
    parser.add_argument('--svo-csv', default=None,
                        help='CSV from ros2 topic echo /svo/pose --csv')
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()

    print(f"Loading IMU from {args.bag_path}...")
    imu = load_imu(args.bag_path)
    print(f"  {len(imu)} IMU measurements, t=[{imu[0,0]:.2f}, {imu[-1,0]:.2f}]s")

    # IMU summary
    omega = np.sqrt(imu[:,1]**2 + imu[:,2]**2 + imu[:,3]**2)
    print(f"\n=== IMU Gyroscope ===")
    print(f"  |ω|: mean={omega.mean():.4f} std={omega.std():.4f} max={omega.max():.4f} rad/s")
    print(f"  wx: mean={imu[:,1].mean():+.4f}  std={imu[:,1].std():.4f}")
    print(f"  wy: mean={imu[:,2].mean():+.4f}  std={imu[:,2].std():.4f}")
    print(f"  wz: mean={imu[:,3].mean():+.4f}  std={imu[:,3].std():.4f}")

    # Integrate rotation in chunks (every 5 seconds)
    print(f"\n=== IMU-Integrated Rotation (chunk=5s) ===")
    print(f"  {'t_start':>8} {'t_end':>8} {'Δroll(°)':>10} {'Δpitch(°)':>11} {'Δyaw(°)':>10} {'|Δrot|(°)':>11}")
    t = imu[0, 0]
    while t < imu[-1, 0] and t < imu[0, 0] + args.duration:
        t_end = min(t + 5.0, imu[-1, 0])
        result = integrate_imu(imu, t, t_end)
        if result is not None:
            _, angles = result
            r_deg = math.degrees(angles['roll'])
            p_deg = math.degrees(angles['pitch'])
            y_deg = math.degrees(angles['yaw'])
            total = math.sqrt(angles['roll']**2 + angles['pitch']**2 + angles['yaw']**2)
            print(f"  {t:8.2f} {t_end:8.2f} {r_deg:+10.2f} {p_deg:+11.2f} {y_deg:+10.2f} {math.degrees(total):+11.2f}")
        t = t_end

    # If SVO CSV provided, compare
    if args.svo_csv:
        print(f"\nLoading SVO poses from {args.svo_csv}...")
        poses = parse_svo_poses(args.svo_csv)
        print(f"  {len(poses)} poses loaded")
        if poses:
            t0 = poses[0]['t']
            print(f"\n=== SVO Pose Rotation (chunk=5s) ===")
            print(f"  {'t_start':>8} {'t_end':>8} {'Δroll(°)':>10} {'Δpitch(°)':>11} {'Δyaw(°)':>10} {'|Δrot|(°)':>11}")
            for i in range(0, len(poses)-1, 50):  # every ~50th pose (~1.65s)
                p1 = poses[i]
                p2 = poses[min(i+50, len(poses)-1)]
                dt = p2['t'] - p1['t']
                dr = p2['roll'] - p1['roll']
                dp = p2['pitch'] - p1['pitch']
                dy = p2['yaw'] - p1['yaw']
                # Normalize angles
                dr = ((dr + math.pi) % (2*math.pi)) - math.pi
                dp = ((dp + math.pi) % (2*math.pi)) - math.pi
                dy = ((dy + math.pi) % (2*math.pi)) - math.pi
                total = math.sqrt(dr**2 + dp**2 + dy**2)
                print(f"  {p1['t']:8.2f} {p2['t']:8.2f} {math.degrees(dr):+10.2f} {math.degrees(dp):+11.2f} {math.degrees(dy):+10.2f} {math.degrees(total):+11.2f}")

            # Overall comparison
            print(f"\n=== Comparison Summary ===")
            # IMU total rotation over SVO duration
            t_start = poses[0]['t']
            t_end = poses[-1]['t']
            imu_result = integrate_imu(imu, t_start, t_end)
            if imu_result is not None:
                _, imu_angles = imu_result
                imu_total = math.sqrt(sum(v**2 for v in imu_angles.values()))
                print(f"  IMU total rotation over SVO period [{t_start:.1f}, {t_end:.1f}]s:")
                print(f"    Δroll={math.degrees(imu_angles['roll']):+.1f}°  Δpitch={math.degrees(imu_angles['pitch']):+.1f}°  Δyaw={math.degrees(imu_angles['yaw']):+.1f}°")
                print(f"    |Δrot|={math.degrees(imu_total):+.1f}°")

            # SVO total rotation
            p0 = poses[0]; pN = poses[-1]
            dr = pN['roll'] - p0['roll']
            dp = pN['pitch'] - p0['pitch']
            dy = pN['yaw'] - p0['yaw']
            dr = ((dr + math.pi) % (2*math.pi)) - math.pi
            dp = ((dp + math.pi) % (2*math.pi)) - math.pi
            dy = ((dy + math.pi) % (2*math.pi)) - math.pi
            svo_total = math.sqrt(dr**2 + dp**2 + dy**2)
            print(f"  SVO total rotation:")
            print(f"    Δroll={math.degrees(dr):+.1f}°  Δpitch={math.degrees(dp):+.1f}°  Δyaw={math.degrees(dy):+.1f}°")
            print(f"    |Δrot|={math.degrees(svo_total):+.1f}°")
    else:
        print(f"\nNOTE: Provide --svo-csv to compare with SVO poses.")
        print(f"  Capture SVO poses with:")
        print(f"    ros2 topic echo /svo/pose --csv > svo_poses.csv")


if __name__ == '__main__':
    main()
