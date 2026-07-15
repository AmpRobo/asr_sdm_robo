#!/usr/bin/env python3
"""
Record pose_graph_path from ROS2 for sparse ON / OFF comparison.
Subscribes to /localization/vins/pose_graph_path and writes
output/sparse_{on,off}/vins_sparse_{on,off}.csv

Format: timestamp_s, x, y, z, qw, qx, qy, qz
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import os, sys, time

class TrajRecorder(Node):
    def __init__(self, out_path):
        super().__init__('traj_recorder')
        self.out_path = out_path
        self.fd = open(out_path, 'w')
        self.cb = self.create_subscription(
            Path, '/localization/vins/pose_graph_path',
            self._cb, 1000)
        self.get_logger().info(f'Recording to {out_path}')

    def _cb(self, msg):
        for p in msg.poses:
            ts = float(p.header.stamp.sec) + float(p.header.stamp.nanosec) * 1e-9
            x, y, z = p.pose.position.x, p.pose.position.y, p.pose.position.z
            qx, qy, qz, qw = (p.pose.orientation.x, p.pose.orientation.y,
                               p.pose.orientation.z, p.pose.orientation.w)
            self.fd.write(f'{ts:.6f},{x:.6f},{y:.6f},{z:.6f},{qw:.6f},{qx:.6f},{qy:.6f},{qz:.6f}\n')

    def close(self):
        self.fd.close()
        self.get_logger().info(f'Saved {self.out_path}')


def main():
    if len(sys.argv) != 2:
        print('Usage: record_traj.py <output_csv>')
        sys.exit(1)
    out_path = sys.argv[1]
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    rclpy.init()
    node = TrajRecorder(out_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
