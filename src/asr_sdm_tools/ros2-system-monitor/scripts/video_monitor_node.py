#!/usr/bin/env python3

import sys
import traceback

import rclpy
from ros2_system_monitor import VideoMonitor


def main(args=None):
    rclpy.init(args=args)
    video_node = None
    try:
        video_node = VideoMonitor()
        rclpy.spin(video_node)
    except KeyboardInterrupt:
        pass
    except Exception:
        from rclpy.logging import get_logger

        get_logger("video_monitor_node").error(traceback.format_exc())
    finally:
        if video_node is not None:
            video_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
