#!/usr/bin/env python3
"""Diagnostic: list all active topics to find exact IMU topic name."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class TopicLister(Node):
    def __init__(self):
        super().__init__('topic_lister')
        self.declare_parameter('wait_seconds', 5.0)
        wait = self.get_parameter('wait_seconds').value
        self.get_logger().info(f'Waiting {wait}s for topics...')
        import time
        time.sleep(wait)
        self.get_logger().info('All active topics:')
        topic_names_and_types = self.get_topic_names_and_types()
        for name, types in sorted(topic_names_and_types.items()):
            print(f'  {name}: {types}')
        self.get_logger().info('Done.')

if __name__ == '__main__':
    rclpy.init()
    node = TopicLister()
    rclpy.shutdown()
