#!/usr/bin/env python3
"""Wait for two distinct, current camera frames before starting real exploration."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = Node('wait_for_camera')
    previous_stamp = None
    ready = False

    def receive(image):
        nonlocal previous_stamp, ready
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds - stamp) / 1e9
        if not (-0.1 <= age <= 1.0 and image.width and image.height
                and image.step and len(image.data) >= image.height * image.step):
            return
        ready = previous_stamp is not None and stamp > previous_stamp
        previous_stamp = stamp

    node.create_subscription(Image, '/camera/image_raw', receive, qos_profile_sensor_data)
    next_notice = 0.0
    try:
        while rclpy.ok() and not ready:
            if time.monotonic() >= next_notice:
                node.get_logger().info('Waiting for fresh /camera/image_raw frames. On the robot run start_camera in the same ROS domain; check camera timestamps if frames are present.')
                next_notice = time.monotonic() + 10.0
            rclpy.spin_once(node, timeout_sec=0.5)
        return 0 if ready else 1
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
