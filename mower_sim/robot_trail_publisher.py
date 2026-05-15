#!/usr/bin/env python3
"""
Subscribe to /odom and publish accumulated trajectory as nav_msgs/Path
to /robot_trail. Visualize in RViz to see where the robot has actually gone.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class RobotTrailPublisher(Node):
    def __init__(self):
        super().__init__('robot_trail_publisher')

        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Subscribe to odom
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Publish accumulated trail
        self.trail_pub = self.create_publisher(Path, '/robot_trail', 10)

        # Republish periodically so RViz always has fresh data
        self.timer = self.create_timer(0.5, self.publish_trail)

        # Distance threshold to add a new pose (avoid millions of stationary points)
        self.last_x = None
        self.last_y = None
        self.min_distance = 0.05  # 5 cm

        self.get_logger().info('Robot trail publisher started')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Only add pose if robot has moved enough
        if self.last_x is None or \
           ((x - self.last_x) ** 2 + (y - self.last_y) ** 2) ** 0.5 > self.min_distance:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.path.poses.append(pose)
            self.last_x = x
            self.last_y = y

    def publish_trail(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.trail_pub.publish(self.path)


def main():
    rclpy.init()
    node = RobotTrailPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
