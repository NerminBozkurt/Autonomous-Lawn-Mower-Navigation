#!/usr/bin/env python3
"""
Send a mowing-style path to Nav2 via the NavigateThroughPoses action.
Waypoints define a small 3-row zigzag pattern in a clear area of turtlebot3_world.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from builtin_interfaces.msg import Time


def make_pose(x, y, yaw=0.0):
    """Create a PoseStamped at (x, y) with given yaw in radians."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    # Quaternion from yaw (Z-axis rotation only)
    import math
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class MowingPathClient(Node):
    def __init__(self):
        super().__init__('mowing_path_client')
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )

    def send_path(self, poses):
        self.get_logger().info('Waiting for NavigateThroughPoses action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateThroughPoses.Goal()
        # Stamp all poses with current time
        now = self.get_clock().now().to_msg()
        for pose in poses:
            pose.header.stamp = now
        goal_msg.poses = poses

        self.get_logger().info(f'Sending mowing path with {len(poses)} waypoints')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, robot is moving')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Waypoints remaining: {feedback.number_of_poses_remaining}, '
            f'Distance: {feedback.distance_remaining:.2f} m'
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Mowing path completed')
        rclpy.shutdown()


def main():
    rclpy.init()

    # ----- Define mowing-style waypoints -----
    # Zigzag pattern: 3 rows in clear area to the right of obstacles
    # Start at (3.0, 0.0), 0.5m row spacing
    waypoints = [
        make_pose(5.0,  0.0, yaw=0.0),        # row 1: go right
        make_pose(5.0,  0.5, yaw=1.5708),     # turn up (pi/2)
        make_pose(3.0,  0.5, yaw=3.1416),     # row 2: go left (pi)
        make_pose(3.0,  1.0, yaw=1.5708),     # turn up
        make_pose(5.0,  1.0, yaw=0.0),        # row 3: go right
    ]

    node = MowingPathClient()
    node.send_path(waypoints)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
