#!/usr/bin/env python3
"""
Send a Fields2Cover-style coverage path straight to Nav2's controller.

The path goes to controller_server's FollowPath action, bypassing the global
planner and the BT navigator, so every controller (RPP, MPPI, DWB) tracks the
identical reference and tracking error is comparable across runs. The cost:
nothing replans around obstacles, which is fine for the empty test field.

The reference is also republished on /coverage_path for RViz.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from mower_sim.coverage_path import boustrophedon_path
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def to_path_msg(poses, frame_id):
    path = Path()
    path.header.frame_id = frame_id
    # Zero stamp = "latest available transform" for tf2 lookups.
    for x, y, yaw in poses:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(pose)
    return path


class MowingPathClient(Node):

    def __init__(self):
        super().__init__('mowing_path_client')

        # swath_spacing matches cutting_width in urdf/mower.urdf.xacro, so
        # adjacent passes touch and no uncut strip is left between them.
        self.declare_parameter('num_swaths', 3)
        self.declare_parameter('swath_length', 5.0)
        self.declare_parameter('swath_spacing', 0.75)
        self.declare_parameter('turn_radius', 0.0)  # 0 -> swath_spacing / 2
        self.declare_parameter('step', 0.05)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')

        p = self.get_parameter
        turn_radius = p('turn_radius').value
        poses = boustrophedon_path(
            num_swaths=p('num_swaths').value,
            swath_length=p('swath_length').value,
            swath_spacing=p('swath_spacing').value,
            turn_radius=turn_radius if turn_radius > 0.0 else None,
            step=p('step').value,
        )
        self.path = to_path_msg(poses, p('frame_id').value)

        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)
        self.create_timer(1.0, lambda: self.path_pub.publish(self.path))

        self._action_client = ActionClient(self, FollowPath, 'follow_path')

    def send_path(self):
        self.get_logger().info('Waiting for FollowPath action server...')
        self._action_client.wait_for_server()

        goal = FollowPath.Goal()
        goal.path = self.path
        goal.controller_id = self.get_parameter('controller_id').value
        goal.goal_checker_id = self.get_parameter('goal_checker_id').value

        self.get_logger().info(
            f'Sending coverage path: {len(self.path.poses)} poses')
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, robot is moving')
        goal_handle.get_result_async().add_done_callback(
            self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Distance to goal: {fb.distance_to_goal:.2f} m, '
            f'speed: {fb.speed:.2f} m/s',
            throttle_duration_sec=1.0)

    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Coverage path completed')
        else:
            self.get_logger().error(f'FollowPath ended with status {status}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MowingPathClient()
    node.send_path()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
