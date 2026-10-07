#!/usr/bin/env python3
"""
Send a Fields2Cover-style coverage path straight to Nav2's controller.

The path goes to controller_server's FollowPath action, bypassing the global
planner and the BT navigator, so every controller (RPP, MPPI, DWB) tracks the
identical reference and tracking error is comparable across runs. The cost:
nothing replans around obstacles, which is fine for the empty test field.

With row_controller and turn_controller set, the swaths are followed with
one controller and the U-turns with the other, switching on the fly (see
path_executor); turn_lead and turn_lag move the switch points that far before
and after each turn. Otherwise controller_id drives the whole path.

The reference is also republished on /coverage_path for RViz.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from mower_sim.coverage_path import boustrophedon_path
from mower_sim.path_executor import PathExecutor
from mower_sim.path_segments import controller_schedule, segments
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
        # Switching: both empty -> controller_id for the whole path.
        self.declare_parameter('row_controller', '')
        self.declare_parameter('turn_controller', '')
        self.declare_parameter('turn_lead', 0.0)
        self.declare_parameter('turn_lag', 0.0)
        self.declare_parameter('goal_checker_id', 'general_goal_checker')
        # controller_server's action server is up a few seconds before Nav2
        # activates it, and a goal sent in that gap is rejected; retry.
        self.declare_parameter('goal_retries', 30)
        self.declare_parameter('goal_retry_period', 2.0)
        self._attempts = 0
        self._retry_timer = None

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

        row, turn = p('row_controller').value, p('turn_controller').value
        if row and turn:
            self.schedule = controller_schedule(
                segments([(x, y) for x, y, _ in poses]), row, turn,
                p('turn_lead').value, p('turn_lag').value)
        else:
            self.schedule = [(0.0, p('controller_id').value)]
        self.get_logger().info('Controller schedule: ' + ', '.join(
            f'{c} from {s:.2f} m' for s, c in self.schedule))

        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)
        self.create_timer(1.0, lambda: self.path_pub.publish(self.path))

        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self._executor = None

    def send_path(self):
        self.get_logger().info('Waiting for FollowPath action server...')
        self._action_client.wait_for_server()
        self._send_goal()

    def _send_goal(self):
        if self._retry_timer is not None:
            self.destroy_timer(self._retry_timer)
            self._retry_timer = None
        self._attempts += 1
        self.get_logger().info(
            f'Sending coverage path: {len(self.path.poses)} poses')
        self._executor = PathExecutor(
            self, self._action_client, self.path, self.schedule,
            goal_checker_id=self.get_parameter('goal_checker_id').value,
            on_feedback=self.feedback_callback, on_done=self.result_callback,
            on_rejected=self.goal_rejected)
        self._executor.start()

    def goal_rejected(self):
        if self._attempts <= self.get_parameter('goal_retries').value:
            period = self.get_parameter('goal_retry_period').value
            self.get_logger().warn(
                f'FollowPath goal rejected, Nav2 probably not active yet; '
                f'retrying in {period:.0f} s')
            self._retry_timer = self.create_timer(period, self._send_goal)
            return
        self.get_logger().error('FollowPath goal rejected')
        rclpy.shutdown()

    def feedback_callback(self, distance_left, speed, controller):
        self.get_logger().info(
            f'Distance to goal: {distance_left:.2f} m, '
            f'speed: {speed:.2f} m/s, controller: {controller}',
            throttle_duration_sec=1.0)

    def result_callback(self, status):
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
