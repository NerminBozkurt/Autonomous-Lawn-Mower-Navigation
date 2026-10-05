#!/usr/bin/env python3
"""
Publish the path the robot has actually driven as nav_msgs/Path on /robot_trail.

The pose comes from /ground_truth/pose (published by
ground_truth_tf_publisher in the map frame), so the trail shows where the
robot really went even when it localizes from wheel odometry alone, and it
lines up with /coverage_path in RViz.

The trail is cleared whenever a new FollowPath goal starts executing, so each
run shows only its own track.
"""

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node


class RobotTrailPublisher(Node):

    def __init__(self):
        super().__init__('robot_trail_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pose_topic', '/ground_truth/pose')
        self.declare_parameter('status_topic', '/follow_path/_action/status')
        # Add a pose only after moving this far, so a parked robot does not
        # grow the trail forever.
        self.declare_parameter('min_distance', 0.02)
        self.declare_parameter('rate', 10.0)

        p = self.get_parameter
        self.map_frame = p('map_frame').value
        self.min_distance = p('min_distance').value

        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.goal_id = None
        self.true_pose = None

        self.create_subscription(PoseStamped, p('pose_topic').value,
                                 self.pose_callback, 10)
        self.trail_pub = self.create_publisher(Path, '/robot_trail', 10)
        self.create_subscription(GoalStatusArray, p('status_topic').value,
                                 self.status_callback, 10)
        self.create_timer(1.0 / p('rate').value, self.update)

        self.get_logger().info(
            f'Robot trail: {p("pose_topic").value} on /robot_trail')

    def status_callback(self, msg):
        for status in msg.status_list:
            gid = bytes(status.goal_info.goal_id.uuid)
            if status.status == GoalStatus.STATUS_EXECUTING and gid != self.goal_id:
                self.goal_id = gid
                self.path.poses.clear()
                self.get_logger().info('New FollowPath goal, trail cleared')

    def pose_callback(self, msg):
        self.true_pose = msg

    def update(self):
        if self.true_pose is None:
            return
        tr = self.true_pose.pose.position
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            if ((tr.x - last.x) ** 2 + (tr.y - last.y) ** 2) ** 0.5 < self.min_distance:
                self.publish()
                return
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.true_pose.header.stamp
        pose.pose = self.true_pose.pose
        self.path.poses.append(pose)
        self.publish()

    def publish(self):
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
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
