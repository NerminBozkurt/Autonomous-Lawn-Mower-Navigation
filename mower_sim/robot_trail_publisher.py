#!/usr/bin/env python3
"""
Publish the path the robot has actually driven as nav_msgs/Path on /robot_trail.

The pose comes from TF (map -> base_footprint by default), which in the
simulation is ground truth thanks to ground_truth_tf_publisher, and the trail
is published in the map frame so it lines up with /coverage_path in RViz.

The trail is cleared whenever a new FollowPath goal starts executing, so each
run shows only its own track.
"""

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class RobotTrailPublisher(Node):

    def __init__(self):
        super().__init__('robot_trail_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('status_topic', '/follow_path/_action/status')
        # Add a pose only after moving this far, so a parked robot does not
        # grow the trail forever.
        self.declare_parameter('min_distance', 0.02)
        self.declare_parameter('rate', 10.0)

        p = self.get_parameter
        self.map_frame = p('map_frame').value
        self.base_frame = p('base_frame').value
        self.min_distance = p('min_distance').value

        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.goal_id = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.trail_pub = self.create_publisher(Path, '/robot_trail', 10)
        self.create_subscription(GoalStatusArray, p('status_topic').value,
                                 self.status_callback, 10)
        self.create_timer(1.0 / p('rate').value, self.update)

        self.get_logger().info(
            f'Robot trail: {self.map_frame} -> {self.base_frame} on /robot_trail')

    def status_callback(self, msg):
        for status in msg.status_list:
            gid = bytes(status.goal_info.goal_id.uuid)
            if status.status == GoalStatus.STATUS_EXECUTING and gid != self.goal_id:
                self.goal_id = gid
                self.path.poses.clear()
                self.get_logger().info('New FollowPath goal, trail cleared')

    def update(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        tr = tf.transform.translation
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            if ((tr.x - last.x) ** 2 + (tr.y - last.y) ** 2) ** 0.5 < self.min_distance:
                self.publish()
                return
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = tf.header.stamp
        pose.pose.position.x = tr.x
        pose.pose.position.y = tr.y
        pose.pose.position.z = tr.z
        pose.pose.orientation = tf.transform.rotation
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
