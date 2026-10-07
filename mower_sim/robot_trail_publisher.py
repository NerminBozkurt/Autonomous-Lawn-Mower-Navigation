#!/usr/bin/env python3
"""
Publish where the robot really drove and where it believes it drove.

- /robot_trail (nav_msgs/Path): the true track, from /ground_truth/pose
  (published by ground_truth_tf_publisher in the map frame), so it shows
  where the robot really went even when it localizes from wheel odometry
  alone, and it lines up with /coverage_path in RViz.
- /estimated_trail (nav_msgs/Path): the robot's own estimate, the
  'map' -> 'base_footprint' transform that Nav2 and the robot model in RViz
  use. With odometry:=encoder it drifts away from /robot_trail; with
  odometry:=ground_truth the two coincide.
- /odometry_drift (visualization_msgs/MarkerArray): a line from the true to
  the estimated pose.
- /odometry_error (std_msgs/Float32MultiArray): [position error, largest
  position error this run, heading error], in m, m and rad (estimated minus
  true); the control panel shows them.

The drift compares the estimate with the true pose at the same instant:
/ground_truth/odom (Gazebo's p3d plugin) is stamped with the physics time
it belongs to, as the odometry is, and is interpolated to the estimate's
stamp. /ground_truth/pose cannot be used for this: /gazebo/model_states has
no stamp, so it carries its arrival time, which is off by up to about 20 ms,
i.e. up to 2 cm of false drift at 1 m/s.

The trails are cleared when a new run starts, so each run shows only its own
track. A run starts when a FollowPath goal starts executing while no other
goal was; the goals a switching configuration sends to change controller
mid-run (see path_executor) preempt a running goal and keep the trail.
"""

from collections import deque
import math

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, Float32MultiArray
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp_ns(stamp):
    return Time.from_msg(stamp).nanoseconds


class RobotTrailPublisher(Node):

    def __init__(self):
        super().__init__('robot_trail_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_topic', '/ground_truth/pose')
        self.declare_parameter('drift_truth_topic', '/ground_truth/odom')
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
        self.estimated_path = Path()
        self.estimated_path.header.frame_id = self.map_frame
        self.goal_id = None
        self.was_executing = False
        self.true_pose = None
        # Recent (stamp ns, x, y, yaw) of the stamped true pose, to
        # interpolate it to each estimate's stamp.
        self.true_history = deque(maxlen=200)
        self.max_drift = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, p('pose_topic').value,
                                 self.pose_callback, 10)
        self.create_subscription(Odometry, p('drift_truth_topic').value,
                                 self.truth_callback, 50)
        self.trail_pub = self.create_publisher(Path, '/robot_trail', 10)
        self.estimated_pub = self.create_publisher(Path, '/estimated_trail', 10)
        self.drift_pub = self.create_publisher(MarkerArray, '/odometry_drift',
                                               10)
        self.error_pub = self.create_publisher(Float32MultiArray,
                                               '/odometry_error', 10)
        self.create_subscription(GoalStatusArray, p('status_topic').value,
                                 self.status_callback, 10)
        self.create_timer(1.0 / p('rate').value, self.update)

        self.get_logger().info(
            f'Robot trail: {p("pose_topic").value} on /robot_trail, '
            f'{self.map_frame} -> {self.base_frame} on /estimated_trail')

    def status_callback(self, msg):
        executing = [bytes(st.goal_info.goal_id.uuid) for st in msg.status_list
                     if st.status == GoalStatus.STATUS_EXECUTING]
        for gid in executing:
            if gid != self.goal_id:
                self.goal_id = gid
                if not self.was_executing:
                    self.path.poses.clear()
                    self.estimated_path.poses.clear()
                    self.max_drift = 0.0
                    self.get_logger().info('New run, trail cleared')
        self.was_executing = bool(executing)

    def pose_callback(self, msg):
        self.true_pose = msg

    def truth_callback(self, msg):
        pose = msg.pose.pose
        self.true_history.append((_stamp_ns(msg.header.stamp),
                                  pose.position.x, pose.position.y,
                                  _yaw(pose.orientation)))

    def true_pose_at(self, t):
        """Interpolate the stamped true pose to time t; None if not covered."""
        h = self.true_history
        if not h or t < h[0][0] or t > h[-1][0]:
            return None
        for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(h, list(h)[1:]):
            if t0 <= t <= t1:
                s = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
                return x0 + s * (x1 - x0), y0 + s * (y1 - y0), a0 + s * da
        return h[-1][1:]

    def update(self):
        if self.true_pose is None:
            return
        self.extend(self.path, self.true_pose)
        estimated = self.estimated_pose()
        if estimated is not None:
            self.extend(self.estimated_path, estimated)
            truth = self.true_pose_at(_stamp_ns(estimated.header.stamp))
            if truth is not None:
                self.publish_drift(estimated, truth)
        stamp = self.get_clock().now().to_msg()
        self.path.header.stamp = stamp
        self.trail_pub.publish(self.path)
        self.estimated_path.header.stamp = stamp
        self.estimated_pub.publish(self.estimated_path)

    def extend(self, path, pose):
        p = pose.pose.position
        if path.poses:
            last = path.poses[-1].pose.position
            if math.hypot(p.x - last.x, p.y - last.y) < self.min_distance:
                return
        path.poses.append(pose)

    def estimated_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = tf.header.stamp
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def publish_drift(self, estimated, truth):
        tx, ty, tyaw = truth
        ep = estimated.pose.position
        drift = math.hypot(ep.x - tx, ep.y - ty)
        self.max_drift = max(self.max_drift, drift)
        dyaw = _yaw(estimated.pose.orientation) - tyaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        self.error_pub.publish(
            Float32MultiArray(data=[drift, self.max_drift, dyaw]))

        line = Marker()
        line.header.frame_id = self.map_frame
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'drift_line'
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        line.scale.x = 0.02
        line.points = [Point(x=tx, y=ty, z=0.3),
                       Point(x=ep.x, y=ep.y, z=0.3)]
        self.drift_pub.publish(MarkerArray(markers=[line]))


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
