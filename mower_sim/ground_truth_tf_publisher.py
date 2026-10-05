#!/usr/bin/env python3
"""
Publish the robot's ground truth pose and the 'map' -> 'odom' transform.

The ground truth pose from /gazebo/model_states always goes out on
/ground_truth/pose (map frame). Metrics and the RViz trail read it there, so
they measure the true pose whatever the robot believes.

'map' -> 'odom' depends on correct_odom:
- true: perfect localization. Gazebo's diff drive plugin owns 'odom' ->
  'base_link', so this node must NOT publish 'map' -> 'base_link' directly (a
  frame can only have one parent). Instead it looks up the odometry's 'odom'
  -> 'base_link' and broadcasts the difference to the ground truth as 'map' ->
  'odom', the way AMCL would, but without error.
- false: dead reckoning. 'map' -> 'odom' stays the identity (the robot
  starts at the map origin), so the robot's pose in the map is whatever its
  odometry says, drift included. Combine with wheel encoder odometry.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_ros


def quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_inv(q):
    """Inverse of a unit (x, y, z, w) quaternion."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rotate(q, v):
    """Rotate vector v by quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    rx, ry, rz, _ = quat_mul(quat_mul(q, qv), quat_inv(q))
    return (rx, ry, rz)


class GroundTruthTFPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_publisher')

        self.declare_parameter('robot_name', 'mower')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Stamp transforms slightly into the future so costmap lookups at the
        # current time never fail by a few milliseconds (same trick as AMCL).
        self.declare_parameter('transform_tolerance', 0.1)
        self.declare_parameter('correct_odom', True)

        self.robot_name = self.get_parameter('robot_name').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.transform_tolerance = self.get_parameter('transform_tolerance').value
        self.correct_odom = self.get_parameter('correct_odom').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)

        self.sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10,
        )

        mode = 'ground truth correction' if self.correct_odom else 'identity'
        self.get_logger().info(
            f'Ground truth TF publisher started for "{self.robot_name}": '
            f'{self.map_frame} -> {self.odom_frame} ({mode})'
        )

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index(self.robot_name)
        except ValueError:
            return  # robot not yet spawned

        # Ground truth: map -> base_link
        gt = msg.pose[idx]
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose = gt
        self.pose_pub.publish(pose)

        if not self.correct_odom:
            self.broadcast((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            return

        t_mb = (gt.position.x, gt.position.y, gt.position.z)
        q_mb = (gt.orientation.x, gt.orientation.y,
                gt.orientation.z, gt.orientation.w)

        # Odometry: odom -> base_link (owned by the Gazebo diff drive plugin)
        try:
            odom_tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f'Waiting for {self.odom_frame} -> {self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        ot = odom_tf.transform.translation
        orot = odom_tf.transform.rotation
        t_ob = (ot.x, ot.y, ot.z)
        q_ob = (orot.x, orot.y, orot.z, orot.w)

        # Invert odom -> base_link to get base_link -> odom
        q_bo = quat_inv(q_ob)
        t_bo = quat_rotate(q_bo, (-t_ob[0], -t_ob[1], -t_ob[2]))

        # Compose: map -> odom = (map -> base_link) * (base_link -> odom)
        q_mo = quat_mul(q_mb, q_bo)
        r_bo = quat_rotate(q_mb, t_bo)
        t_mo = (t_mb[0] + r_bo[0], t_mb[1] + r_bo[1], t_mb[2] + r_bo[2])
        self.broadcast(t_mo, q_mo)

    def broadcast(self, t_mo, q_mo):
        stamp = self.get_clock().now() + Duration(seconds=self.transform_tolerance)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = t_mo[0]
        t.transform.translation.y = t_mo[1]
        t.transform.translation.z = t_mo[2]
        t.transform.rotation.x = q_mo[0]
        t.transform.rotation.y = q_mo[1]
        t.transform.rotation.z = q_mo[2]
        t.transform.rotation.w = q_mo[3]

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GroundTruthTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
