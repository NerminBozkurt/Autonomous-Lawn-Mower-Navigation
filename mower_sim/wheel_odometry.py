#!/usr/bin/env python3
"""
Wheel encoder odometry: integrate the drive wheels' joint angles into /odom.

This is what a real robot's base controller does with its encoders: read
how far each wheel has turned, convert that to distance with the wheel
radius, and integrate the differential drive kinematics. It publishes
nav_msgs/Odometry on /odom and the 'odom' -> 'base_footprint' transform.

Gazebo's diff drive plugin has an encoder mode, but it is not used: in tight
turns the robot rotates about 12 % less than commanded, and the plugin's
encoder odometry still reports the commanded rotation, while the wheel joint
angles match the true motion to within half a degree. Integrating the joint
angles from /joint_states gives the odometry a real encoder would.

wheel_radius and wheel_separation default to the URDF values. Setting them
slightly off simulates a miscalibrated robot.
"""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('wheel_separation', 0.70)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        p = self.get_parameter
        self.radius = p('wheel_radius').value
        self.separation = p('wheel_separation').value
        self.left_joint = p('left_joint').value
        self.right_joint = p('right_joint').value
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value
        self.publish_tf = p('publish_tf').value

        self.x = self.y = self.yaw = 0.0
        self.last = None  # (time [s], left angle, right angle)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_callback, 50)

        self.get_logger().info(
            f'Wheel odometry: r = {self.radius} m, b = {self.separation} m, '
            f'{self.odom_frame} -> {self.base_frame}')

    def joint_callback(self, msg):
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except ValueError:
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.last is None:
            self.last = (t, left, right)
            return
        t0, left0, right0 = self.last
        dt = t - t0
        if dt <= 0.0:
            return
        self.last = (t, left, right)

        d_left = self.radius * (left - left0)
        d_right = self.radius * (right - right0)
        ds = 0.5 * (d_left + d_right)
        dyaw = (d_right - d_left) / self.separation
        # Midpoint integration: move along the average heading of the step.
        heading = self.yaw + 0.5 * dyaw
        self.x += ds * math.cos(heading)
        self.y += ds * math.sin(heading)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw),
                              math.cos(self.yaw + dyaw))
        self.publish(msg.header.stamp, ds / dt, dyaw / dt)

    def publish(self, stamp, v, w):
        qz = math.sin(0.5 * self.yaw)
        qw = math.cos(0.5 * self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
