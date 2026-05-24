#!/usr/bin/env python3
"""
Subscribe to Gazebo's /gazebo/model_states and publish the robot's
ground truth pose as a TF transform from 'map' to 'base_link'.

This bypasses AMCL and provides perfect localization for controller benchmarking.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthTFPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_publisher')

        # Robot model name in Gazebo (must match -entity in spawn)
        self.robot_name = 'turtlebot3_burger'

        # Subscribe to Gazebo's model states
        self.sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10,
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Ground truth TF publisher started for "{self.robot_name}"'
        )

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index(self.robot_name)
        except ValueError:
            return  # robot not yet spawned

        pose = msg.pose[idx]

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation = pose.orientation

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