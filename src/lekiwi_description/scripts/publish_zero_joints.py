#!/usr/bin/env python3
"""
Simple joint state publisher for testing URDF.
Publishes static joint positions (all zeros) so robot_state_publisher can compute TF.

Usage:
    ros2 run lekiwi_description publish_zero_joints.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ZeroJointPublisher(Node):
    def __init__(self):
        super().__init__('zero_joint_publisher')

        # Publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Joint names from your URDF
        self.joint_names = [
            'left_wheel_joint',
            'rear_wheel_joint',
            'right_wheel_joint',
        ]

        self.get_logger().info('Publishing zero joint states for URDF preview...')
        self.get_logger().info(f'Joints: {self.joint_names}')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)  # All joints at zero position
        msg.velocity = []
        msg.effort = []

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZeroJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
