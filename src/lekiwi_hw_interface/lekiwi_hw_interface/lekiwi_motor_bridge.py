"""ROS 2 node that bridges Feetech servos <-> cmd_vel.

It:
* subscribes to `/cmd_vel` (geometry_msgs/Twist)
  converts Twist commands to base actions for LeKiwi mobile base
  send base actions to motors.
* publishes ... ?

Make sure the USB-to-UART adapter of the servo bus is accessible
(e.g. `/dev/ttyACM0`) and you have permission to open it.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

import base64
import json
import logging
import time
from dataclasses import dataclass, field

from lerobot.robots.lekiwi.lekiwi import LeKiwi
from lerobot.robots.lekiwi.config_lekiwi import LeKiwiConfig


class MotorBridge(Node):
    def __init__(self):
        super().__init__('lekiwi_motor_bridge')
        
        # Parameters:
        
        # verbose
        self.declare_parameter("verbose", True)
        self.verbose = self.get_parameter('verbose').value          
        self.get_logger().info('Verbose : "%s"' % self.verbose)

        # topic_name
        self.declare_parameter("topic_name", "/cmd_vel")
        self.topic_name = self.get_parameter('topic_name').value
        self.get_logger().info('Topic name : "%s"' % self.topic_name)
        
        self.subscriber = self.create_subscription(
            Twist, 
            "cmd_vel", 
            self.cmd_vel_callback, 
            10
        )
        
        # Define three speed levels and a current index
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},  # slow
            {"xy": 0.2, "theta": 60},  # medium
            {"xy": 0.3, "theta": 90},  # fast
        ]
        self.speed_index = 0  # Start at slow        
        
        self.get_logger().info("Configuring LeKiwi")
        self.robot_config = LeKiwiConfig(
            id="lekiwi",
            port="/dev/ttyACM0",
            disable_torque_on_disconnect=True,
            cameras={},
            use_degrees=True
        )
        self.get_logger().info('LeKiwi config : "%s"' % self.robot_config)
        self.robot = LeKiwi(self.robot_config)
        
        self.get_logger().info("Connecting LeKiwi")
        self.robot.connect()

        self.robot_position = self.robot.get_observation()
        self.get_logger().info('LeKiwi position : "%s"' % self.robot_position)
        
        self.arm_action = dict(list(self.robot_position.items())[:6])
        self.get_logger().info('LeKiwi arm : "%s"' % self.arm_action)

        self.get_logger().info("Done !")
        
    def cmd_vel_callback(self, twist_msg):
        self.twist_msg = twist_msg
        if self.verbose:
            self.get_logger().info('Twist : "%s"' % self.twist_msg)

        # convert x,y,z translation/rotations commands to x/y/theta commands for Lekiwi
        # commands will be scaled according to speed_levels
        self.base_action = self.twist_to_base_action()
        if self.verbose:
            self.get_logger().info('Action : "%s"' % self.base_action)
        
        # robot expects actions for both arm and base, so use initial position of arm   
        self.robot_action = {**self.arm_action, **self.base_action} if len(self.base_action) > 0 else self.arm_action            

        # send actions to LeKiwi robot (base+arm)
        # base x/y/theta commands will be converted to raw commands for LeKiwi's 3 wheels
        self.robot.send_action(self.robot_action)

    def twist_to_base_action(self):
        if self.twist_msg:
            speed_setting = self.speed_levels[self.speed_index]
            xy_speed = speed_setting["xy"]  # e.g. 0.1, 0.25, or 0.4
            theta_speed = speed_setting["theta"]  # e.g. 30, 60, or 90

            x_cmd = 0.0  # m/s forward/backward
            y_cmd = 0.0  # m/s lateral
            theta_cmd = 0.0  # deg/s rotation

            if self.twist_msg.linear.x > 0.1:
                x_cmd += xy_speed
            elif self.twist_msg.linear.x < -0.1:
                x_cmd -= xy_speed
            if self.twist_msg.angular.z > 0.1:
                theta_cmd += theta_speed
            elif self.twist_msg.angular.z < -0.1:
                theta_cmd -= theta_speed
            self.twist_msg = None
            return {
                "x.vel": x_cmd,
                "y.vel": y_cmd,
                "theta.vel": theta_cmd,
            }
        else:
            return None





# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------

def main():
    """Entry-point."""
    rclpy.init()
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
