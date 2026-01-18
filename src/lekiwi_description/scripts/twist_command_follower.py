#!/usr/bin/env python3
import rclpy
from rclpy. node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

from dataclasses import dataclass, field
from typing import Any

class TwistCommandFollower(Node):
    def __init__(self):
        super().__init__('twist_command_follower')

        # verbose
        self.declare_parameter("verbose", False)
        self.verbose = self.get_parameter('verbose').value          
        self.get_logger().info('Verbose : "%s"' % self.verbose)
        
        self.joint_names = ['left_wheel_joint', 'rear_wheel_joint', 'right_wheel_joint']
        
        # Define three speed levels and a current index
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},  # slow
            {"xy": 0.2, "theta": 60},  # medium
            {"xy": 0.3, "theta": 90},  # fast
        ]
        self.speed_index = 0  # Start at slow 
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_command_callback,
            10)
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/omni_wheel_drive_controller/commands',
            10)
        
        self.get_logger().info('Twist command follower (VELOCITY mode with calculation) started')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def twist_command_callback(self, msg):
        self.twist_msg = msg
        
        if self.verbose:
            self.get_logger().info('Base Twist : "%s"' % self.twist_msg)
            # Base Twist : "geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))"

        # convert x,y,z translation/rotations commands to x/y/theta commands for Lekiwi
        # commands will be scaled according to speed_levels
        self.base_action = self._twist_to_base_action()
        if self.verbose:
            self.get_logger().info('Base Action : "%s"' % self.base_action)
            # Base Action : "{'x.vel': 0.1, 'y.vel': 0.0, 'theta.vel': 0.0}"
        
        # convert x/y/theta commands to raw commands for LeKiwi's 3 wheels
        self.base_commands = self._body_to_wheel_raw(
            self.base_action["x.vel"],
            self.base_action["y.vel"],
            self.base_action["theta.vel"]
        )
        if self.verbose:
            self.get_logger().info('Base Commands : "%s"' % self.base_commands)
            # Base Commands : "{'left_wheel_joint': 0, 'rear_wheel_joint': 0, 'right_wheel_joint': 0}"

        # Get raw wheel velocities        
        wheel_velocities = []
        wheel_velocities.append( self.base_commands["left_wheel_joint"] )
        wheel_velocities.append( self.base_commands["rear_wheel_joint"] )
        wheel_velocities.append( self.base_commands["right_wheel_joint"] )
        
        # Send to Gazebo controller
        cmd = Float64MultiArray()
        cmd.data = wheel_velocities
        self.publisher.publish(cmd)


    # ---------------------------------------------------------------------
    # Utility Functions
    #    references: 
    #        https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/lekiwi/lekiwi.py
    #        https://github.com/astroyat/lerobot/blob/ros2-latest/src/lerobot/robots/lekiwi/lekiwi.py
    # ---------------------------------------------------------------------     
    def _twist_to_base_action(self):
        if self.twist_msg:
            speed_setting = self.speed_levels[self.speed_index]
            xy_speed = speed_setting["xy"]  # e.g. 0.1, 0.25, or 0.4
            theta_speed = speed_setting["theta"]  # e.g. 30, 60, or 90

            x_cmd = 0.0  # m/s forward/backward
            y_cmd = 0.0  # m/s lateral
            theta_cmd = 0.0  # deg/s rotation

            #if self.twist_msg.linear.x > 0.1:
            #    x_cmd += xy_speed
            #elif self.twist_msg.linear.x < -0.1:
            #    x_cmd -= xy_speed
            x_cmd = self.twist_msg.linear.x
            #if self.twist_msg.angular.z > 0.1:
            #    theta_cmd += theta_speed
            #elif self.twist_msg.angular.z < -0.1:
            #    theta_cmd -= theta_speed
            theta_cmd = self.twist_msg.angular.z
                        
            self.twist_msg = None
            return {
                "x.vel": x_cmd,
                "y.vel": y_cmd,
                "theta.vel": theta_cmd,
            }
        else:
            return None

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        steps_per_deg = 4096.0 / 360.0
        speed_in_steps = degps * steps_per_deg
        speed_int = int(round(speed_in_steps))
        # Cap the value to fit within signed 16-bit range (-32768 to 32767)
        if speed_int > 0x7FFF:
            speed_int = 0x7FFF  # 32767 -> maximum positive value
        elif speed_int < -0x8000:
            speed_int = -0x8000  # -32768 -> minimum negative value
        return speed_int

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        steps_per_deg = 4096.0 / 360.0
        magnitude = raw_speed
        degps = magnitude / steps_per_deg
        return degps

    def _body_to_wheel_raw(
        self,
        x: float,
        y: float,
        theta: float,
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
        max_raw: int = 3000,
    ) -> dict:
        """
        Convert desired body-frame velocities into wheel raw commands.

        Parameters:
          x_cmd      : Linear velocity in x (m/s).
          y_cmd      : Linear velocity in y (m/s).
          theta_cmd  : Rotational velocity (deg/s).
          wheel_radius: Radius of each wheel (meters).
          base_radius : Distance from the center of rotation to each wheel (meters).
          max_raw    : Maximum allowed raw command (ticks) per wheel.

        Returns:
          A dictionary with wheel raw commands:
             {"base_left_wheel": value, "base_back_wheel": value, "base_right_wheel": value}.

        Notes:
          - Internally, the method converts theta_cmd to rad/s for the kinematics.
          - The raw command is computed from the wheels angular speed in deg/s
            using _degps_to_raw(). If any command exceeds max_raw, all commands
            are scaled down proportionally.
        """
        # Convert rotational velocity from deg/s to rad/s.
        theta_rad = theta * (np.pi / 180.0)
        # Create the body velocity vector [x, y, theta_rad].
        velocity_vector = np.array([x, y, theta_rad])

        # Define the wheel mounting angles with a -90° offset.
        angles = np.radians(np.array([240, 0, 120]) - 90)
        # Build the kinematic matrix: each row maps body velocities to a wheel’s linear speed.
        # The third column (base_radius) accounts for the effect of rotation.
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

        # Compute each wheel’s linear speed (m/s) and then its angular speed (rad/s).
        wheel_linear_speeds = m.dot(velocity_vector)
        wheel_angular_speeds = wheel_linear_speeds / wheel_radius

        # Convert wheel angular speeds from rad/s to deg/s.
        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

        # Scaling
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > max_raw:
            scale = max_raw / max_raw_computed
            wheel_degps = wheel_degps * scale

        # Convert each wheel’s angular speed (deg/s) to a raw integer.
        wheel_raw = [self._degps_to_raw(deg) for deg in wheel_degps]

        #return {
        #    "base_left_wheel": wheel_raw[0],
        #    "base_back_wheel": wheel_raw[1],
        #    "base_right_wheel": wheel_raw[2],
        #}
        return {
            "left_wheel_joint": wheel_raw[0],
            "rear_wheel_joint": wheel_raw[1],
            "right_wheel_joint": wheel_raw[2],
        }

    def _wheel_raw_to_body(
        self,
        left_wheel_speed,
        back_wheel_speed,
        right_wheel_speed,
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
    ) -> dict[str, Any]:
        """
        Convert wheel raw command feedback back into body-frame velocities.

        Parameters:
          wheel_raw   : Vector with raw wheel commands ("base_left_wheel", "base_back_wheel", "base_right_wheel").
          wheel_radius: Radius of each wheel (meters).
          base_radius : Distance from the robot center to each wheel (meters).

        Returns:
          A dict (x.vel, y.vel, theta.vel) all in m/s
        """

        # Convert each raw command back to an angular speed in deg/s.
        wheel_degps = np.array(
            [
                self._raw_to_degps(left_wheel_speed),
                self._raw_to_degps(back_wheel_speed),
                self._raw_to_degps(right_wheel_speed),
            ]
        )

        # Convert from deg/s to rad/s.
        wheel_radps = wheel_degps * (np.pi / 180.0)
        # Compute each wheel’s linear speed (m/s) from its angular speed.
        wheel_linear_speeds = wheel_radps * wheel_radius

        # Define the wheel mounting angles with a -90° offset.
        angles = np.radians(np.array([240, 0, 120]) - 90)
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

        # Solve the inverse kinematics: body_velocity = M⁻¹ · wheel_linear_speeds.
        m_inv = np.linalg.inv(m)
        velocity_vector = m_inv.dot(wheel_linear_speeds)
        x, y, theta_rad = velocity_vector
        theta = theta_rad * (180.0 / np.pi)
        return {
            "x.vel": x,
            "y.vel": y,
            "theta.vel": theta,
        }  # m/s and deg/s
        


def main(args=None):
    rclpy.init(args=args)
    node = TwistCommandFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  
    main()
