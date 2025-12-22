#!/usr/bin/env python3
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
from sensor_msgs.msg import JointState
import threading

import base64
import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any

import sys
import math
import yaml, pathlib
from ament_index_python.packages import get_package_share_directory
import numpy as np

# -----------------------------------------------------------------------------

from lekiwi_hw_interface.motors.feetech.feetech import FeetechMotorsBus
from lekiwi_hw_interface.motors import Motor, MotorNormMode

PORT_DEFAULT = "/dev/ttyACM0"

JOINTS = {
    #"shoulder_pan": {"id": 1, "model": "sts3215"},
    #"shoulder_lift": {"id": 2, "model": "sts3215"},
    #"elbow_flex": {"id": 3, "model": "sts3215"},
    #"wrist_flex": {"id": 4, "model": "sts3215"},
    #"wrist_roll": {"id": 5, "model": "sts3215"},
    #"gripper": {"id": 6, "model": "sts3215"},
    "left_wheel_joint": {"id": 7, "model": "sts3215"},
    "rear_wheel_joint": {"id": 8, "model": "sts3215"},
    "right_wheel_joint": {"id": 9, "model": "sts3215"},
}


# Default calibration file distributed with the package (can be overridden by
# the ROS parameter "calib_file").
CALIB_FILE = (
    pathlib.Path(get_package_share_directory("lekiwi_hw_interface"))
    / "config/lekiwi_calibration.yaml"
)

class MotorBridge(Node):
    def __init__(self):
        super().__init__('lekiwi_motor_bridge')
        
        # Declare parameters so they can be overridden from launch/CLI
        self.declare_parameter("port", PORT_DEFAULT)
        port = self.get_parameter("port").get_parameter_value().string_value
        if not port:
            port = PORT_DEFAULT
        #
        self.declare_parameter("calib_file", str(CALIB_FILE))
        calib_path = pathlib.Path(self.get_parameter("calib_file").get_parameter_value().string_value).expanduser()
        
        # verbose
        self.declare_parameter("verbose", True)
        self.verbose = self.get_parameter('verbose').value          
        self.get_logger().info('Verbose : "%s"' % self.verbose)

        # topic_name
        self.declare_parameter("topic_name", "/cmd_vel")
        self.topic_name = self.get_parameter('topic_name').value
        self.get_logger().info('Topic name : "%s"' % self.topic_name)

        # Build motor objects
        motors = {
            name: Motor(cfg["id"], cfg["model"], MotorNormMode.DEGREES)
            for name, cfg in JOINTS.items()
        }
        self.bus = FeetechMotorsBus(port, motors)

        self.get_logger().info(f"Connecting to Feetech bus on {port} …")
        try:
            self.bus.connect()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Could not open motor bus: {exc}")
            raise
        self.bus.configure_motors()
        self.bus.enable_torque()
        self.get_logger().info("Motor bus connected and configured.")

        # Publishers / Subscribers
        self.joint_states_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_commands_sub = self.create_subscription(
            JointState,
            "/joint_commands",
            self._arm_command_cb,
            10,
        )
        self.base_commands_sub = self.create_subscription(
            Twist, 
            "cmd_vel", 
            self._base_command_cb, 
            10
        )

        # Command cache (rad)
        self.current_commands: dict[str, float] = {name: 0.0 for name in JOINTS}

        # Feetech STS3215 resolution: 4096 steps per 2π rad
        self._steps_per_rad = 4096.0 / (2 * math.pi)

        # Load calibration (if file exists) else fall back to first-read capture
        calib_path = pathlib.Path(self.get_parameter("calib_file").get_parameter_value().string_value).expanduser()
        self._home_offsets: dict[str, int] | None = None
        self._limits: dict[str, tuple[int, int]] | None = None
        if calib_path.is_file():
            with open(calib_path, "r", encoding="utf-8") as fp:
                calib = yaml.safe_load(fp)
            self._home_offsets = {j: calib[j]["homing_offset"] for j in JOINTS if j in calib}
            self._limits = {j: (calib[j]["range_min"], calib[j]["range_max"]) for j in JOINTS if j in calib}
            self.get_logger().info(f"Loaded calibration from {calib_path}")
        else:
            self.get_logger().warn(f"Calibration file {calib_path} not found – will capture offsets on first read.")

        # Timer for periodic read/write (50 Hz)
        self.timer = self.create_timer(0.02, self._timer_cb)

        # Flag to ensure we only command the initial middle position once
        self._initial_move_done = False

        # Define three speed levels and a current index
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},  # slow
            {"xy": 0.2, "theta": 60},  # medium
            {"xy": 0.3, "theta": 90},  # fast
        ]
        self.speed_index = 0  # Start at slow        
        
        self.get_logger().info("Done !")

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def _arm_command_cb(self, msg: JointState):
        """Store desired joint positions from topic (rad)."""
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_commands:
                self.current_commands[name] = pos

    def _base_command_cb(self, twist_msg: Twist):
        self.twist_msg = twist_msg
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
            # Base Commands : "{'base_left_wheel': -1129, 'base_back_wheel': 0, 'base_right_wheel': 1129}"

        #self.current_commands["left_wheel_joint"] = self.base_commands["base_left_wheel"]
        #self.current_commands["back_wheel_joint"] = self.base_commands["base_back_wheel"]
        #self.current_commands["right_wheel_joint"] = self.base_commands["base_right_wheel"]
        self.bus.sync_write("Goal_Velocity", self.base_commands)
                
    def _timer_cb(self):
        # --- Read present positions
        try:
            raw_positions = self.bus.sync_read("Present_Position", normalize=False)

            # Establish home offsets once (first successful read)
            if self._home_offsets is None:
                self._home_offsets = raw_positions
                self.get_logger().info("Captured home offsets: %s" % self._home_offsets)

            # convert to radians relative to home
            positions = {
                n: (raw - self._home_offsets.get(n, 0)) * (2 * math.pi) / 4096.0 if self._home_offsets else 0.0
                for n, raw in raw_positions.items()
            }
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"sync_read failed: {exc}")
            return

        # Publish joint states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINTS.keys())
        js.position = [positions[n] for n in js.name]
        js.velocity = []
        js.effort = []
        self.joint_states_pub.publish(js)

        # -----------------------------------------------------------------
        # After the first successful read AND once we have both limits and
        # home offsets, send a single command to drive each joint to the
        # middle of its calibrated range. This provides a well-defined pose
        # on startup without relying on external controllers.
        # -----------------------------------------------------------------
        if (
            not self._initial_move_done
            and self._home_offsets is not None
            and self._limits is not None
        ):
            for name in JOINTS:
                if name in self._limits and name in self._home_offsets:
                    low, high = self._limits[name]
                    mid_raw = int((low + high) / 2)
                    # Convert to radians relative to home
                    self.current_commands[name] = (mid_raw - self._home_offsets[name]) * (2 * math.pi) / 4096.0
            self._initial_move_done = True
            self.get_logger().info("Issued initial command to move joints to mid-range position.")

        # --- Write goal positions
        try:
            # convert desired rad to raw steps relative to home
            raw_goals = {}
            for n, rad in self.current_commands.items():
                home = self._home_offsets.get(n, 0) if self._home_offsets else 0
                raw = int(home + rad * self._steps_per_rad)
                # clamp to limits if available
                if self._limits and n in self._limits:
                    low, high = self._limits[n]
                    raw = max(low, min(high, raw))
                raw_goals[n] = raw
            self.bus.sync_write("Goal_Position", raw_goals, normalize=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"sync_write failed: {exc}")
    
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
        node.bus.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
