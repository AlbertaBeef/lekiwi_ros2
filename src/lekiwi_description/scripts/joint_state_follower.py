#!/usr/bin/env python3
import rclpy
from rclpy. node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class JointStateFollower(Node):
    def __init__(self):
        super().__init__('joint_state_follower')
        
        self.joint_names = ['left_wheel_joint', 'rear_wheel_joint', 'right_wheel_joint']
        
        # Store previous positions and time for velocity calculation
        self.prev_positions = None
        self.prev_time = None
        
        # Smoothing filter (optional)
        self.velocity_filter_alpha = 0.3  # Lower = more smoothing (0-1)
        self.filtered_velocities = [0.0, 0.0, 0.0]
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/omni_wheel_drive_controller/commands',
            10)
        
        self.get_logger().info('Joint state follower (VELOCITY mode with calculation) started')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def joint_state_callback(self, msg):
        positions = []
        
        #self.get_logger().info(f'Joint state callback {msg}')
        
        for joint_name in self. joint_names:
            try:  
                idx = msg.name.index(joint_name)
                if msg.position and len(msg.position) > idx:
                    positions.append(msg.position[idx])
                else:
                    positions.append(0.0)
            except ValueError:
                positions.append(0.0)
        
        #self.get_logger().info(f'positions={positions}')
        
        if len(positions) != 3:
            return
        
        # Calculate velocities from position changes
        current_time = self.get_clock().now()
        
        if self.prev_positions is not None and self.prev_time is not None:
            dt = (current_time - self. prev_time).nanoseconds / 1e9
            
            if dt > 0.001:  # Avoid division by very small dt
                calculated_velocities = []
                
                for i in range(3):
                    # Calculate position difference (handle wrap-around for continuous joints)
                    pos_diff = self.normalize_angle(positions[i] - self.prev_positions[i])
                    vel = pos_diff / dt
                    
                    # Apply smoothing filter
                    self.filtered_velocities[i] = (
                        self.velocity_filter_alpha * vel + 
                        (1.0 - self.velocity_filter_alpha) * self.filtered_velocities[i]
                    )
                    calculated_velocities.append(self. filtered_velocities[i])

                #self.get_logger().info(f'velocities={calculated_velocities}')
                
                # Send to Gazebo controller
                cmd = Float64MultiArray()
                cmd.data = calculated_velocities
                self.publisher.publish(cmd)

                #self.get_logger().info(f'Joint state callback published={cmd}')
        
        # Store for next iteration
        self.prev_positions = positions
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = JointStateFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  
    main()
