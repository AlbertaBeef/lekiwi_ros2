import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraFrameFixer(Node):
    def __init__(self):
        super().__init__('camera_frame_fixer')
        # Subscribe to original Gazebo image topic
        self.sub = self.create_subscription(
            Image,
            '/lekiwi_front_camera/image',
            self.cb,
            10
        )
        # Publisher for corrected topic
        self.pub = self.create_publisher(Image, '/lekiwi_front_camera/fixed_image', 10)

    def cb(self, msg):
        # Correct the frame_id to match your TF tree!
        msg.header.frame_id = 'camera_link'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CameraFrameFixer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
