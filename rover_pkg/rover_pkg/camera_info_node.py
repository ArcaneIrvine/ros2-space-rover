import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoNode(Node):
    def __init__(self):
        super().__init__('camera_info_node')

        self.publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.timer = self.create_timer(1.0, self.publish_info)

        # Approximate intrinsics for RPi Camera v2 (IMX219) at 640x480
        # These are reasonable defaults — can be refined with proper calibration later
        self.width  = 640
        self.height = 480
        self.fx = 530.0
        self.fy = 530.0
        self.cx = self.width  / 2.0
        self.cy = self.height / 2.0

        self.get_logger().info('Camera info node started')

    def publish_info(self):
        msg = CameraInfo()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical'
        msg.width  = self.width
        msg.height = self.height
        msg.k = [self.fx, 0.0, self.cx,
                  0.0, self.fy, self.cy,
                  0.0, 0.0, 1.0]
        msg.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = 'plumb_bob'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
