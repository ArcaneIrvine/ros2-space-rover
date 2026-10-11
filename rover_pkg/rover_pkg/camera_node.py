import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
from libcamera import Transform

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Camera setup — identical config to your test file
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480)},
            transform=Transform(hflip=1, vflip=1)
        )
        self.picam2.configure(config)
        self.picam2.start()

        self.bridge     = CvBridge()
        self.publisher  = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer      = self.create_timer(0.2, self.capture_and_publish)  # ~5fps

        self.get_logger().info('Camera node started')

    def capture_and_publish(self):
        frame = self.picam2.capture_array()
        # Convert XBGR8888 (4 channels) to BGR (3 channels)
        import cv2
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        msg   = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical'
        self.publisher.publish(msg)

    def destroy_node(self):
        self.picam2.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
