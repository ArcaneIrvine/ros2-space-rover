"""Save one camera frame and its MiDaS depth map side by side.

Run while the rover is running:  python3 depth_snapshot.py
Output: docs/images/rgb-vs-depth.png (plus the two halves separately)
"""
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT_DIR = os.path.expanduser('~/rover_ws/src/docs/images')
MIN_DEPTH = 0.2   # metres, same as depth_node
MAX_DEPTH = 1.0


class DepthSnapshot(Node):
    def __init__(self):
        super().__init__('depth_snapshot')
        self.bridge = CvBridge()
        self.rgb = None
        self.done = False
        self.create_subscription(Image, '/camera/image_raw', self.on_rgb, 1)
        self.create_subscription(Image, '/camera/depth', self.on_depth, 1)
        self.get_logger().info('Waiting for camera and depth frames...')

    def on_rgb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def on_depth(self, msg):
        if self.rgb is None or self.done:
            return
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Map 0.2-1.0 m to 255-0 so near objects are bright
        norm = np.clip((depth - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH), 0.0, 1.0)
        depth_u8 = ((1.0 - norm) * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)

        rgb = self.rgb.copy()
        if rgb.shape[:2] != depth_color.shape[:2]:
            depth_color = cv2.resize(depth_color, (rgb.shape[1], rgb.shape[0]))

        cv2.putText(rgb, 'Camera', (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (255, 255, 255), 2)
        cv2.putText(depth_color, 'MiDaS depth (near = bright)', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        os.makedirs(OUT_DIR, exist_ok=True)
        cv2.imwrite(os.path.join(OUT_DIR, 'camera-frame.png'), self.rgb)
        cv2.imwrite(os.path.join(OUT_DIR, 'depth-frame.png'), depth_color)
        cv2.imwrite(os.path.join(OUT_DIR, 'rgb-vs-depth.png'),
                    np.hstack([rgb, depth_color]))
        self.get_logger().info(f'Saved images to {OUT_DIR}')
        self.done = True


def main():
    rclpy.init()
    node = DepthSnapshot()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
