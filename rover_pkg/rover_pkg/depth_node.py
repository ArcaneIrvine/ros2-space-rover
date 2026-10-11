import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import numpy as np
import cv2

class DepthNode(Node):
    def __init__(self):
        super().__init__('depth_node')

        self.previous_depth  = None
        self.smoothing_alpha = 0.6  # weight for current frame vs previous

        self.bridge = CvBridge()
        self.get_logger().info('Loading MiDaS depth model...')
        self.device = torch.device('cpu')  # RPi 5 has no CUDA
        self.model  = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', trust_repo=True)
        self.model.to(self.device)
        self.model.eval()
        self.transform = torch.hub.load(
            'intel-isl/MiDaS', 'transforms', trust_repo=True).small_transform
        self.get_logger().info('MiDaS depth model loaded')

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 1)
        self.depth_pub = self.create_publisher(
            Image, '/camera/depth', 10)

        self.frame_count   = 0
        self.process_every = 5
        self.get_logger().info('Depth node started')

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every != 0:
            return

        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        original_size = frame_bgr.shape[:2]  # (height, width) = (480, 640)

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_rgb_small = cv2.resize(frame_rgb, (256, 192))  # smaller for faster inference

        input_batch = self.transform(frame_rgb_small).to(self.device)

        with torch.no_grad():
            prediction = self.model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=original_size,  # resize back UP to match RGB image exactly
                mode='bicubic',
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()
        
        depth_map = np.maximum(depth_map, 1e-3)

        # Fixed scale factor — NOT recalculated per frame
        # MiDaS small typically outputs values in a roughly consistent range
        # for indoor scenes; we use a fixed divisor instead of per-frame min/max
        DEPTH_SCALE_FACTOR = 151.0   # calibrated from real measurements

        metric_depth = DEPTH_SCALE_FACTOR / depth_map
        metric_depth = np.clip(metric_depth, 0.2, 5.0)
        metric_depth = metric_depth.astype(np.float32)
        metric_depth = cv2.medianBlur(metric_depth.astype(np.float32), 5)
        
        if self.previous_depth is not None and self.previous_depth.shape == metric_depth.shape:
            metric_depth = (self.smoothing_alpha * metric_depth +
                             (1 - self.smoothing_alpha) * self.previous_depth)

        self.previous_depth = metric_depth.copy()

        # Publish as 32-bit float depth image — required format for RTAB-Map
        depth_msg = self.bridge.cv2_to_imgmsg(metric_depth, encoding='32FC1')
        depth_msg.header = msg.header
        self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
