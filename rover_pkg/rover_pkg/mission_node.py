import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Range
import json
import math

# Autonomous behaviour parameters
EXPLORE_SPEED    = 0.3   # forward speed during exploration
TURN_SPEED       = 0.4   # turning speed when avoiding
CLEAR_DISTANCE   = 0.8   # meters — consider path clear above this

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # Current mode — starts in manual
        self.mode           = 'manual'
        self.distance       = 999.0
        self.last_detection = []
        self.turn_direction = 1.0   # 1.0 = left, -1.0 = right
        self.state          = 'exploring'  # exploring / avoiding

        # Subscribe to mode switch commands
        self.mode_sub = self.create_subscription(
            String, '/rover_mode', self.mode_callback, 10)

        # Subscribe to safety status
        self.safety_sub = self.create_subscription(
            String, '/safety_status', self.safety_callback, 10)

        # Subscribe to ultrasonic range
        self.range_sub = self.create_subscription(
            Range, '/ultrasonic/range', self.range_callback, 10)

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            String, '/detections', self.detection_callback, 10)

        # Publish commands to safety layer
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_raw', 10)

        # Publish mission status
        self.status_pub = self.create_publisher(
            String, '/mission_status', 10)

        # Autonomous behaviour loop at 10Hz
        self.timer = self.create_timer(0.1, self.mission_loop)

        self.get_logger().info('Mission node started — mode: MANUAL')

    def mode_callback(self, msg):
        new_mode = msg.data.lower()
        if new_mode in ['manual', 'autonomous']:
            self.mode = new_mode
            self.state = 'exploring'
            self.get_logger().info(f'Mode switched to: {self.mode.upper()}')
            if self.mode == 'manual':
                # Stop rover when switching to manual
                self.cmd_pub.publish(Twist())

    def safety_callback(self, msg):
        self.safety_status = msg.data

    def range_callback(self, msg):
        self.distance = msg.range

    def detection_callback(self, msg):
        try:
            self.last_detection = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def mission_loop(self):
        if self.mode != 'autonomous':
            return

        cmd    = Twist()
        status = String()

        if self.state == 'exploring':
            if self.distance < CLEAR_DISTANCE:
                # Obstacle ahead — switch to avoiding
                self.state          = 'avoiding'
                self.turn_direction = 1.0  # try turning left first
                self.get_logger().info('Obstacle detected — avoiding')
            else:
                # Path clear — drive forward
                cmd.linear.x = EXPLORE_SPEED

                # Log any interesting detections
                if self.last_detection:
                    labels = [d['label'] for d in self.last_detection]
                    self.get_logger().info(f'Objects in view: {labels}')
                    self.last_detection = []

            status.data = f'AUTONOMOUS — EXPLORING — Distance: {self.distance:.2f}m'

        elif self.state == 'avoiding':
            if self.distance >= CLEAR_DISTANCE:
                # Path clear again — resume exploring
                self.state = 'exploring'
                self.get_logger().info('Path clear — resuming exploration')
            else:
                # Turn until clear
                cmd.angular.z = TURN_SPEED * self.turn_direction

            status.data = f'AUTONOMOUS — AVOIDING — Distance: {self.distance:.2f}m'

        self.cmd_pub.publish(cmd)
        self.status_pub.publish(status)

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
