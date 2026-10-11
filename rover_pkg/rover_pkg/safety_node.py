import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String
import math

# Safety thresholds
STOP_DISTANCE  = 0.25  # meters — stop if closer than this
SLOW_DISTANCE  = 0.60  # meters — slow down if closer than this
MAX_TILT_ANGLE = 40.0  # degrees — stop if tilting more than this

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Latest sensor readings
        self.distance     = 999.0
        self.pitch        = 0.0
        self.roll         = 0.0
        self.last_cmd_vel = Twist()

        self.range_sub = self.create_subscription(
            Range, '/ultrasonic/range', self.range_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_callback, 10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',        10)
        self.status_pub = self.create_publisher(String, '/safety_status',  10)

        self.timer = self.create_timer(0.05, self.safety_check)

        self.get_logger().info('Safety node started')

    def range_callback(self, msg):
        self.distance = msg.range

    def imu_callback(self, msg):
        # Use the IMU node's already-filtered orientation quaternion
        # rather than recomputing pitch/roll from raw acceleration here.
        # The IMU node applies a complementary filter — much more stable
        # than a naive atan2 on raw accel, which is sensitive to motor vibration.

        # Check that orientation data is valid
        # (imu_node sets covariance[0] > 0 only when data passed all sanity checks)
        if msg.orientation_covariance[0] <= 0:
            return

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Quaternion to roll (rotation around X)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        self.roll  = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        # Quaternion to pitch (rotation around Y)
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))  # clamp for numerical safety
        self.pitch = math.degrees(math.asin(sinp))

    def cmd_callback(self, msg):
        self.last_cmd_vel = msg

    def safety_check(self):
        cmd    = Twist()
        status = 'OK'

        # Tilt check
        if abs(self.pitch) > MAX_TILT_ANGLE or abs(self.roll) > MAX_TILT_ANGLE:
            status = 'TILT_DANGER'
            self.cmd_pub.publish(cmd)  # empty Twist = stop
            self.get_logger().warn(
                f'TILT DANGER — Pitch: {self.pitch:.1f} Roll: {self.roll:.1f}')

        # Obstacle check
        elif self.distance < STOP_DISTANCE:
            status = 'OBSTACLE_STOP'
            cmd.linear.x  = min(0.0, self.last_cmd_vel.linear.x)
            cmd.angular.z = self.last_cmd_vel.angular.z
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(
                f'OBSTACLE STOP — Distance: {self.distance:.2f}m')

        elif self.distance < SLOW_DISTANCE:
            status = 'OBSTACLE_SLOW'
            scale = (self.distance - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
            cmd.linear.x  = self.last_cmd_vel.linear.x * scale
            cmd.angular.z = self.last_cmd_vel.angular.z
            self.cmd_pub.publish(cmd)

        else:
            cmd = self.last_cmd_vel
            self.cmd_pub.publish(cmd)

        status_msg      = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
