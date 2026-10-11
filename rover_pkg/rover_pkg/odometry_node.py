import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Robot state
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Velocity state
        self.linear_vel  = 0.0
        self.angular_vel = 0.0

        # Track whether we have received a valid IMU reading yet
        self.imu_valid = False

        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Subscribe to safety-filtered cmd_vel so odometry reflects
        # what the rover actually did, not what was commanded before filtering
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        self.timer = self.create_timer(0.05, self.update_odometry)  # 20Hz

        self.get_logger().info('Odometry node started')

    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        # Only use cmd_vel angular if we don't have valid IMU data
        if not self.imu_valid:
            self.angular_vel = msg.angular.z

    def imu_callback(self, msg):
        # Only use IMU angular velocity if orientation data passed sanity checks
        # (imu_node sets covariance[0] > 0 only on valid cycles)
        if msg.orientation_covariance[0] > 0:
            self.angular_vel = msg.angular_velocity.z
            self.imu_valid   = True

    def update_odometry(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Cap dt to avoid large integration jumps if a cycle was slow
        dt = min(dt, 0.2)

        # Integrate position
        self.theta += self.angular_vel * dt
        self.x     += self.linear_vel * math.cos(self.theta) * dt
        self.y     += self.linear_vel * math.sin(self.theta) * dt

        # Normalize theta to -pi / pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Theta to quaternion (yaw only — ground rover stays flat)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Publish odom -> base_link transform
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel

        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
