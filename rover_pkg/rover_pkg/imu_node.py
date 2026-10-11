import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import math, time

MPU_ADDR     = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43
ACCEL_SCALE  = 16384.0
GYRO_SCALE   = 131.0
MAX_DT       = 0.2  # cap dt to avoid huge integration steps on slow cycles

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.bus = SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
        time.sleep(0.2)

        self.get_logger().info('Calibrating gyro — keep rover still...')
        self.gx_off, self.gy_off, self.gz_off = self._calibrate_gyro()
        self.get_logger().info(
            f'Gyro offsets: {self.gx_off:.1f}, {self.gy_off:.1f}, {self.gz_off:.1f}')

        self.pitch      = 0.0
        self.roll       = 0.0
        self.last_time  = time.time()
        self.skip_count = 0

        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.timer     = self.create_timer(0.05, self.read_and_publish)  # 20Hz

        self.get_logger().info('IMU node started')

    def _read_word(self, reg, retries=3):
        """Returns the signed 16-bit value, or None if all retries fail."""
        for attempt in range(retries):
            try:
                high  = self.bus.read_byte_data(MPU_ADDR, reg)
                low   = self.bus.read_byte_data(MPU_ADDR, reg + 1)
                value = (high << 8) | low
                if value >= 0x8000:
                    value = -((65535 - value) + 1)
                return value
            except OSError:
                if attempt < retries - 1:
                    time.sleep(0.002)
        return None  # unambiguous failure — never silently 0

    def _calibrate_gyro(self, samples=200):
        gx_off = gy_off = gz_off = 0
        valid_samples = 0
        for _ in range(samples):
            gx = self._read_word(GYRO_XOUT_H)
            gy = self._read_word(GYRO_XOUT_H + 2)
            gz = self._read_word(GYRO_XOUT_H + 4)
            if gx is not None and gy is not None and gz is not None:
                gx_off += gx
                gy_off += gy
                gz_off += gz
                valid_samples += 1
            time.sleep(0.005)

        if valid_samples == 0:
            self.get_logger().error('Gyro calibration failed — using zero offsets')
            return 0.0, 0.0, 0.0

        return gx_off / valid_samples, gy_off / valid_samples, gz_off / valid_samples

    def read_and_publish(self):
        now = time.time()
        dt  = now - self.last_time
        self.last_time = now
        dt  = min(dt, MAX_DT)  # cap dt

        ax = self._read_word(ACCEL_XOUT_H)
        ay = self._read_word(ACCEL_XOUT_H + 2)
        az = self._read_word(ACCEL_XOUT_H + 4)
        gx = self._read_word(GYRO_XOUT_H)
        gy = self._read_word(GYRO_XOUT_H + 2)
        gz = self._read_word(GYRO_XOUT_H + 4)

        # Skip if ANY axis failed — partial data corrupts atan2/integration
        if None in (ax, ay, az, gx, gy, gz):
            self.skip_count += 1
            if self.skip_count % 10 == 1:
                self.get_logger().warn(
                    f'Skipping IMU cycle — read failure (total skipped: {self.skip_count})')
            return

        ax_g = ax / ACCEL_SCALE
        ay_g = ay / ACCEL_SCALE
        az_g = az / ACCEL_SCALE
        gx_dps = (gx - self.gx_off) / GYRO_SCALE
        gy_dps = (gy - self.gy_off) / GYRO_SCALE
        gz_dps = (gz - self.gz_off) / GYRO_SCALE

        # Sanity check — total acceleration should be close to 1g
        magnitude = math.sqrt(ax_g**2 + ay_g**2 + az_g**2)
        if magnitude < 0.5 or magnitude > 1.5:
            self.get_logger().warn(f'Implausible accel magnitude: {magnitude:.2f}g — skipping')
            return

        # Complementary filter for pitch and roll
        alpha       = 0.98
        accel_pitch = math.degrees(math.atan2(ay_g, az_g))
        accel_roll  = math.degrees(math.atan2(-ax_g, az_g))
        self.pitch  = alpha * (self.pitch + gy_dps * dt) + (1 - alpha) * accel_pitch
        self.roll   = alpha * (self.roll  + gx_dps * dt) + (1 - alpha) * accel_roll

        # Convert filtered pitch/roll to quaternion so other nodes
        # can use the filtered orientation directly without recomputing
        half_pitch = math.radians(self.pitch) / 2.0
        half_roll  = math.radians(self.roll)  / 2.0

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_sensor'

        # Filtered orientation as quaternion
        msg.orientation.x = math.sin(half_roll) * math.cos(half_pitch)
        msg.orientation.y = math.cos(half_roll) * math.sin(half_pitch)
        msg.orientation.z = 0.0
        msg.orientation.w = math.cos(half_roll) * math.cos(half_pitch)

        # Non-zero covariance signals valid orientation data to subscribers
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01

        # Raw sensor data (already validated above)
        msg.linear_acceleration.x = ax_g * 9.81
        msg.linear_acceleration.y = ay_g * 9.81
        msg.linear_acceleration.z = az_g * 9.81

        msg.angular_velocity.x = math.radians(gx_dps)
        msg.angular_velocity.y = math.radians(gy_dps)
        msg.angular_velocity.z = math.radians(gz_dps)

        self.publisher.publish(msg)

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
