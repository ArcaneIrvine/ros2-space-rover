import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import OutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import time

Device.pin_factory = LGPIOFactory()

# --- Pin definitions (from your verified motor_test.py) ---
BIN1   = 22; BIN2   = 23; AIN1   = 17; AIN2   = 27; STBY1 = 24
BIN1_2 = 26; BIN2_2 = 19; AIN1_2 = 16; AIN2_2 = 20; STBY2 = 25
PWMA   = 12  # right side
PWMB   = 13  # left side

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Direction pins
        self.bin1   = OutputDevice(BIN1)
        self.bin2   = OutputDevice(BIN2)
        self.ain1   = OutputDevice(AIN1)
        self.ain2   = OutputDevice(AIN2)
        self.bin1_2 = OutputDevice(BIN1_2)
        self.bin2_2 = OutputDevice(BIN2_2)
        self.ain1_2 = OutputDevice(AIN1_2)
        self.ain2_2 = OutputDevice(AIN2_2)

        # Standby pins — set HIGH to enable drivers
        self.stby1  = OutputDevice(STBY1, initial_value=True)
        self.stby2  = OutputDevice(STBY2, initial_value=True)

        # PWM speed pins
        self.pwm_a  = PWMOutputDevice(PWMA, frequency=1000)  # right
        self.pwm_b  = PWMOutputDevice(PWMB, frequency=1000)  # left

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('Motor driver node started')

    def set_left(self, speed):
        # speed is -1.0 to 1.0
        if speed > 0:
            self.bin1.on();  self.bin2.off()
            self.bin1_2.on(); self.bin2_2.off()
        elif speed < 0:
            self.bin1.off(); self.bin2.on()
            self.bin1_2.off(); self.bin2_2.on()
        else:
            self.bin1.off(); self.bin2.off()
            self.bin1_2.off(); self.bin2_2.off()
        self.pwm_b.value = abs(speed)

    def set_right(self, speed):
        if speed > 0:
            self.ain1.on();  self.ain2.off()
            self.ain1_2.on(); self.ain2_2.off()
        elif speed < 0:
            self.ain1.off(); self.ain2.on()
            self.ain1_2.off(); self.ain2_2.on()
        else:
            self.ain1.off(); self.ain2.off()
            self.ain1_2.off(); self.ain2_2.off()
        self.pwm_a.value = abs(speed)

    def cmd_vel_callback(self, msg):
        linear  = msg.linear.x   # -1.0 to 1.0
        angular = msg.angular.z  # -1.0 to 1.0

        left_speed  = linear - angular
        right_speed = linear + angular

        # Clamp to -1.0 / 1.0
        left_speed  = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        self.set_left(left_speed)
        self.set_right(right_speed)

    def destroy_node(self):
        self.get_logger().info('Shutting down — stopping motors')
        self.set_left(0)
        self.set_right(0)
        self.stby1.off()
        self.stby2.off()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
