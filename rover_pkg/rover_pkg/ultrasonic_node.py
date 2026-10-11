import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

Device.pin_factory = LGPIOFactory()

TRIG = 5
ECHO = 6

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # gpiozero's DistanceSensor handles all the timing internally
        # max_distance in meters, threshold_distance for trigger events
        self.sensor = DistanceSensor(
            echo=ECHO,
            trigger=TRIG,
            max_distance=4.0
        )

        self.publisher = self.create_publisher(Range, '/ultrasonic/range', 10)
        self.timer = self.create_timer(0.1, self.read_and_publish)

        self.get_logger().info('Ultrasonic node started')

    def read_and_publish(self):
        distance_m = self.sensor.distance  # gpiozero returns meters directly

        if distance_m is None:
            return  # no reading, skip

        msg = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_sensor'
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = 0.26
        msg.min_range       = 0.02
        msg.max_range       = 4.00
        msg.range           = float(distance_m)

        self.publisher.publish(msg)

    def destroy_node(self):
        self.sensor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
