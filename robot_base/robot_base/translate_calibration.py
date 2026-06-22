#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TranslateCalibration(Node):
    def __init__(self, linear_speed, target_dist):
        super().__init__('translate_calibration')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = linear_speed
        self.duration = abs(target_dist / linear_speed)

    def run(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        start = time.time()
        rate_hz = 20
        self.get_logger().info(f'Maju {self.duration:.2f}s pada {self.linear_speed} m/s')
        while time.time() - start < self.duration:
            self.pub.publish(msg)
            time.sleep(1.0 / rate_hz)
        self.pub.publish(Twist())
        self.get_logger().info('Selesai, robot di-stop.')


def main():
    rclpy.init()
    linear_speed = 0.15
    target_dist = 0.5  # maju 50cm
    node = TranslateCalibration(linear_speed, target_dist)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()