#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RotateCalibration(Node):
    def __init__(self, angular_speed, target_rad):
        super().__init__('rotate_calibration')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.angular_speed = angular_speed
        self.duration = abs(target_rad / angular_speed)

    def run(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        start = time.time()
        rate_hz = 20
        self.get_logger().info(f'Berputar {self.duration:.2f}s pada {self.angular_speed} rad/s')
        while time.time() - start < self.duration:
            self.pub.publish(msg)
            time.sleep(1.0 / rate_hz)
        self.pub.publish(Twist())  # stop
        self.get_logger().info('Selesai, robot di-stop.')


def main():
    rclpy.init()
    angular_speed = 0.4   # rad/s, sesuaikan dengan kemampuan motor
    # target_rad = 6.2832   # 360 derajat = 2*pi rad, ganti sesuai kebutuhan
    target_rad = 3.315
    node = RotateCalibration(angular_speed, target_rad)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()