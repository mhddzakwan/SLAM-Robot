#!/usr/bin/env python3
import math
import threading
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_diameter', 0.052)
        self.declare_parameter('wheelbase', 0.179)
        self.declare_parameter('counts_per_rev', 1012.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.counts_per_rev = self.get_parameter('counts_per_rev').value

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.lock = threading.Lock()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_ticks_l = None
        self.prev_ticks_r = None
        self.prev_time = self.get_clock().now()

        self.create_timer(0.01, self.read_serial)
        self.get_logger().info(f'Serial bridge aktif di {port} @ {baud}')

    def cmd_vel_callback(self, msg: Twist):
        line = f'{msg.linear.x:.3f},{msg.angular.z:.3f}\n'
        with self.lock:
            self.ser.write(line.encode('utf-8'))

    def read_serial(self):
        with self.lock:
            if self.ser.in_waiting == 0:
                return
            raw = self.ser.readline().decode('utf-8', errors='ignore').strip()

        if not raw.startswith('#') or raw == '#ROBOT_READY':
            return

        fields = raw[1:].split(',')
        if len(fields) != 11:
            return

        try:
            (_, ticks_l, ticks_r, rpm_l, rpm_r,
             ax, ay, az, gx, gy, gz) = fields
            ticks_l, ticks_r = int(ticks_l), int(ticks_r)
            ax, ay, az = float(ax), float(ay), float(az)
            gx, gy, gz = float(gx), float(gy), float(gz)
        except ValueError:
            return

        now = self.get_clock().now()
        self.publish_odom(ticks_l, ticks_r, now)
        self.publish_imu(ax, ay, az, gx, gy, gz, now)

    def publish_odom(self, ticks_l, ticks_r, stamp):
        if self.prev_ticks_l is None:
            self.prev_ticks_l, self.prev_ticks_r = ticks_l, ticks_r
            self.prev_time = stamp
            return

        dt = (stamp - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return

        delta_l = ticks_l - self.prev_ticks_l
        delta_r = ticks_r - self.prev_ticks_r
        self.prev_ticks_l, self.prev_ticks_r = ticks_l, ticks_r
        self.prev_time = stamp

        dist_per_tick = (math.pi * self.wheel_diameter) / self.counts_per_rev
        dist_l = delta_l * dist_per_tick
        dist_r = delta_r * dist_per_tick

        dist_center = (dist_l + dist_r) / 2.0
        dtheta = (dist_r - dist_l) / self.wheelbase

        self.x += dist_center * math.cos(self.theta + dtheta / 2.0)
        self.y += dist_center * math.sin(self.theta + dtheta / 2.0)
        self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))

        vx = dist_center / dt
        vth = dtheta / dt

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

    def publish_imu(self, ax, ay, az, gx, gy, gz, stamp):
      imu = Imu()
      imu.header.stamp = stamp.to_msg()
      imu.header.frame_id = 'imu_link'
      imu.linear_acceleration.x = ax
      imu.linear_acceleration.y = ay
      imu.linear_acceleration.z = az
      imu.angular_velocity.x = gx
      imu.angular_velocity.y = gy
      imu.angular_velocity.z = gz

      imu.orientation_covariance[0] = -1.0  # tidak ada data orientasi absolut

      accel_var = 0.05
      gyro_var = 0.01
      imu.linear_acceleration_covariance[0] = accel_var
      imu.linear_acceleration_covariance[4] = accel_var
      imu.linear_acceleration_covariance[8] = accel_var
      imu.angular_velocity_covariance[0] = gyro_var
      imu.angular_velocity_covariance[4] = gyro_var
      imu.angular_velocity_covariance[8] = gyro_var

      self.imu_pub.publish(imu)

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()