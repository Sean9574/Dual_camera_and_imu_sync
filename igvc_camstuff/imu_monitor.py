#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuMonitor(Node):
    def __init__(self):
        super().__init__('imu_monitor')
        self.create_subscription(Imu, '/imu/data', self.cb, 10)
        self.count = 0

    def cb(self, msg):
        self.count += 1
        if self.count % 10 != 0:
            return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        mag = math.sqrt(ax**2 + ay**2 + az**2)

        cax = msg.linear_acceleration_covariance[0]
        cay = msg.linear_acceleration_covariance[4]
        caz = msg.linear_acceleration_covariance[8]
        cgx = msg.angular_velocity_covariance[0]
        cgy = msg.angular_velocity_covariance[4]
        cgz = msg.angular_velocity_covariance[8]

        self.get_logger().info(
            f'\n'
            f'  ── Measurements ────────────────────────────────\n'
            f'  Accel (m/s²)        : x={ax:>8.4f}  y={ay:>8.4f}  z={az:>8.4f}  |a|={mag:>7.4f}\n'
            f'  Gyro  (rad/s)       : x={gx:>8.5f}  y={gy:>8.5f}  z={gz:>8.5f}\n'
            f'  ── Covariances ─────────────────────────────────\n'
            f'  Accel Covariance    : x={cax:.3e}  y={cay:.3e}  z={caz:.3e}\n'
            f'  Gyro  Covariance    : x={cgx:.3e}  y={cgy:.3e}  z={cgz:.3e}'
        )

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuMonitor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
