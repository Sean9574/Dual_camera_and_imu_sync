#!/usr/bin/env python3
import struct

import numpy as np
import rclpy
import serial
from cobs import cobs
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

# ----------------------------------------------------------------------
# SENSOR FRAME -> PUBLISHED IMU FRAME REMAP
#
# Edit ONLY this matrix when you need to remap IMU axes.
# Rows = published x,y,z
# Cols = raw sensor x,y,z
#
# Identity example:
#   pub = raw
#
# Flip Z example:
#   [[1,0,0],
#    [0,1,0],
#    [0,0,-1]]
#
# 180 deg about X example:
#   [[1,0,0],
#    [0,-1,0],
#    [0,0,-1]]
#
# 180 deg about Y example:
#   [[-1,0,0],
#    [0,1,0],
#    [0,0,-1]]
# ----------------------------------------------------------------------
AXIS_MAP = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=float)


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gyro_scale', 1.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)

        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.trigger_pub = self.create_publisher(Header, '/camera/trigger', 10)

        self.ser = serial.Serial(port, baud, timeout=1)
        self.buffer = b''
        self.err_count = 0

        self.host_time_ref = None
        self.imu_time_ref_ms = None
        self.imu_time_wrap_offset_ms = 0
        self.prev_t_ms = None

        self.get_logger().info(f'IMU connected on {port} at {baud} baud')
        self.get_logger().info(f'Publishing IMU frame_id={self.frame_id}, gyro_scale={self.gyro_scale}')
        self.get_logger().info(f'AXIS_MAP=\n{AXIS_MAP}')
        self.create_timer(0.005, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            self.buffer += self.ser.read(self.ser.in_waiting)

        while b'\x00' in self.buffer:
            frame, self.buffer = self.buffer.split(b'\x00', 1)
            if len(frame) > 5:
                self.parse_cobs(frame)

    def get_sensor_stamp(self, t_ms):
        if self.prev_t_ms is not None and t_ms < self.prev_t_ms:
            if (self.prev_t_ms - t_ms) > 1000:
                self.imu_time_wrap_offset_ms += 2**32
                self.get_logger().warn('Detected IMU t_ms wraparound')

        self.prev_t_ms = t_ms
        t_ms_extended = t_ms + self.imu_time_wrap_offset_ms

        now = self.get_clock().now()

        if self.host_time_ref is None:
            self.host_time_ref = now
            self.imu_time_ref_ms = t_ms_extended
            return now.to_msg()

        dt_sec = (t_ms_extended - self.imu_time_ref_ms) / 1000.0
        stamp_time = self.host_time_ref + Duration(seconds=dt_sec)
        return stamp_time.to_msg()

    def parse_cobs(self, frame):
        try:
            data = cobs.decode(frame)
        except Exception:
            self.err_count += 1
            return

        if len(data) < 87:
            self.err_count += 1
            return

        try:
            offset = 0
            pkt_type = data[offset]
            offset += 1

            seq = struct.unpack_from('<H', data, offset)[0]
            offset += 2

            sensor_id = data[offset]
            offset += 1

            flags = data[offset]
            offset += 1

            t_ms = struct.unpack_from('<I', data, offset)[0]
            offset += 4

            qw, qx, qy, qz = struct.unpack_from('<ffff', data, offset)
            offset += 16

            gx, gy, gz = struct.unpack_from('<fff', data, offset)
            offset += 12

            ax, ay, az = struct.unpack_from('<fff', data, offset)
            offset += 12

            cov_ori_x, cov_ori_y, cov_ori_z = struct.unpack_from('<fff', data, offset)
            offset += 12

            cov_gyr_x, cov_gyr_y, cov_gyr_z = struct.unpack_from('<fff', data, offset)
            offset += 12

            cov_acc_x, cov_acc_y, cov_acc_z = struct.unpack_from('<fff', data, offset)
            offset += 12

            if ax == 0.0 and ay == 0.0 and az == 0.0:
                return

            stamp = self.get_sensor_stamp(t_ms)

            raw_acc = np.array([float(ax), float(ay), float(az)], dtype=float)
            raw_gyr = np.array([float(gx), float(gy), float(gz)], dtype=float)

            pub_acc = AXIS_MAP @ raw_acc
            pub_gyr = AXIS_MAP @ raw_gyr

            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id

            orientation_valid = bool(flags & 0x01)
            if orientation_valid:
                msg.orientation.w = float(qw)
                msg.orientation.x = float(qx)
                msg.orientation.y = float(qy)
                msg.orientation.z = float(qz)
                msg.orientation_covariance = [
                    float(cov_ori_x), 0.0, 0.0,
                    0.0, float(cov_ori_y), 0.0,
                    0.0, 0.0, float(cov_ori_z)
                ]
            else:
                msg.orientation_covariance[0] = -1.0

            msg.angular_velocity.x = float(pub_gyr[0]) * self.gyro_scale
            msg.angular_velocity.y = float(pub_gyr[1]) * self.gyro_scale
            msg.angular_velocity.z = float(pub_gyr[2]) * self.gyro_scale
            msg.angular_velocity_covariance = [
                float(cov_gyr_x), 0.0, 0.0,
                0.0, float(cov_gyr_y), 0.0,
                0.0, 0.0, float(cov_gyr_z)
            ]

            msg.linear_acceleration.x = float(pub_acc[0])
            msg.linear_acceleration.y = float(pub_acc[1])
            msg.linear_acceleration.z = float(pub_acc[2])
            msg.linear_acceleration_covariance = [
                float(cov_acc_x), 0.0, 0.0,
                0.0, float(cov_acc_y), 0.0,
                0.0, 0.0, float(cov_acc_z)
            ]

            self.pub.publish(msg)

            if flags & 0x02:
                h = Header()
                h.stamp = stamp
                h.frame_id = 'camera_trigger'
                self.trigger_pub.publish(h)

        except Exception as e:
            self.err_count += 1
            if self.err_count % 50 == 0:
                self.get_logger().warn(f'IMU parse errors: {self.err_count}, last error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()