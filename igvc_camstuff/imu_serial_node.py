#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial
import struct
from cobs import cobs

class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.pub         = self.create_publisher(Imu, '/imu/data', 10)
        # Camera trigger timestamps: published when an IMU sample fired GPIO5
        # (firmware flags bit1). Carries that sample's exact stamp.
        self.trigger_pub = self.create_publisher(Header, '/camera/trigger', 10)
        self.ser       = serial.Serial(port, baud, timeout=1)
        self.buffer    = b''
        self.err_count = 0
        self.get_logger().info(f'IMU connected on {port} at {baud} baud')
        self.create_timer(0.005, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            self.buffer += self.ser.read(self.ser.in_waiting)
        while b'\x00' in self.buffer:
            frame, self.buffer = self.buffer.split(b'\x00', 1)
            if len(frame) > 5:
                self.parse_cobs(frame)

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
            pkt_type  = data[offset];                              offset += 1
            seq       = struct.unpack_from('<H', data, offset)[0]; offset += 2
            sensor_id = data[offset];                              offset += 1
            flags     = data[offset];                              offset += 1
            t_ms      = struct.unpack_from('<I', data, offset)[0]; offset += 4
            qw, qx, qy, qz = struct.unpack_from('<ffff', data, offset); offset += 16
            gx, gy, gz      = struct.unpack_from('<fff',  data, offset); offset += 12
            ax, ay, az      = struct.unpack_from('<fff',  data, offset); offset += 12
            cov_ori_x, cov_ori_y, cov_ori_z = struct.unpack_from('<fff', data, offset); offset += 12
            cov_gyr_x, cov_gyr_y, cov_gyr_z = struct.unpack_from('<fff', data, offset); offset += 12
            cov_acc_x, cov_acc_y, cov_acc_z = struct.unpack_from('<fff', data, offset); offset += 12

            if ax == 0.0 and ay == 0.0 and az == 0.0:
                return

            # One stamp shared by the IMU sample and (if triggered) the camera frame
            stamp = self.get_clock().now().to_msg()

            msg = Imu()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'imu_link'

            orientation_valid = bool(flags & 0x01)
            if orientation_valid:
                msg.orientation.w = float(qw)
                msg.orientation.x = float(qx)
                msg.orientation.y = float(qy)
                msg.orientation.z = float(qz)
                msg.orientation_covariance = [
                    cov_ori_x, 0.0, 0.0,
                    0.0, cov_ori_y, 0.0,
                    0.0, 0.0, cov_ori_z
                ]
            else:
                msg.orientation_covariance[0] = -1.0

            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)
            msg.angular_velocity_covariance = [
                cov_gyr_x, 0.0, 0.0,
                0.0, cov_gyr_y, 0.0,
                0.0, 0.0, cov_gyr_z
            ]

            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)
            msg.linear_acceleration_covariance = [
                cov_acc_x, 0.0, 0.0,
                0.0, cov_acc_y, 0.0,
                0.0, 0.0, cov_acc_z
            ]

            self.pub.publish(msg)

            # Camera trigger flag (bit1): this exact sample fired GPIO5.
            if flags & 0x02:
                h = Header()
                h.stamp    = stamp
                h.frame_id = 'camera_trigger'
                self.trigger_pub.publish(h)

        except Exception:
            self.err_count += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuSerialNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
