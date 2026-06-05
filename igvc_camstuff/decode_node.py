#!/usr/bin/env python3
"""
decode_node.py
Compressed -> mono8 image_raw. Subscribes BEST_EFFORT (matches camera_node),
publishes image_raw RELIABLE so RELIABLE consumers (e.g. camera_calibration)
connect. Preserves each frame's exact header/stamp. Logs rx/tx rates.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import cv2

class DecodeNode(Node):
    def __init__(self):
        super().__init__('decode_node')
        sub_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        pub_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.pub_l = self.create_publisher(Image, '/camera_left/image_raw',  pub_qos)
        self.pub_r = self.create_publisher(Image, '/camera_right/image_raw', pub_qos)
        self.create_subscription(CompressedImage, '/camera_left/compressed',  self.cb_l, sub_qos)
        self.create_subscription(CompressedImage, '/camera_right/compressed', self.cb_r, sub_qos)
        self.rx_l = self.rx_r = self.tx_l = self.tx_r = 0
        self.create_timer(5.0, self.stats)
        self.get_logger().info('decode_node up: compressed -> image_raw (mono8, RELIABLE)')

    def decode_pub(self, msg, pub):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return False
        out = Image()
        out.header = msg.header
        out.height = img.shape[0]; out.width = img.shape[1]
        out.encoding = 'mono8'; out.is_bigendian = 0; out.step = img.shape[1]
        out.data = img.tobytes()
        pub.publish(out)
        return True

    def cb_l(self, msg):
        self.rx_l += 1
        if self.decode_pub(msg, self.pub_l):
            self.tx_l += 1

    def cb_r(self, msg):
        self.rx_r += 1
        if self.decode_pub(msg, self.pub_r):
            self.tx_r += 1

    def stats(self):
        self.get_logger().info(
            f'rx L={self.rx_l/5:.1f} R={self.rx_r/5:.1f} | '
            f'tx L={self.tx_l/5:.1f} R={self.tx_r/5:.1f} Hz')
        self.rx_l = self.rx_r = self.tx_l = self.tx_r = 0

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DecodeNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
