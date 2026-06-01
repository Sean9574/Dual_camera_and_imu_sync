#!/usr/bin/env python3
"""
camera_display_node.py
Subscribes to /camera_left and /camera_right and displays
via GStreamer xvimagesink. Runs standalone, separate from camera_node.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import threading
import os

Gst.init(None)

class GstDisplay:
    def __init__(self, width, height, title):
        self.width  = width
        self.height = height

        pipeline_str = (
            f'appsrc name=src format=time caps=video/x-raw,format=BGR,'
            f'width={width},height={height},framerate=30/1 ! '
            f'videoconvert ! '
            f'videoscale ! video/x-raw,width=640,height=400 ! '
            f'xvimagesink sync=false'
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.src      = self.pipeline.get_by_name('src')
        self.src.set_property('block', False)
        self.pipeline.set_state(Gst.State.PLAYING)

    def push_frame(self, frame):
        data = frame.tobytes()
        buf  = Gst.Buffer.new_wrapped(data)
        self.src.emit('push-buffer', buf)


class CameraDisplayNode(Node):
    def __init__(self):
        super().__init__('camera_display_node')

        display = os.environ.get('DISPLAY', 'localhost:10.0')
        os.environ['DISPLAY'] = display
        self.get_logger().info(f'Display node started on DISPLAY={display}')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.disp_left  = None
        self.disp_right = None

        self.create_subscription(Image, '/camera_left',  self.cb_left,  qos)
        self.create_subscription(Image, '/camera_right', self.cb_right, qos)

        self.loop   = GLib.MainLoop()
        self.thread = threading.Thread(target=self.loop.run, daemon=True)
        self.thread.start()

    def cb_left(self, msg):
        if self.disp_left is None:
            self.disp_left = GstDisplay(msg.width, msg.height, 'Camera Left')
        frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)
        )
        self.disp_left.push_frame(frame)

    def cb_right(self, msg):
        if self.disp_right is None:
            self.disp_right = GstDisplay(msg.width, msg.height, 'Camera Right')
        frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)
        )
        self.disp_right.push_frame(frame)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CameraDisplayNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
