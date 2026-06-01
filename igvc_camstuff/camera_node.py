#!/usr/bin/env python3
"""
camera_node.py
Hardware-triggered stereo capture (MJPG passthrough -> CompressedImage).
Both OV9281 cameras share the XIAO GPIO5 trigger, which fires on every 4th IMU
sample. The firmware flags that IMU sample, and imu_serial_node republishes its
exact timestamp on /camera/trigger. This node stamps each stereo pair with the
latest trigger timestamp -> true hardware-level camera-IMU alignment, no
inference. Robust to camera drops: the IMU is reliable, so each event's trigger
stamp always arrives (just before its frame); a dropped frame simply picks up
the next correct trigger stamp.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import threading

Gst.init(None)

TRIGGER_PERIOD = 0.040   # 25fps, used only as a fallback predictor

class GstCamera:
    def __init__(self, device_id, width, height, callback):
        self.callback = callback
        device = f'/dev/video{device_id}'
        pipeline_str = (
            f'v4l2src device={device} ! '
            f'image/jpeg,width={width},height={height} ! '
            f'appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
        )
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink     = self.pipeline.get_by_name('sink')
        self.sink.connect('new-sample', self.on_frame)
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(f'Failed to start pipeline for {device}')

    def on_frame(self, sink):
        sample = sink.emit('pull-sample')
        buf    = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if ok:
            data = bytes(mapinfo.data)
            buf.unmap(mapinfo)
            self.callback(data)
        return Gst.FlowReturn.OK


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_left',  0)
        self.declare_parameter('device_right', 2)
        self.declare_parameter('width',  1280)
        self.declare_parameter('height', 800)

        left  = self.get_parameter('device_left').value
        right = self.get_parameter('device_right').value
        w     = self.get_parameter('width').value
        h     = self.get_parameter('height').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2
        )

        self.pub_left  = self.create_publisher(CompressedImage, '/camera_left/compressed',  qos)
        self.pub_right = self.create_publisher(CompressedImage, '/camera_right/compressed', qos)

        # Hardware trigger timestamps from the IMU node (flagged sample times)
        self.create_subscription(Header, '/camera/trigger', self.cb_trigger, 50)
        self.latest_trigger     = None   # seconds, most recent trigger stamp
        self.last_trigger_value = None   # last trigger value actually consumed
        self.last_assigned      = None   # last stamp assigned to a frame
        self.trig_lock = threading.Lock()

        self.shared_stamp = None
        self.stamp_lock   = threading.Lock()

        self.cam_left  = GstCamera(left,  w, h, self.cb_left)
        self.cam_right = GstCamera(right, w, h, self.cb_right)

        self.loop   = GLib.MainLoop()
        self.thread = threading.Thread(target=self.loop.run, daemon=True)
        self.thread.start()

        self.n_left = self.n_right = self.predicted = 0
        self.create_timer(5.0, self.stats)

        self.get_logger().info(
            f'Camera node started (HW trigger, stamped from /camera/trigger)\n'
            f'  Left:  /dev/video{left} -> /camera_left/compressed\n'
            f'  Right: /dev/video{right} -> /camera_right/compressed\n'
            f'  Resolution: {w}x{h}'
        )

    def cb_trigger(self, msg):
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        with self.trig_lock:
            self.latest_trigger = t

    def next_stamp(self):
        with self.trig_lock:
            t = self.latest_trigger
        if t is None:
            # No trigger seen yet -> fall back to wall clock
            return self.get_clock().now().nanoseconds * 1e-9
        if t == self.last_trigger_value:
            # Frame arrived before a fresh trigger msg -> predict to avoid a dup
            self.predicted += 1
            stamp = (self.last_assigned + TRIGGER_PERIOD
                     if self.last_assigned is not None else t)
        else:
            stamp = t
            self.last_trigger_value = t
        self.last_assigned = stamp
        return stamp

    def make_msg(self, data, stamp_sec):
        msg = CompressedImage()
        msg.header.stamp    = Time(seconds=stamp_sec).to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format          = 'jpeg'
        msg.data            = data
        return msg

    def cb_left(self, data):
        stamp = self.next_stamp()
        with self.stamp_lock:
            self.shared_stamp = stamp
        self.pub_left.publish(self.make_msg(data, stamp))
        self.n_left += 1

    def cb_right(self, data):
        with self.stamp_lock:
            stamp = self.shared_stamp
        if stamp is None:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self.pub_right.publish(self.make_msg(data, stamp))
        self.n_right += 1

    def stats(self):
        self.get_logger().info(
            f'Rates: left={self.n_left/5:.1f}Hz right={self.n_right/5:.1f}Hz '
            f'predicted={self.predicted}'
        )
        self.n_left = self.n_right = 0


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CameraNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
