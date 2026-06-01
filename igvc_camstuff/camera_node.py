#!/usr/bin/env python3
"""
camera_node.py
Hardware-triggered stereo capture (MJPG passthrough -> CompressedImage).
Each frame is stamped with the EXACT integer timestamp of the IMU sample that
fired its trigger (from /camera/trigger), in integer nanoseconds (no float
round-trip) so the stamp is byte-identical to the IMU sample's stamp.
Frames that arrive before the first trigger (startup) are skipped, so no frame
is ever emitted with an un-anchored wall-clock stamp.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import threading

Gst.init(None)

TRIGGER_PERIOD_NS = 40_000_000   # 40ms, fallback predictor only

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

        self.create_subscription(Header, '/camera/trigger', self.cb_trigger, 50)
        self.latest_trigger_ns   = None
        self.last_trigger_val_ns = None
        self.last_assigned_ns    = None
        self.trig_lock = threading.Lock()

        self.shared_ns  = None
        self.stamp_lock = threading.Lock()

        self.cam_left  = GstCamera(left,  w, h, self.cb_left)
        self.cam_right = GstCamera(right, w, h, self.cb_right)

        self.loop   = GLib.MainLoop()
        self.thread = threading.Thread(target=self.loop.run, daemon=True)
        self.thread.start()

        self.n_left = self.n_right = self.predicted = self.skipped = 0
        self.create_timer(5.0, self.stats)

        self.get_logger().info(
            f'Camera node started (HW trigger, exact integer-ns stamps)\n'
            f'  Left:  /dev/video{left} -> /camera_left/compressed\n'
            f'  Right: /dev/video{right} -> /camera_right/compressed\n'
            f'  Resolution: {w}x{h}'
        )

    def cb_trigger(self, msg):
        ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        with self.trig_lock:
            self.latest_trigger_ns = ns

    def next_stamp_ns(self):
        with self.trig_lock:
            ns = self.latest_trigger_ns
        if ns is None:
            return None  # no trigger yet -> caller skips this frame
        if ns == self.last_trigger_val_ns:
            self.predicted += 1
            out = (self.last_assigned_ns + TRIGGER_PERIOD_NS
                   if self.last_assigned_ns is not None else ns)
        else:
            out = ns
            self.last_trigger_val_ns = ns
        self.last_assigned_ns = out
        return out

    def make_msg(self, data, ns):
        msg = CompressedImage()
        msg.header.stamp.sec     = ns // 1_000_000_000
        msg.header.stamp.nanosec = ns %  1_000_000_000
        msg.header.frame_id      = 'camera_link'
        msg.format               = 'jpeg'
        msg.data                 = data
        return msg

    def cb_left(self, data):
        ns = self.next_stamp_ns()
        if ns is None:
            self.skipped += 1
            return  # no trigger stamp yet; don't emit an un-anchored frame
        with self.stamp_lock:
            self.shared_ns = ns
        self.pub_left.publish(self.make_msg(data, ns))
        self.n_left += 1

    def cb_right(self, data):
        with self.stamp_lock:
            ns = self.shared_ns
        if ns is None:
            self.skipped += 1
            return  # left hasn't anchored a stamp yet
        self.pub_right.publish(self.make_msg(data, ns))
        self.n_right += 1

    def stats(self):
        self.get_logger().info(
            f'Rates: left={self.n_left/5:.1f}Hz right={self.n_right/5:.1f}Hz '
            f'predicted={self.predicted} skipped={self.skipped}'
        )
        self.n_left = self.n_right = 0


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CameraNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
