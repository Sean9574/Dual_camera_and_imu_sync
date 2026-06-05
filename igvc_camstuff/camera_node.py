#!/usr/bin/env python3
"""
camera_node.py
Hardware-triggered stereo capture via GStreamer (deferred-gi to dodge the
gi+rclpy segfault). Left and right frames are explicitly PAIRED before publish
and stamped with a single shared trigger timestamp, so the two cameras can never
drift onto different or duplicate stamps (fixes the racy shared_ns hand-off).
Publishes /camera_{left,right}/compressed, each frame carrying the exact
integer-ns timestamp of its triggering IMU sample.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from collections import deque
import threading
import glob
import subprocess
import os

TRIGGER_PERIOD_NS = 40_000_000
CAMERA_NAME_MATCH = 'OV9281'
MAX_PENDING = 5   # drop oldest if one camera gets this far ahead (resync guard)


def card_name(dev):
    try:
        out = subprocess.run(['v4l2-ctl', '-d', dev, '--info'],
                             capture_output=True, text=True, timeout=2).stdout
        for line in out.splitlines():
            if 'Card type' in line:
                return line.split(':', 1)[1].strip()
    except Exception:
        pass
    return ''


def has_mjpg(dev):
    try:
        out = subprocess.run(['v4l2-ctl', '-d', dev, '--list-formats'],
                             capture_output=True, text=True, timeout=2).stdout
        return 'MJPG' in out or 'Motion-JPEG' in out
    except Exception:
        return False


def usb_port_key(dev):
    try:
        real = os.path.realpath(f'/sys/class/video4linux/{os.path.basename(dev)}/device')
        return os.path.basename(real).split(':')[0]
    except Exception:
        return dev


def vnum(dev):
    return int(''.join(filter(str.isdigit, os.path.basename(dev))) or 0)


def discover_ov9281():
    by_port = {}
    for dev in sorted(glob.glob('/dev/video*'), key=vnum):
        if CAMERA_NAME_MATCH in card_name(dev) and has_mjpg(dev):
            port = usb_port_key(dev)
            if port not in by_port:
                by_port[port] = dev
    return [by_port[p] for p in sorted(by_port)]


class GstCamera:
    """Built only after gi is imported; Gst module passed in."""
    def __init__(self, Gst, device, width, height, callback):
        self.Gst = Gst
        self.callback = callback
        pipeline_str = (
            f'v4l2src device={device} ! '
            f'image/jpeg,width={width},height={height} ! '
            f'appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
        )
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name('sink')
        self.sink.connect('new-sample', self.on_frame)
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(f'Failed to start pipeline for {device}')

    def on_frame(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        ok, mapinfo = buf.map(self.Gst.MapFlags.READ)
        if ok:
            data = bytes(mapinfo.data)
            buf.unmap(mapinfo)
            self.callback(data)
        return self.Gst.FlowReturn.OK


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')   # created BEFORE any gi import
        self.declare_parameter('device_left',  '')
        self.declare_parameter('device_right', '')
        self.declare_parameter('swap_lr', False)
        self.declare_parameter('width',  1280)
        self.declare_parameter('height', 800)

        man_l = self.get_parameter('device_left').value
        man_r = self.get_parameter('device_right').value
        swap  = self.get_parameter('swap_lr').value
        self.w = self.get_parameter('width').value
        self.h = self.get_parameter('height').value

        if man_l and man_r:
            self.left, self.right = man_l, man_r
        else:
            devs = discover_ov9281()
            if len(devs) < 2:
                raise RuntimeError(f'Expected 2 OV9281 cameras, found {len(devs)}: {devs}')
            self.left, self.right = devs[0], devs[1]
        if swap:
            self.left, self.right = self.right, self.left

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.pub_left  = self.create_publisher(CompressedImage, '/camera_left/compressed',  qos)
        self.pub_right = self.create_publisher(CompressedImage, '/camera_right/compressed', qos)

        self.create_subscription(Header, '/camera/trigger', self.cb_trigger, 50)
        self.latest_trigger_ns = self.last_trigger_val_ns = self.last_assigned_ns = None
        self.trig_lock = threading.Lock()

        # explicit L/R pairing replaces the racy shared_ns
        self.pending_left  = deque()
        self.pending_right = deque()
        self.pair_lock = threading.Lock()

        self.n_left = self.n_right = self.predicted = self.skipped = 0
        self.create_timer(5.0, self.stats)
        self.get_logger().info(f'camera_node created; cameras L={self.left} R={self.right}')

    def start_capture(self):
        """Deferred gi import + GStreamer pipelines AFTER the node exists."""
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst, GLib
        Gst.init(None)
        self._GLib = GLib
        self.cam_left  = GstCamera(Gst, self.left,  self.w, self.h, self.cb_left)
        self.cam_right = GstCamera(Gst, self.right, self.w, self.h, self.cb_right)
        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()
        self.get_logger().info(f'GStreamer capture started ({self.w}x{self.h})')

    def cb_trigger(self, msg):
        ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        with self.trig_lock:
            self.latest_trigger_ns = ns

    def next_stamp_ns(self):
        with self.trig_lock:
            ns = self.latest_trigger_ns
        if ns is None:
            return None
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
        m = CompressedImage()
        m.header.stamp.sec     = ns // 1_000_000_000
        m.header.stamp.nanosec = ns %  1_000_000_000
        m.header.frame_id = 'camera_link'
        m.format = 'jpeg'
        m.data = data
        return m

    def cb_left(self, data):
        with self.pair_lock:
            self.pending_left.append(data)
            if len(self.pending_left) > MAX_PENDING:
                self.pending_left.popleft()
                self.skipped += 1
            self._try_pair()

    def cb_right(self, data):
        with self.pair_lock:
            self.pending_right.append(data)
            if len(self.pending_right) > MAX_PENDING:
                self.pending_right.popleft()
                self.skipped += 1
            self._try_pair()

    def _try_pair(self):
        # caller holds pair_lock. Match left+right and stamp both identically.
        while self.pending_left and self.pending_right:
            ld = self.pending_left.popleft()
            rd = self.pending_right.popleft()
            ns = self.next_stamp_ns()
            if ns is None:
                continue
            self.pub_left.publish(self.make_msg(ld, ns))
            self.pub_right.publish(self.make_msg(rd, ns))
            self.n_left += 1
            self.n_right += 1

    def stats(self):
        self.get_logger().info(
            f'Rates: left={self.n_left/5:.1f}Hz right={self.n_right/5:.1f}Hz '
            f'predicted={self.predicted} skipped={self.skipped}')
        self.n_left = self.n_right = 0


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    node.start_capture()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
