#!/usr/bin/env python3
"""
sync_test.py
VIO-readiness sync diagnostic for the stereo + IMU rig.

Run (with the sensor launch active):
    ros2 run igvc_camstuff sync_test

Because the camera frame is hardware-stamped with its triggering IMU sample's
time, the camera-IMU OFFSET is zero by construction. The metrics that actually
drive VIO quality are reported here:

  - LOCK:     does each camera frame land exactly on an IMU sample? (offset ~0)
  - L-R:      stereo pair timestamp offset (should be 0)
  - JITTER:   regularity of the IMU + camera timelines (std dev). THIS is the
              number to minimize for the best VIO -- it reflects timestamp
              wobble from serial arrival timing.
  - COVERAGE: IMU samples per camera frame (should be ~4.0 at 100Hz / 25fps)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
import collections
import statistics

REPORT_PERIOD = 3.0

def stats(vals):
    if not vals:
        return (0.0, 0.0, 0.0, 0.0)
    avg = sum(vals) / len(vals)
    sd  = statistics.stdev(vals) if len(vals) > 1 else 0.0
    return (avg, sd, min(vals), max(vals))

def verdict(ok):
    return "PASS" if ok else "WARN"

class SyncTest(Node):
    def __init__(self):
        super().__init__('sync_test')

        cam_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # rates
        self.n_left = self.n_right = self.n_imu = 0
        self.create_subscription(CompressedImage, '/camera_left/compressed',  self.cb_left,  cam_qos)
        self.create_subscription(CompressedImage, '/camera_right/compressed', self.cb_right, cam_qos)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 50)  # reliable by default

        # IMU timeline
        self.imu_stamps    = collections.deque(maxlen=500)
        self.last_imu_t    = None
        self.imu_intervals = []
        self.imu_since_frame = 0

        # camera timeline
        self.last_cam_t    = None
        self.cam_intervals = []
        self.cam_gaps      = 0
        self.imu_per_frame = []
        self.cam_imu_offsets = []

        # stereo
        ls = message_filters.Subscriber(self, CompressedImage, '/camera_left/compressed',  qos_profile=cam_qos)
        rs = message_filters.Subscriber(self, CompressedImage, '/camera_right/compressed', qos_profile=cam_qos)
        ts = message_filters.ApproximateTimeSynchronizer([ls, rs], 30, 0.005)
        ts.registerCallback(self.cb_pair)
        self.lr_offsets = []
        self.pairs = 0

        self.create_timer(REPORT_PERIOD, self.report)
        print("\n" + "=" * 70)
        print("  VIO SYNC TEST  (Ctrl-C to stop)")
        print("  offset is hardware-locked to 0; minimize JITTER for best VIO")
        print("=" * 70)

    def to_sec(self, s):
        return s.sec + s.nanosec * 1e-9

    def cb_imu(self, msg):
        self.n_imu += 1
        t = self.to_sec(msg.header.stamp)
        self.imu_stamps.append(t)
        if self.last_imu_t is not None:
            d = (t - self.last_imu_t) * 1000.0
            if 0 < d < 50:
                self.imu_intervals.append(d)
        self.last_imu_t = t
        self.imu_since_frame += 1

    def cb_left(self, msg):
        self.n_left += 1
        t = self.to_sec(msg.header.stamp)
        if self.last_cam_t is not None:
            d = (t - self.last_cam_t) * 1000.0
            if 30 < d < 50:
                self.cam_intervals.append(d)
            elif d >= 50:
                self.cam_gaps += 1
        self.last_cam_t = t
        self.imu_per_frame.append(self.imu_since_frame)
        self.imu_since_frame = 0
        if self.imu_stamps:
            nearest = min(self.imu_stamps, key=lambda x: abs(x - t))
            self.cam_imu_offsets.append(abs(t - nearest) * 1000.0)

    def cb_right(self, msg):
        self.n_right += 1

    def cb_pair(self, l, r):
        self.pairs += 1
        sl = self.to_sec(l.header.stamp)
        sr = self.to_sec(r.header.stamp)
        self.lr_offsets.append(abs(sl - sr) * 1000.0)

    def report(self):
        rl = self.n_left / REPORT_PERIOD
        rr = self.n_right / REPORT_PERIOD
        ri = self.n_imu / REPORT_PERIOD
        self.n_left = self.n_right = self.n_imu = 0

        imu_avg, imu_sd, imu_min, imu_max = stats(self.imu_intervals)
        cam_avg, cam_sd, cam_min, cam_max = stats(self.cam_intervals)
        lr_avg = sum(self.lr_offsets)/len(self.lr_offsets) if self.lr_offsets else 0.0
        lr_max = max(self.lr_offsets) if self.lr_offsets else 0.0
        ci_avg = sum(self.cam_imu_offsets)/len(self.cam_imu_offsets) if self.cam_imu_offsets else 0.0
        ci_max = max(self.cam_imu_offsets) if self.cam_imu_offsets else 0.0
        cov_avg = sum(self.imu_per_frame)/len(self.imu_per_frame) if self.imu_per_frame else 0.0

        print("\n" + "-" * 70)
        print(f"  Rates    | left={rl:5.1f}Hz  right={rr:5.1f}Hz  imu={ri:6.1f}Hz")
        print(f"  Stereo   | L-R offset avg={lr_avg:4.2f}ms max={lr_max:4.2f}ms  pairs={self.pairs}")
        print(f"  Camera-Imu Sync | frame->nearest IMU sample avg={ci_avg:4.2f}ms max={ci_max:4.2f}ms "
              f"[{verdict(ci_max < 1.0)}]")
        print(f"  JITTER   | IMU interval  avg={imu_avg:5.2f}ms std={imu_sd:4.2f}ms "
              f"(min={imu_min:4.1f} max={imu_max:4.1f})  <-- minimize for VIO [{verdict(imu_sd < 2.0)}]")
        print(f"           | Cam interval  avg={cam_avg:5.2f}ms std={cam_sd:4.2f}ms  gaps={self.cam_gaps}")
        print(f"  COVERAGE | IMU samples per frame avg={cov_avg:4.2f} (target 4.0) "
              f"[{verdict(abs(cov_avg-4.0) < 0.3)}]")

        self.imu_intervals = []
        self.cam_intervals = []
        self.cam_imu_offsets = []
        self.imu_per_frame = []

def main(args=None):
    rclpy.init(args=args)
    node = SyncTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nSync test stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
