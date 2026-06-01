#!/usr/bin/env python3
"""
sync_test.py  -- sensor sync diagnostic for the stereo + IMU rig.
Run (with the sensor launch active):  ros2 run igvc_camstuff sync_test
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
import collections
import statistics

REPORT_PERIOD = 3.0

def sd(vals):
    return statistics.stdev(vals) if len(vals) > 1 else 0.0

class SyncTest(Node):
    def __init__(self):
        super().__init__('sync_test')
        cam_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.n_left = self.n_right = self.n_imu = 0
        self.create_subscription(CompressedImage, '/camera_left/compressed',  self.cb_left,  cam_qos)
        self.create_subscription(CompressedImage, '/camera_right/compressed', self.cb_right, cam_qos)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 50)

        self.imu_stamps    = collections.deque(maxlen=500)
        self.imu_exact     = collections.deque(maxlen=500)
        self.imu_exact_set = set()
        self.last_imu_t    = None
        self.imu_intervals = []
        self.imu_since_frame = 0

        self.last_cam_t    = None
        self.cam_intervals = []
        self.imu_per_frame = []
        self.cam_imu_off    = []
        self.match_hit = self.match_miss = 0   # windowed (reset each report)

        ls = message_filters.Subscriber(self, CompressedImage, '/camera_left/compressed',  qos_profile=cam_qos)
        rs = message_filters.Subscriber(self, CompressedImage, '/camera_right/compressed', qos_profile=cam_qos)
        message_filters.ApproximateTimeSynchronizer([ls, rs], 30, 0.005).registerCallback(self.cb_pair)
        self.lr_off = []
        self.pairs = 0

        self.report_num = 0
        self.create_timer(REPORT_PERIOD, self.report)

    def to_sec(self, s): return s.sec + s.nanosec * 1e-9

    def cb_imu(self, msg):
        self.n_imu += 1
        st = msg.header.stamp; t = self.to_sec(st)
        self.imu_stamps.append(t)
        key = (st.sec, st.nanosec)
        if len(self.imu_exact) == self.imu_exact.maxlen:
            self.imu_exact_set.discard(self.imu_exact[0])
        self.imu_exact.append(key); self.imu_exact_set.add(key)
        if self.last_imu_t is not None:
            d = (t - self.last_imu_t) * 1000.0
            if 0 < d < 50: self.imu_intervals.append(d)
        self.last_imu_t = t
        self.imu_since_frame += 1

    def cb_left(self, msg):
        self.n_left += 1
        st = msg.header.stamp; t = self.to_sec(st)
        if (st.sec, st.nanosec) in self.imu_exact_set: self.match_hit += 1
        else: self.match_miss += 1
        if self.last_cam_t is not None:
            d = (t - self.last_cam_t) * 1000.0
            if 30 < d < 50: self.cam_intervals.append(d)
        self.last_cam_t = t
        self.imu_per_frame.append(self.imu_since_frame); self.imu_since_frame = 0
        if self.imu_stamps:
            nearest = min(self.imu_stamps, key=lambda x: abs(x - t))
            self.cam_imu_off.append(abs(t - nearest) * 1000.0)

    def cb_right(self, msg): self.n_right += 1

    def cb_pair(self, l, r):
        self.pairs += 1
        self.lr_off.append(abs(self.to_sec(l.header.stamp) - self.to_sec(r.header.stamp)) * 1000.0)

    def row(self, label, value, ok):
        mark = "OK " if ok else "!! "
        print(f"  [{mark}] {label:<22} {value}")

    def report(self):
        self.report_num += 1
        rl, rr, ri = self.n_left/REPORT_PERIOD, self.n_right/REPORT_PERIOD, self.n_imu/REPORT_PERIOD
        self.n_left = self.n_right = self.n_imu = 0

        lr   = max(self.lr_off) if self.lr_off else 0.0
        cam  = max(self.cam_imu_off) if self.cam_imu_off else 0.0
        imu_sd = sd(self.imu_intervals)
        cov  = sum(self.imu_per_frame)/len(self.imu_per_frame) if self.imu_per_frame else 0.0
        total = self.match_hit + self.match_miss

        ok_rates = abs(rl-25)<3 and abs(rr-25)<3 and abs(ri-100)<10
        ok_lr    = lr < 1.0
        ok_cam   = cam < 1.0
        ok_match = (self.match_miss == 0 and self.match_hit > 0)
        ok_jit   = imu_sd < 2.0
        ok_cov   = abs(cov-4.0) < 0.3
        all_ok   = all([ok_rates, ok_lr, ok_cam, ok_match, ok_jit, ok_cov])

        print(f"\n┌─ SYNC TEST  #{self.report_num} " + "─"*42)
        self.row("Rates (L/R/IMU Hz)", f"{rl:.0f} / {rr:.0f} / {ri:.0f}", ok_rates)
        self.row("Stereo L-R offset",  f"{lr:.2f} ms", ok_lr)
        self.row("Camera-IMU sync",    f"{cam:.2f} ms", ok_cam)
        self.row("Exact stamp match",  f"{self.match_hit}/{total} (miss {self.match_miss})", ok_match)
        self.row("Timeline jitter",    f"{imu_sd:.2f} ms std", ok_jit)
        self.row("IMU per frame",      f"{cov:.2f} (want 4.0)", ok_cov)
        print("└─ " + ("ALL GOOD ✓" if all_ok else "CHECK FLAGGED ROWS ✗") + " " + "─"*40)

        self.imu_intervals = []; self.cam_intervals = []
        self.cam_imu_off = []; self.imu_per_frame = []
        self.match_hit = self.match_miss = 0; self.pairs = 0
        self.lr_off = []

def main(args=None):
    rclpy.init(args=args)
    node = SyncTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
