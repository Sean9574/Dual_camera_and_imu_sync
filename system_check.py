#!/usr/bin/env python3
"""
system_check.py
End-to-end hardware + sync validation for the stereo-inertial rig.
Validates the FULL chain, including the image_raw layer OpenVINS consumes
(which sync_test never checked). Keep the rig STILL during the run so the
IMU rest checks are meaningful.

Run:  python3 ~/ros2_ws/src/igvc_camstuff/system_check.py
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, CompressedImage, Image
from std_msgs.msg import Header

DURATION = 15.0  # seconds to collect


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Check(Node):
    def __init__(self):
        super().__init__('system_check')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=50)
        self.imu = []      # (ns, ax,ay,az, gx,gy,gz)
        self.trig = []     # ns
        self.cL = []; self.cR = []      # compressed stamps
        self.rL = []; self.rR = []      # image_raw stamps
        self.create_subscription(Imu, '/imu/data', self.cb_imu, qos)
        self.create_subscription(Header, '/camera/trigger', self.cb_trig, qos)
        self.create_subscription(CompressedImage, '/camera_left/compressed',  lambda m: self.cL.append(ns(m.header.stamp)), qos)
        self.create_subscription(CompressedImage, '/camera_right/compressed', lambda m: self.cR.append(ns(m.header.stamp)), qos)
        self.create_subscription(Image, '/camera_left/image_raw',  lambda m: self.rL.append(ns(m.header.stamp)), qos)
        self.create_subscription(Image, '/camera_right/image_raw', lambda m: self.rR.append(ns(m.header.stamp)), qos)

    def cb_imu(self, m):
        a = m.linear_acceleration; g = m.angular_velocity
        self.imu.append((ns(m.header.stamp), a.x, a.y, a.z, g.x, g.y, g.z))

    def cb_trig(self, m):
        self.trig.append(ns(m.stamp))


def rate(stamps, dur):
    return len(stamps) / dur if dur > 0 else 0.0


def tag(ok, warn=False):
    return "\033[92mPASS\033[0m" if ok else ("\033[93mWARN\033[0m" if warn else "\033[91mFAIL\033[0m")


def report(c):
    dur = DURATION
    print("\n" + "=" * 60)
    print("  SYSTEM CHECK REPORT  (rig should have been STILL)")
    print("=" * 60)

    # ---- IMU ----
    print("\n[IMU]  /imu/data")
    n = len(c.imu)
    r = rate(c.imu, dur)
    print(f"  rate            {r:6.1f} Hz                      {tag(95 <= r <= 105)}")
    if n > 10:
        stamps = sorted(s[0] for s in c.imu)
        gaps = [(stamps[i+1]-stamps[i])/1e6 for i in range(len(stamps)-1)]
        maxgap = max(gaps); ngap = sum(1 for g in gaps if g > 25)
        print(f"  max stamp gap   {maxgap:6.1f} ms  (dropped>25ms: {ngap})  {tag(maxgap < 25)}")
        ax = sum(s[1] for s in c.imu)/n; ay = sum(s[2] for s in c.imu)/n; az = sum(s[3] for s in c.imu)/n
        amag = math.sqrt(ax*ax+ay*ay+az*az)
        print(f"  |accel| at rest {amag:6.3f} m/s^2  (want ~9.81)     {tag(9.6 <= amag <= 10.0, warn=True)}")
        gx = sum(s[4] for s in c.imu)/n; gy = sum(s[5] for s in c.imu)/n; gz = sum(s[6] for s in c.imu)/n
        gmag = math.sqrt(gx*gx+gy*gy+gz*gz)
        print(f"  gyro bias       {gmag:6.4f} rad/s (want ~0 at rest) {tag(gmag < 0.1, warn=True)}")

    # ---- TRIGGER ----
    print("\n[TRIGGER]  /camera/trigger")
    rt = rate(c.trig, dur)
    print(f"  rate            {rt:6.1f} Hz  (want ~25)            {tag(23 <= rt <= 27)}")
    if rt > 0:
        ratio = r / rt
        print(f"  IMU per trigger {ratio:6.2f}     (want ~4.0)          {tag(3.8 <= ratio <= 4.2)}")

    # ---- COMPRESSED SYNC ----
    print("\n[COMPRESSED]  /camera_*/compressed")
    rcl = rate(c.cL, dur); rcr = rate(c.cR, dur)
    print(f"  rate L/R        {rcl:5.1f} / {rcr:5.1f} Hz  (want ~25)     {tag(23 <= rcl <= 27 and 23 <= rcr <= 27)}")
    if c.cL and c.cR:
        m = len(set(c.cL) & set(c.cR))
        pct = 100.0 * m / max(1, min(len(c.cL), len(c.cR)))
        print(f"  L-R exact match {pct:5.1f}%   ({m} pairs)             {tag(pct > 98)}")

    # ---- IMAGE_RAW (what OpenVINS uses) ----
    print("\n[IMAGE_RAW -> OpenVINS]  /camera_*/image_raw")
    rrl = rate(c.rL, dur); rrr = rate(c.rR, dur)
    print(f"  rate L/R        {rrl:5.1f} / {rrr:5.1f} Hz")
    print(f"  L/R balanced    {'yes' if abs(rrl-rrr) < 1.5 else 'NO'}                       {tag(abs(rrl-rrr) < 1.5)}")
    if c.rL and c.rR:
        matched = len(set(c.rL) & set(c.rR))
        mrate = matched / dur
        pct = 100.0 * matched / max(1, min(len(c.rL), len(c.rR)))
        print(f"  MATCHED stereo  {mrate:5.1f} Hz  ({pct:.1f}% paired)        {tag(pct > 90)}")
        print(f"     ^ this is the real stereo rate OpenVINS can use")
        dupL = len(c.rL) - len(set(c.rL)); dupR = len(c.rR) - len(set(c.rR))
        print(f"  duplicate stamps L={dupL} R={dupR}                    {tag(dupL == 0 and dupR == 0)}")
        mono = all(c.rL[i] <= c.rL[i+1] for i in range(len(c.rL)-1))
        print(f"  L stamps monotonic {'yes' if mono else 'NO'}                    {tag(mono)}")

    # ---- IMU vs IMAGE_RAW alignment ----
    print("\n[IMU <-> IMAGE_RAW alignment]")
    if c.rL and c.imu:
        ist = sorted(s[0] for s in c.imu)
        offs = []
        for s in c.rL:
            lo, hi = 0, len(ist) - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if ist[mid] < s: lo = mid + 1
                else: hi = mid
            cand = [ist[max(0, lo-1)], ist[min(len(ist)-1, lo)]]
            offs.append(min(abs(s - x) for x in cand) / 1e6)
        if offs:
            print(f"  mean/max offset {sum(offs)/len(offs):5.2f} / {max(offs):5.2f} ms        {tag(max(offs) < 8)}")
            print(f"     (image_raw stamps should land on real IMU sample times)")

    print("\n" + "=" * 60)
    print("  Reading it: COMPRESSED should be flawless (proves the HW sync).")
    print("  The MATCHED stereo rate under IMAGE_RAW is what actually feeds")
    print("  OpenVINS -- if that's much lower than ~10Hz or paired% is low,")
    print("  decode_node is desyncing the stereo pair and THAT is the bug.")
    print("=" * 60 + "\n")


def main():
    rclpy.init()
    c = Check()
    print(f"Collecting for {DURATION:.0f}s -- keep the rig STILL ...")
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < DURATION:
        rclpy.spin_once(c, timeout_sec=0.1)
    report(c)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
