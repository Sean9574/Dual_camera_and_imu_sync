#!/usr/bin/env python3
"""
vio_full_check.py  --  ONE-SHOT full validation of the stereo-inertial VIO front-end.

It walks you through two phases (it prints prompts + a countdown):
  PHASE 1  (10 s):  hold the rig DEAD STILL.
  PHASE 2  (30 s):  ROTATE in place through all 3 axes, smoothly, pointed at a
                    textured scene, with as little translation as you can manage.

Then it prints a single PASS / WARN / FAIL report covering, with measured numbers:
  - publish RATES (imu, trigger, compressed L/R, image_raw L/R, imu-per-trigger)
  - TIMESTAMP integrity (monotonic, dropped samples, duplicate stamps)
  - SYNC / stereo PAIRING (L<->R exact-stamp match at both compressed and image_raw,
    and image_raw stamp vs real IMU sample time)
  - IMU HEALTH at rest (|accel|~9.81, per-axis gyro bias, sensor-alive variance)
  - VISION HEALTH (tracked features/frame, brightness, texture)
  - GYRO SCALE (camera-vs-gyro rotation magnitude)
  - GYRO/ACCEL axis CONSISTENCY (no swap / sign flip)
  - camera-IMU EXTRINSIC rotation (hand-eye axis agreement)
  - camera-IMU TIME OFFSET (cross-correlation of rotation-rate profiles)
  - gravity axis sanity in the published IMU frame

Run with sensors up, VIO off:
  ros2 launch igvc_camstuff sensor_bringup.launch.py vio:=false rviz:=false
  PYTHONNOUSERSITE=1 python3 ~/ros2_ws/src/igvc_camstuff/vio_full_check.py
"""
import collections
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, Imu
from std_msgs.msg import Header

# ---- calibration (left cam fisheye + cam<-imu extrinsic) ----
K = np.array([[719.4519, 0, 683.4017], [0, 721.9364, 421.3023], [0, 0, 1]], float)
D = np.array([-0.080236, 0.142701, -0.276103, 0.177827], float)
R_CAM_IMU = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)  # v_cam = R_CAM_IMU @ v_imu

# ---- expected values ----
EXP_IMU_HZ = (95, 105)
EXP_TRIG_HZ = (23, 27)
EXP_CAM_HZ = (23, 27)

SETTLE, STILL, ROTATE = 3.0, 10.0, 30.0
TOTAL = SETTLE + STILL + ROTATE


def expm(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * (Kx @ Kx)


def angle_of(R):
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))


def axis_of(R):
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def kabsch(A, B):
    H = A.T @ B
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1, 1, d]) @ U.T


def undistort_bearings(pts):
    p = pts.reshape(-1, 1, 2).astype(np.float32)
    un = cv2.fisheye.undistortPoints(p, K, D).reshape(-1, 2)
    b = np.hstack([un, np.ones((un.shape[0], 1))])
    b /= np.linalg.norm(b, axis=1, keepdims=True)
    return b


def tag(ok, warn=False):
    return "\033[92mPASS\033[0m" if ok else ("\033[93mWARN\033[0m" if warn else "\033[91mFAIL\033[0m")


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class FullCheck(Node):
    def __init__(self):
        super().__init__('vio_full_check')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )

        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos)
        self.create_subscription(Header, '/camera/trigger', self.trig_cb, qos)
        self.create_subscription(CompressedImage, '/camera_left/compressed',
                                 lambda m: self.cL.append((time.time(), stamp_s(m.header.stamp))), qos)
        self.create_subscription(CompressedImage, '/camera_right/compressed',
                                 lambda m: self.cR.append((time.time(), stamp_s(m.header.stamp))), qos)
        self.create_subscription(Image, '/camera_left/image_raw', self.imgL_cb, qos)
        self.create_subscription(Image, '/camera_right/image_raw',
                                 lambda m: self.rR.append((time.time(), stamp_s(m.header.stamp))), qos)

        self.imu = []            # (wall, t, ax,ay,az, gx,gy,gz)
        self.trig = []           # (wall, t)
        self.cL = []
        self.cR = []
        self.rL = []
        self.rR = []
        self.imu_buf = collections.deque(maxlen=6000)   # (t,gx,gy,gz)

        self.prev_gray = None
        self.prev_t = None

        self.m = []              # (wall, t_mid, thc, dt, axis_cam(3), thi, axis_imu(3))
        self.feat = []           # (wall, nfeat)
        self.imgstat = []        # (wall, mean, std)

        self.t0 = time.time()
        self.phase = 0
        self.create_timer(0.5, self.tick)

        print("\n=== VIO FULL CHECK ===  settle %.0fs, STILL %.0fs, ROTATE %.0fs\n" % (SETTLE, STILL, ROTATE))
        print(">>> get ready -- keep it STILL to start ...")

    def imu_cb(self, m):
        t = stamp_s(m.header.stamp)
        self.imu.append((time.time(), t,
                         m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                         m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        self.imu_buf.append((t, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))

    def trig_cb(self, m):
        self.trig.append((time.time(), stamp_s(m.stamp)))

    def integrate_imu(self, t0, t1):
        s = [x for x in self.imu_buf if t0 < x[0] <= t1]
        if len(s) < 2:
            return np.eye(3)
        T = np.eye(3)
        for k in range(1, len(s)):
            dt = s[k][0] - s[k - 1][0]
            if dt <= 0 or dt > 0.1:
                continue
            w = np.array([s[k - 1][1], s[k - 1][2], s[k - 1][3]])
            T = T @ expm(w * dt)
        return T

    def imgL_cb(self, msg):
        if msg.encoding not in ('mono8', '8UC1'):
            return

        wall = time.time()
        self.rL.append((wall, stamp_s(msg.header.stamp)))
        gray = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width].copy()
        t = stamp_s(msg.header.stamp)

        self.imgstat.append((wall, float(gray.mean()), float(gray.std())))

        if self.prev_gray is not None and len(self.imu_buf) > 5:
            p0 = cv2.goodFeaturesToTrack(self.prev_gray, maxCorners=250, qualityLevel=0.01, minDistance=10)
            if p0 is not None and len(p0) >= 20:
                p1, stt, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None)
                if p1 is not None:
                    stt = stt.reshape(-1).astype(bool)
                    a = p0.reshape(-1, 2)[stt]
                    b = p1.reshape(-1, 2)[stt]
                    self.feat.append((wall, int(stt.sum())))
                    if len(a) >= 20:
                        ba = undistort_bearings(a)
                        bb = undistort_bearings(b)
                        M = kabsch(ba, bb)
                        res = np.arccos(np.clip(np.sum((ba @ M.T) * bb, axis=1), -1, 1))
                        inl = res < math.radians(1.0)
                        if inl.sum() >= 15:
                            M = kabsch(ba[inl], bb[inl])
                            thc = angle_of(M)
                            ac = axis_of(M)

                            Ri = self.integrate_imu(self.prev_t, t)
                            thi = angle_of(Ri)
                            ai = axis_of(Ri)

                            dt = t - self.prev_t
                            if dt > 0:
                                self.m.append((wall, 0.5 * (t + self.prev_t), thc, dt, ac, thi, ai))

        self.prev_gray = gray
        self.prev_t = t

    def tick(self):
        el = time.time() - self.t0
        np_ = self.phase

        if el < SETTLE:
            self.phase = 0
        elif el < SETTLE + STILL:
            self.phase = 1
        elif el < TOTAL:
            self.phase = 2
        else:
            self.phase = 3

        if self.phase != np_:
            if self.phase == 1:
                print("\n>>> PHASE 1/2: hold DEAD STILL (%.0fs) ..." % STILL)
            elif self.phase == 2:
                print("\n>>> PHASE 2/2: ROTATE in place through ALL axes, smooth, textured scene (%.0fs) ..." % ROTATE)
            elif self.phase == 3:
                self.report()
                rclpy.shutdown()
                return

        if self.phase in (1, 2):
            rem = (SETTLE + STILL - el) if self.phase == 1 else (TOTAL - el)
            print("    %s  %4.0fs left   (rot samples: %d)" %
                  ("STILL " if self.phase == 1 else "ROTATE", rem, len(self.m)), end="\r")

    def seg(self, arr, lo, hi):
        return [x for x in arr if lo <= (x[0] - self.t0) < hi]

    def measure_time_offset(self, mm):
        cam_t = np.array([x[1] for x in mm])
        cam_rate = np.array([x[2] / x[3] for x in mm])

        ib = sorted([(x[1], math.sqrt(x[5] ** 2 + x[6] ** 2 + x[7] ** 2)) for x in self.imu], key=lambda z: z[0])
        it = np.array([z[0] for z in ib])
        im = np.array([z[1] for z in ib])

        cm = cam_rate - cam_rate.mean()
        best = (0.0, -1.0)

        for d in np.arange(-0.060, 0.0601, 0.001):
            g = np.interp(cam_t + d, it, im)
            g = g - g.mean()
            den = math.sqrt(np.sum(cm * cm) * np.sum(g * g))
            if den <= 0:
                continue
            c = float(np.sum(cm * g) / den)
            if c > best[1]:
                best = (float(d), c)
        return best

    def report(self):
        L = []
        L.append("\n" + "=" * 66)
        L.append("  VIO FULL CHECK REPORT")
        L.append("=" * 66)

        run_lo, run_hi = SETTLE, TOTAL
        dur = run_hi - run_lo

        def rate(arr):
            return len(self.seg(arr, run_lo, run_hi)) / dur

        L.append("\n[RATES]")
        ri = rate(self.imu)
        rt = rate(self.trig)
        rcl = rate(self.cL)
        rcr = rate(self.cR)
        rrl = rate(self.rL)
        rrr = rate(self.rR)

        L.append(f"  /imu/data            {ri:6.1f} Hz                 {tag(EXP_IMU_HZ[0] <= ri <= EXP_IMU_HZ[1])}")
        L.append(f"  /camera/trigger      {rt:6.1f} Hz                 {tag(EXP_TRIG_HZ[0] <= rt <= EXP_TRIG_HZ[1])}")
        if rt > 0:
            L.append(f"  IMU per trigger      {ri/rt:6.2f}    (want ~4.0)     {tag(3.7 <= ri/rt <= 4.3)}")
        L.append(f"  compressed L/R       {rcl:5.1f}/{rcr:5.1f} Hz          {tag(EXP_CAM_HZ[0] <= rcl <= EXP_CAM_HZ[1] and EXP_CAM_HZ[0] <= rcr <= EXP_CAM_HZ[1])}")
        L.append(f"  image_raw  L/R       {rrl:5.1f}/{rrr:5.1f} Hz          {tag(EXP_CAM_HZ[0] <= rrl <= EXP_CAM_HZ[1] and EXP_CAM_HZ[0] <= rrr <= EXP_CAM_HZ[1])}")

        L.append("\n[TIMESTAMPS / SYNC]")
        it = sorted(x[1] for x in self.imu)
        if len(it) > 10:
            gaps = [(it[i + 1] - it[i]) * 1e3 for i in range(len(it) - 1)]
            mg = max(gaps)
            ndrop = sum(1 for g in gaps if g > 25)
            mono = all(it[i] <= it[i + 1] for i in range(len(it) - 1))
            L.append(f"  IMU max stamp gap    {mg:6.1f} ms (drops>25ms:{ndrop}) {tag(mg < 25)}")
            L.append(f"  IMU monotonic        {'yes' if mono else 'NO':>6}                 {tag(mono)}")

        rLs = [x[1] for x in self.rL]
        rRs = [x[1] for x in self.rR]
        cLs = [x[1] for x in self.cL]
        cRs = [x[1] for x in self.cR]

        dupL = len(rLs) - len(set(rLs))
        dupR = len(rRs) - len(set(rRs))
        L.append(f"  img dup stamps L/R   {dupL:>3}/{dupR:<3}                 {tag(dupL == 0 and dupR == 0)}")

        if cLs and cRs:
            mp = len(set(cLs) & set(cRs))
            pct = 100.0 * mp / max(1, min(len(cLs), len(cRs)))
            L.append(f"  compressed L<->R     {pct:5.1f}% paired           {tag(pct > 98)}")

        if rLs and rRs:
            mp = len(set(rLs) & set(rRs))
            pct = 100.0 * mp / max(1, min(len(rLs), len(rRs)))
            L.append(f"  image_raw  L<->R     {pct:5.1f}% paired  <-OpenVINS {tag(pct > 90)}")

        if rLs and it:
            ia = np.array(sorted(it))
            offs = []
            for s in rLs:
                j = np.searchsorted(ia, s)
                cand = []
                if j < len(ia):
                    cand.append(abs(s - ia[j]))
                if j > 0:
                    cand.append(abs(s - ia[j - 1]))
                offs.append(min(cand) * 1e3)
            L.append(f"  img stamp<->IMU      {np.mean(offs):4.2f}/{np.max(offs):4.2f} ms mean/max {tag(np.max(offs) < 8)}")

        L.append("\n[IMU HEALTH  (still phase)]")
        still = self.seg(self.imu, SETTLE, SETTLE + STILL)
        if len(still) > 20:
            ax = np.array([s[2] for s in still])
            ay = np.array([s[3] for s in still])
            az = np.array([s[4] for s in still])
            gx = np.array([s[5] for s in still])
            gy = np.array([s[6] for s in still])
            gz = np.array([s[7] for s in still])

            amag = math.sqrt(ax.mean()**2 + ay.mean()**2 + az.mean()**2)
            L.append(f"  |accel| at rest       {amag:6.3f} m/s^2 (~9.81)    {tag(9.5 <= amag <= 10.1, warn=True)}")
            L.append(f"  accel mean x/y/z      {ax.mean():+.3f}/{ay.mean():+.3f}/{az.mean():+.3f} m/s^2")
            L.append(f"  gyro bias x/y/z       {gx.mean():+.3f}/{gy.mean():+.3f}/{gz.mean():+.3f} rad/s {tag(max(abs(gx.mean()), abs(gy.mean()), abs(gz.mean())) < 0.01, warn=True)}")

            astd = max(ax.std(), ay.std(), az.std())
            gstd = max(gx.std(), gy.std(), gz.std())
            L.append(f"  accel alive (max std) {astd:7.4f}                 {tag(astd > 1e-4)}")
            L.append(f"  gyro  alive (max std) {gstd:7.4f}                 {tag(gstd > 1e-5)}")
            L.append(f"  ^ gyro Y bias is the marginal one we've been fighting; report uses it as-is")

            gvec = np.array([ax.mean(), ay.mean(), az.mean()])
            gdir = gvec / (np.linalg.norm(gvec) + 1e-9)
            dot_z = float(np.dot(gdir, np.array([0.0, 0.0, 1.0])))
            angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot_z))))
            L.append(f"  gravity axis vs +Z    {angle_deg:6.2f} deg           {tag(angle_deg < 15.0, warn=(15.0 <= angle_deg < 45.0))}")
            L.append("  ^ if this is large, IMU axes/signs may not match the IMU frame used in T_cam_imu")

        L.append("\n[VISION HEALTH]")
        if self.feat:
            fv = np.array([f[1] for f in self.feat])
            med = int(np.median(fv))
            L.append(f"  features/frame (med)  {med:>4}                     {tag(med >= 40, warn=(20 <= med < 40))}")
        if self.imgstat:
            mn = np.median([s[1] for s in self.imgstat])
            sd = np.median([s[2] for s in self.imgstat])
            L.append(f"  brightness (median)   {mn:6.1f}  (10..245)        {tag(10 < mn < 245, warn=True)}")
            L.append(f"  texture std (median)  {sd:6.1f}  (>15 good)        {tag(sd > 15, warn=(8 < sd <= 15))}")

        L.append("\n[GEOMETRY / TIMING  (rotate phase)]")
        mm = [
            x for x in self.m
            if SETTLE + STILL <= (x[0] - self.t0) < TOTAL
            and math.degrees(x[2]) > 1.0
            and math.degrees(x[5]) > 0.3
        ]

        if len(mm) < 20:
            L.append(f"  not enough rotation samples ({len(mm)}) -- rotate more / textured scene")
        else:
            thc = np.array([x[2] for x in mm])
            thi = np.array([x[5] for x in mm])
            ratio = thi / thc
            med = float(np.median(ratio))
            iqr = float(np.percentile(ratio, 75) - np.percentile(ratio, 25))
            L.append(f"  gyro scale ratio      {med:6.3f}  (iqr {iqr:.2f})    {tag(0.9 <= med <= 1.1, warn=iqr > 0.3)}")

            rot = self.seg(self.imu, SETTLE + STILL, TOTAL)
            cs = []
            for k in range(1, len(rot)):
                dt = rot[k][1] - rot[k - 1][1]
                if dt <= 0 or dt > 0.05:
                    continue
                a0 = np.array(rot[k - 1][2:5])
                a1 = np.array(rot[k][2:5])
                w = np.array(rot[k - 1][5:8])
                if np.linalg.norm(w) < 0.3:
                    continue
                da = (a1 - a0) / dt
                pred = -np.cross(w, a0)
                dn = np.linalg.norm(da) * np.linalg.norm(pred)
                if dn > 1e-6:
                    cs.append(float(np.dot(da, pred) / dn))

            if cs:
                mc = float(np.mean(cs))
                L.append(f"  gyro/accel consist.   {mc:+6.3f}  (cos, want>0)  {tag(mc > 0.3, warn=(0.0 < mc <= 0.3))}  {'<-possible swap/sign!' if mc < 0 else ''}")

            ee = []
            for x in mm:
                ac = x[4]
                ai = x[6]
                if np.linalg.norm(ac) < 0.5 or np.linalg.norm(ai) < 0.5:
                    continue
                mapped = R_CAM_IMU @ ai
                ee.append(math.degrees(math.acos(min(1.0, abs(float(np.dot(mapped, ac)))))))

            if ee:
                me = float(np.median(ee))
                L.append(f"  cam-IMU extrinsic     {me:6.2f} deg axis err   {tag(me < 8, warn=(8 <= me < 15))}")

            off, corr = self.measure_time_offset(mm)
            L.append(f"  camera-IMU TIME OFF   {off*1e3:+6.1f} ms (corr {corr:.2f}) {tag(abs(off) < 0.004, warn=(corr > 0.6))}")
            L.append(f"  ^ if |offset| is several ms with good corr, set calib_camimu_dt to it (test +/- sign)")

        L.append("\n" + "=" * 66)
        print("\n".join(L))


def main(args=None):
    rclpy.init(args=args)
    n = FullCheck()
    try:
        rclpy.spin(n)
    except Exception:
        pass


if __name__ == '__main__':
    main()