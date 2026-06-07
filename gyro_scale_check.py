#!/usr/bin/env python3
"""
gyro_scale_check.py
The extrinsic check confirmed the gyro/camera rotation AXES agree (R is correct).
This checks the one thing axis/cosine tests are blind to: the gyro MAGNITUDE.
For each frame pair it measures the camera rotation angle (truth) and the
gyro-integrated angle over the same interval, and reports their ratio.

  ratio ~ 1.0            -> gyro scale CORRECT (not the bug; look at time offset next)
  ratio clearly != 1.0   -> gyro is mis-scaled by that factor (firmware sensitivity/units bug);
                            divide the published gyro by the ratio to fix.

Run with sensors up:
  PYTHONNOUSERSITE=1 python3 ~/ros2_ws/src/igvc_camstuff/gyro_scale_check.py
Then ROTATE in place through all axes smoothly ~30 s, pointed at a textured
scene, avoiding translation.
"""
import math, collections
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, Image

K = np.array([[719.4519, 0, 683.4017], [0, 721.9364, 421.3023], [0, 0, 1]], float)
D = np.array([-0.080236, 0.142701, -0.276103, 0.177827], float)


def expm(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * (Kx @ Kx)


def angle_of(R):
    c = (np.trace(R) - 1) / 2
    return math.acos(max(-1.0, min(1.0, c)))


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


class ScaleCheck(Node):
    def __init__(self):
        super().__init__('gyro_scale_check')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos)
        self.create_subscription(Image, '/camera_left/image_raw', self.img_cb, qos)
        self.imu_buf = collections.deque(maxlen=4000)
        self.prev_gray = None
        self.prev_t = None
        self.thc = []
        self.thi = []
        self.create_timer(3.0, self.report)
        self.get_logger().info('gyro scale check -- ROTATE in place through all axes ~30s; avoid translating')

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_buf.append((t, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

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

    def img_cb(self, msg):
        if msg.encoding not in ('mono8', '8UC1'):
            return
        gray = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width].copy()
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_gray is not None and len(self.imu_buf) > 5:
            p0 = cv2.goodFeaturesToTrack(self.prev_gray, maxCorners=250, qualityLevel=0.01, minDistance=10)
            if p0 is not None and len(p0) >= 20:
                p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None)
                if p1 is not None:
                    st = st.reshape(-1).astype(bool)
                    a = p0.reshape(-1, 2)[st]
                    b = p1.reshape(-1, 2)[st]
                    if len(a) >= 20:
                        ba = undistort_bearings(a)
                        bb = undistort_bearings(b)
                        M = kabsch(ba, bb)
                        res = np.arccos(np.clip(np.sum((ba @ M.T) * bb, axis=1), -1, 1))
                        inl = res < math.radians(1.0)
                        if inl.sum() >= 15:
                            M = kabsch(ba[inl], bb[inl])
                            thc = angle_of(M)
                            thi = angle_of(self.integrate_imu(self.prev_t, t))
                            if math.degrees(thc) > 1.0 and math.degrees(thi) > 0.3:
                                self.thc.append(thc)
                                self.thi.append(thi)
        self.prev_gray = gray
        self.prev_t = t

    def report(self):
        n = len(self.thc)
        if n < 40:
            self.get_logger().info(f'collecting... {n} rotation samples (rotate through all axes)')
            return
        thc = np.array(self.thc)
        thi = np.array(self.thi)
        ratio = thi / thc
        med = float(np.median(ratio))
        slope = float(np.sum(thc * thi) / np.sum(thc * thc))
        iqr = float(np.percentile(ratio, 75) - np.percentile(ratio, 25))
        self.get_logger().info(f'n={n} | gyro/camera angle ratio: median={med:.3f} ls_slope={slope:.3f} iqr={iqr:.3f}')
        if iqr > 0.4:
            self.get_logger().info('=> ratios noisy (translation/weak texture); rotate slower in place, more texture.')
        elif 0.9 <= med <= 1.1:
            self.get_logger().info('=> gyro scale CORRECT. Not the bug -> next suspect is the camera-IMU TIME OFFSET.')
        else:
            self.get_logger().info(f'=> GYRO MIS-SCALED by ~{med:.2f}x. Divide published gyro by {med:.3f} (firmware sensitivity/units bug). That is the cause.')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScaleCheck())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
