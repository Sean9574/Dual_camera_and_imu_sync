#!/usr/bin/env python3
"""
extrinsic_check.py
Measures the camera<-IMU rotation R directly and compares it to the hand-set R
in your config. This is the rotational half of a hand-eye calibration; pure
rotation makes it target-free.

Gyro gives the rotation axis in the IMU frame; the image rotation gives the same
rotation's axis in the camera frame. R maps one to the other. We collect many
axis pairs while you rotate, solve R (Kabsch), and report how far it is from the
hand-set R.

  discrepancy < ~8 deg   -> extrinsic basically CORRECT; R is not the bug (Kalibr won't help)
  discrepancy > ~15 deg  -> extrinsic WRONG; that's the cause. Use the printed R or run Kalibr.

Run with sensors up (VIO off is fine):
  PYTHONNOUSERSITE=1 python3 ~/ros2_ws/src/igvc_camstuff/extrinsic_check.py
Then ROTATE the rig in place through ALL three axes (yaw, pitch, roll) smoothly
for ~30 s. Avoid translating.
"""
import math, collections
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, Image

# LEFT camera (cam0) fisheye intrinsics from your calibration
K = np.array([[719.4519, 0, 683.4017],
              [0, 721.9364, 421.3023],
              [0, 0, 1]], float)
D = np.array([-0.080236, 0.142701, -0.276103, 0.177827], float)
# hand-set camera<-imu rotation (rotation part of T_cam0_imu) from config
R_HANDSET = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)


def expm(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * (Kx @ Kx)


def axis_angle(R):
    c = (np.trace(R) - 1) / 2
    c = max(-1.0, min(1.0, c))
    th = math.acos(c)
    if th < 1e-9:
        return np.array([1.0, 0, 0]), 0.0
    a = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(th))
    n = np.linalg.norm(a)
    if n < 1e-9:
        return np.array([1.0, 0, 0]), 0.0
    return a / n, th


def kabsch(A, B):
    H = A.T @ B
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1, 1, d]) @ U.T


def ang_between(Ra, Rb):
    _, th = axis_angle(Ra.T @ Rb)
    return math.degrees(th)


def undistort_bearings(pts):
    p = pts.reshape(-1, 1, 2).astype(np.float32)
    un = cv2.fisheye.undistortPoints(p, K, D).reshape(-1, 2)
    b = np.hstack([un, np.ones((un.shape[0], 1))])
    b /= np.linalg.norm(b, axis=1, keepdims=True)
    return b


class ExtrinsicCheck(Node):
    def __init__(self):
        super().__init__('extrinsic_check')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos)
        self.create_subscription(Image, '/camera_left/image_raw', self.img_cb, qos)
        self.imu_buf = collections.deque(maxlen=4000)
        self.prev_gray = None
        self.prev_t = None
        self.axes_imu = []
        self.axes_cam = []
        self.create_timer(3.0, self.report)
        self.get_logger().info('extrinsic check -- ROTATE in place through ALL axes (yaw/pitch/roll) ~30s; avoid translating')

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
            T = T @ expm(-w * dt)   # bearing-transport = integrate(-omega)
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
                            ac, thc = axis_angle(M)
                            Timu = self.integrate_imu(self.prev_t, t)
                            ai, thi = axis_angle(Timu)
                            if math.degrees(thc) > 1.0 and math.degrees(thi) > 1.0:
                                self.axes_cam.append(ac)
                                self.axes_imu.append(ai)
        self.prev_gray = gray
        self.prev_t = t

    def report(self):
        n = len(self.axes_imu)
        if n < 40:
            self.get_logger().info(f'collecting... {n} rotation samples (rotate through all axes)')
            return
        A = np.array(self.axes_imu)
        B = np.array(self.axes_cam)
        spread = float(np.linalg.eigvalsh(A.T @ A / len(A))[0])
        R_est = kabsch(A, B)
        disc = ang_between(R_HANDSET, R_est)
        fit = math.degrees(np.mean(np.arccos(np.clip(np.sum((A @ R_est.T) * B, axis=1), -1, 1))))
        self.get_logger().info(f'n={n} spread={spread:.3f} fit_resid={fit:.1f}deg || discrepancy(handset vs measured)={disc:.1f} deg')
        if spread < 0.05:
            self.get_logger().info('=> rotation axes too collinear; rotate about ALL 3 axes (yaw AND pitch AND roll).')
            return
        if fit > 8:
            self.get_logger().info('=> fit noisy (translation or weak texture); rotate slower in place, point at a textured scene.')
        if disc < 8:
            self.get_logger().info('=> extrinsic basically CORRECT. R is NOT the bug -> Kalibr will not help; look elsewhere.')
        elif disc < 15:
            self.get_logger().info('=> extrinsic MODERATELY off -> plausible cause; worth fixing.')
        else:
            self.get_logger().info('=> extrinsic BADLY WRONG -> this is the bug. Fix R (use measured below or run Kalibr).')
        self.get_logger().info('measured R_cam_imu =\n' + np.array2string(np.round(R_est, 4)))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ExtrinsicCheck())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
