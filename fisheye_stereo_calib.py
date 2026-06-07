#!/usr/bin/env python3
"""
fisheye_stereo_calib.py
Robust fisheye (equidistant) STEREO calibration for the OV9281 pair.

Captures synchronized stereo pairs of an 8x6 checkerboard from the live ROS2
topics and calibrates with the recipe that AVOIDS OpenCV's fisheye stereo
crashes:
  1) mono-calibrate each camera   (cv2.fisheye.calibrate, NO CALIB_CHECK_COND,
     with per-view reprojection rejection)
  2) stereo with CALIB_FIX_INTRINSIC  (only solve R,T -> well-conditioned)
Produces self-consistent cam0/cam1 fisheye intrinsics + baseline and writes
openvins_config/kalibr_imucam_chain_fisheye_new.yaml.

Run with system OpenCV (GTK GUI) + system numpy:
  PYTHONNOUSERSITE=1 python3 ~/ros2_ws/src/igvc_camstuff/fisheye_stereo_calib.py

Keys in the preview window:
  SPACE = capture pair (only when BOTH cameras see the full board)
  u     = undo last capture
  c     = calibrate with what you've got
  q     = quit
"""
import os
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
import message_filters

# ---- board / output config ----
BOARD = (8, 6)            # inner corners
SQUARE = 0.020            # meters (your measured square)
W, H = 1280, 800
OUT_DIR = os.path.expanduser('~/ros2_ws/src/igvc_camstuff/openvins_config')
OUT_YAML = os.path.join(OUT_DIR, 'kalibr_imucam_chain_fisheye_new.yaml')
OUT_TXT  = os.path.join(OUT_DIR, 'fisheye_calib_result.txt')

# cam0 (left) IMU->cam transform from your gravity test. Only edit if you
# refined it; the intrinsics + baseline are what this tool actually fixes.
R_C0_I = np.array([[0, -1, 0],
                   [0,  0, -1],
                   [1,  0,  0]], float)
t_C0_I = np.array([0.035, 0.0, 0.0])

CORNER_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                cv2.CALIB_CB_NORMALIZE_IMAGE |
                cv2.CALIB_CB_FAST_CHECK)
SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
CALIB_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7)


def objp_template():
    o = np.zeros((1, BOARD[0] * BOARD[1], 3), np.float64)
    o[0, :, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE
    return o


def find_board(gray):
    ok, c = cv2.findChessboardCorners(gray, BOARD, CORNER_FLAGS)
    if not ok:
        return None
    c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), SUBPIX)
    return c  # (N,1,2) float32


class Collector(Node):
    def __init__(self):
        super().__init__('fisheye_stereo_calib')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        subL = message_filters.Subscriber(self, Image, '/camera_left/image_raw',  qos_profile=qos)
        subR = message_filters.Subscriber(self, Image, '/camera_right/image_raw', qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer([subL, subR], 10, 0.02)
        self.sync.registerCallback(self.cb)
        self.lock = threading.Lock()
        self.latest = None  # (grayL, grayR)
        self.get_logger().info('waiting for synced /camera_*/image_raw ...')

    def cb(self, ml, mr):
        gl = np.frombuffer(ml.data, np.uint8).reshape(ml.height, ml.width)
        gr = np.frombuffer(mr.data, np.uint8).reshape(mr.height, mr.width)
        with self.lock:
            self.latest = (gl.copy(), gr.copy())

    def get(self):
        with self.lock:
            return self.latest


def mono_calibrate(objpoints, imgpoints, size):
    """Mono fisheye calibrate with per-view reprojection rejection.
    Returns rms, K, D, kept_indices."""
    idx = list(range(len(objpoints)))
    for _ in range(5):
        obj = [objpoints[i] for i in idx]
        img = [imgpoints[i] for i in idx]
        K = np.zeros((3, 3)); D = np.zeros((4, 1))
        rv = [np.zeros((1, 1, 3)) for _ in obj]
        tv = [np.zeros((1, 1, 3)) for _ in obj]
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        try:
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                obj, img, size, K, D, rv, tv, flags, CALIB_CRIT)
        except cv2.error as e:
            # drop the last view and retry if a degenerate pose slipped in
            if len(idx) > 8:
                idx = idx[:-1]
                continue
            raise
        # per-view error
        errs = []
        for j in range(len(obj)):
            proj, _ = cv2.fisheye.projectPoints(obj[j], rvecs[j], tvecs[j], K, D)
            e = np.linalg.norm(img[j].reshape(-1, 2) - proj.reshape(-1, 2), axis=1).mean()
            errs.append(e)
        errs = np.array(errs)
        bad = np.where(errs > max(1.0, errs.mean() + 2 * errs.std()))[0]
        if len(bad) == 0 or len(idx) - len(bad) < 10:
            return rms, K, D, idx
        idx = [idx[j] for j in range(len(idx)) if j not in set(bad)]
    return rms, K, D, idx


def calibrate(objpoints, imgL, imgR, size):
    rms0, K0, D0, keptL = mono_calibrate(objpoints, imgL, size)
    rms1, K1, D1, keptR = mono_calibrate(objpoints, imgR, size)
    sidx = sorted(set(keptL) & set(keptR))
    if len(sidx) < 8:
        sidx = list(range(len(objpoints)))
    obj = [objpoints[i] for i in sidx]
    iL = [imgL[i] for i in sidx]
    iR = [imgR[i] for i in sidx]
    R = np.zeros((3, 3)); T = np.zeros((3, 1))
    rms, _, _, _, _, R, T = cv2.fisheye.stereoCalibrate(
        obj, iL, iR, K0, D0, K1, D1, size, R, T,
        cv2.fisheye.CALIB_FIX_INTRINSIC, CALIB_CRIT)
    return dict(rms0=rms0, rms1=rms1, stereo_rms=rms,
                K0=K0, D0=D0, K1=K1, D1=D1, R=R, T=T,
                n_mono_L=len(keptL), n_mono_R=len(keptR), n_stereo=len(sidx))


def mat_rows(M):
    return "\n".join("      - [%s]" % ", ".join("%.8f" % v for v in row) for row in M)


def write_outputs(res):
    K0, D0, K1, D1, R, T = (res['K0'], res['D0'], res['K1'], res['D1'], res['R'], res['T'])
    T_C0_I = np.eye(4); T_C0_I[:3, :3] = R_C0_I; T_C0_I[:3, 3] = t_C0_I
    T_10 = np.eye(4); T_10[:3, :3] = R; T_10[:3, 3] = T.ravel()   # cam0 -> cam1
    T_C1_I = T_10 @ T_C0_I
    fx0, fy0, cx0, cy0 = K0[0, 0], K0[1, 1], K0[0, 2], K0[1, 2]
    fx1, fy1, cx1, cy1 = K1[0, 0], K1[1, 1], K1[0, 2], K1[1, 2]
    d0 = D0.ravel(); d1 = D1.ravel()
    baseline = np.linalg.norm(T) * 1000.0

    yaml = f"""%YAML:1.0
cam0:
   T_cam_imu:
{mat_rows(T_C0_I)}
   cam_overlaps: [1]
   camera_model: pinhole
   distortion_coeffs: [{d0[0]:.6f}, {d0[1]:.6f}, {d0[2]:.6f}, {d0[3]:.6f}]
   distortion_model: equidistant
   intrinsics: [{fx0:.4f}, {fy0:.4f}, {cx0:.4f}, {cy0:.4f}]
   resolution: [{W}, {H}]
   rostopic: /camera_left/image_raw
cam1:
   T_cam_imu:
{mat_rows(T_C1_I)}
   T_cn_cnm1:
{mat_rows(T_10)}
   cam_overlaps: [0]
   camera_model: pinhole
   distortion_coeffs: [{d1[0]:.6f}, {d1[1]:.6f}, {d1[2]:.6f}, {d1[3]:.6f}]
   distortion_model: equidistant
   intrinsics: [{fx1:.4f}, {fy1:.4f}, {cx1:.4f}, {cy1:.4f}]
   resolution: [{W}, {H}]
   rostopic: /camera_right/image_raw
"""
    with open(OUT_YAML, 'w') as f:
        f.write(yaml)

    txt = (f"=== FISHEYE STEREO CALIBRATION RESULT ===\n"
           f"mono RMS    L={res['rms0']:.4f}px  R={res['rms1']:.4f}px\n"
           f"stereo RMS  {res['stereo_rms']:.4f}px\n"
           f"views used  monoL={res['n_mono_L']} monoR={res['n_mono_R']} stereo={res['n_stereo']}\n"
           f"baseline    {baseline:.2f} mm\n\n"
           f"cam0 intrinsics [fx,fy,cx,cy]: {fx0:.4f}, {fy0:.4f}, {cx0:.4f}, {cy0:.4f}\n"
           f"cam0 distortion [k1..k4]:      {d0[0]:.6f}, {d0[1]:.6f}, {d0[2]:.6f}, {d0[3]:.6f}\n"
           f"cam1 intrinsics [fx,fy,cx,cy]: {fx1:.4f}, {fy1:.4f}, {cx1:.4f}, {cy1:.4f}\n"
           f"cam1 distortion [k1..k4]:      {d1[0]:.6f}, {d1[1]:.6f}, {d1[2]:.6f}, {d1[3]:.6f}\n"
           f"stereo R (cam0->cam1):\n{R}\n"
           f"stereo T (m): {T.ravel()}\n")
    with open(OUT_TXT, 'w') as f:
        f.write(txt)
    return txt, baseline


def main():
    rclpy.init()
    node = Collector()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    objt = objp_template()
    objpoints, imgL, imgR = [], [], []
    last_centroid = None
    cells = set()

    cv2.namedWindow('fisheye stereo calib', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('fisheye stereo calib', 1280, 420)

    while rclpy.ok():
        pair = node.get()
        if pair is None:
            if cv2.waitKey(50) & 0xFF == ord('q'):
                break
            continue
        gl, gr = pair
        cL = find_board(gl); cR = find_board(gr)
        visL = cv2.cvtColor(gl, cv2.COLOR_GRAY2BGR)
        visR = cv2.cvtColor(gr, cv2.COLOR_GRAY2BGR)
        if cL is not None:
            cv2.drawChessboardCorners(visL, BOARD, cL, True)
        if cR is not None:
            cv2.drawChessboardCorners(visR, BOARD, cR, True)
        both = cL is not None and cR is not None
        disp = np.hstack([visL, visR])
        disp = cv2.resize(disp, (1280, 400))
        color = (0, 255, 0) if both else (0, 0, 255)
        cv2.putText(disp, f"captured={len(objpoints)}  cells={len(cells)}/9  "
                          f"board={'BOTH' if both else 'no'}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(disp, "SPACE=capture  u=undo  c=calibrate  q=quit",
                    (10, 385), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
        cv2.imshow('fisheye stereo calib', disp)

        k = cv2.waitKey(30) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('u') and objpoints:
            objpoints.pop(); imgL.pop(); imgR.pop()
            node.get_logger().info(f'undo -> {len(objpoints)} pairs')
        elif k == ord(' ') and both:
            cen = cL.reshape(-1, 2).mean(0)
            if last_centroid is not None and np.linalg.norm(cen - last_centroid) < 30:
                node.get_logger().info('too similar to last capture, skipped')
            else:
                objpoints.append(objt.copy())
                imgL.append(cL.reshape(1, -1, 2).astype(np.float64))
                imgR.append(cR.reshape(1, -1, 2).astype(np.float64))
                last_centroid = cen
                cells.add((int(cen[0] // (W / 3)), int(cen[1] // (H / 3))))
                node.get_logger().info(f'captured pair {len(objpoints)}')
        elif k == ord('c'):
            if len(objpoints) < 12:
                node.get_logger().warn(f'need >=12 pairs, have {len(objpoints)}')
                continue
            node.get_logger().info('calibrating ...')
            try:
                res = calibrate(objpoints, imgL, imgR, (W, H))
            except cv2.error as e:
                node.get_logger().error(f'calibration failed: {e}')
                continue
            txt, baseline = write_outputs(res)
            print('\n' + txt)
            print(f'wrote {OUT_YAML}')
            print(f'wrote {OUT_TXT}')
            node.get_logger().info('done - press q to quit')

    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
