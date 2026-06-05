#!/usr/bin/env python3
"""
fisheye_calib.py -- OpenCV fisheye (equidistant) stereo calibration with a live
preview + coverage gating (like ROS cameracalibrator's X/Y/Size/Skew bars).
Only accepts views that ADD coverage, so you can't fill it with duplicates.

Run WITH the system OpenCV (for the GUI window):
  PYTHONNOUSERSITE=1 python3 ~/ros2_ws/src/igvc_camstuff/fisheye_calib.py

Move the board to fill all four bars (X, Y, SIZE, SKEW). Press 'c' to calibrate
when they're full (or it auto-calibrates at the target count). 'q' to quit.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
import numpy as np
import cv2, math, os

BOARD  = (8, 6)
SQUARE = 0.020
TARGET = 30
DIST_THRESH = 0.15      # min pose difference to accept (rejects duplicates)
OUT = os.path.expanduser(
    '~/ros2_ws/src/igvc_camstuff/openvins_config/kalibr_imucam_chain_fisheye.yaml')

R_CAM0_IMU = np.array([[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]])
T_CAM0_IMU = np.array([0.035, 0., 0.])


def outside(c, board):
    cols, rows = board
    return (c[0], c[cols-1], c[-1], c[(rows-1)*cols])

def area(o):
    ul, ur, dr, dl = o
    p = (dr-ur) + (dl-dr); q = (ur-ul) + (dr-ur)
    return abs(p[0]*q[1] - p[1]*q[0]) / 2.

def skew(o):
    ul, ur, dr, _ = o
    ba, bc = ul-ur, dr-ur
    cosv = np.dot(ba, bc)/(np.linalg.norm(ba)*np.linalg.norm(bc)+1e-9)
    ang = math.acos(max(-1, min(1, cosv)))
    return min(1.0, 2.*abs(math.pi/2. - ang))

def params(corners, w, h):
    c = corners.reshape(-1, 2)
    o = outside(c, BOARD)
    a = area(o); s = skew(o); border = math.sqrt(a)
    px = min(1, max(0, (np.mean(c[:,0]) - border/2)/(w - border)))
    py = min(1, max(0, (np.mean(c[:,1]) - border/2)/(h - border)))
    psize = math.sqrt(a/(w*h))
    return np.array([px, py, psize, s])


class Calib(Node):
    def __init__(self):
        super().__init__('fisheye_calib')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        sl = message_filters.Subscriber(self, Image, '/camera_left/image_raw',  qos_profile=qos)
        sr = message_filters.Subscriber(self, Image, '/camera_right/image_raw', qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer([sl, sr], 10, 0.02)
        self.sync.registerCallback(self.cb)
        op = np.zeros((1, BOARD[0]*BOARD[1], 3), np.float64)
        op[0,:,:2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1,2); op *= SQUARE
        self.op = op
        self.objpoints, self.iL, self.iR, self.pdb = [], [], [], []
        self.size = None
        cv2.namedWindow('fisheye_calib', cv2.WINDOW_NORMAL)
        self.get_logger().info("Fill the bars (X/Y/SIZE/SKEW). 'c'=calibrate, 'q'=quit")

    def gray(self, m): return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)

    def novel(self, p):
        if not self.pdb: return True
        return min(np.sum(np.abs(p - q)) for q in self.pdb) > DIST_THRESH

    def coverage(self):
        if not self.pdb: return [0,0,0,0]
        arr = np.array(self.pdb)
        rng = arr.max(0) - arr.min(0)
        tgt = np.array([0.7, 0.7, 0.4, 0.5])
        return list(np.minimum(1.0, rng/tgt))

    def cb(self, ml, mr):
        gl, gr = self.gray(ml), self.gray(mr)
        if self.size is None: self.size = (gl.shape[1], gl.shape[0])
        w, h = self.size
        fl = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        okl, cl = cv2.findChessboardCorners(gl, BOARD, fl | cv2.CALIB_CB_FAST_CHECK)
        okr, cr = cv2.findChessboardCorners(gr, BOARD, fl | cv2.CALIB_CB_FAST_CHECK)
        disp = cv2.cvtColor(cv2.resize(gl, (w//2, h//2)), cv2.COLOR_GRAY2BGR)
        if okl and okr:
            p = params(cl, w, h)
            if self.novel(p) and len(self.objpoints) < TARGET:
                crit = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                cl2 = cv2.cornerSubPix(gl, cl, (5,5), (-1,-1), crit)
                cr2 = cv2.cornerSubPix(gr, cr, (5,5), (-1,-1), crit)
                self.objpoints.append(self.op.copy())
                self.iL.append(cl2.reshape(1,-1,2).astype(np.float64))
                self.iR.append(cr2.reshape(1,-1,2).astype(np.float64))
                self.pdb.append(p)
                self.get_logger().info(f'ACCEPTED {len(self.objpoints)}/{TARGET}')
            cv2.drawChessboardCorners(disp, BOARD, cl*0.5, okl)
        cov = self.coverage()
        labels = ['X','Y','SIZE','SKEW']
        for i,(lb,v) in enumerate(zip(labels, cov)):
            y = 20 + i*22
            cv2.rectangle(disp, (60,y-12),(60+int(150*v),y), (0,255,0),-1)
            cv2.rectangle(disp, (60,y-12),(210,y),(120,120,120),1)
            cv2.putText(disp, lb, (10,y), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(disp, f'{len(self.objpoints)}/{TARGET}  c=calib q=quit',
                    (10, h//2-10), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),1)
        cv2.imshow('fisheye_calib', disp)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): rclpy.shutdown()
        elif k == ord('c') or len(self.objpoints) >= TARGET:
            if len(self.objpoints) >= 8:
                self.calibrate(); rclpy.shutdown()
            else:
                self.get_logger().warn(f'only {len(self.objpoints)} - collect more')

    def calibrate(self):
        N = len(self.objpoints)
        self.get_logger().info(f'Calibrating with {N} views...')
        Kl,Dl,Kr,Dr = np.zeros((3,3)),np.zeros((4,1)),np.zeros((3,3)),np.zeros((4,1))
        rv=[np.zeros((1,1,3)) for _ in range(N)]; tv=[np.zeros((1,1,3)) for _ in range(N)]
        fl = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        cr = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        try:
            rl,Kl,Dl,_,_ = cv2.fisheye.calibrate(self.objpoints,self.iL,self.size,Kl,Dl,rv,tv,fl,cr)
            rr,Kr,Dr,_,_ = cv2.fisheye.calibrate(self.objpoints,self.iR,self.size,Kr,Dr,rv,tv,fl,cr)
        except cv2.error as e:
            self.get_logger().error(f'mono failed: {e}'); return
        self.get_logger().info(f'Mono RMS left={rl:.3f} right={rr:.3f} px')
        R,T = np.zeros((3,3)),np.zeros((3,1))
        try:
            ret = cv2.fisheye.stereoCalibrate(self.objpoints,self.iL,self.iR,Kl,Dl,Kr,Dr,
                    self.size,R,T,cv2.fisheye.CALIB_FIX_INTRINSIC,cr)
            R,T = ret[5], ret[6]
            self.get_logger().info(f'Stereo RMS={ret[0]:.3f} px  baseline={np.linalg.norm(T)*1000:.1f} mm')
        except cv2.error as e:
            self.get_logger().error(f'stereo failed: {e}'); return
        self.write(Kl,Dl,Kr,Dr,R,T.reshape(3))

    def write(self,Kl,Dl,Kr,Dr,R,T):
        def m(M): return '\n'.join('    - ['+', '.join(f'{v:.8f}' for v in r)+']' for r in M)
        Tc0=np.eye(4); Tc0[:3,:3]=R_CAM0_IMU; Tc0[:3,3]=T_CAM0_IMU
        Tcn=np.eye(4); Tcn[:3,:3]=R; Tcn[:3,3]=T; Tc1=Tcn@Tc0
        il=[Kl[0,0],Kl[1,1],Kl[0,2],Kl[1,2]]; ir=[Kr[0,0],Kr[1,1],Kr[0,2],Kr[1,2]]
        dl=Dl.flatten()[:4]; dr=Dr.flatten()[:4]
        t=f"""%YAML:1.0
cam0:
  T_cam_imu:
{m(Tc0)}
  cam_overlaps: [1]
  camera_model: pinhole
  distortion_coeffs: [{dl[0]:.8f}, {dl[1]:.8f}, {dl[2]:.8f}, {dl[3]:.8f}]
  distortion_model: equidistant
  intrinsics: [{il[0]:.6f}, {il[1]:.6f}, {il[2]:.6f}, {il[3]:.6f}]
  resolution: [{self.size[0]}, {self.size[1]}]
  rostopic: /camera_left/image_raw
  timeshift_cam_imu: 0.0
cam1:
  T_cam_imu:
{m(Tc1)}
  T_cn_cnm1:
{m(Tcn)}
  cam_overlaps: [0]
  camera_model: pinhole
  distortion_coeffs: [{dr[0]:.8f}, {dr[1]:.8f}, {dr[2]:.8f}, {dr[3]:.8f}]
  distortion_model: equidistant
  intrinsics: [{ir[0]:.6f}, {ir[1]:.6f}, {ir[2]:.6f}, {ir[3]:.6f}]
  resolution: [{self.size[0]}, {self.size[1]}]
  rostopic: /camera_right/image_raw
  timeshift_cam_imu: 0.0
"""
        open(OUT,'w').write(t); self.get_logger().info(f'WROTE {OUT}'); print(t)

def main():
    rclpy.init(); n=Calib()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    cv2.destroyAllWindows(); 
    try: rclpy.shutdown()
    except: pass

if __name__=='__main__': main()
