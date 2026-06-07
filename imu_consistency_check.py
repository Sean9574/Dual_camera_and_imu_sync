#!/usr/bin/env python3
"""
imu_consistency_check.py  (v2: full 48 signed-axis-permutation search)
Finds the gyro axis/sign remapping that makes the gyroscope consistent with the
accelerometer. Physics: rotating while accel ~ gravity, d(g_hat)/dt = -(w x g_hat).
We test all 48 signed permutations of the gyro axes and report which makes the
gyro prediction match the accel-measured gravity-direction change.

  best = (+x,+y,+z), score ~ +0.9  -> IMU CONSISTENT (problem is the extrinsic)
  best = some other mapping ~ +0.9 -> apply THAT remap to the gyro (firmware bug)

Run, then ROTATE the rig smoothly IN PLACE through all 3 axes for ~25 s.
"""
import math, itertools
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def nrm(a):
    return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

PERMS = list(itertools.permutations((0, 1, 2)))
SIGNS = [(sx, sy, sz) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
XFORMS = [(p, s) for p in PERMS for s in SIGNS]  # 48
AX = 'xyz'
def apply_x(w, xf):
    p, s = xf
    return (s[0]*w[p[0]], s[1]*w[p[1]], s[2]*w[p[2]])
def label(xf):
    p, s = xf
    return 'gyro=(' + ', '.join(('-' if s[i] < 0 else '+') + AX[p[i]] for i in range(3)) + ')'

class Check(Node):
    def __init__(self):
        super().__init__('imu_consistency_check')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, '/imu/data', self.cb, qos)
        self.prev_g = None; self.prev_t = None
        self.sd = {xf: 0.0 for xf in XFORMS}
        self.sp = {xf: 0.0 for xf in XFORMS}
        self.sm = 0.0; self.n = 0
        self.create_timer(2.0, self.report)
        self.get_logger().info('IMU consistency v2 -- ROTATE the rig through all axes in place ~25s')

    def cb(self, msg):
        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        w = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        na = nrm(a)
        if na < 1e-6:
            return
        g = (a[0]/na, a[1]/na, a[2]/na)
        if self.prev_g is not None:
            dt = t - self.prev_t
            if 1e-4 < dt < 0.1:
                mdg = ((g[0]-self.prev_g[0])/dt, (g[1]-self.prev_g[1])/dt, (g[2]-self.prev_g[2])/dt)
                if nrm(w) > 0.15 and nrm(mdg) > 0.05 and 8.5 < na < 11.0:
                    for xf in XFORMS:
                        wc = apply_x(w, xf); pc = cross(wc, self.prev_g)
                        pdg = (-pc[0], -pc[1], -pc[2])
                        self.sd[xf] += mdg[0]*pdg[0]+mdg[1]*pdg[1]+mdg[2]*pdg[2]
                        self.sp[xf] += pdg[0]*pdg[0]+pdg[1]*pdg[1]+pdg[2]*pdg[2]
                    self.sm += nrm(mdg)**2; self.n += 1
        self.prev_g = g; self.prev_t = t

    def report(self):
        if self.n < 30:
            self.get_logger().info(f'collecting... {self.n} (keep rotating in place)')
            return
        cos = {xf: (self.sd[xf]/math.sqrt(self.sm*self.sp[xf]) if self.sp[xf] > 0 else 0) for xf in XFORMS}
        r = sorted(cos.items(), key=lambda kv: kv[1], reverse=True)
        ident = cos[((0, 1, 2), (1, 1, 1))]
        top = ' | '.join(f'{label(xf)} {v:+.2f}' for xf, v in r[:3])
        self.get_logger().info(f'n={self.n} identity={ident:+.2f} || TOP: {top}')
        best, bv = r[0]
        if bv < 0.55:
            self.get_logger().info('=> best is weak; rotate more, in place (less translation). If it stays low, accel axes may also be off.')
        elif best == ((0, 1, 2), (1, 1, 1)):
            self.get_logger().info('=> IMU CONSISTENT. Problem is the cam-IMU extrinsic, not the IMU.')
        else:
            self.get_logger().info(f'=> FIX: remap published gyro to {label(best)} (axes/signs from raw x,y,z). That is the root cause.')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Check())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
