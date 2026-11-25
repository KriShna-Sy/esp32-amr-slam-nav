#!/usr/bin/env python3
"""
nav_diag: run WHILE a goto is active. Watches the Nav2 -> ESP32 command chain for 10 s
and finds where the motion command dies.
  T2: ros2 run esp32_odom_bridge landmark_manager goto door_point
  T3: ros2 run esp32_odom_bridge nav_diag
Saves ~/mapping_ws/logs/nav_diag.txt
"""
import os
import time
import math
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
import numpy as np


def cloud_xyz(m):
    a = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.asarray(a, dtype=np.float64).reshape(-1, 3)


def to_base(tfb, m, pts):
    if m.header.frame_id in ('base_link', ''):
        return pts
    t = tfb.lookup_transform('base_link', m.header.frame_id, rclpy.time.Time())
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    tr = t.transform.translation
    return pts @ R.T + np.array([tr.x, tr.y, tr.z])

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:
    CollisionMonitorState = None

OUT = os.path.expanduser('~/mapping_ws/logs/nav_diag.txt')
DUR = 10.0
STOP_BOX = (0.0, 0.75, 0.32)   # forward stop zone in nav2_params (PolygonStop.forward)
SLOW_BOX = (0.0, 1.20, 0.40)
ACTIONS = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH', 4: 'LIMIT'}


class Diag(Node):
    def __init__(self):
        super().__init__('nav_diag')
        self.tw = {}
        for t in ['/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel_safe', '/cmd_vel']:
            self.tw[t] = {'n': 0, 'nonzero': 0, 'max_lin': 0.0, 'max_ang': 0.0, 'last': None}
            self.create_subscription(Twist, t, lambda m, t=t: self.on_tw(t, m), 10)
        self.cm = []
        if CollisionMonitorState:
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.on_cm, 10)
        self.odom = {'n': 0, 'max_v': 0.0, 'max_w': 0.0}
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.wodom = {'n': 0, 'max_v': 0.0}
        self.create_subscription(Odometry, '/wheel_odom', self.on_wodom, 10)
        self.cloud = {'n': 0, 'frame': '', 'stop': [], 'slow': [], 'pts': [], 'near': None}
        self.create_subscription(PointCloud2, '/obstacles_cloud', self.on_cloud, qos_profile_sensor_data)
        self.raw = {}
        for t in ['/oak/points', '/ground_cloud', '/camera_cloud']:
            self.raw[t] = {'n': 0, 'frame': '', 'pts': 0, 'stamp_age': None}
            self.create_subscription(PointCloud2, t, lambda m, t=t: self.on_raw(t, m), qos_profile_sensor_data)
        self.floor_samples = []
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)

    def on_raw(self, t, m):
        r = self.raw[t]
        r['n'] += 1
        r['frame'] = m.header.frame_id
        r['pts'] = m.width * m.height
        now = self.get_clock().now().nanoseconds * 1e-9
        r['stamp_age'] = now - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if t == '/oak/points':
            try:
                self.floor_samples.append(to_base(self.tfb, m, cloud_xyz(m)))
                self.floor_samples = self.floor_samples[-10:]
            except Exception:
                pass

    def on_tw(self, t, m):
        d = self.tw[t]
        d['n'] += 1
        d['last'] = (m.linear.x, m.angular.z)
        if abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3:
            d['nonzero'] += 1
        d['max_lin'] = max(d['max_lin'], abs(m.linear.x))
        d['max_ang'] = max(d['max_ang'], abs(m.angular.z))

    def on_cm(self, m):
        self.cm.append((ACTIONS.get(m.action_type, str(m.action_type)), m.polygon_name))

    def on_odom(self, m):
        self.odom['n'] += 1
        self.odom['max_v'] = max(self.odom['max_v'], abs(m.twist.twist.linear.x))
        self.odom['max_w'] = max(self.odom['max_w'], abs(m.twist.twist.angular.z))

    def on_wodom(self, m):
        self.wodom['n'] += 1
        self.wodom['max_v'] = max(self.wodom['max_v'], abs(m.twist.twist.linear.x))

    def on_cloud(self, m):
        c = self.cloud
        c['n'] += 1
        c['frame'] = m.header.frame_id
        stop = slow = 0
        near = None
        n = 0
        try:
            P = to_base(self.tfb, m, cloud_xyz(m))
        except Exception:
            return
        c['heights'] = P[:, 2]
        for x, y, z in P:
            n += 1
            if STOP_BOX[0] <= x <= STOP_BOX[1] and abs(y) <= STOP_BOX[2]:
                stop += 1
                if near is None or x < near[0]:
                    near = (float(x), float(y), float(z))
            if SLOW_BOX[0] <= x <= SLOW_BOX[1] and abs(y) <= SLOW_BOX[2]:
                slow += 1
        c['stop'].append(stop)
        c['slow'].append(slow)
        c['pts'].append(n)
        if near:
            c['near'] = near


def main():
    rclpy.init()
    d = Diag()
    print(f"nav_diag: watching for {DUR:.0f} s - keep the goto running...")
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(d, timeout_sec=0.05)

    L = [f"nav_diag {time.strftime('%Y-%m-%d %H:%M:%S')}  ({DUR:.0f} s)", ""]
    L.append("Command chain (Twist msgs: count / non-zero / max lin / max ang / last):")
    for t, v in d.tw.items():
        L.append(f"  {t:<18} n={v['n']:<4} nonzero={v['nonzero']:<4} "
                 f"max_lin={v['max_lin']:.3f} max_ang={v['max_ang']:.3f} last={v['last']}")
    cms = {}
    for a, p in d.cm:
        cms[(a, p)] = cms.get((a, p), 0) + 1
    L.append(f"collision_monitor_state: {cms if cms else '(no msgs)'}")
    c = d.cloud
    if c['n']:
        L.append(f"/obstacles_cloud: {c['n']} msgs, frame='{c['frame']}', pts/msg avg={sum(c['pts'])/c['n']:.0f}")
        L.append(f"  points in STOP box (x 0-0.75, |y|<0.32): max={max(c['stop'])} avg={sum(c['stop'])/c['n']:.1f}  (>=4 -> stop)")
        L.append(f"  points in SLOW box (x 0-1.2, |y|<0.40): max={max(c['slow'])} avg={sum(c['slow'])/c['n']:.1f}")
        if c['near']:
            L.append(f"  nearest stop-box point (x,y,z) = ({c['near'][0]:.2f}, {c['near'][1]:.2f}, {c['near'][2]:.2f})")
    else:
        L.append("/obstacles_cloud: NO MESSAGES (collision_monitor source timeout -> permanent stop)")
    for t, r in d.raw.items():
        age = f"{r['stamp_age']:+.2f}s" if r['stamp_age'] is not None else '-'
        L.append(f"{t}: n={r['n']} frame='{r['frame']}' pts={r['pts']} stamp_age={age}")
    pf = d.raw['/oak/points']['frame']
    if pf:
        try:
            d.tfb.lookup_transform('base_link', pf, rclpy.time.Time())
            L.append(f"TF base_link <- {pf}: OK")
        except Exception as e:
            L.append(f"TF base_link <- {pf}: FAILED ({type(e).__name__})")
    if d.floor_samples:
        P = np.vstack(d.floor_samples)
        P = P[np.abs(P[:, 1]) < 1.0]
        L.append("Floor check (/oak/points in base_link, |y|<1 m): per distance bin, low-percentile height")
        L.append("  should be ~0.00 everywhere; rising/falling with distance = camera pitch wrong")
        xs, zs = [], []
        for x0 in np.arange(0.5, 4.01, 0.5):
            b = P[(P[:, 0] >= x0) & (P[:, 0] < x0 + 0.5)]
            if len(b) < 30:
                L.append(f"  x {x0:.1f}-{x0+0.5:.1f} m: {len(b)} pts (too few)")
                continue
            z5 = float(np.percentile(b[:, 2], 5))
            L.append(f"  x {x0:.1f}-{x0+0.5:.1f} m: n={len(b):5d}  floor z(p5)={z5:+.3f}  p50={float(np.median(b[:, 2])):+.3f}")
            xs.append(x0 + 0.25)
            zs.append(z5)
        if len(xs) >= 2:
            k, c0 = np.polyfit(xs, zs, 1)
            L.append(f"  floor fit: z = {k:+.4f}*x {c0:+.3f}  -> pitch error {math.degrees(math.atan(k)):+.2f} deg "
                     f"(cam_pitch should be about {0.109 + math.atan(k):.3f} rad)")
    hs = d.cloud.get('heights')
    if hs is not None and len(hs):
        L.append(f"/obstacles_cloud heights in base_link: min={hs.min():+.2f} p10={np.percentile(hs,10):+.2f} "
                 f"median={np.median(hs):+.2f} max={hs.max():+.2f}")
    L.append(f"/odom: n={d.odom['n']} max_v={d.odom['max_v']:.3f} max_w={d.odom['max_w']:.3f}")
    L.append(f"/wheel_odom: n={d.wodom['n']} max_v={d.wodom['max_v']:.3f}")

    nav, sm, safe, tele = (d.tw[k] for k in ['/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel_safe', '/cmd_vel'])
    L.append("")
    if nav['nonzero'] == 0:
        v = "controller sends nothing/zero -> controller or goal problem (is a goto running?)"
    elif sm['nonzero'] == 0:
        v = "velocity_smoother outputs zero -> smoother (deadband/limits) is eating the command"
    elif safe['nonzero'] == 0 and c['n'] == 0:
        if d.raw['/oak/points']['n'] == 0:
            v = "camera bridge publishes NO /oak/points -> no obstacle cloud -> collision_monitor blocks"
        else:
            v = "/oak/points OK but obstacles_detection publishes nothing (TF/stamp problem) -> collision_monitor blocks"
    elif safe['nonzero'] == 0:
        v = "collision_monitor outputs zero -> it sees an obstacle in the STOP box"
    elif tele['n'] > 0:
        v = "something publishes /cmd_vel (teleop priority) -> Nav2 is overridden by the bridge"
    elif d.wodom['max_v'] < 0.01 and d.odom['max_w'] < 0.02:
        v = "commands reach /cmd_vel_safe but wheels don't move -> bridge/serial/ESP32/motor power"
    else:
        v = "chain looks OK - robot should be moving"
    L.append("VERDICT: " + v)
    print("\n".join(L))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        f.write("\n".join(L) + "\n")
    print(f"Saved {OUT}")
    d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
