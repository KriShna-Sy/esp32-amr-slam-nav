#!/usr/bin/env python3
"""
cam_level: measure the camera's REAL height and tilt from the floor it sees.
Robot standing still, 1.5 m+ of clear, flat floor in front of the camera.
  ros2 run esp32_odom_bridge cam_level
Fits the floor plane (RANSAC) in the camera's own optical frame, so the result does NOT
depend on the cam_z / cam_pitch currently set in the launch files.
Prints the values to put in mapping.launch.py + localization.launch.py; saves logs/cam_level.txt
"""
import os
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

OUT = os.path.expanduser('~/mapping_ws/logs/cam_level.txt')


class Grab(Node):
    def __init__(self):
        super().__init__('cam_level')
        self.clouds = []
        self.frame = ''
        self.create_subscription(PointCloud2, '/oak/points', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.frame = m.header.frame_id
        a = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        self.clouds.append(np.asarray(a, dtype=np.float64).reshape(-1, 3))


def ransac_floor(P, iters=600, thr=0.015):
    rng = np.random.default_rng(0)
    best, best_n, best_d = 0, None, None
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        if n[1] > 0:            # make n point UP (optical frame: y is down)
            n = -n
        # floor must be roughly horizontal: up vector within 60 deg of optical -y / -z mix
        if -n[1] < 0.5:
            continue
        d = -n @ s[0]
        inl = np.sum(np.abs(P @ n + d) < thr)
        if inl > best:
            best, best_n, best_d = inl, n, d
    if best_n is None:
        return None
    # refine with least squares on inliers
    mask = np.abs(P @ best_n + best_d) < thr
    Q = P[mask]
    c = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - c)
    n = vt[2]
    if n[1] > 0:
        n = -n
    d = -n @ c
    return n, d, int(mask.sum()), len(P)


def main():
    rclpy.init()
    g = Grab()
    print("cam_level: collecting /oak/points for 4 s - keep the robot still, floor in view...")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(g, timeout_sec=0.1)
    g.destroy_node()
    rclpy.shutdown()
    if not g.clouds:
        print("No /oak/points received. Is the camera bridge running (localization/mapping launch)?")
        return
    P = np.vstack(g.clouds)
    P = P[(P[:, 2] > 0.3) & (P[:, 2] < 3.5) & (P[:, 1] > 0.0)]   # in front, below the optical axis
    if len(P) > 60000:
        P = P[np.random.default_rng(1).choice(len(P), 60000, replace=False)]
    r = ransac_floor(P) if len(P) > 200 else None
    L = [f"cam_level {time.strftime('%Y-%m-%d %H:%M:%S')}  frame={g.frame}  points used={len(P)}"]
    if r is None:
        L.append("Could not find a floor plane - point the camera at more open floor.")
    else:
        n, d, inl, tot = r
        # up vector in optical frame = (sin(roll)*cos(pitch)?, -cos(p), -sin(p)); see derivation in code comments
        pitch = math.atan2(-n[2], -n[1])          # + = tilted DOWN
        roll = math.atan2(n[0], math.hypot(n[1], n[2]))
        height = abs(d)
        L.append(f"floor plane: {inl}/{tot} inliers ({100.0 * inl / tot:.0f}%)")
        L.append(f"camera height above floor : {height:.3f} m")
        L.append(f"camera tilt (pitch, down+) : {math.degrees(pitch):+.1f} deg = {pitch:+.3f} rad")
        L.append(f"camera roll                : {math.degrees(roll):+.1f} deg = {roll:+.3f} rad")
        if inl / tot < 0.25:
            L.append("WARNING: few floor points - result may be off; retry facing open floor.")
        L.append("")
        L.append("Put these in BOTH launch files (mapping + localization), or pass as launch args:")
        L.append(f"  cam_z:={height:.3f} cam_pitch:={pitch:.3f} cam_roll:={roll:.3f}")
    print("\n".join(L))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        f.write("\n".join(L) + "\n")
    print(f"Saved {OUT}")


if __name__ == '__main__':
    main()
