#!/usr/bin/env python3
"""
imu_tilt: camera tilt (pitch) and roll from the OAK IMU's gravity reading.
Robot STILL on level floor, camera bridge running (mapping or bringup launch).
  ros2 run esp32_odom_bridge imu_tilt            # measure only
  ros2 run esp32_odom_bridge imu_tilt --apply    # also write cam_pitch/cam_roll into both launch files
Uses /oak/imu/data (already in ROS body axes: x fwd, y left, z up).
At rest the accelerometer reads +g "up":  up_body = (-sin p, sin r cos p, cos r cos p)
  -> pitch (down +) = atan2(-ax, az),  roll = atan2(ay, az)
"""
import os
import re
import sys
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

LAUNCH = os.path.expanduser('~/mapping_ws/src/esp32_odom_bridge/launch')
OUT = os.path.expanduser('~/mapping_ws/logs/imu_tilt.txt')


class Grab(Node):
    def __init__(self):
        super().__init__('imu_tilt')
        self.a, self.g = [], []
        self.create_subscription(Imu, '/oak/imu/data', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.a.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        self.g.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))


def apply(pitch, roll):
    done = []
    for fn in ['mapping.launch.py', 'localization.launch.py']:
        p = os.path.join(LAUNCH, fn)
        s = open(p).read()
        n0 = s
        # DeclareLaunchArgument('cam_pitch', default_value='x')
        s = re.sub(r"(DeclareLaunchArgument\('cam_pitch',\s*default_value=')[^']*(')",
                   lambda m: f"{m.group(1)}{pitch:.3f}{m.group(2)}", s)
        s = re.sub(r"(DeclareLaunchArgument\('cam_roll',\s*default_value=')[^']*(')",
                   lambda m: f"{m.group(1)}{roll:.3f}{m.group(2)}", s)
        # localization fallback dict: 'cam_pitch': 'x'
        s = re.sub(r"('cam_pitch':\s*')[^']*(')", lambda m: f"{m.group(1)}{pitch:.3f}{m.group(2)}", s)
        s = re.sub(r"('cam_roll':\s*')[^']*(')", lambda m: f"{m.group(1)}{roll:.3f}{m.group(2)}", s)
        if s != n0:
            open(p, 'w').write(s)
            done.append(fn)
    return done


def main():
    rclpy.init()
    g = Grab()
    print("imu_tilt: robot STILL, reading /oak/imu/data for 4 s ...")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(g, timeout_sec=0.05)
    g.destroy_node()
    rclpy.shutdown()
    if len(g.a) < 50:
        print("Not enough IMU data. Is the camera bridge running (mapping or bringup launch)?")
        return
    A = np.array(g.a)
    G = np.array(g.g)
    ax, ay, az = A.mean(axis=0)
    gyro_std = float(np.linalg.norm(G.std(axis=0)))
    acc_std = float(np.linalg.norm(A.std(axis=0)))
    pitch = math.atan2(-ax, az)
    roll = math.atan2(ay, az)
    L = [f"imu_tilt {time.strftime('%Y-%m-%d %H:%M:%S')}  samples={len(A)}",
         f"accel mean (x,y,z) = {ax:+.3f} {ay:+.3f} {az:+.3f}  |g| = {np.linalg.norm([ax, ay, az]):.3f}",
         f"camera tilt (pitch, down+) : {math.degrees(pitch):+.2f} deg = {pitch:+.3f} rad",
         f"camera roll                : {math.degrees(roll):+.2f} deg = {roll:+.3f} rad"]
    if acc_std > 0.15 or gyro_std > 0.02:
        L.append(f"WARNING: robot was moving/vibrating (accel std {acc_std:.2f}, gyro std {gyro_std:.3f}) - redo standing still.")
    if abs(np.linalg.norm([ax, ay, az]) - 9.81) > 0.5:
        L.append("WARNING: |g| far from 9.81 - IMU axis mapping or scale is off.")
    if '--apply' in sys.argv:
        done = apply(pitch, roll)
        L.append(f"APPLIED cam_pitch={pitch:.3f} cam_roll={roll:.3f} to: {', '.join(done) if done else 'nothing changed'}")
        L.append("Rebuild: cd ~/mapping_ws && colcon build --packages-select esp32_odom_bridge")
    else:
        L.append(f"Use: cam_pitch:={pitch:.3f} cam_roll:={roll:.3f}   (or rerun with --apply)")
    print("\n".join(L))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    open(OUT, 'w').write("\n".join(L) + "\n")


if __name__ == '__main__':
    main()
