#!/usr/bin/env python3
"""
yaw_check: find out which heading source (wheels / IMU axis / EKF) turns the RIGHT way.

Needs mapping.launch.py running (bridge + camera + EKF). Robot pivots LEFT in place.
  ros2 run esp32_odom_bridge yaw_check
Result is printed and saved to ~/mapping_ws/logs/yaw_check.txt
"""
import math
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

OUT = os.path.expanduser('~/mapping_ws/logs/yaw_check.txt')
TURN_RATE = 0.25   # rad/s (mapping launch caps at 0.25)
TURN_TIME = 5.0    # s


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class YawCheck(Node):
    def __init__(self):
        super().__init__('yaw_check')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wheel, self.imu, self.ekf = [], [], []
        self.create_subscription(Odometry, '/wheel_odom', lambda m: self.wheel.append(
            (time.time(), m.twist.twist.angular.z)), 50)
        self.create_subscription(Imu, '/oak/imu/data', lambda m: self.imu.append(
            (time.time(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
             m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)), 200)
        self.create_subscription(Odometry, '/odom', lambda m: self.ekf.append(
            (time.time(), yaw_of(m.pose.pose.orientation))), 50)

    def spin_for(self, sec, cmd=None):
        end = time.time() + sec
        while time.time() < end:
            if cmd is not None:
                t = Twist(); t.angular.z = cmd; self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)


def integ(samples, t0, t1, col):
    s = [(r[0], r[col]) for r in samples if t0 <= r[0] <= t1]
    if len(s) < 2:
        return float('nan')
    tot = 0.0
    for (ta, va), (tb, vb) in zip(s, s[1:]):
        tot += 0.5 * (va + vb) * (tb - ta)
    return math.degrees(tot)


def main():
    rclpy.init()
    n = YawCheck()
    lines = []
    def out(s=''):
        print(s); lines.append(s)

    print("Waiting for /wheel_odom, /oak/imu/data, /odom ...")
    n.spin_for(3.0)
    out(f"samples: wheel={len(n.wheel)} imu={len(n.imu)} ekf={len(n.ekf)}")
    if not n.wheel or not n.imu or not n.ekf:
        out("ERROR: missing topics. Start: ros2 launch esp32_odom_bridge mapping.launch.py")
        return

    imu = np.array(n.imu)
    acc = imu[:, 4:7].mean(axis=0); gyr = imu[:, 1:4].mean(axis=0)
    out(f"STILL  accel mean (ros x,y,z) = {acc[0]:+.2f} {acc[1]:+.2f} {acc[2]:+.2f}  "
        f"(gravity should be ~+9.8 on z)")
    out(f"STILL  gyro mean  (ros x,y,z) = {gyr[0]:+.4f} {gyr[1]:+.4f} {gyr[2]:+.4f} rad/s")

    input(f"\nClear 1 m around the robot. It will PIVOT LEFT (counter-clockwise seen from above) "
          f"for {TURN_TIME:.0f}s. Press Enter...")
    t0 = time.time()
    ekf0 = n.ekf[-1][1]
    n.spin_for(TURN_TIME, cmd=TURN_RATE)
    n.spin_for(0.3, cmd=0.0)
    n.spin_for(1.5)
    t1 = time.time()
    ekf1 = n.ekf[-1][1]

    ans = input("Did the robot physically turn LEFT / counter-clockwise? [y/n]: ").strip().lower()
    deg = input("Roughly how many degrees did it turn? (number, Enter to skip): ").strip()
    truth = 1 if ans.startswith('y') else -1

    w = integ(n.wheel, t0, t1, 1)
    ix, iy, iz = (integ(n.imu, t0, t1, c) for c in (1, 2, 3))
    e = math.degrees(math.atan2(math.sin(ekf1 - ekf0), math.cos(ekf1 - ekf0)))
    out("")
    out(f"physical turn : {'LEFT (+)' if truth > 0 else 'RIGHT (-)'}  ~{deg or '?'} deg")
    out(f"wheel odom    : {w:+7.1f} deg")
    out(f"IMU ros x     : {ix:+7.1f} deg")
    out(f"IMU ros y     : {iy:+7.1f} deg")
    out(f"IMU ros z     : {iz:+7.1f} deg   <- bridge currently uses this as yaw")
    out(f"EKF /odom yaw : {e:+7.1f} deg")
    out("")
    out(f"VERDICT wheel : {'OK' if w * truth > 0 else 'WRONG SIGN'}")
    axes = {'x': ix, 'y': iy, 'z': iz}
    best = max(axes, key=lambda k: abs(axes[k]) if not math.isnan(axes[k]) else -1)
    sign = '+' if axes[best] * truth > 0 else '-'
    out(f"VERDICT imu   : real yaw axis = ros {best} with sign {sign}"
        f" -> {'OK' if (best == 'z' and sign == '+') else 'WRONG MAPPING'}")
    out(f"VERDICT ekf   : {'OK' if e * truth > 0 else 'WRONG SIGN'}")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        f.write(time.strftime('%Y-%m-%d %H:%M:%S') + '\n' + '\n'.join(lines) + '\n')
    print(f"\nSaved {OUT}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
