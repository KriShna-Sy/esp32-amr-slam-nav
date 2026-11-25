#!/usr/bin/env python3
"""
responsive_teleop.py - keyboard teleop with stop-on-release, matched to the bridge limits.

Fixes vs the old version:
  * Terminal stays in cbreak mode for the whole session (old code switched raw<->cooked
    every 60 ms: keys typed in between were echoed / line-buffered / lost -> missed or
    delayed presses).
  * Speeds are clamped to what the robot will REALLY do. The bridge clamps to
    max_linear_speed / max_angular_speed (mapping: 0.12 m/s, 0.25 rad/s) and boosts any
    in-place turn below 0.18 rad/s up to 0.18. Old defaults (0.15 / 0.30) and the speed
    keys therefore did nothing visible ("q/z/c don't work").
  * Status line shows the command actually being sent.
Params (ros2 run ... --ros-args -p max_lin:=0.12 -p max_ang:=0.25):
  max_lin, max_ang, min_pivot (must match the bridge), speed, turn (start values)
"""
import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MOVE = {
    'i': (1.0, 0.0), ',': (-1.0, 0.0), 'j': (0.0, 1.0), 'l': (0.0, -1.0),
    'u': (1.0, 1.0), 'o': (1.0, -1.0), 'm': (-1.0, -1.0), '.': (-1.0, 1.0),
}
SPEED = {'q': (1.1, 1.1), 'z': (0.9, 0.9), 'w': (1.1, 1.0), 'x': (0.9, 1.0), 'e': (1.0, 1.1), 'c': (1.0, 0.9)}

HELP = """
--------------------------------------------------
   TELEOP  (hold a key to drive, release = stop)
--------------------------------------------------
   u    i    o
   j    k    l        k / space : stop
   m    ,    .        q/z all speeds, w/x linear, e/c turn (+-10 %)
Ctrl-C to quit
--------------------------------------------------"""


class Teleop(Node):
    def __init__(self):
        super().__init__('responsive_teleop')
        self.max_lin = float(self.declare_parameter('max_lin', 0.12).value)
        self.max_ang = float(self.declare_parameter('max_ang', 0.25).value)
        self.min_pivot = float(self.declare_parameter('min_pivot', 0.18).value)
        self.speed = float(self.declare_parameter('speed', 0.12).value)
        self.turn = float(self.declare_parameter('turn', 0.25).value)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cur = (0.0, 0.0)
        self.moving = False
        self.last_pub = 0.0
        self.clamp_speeds()

    def clamp_speeds(self):
        self.speed = min(max(self.speed, 0.03), self.max_lin)
        self.turn = min(max(self.turn, self.min_pivot), self.max_ang)

    def send(self, lin, ang):
        self.cur = (lin, ang)
        self.last_pub = time.time()
        m = Twist()
        m.linear.x, m.angular.z = float(lin), float(ang)
        self.pub.publish(m)

    def stop(self):
        self.send(0.0, 0.0)
        self.moving = False


def status(n, tag):
    lin, ang = n.cur
    sys.stdout.write(f"\r[{tag:<7}] lin {lin:+.2f} m/s  ang {ang:+.2f} rad/s | "
                     f"set: speed {n.speed:.2f} (max {n.max_lin:.2f})  turn {n.turn:.2f} "
                     f"({n.min_pivot:.2f}-{n.max_ang:.2f})   ")
    sys.stdout.flush()


def main():
    rclpy.init()
    n = Teleop()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(HELP)
    tty.setcbreak(fd)                 # once, for the whole session
    last_key, last_key_t, repeating = '', 0.0, False
    try:
        status(n, 'READY')
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.03)
            now = time.time()
            if r:
                k = sys.stdin.read(1)
                if k == '\x03':
                    break
                if k in MOVE:
                    a, b = MOVE[k]
                    repeating = n.moving and k == last_key
                    n.send(a * n.speed, b * n.turn)
                    n.moving, last_key, last_key_t = True, k, now
                    status(n, 'DRIVE')
                elif k in SPEED:
                    n.speed *= SPEED[k][0]
                    n.turn *= SPEED[k][1]
                    n.clamp_speeds()
                    if n.moving and last_key in MOVE:
                        a, b = MOVE[last_key]
                        n.send(a * n.speed, b * n.turn)
                    status(n, 'SPEED')
                elif k in ('k', ' '):
                    n.stop()
                    status(n, 'STOP')
            else:
                # first press: OS auto-repeat starts after ~0.25-0.6 s -> 0.7 s grace;
                # once repeating, release is detected within 0.15 s
                if n.moving and now - last_key_t > (0.15 if repeating else 0.7):
                    n.stop()
                    status(n, 'RELEASE')
                elif n.moving and now - n.last_pub > 0.1:
                    n.send(*n.cur)    # keep-alive (bridge watchdog 0.25 s)
            rclpy.spin_once(n, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        n.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\nTeleop stopped.")
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
