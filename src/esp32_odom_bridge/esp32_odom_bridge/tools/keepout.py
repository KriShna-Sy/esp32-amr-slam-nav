#!/usr/bin/env python3
"""
keepout: mark no-go zones on the active map (things the camera map missed: low edges,
cables, platform lips, glass...). Zones are stored in ~/mapping_ws/nav_data/keepout.yaml
and painted as LETHAL into /map_nav by map_downsampler within ~2 s (no restart needed).
The planner then routes around them.

  ros2 run esp32_odom_bridge keepout front <name> [depth_m] [width_m]
        robot parked with its nose just in front of the hazard: marks a box starting at the
        front bumper, depth_m ahead (default 0.40), width_m wide (default 0.60)
  ros2 run esp32_odom_bridge keepout click <name>
        click the corners in RViz with the "Publish Point" tool (3+ points), then press Enter
  ros2 run esp32_odom_bridge keepout box <name> <x> <y> <size_x> <size_y>
        axis-aligned box centred on map x,y
  ros2 run esp32_odom_bridge keepout list
  ros2 run esp32_odom_bridge keepout remove <name>
"""
import os
import sys
import math
import time
import select
import yaml

KEEPOUT = os.path.expanduser('~/mapping_ws/nav_data/keepout.yaml')
FRONT_BUMPER = 0.42   # base_link (drive axle) -> front edge


def load():
    try:
        d = yaml.safe_load(open(KEEPOUT)) or {}
        return d.get('zones', {}) or {}
    except FileNotFoundError:
        return {}


def save(zones):
    os.makedirs(os.path.dirname(KEEPOUT), exist_ok=True)
    tmp = KEEPOUT + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump({'zones': zones}, f, default_flow_style=None)
    os.replace(tmp, KEEPOUT)


def add(name, pts):
    zones = load()
    zones[name] = [[round(float(x), 3), round(float(y), 3)] for x, y in pts]
    save(zones)
    print(f"[OK] keepout '{name}' saved ({len(pts)} corners):")
    for x, y in zones[name]:
        print(f"     ({x:+.2f}, {y:+.2f})")
    print("     map_downsampler paints it into /map_nav within ~2 s (check RViz /map_nav or the global costmap)")


def robot_pose():
    import rclpy
    from rclpy.node import Node
    import tf2_ros
    rclpy.init()
    n = Node('keepout_pose')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    t0 = time.time()
    pose = None
    while time.time() - t0 < 4.0 and pose is None:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            pose = (t.transform.translation.x, t.transform.translation.y, yaw)
        except Exception:
            pass
    n.destroy_node()
    rclpy.shutdown()
    return pose


def cmd_front(name, depth=0.40, width=0.60):
    p = robot_pose()
    if p is None:
        print("[ERROR] no map->base_link pose. Is bringup running and the robot localized (whereami --wait)?")
        return
    x, y, yaw = p
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(FRONT_BUMPER, width / 2), (FRONT_BUMPER + depth, width / 2),
               (FRONT_BUMPER + depth, -width / 2), (FRONT_BUMPER, -width / 2)]
    pts = [(x + c * a - s * b, y + s * a + c * b) for a, b in corners]
    print(f"[INFO] robot at ({x:+.2f}, {y:+.2f}) heading {math.degrees(yaw):+.0f} deg; "
          f"zone = {depth:.2f} m deep x {width:.2f} m wide in front of the bumper")
    add(name, pts)


def cmd_click(name):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PointStamped
    rclpy.init()
    n = Node('keepout_click')
    pts = []

    def cb(m):
        pts.append((m.point.x, m.point.y))
        print(f"  corner {len(pts)}: ({m.point.x:+.2f}, {m.point.y:+.2f})")
    n.create_subscription(PointStamped, '/clicked_point', cb, 10)
    print("RViz: select 'Publish Point' in the top toolbar, click each corner of the zone on the map.")
    print("Press Enter when done (3 or more corners).")
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
    n.destroy_node()
    rclpy.shutdown()
    if len(pts) < 3:
        print(f"[ERROR] only {len(pts)} corners - nothing saved")
        return
    add(name, pts)


def main():
    a = sys.argv[1:]
    if not a or a[0] in ('-h', '--help', 'help'):
        print(__doc__)
        return
    cmd = a[0]
    if cmd == 'list':
        z = load()
        if not z:
            print("no keepout zones")
        for k, v in z.items():
            xs, ys = [p[0] for p in v], [p[1] for p in v]
            print(f"{k:20s} {len(v)} corners, x {min(xs):+.2f}..{max(xs):+.2f}, y {min(ys):+.2f}..{max(ys):+.2f}")
    elif cmd == 'remove' and len(a) >= 2:
        z = load()
        if z.pop(a[1], None) is None:
            print(f"[ERROR] no zone '{a[1]}'")
        else:
            save(z)
            print(f"[OK] removed '{a[1]}'")
    elif cmd == 'front' and len(a) >= 2:
        cmd_front(a[1], *(float(v) for v in a[2:4]))
    elif cmd == 'click' and len(a) >= 2:
        cmd_click(a[1])
    elif cmd == 'box' and len(a) >= 6:
        x, y, sx, sy = (float(v) for v in a[2:6])
        add(a[1], [(x - sx / 2, y - sy / 2), (x + sx / 2, y - sy / 2),
                   (x + sx / 2, y + sy / 2), (x - sx / 2, y + sy / 2)])
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
