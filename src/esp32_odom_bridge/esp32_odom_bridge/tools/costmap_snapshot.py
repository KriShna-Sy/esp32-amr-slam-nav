#!/usr/bin/env python3
"""
costmap_snapshot: picture + report of what Nav2 thinks is blocked around the robot.
  ros2 run esp32_odom_bridge costmap_snapshot [landmark]
With a landmark name it also checks the STRAIGHT line robot -> landmark and lists
every blocked cell on it (why Nav2 took a detour).
Saves ~/mapping_ws/logs/costmap_snapshot.png + costmap_snapshot.txt
"""
import os
import sys
import math
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import tf2_ros

LOGDIR = os.path.expanduser('~/mapping_ws/logs')
LANDMARKS = os.path.expanduser('~/mapping_ws/nav_data/landmarks.yaml')
FOOT = [(0.42, 0.30), (0.42, -0.30), (-0.10, -0.30), (-0.10, 0.30)]
VIEW = 4.0   # half-width of picture window (m)


class Snap(Node):
    def __init__(self):
        super().__init__('costmap_snapshot')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.grids = {}
        for t in ['/map', '/global_costmap/costmap', '/local_costmap/costmap']:
            self.create_subscription(OccupancyGrid, t, lambda m, t=t: self.grids.__setitem__(t, m), latched)
            self.create_subscription(OccupancyGrid, t, lambda m, t=t: self.grids.__setitem__(t, m), 1)
        self.plan = None
        self.create_subscription(Path, '/plan', lambda m: setattr(self, 'plan', m), 1)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)


def grid_np(g):
    return np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)


def world_to_cell(g, x, y):
    return (int((x - g.info.origin.position.x) / g.info.resolution),
            int((y - g.info.origin.position.y) / g.info.resolution))


def lookup(tfb, target, source):
    t = tfb.lookup_transform(target, source, rclpy.time.Time())
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


PAD = 0.04   # global_costmap footprint_padding


def foot_max_cost(G, g, x, y, yaw, pad=PAD):
    """Max cost of any cell inside the padded footprint at pose (x, y, yaw)."""
    r = g.info.resolution
    c, s = math.cos(yaw), math.sin(yaw)
    worst, wcell = -1, None
    a = -0.10 - pad
    while a <= 0.42 + pad + 1e-9:
        b = -0.30 - pad
        while b <= 0.30 + pad + 1e-9:
            wx, wy = x + c * a - s * b, y + s * a + c * b
            cx, cy = world_to_cell(g, wx, wy)
            if 0 <= cx < g.info.width and 0 <= cy < g.info.height:
                v = int(G[cy, cx])
                if v > worst:
                    worst, wcell = v, (wx, wy)
            b += r / 2
        a += r / 2
    return worst, wcell


def plan_to(n, x, y, yaw):
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    if not ac.wait_for_server(timeout_sec=3.0):
        return None
    g = ComputePathToPose.Goal()
    g.goal = PoseStamped()
    g.goal.header.frame_id = 'map'
    g.goal.pose.position.x, g.goal.pose.position.y = x, y
    g.goal.pose.orientation.z, g.goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    g.planner_id = 'GridBased'
    g.use_start = False
    fut = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=10.0)
    res = rf.result()
    return res.result.path if res and res.result.path.poses else None


def main():
    goal_name = sys.argv[1] if len(sys.argv) > 1 else None
    rclpy.init()
    n = Snap()
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    L = [f"costmap_snapshot {time.strftime('%Y-%m-%d %H:%M:%S')}"]
    try:
        rx, ry, ryaw = lookup(n.tfb, 'map', 'base_link')
    except Exception as e:
        print(f"No map->base_link TF ({e}). Is localization running?")
        return
    L.append(f"robot (map): x={rx:.2f} y={ry:.2f} yaw={math.degrees(ryaw):.1f} deg")
    L.append("grids received: " + ", ".join(f"{k} {v.info.width}x{v.info.height}@{v.info.resolution:.2f}"
                                            for k, v in n.grids.items()))
    gc = n.grids.get('/global_costmap/costmap')
    st = n.grids.get('/map')

    goal = None
    if goal_name:
        try:
            lm = yaml.safe_load(open(LANDMARKS)) or {}
            goal = (lm[goal_name]['x'], lm[goal_name]['y'])
            d = math.hypot(goal[0] - rx, goal[1] - ry)
        except Exception as e:
            L.append(f"landmark '{goal_name}' not found: {e}")

    if goal and gc is not None:
        G = grid_np(gc)
        S = grid_np(st) if st is not None else None
        d = math.hypot(goal[0] - rx, goal[1] - ry)
        steps = int(d / gc.info.resolution) + 1
        L.append(f"\nStraight line robot -> {goal_name}: {d:.2f} m")
        L.append("  dist   x      y     global_cost  static_map")
        worst = 0
        blocked = 0
        for i in range(steps + 1):
            f = i / max(steps, 1)
            x, y = rx + f * (goal[0] - rx), ry + f * (goal[1] - ry)
            cx, cy = world_to_cell(gc, x, y)
            if not (0 <= cx < gc.info.width and 0 <= cy < gc.info.height):
                continue
            c = int(G[cy, cx])
            sm = '-'
            if S is not None:
                sx, sy = world_to_cell(st, x, y)
                if 0 <= sx < st.info.width and 0 <= sy < st.info.height:
                    sm = int(S[sy, sx])
            worst = max(worst, c)
            if c >= 99:
                blocked += 1
            if c >= 70 or i % 10 == 0:
                tag = '  <-- LETHAL (inside robot footprint)' if c >= 99 else ('  <-- high' if c >= 70 else '')
                L.append(f"  {f*d:4.2f}  {x:+.2f} {y:+.2f}   {c:4d}        {sm}{tag}")
        L.append(f"  max cost on line = {worst} (100 lethal, 99 inscribed), lethal cells = {blocked}")
        L.append("  -> straight route BLOCKED" if blocked else "  -> straight route is open in the costmap")

    # Footprint check along the straight line (what the lattice planner actually tests)
    if goal and gc is not None:
        G = grid_np(gc)
        hd = math.atan2(goal[1] - ry, goal[0] - rx)
        L.append(f"\nFootprint (+{PAD} m pad) along the line, heading {math.degrees(hd):.0f} deg:")
        L.append("  dist    x      y    max_cost_under_body  worst_cell")
        k = 0.0
        while k <= d + 1e-6:
            x, y = rx + k / d * (goal[0] - rx), ry + k / d * (goal[1] - ry)
            mc, wc = foot_max_cost(G, gc, x, y, hd)
            tag = '  <-- COLLISION' if mc >= 100 else ('  <-- inscribed' if mc >= 99 else '')
            L.append(f"  {k:4.2f}  {x:+.2f} {y:+.2f}      {mc:4d}            "
                     f"({wc[0]:+.2f},{wc[1]:+.2f}){tag}" if wc else f"  {k:4.2f}  off-map")
            k += 0.25
        gyaw = math.radians(lm[goal_name].get('yaw_deg', 0.0))
        L.append(f"\nGoal pose footprint, 16 headings (goal yaw {math.degrees(gyaw):.0f}):")
        row = []
        for h in range(16):
            yy = h * math.pi / 8
            mc, _ = foot_max_cost(G, gc, goal[0], goal[1], yy)
            row.append(f"{h*22.5:5.1f}:{mc:3d}")
        L.append("  " + "  ".join(row[:8]))
        L.append("  " + "  ".join(row[8:]))
        mc, wc = foot_max_cost(G, gc, goal[0], goal[1], gyaw)
        L.append(f"  at goal yaw: {mc}" + (f" worst cell ({wc[0]:+.2f},{wc[1]:+.2f})" if wc else ""))

        # Probe: how far along the line can the planner reach?
        L.append("\nPlanner probes along the line (heading = line heading):")
        for frac in (0.2, 0.4, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95):
            x, y = rx + frac * (goal[0] - rx), ry + frac * (goal[1] - ry)
            p = plan_to(n, x, y, hd)
            L.append(f"  {frac*d:4.2f} m  ({x:+.2f},{y:+.2f})  -> {'OK' if p else 'NO PATH'}")

        # Raw dump for offline analysis
        os.makedirs(LOGDIR, exist_ok=True)
        np.savez_compressed(os.path.join(LOGDIR, f'costmap_{goal_name}.npz'), G=G,
                            ox=gc.info.origin.position.x, oy=gc.info.origin.position.y,
                            res=gc.info.resolution,
                            S=grid_np(st) if st is not None else np.zeros((1, 1)),
                            sox=st.info.origin.position.x if st else 0, soy=st.info.origin.position.y if st else 0,
                            sres=st.info.resolution if st else 0,
                            robot=np.array([rx, ry, ryaw]), goal=np.array([goal[0], goal[1], gyaw]))
        L.append(f"raw grids: {LOGDIR}/costmap_{goal_name}.npz")

    # Ask the planner for a real path (planner only - the robot does NOT move)
    if goal:
        ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
        if ac.wait_for_server(timeout_sec=3.0):
            g = ComputePathToPose.Goal()
            g.goal = PoseStamped()
            g.goal.header.frame_id = 'map'
            g.goal.pose.position.x, g.goal.pose.position.y = goal
            yaw = math.radians(lm[goal_name].get('yaw_deg', 0.0))
            g.goal.pose.orientation.z, g.goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            g.planner_id = 'GridBased'
            g.use_start = False
            fut = ac.send_goal_async(g)
            rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
            gh = fut.result()
            if gh is not None and gh.accepted:
                rf = gh.get_result_async()
                rclpy.spin_until_future_complete(n, rf, timeout_sec=10.0)
                res = rf.result()
                path = res.result.path if res else None
                if path is not None and path.poses:
                    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
                    plen = sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1]) for i in range(len(pts)-1))
                    n.plan = path
                    off = max(abs((goal[1]-ry)*(x-rx) - (goal[0]-rx)*(y-ry)) / max(d, 1e-6) for x, y in pts)
                    L.append(f"\nPLANNER: path found, {plen:.2f} m (straight {d:.2f} m), max sideways offset {off:.2f} m")
                    L.append("  -> OK, normal route" if plen < 1.5 * d + 1.0 else "  -> DETOUR: planner avoids the direct corridor")
                else:
                    L.append(f"\nPLANNER: NO PATH (status={res.status if res else 'timeout'}, "
                             f"error={getattr(res.result, 'error_code', '?') if res else '?'})")
            else:
                L.append("\nPLANNER: goal rejected")
        else:
            L.append("\nPLANNER: compute_path_to_pose server not available (is Nav2 up?)")

    # picture
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(14, 7))
        for ax, key in zip(axes, ['/global_costmap/costmap', '/local_costmap/costmap']):
            g = n.grids.get(key)
            ax.set_title(key)
            if g is None:
                ax.text(0.5, 0.5, 'no data', ha='center')
                continue
            A = grid_np(g).astype(float)
            A[A < 0] = np.nan
            ox, oy, r = g.info.origin.position.x, g.info.origin.position.y, g.info.resolution
            ext = [ox, ox + g.info.width * r, oy, oy + g.info.height * r]
            frame = 'map' if key.startswith('/global') else g.header.frame_id
            px, py, pyaw = (rx, ry, ryaw)
            if frame != 'map':
                try:
                    px, py, pyaw = lookup(n.tfb, frame, 'base_link')
                except Exception:
                    pass
            ax.imshow(A, origin='lower', extent=ext, cmap='magma_r', vmin=0, vmax=100)
            c, s = math.cos(pyaw), math.sin(pyaw)
            fp = [(px + c * a - s * b, py + s * a + c * b) for a, b in FOOT + [FOOT[0]]]
            ax.plot([p[0] for p in fp], [p[1] for p in fp], 'c-', lw=2)
            if frame == 'map':
                if goal:
                    ax.plot([rx, goal[0]], [ry, goal[1]], 'b--', lw=1)
                    ax.plot(goal[0], goal[1], 'bo')
                if n.plan is not None and n.plan.poses:
                    ax.plot([p.pose.position.x for p in n.plan.poses],
                            [p.pose.position.y for p in n.plan.poses], 'g-', lw=2)
            if frame == 'map' and goal:
                cxm, cym = (rx + goal[0]) / 2, (ry + goal[1]) / 2
                hw = max(VIEW, 0.5 * max(abs(goal[0] - rx), abs(goal[1] - ry)) + 1.5)
                ax.set_xlim(cxm - hw, cxm + hw)
                ax.set_ylim(cym - hw, cym + hw)
            else:
                ax.set_xlim(px - VIEW, px + VIEW)
                ax.set_ylim(py - VIEW, py + VIEW)
            ax.set_aspect('equal')
            ax.grid(alpha=0.3)
        plt.tight_layout()
        png = os.path.join(LOGDIR, 'costmap_snapshot.png')
        plt.savefig(png, dpi=90)
        L.append(f"\npicture: {png}  (cyan=robot footprint, blue dashed=straight line, green=Nav2 plan)")
    except Exception as e:
        L.append(f"(no picture: {e})")

    print("\n".join(L))
    os.makedirs(LOGDIR, exist_ok=True)
    with open(os.path.join(LOGDIR, 'costmap_snapshot.txt'), 'w') as f:
        f.write("\n".join(L) + "\n")
    if goal_name:
        import shutil
        shutil.copy(os.path.join(LOGDIR, 'costmap_snapshot.txt'), os.path.join(LOGDIR, f'snap_{goal_name}.txt'))
        if os.path.exists(os.path.join(LOGDIR, 'costmap_snapshot.png')):
            shutil.copy(os.path.join(LOGDIR, 'costmap_snapshot.png'), os.path.join(LOGDIR, f'snap_{goal_name}.png'))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
