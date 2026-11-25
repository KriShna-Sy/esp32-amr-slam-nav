#!/usr/bin/env python3
"""
map_downsampler: /map (RTAB-Map, 3 cm) -> /map_nav (5 cm) for Nav2's global costmap.

1. Path carving (filter): every pose in RTAB-Map's graph (/mapGraph) is a place the robot's
   body physically was, so occupied specks under its footprint there are noise -> set free.
   Real obstacles are never removed this way (the robot could not have stood on them).
2. Max-pooling 3 cm -> 5 cm: a 5 cm cell is occupied if ANY 3 cm cell in it is occupied.
3. Second carve on the 5 cm grid with the PLANNER's footprint (+footprint_padding + half a cell).
   Max-pooling re-thickens walls by up to one cell, so a doorway the robot really drove through
   could come out narrower than the padded footprint -> Smac Lattice "no valid path".
   Carving the padded footprint at every driven pose guarantees the driven route is plannable.
4. Live carving: the robot's CURRENT pose (map->base_link) is also carved with the planner
   footprint and remembered. Where the robot physically stands is free, so Nav2 can never
   report "Start occupied" because a wall was drawn a few cm too thick.
"""
import copy
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import os
import yaml

KEEPOUT = os.path.expanduser("~/mapping_ws/nav_data/keepout.yaml")

try:
    from rtabmap_msgs.msg import MapGraph
except ImportError:
    MapGraph = None

# robot body (from drive axle), shrunk 3 cm each side so we only clear where it surely was
CARVE = (-0.07, 0.39, 0.27)   # x_min, x_max, |y|
# planner footprint: x -0.10..0.42, |y| 0.30, + footprint_padding 0.04 + 0.03 (outline rasterisation)
CARVE_NAV = (-0.17, 0.49, 0.37)


class MapDownsampler(Node):
    def __init__(self):
        super().__init__('map_downsampler')
        self.res = float(self.declare_parameter('resolution', 0.05).value)
        self.carve = bool(self.declare_parameter('carve_robot_path', True).value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map_nav', latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.poses = []
        if MapGraph is not None and self.carve:
            self.create_subscription(MapGraph, '/mapGraph', self.on_graph, 1)
        elif self.carve:
            self.get_logger().warn("rtabmap_msgs not importable -> no path carving")
        self.map = None
        self.dirty = False
        self.live = bool(self.declare_parameter('carve_live_pose', True).value)
        self.live_poses = []
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        if self.live:
            self.create_timer(0.5, self.on_live_pose)
        self.keepout = []
        self.keepout_mtime = None
        self.create_timer(2.0, self.check_keepout)
        self.check_keepout()
        self.create_timer(1.0, self.process)

    def check_keepout(self):
        """Reload nav_data/keepout.yaml when it changes (written by: ros2 run ... keepout)."""
        try:
            mt = os.path.getmtime(KEEPOUT)
        except OSError:
            mt = None
        if mt == self.keepout_mtime:
            return
        self.keepout_mtime = mt
        zones = []
        if mt is not None:
            try:
                d = yaml.safe_load(open(KEEPOUT)) or {}
                for name, pts in (d.get('zones') or {}).items():
                    if pts and len(pts) >= 3:
                        zones.append((name, np.array(pts, dtype=float)))
            except Exception as e:
                self.get_logger().warn(f"keepout.yaml unreadable: {e}")
        self.keepout = zones
        self.dirty = True
        self.get_logger().info(f"keepout zones: {[z[0] for z in zones] or 'none'}")

    def paint_keepout(self, A, info):
        """Fill every keepout polygon with LETHAL (100). Done LAST so carving never erases it."""
        if not self.keepout:
            return 0
        r = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        h, w = A.shape
        n = 0
        for _, P in self.keepout:
            j0 = max(0, int((P[:, 0].min() - ox) / r) - 1)
            j1 = min(w - 1, int((P[:, 0].max() - ox) / r) + 1)
            i0 = max(0, int((P[:, 1].min() - oy) / r) - 1)
            i1 = min(h - 1, int((P[:, 1].max() - oy) / r) + 1)
            if j1 < j0 or i1 < i0:
                continue
            jj, ii = np.meshgrid(np.arange(j0, j1 + 1), np.arange(i0, i1 + 1))
            px = ox + (jj + 0.5) * r
            py = oy + (ii + 0.5) * r
            inside = np.zeros(px.shape, dtype=bool)
            xs, ys = P[:, 0], P[:, 1]
            k = len(P) - 1
            for m in range(len(P)):                       # even-odd ray casting
                c = ((ys[m] > py) != (ys[k] > py)) & \
                    (px < (xs[k] - xs[m]) * (py - ys[m]) / (ys[k] - ys[m] + 1e-12) + xs[m])
                inside ^= c
                k = m
            A[ii[inside], jj[inside]] = 100
            n += int(inside.sum())
        return n

    def on_live_pose(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = t.transform.translation.x, t.transform.translation.y
        if self.live_poses:
            lx, ly, lyaw = self.live_poses[-1]
            dyaw = abs((yaw - lyaw + math.pi) % (2 * math.pi) - math.pi)
            if math.hypot(x - lx, y - ly) < 0.05 and dyaw < 0.17:
                return
        self.live_poses.append((x, y, yaw))
        if len(self.live_poses) > 5000:
            self.live_poses = self.live_poses[-5000:]
        self.dirty = True

    def on_map(self, m):
        self.map = m
        self.dirty = True

    def on_graph(self, g):
        poses = []
        for p in g.poses:
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            poses.append((p.position.x, p.position.y, yaw))
        if len(poses) != len(self.poses):
            self.poses = poses
            self.dirty = True

    def carve_path(self, A, info, box=CARVE):
        r = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        h, w = A.shape
        xs = np.arange(box[0], box[1] + 1e-9, r / 2)
        ys = np.arange(-box[2], box[2] + 1e-9, r / 2)
        bx, by = np.meshgrid(xs, ys)
        bx, by = bx.ravel(), by.ravel()
        cleared = 0
        for x, y, yaw in self.poses:
            c, s = math.cos(yaw), math.sin(yaw)
            j = ((x + c * bx - s * by - ox) / r).astype(int)
            i = ((y + s * bx + c * by - oy) / r).astype(int)
            ok = (i >= 0) & (i < h) & (j >= 0) & (j < w)
            i, j = i[ok], j[ok]
            occ = A[i, j] > 50
            cleared += int(occ.sum())
            A[i, j] = 0
        return cleared

    def process(self):
        if not self.dirty or self.map is None:
            return
        self.dirty = False
        m = self.map
        r0, w0, h0 = m.info.resolution, m.info.width, m.info.height
        if w0 == 0 or h0 == 0:
            return
        A = np.array(m.data, dtype=np.int16).reshape(h0, w0)
        cleared = self.carve_path(A, m.info) if (self.carve and self.poses) else 0
        if abs(r0 - self.res) < 1e-6:
            out, w1, h1 = A, w0, h0
        else:
            w1 = int(np.ceil(w0 * r0 / self.res))
            h1 = int(np.ceil(h0 * r0 / self.res))
            jx = np.minimum(((np.arange(w0) + 0.5) * r0 / self.res).astype(int), w1 - 1)
            iy = np.minimum(((np.arange(h0) + 0.5) * r0 / self.res).astype(int), h1 - 1)
            out = np.full((h1, w1), -1, dtype=np.int16)
            II, JJ = np.meshgrid(iy, jx, indexing='ij')
            np.maximum.at(out, (II.ravel(), JJ.ravel()), A.ravel())   # occupied > free > unknown
        msg = OccupancyGrid()
        msg.header = m.header
        msg.info = copy.deepcopy(m.info)   # never mutate the stored /map header (crash fix)
        msg.info.resolution = self.res
        msg.info.width, msg.info.height = int(w1), int(h1)
        if self.carve and self.poses:
            cleared += self.carve_path(out, msg.info, CARVE_NAV)
        live_cleared = 0
        if self.live_poses:
            graph = self.poses
            self.poses = self.live_poses
            live_cleared = self.carve_path(out, msg.info, CARVE_NAV)
            self.poses = graph
        ko = self.paint_keepout(out, msg.info)
        msg.data = out.astype(np.int8).ravel().tolist()
        self.pub.publish(msg)
        self.get_logger().info(f"/map {w0}x{h0}@{r0:.3f} -> /map_nav {w1}x{h1}@{self.res:.3f} | "
                               f"path carving: {len(self.poses)} poses, {cleared} noise cells cleared | "
                               f"live: {len(self.live_poses)} poses, {live_cleared} cells | keepout: {len(self.keepout)} zones, {ko} cells", throttle_duration_sec=10.0)


def main():
    rclpy.init()
    n = MapDownsampler()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
