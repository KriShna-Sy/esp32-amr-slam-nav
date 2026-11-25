#!/usr/bin/env python3
"""
landmark_manager.py
Manage and navigate to named landmarks/benchmarks in the map:
  - save <name>    : Save current robot pose (map -> base_link) as a landmark
  - list           : Display all saved landmarks
  - delete <name>  : Remove a saved landmark
  - goto <name>    : Plan path and drive autonomously to landmark using Nav2
"""
import os
import sys
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from rtabmap_msgs.msg import Info
import tf_transformations
try:
    from . import robot_logger
except ImportError:
    import robot_logger

# ONLY the fresh navigation data folder is used (map + landmarks made together).
NAV_DATA_DIR = os.path.expanduser('~/mapping_ws/nav_data')
DEFAULT_LANDMARKS_FILE = os.path.join(NAV_DATA_DIR, 'landmarks.yaml')


def yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# Robot body (base_link = drive axle): front +0.42, rear -0.10, half-width 0.30
BODY = (-0.10, 0.42, 0.30)
ZONE_MARGIN = 0.15      # default; override: goto <name> --zone 0.075  | zone = body outline enlarged by 15 % on every side
ZONE_HOLD_S = 0.6       # must stay inside this long (ignores single localization jumps)


def body_in_zone(pose, target, margin=ZONE_MARGIN):
    """True if ALL 4 corners of the robot body at 'pose' lie inside the target body
    outline enlarged by margin*length / margin*width on every side.
    Returns (inside, dx_fwd, dy_side, dyaw_deg) in the landmark frame."""
    xr, xf, hw = BODY
    L, W = xf - xr, 2 * hw
    zx0, zx1 = xr - margin * L, xf + margin * L
    zhw = hw + margin * W
    ty = math.radians(target['yaw_deg'])
    ry = math.radians(pose['yaw_deg'])
    ct, st = math.cos(ty), math.sin(ty)
    inside = True
    for a, b in ((xf, hw), (xf, -hw), (xr, -hw), (xr, hw)):
        wx = pose['x'] + math.cos(ry) * a - math.sin(ry) * b
        wy = pose['y'] + math.sin(ry) * a + math.cos(ry) * b
        ex, ey = wx - target['x'], wy - target['y']
        lx, ly = ct * ex + st * ey, -st * ex + ct * ey
        if not (zx0 <= lx <= zx1 and -zhw <= ly <= zhw):
            inside = False
    ex, ey = pose['x'] - target['x'], pose['y'] - target['y']
    dyaw = (pose['yaw_deg'] - target['yaw_deg'] + 180.0) % 360.0 - 180.0
    return inside, ct * ex + st * ey, -st * ex + ct * ey, dyaw


class LandmarkManager(Node):
    def __init__(self, filepath=DEFAULT_LANDMARKS_FILE):
        super().__init__('landmark_manager')
        self.filepath = filepath
        self.active_log_session = None
        self.last_fb_log_time = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # fine approach goes THROUGH the collision monitor (its input topic), never around it
        self.fine_pub = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)
        self.target_marker_pub = self.create_publisher(MarkerArray, '/target_landmark_marker', 10)
        self.all_markers_pub = self.create_publisher(MarkerArray, '/landmark_markers', 10)
        self.current_target_name = None
        self.current_target_pose = None

        # RTAB-Map live visual localization state (subscribing to both root and namespaced topics)
        self.latest_loop_closure_id = 0
        self.latest_ref_id = 0
        self.total_loop_closures = 0
        self.last_info_time = 0.0
        self.latest_loc_pose = None
        self.last_loc_pose_time = 0.0
        self.last_match_time = 0.0
        self.nav_match_count = 0

        self.create_subscription(Info, '/info', self._rtabmap_info_cb, 10)
        self.create_subscription(Info, '/rtabmap/info', self._rtabmap_info_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/localization_pose', self._rtabmap_loc_pose_cb, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/rtabmap/localization_pose', self._rtabmap_loc_pose_cb, 10
        )

        # Publish all landmarks on map
        self.publish_all_landmark_markers()

    def _rtabmap_info_cb(self, msg: Info):
        self.latest_ref_id = msg.ref_id
        self.last_info_time = time.time()
        if msg.loop_closure_id > 0:
            self.latest_loop_closure_id = msg.loop_closure_id
            self.total_loop_closures += 1
            self.last_match_time = time.time()
            self.nav_match_count += 1
        elif getattr(msg, 'proximity_detection_id', 0) > 0:
            self.latest_loop_closure_id = msg.proximity_detection_id
            self.last_match_time = time.time()

    def wait_fresh_match(self, timeout):
        """True only if RTAB-Map matches the camera to the map AFTER this call started
        (old/latched messages and odometry-only poses do not count)."""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_match_time > t0:
                return True
        return False

    def match_age(self):
        t = max(self.last_match_time, self.last_loc_pose_time)
        return (time.time() - t) if t > 0 else None

    def averaged_fix(self, duration=4.0, min_fixes=3):
        """Robot standing still: collect fresh visual fixes for 'duration' s and return the
        MEDIAN pose {'x','y','yaw_deg','n'} (one bad match cannot move it), or None."""
        self.fix_buf = []
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        fixes = self.fix_buf
        self.fix_buf = None
        if len(fixes) < min_fixes:
            return None
        xs = sorted(f[0] for f in fixes)
        ys = sorted(f[1] for f in fixes)
        # yaw median via unwrapped angles around the first fix
        y0 = fixes[0][2]
        yaws = sorted(y0 + (f[2] - y0 + math.pi) % (2 * math.pi) - math.pi for f in fixes)
        m = len(fixes) // 2
        return {'x': xs[m], 'y': ys[m], 'yaw_deg': math.degrees(yaws[m]), 'n': len(fixes),
                'spread_cm': 100 * math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])}

    def _rtabmap_loc_pose_cb(self, msg: PoseWithCovarianceStamped):
        # only count poses that come with an actual visual match (Info loop/proximity id)
        if getattr(self, 'fix_buf', None) is not None and time.time() - self.last_match_time < 1.2:
            p = msg.pose.pose
            self.fix_buf.append((p.position.x, p.position.y, yaw_from_quaternion(p.orientation)))
        self.latest_loc_pose = msg
        self.last_loc_pose_time = time.time()
        self.nav_match_count += 1

    def active_map(self):
        try:
            return open(os.path.join(os.path.dirname(self.filepath), 'PROFILE')).read().strip()
        except Exception:
            return '?'

    def load_landmarks(self):
        if not os.path.exists(self.filepath):
            return {}
        try:
            with open(self.filepath, 'r') as f:
                data = yaml.safe_load(f)
                return data if data else {}
        except Exception as e:
            self.get_logger().error(f"Failed to load landmarks: {e}")
            return {}

    def save_landmarks(self, data):
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        with open(self.filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.publish_all_landmark_markers()

    def publish_all_landmark_markers(self):
        """Publish all known landmarks as 3D ground discs and text labels in RViz."""
        landmarks = self.load_landmarks()
        if not landmarks:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (name, p) in enumerate(landmarks.items()):
            # 1. Base cylinder disc on the ground
            m_disc = Marker()
            m_disc.header.frame_id = 'map'
            m_disc.header.stamp = now
            m_disc.ns = 'landmarks_pins'
            m_disc.id = i * 2
            m_disc.type = Marker.CYLINDER
            m_disc.action = Marker.ADD
            m_disc.pose.position.x = float(p['x'])
            m_disc.pose.position.y = float(p['y'])
            m_disc.pose.position.z = 0.03
            m_disc.scale.x = 0.35
            m_disc.scale.y = 0.35
            m_disc.scale.z = 0.06
            m_disc.color.r = 0.0
            m_disc.color.g = 0.8
            m_disc.color.b = 0.95
            m_disc.color.a = 0.85
            ma.markers.append(m_disc)

            # 2. Text label above the landmark
            m_text = Marker()
            m_text.header.frame_id = 'map'
            m_text.header.stamp = now
            m_text.ns = 'landmarks_labels'
            m_text.id = i * 2 + 1
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = float(p['x'])
            m_text.pose.position.y = float(p['y'])
            m_text.pose.position.z = 0.35
            m_text.scale.z = 0.22
            m_text.color.r = 1.0
            m_text.color.g = 1.0
            m_text.color.b = 1.0
            m_text.color.a = 0.95
            m_text.text = name
            ma.markers.append(m_text)

        self.all_markers_pub.publish(ma)

    def publish_target_marker(self, name, p):
        """Publish a prominent 3D flagpole, target ring, and label at the active navigation goal."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. Target flagpole cylinder
        m_pole = Marker()
        m_pole.header.frame_id = 'map'
        m_pole.header.stamp = now
        m_pole.ns = 'target_pin'
        m_pole.id = 100
        m_pole.type = Marker.CYLINDER
        m_pole.action = Marker.ADD
        m_pole.pose.position.x = float(p['x'])
        m_pole.pose.position.y = float(p['y'])
        m_pole.pose.position.z = 0.50
        m_pole.scale.x = 0.08
        m_pole.scale.y = 0.08
        m_pole.scale.z = 1.00
        m_pole.color.r = 1.0
        m_pole.color.g = 0.84
        m_pole.color.b = 0.0
        m_pole.color.a = 0.95
        ma.markers.append(m_pole)

        # 2. Target ground glowing ring
        m_ring = Marker()
        m_ring.header.frame_id = 'map'
        m_ring.header.stamp = now
        m_ring.ns = 'target_ring'
        m_ring.id = 101
        m_ring.type = Marker.CYLINDER
        m_ring.action = Marker.ADD
        m_ring.pose.position.x = float(p['x'])
        m_ring.pose.position.y = float(p['y'])
        m_ring.pose.position.z = 0.02
        m_ring.scale.x = 0.70
        m_ring.scale.y = 0.70
        m_ring.scale.z = 0.04
        m_ring.color.r = 1.0
        m_ring.color.g = 0.2
        m_ring.color.b = 0.2
        m_ring.color.a = 0.80
        ma.markers.append(m_ring)

        # 3. Target text banner
        m_text = Marker()
        m_text.header.frame_id = 'map'
        m_text.header.stamp = now
        m_text.ns = 'target_label'
        m_text.id = 102
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = float(p['x'])
        m_text.pose.position.y = float(p['y'])
        m_text.pose.position.z = 1.15
        m_text.scale.z = 0.32
        m_text.color.r = 1.0
        m_text.color.g = 0.9
        m_text.color.b = 0.0
        m_text.color.a = 1.0
        m_text.text = f">>> GOAL: {name} <<<"
        ma.markers.append(m_text)

        # 4. Target heading arrow
        m_arrow = Marker()
        m_arrow.header.frame_id = 'map'
        m_arrow.header.stamp = now
        m_arrow.ns = 'target_arrow'
        m_arrow.id = 103
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.pose.position.x = float(p['x'])
        m_arrow.pose.position.y = float(p['y'])
        m_arrow.pose.position.z = 0.15
        m_arrow.pose.orientation.x = float(p['qx'])
        m_arrow.pose.orientation.y = float(p['qy'])
        m_arrow.pose.orientation.z = float(p['qz'])
        m_arrow.pose.orientation.w = float(p['qw'])
        m_arrow.scale.x = 0.60
        m_arrow.scale.y = 0.10
        m_arrow.scale.z = 0.10
        m_arrow.color.r = 1.0
        m_arrow.color.g = 0.15
        m_arrow.color.b = 0.15
        m_arrow.color.a = 1.0
        ma.markers.append(m_arrow)

        self.target_marker_pub.publish(ma)

    def get_current_pose(self, timeout_sec=5.0):
        self.get_logger().info("Looking up current robot pose in 'map' frame...")
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time()
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                yaw = yaw_from_quaternion(r)
                return {
                    'x': float(t.x),
                    'y': float(t.y),
                    'z': float(t.z),
                    'qx': float(r.x),
                    'qy': float(r.y),
                    'qz': float(r.z),
                    'qw': float(r.w),
                    'yaw_deg': float(math.degrees(yaw)),
                }
            except Exception:
                continue
        return None

    def save_current_location(self, name, force=False, unverified=False):
        existing = self.load_landmarks().get(name)
        if existing is not None and not force:
            print(f"\n[REFUSED] '{name}' already exists at X={existing['x']:.3f}, "
                  f"Y={existing['y']:.3f}, Heading={existing.get('yaw_deg', 0.0):.1f} deg")
            print(f"  Overwrite:  landmark_manager save {name} --force")
            print(f"  Or use a new name, e.g.  landmark_manager save {name}_2\n")
            return False
        if not unverified:
            print("[CHECK] Waiting up to 8 s for a fresh visual match with the map...")
            if not self.wait_fresh_match(8.0):
                if self.last_info_time == 0.0 and self.last_loc_pose_time == 0.0:
                    print("\n[ERROR] No messages from RTAB-Map at all - localization is not running.")
                    print("  Start it first:  ros2 launch esp32_odom_bridge autonomous_bringup.launch.py\n")
                    return False
                age = self.match_age()
                print(f"\n[REFUSED] No visual match with the map right now "
                      f"(last match: {'never' if age is None else f'{age:.0f} s ago'}).")
                print("  The position is coming from wheel odometry only and may have drifted,")
                print("  so this landmark could be saved in the wrong place.")
                print("  Turn the robot a little so the camera sees walls / furniture, then retry.")
                print(f"  (Save anyway: landmark_manager save {name} --unverified)\n")
                return False
        pose = self.get_current_pose()
        if pose is None:
            print("\n[ERROR] Could not get robot pose from TF ('map' -> 'base_link').")
            print("Make sure localization (localization.launch.py) is running and localized!\n")
            return False

        landmarks = self.load_landmarks()
        landmarks[name] = pose
        self.save_landmarks(landmarks)
        robot_logger.log_simple_event('landmark', f"Saved landmark '{name}' at X={pose['x']:.3f}, Y={pose['y']:.3f}, Heading={pose['yaw_deg']:.1f}°")
        print(f"\n[SUCCESS] Landmark '{name}' saved:")
        print(f"  X: {pose['x']:.3f} m,  Y: {pose['y']:.3f} m,  Heading: {pose['yaw_deg']:.1f}°\n")
        return True

    def trigger_global_relocalization(self):
        """Send empty request to /rtabmap/global_relocalization to search all nodes."""
        try:
            reloc_client = self.create_client(Empty, '/rtabmap/global_relocalization')
            if reloc_client.wait_for_service(timeout_sec=0.5):
                reloc_client.call_async(Empty.Request())
        except Exception:
            pass

    def show_status(self, wait_mode=False):
        print("\n" + "=" * 65)
        print("     ROBOT LOCALIZATION & KIDNAPPED POSITION VERIFIER")
        print("=" * 65)
        print("[STATUS] Checking live RTAB-Map visual relocalization...")

        # Proactively trigger global relocalization search across the database
        self.trigger_global_relocalization()

        start_time = time.time()
        timeout = 60.0 if wait_mode else 3.5
        last_info_print = 0.0

        while rclpy.ok() and (time.time() - start_time < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)

            # Confirmed ONLY by a visual match that arrives after this check started
            is_confirmed = self.last_match_time > start_time

            if is_confirmed:
                pose = self.get_current_pose(timeout_sec=1.5)
                if pose:
                    match_id = (f"Node #{self.latest_loop_closure_id}" if self.last_match_time > start_time
                                else "localization pose")
                    print("\n" + "*" * 65)
                    print(f" >>> [STATUS] CONFIRMED LOCALIZED! Fresh visual match ({match_id})")
                    print("*" * 65)
                    print(f"  True Live Robot Position in Map:")
                    print(f"    X       = {pose['x']:>7.3f} m")
                    print(f"    Y       = {pose['y']:>7.3f} m")
                    print(f"    Heading = {pose['yaw_deg']:>7.1f}°")
                    landmarks = self.load_landmarks()
                    if landmarks:
                        closest_name, closest_dist = None, float('inf')
                        for name, p in landmarks.items():
                            d = math.hypot(pose['x'] - p['x'], pose['y'] - p['y'])
                            if d < closest_dist:
                                closest_dist = d
                                closest_name = name
                        if closest_name:
                            print(f"  Nearest Reference Landmark: '{closest_name}' ({closest_dist:.2f} m away)\n")
                    return True

            if wait_mode and (time.time() - last_info_print > 1.5):
                last_info_print = time.time()
                elapsed = int(time.time() - start_time)
                print(f"  [SEARCHING] ({elapsed}s elapsed) Scanning database... Face camera towards mapped features.", flush=True)

        # If we reach here, no visual match was confirmed in the time window
        print("\n" + "-" * 65)
        print(" [STATUS] SEARCHING / UNVERIFIED: RTAB-Map has NOT visually matched yet.")
        print("-" * 65)
        print("  Reason: The robot is at an unconfirmed position (Kidnapped).")
        print("          No fresh visual match with the map in this time window.")
        print("          The position shown by TF is from wheel odometry only (may have drifted).")
        print("\n  WHAT TO DO:")
        print("  1. Rotate or slowly move the robot so the camera faces a mapped feature/wall.")
        print("  2. To monitor continuously until locked, run:")
        print("       ros2 run esp32_odom_bridge landmark_manager whereami --wait\n")
        return False

    def list_landmarks(self):
        landmarks = self.load_landmarks()
        if not landmarks:
            print(f"\nNo landmarks saved yet. (File: {self.filepath})")
            print("To save one: ros2 run esp32_odom_bridge landmark_manager save <name>\n")
            return

        print("\n" + "=" * 65)
        print(f"{'NAME':<20} | {'X (m)':<8} | {'Y (m)':<8} | {'HEADING':<10}")
        print("=" * 65)
        for name, p in sorted(landmarks.items()):
            print(f"{name:<20} | {p['x']:>7.3f}  | {p['y']:>7.3f}  | {p['yaw_deg']:>7.1f}°")
        print("=" * 65 + "\n")

    def delete_landmark(self, name):
        landmarks = self.load_landmarks()
        if name not in landmarks:
            print(f"\n[ERROR] Landmark '{name}' does not exist.\n")
            return False
        del landmarks[name]
        self.save_landmarks(landmarks)
        robot_logger.log_simple_event('landmark', f"Deleted landmark '{name}'")
        print(f"\n[SUCCESS] Landmark '{name}' deleted.\n")
        return True

    def set_initial_pose(self, name='start'):
        """Publish a landmark as /initialpose so RTAB-Map/Nav2 know where the robot is."""
        landmarks = self.load_landmarks()
        if name not in landmarks:
            print(f"\n[ERROR] Landmark '{name}' not found. Available: {list(landmarks.keys())}\n")
            return False

        p = landmarks[name]
        robot_logger.log_simple_event('localize', f"Initial pose set to '{name}' (X={p['x']:.3f}, Y={p['y']:.3f}, Heading={p['yaw_deg']:.1f}°)")
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = p['x']
        msg.pose.pose.position.y = p['y']
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = p['qx']
        msg.pose.pose.orientation.y = p['qy']
        msg.pose.pose.orientation.z = p['qz']
        msg.pose.pose.orientation.w = p['qw']
        # Low covariance = high confidence in this starting pose
        msg.pose.covariance[0]  = 0.10   # x variance
        msg.pose.covariance[7]  = 0.10   # y variance
        msg.pose.covariance[35] = 0.05   # yaw variance

        print(f"\n[LOCALIZE] Setting initial pose to landmark '{name}':")
        print(f"  X={p['x']:.3f} m, Y={p['y']:.3f} m, Heading={p['yaw_deg']:.1f}°")
        print("[LOCALIZE] Publishing /initialpose ... (publishing 3x to ensure delivery)")
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.3)

        print("[LOCALIZE] Done! Published initial pose to RTAB-Map.")
        # Trigger RTAB-Map global relocalization if service available
        try:
            reloc_client = self.create_client(Empty, '/rtabmap/global_relocalization')
            if reloc_client.wait_for_service(timeout_sec=0.5):
                reloc_client.call_async(Empty.Request())
        except Exception:
            pass

        # Trigger Nav2 lifecycle activation if waiting
        self.ensure_nav2_active(timeout_sec=3.0)

        # Confirm whether map -> base_link TF is actively localized
        print("[LOCALIZE] Verifying active robot pose in 'map' frame...")
        confirmed_pose = None
        for _ in range(8):
            rclpy.spin_once(self, timeout_sec=0.2)
            current = self.get_current_pose(timeout_sec=0.2)
            if current:
                confirmed_pose = current
                break
            time.sleep(0.2)

        if confirmed_pose:
            dx = confirmed_pose['x'] - p['x']
            dy = confirmed_pose['y'] - p['y']
            dist_err = math.hypot(dx, dy)
            print(f"[LOCALIZE] Live Map Pose: X={confirmed_pose['x']:.3f} m, Y={confirmed_pose['y']:.3f} m, Heading={confirmed_pose['yaw_deg']:.1f}°")
            if dist_err < 0.6:
                print(f"[LOCALIZE] SUCCESS: Robot confirmed localized at '{name}' (position error: {dist_err:.2f} m). Ready to navigate!\n")
            else:
                print(f"[LOCALIZE] Note: Current pose offset is {dist_err:.2f} m from '{name}'. Ready to navigate.\n")
        else:
            print("[LOCALIZE] Note: map -> base_link transform still synchronizing. Wait 2s before sending navigation goal.\n")
        return True

    def ensure_nav2_active(self, timeout_sec=5.0):
        """If Nav2 lifecycle nodes are inactive, automatically trigger STARTUP."""
        # If action server is already responding, Nav2 nodes are already active!
        if self.nav_client.wait_for_server(timeout_sec=0.5):
            return True

        client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        if not client.wait_for_service(timeout_sec=1.5):
            return False

        req = ManageLifecycleNodes.Request()
        req.command = 0  # 0 = STARTUP (configure and activate all Nav2 nodes)
        print("[NAV2] Triggering Nav2 Lifecycle Manager STARTUP...")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.done():
            res = future.result()
            if res and res.success:
                print("[NAV2] Nav2 Stack is now ACTIVE and READY!")
                return True
        return False

    # ------------------------------------------------------------------
    # Fine approach: last few cm without Nav2 (Nav2's goal checker declares
    # success up to xy_goal_tolerance short, and a re-sent goal 12 cm away
    # "arrives" instantly). Slow rotate-drive-rotate on the live map pose.
    # ------------------------------------------------------------------
    def _tf_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quaternion(t.transform.rotation))

    def _odom_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return yaw_from_quaternion(t.transform.rotation)

    def rotate_by(self, dyaw, tol_deg=1.5, timeout=10.0):
        """Turn in place by dyaw (rad) measured on ODOM yaw (wheel+gyro: smooth, no map
        corrections mid-turn). Used for the last heading touch-up from the visual median."""
        wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
        y0 = None
        t0 = time.time()
        while y0 is None and time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            y0 = self._odom_yaw()
        if y0 is None:
            return
        goal = wrap(y0 + dyaw)
        # stop early by the coast at min pivot speed (0.18 rad/s x ~0.3 s latency ~ 3 deg)
        lead = math.radians(2.5)
        try:
            while rclpy.ok() and time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.03)
                y = self._odom_yaw()
                if y is None:
                    continue
                e = wrap(goal - y)
                if abs(e) < max(math.radians(tol_deg), lead if abs(e) < math.radians(6) else 0):
                    break
                self._fine_cmd(0.0, math.copysign(0.18, e))
        finally:
            for _ in range(3):
                self._fine_cmd(0.0, 0.0)
                time.sleep(0.05)
        time.sleep(0.6)

    def _fine_cmd(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.fine_pub.publish(m)

    def fine_approach(self, target, pos_tol=0.015, yaw_tol_deg=3.0, timeout=30.0):
        wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
        gx, gy, gyaw = target['x'], target['y'], math.radians(target['yaw_deg'])
        t0 = time.time()
        backups = 0
        try:
            # phase 1: position
            while rclpy.ok() and time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.05)
                p = self._tf_pose()
                if p is None:
                    continue
                x, y, yaw = p
                ex, ey = gx - x, gy - y
                dist = math.hypot(ex, ey)
                if dist < pos_tol:
                    break
                along = ex * math.cos(yaw) + ey * math.sin(yaw)
                if dist < 0.06:
                    lateral = -ex * math.sin(yaw) + ey * math.cos(yaw)
                    if abs(lateral) > 0.02 and backups < 1:
                        # sideways offset can't be fixed by creeping: back off ~12 cm, re-approach
                        backups += 1
                        t2 = time.time()
                        while rclpy.ok() and time.time() - t2 < 3.0:
                            self._fine_cmd(-0.04, 0.0)
                            rclpy.spin_once(self, timeout_sec=0.1)
                        self._fine_cmd(0.0, 0.0)
                        time.sleep(0.4)
                        continue
                    # too close for a stable bearing: only creep straight along the body axis.
                    # stop ~1 cm early: 0.04 m/s x ~0.3 s command latency = 1.2 cm coast
                    if abs(along) < 0.012:
                        break
                    self._fine_cmd(math.copysign(0.04, along), 0.0)
                    continue
                e = wrap(math.atan2(ey, ex) - yaw)
                rev = abs(e) > math.pi / 2 and dist < 0.35       # target just behind -> back up
                if rev:
                    e = wrap(e - math.pi)
                if abs(e) > math.radians(8):
                    self._fine_cmd(0.0, math.copysign(min(0.30, max(0.18, abs(e))), e))
                else:
                    v = min(0.07, max(0.04, 0.5 * dist))
                    self._fine_cmd(-v if rev else v, max(-0.15, min(0.15, e)))
            self._fine_cmd(0.0, 0.0)
            time.sleep(0.4)
            # phase 2: heading (bridge minimum pivot speed is 0.18 rad/s -> bang-bang + settle)
            for _ in range(6):
                rclpy.spin_once(self, timeout_sec=0.05)
                p = self._tf_pose()
                if p is None:
                    continue
                dyaw = wrap(gyaw - p[2])
                if abs(dyaw) < math.radians(yaw_tol_deg):
                    break
                t1 = time.time()
                while rclpy.ok() and time.time() - t1 < 8.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    p = self._tf_pose()
                    if p is None:
                        continue
                    dyaw = wrap(gyaw - p[2])
                    if abs(dyaw) < math.radians(yaw_tol_deg * 0.6):
                        break
                    self._fine_cmd(0.0, math.copysign(0.18 if abs(dyaw) < 0.35 else 0.25, dyaw))
                self._fine_cmd(0.0, 0.0)
                time.sleep(0.6)
        finally:
            for _ in range(3):
                self._fine_cmd(0.0, 0.0)
                time.sleep(0.05)

    def navigate_to(self, name, unverified=False, _attempt=0, zone=None):
        zone = ZONE_MARGIN if zone is None else zone
        landmarks = self.load_landmarks()
        if name not in landmarks:
            print(f"\n[ERROR] Unknown landmark '{name}'. Use 'list' to see available benchmarks.\n")
            robot_logger.log_simple_event('navigation', f"Requested navigation to unknown landmark '{name}'", level="ERROR")
            return False

        target = landmarks[name]
        print(f"\n[NAV2] Preparing path planning to '{name}':")
        print(f"  Target: X={target['x']:.3f} m, Y={target['y']:.3f} m, Heading={target['yaw_deg']:.1f}°")

        # Start persistent, timestamped logging session
        session_meta = {
            'Target Name': name,
            'Target Pose': f"X={target['x']:.3f} m, Y={target['y']:.3f} m, Yaw={target['yaw_deg']:.1f}°",
        }

        # 0. Pre-flight Check: robot must be localized (map -> base_link TF available)
        current_pose = self.get_current_pose(timeout_sec=2.0)

        if current_pose:
            session_meta['Start Pose'] = f"X={current_pose['x']:.3f} m, Y={current_pose['y']:.3f} m, Yaw={current_pose['yaw_deg']:.1f}°"
            initial_dist = math.hypot(target['x'] - current_pose['x'], target['y'] - current_pose['y'])
            session_meta['Initial Dist'] = f"{initial_dist:.2f} m"
        else:
            session_meta['Start Pose'] = "UNKNOWN (TF unavailable)"

        session = robot_logger.start_session('navigation', f"Autonomous Drive to '{name}'", session_meta)
        self.active_log_session = session

        if current_pose is None:
            # Do NOT fake a pose here: injecting 'start' when the robot is elsewhere makes
            # Nav2 drive on a wrong map pose straight into walls.
            if current_pose:
                msg = f"Pose (X={current_pose['x']:.2f}, Y={current_pose['y']:.2f}) is outside map bounds"
            else:
                msg = "Robot pose in 'map' frame not available (not localized yet)"
            print(f"\n[PRE-FLIGHT ABORT] {msg}.")
            print("  Fix: drive a little with teleop in view of mapped areas until RTAB-Map relocalizes,")
            print("  or, if the robot is physically AT a landmark:  landmark_manager setpose <name>\n")
            session.error(msg, debug_hint="Localize first (whereami --wait / setpose <landmark>).")
            session.finish(success=False, summary="Not localized - navigation refused")
            self.active_log_session = None
            return False

        # Pre-flight: the map pose must come from a FRESH camera match, not from the pose
        # RTAB-Map restored from the database at startup or from wheel odometry alone.
        if _attempt == 0 and not unverified:
            print("[CHECK] Need a visual match in the last 30 s (else wait 15 s) with the map...")
            recent = self.last_match_time > 0 and time.time() - self.last_match_time < 30.0
            if not recent and not self.wait_fresh_match(15.0):
                age = self.match_age()
                print(f"\n[PRE-FLIGHT ABORT] No fresh visual match (last match: "
                      f"{'never' if self.last_match_time <= 0 else f'{time.time() - self.last_match_time:.0f} s ago'}).")
                print("  The map pose may be the one restored from the database or wheel-odometry drift.")
                print("  Fix: turn the robot so the camera sees mapped walls/furniture, run")
                print("       landmark_manager whereami --wait   until CONFIRMED, then goto again.")
                print("  (Drive anyway: landmark_manager goto <name> --unverified)\n")
                session.error("No fresh visual match before start", debug_hint="whereami --wait first")
                session.finish(success=False, summary="Not visually localized - navigation refused")
                self.active_log_session = None
                return False
            current_pose = self.get_current_pose(timeout_sec=1.0) or current_pose

        if current_pose:
            dist = math.hypot(target['x'] - current_pose['x'], target['y'] - current_pose['y'])
            print(f"[NAV2] Current Robot Pose: X={current_pose['x']:.3f} m, Y={current_pose['y']:.3f} m | Distance to '{name}': {dist:.2f} m")
            session.step("Pre-flight Pose Verification", status="OK", detail=f"Robot at ({current_pose['x']:.2f}, {current_pose['y']:.2f}), distance: {dist:.2f}m")
        else:
            session.warn("Could not determine starting pose in map frame prior to driving.")

        # 1. If Nav2 action server is not ready, proactively start up the lifecycle manager
        session.step("Connect to Nav2 Action Server", status="IN_PROGRESS")
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            print("[NAV2] Nav2 action server not ready yet. Activating stack via lifecycle manager...")
            session.log("INFO", "Nav2 action server not active. Calling lifecycle manager...")
            self.ensure_nav2_active(timeout_sec=6.0)

        print("[NAV2] Waiting for Nav2 'navigate_to_pose' action server...")
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            print("\n[ERROR] Nav2 action server is not running!")
            print("Start Nav2 first in another terminal:")
            print("  ros2 launch esp32_odom_bridge autonomous_bringup.launch.py\n")
            session.error("Nav2 action server unreachable", debug_hint="Verify nav2.launch.py is running and lifecycle nodes are ACTIVE.")
            session.finish(success=False, summary="Nav2 action server offline")
            print(f"[LOGGER] Failure logged to: {session.session_file}")
            print("[LOGGER] Check with: ros2 run esp32_odom_bridge view_logs --last\n")
            return False
        session.step("Connect to Nav2 Action Server", status="OK")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = rclpy.time.Time().to_msg()
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = target['qx']
        goal_msg.pose.pose.orientation.y = target['qy']
        goal_msg.pose.pose.orientation.z = target['qz']
        goal_msg.pose.pose.orientation.w = target['qw']

        self.current_target_name = name
        self.current_target_pose = target

        # Publish RViz markers
        self.goal_pub.publish(goal_msg.pose)
        self.publish_target_marker(name, target)
        self.publish_all_landmark_markers()

        print("[NAV2] Sending goal to Nav2...")
        session.step("Send Goal to Nav2", status="SENT")
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            print("[ERROR] Goal was rejected by Nav2 (target may be inside an obstacle or out of bounds)!")
            session.error(f"Goal to '{name}' was rejected by Nav2",
                          debug_hint=f"Target ({target['x']:.2f}, {target['y']:.2f}) may overlap a costmap obstacle or lie outside global costmap boundaries.")
            session.finish(success=False, summary="Goal rejected by Nav2")
            print(f"[LOGGER] Failure logged to: {session.session_file}")
            print("[LOGGER] Quick debug check: ros2 run esp32_odom_bridge view_logs --last\n")
            return False

        print(f"[NAV2] Goal accepted! Nav2 planned path to '{name}'. Driving...")
        self.nav_match_count = 0
        nav_t0 = time.time()
        session.step("Plan Path & Drive", status="ACCEPTED", detail="Global planner found path. Autonomous execution underway.")
        result_future = goal_handle.get_result_async()
        # Zone stop: as soon as the whole body sits inside the target zone (held for
        # ZONE_HOLD_S), cancel Nav2 and stay stopped - no more fine-tuning / hunting.
        zone_since = None
        zone_stop = False
        last_check = 0.0
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            if now - last_check < 0.1:
                continue
            last_check = now
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except Exception:
                continue
            t, r = tf.transform.translation, tf.transform.rotation
            pose = {'x': t.x, 'y': t.y, 'yaw_deg': math.degrees(yaw_from_quaternion(r))}
            inside, _, _, _ = body_in_zone(pose, target, zone)
            if inside:
                zone_since = zone_since or now
                if now - zone_since >= ZONE_HOLD_S:
                    zone_stop = True
                    print(f"\n[ZONE] Robot body is inside the target zone (+{zone*100:.1f} %) -> stopping here.")
                    session.log("INFO", f"ZONE STOP triggered (margin {zone*100:.1f} %) at "
                                f"({pose['x']:.3f}, {pose['y']:.3f}, {pose['yaw_deg']:.1f} deg)")
                    cf = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cf, timeout_sec=3.0)
                    break
            else:
                zone_since = None
        if not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        result = result_future.result()

        status = 4 if zone_stop else (result.status if result else 6)
        # STATUS_SUCCEEDED = 4 in action_msgs/msg/GoalStatus
        if status == 4:
            drive_s = time.time() - nav_t0
            age = self.match_age()
            print(f"\n[NAV2] Nav2 reports arrival at '{name}' ({drive_s:.0f} s, "
                  f"{self.nav_match_count} visual matches during the drive, last "
                  f"{'never' if age is None else f'{age:.0f} s ago'}).")
            # Nav2 only knows the MAP pose. Verify it with a fresh camera match while stopped.
            print("[VERIFY] Robot stopped - waiting up to 10 s for a fresh visual match...")
            verified = self.wait_fresh_match(10.0)
            if verified:
                time.sleep(0.5)
                for _ in range(5):
                    rclpy.spin_once(self, timeout_sec=0.1)
            arrived = self.get_current_pose(timeout_sec=1.0)
            print("[VERIFY] Averaging visual fixes for 4 s (median)...")
            avg = self.averaged_fix(5.0, min_fixes=2)
            if avg:
                print(f"[VERIFY] {avg['n']} fixes, spread {avg['spread_cm']:.1f} cm -> using median pose")
                session.log("INFO", f"Averaged fix: n={avg['n']}, spread {avg['spread_cm']:.1f} cm")
                arrived = avg
            else:
                print("[VERIFY] <3 fixes while stopped - using current TF pose")
            err = math.hypot(arrived['x'] - target['x'], arrived['y'] - target['y']) if arrived else None
            yerr = None
            if arrived:
                yerr = (arrived['yaw_deg'] - target['yaw_deg'] + 180.0) % 360.0 - 180.0
            if not verified and unverified:
                print("[DEMO] No visual match at the goal - fine approach on odometry/TF pose (--unverified)")
            elif not verified:
                print("\n[UNVERIFIED] No visual match at the goal: the position is wheel-odometry only.")
                print(f"  Map says {err:.2f} m from '{name}', but the real error can be much larger.")
                session.finish(success=False, summary=f"Arrival at '{name}' NOT visually verified "
                               f"(map err {err:.2f} m, {self.nav_match_count} matches en route)",
                               status_code=status)
                print(f"\n[LOGGER] Mission log saved to: {session.session_file}\n")
                return False
            zin, zdx, zdy, zdyaw = body_in_zone(arrived, target, zone) if arrived else (False, 0, 0, 0)
            session.log("INFO", f"ZONE CHECK margin {zone*100:.1f} %: fwd {zdx*100:+.1f} cm, side {zdy*100:+.1f} cm, "
                        f"yaw {zdyaw:+.1f} deg, zone_stop={zone_stop}, inside={zin}")
            print(f"[ZONE] forward {zdx*100:+.1f} cm, sideways {zdy*100:+.1f} cm, heading {zdyaw:+.1f} deg "
                  f"-> {'INSIDE' if zin else 'OUTSIDE'} zone (body +{zone*100:.1f} %)")
            for k in range(2):
                if err is None or err > 0.40:
                    break
                if zin and err <= 0.03 and abs(yerr) <= 3.0:
                    break                                   # already within 3 cm / 3 deg
                print(f"[FINE] {err*100:.0f} cm / {yerr:+.1f} deg off -> slow fine approach (pass {k + 1}/2)")
                session.log("INFO", f"FINE APPROACH pass {k + 1}: start err {err*100:.1f} cm, yaw {yerr:+.1f} deg")
                self.fine_approach(target)
                avg = self.averaged_fix(4.0, min_fixes=2)
                arrived = avg or self.get_current_pose(timeout_sec=1.0)
                if not arrived:
                    break
                err = math.hypot(arrived['x'] - target['x'], arrived['y'] - target['y'])
                yerr = (arrived['yaw_deg'] - target['yaw_deg'] + 180.0) % 360.0 - 180.0
                zin, zdx, zdy, zdyaw = body_in_zone(arrived, target, zone)
                print(f"[ZONE] forward {zdx*100:+.1f} cm, sideways {zdy*100:+.1f} cm, heading {zdyaw:+.1f} deg "
                      f"-> {'INSIDE' if zin else 'OUTSIDE'} zone")
                session.log("INFO", f"ZONE CHECK after fine pass {k + 1}: fwd {zdx*100:+.1f} cm, side {zdy*100:+.1f} cm, "
                            f"yaw {zdyaw:+.1f} deg, inside={zin}")
            for k in range(2):
                if not arrived or yerr is None or abs(yerr) <= 3.0 or err > 0.05:
                    break
                print(f"[HEADING] {yerr:+.1f} deg off -> turning {-yerr:+.1f} deg on odometry (touch-up {k + 1}/2)")
                session.log("INFO", f"HEADING TOUCH-UP {k + 1}: yaw {yerr:+.1f} deg")
                self.rotate_by(math.radians(-yerr))
                avg = self.averaged_fix(4.0, min_fixes=2)
                arrived = avg or self.get_current_pose(timeout_sec=1.0)
                if not arrived:
                    break
                err = math.hypot(arrived['x'] - target['x'], arrived['y'] - target['y'])
                yerr = (arrived['yaw_deg'] - target['yaw_deg'] + 180.0) % 360.0 - 180.0
                zin, zdx, zdy, zdyaw = body_in_zone(arrived, target, zone)
                print(f"[ZONE] forward {zdx*100:+.1f} cm, sideways {zdy*100:+.1f} cm, heading {zdyaw:+.1f} deg "
                      f"-> {'INSIDE' if zin else 'OUTSIDE'} zone")
            ok = zin and err is not None and err <= 0.05 and abs(yerr) <= 5.0
            print(f"\n[{'SUCCESS' if ok else 'OFF-TARGET'}] '{name}': VERIFIED error {err:.2f} m, heading {yerr:+.1f} deg")
            session.finish(success=ok, summary=f"Arrived at '{name}', verified err {err:.2f} m / {yerr:+.1f} deg",
                           status_code=status)
            print(f"\n[LOGGER] Mission log saved to: {session.session_file}")
            print("[LOGGER] View history: ros2 run esp32_odom_bridge view_logs\n")
            return ok
        else:
            code_desc = robot_logger.NAV2_STATUS_NAMES.get(status, f"CODE_{status}")
            err_code = getattr(result.result, 'error_code', 0) if hasattr(result, 'result') else 0
            err_name, err_desc = robot_logger.NAV2_ERROR_DESCRIPTIONS.get(
                err_code, ("UNKNOWN_ERROR", "No description available.")
            )

            print(f"\n[NAV2] Navigation ended with status {status} ({code_desc}).")
            if err_code:
                print(f"[NAV2] Nav2 error code: {err_code} ({err_name})")
                print(f"[NAV2] Explanation: {err_desc}")

            session.finish(
                success=False,
                summary=f"Navigation aborted before reaching '{name}'",
                status_code=status,
                error_code=err_code,
                diagnostic_hint=err_desc
            )
            print(f"\n[LOGGER] Detailed diagnostic log saved to: {session.session_file}")
            print(f"[LOGGER] Quick debug command: ros2 run esp32_odom_bridge view_logs --last\n")
            return False

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        rem = fb.distance_remaining
        time_rem = fb.estimated_time_remaining.sec
        print(f"\r[NAV2] Driving... Distance to target: {rem:.2f} m | Est. Time: {time_rem}s ", end="", flush=True)

        # Log milestone telemetry every 2.5 seconds to keep log clean
        now = time.time()
        if now - self.last_fb_log_time >= 2.5:
            self.last_fb_log_time = now
            if self.active_log_session:
                self.active_log_session.telemetry({
                    'dist_remaining_m': f"{rem:.2f}",
                    'est_time_s': f"{time_rem}",
                })

        # Periodically refresh target & landmark markers in RViz
        if self.current_target_name and self.current_target_pose:
            self.publish_target_marker(self.current_target_name, self.current_target_pose)

    def run_circuit(self, waypoints=None, pause_sec=3.0, loop=False, unattended=False, unverified=False):
        """Visit landmarks in order. loop=True repeats until Ctrl+C.
        unattended=True: retry a failed stop once, then skip it without asking."""
        self.circuit_unverified = unverified
        lap = 1
        while True:
            if loop:
                print(f"\n################  PATROL LAP {lap}  ################")
            ok = self._run_circuit_once(waypoints, pause_sec, unattended or loop)
            if not loop:
                return ok
            lap += 1
            time.sleep(pause_sec)

    def _run_circuit_once(self, waypoints=None, pause_sec=3.0, unattended=False):
        landmarks = self.load_landmarks()
        if not landmarks:
            print("\n[ERROR] No landmarks found in database. Cannot run circuit.\n")
            return False

        if not waypoints:
            # Default circuit = landmarks in the order they were saved
            waypoints = list(landmarks.keys())

        print("\n" + "=" * 65)
        print("         AUTONOMOUS FULL CIRCUIT MISSION PLANNER")
        print("=" * 65)
        print(f"Total Circuit Stops: {len(waypoints)}")
        for idx, wp in enumerate(waypoints, 1):
            p = landmarks.get(wp, {})
            print(f"  [{idx}/{len(waypoints)}] -> {wp:<18} (X={p.get('x', 0):>6.2f}m, Y={p.get('y', 0):>6.2f}m)")
        print("=" * 65 + "\n")

        for idx, wp in enumerate(waypoints, 1):
            print(f"\n=======================================================")
            print(f" >>> EXECUTING STOP {idx}/{len(waypoints)}: '{wp}'")
            print(f"=======================================================")
            success = self.navigate_to(wp, unverified=getattr(self, 'circuit_unverified', False))
            if not success and unattended:
                print(f"\n[CIRCUIT] '{wp}' failed - retrying once in {pause_sec:.0f}s...")
                time.sleep(pause_sec)
                success = self.navigate_to(wp, unverified=getattr(self, 'circuit_unverified', False))
                if not success:
                    print(f"[CIRCUIT] '{wp}' failed twice - skipping.")
                    continue
            if not success:
                print(f"\n[CIRCUIT WARNING] Navigation to '{wp}' was interrupted or failed.")
                try:
                    choice = input(f"Skip '{wp}' and continue to next stop? [Y/n]: ").strip().lower()
                except Exception:
                    choice = 'y'
                if choice == 'n':
                    print("[CIRCUIT] Mission aborted by user.\n")
                    return False
            else:
                print(f"[CIRCUIT] Successfully reached stop {idx}/{len(waypoints)}: '{wp}'")

            if idx < len(waypoints):
                print(f"[CIRCUIT] Pausing for {pause_sec:.0f} seconds before moving to next stop...\n")
                time.sleep(pause_sec)

        print("\n" + "=" * 65)
        print("   MISSION COMPLETE: FULL CIRCUIT FINISHED SUCCESSFULLY!")
        print("=" * 65 + "\n")
        return True



def interactive_menu(mgr):
    while True:
        print("\n=== Robot Benchmark / Landmark Navigation ===")
        print("0. Check Live Localization Status (whereami)")
        print("1. List all saved landmarks")
        print("2. Save current robot position as a landmark")
        print("3. Navigate to a landmark (Auto Path Planning)")
        print("4. Run full circuit patrol (all landmarks sequentially)")
        print("5. SET INITIAL POSE (manual fallback hint)")
        print("6. Delete a landmark")
        print("7. Exit")
        choice = input("Enter choice [0-7]: ").strip()

        if choice == '0':
            mgr.show_status(wait_mode=False)
        elif choice == '1':
            mgr.list_landmarks()
        elif choice == '2':
            name = input("Enter landmark name (e.g. 'car', 'bench_1'): ").strip()
            if name:
                mgr.save_current_location(name)
        elif choice == '3':
            mgr.list_landmarks()
            name = input("Enter landmark to navigate to: ").strip()
            if name:
                mgr.navigate_to(name)
        elif choice == '4':
            mgr.run_circuit()
        elif choice == '5':
            mgr.list_landmarks()
            name = input("Enter landmark name to use as start pose [default: start]: ").strip()
            mgr.set_initial_pose(name if name else 'start')
        elif choice == '6':
            name = input("Enter landmark to delete: ").strip()
            if name:
                mgr.delete_landmark(name)
        elif choice == '7':
            break




def main():
    rclpy.init()

    filtered_args = list(sys.argv)
    mgr = LandmarkManager(filepath=DEFAULT_LANDMARKS_FILE)
    print(f"[INFO] Active map: {mgr.active_map()} | Landmarks file: {DEFAULT_LANDMARKS_FILE}")

    if len(filtered_args) < 2:
        interactive_menu(mgr)
    else:
        cmd = filtered_args[1].lower()
        if cmd == 'save' and len(filtered_args) >= 3:
            force = any(a in ('--force', '-f') for a in filtered_args[3:])
            unverified = '--unverified' in filtered_args[3:]
            mgr.save_current_location(filtered_args[2], force=force, unverified=unverified)
        elif cmd == 'list':
            mgr.list_landmarks()
        elif cmd == 'delete' and len(filtered_args) >= 3:
            mgr.delete_landmark(filtered_args[2])
        elif cmd == 'goto' and len(filtered_args) >= 3:
            zone = None
            if '--zone' in filtered_args:
                zone = float(filtered_args[filtered_args.index('--zone') + 1])
            mgr.navigate_to(filtered_args[2], unverified='--unverified' in filtered_args[3:], zone=zone)
        elif cmd in ['circuit', 'patrol']:
            # circuit [a b c]          -> one pass, asks on failure
            # patrol  [a b c] [--loop] -> unattended (retry once, skip); --loop repeats till Ctrl+C
            flags = {'--loop', '-l', '--auto', '-y', '--unverified'}
            custom_waypoints = [a for a in filtered_args[2:] if a not in flags] or None
            loop = any(a in filtered_args for a in ['--loop', '-l'])
            unattended = cmd == 'patrol' or any(a in filtered_args for a in ['--auto', '-y'])
            try:
                mgr.run_circuit(waypoints=custom_waypoints, loop=loop, unattended=unattended,
                                unverified='--unverified' in filtered_args)
            except KeyboardInterrupt:
                print("\n[CIRCUIT] Stopped by user.")
        elif cmd in ['setpose', 'localize', 'init']:
            # Set initial pose: optionally pass landmark name, default = 'start'
            pose_name = filtered_args[2] if len(filtered_args) >= 3 else 'start'
            mgr.set_initial_pose(pose_name)
        elif cmd in ['whereami', 'status', 'pose']:
            wait_mode = any(a in filtered_args for a in ['--wait', '-w', 'wait'])
            mgr.show_status(wait_mode=wait_mode)
        else:
            print("\nUsage:")
            print("  ros2 run esp32_odom_bridge landmark_manager whereami          (check true live localized position in map)")
            print("  ros2 run esp32_odom_bridge landmark_manager whereami --wait   (live monitor until visual relocalization confirms)")
            print("  ros2 run esp32_odom_bridge landmark_manager goto <name>       (drive autonomously to any landmark)")
            print("  ros2 run esp32_odom_bridge landmark_manager setpose <name>    (manually hint initial pose to a landmark)")
            print("  ros2 run esp32_odom_bridge landmark_manager list              (list all saved landmarks)")
            print("  ros2 run esp32_odom_bridge landmark_manager save <name>")
            print("  ros2 run esp32_odom_bridge landmark_manager delete <name>")
            print("  ros2 run esp32_odom_bridge landmark_manager circuit [a b c]   (one pass through landmarks)")
            print("  ros2 run esp32_odom_bridge landmark_manager patrol [a b c] --loop  (unattended, repeat until Ctrl+C)")
            print("  ros2 run esp32_odom_bridge landmark_manager (interactive menu)\n")

    mgr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
