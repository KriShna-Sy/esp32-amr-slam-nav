#!/usr/bin/env python3
"""
activity_monitor.py
Background node that monitors mapping and navigation sessions:
  - Logs session start & configuration
  - Tracks odometry quality, distance driven, and dropped frames
  - Tracks RTAB-Map node creation, loop closures, and memory usage
  - Generates comprehensive session summary and debugging recommendations on shutdown
"""

import os
import math
import time
import signal
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info

try:
    from . import robot_logger
except ImportError:
    import robot_logger


class ActivityMonitorNode(Node):
    def __init__(self):
        super().__init__('activity_monitor')

        self.declare_parameter('activity_type', 'mapping')
        self.declare_parameter('title', 'SLAM Mapping Session')
        self.declare_parameter('database_path', os.path.expanduser('~/.ros/rtabmap.db'))

        self.activity_type = self.get_parameter('activity_type').value
        self.title = self.get_parameter('title').value
        self.database_path = self.get_parameter('database_path').value

        meta = {
            'Mode': self.activity_type.upper(),
            'Database': self.database_path,
        }
        self.session = robot_logger.start_session(self.activity_type, self.title, meta)

        self.total_distance = 0.0
        self.last_x = None
        self.last_y = None
        self.odom_updates = 0
        self.total_nodes = 0
        self.loop_closures = 0
        self.last_log_time = time.time()
        self.low_quality_count = 0
        self.good_quality_count = 0

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Info, '/info', self._rtabmap_info_cb, 10)
        self.create_subscription(Info, '/rtabmap/info', self._rtabmap_info_cb, 10)

        # Periodic timer for progress logging (every 15 seconds)
        self.create_timer(15.0, self._periodic_log)

        self.session.step("Sensors & Stack Initialization", status="IN_PROGRESS", detail="Awaiting odometry and map topics")
        self.get_logger().info(f"ActivityMonitor active. Session logging to: {self.session.session_file}")

    def _odom_cb(self, msg: Odometry):
        self.odom_updates += 1
        pos = msg.pose.pose.position
        if self.last_x is not None:
            dx = pos.x - self.last_x
            dy = pos.y - self.last_y
            step_d = math.hypot(dx, dy)
            if step_d < 1.0:  # ignore large jumps
                self.total_distance += step_d
        self.last_x = pos.x
        self.last_y = pos.y

    def _rtabmap_info_cb(self, msg: Info):
        if msg.ref_id > self.total_nodes:
            self.total_nodes = msg.ref_id
        if msg.loop_closure_id > 0:
            self.loop_closures += 1
            self.session.log("LOOP_CLOSURE", f"Visual loop closure verified with Node #{msg.loop_closure_id} (Ref #{msg.ref_id})")

    def _periodic_log(self):
        db_size_mb = 0.0
        if os.path.exists(self.database_path):
            try:
                db_size_mb = os.path.getsize(self.database_path) / (1024 * 1024)
            except OSError:
                pass

        self.session.telemetry({
            'nodes': self.total_nodes,
            'loop_closures': self.loop_closures,
            'distance_m': f"{self.total_distance:.2f}",
            'db_size_mb': f"{db_size_mb:.1f}",
            'odom_updates': self.odom_updates,
        })

    def shutdown(self):
        db_size_mb = 0.0
        if os.path.exists(self.database_path):
            try:
                db_size_mb = os.path.getsize(self.database_path) / (1024 * 1024)
            except OSError:
                pass

        summary = (
            f"Completed with {self.total_nodes} nodes, {self.loop_closures} loop closures, "
            f"{self.total_distance:.2f} m driven, {db_size_mb:.1f} MB database."
        )

        hints = []
        if self.total_nodes > 0 and self.loop_closures == 0:
            hints.append("Notice: Zero loop closures were detected. For best map consistency, ensure you complete a closed loop back to your starting position.")
        if self.total_distance > 5.0 and db_size_mb < 2.0:
            hints.append("Notice: Database size is unexpectedly small relative to distance driven. Verify camera depth image alignment.")

        diag_hint = " ".join(hints) if hints else "Session concluded cleanly without errors."

        self.session.finish(
            success=True,
            summary=summary,
            diagnostic_hint=diag_hint
        )
        self.get_logger().info(f"ActivityMonitor finished. Session saved: {self.session.session_file}")


def main(args=None):
    rclpy.init(args=args)
    node = ActivityMonitorNode()

    def sig_handler(sig, frame):
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
