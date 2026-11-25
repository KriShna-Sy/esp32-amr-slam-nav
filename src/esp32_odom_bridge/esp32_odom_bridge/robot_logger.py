#!/usr/bin/env python3
"""
robot_logger.py
Centralized, human-readable logging system for all robot activities:
  - Mapping sessions
  - Nav2 autonomous navigation
  - Landmark management (goto, save, delete)
  - Hardware bridges (ESP32 serial, OAK-D camera)

Writes timestamped step-by-step logs to:
  - Master log: ~/mapping_ws/logs/robot_activity.log
  - Session log: ~/mapping_ws/logs/sessions/<activity>_<timestamp>.txt
  - Symlink:    ~/mapping_ws/logs/latest.log
"""

import os
import sys
import time
import threading
from datetime import datetime

LOGS_DIR = os.path.expanduser('~/mapping_ws/logs')
SESSIONS_DIR = os.path.join(LOGS_DIR, 'sessions')
MASTER_LOG_FILE = os.path.join(LOGS_DIR, 'robot_activity.log')
LATEST_LINK = os.path.join(LOGS_DIR, 'latest.log')

# Nav2 GoalStatus definitions
NAV2_STATUS_NAMES = {
    0: "STATUS_UNKNOWN",
    1: "STATUS_ACCEPTED",
    2: "STATUS_EXECUTING",
    3: "STATUS_CANCELING",
    4: "STATUS_SUCCEEDED",
    5: "STATUS_CANCELED",
    6: "STATUS_ABORTED",
}

# Nav2 Error Code Explanations
NAV2_ERROR_DESCRIPTIONS = {
    0: ("NO_ERROR", "Action completed without error."),
    100: ("UNKNOWN", "Unknown planner error."),
    101: ("PLAN_EMPTY", "Global planner produced an empty path."),
    102: ("FAILED_TO_MAKE_PLAN", "Path planner could not compute a path to the requested coordinates."),
    103: ("PATHTOGOAL_BLOCKED",
          "Global planner could not find a collision-free path within tolerance. "
          "DEBUG HINT: Target landmark might be too close to a wall/obstacle, "
          "or costmap inflation radius prevents navigation through a narrow doorway/corridor."),
    104: ("FAILED_TO_COLLISION_CHECK", "Collision checker failed to evaluate costmap."),
    105: ("GOAL_IN_COLLISION",
          "Goal point is located inside an obstacle or inflated lethal zone. "
          "DEBUG HINT: Move the landmark 10-20 cm away from walls/tables into open space."),
    106: ("START_IN_COLLISION",
          "Current robot position is reported inside an obstacle on the costmap. "
          "DEBUG HINT: Drive slightly in teleop to clear footprint, or clear costmaps."),
    107: ("PLANNER_TIMEOUT", "Planner timed out before a valid path could be found."),
    201: ("CONTROLLER_FAILED",
          "Controller server could not follow the path. "
          "DEBUG HINT: Check if wheels are slipping or an unexpected obstacle blocked the robot."),
    202: ("PATHTOLONG", "Requested path exceeds controller lookahead length."),
    203: ("PATHTOSHORT", "Requested path is shorter than goal tolerance."),
    204: ("CONTROLLER_TF_ERROR",
          "Transform timeout between 'map', 'odom', or 'base_link'. "
          "DEBUG HINT: Check if visual odometry or RTAB-Map dropped TF publication."),
    301: ("RECOVERY_FAILED",
          "All recovery behaviors (spin, backup, clear costmap) failed. Robot is stuck."),
}


def _ensure_dirs():
    os.makedirs(SESSIONS_DIR, exist_ok=True)


def get_timestamp():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


class ActivitySession:
    """Represents a single tracked activity session with step logging."""

    def __init__(self, activity_type: str, title: str, metadata: dict = None):
        _ensure_dirs()
        self.lock = threading.Lock()
        self.activity_type = activity_type
        self.title = title
        self.metadata = metadata or {}
        self.start_time = datetime.now()
        self.start_timestamp_str = self.start_time.strftime("%Y%m%d_%H%M%S")
        self.session_id = f"{activity_type.lower()}_{self.start_timestamp_str}"
        self.session_file = os.path.join(SESSIONS_DIR, f"{self.session_id}.txt")
        self.completed = False
        self.success = None
        self.steps = []

        # Write header to session file and master log
        header_text = [
            "=" * 80,
            f"[{get_timestamp()}] ACTIVITY START: {self.activity_type.upper()}",
            f"Session ID : {self.session_id}",
            f"Title      : {self.title}",
        ]
        for k, v in self.metadata.items():
            header_text.append(f"{k.ljust(11)}: {v}")
        header_text.append("-" * 80)
        self._write_lines(header_text)

        # Update symlink to point to this latest session
        try:
            if os.path.islink(LATEST_LINK) or os.path.exists(LATEST_LINK):
                os.remove(LATEST_LINK)
            os.symlink(self.session_file, LATEST_LINK)
        except OSError:
            pass

    def _write_lines(self, lines: list):
        with self.lock:
            text = "\n".join(lines) + "\n"
            # Append to session file
            with open(self.session_file, "a", encoding="utf-8") as f:
                f.write(text)
            # Append to master log
            with open(MASTER_LOG_FILE, "a", encoding="utf-8") as f:
                f.write(text)

    def log(self, tag: str, message: str):
        line = f"[{get_timestamp()}] [{tag.upper()}] {message}"
        self.steps.append(line)
        self._write_lines([line])

    def step(self, step_name: str, status: str = "IN_PROGRESS", detail: str = ""):
        msg = f"{step_name} [{status}]"
        if detail:
            msg += f" - {detail}"
        self.log("STEP", msg)

    def telemetry(self, data: dict):
        parts = [f"{k}={v}" for k, v in data.items()]
        self.log("TELEMETRY", " | ".join(parts))

    def warn(self, message: str):
        self.log("WARN", message)

    def error(self, message: str, debug_hint: str = ""):
        self.log("ERROR", message)
        if debug_hint:
            self.log("DEBUG_HINT", debug_hint)

    def finish(self, success: bool, summary: str = "", status_code: int = None,
               error_code: int = None, diagnostic_hint: str = ""):
        if self.completed:
            return
        self.completed = True
        self.success = success
        end_time = datetime.now()
        duration_sec = (end_time - self.start_time).total_seconds()

        result_tag = "SUCCESS" if success else "FAILED"
        lines = [
            "-" * 80,
            f"[{get_timestamp()}] RESULT: {result_tag}",
            f"Duration   : {duration_sec:.2f} seconds",
        ]
        if summary:
            lines.append(f"Summary    : {summary}")
        if status_code is not None:
            status_desc = NAV2_STATUS_NAMES.get(status_code, f"CODE_{status_code}")
            lines.append(f"Status Code: {status_code} ({status_desc})")
        if error_code is not None and error_code != 0:
            err_name, err_desc = NAV2_ERROR_DESCRIPTIONS.get(
                error_code, ("UNKNOWN_ERROR", "No description available.")
            )
            lines.append(f"Error Code : {error_code} ({err_name})")
            lines.append(f"Explanation: {err_desc}")
            if not diagnostic_hint:
                diagnostic_hint = err_desc

        if diagnostic_hint:
            lines.append(f"DEBUG HINT : {diagnostic_hint}")

        lines.append(f"[{get_timestamp()}] ACTIVITY END: {self.activity_type.upper()}")
        lines.append("=" * 80 + "\n")
        self._write_lines(lines)


def start_session(activity_type: str, title: str, metadata: dict = None) -> ActivitySession:
    """Convenience function to start a new activity logging session."""
    return ActivitySession(activity_type, title, metadata)


def log_simple_event(category: str, message: str, level: str = "INFO"):
    """Quick one-line logger for one-off actions without creating a full session."""
    _ensure_dirs()
    line = f"[{get_timestamp()}] [{level.upper()}] [{category.upper()}] {message}\n"
    with open(MASTER_LOG_FILE, "a", encoding="utf-8") as f:
        f.write(line)
