#!/usr/bin/env python3
"""
Wheel Odometry Interactive Calibration Tool
---------------------------------------------
Calibrates ticks_per_rev (linear distance) and wheelbase (angular rotation)
by comparing measured physical ground-truth against /wheel_odom readings.
Works live while mapping.launch.py and teleop.sh are running!
"""

import os
import sys
import math
import re
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomCalibrator(Node):
    def __init__(self):
        super().__init__('odom_calibrator')
        self.sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.odom_callback,
            10
        )
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_raw_yaw = 0.0
        self.total_yaw = 0.0
        self.received_first = False
        self.total_dist_traveled = 0.0
        self.last_x = 0.0
        self.last_y = 0.0

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        if not self.received_first:
            self.current_x = x
            self.current_y = y
            self.current_yaw = yaw
            self.last_raw_yaw = yaw
            self.total_yaw = 0.0
            self.total_dist_traveled = 0.0
            self.last_x = x
            self.last_y = y
            self.received_first = True
            return

        # Track incremental distance traveled
        dx = x - self.last_x
        dy = y - self.last_y
        step = math.sqrt(dx * dx + dy * dy)
        self.total_dist_traveled += step
        self.last_x = x
        self.last_y = y

        # Track continuous unwrapped yaw
        dyaw = yaw - self.last_raw_yaw
        if dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2.0 * math.pi
        self.total_yaw += dyaw
        self.last_raw_yaw = yaw

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw


def get_running_param(name):
    """Value the RUNNING odom bridge uses (ros2 param get), or None."""
    try:
        out = subprocess.run(['ros2', 'param', 'get', '/esp32_odom_bridge', name],
                             capture_output=True, text=True, timeout=8).stdout
        m = re.search(r'value is:\s*([-0-9.eE]+)', out)
        return float(m.group(1)) if m else None
    except Exception:
        return None


def get_launch_default(name):
    path = os.path.expanduser('~/mapping_ws/src/esp32_odom_bridge/launch/mapping.launch.py')
    try:
        m = re.search(r"DeclareLaunchArgument\('%s',\s*default_value='([^']*)'\)" % name, open(path).read())
        return float(m.group(1)) if m else None
    except Exception:
        return None


def update_launch_files(new_ticks=None, new_wheelbase=None):
    """Update ticks_per_rev and/or wheelbase defaults in mapping and localization launch files."""
    launch_paths = [
        os.path.expanduser('~/mapping_ws/src/esp32_odom_bridge/launch/mapping.launch.py'),
        os.path.expanduser('~/mapping_ws/src/esp32_odom_bridge/launch/localization.launch.py'),
    ]

    for path in launch_paths:
        if not os.path.exists(path):
            continue
        with open(path, 'r') as f:
            content = f.read()

        import re
        if new_ticks is not None:
            content = re.sub(
                r"ticks_per_rev_arg\s*=\s*DeclareLaunchArgument\('ticks_per_rev',\s*default_value='[^']*'\)",
                f"ticks_per_rev_arg = DeclareLaunchArgument('ticks_per_rev', default_value='{int(new_ticks)}')",
                content
            )
        if new_wheelbase is not None:
            content = re.sub(
                r"wheelbase_arg\s*=\s*DeclareLaunchArgument\('wheelbase',\s*default_value='[^']*'\)",
                f"wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='{new_wheelbase:.4f}')",
                content
            )

        with open(path, 'w') as f:
            f.write(content)
        print(f"[SUCCESS] Updated {path}")


def main():
    rclpy.init()
    node = OdomCalibrator()
    # Spin in the background: callbacks MUST keep running while input() blocks,
    # otherwise only the last ~10 queued messages are seen (a 360 deg turn read as ~10 deg).
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print("=" * 65)
    print("      WHEEL ODOMETRY INTERACTIVE CALIBRATION TOOL")
    print("=" * 65)
    print("[INFO] Waiting for /wheel_odom messages...")
    print("       (Make sure mapping.launch.py or odom_bridge_node is running!)")

    # Spin briefly to check connection
    t0 = time.time()
    while not node.received_first:
        time.sleep(0.1)
        if time.time() - t0 > 5.0:
            print("\n[WARNING] No messages received on /wheel_odom yet.")
            print("Please ensure your robot launch file is running in another tab:")
            print("  ros2 launch esp32_odom_bridge mapping.launch.py")
            t0 = time.time()

    print("\n[CONNECTED] Live telemetry active from /wheel_odom!")

    while True:
        print("\n" + "-" * 55)
        print("  SELECT CALIBRATION ACTION:")
        print("  [1] Linear Distance Calibration (ticks_per_rev)")
        print("  [2] Angular Turn Calibration (wheelbase)")
        print("  [3] Live Odometry Monitor (Test current accuracy)")
        print("  [q] Quit")
        print("-" * 55)

        choice = input("Enter choice (1, 2, 3, q): ").strip().lower()

        if choice == '1':
            calibrate_linear(node)
        elif choice == '2':
            calibrate_angular(node)
        elif choice == '3':
            monitor_live(node)
        elif choice in ('q', 'quit', 'exit'):
            break
        else:
            print("Invalid choice. Please enter 1, 2, 3, or q.")

    node.destroy_node()
    rclpy.shutdown()
    print("\nCalibration tool exited cleanly.")


def calibrate_linear(node: OdomCalibrator):
    print("\n" + "=" * 60)
    print("         LINEAR DISTANCE CALIBRATION")
    print("=" * 60)
    print("INSTRUCTIONS:")
    print("  1. Align the robot's front wheel or chassis edge with a line on the floor.")
    print("  2. Place a tape measure along the driving path (1.0m to 3.0m).")
    print("  3. Press ENTER when the robot is ready at the starting mark.")
    print("=" * 60)
    input("Press ENTER to set baseline start position...")

    # Flush current pose
    time.sleep(0.1)
    start_x = node.current_x
    start_y = node.current_y
    node.total_dist_traveled = 0.0

    print("\n>>> BASELINE SET! <<<")
    print("Now drive the robot forward in a straight line using teleop (or push it by hand).")
    print("When you reach your target stopping mark, press ENTER.")
    input("\nPress ENTER when stopped at the finish mark...")

    # Capture final pose
    for _ in range(5):
        time.sleep(0.05)

    dx = node.current_x - start_x
    dy = node.current_y - start_y
    net_disp = math.sqrt(dx * dx + dy * dy)
    accum_dist = node.total_dist_traveled

    print("\n" + "-" * 50)
    print("ODOMETRY REPORT:")
    print(f"  Net Euclidean Displacement : {net_disp:.4f} meters")
    print(f"  Total Accumulated Path     : {accum_dist:.4f} meters")
    print("-" * 50)

    val_str = input("\nEnter the ACTUAL measured distance from your tape measure in meters (e.g. 1.00): ").strip()
    try:
        actual_dist = float(val_str)
        if actual_dist <= 0.01:
            print("Error: Distance must be greater than 0.")
            return

        measured_odom = net_disp if net_disp > 0.05 else accum_dist
        if measured_odom <= 0.001:
            print("Error: Reported odometry distance was zero. Did the wheels turn?")
            return

        scale = actual_dist / measured_odom
        run = get_running_param('ticks_per_rev')
        current_ticks = int(run) if run else int(get_launch_default('ticks_per_rev') or 49415)
        print(f"[INFO] ticks_per_rev used by the running bridge: {current_ticks}")
        # Prompt user to confirm current ticks_per_rev or default
        ticks_input = input(f"Enter currently configured ticks_per_rev [{current_ticks}]: ").strip()
        if ticks_input:
            try:
                current_ticks = int(ticks_input)
            except ValueError:
                pass

        calibrated_ticks = int(round(current_ticks / scale))

        print("\n" + "=" * 55)
        print("CALIBRATION RESULTS:")
        print(f"  Physical Actual Distance : {actual_dist:.4f} m")
        print(f"  Odometry Distance        : {measured_odom:.4f} m")
        print(f"  Correction Scale Factor  : {scale:.4f} ({scale*100:.1f}%)")
        print(f"  Previous ticks_per_rev   : {current_ticks}")
        print(f"  CALIBRATED ticks_per_rev : {calibrated_ticks}")
        print("=" * 55)

        apply_it = input("\nAutomatically update launch files with this new value? (y/n) [y]: ").strip().lower()
        if apply_it in ('', 'y', 'yes'):
            update_launch_files(new_ticks=calibrated_ticks)
            print("[DONE] Run 'colcon build --symlink-install' to activate.")

    except ValueError:
        print("Invalid number entered. Linear calibration aborted.")


def calibrate_angular(node: OdomCalibrator):
    print("\n" + "=" * 60)
    print("         ANGULAR ROTATION CALIBRATION (WHEELBASE)")
    print("=" * 60)
    print("INSTRUCTIONS:")
    print("  1. Note the robot's heading alignment against a reference line or tile joint.")
    print("  2. You will rotate the robot in place 360 degrees (1 full turn) or 720 degrees (2 turns).")
    print("  3. Press ENTER when the robot is at the initial heading.")
    print("=" * 60)
    input("Press ENTER to set baseline start heading...")

    time.sleep(0.1)
    node.total_yaw = 0.0

    print("\n>>> BASELINE HEADING SET! <<<")
    print("Now rotate the robot in place using teleop ('j' or 'l') or by hand.")
    print("Rotate exactly 1 full 360° turn (or 2 full 720° turns) back to the reference line.")
    input("\nPress ENTER when finished rotating...")

    for _ in range(5):
        time.sleep(0.05)

    odom_yaw_rad = abs(node.total_yaw)
    odom_yaw_deg = math.degrees(odom_yaw_rad)

    print("\n" + "-" * 50)
    print("ODOMETRY REPORT:")
    print(f"  Reported Rotation : {odom_yaw_deg:.2f} degrees ({odom_yaw_rad:.4f} radians)")
    print("-" * 50)

    turns_str = input("\nEnter the number of full turns completed (e.g. 1 for 360°, 2 for 720°): ").strip()
    try:
        turns = float(turns_str)
        if turns <= 0:
            print("Error: Turns must be greater than 0.")
            return

        actual_yaw_deg = turns * 360.0
        actual_yaw_rad = turns * 2.0 * math.pi

        if odom_yaw_deg <= 5.0:
            print("Error: Reported rotation was near zero. Did the robot rotate?")
            return

        # For differential drive: dth = (dr - dl) / wheelbase
        # If odometry reported less angle than actual, wheelbase is configured too large!
        # calibrated_wheelbase = current_wheelbase * (odom_yaw / actual_yaw)
        run_wb = get_running_param('wheelbase')
        run_ticks = get_running_param('ticks_per_rev')
        file_ticks = get_launch_default('ticks_per_rev')
        current_wheelbase = run_wb if run_wb else 0.56
        print(f"[INFO] running bridge: wheelbase={current_wheelbase} ticks_per_rev={run_ticks}")
        wb_input = input(f"Enter currently configured wheelbase [{current_wheelbase}]: ").strip()
        if wb_input:
            try:
                current_wheelbase = float(wb_input)
            except ValueError:
                pass

        calibrated_wheelbase = current_wheelbase * (odom_yaw_rad / actual_yaw_rad)
        # If ticks_per_rev was re-calibrated but the bridge still runs the old value,
        # correct for it: true rotation scales with metres-per-tick.
        if run_ticks and file_ticks and abs(run_ticks - file_ticks) > 1:
            calibrated_wheelbase *= run_ticks / file_ticks
            print(f"[INFO] corrected for ticks {run_ticks:.0f} (running) -> {file_ticks:.0f} (launch file)")
        if not 0.3 <= calibrated_wheelbase <= 0.9:
            print(f"\n[REJECTED] wheelbase {calibrated_wheelbase:.4f} m is physically impossible "
                  "(expected ~0.5-0.6 m). Nothing written. Redo the turn and count turns carefully.")
            return

        print("\n" + "=" * 55)
        print("CALIBRATION RESULTS:")
        print(f"  Actual Physical Rotation : {actual_yaw_deg:.1f}° ({turns} turn(s))")
        print(f"  Odometry Rotation        : {odom_yaw_deg:.1f}°")
        print(f"  Angular Ratio            : {odom_yaw_deg / actual_yaw_deg:.4f}")
        print(f"  Previous wheelbase       : {current_wheelbase:.4f} m")
        print(f"  CALIBRATED wheelbase     : {calibrated_wheelbase:.4f} m")
        print("=" * 55)

        apply_it = input("\nAutomatically update launch files with this new value? (y/n) [y]: ").strip().lower()
        if apply_it in ('', 'y', 'yes'):
            update_launch_files(new_wheelbase=calibrated_wheelbase)
            print("[DONE] Run 'colcon build --symlink-install' to activate.")

    except ValueError:
        print("Invalid number entered. Angular calibration aborted.")


def monitor_live(node: OdomCalibrator):
    print("\n" + "=" * 60)
    print("         LIVE ODOMETRY MONITOR (Ctrl+C to stop)")
    print("=" * 60)
    print("Drive the robot to watch real-time coordinates and heading:")
    print("  x [m]      | y [m]      | yaw [deg]  | total_dist [m]")
    print("-" * 60)

    try:
        while True:
            time.sleep(0.1)
            yaw_deg = math.degrees(node.current_yaw)
            sys.stdout.write(
                f"\r  x={node.current_x:+7.3f}m | y={node.current_y:+7.3f}m | "
                f"yaw={yaw_deg:+6.1f}° | path={node.total_dist_traveled:6.3f}m"
            )
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[INFO] Monitor stopped.")


if __name__ == '__main__':
    main()
