#!/bin/bash
# ==============================================================================
# Autonomous Robot One-Click Bringup Script
# Launches entire autonomous navigation stack (Hardware + SLAM + Nav2) in one command.
# ==============================================================================

set -e

echo "======================================================================"
echo "          STARTING AUTONOMOUS ROBOT NAVIGATION STACK"
echo "======================================================================"

# 1. Clean up any stale/zombie processes from previous runs
echo "[1/4] Cleaning up any lingering processes on camera and serial bus..."
pkill -9 -f "nav2|rtabmap|oak_ros_bridge|odom_bridge|rviz2|robot_state_publisher" 2>/dev/null || true
sleep 1

# 2. Check serial port
SERIAL_PORT="/dev/ttyUSB0"
if [ ! -e "$SERIAL_PORT" ]; then
    echo "[WARN] $SERIAL_PORT not found! Checking /dev/ttyUSB*..."
    ls -l /dev/ttyUSB* 2>/dev/null || echo "[ERROR] No USB serial ports detected. Check ESP32 cable."
else
    echo "[2/4] Verified serial port: $SERIAL_PORT"
fi

# 3. Source ROS 2 environments
echo "[3/4] Sourcing ROS 2 Jazzy and Workspace..."
source /opt/ros/jazzy/setup.bash
source /home/krishna/mapping_ws/install/setup.bash

# Ensure cleanup on script exit
cleanup() {
    echo ""
    echo "[SHUTDOWN] Stopping all autonomous robot processes..."
    pkill -9 -f "nav2|rtabmap|oak_ros_bridge|odom_bridge|rviz2|robot_state_publisher" 2>/dev/null || true
    echo "[SHUTDOWN] All devices and ports cleanly released."
}
trap cleanup EXIT INT TERM

# 4. Launch unified autonomous bringup
echo "[4/4] Launching Autonomous Bringup (Hardware + Localization + Nav2)..."
echo "      - Hardware & Localization starts immediately"
echo "      - Nav2 initiates cleanly after sensors & TF are streaming"
echo "======================================================================"
echo "  *** CRITICAL: DO NOT HIT CTRL+C IN THIS TERMINAL! ***"
echo "  This terminal MUST STAY RUNNING as the robot brain / server."
echo ""
echo "  To command the robot to drive, OPEN A NEW SEPARATE TERMINAL (Terminal 2):"
echo "    source /opt/ros/jazzy/setup.bash && source ~/mapping_ws/install/setup.bash"
echo "    ros2 run esp32_odom_bridge landmark_manager goto hello_rami"
echo "    ros2 run esp32_odom_bridge landmark_manager circuit"
echo "======================================================================"
echo ""

ros2 launch esp32_odom_bridge autonomous_bringup.launch.py launch_viz:=true "$@"
