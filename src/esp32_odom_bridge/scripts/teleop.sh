#!/bin/bash
# ==============================================================================
# Responsive Teleop Launcher
# Instant braking on key release (no coasting or lingering motion)
# ==============================================================================

source /opt/ros/jazzy/setup.bash
source "$HOME/mapping_ws/install/setup.bash"

ros2 run esp32_odom_bridge responsive_teleop "$@"
