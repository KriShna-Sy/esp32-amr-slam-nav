#!/usr/bin/env bash
# OAK-D Lite IMU zero-rate bias calibration (robot must be STATIONARY on level floor).
# Writes config/imu_calib.yaml. Needs the depthai venv.
set -e
PY=/home/krishna/ros_oak_env/bin/python3
SRC=$HOME/mapping_ws/src/esp32_odom_bridge/scripts/calibrate_imu.py
[ -x "$PY" ] || { echo "[ERROR] $PY not found"; exit 1; }
exec "$PY" "$SRC" "$@"
