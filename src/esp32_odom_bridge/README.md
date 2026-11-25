# esp32_odom_bridge

Diff-drive AMR: ESP32 base (IG52 + IBT-2, 0.56 m track, 125 mm wheels, 15902 ticks/rev),
OAK-D Lite at 1.24 m. ROS 2 Jazzy, RTAB-Map, robot_localization EKF, Nav2.

## Layout
```
esp32_odom_bridge/
├── esp32_odom_bridge/          ROS 2 nodes
│   ├── odom_bridge_node.py     serial <-> /cmd_vel, /cmd_vel_safe, /wheel_odom
│   ├── landmark_manager.py     save / goto / circuit / patrol landmarks (Nav2)
│   ├── responsive_teleop.py    keyboard teleop, instant stop
│   ├── activity_monitor.py, robot_logger.py, view_logs.py
│   └── tools/                  calibrate_odometry, calibrate_ticks, live_ticks, motor_diagnostics
├── launch/
│   ├── mapping.launch.py              SLAM (new map)
│   ├── localization.launch.py         RTAB-Map localization + 3D obstacle cloud
│   ├── nav2.launch.py                 Nav2 only
│   └── autonomous_bringup.launch.py   localization + Nav2
├── config/   ekf.yaml, nav2_params.yaml, navigate_landmarks_bt.xml, imu_calib.yaml, *.rviz
├── scripts/  start_robot.sh, teleop.sh, save_nav_map.sh, view_pointcloud.sh, calibrate_imu.sh/.py
└── firmware/
    ├── amr_base_pid/amr_base_pid.ino          NEW: watchdog + closed-loop wheel speed
    └── clean_camera_line_follower_mapping.ino  currently flashed (open-loop, no watchdog)
```

## Build
```bash
cd ~/mapping_ws
chmod +x src/esp32_odom_bridge/scripts/*
colcon build --symlink-install --packages-select esp32_odom_bridge
source install/setup.bash
```

## 1. Map
```bash
ros2 launch esp32_odom_bridge mapping.launch.py delete_db:=true   # T1
ros2 run esp32_odom_bridge teleop.sh                              # T2: slow full loop, return to start
# Ctrl+C T1, then:
ros2 run esp32_odom_bridge save_nav_map.sh                        # -> nav_data/nav_map.db (+ empties nav_data/landmarks.yaml)
ros2 run esp32_odom_bridge view_pointcloud.sh                     # optional 3D check
```

## 2. Landmarks
```bash
ros2 launch esp32_odom_bridge autonomous_bringup.launch.py        # T1 (or start_robot.sh)
ros2 run esp32_odom_bridge teleop.sh                              # T2: drive to a spot
ros2 run esp32_odom_bridge landmark_manager whereami --wait       # T3: confirm localized
ros2 run esp32_odom_bridge landmark_manager save <name>           # repeat per landmark
```

## 3. Navigate (with obstacle avoidance)
```bash
ros2 run esp32_odom_bridge landmark_manager goto <name>
ros2 run esp32_odom_bridge landmark_manager circuit a b c         # one pass
ros2 run esp32_odom_bridge landmark_manager patrol a b c --loop   # unattended, Ctrl+C to stop
ros2 run esp32_odom_bridge view_logs                              # mission logs
```
Obstacle check: `ros2 topic hz /obstacles_cloud` (~10-15 Hz). RViz: `/obstacles_cloud`,
`/local_costmap/costmap`, `/polygon_stop`. If the cloud is empty/garbage, compare widths of
`/oak/stereo/image_raw` and `/oak/rgb/camera_info`; if they differ launch with
`depth_info_topic:=/oak/stereo/camera_info`.

## Calibration / diagnostics
```bash
ros2 run esp32_odom_bridge live_ticks [/dev/ttyUSB0]         # hand-spin wheel, watch ticks
ros2 run esp32_odom_bridge motor_diagnostics [/dev/ttyUSB0]  # drive each motor (wheels up!)
ros2 run esp32_odom_bridge yaw_check                        # heading sign check (wheels / IMU / EKF)
ros2 run esp32_odom_bridge calibrate_ticks [/dev/ttyUSB0]
ros2 run esp32_odom_bridge calibrate_odometry               # needs mapping.launch.py running
ros2 run esp32_odom_bridge calibrate_imu.sh                 # robot stationary
```

## Firmware (amr_base_pid)
Same pins + protocol as the old sketch. Flash `firmware/amr_base_pid/amr_base_pid.ino`, wheels off ground:
```bash
python3 -m serial.tools.miniterm --eol LF /dev/ttyUSB0 115200
```
`V` debug on, `F` forward 1.2 s (DBG measured speed ~ +0.15), `0.1,0` once -> stops after 0.4 s.
`FAULT,ENC_DIR_L/R` -> flip `ENC_FWD_SIGN_L/R`. `C` clears faults, `P` toggles old open-loop mode.

## Robot constants (keep in sync)
| What | Value | Where |
|---|---|---|
| ticks/rev | 15902 | launch files, odom_bridge_node, firmware |
| wheelbase | 0.56 m | launch files, firmware |
| camera | x 0.30, z 1.24, pitch 0.109 | launch files |
| footprint | 0.72 x 0.66 m (MEASURE) | config/nav2_params.yaml |
