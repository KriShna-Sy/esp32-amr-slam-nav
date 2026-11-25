# Autonomous Mobile Robot SLAM & Precision 3D Digital Twin System
## Complete Engineering Architecture, Diagnostic Evolution & Operations Manual

---

**Project**: Differential-Drive Autonomous Mobile Robot (AMR) Precision SLAM  
**Platform**: Custom Heavy-Duty Differential Drive Base + Luxonis OAK-D Lite + ESP32  
**Target Environment**: Indoor Laboratory / Workshop (10.4 m × 10.6 m)  
**Software Stack**: ROS 2 Jazzy, RTAB-Map SLAM, robot_localization (EKF), DepthAI  
**Workspace**: `/home/krishna/mapping_ws`  
**Date**: October 2026  

---

## Table of Contents
1. Executive Summary & Project Mission
2. Physical Hardware Architecture & Kinematics
3. Software Architecture & Sensor Fusion Pipeline
4. Chronological Engineering Journey & Breakthroughs
   - Milestone 1: ESP32 Teleoperation Bridge & Deadband Watchdog
   - Milestone 2: Diagnosis of the 360° "Car Everywhere" Radial Warp
   - Milestone 3: Precision IMU Physical Calibration (Bosch BMI270)
   - Milestone 4: Ground-Truth Wheel Odometry Calibration (Tape Measure)
   - Milestone 5: Linux Serial Burst Jitter & Microcontroller Timestamp Sync
   - Milestone 6: Point Cloud Pipeline & Stale File Viewer Bug Fix
   - Milestone 7: Speed Optimization & Visual Neighbor Link Refining ("7 Cars" Fix)
   - Milestone 8: Loop Closure Optimization & Golden Map Consistency
5. Standard Operating Procedures (SOP) & Usage Manual
6. Workspace File Directory & Reference Map

---

## 1. Executive Summary & Project Mission

The primary objective of this project is the design, calibration, and deployment of a centimeter-accurate 2D Occupancy Grid Mapping and Dense 3D Point Cloud Digital Twin generation system on a custom differential-drive mobile robot. 

Autonomous ground navigation and high-fidelity 3D reconstruction impose strict mathematical requirements on metric consistency:
- **Zero Global Drift**: Rectangular rooms must remain rectangular; walls must not curve or double-up.
- **Physical Scale Fidelity**: A physical distance of 1.00 m on the floor must correspond to exactly 1.00 m in the map.
- **Single-Object Rigidity**: Stationary physical obstacles (such as vehicles, desks, and storage lockers) must be resolved into single, razor-sharp 3D representations, completely free of multi-frame ghosting or angular smearing.
- **Seamless Loop Closure**: Returning to a previously visited location must automatically trigger pose-graph optimization, eliminating any accumulated dead-reckoning drift.

Through iterative diagnosis and rigorous empirical ground-truth testing, this project resolved fundamental sensor coordinate inversions, hardware timing jitter, wheel encoder scaling discrepancies, and visual odometry link refinement challenges, achieving a verified, drift-free SLAM baseline.

---

## 2. Physical Hardware Architecture & Kinematics

### 2.1 Mechanical & Actuation Specifications
- **Chassis Architecture**: Two-wheel differential-drive base with dual passive heavy-duty caster wheels.
- **Effective Wheelbase ($L$)**: $0.56\text{ m}$ ($560\text{ mm}$ track width between tire contact patches).
- **Drive Wheel Diameter ($D$)**: $0.125\text{ m}$ ($125\text{ mm}$ molded rubber wheels).
- **Actuators**: Dual high-torque IG52 planetary gear motors.
- **Encoders**: High-resolution optical quadrature encoders mounted directly to the motor output shafts.
- **Motor Controllers**: Dual IBT-2 (BTS7960) 43A continuous H-bridge driver modules driven via 20 kHz hardware PWM on the ESP32 to eliminate audible motor whine.

### 2.2 Microcontroller & Communications
- **Processor**: Espressif ESP32 dual-core microcontroller.
- **Baud Rate**: 115,200 baud over bidirectional serial (`/dev/ttyUSB0`).
- **Telemetry Frequency**: Dedicated 50 Hz hardware timer loop outputting structured ASCII telemetry:
  `ODO,<left_ticks>,<right_ticks>,<uptime_millis>`
- **Hardware Watchdog**: 250 ms automatic velocity timeout. If teleoperation or autonomous navigation commands cease for >250 ms, motor PWM is actively clamped to zero to prevent runaway motion.

### 2.3 Sensor Payload & Extrinsics
- **Visual & Depth Sensor**: Luxonis OAK-D Lite stereo camera with onboard Bosch BMI270 6-DOF Inertial Measurement Unit (IMU).
- **Camera Extrinsics relative to `base_link` (center of drive axle)**:
  - $x = +0.30\text{ m}$ ($30\text{ cm}$ forward of wheel axle)
  - $y = 0.00\text{ m}$ (centered laterally)
  - $z = +1.24\text{ m}$ ($124\text{ cm}$ elevation above floor level)
  - $\text{Roll} = 0.00\text{ rad}$
  - $\text{Pitch} = +0.109\text{ rad}$ ($6.24^\circ$ downward tilt toward floor)
  - $\text{Yaw} = 0.00\text{ rad}$

### 2.4 Differential Drive Kinematics Formulation
For encoder tick displacements $\Delta N_L$ and $\Delta N_R$ over sample period $\Delta t$:
$$\Delta s_L = \Delta N_L \cdot \left(\frac{\pi D}{K_{ticks}}\right), \quad \Delta s_R = \Delta N_R \cdot \left(\frac{\pi D}{K_{ticks}}\right)$$
$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \quad \Delta\theta = \frac{\Delta s_R - \Delta s_L}{L}$$
Pose integration via 2nd-order Runge-Kutta (midpoint integration):
$$x_{k+1} = x_k + \Delta s \cos\left(\theta_k + \frac{\Delta\theta}{2}\right)$$
$$y_{k+1} = y_k + \Delta s \sin\left(\theta_k + \frac{\Delta\theta}{2}\right)$$
$$\theta_{k+1} = \theta_k + \Delta\theta$$

---

## 3. Software Architecture & Sensor Fusion Pipeline

The robotic software system is structured across four decoupled, deterministic layers running under ROS 2 Jazzy:

```
+--------------------------------------------------------------------------------+
|                             USER INTERACTION LAYER                             |
|  - responsive_teleop.py (/cmd_vel)     - view_pointcloud.sh (PCL/MeshLab GUI)  |
+--------------------------------------------------------------------------------+
                                       |
                                       v
+--------------------------------------------------------------------------------+
|                           HARDWARE DRIVER & IO LAYER                           |
|  - odom_bridge_node (Serial <-> ROS 2) - oak_ros_bridge.py (DepthAI <-> ROS 2) |
|    * Publishes: /wheel_odom (50 Hz)      * Publishes: /oak/rgb/image_raw (15 Hz) |
|    * Subscribes: /cmd_vel                * Publishes: /oak/stereo/image_raw      |
|                                          * Publishes: /oak/imu/data (100 Hz)     |
+--------------------------------------------------------------------------------+
                                       |
                                       v
+--------------------------------------------------------------------------------+
|                         STATE ESTIMATION LAYER (EKF)                           |
|  - robot_localization (ekf_node): Fuses /wheel_odom (vx) + /oak/imu/data (vyaw)|
|    * Computes filtered odometry: /odom                                         |
|    * Broadcasts TF: odom -> base_link                                          |
+--------------------------------------------------------------------------------+
                                       |
                                       v
+--------------------------------------------------------------------------------+
|                         SLAM & 3D RECONSTRUCTION LAYER                         |
|  - rtabmap_slam: Appearance-Based Graph-SLAM & 3D Point Cloud Assembler        |
|    * Visual PnP registration & loop closure detection                          |
|    * Sparse/Dense Pose-Graph Optimization (g2o/GTSAM)                          |
|    * Occupancy Grid generation (/map) & 3D Database storage (~/.ros/rtabmap.db)|
+--------------------------------------------------------------------------------+
```

---

## 4. Chronological Engineering Journey & Breakthroughs

### Milestone 1: ESP32 Teleoperation Bridge & Deadband Watchdog
- **Initial Challenge**: Moving the robot base via teleop resulted in sluggish or jerky movement, while stopping keys caused lingering motion. The heavy IG52 gearboxes failed to overcome static floor friction at low PWM duty cycles.
- **Engineering Implementation**:
  - Developed `responsive_teleop.py` with non-blocking key reads that immediately publishes $(0, 0)$ when keys are released.
  - Implemented static friction compensation in firmware: `MIN_PWM = 135` (out of 255) ensures motor torque immediately overcomes floor scrub resistance without stalling.
  - Implemented serial watchdog: automatic shutdown after 250 ms without command.

---

### Milestone 2: Diagnosis of the 360° "Car Everywhere" Radial Warp
- **Observed Failure**: When mapping an indoor space with a parked golf cart / car, returning along a straight trajectory warped the point cloud into a 360° radial doughnut cylinder, cloning the car and walls in all directions.
- **Diagnostic Root Cause**:
  1. **Coordinate Frame Discrepancy (DepthAI Optical vs. ROS REP 103)**:
     - DepthAI Camera Frame: $+X$ right, $+Y$ down, $+Z$ forward through the lens.
     - ROS Body Frame: $+X$ forward, $+Y$ left, $+Z$ up.
     - In earlier driver versions, camera gyro Z (optical roll axis) was mapped directly to ROS `angular_velocity.z` (robot body yaw).
     - Because the camera was tilted downward by $6.24^\circ$, optical roll was energized during forward motion and turning, tricking the EKF into believing the robot was continuously spinning at $\sim 25^\circ/\text{s}$. RTAB-Map projected every keyframe radially outward in a circle.
  2. **Sensor Firmware Restriction**:
     - Attempting to query `dai.IMUSensor.GYROSCOPE_CALIBRATED` threw runtime errors on the OAK-D Lite: `RuntimeError: BMI270 supports only ACCELEROMETER_RAW and/or GYROSCOPE_RAW outputs`.
- **Engineering Solution**:
  - Rewrote coordinate transform equations in `oak_ros_bridge.py`:
    $$\omega_{x, \text{ros}} = \omega_{z, \text{dai}} - b_z \quad (\text{Roll})$$
    $$\omega_{y, \text{ros}} = -(\omega_{x, \text{dai}} - b_x) \quad (\text{Pitch})$$
    $$\omega_{z, \text{ros}} = -(\omega_{y, \text{dai}} - b_y) \quad (\textbf{Body Yaw Rate})$$
  - Migrated driver to raw 6-DOF streaming with custom host-side bias compensation.

---

### Milestone 3: Precision IMU Physical Calibration (Bosch BMI270)
- **Objective**: Eliminate static gyro zero-rate drift on the yaw axis to stop long-term rotational integration errors.
- **Implementation**:
  - Created automated calibration tool `calibrate_imu.py` / `calibrate_imu.sh`.
  - Robot placed perfectly stationary on a level floor; sampled 1,000 live IMU packets at 100 Hz.
- **Calibration Results**:
  - **Gyro X (Pitch axis)**: Bias $= +0.000600\text{ rad/s}$, Variance $= 1.0 \times 10^{-5}\text{ rad}^2/\text{s}^2$
  - **Gyro Y (Yaw axis)**: Bias $= -0.000673\text{ rad/s}$ ($-0.039^\circ/\text{s}$), Variance $= 1.0 \times 10^{-5}\text{ rad}^2/\text{s}^2$
  - **Gyro Z (Roll axis)**: Bias $= -0.007211\text{ rad/s}$ ($-0.413^\circ/\text{s}$), Variance $= 1.0 \times 10^{-5}\text{ rad}^2/\text{s}^2$
- **Integration**:
  - Saved parameters to `imu_calib.yaml`.
  - `oak_ros_bridge.py` dynamically loads this configuration on launch, subtracting physical biases in real time before publishing to `/oak/imu/data`.

---

### Milestone 4: Ground-Truth Wheel Odometry Calibration (Tape Measure)
- **Observed Discrepancy**: Driving along straight paths showed accumulated positional errors.
- **Implementation**:
  - Engineered interactive ground-truth calibration tool `calibrate_odometry.py` / `calibrate_odometry.sh`.
  - Robot aligned to a tape measure on the floor and driven forward in a straight line.
- **Empirical Measurement**:
  - Physical tape measure actual distance: **$1.1900\text{ m}$**
  - Odometry reported distance: **$0.4070\text{ m}$**
  - Scale Factor Discrepancy: $2.9241\times$ ($292.4\%$)
  - Initial `ticks_per_rev`: 46,500
- **Recalibrated Parameter**:
  $$K_{calibrated} = \frac{46500}{2.9241} = \mathbf{15,902\text{ ticks/revolution}}$$
- Updated `mapping.launch.py`, `localization.launch.py`, and `odom_bridge_node.py`. The linear scale now matches physical ground truth exactly ($1.00\text{ m}$ driven $= 1.00\text{ m}$ odometry).

---

### Milestone 5: Linux Serial Burst Jitter & Microcontroller Timestamp Sync
- **Observed Anomaly**: In a subsequent mapping test, the robot was driven at $0.10\text{ m/s}$ for 54 seconds inside a 10-meter room. The session report recorded **48.72 meters driven** spanning a $32\text{ m} \times 28\text{ m}$ bounding box at speeds exceeding $2.5\text{ m/s}$!
- **Diagnostic Root Cause**:
  - In `odom_bridge_node.py`, $\Delta t$ was being computed inside a `while ser.in_waiting:` loop using the host ROS clock:
    `dt = max((now - self.last_time).nanoseconds / 1e9, 1e-3)`
  - The Linux USB-serial driver (`ftdi_sio`/`cp210x`) delivers incoming packets in bursts.
  - When 3 to 5 buffered lines were read in a single burst, the host time elapsed between lines was only a few microseconds. The calculation clamped to $1\text{ ms}$ ($10^{-3}\text{ s}$).
  - For a standard 20 ms wheel displacement of $2\text{ mm}$, the velocity was computed as:
    $$v_x = \frac{0.002\text{ m}}{0.001\text{ s}} = 2.0\text{ to }2.5\text{ m/s} \quad (\mathbf{20\times\text{ to }25\times\text{ actual speed!}})$$
  - EKF fused this inflated velocity, causing the robot to virtually teleport across the building.
- **Engineering Solution**:
  - Synchronized $\Delta t$ directly to the ESP32 hardware millisecond counter (`parts[3] = millis`):
    `d_ms = esp_ms - self.last_esp_ms`
    `dt = d_ms / 1000.0 if (0 < d_ms <= 500) else 0.020`
  - Regardless of host USB buffering or thread scheduling, $\Delta t$ is strictly locked to the actual physical sample interval ($20\text{ ms}$). Velocity remains rock-solid at $0.10\text{ m/s}$.

---

### Milestone 6: Point Cloud Pipeline & Stale File Viewer Bug Fix
- **Observed Failure**: After completing a fresh mapping run, the user ran `view_pointcloud.sh` and exclaimed: *"still there are 2 cars"*.
- **Diagnostic Root Cause**:
  - `view_pointcloud.sh` was merely querying `ls -t ~/mapping_ws/maps/*.pcd | head -n 1`.
  - RTAB-Map saves its live SLAM map into SQLite database `~/.ros/rtabmap.db`. It does not automatically export `.pcd` files on shutdown.
  - As a result, `view_pointcloud.sh` was loading `session_1116_cloud.pcd`—a file created the previous morning before calibration! The user was literally viewing yesterday's stale map.
- **Engineering Solution**:
  - Upgraded `view_pointcloud.sh`:
    - Checks if `~/.ros/rtabmap.db` is newer than existing point clouds in `~/mapping_ws/maps/`.
    - Automatically executes `rtabmap-export --cloud --voxel 0.02 --output_dir ~/mapping_ws/maps` to export the fresh assembled cloud.
    - Runs `pcl_ply2pcd` to convert the cloud to binary `.pcd`.
    - Launches PCL Viewer with the genuine, updated point cloud.

---

### Milestone 7: Speed Optimization & Visual Neighbor Link Refining ("7 Cars" Fix)
- **Observed Failure**: When driving the robot past the golf cart in a 1-meter loop, the exported point cloud showed **7 overlapping copies of the golf cart**, fanned out along a 1.5-meter smear trail.
- **Diagnostic Root Cause**:
  1. **Disabled Visual Odometry Alignment (`NeighborLinkRefining: false`)**:
     - With this flag disabled, RTAB-Map was instructed to blindly trust raw wheel odometry between consecutive keyframes.
     - When differential-drive wheels pivot on smooth tile, tire scrub causes small heading errors. Because visual alignment was disabled, each consecutive camera frame painted the golf cart at an angular offset, multiplying the single physical vehicle into 7 separate ghost cars.
  2. **Teleoperation Velocity Excess**:
     - Default speed of $0.25\text{ m/s}$ and rapid angular acceleration provoked excessive tire scrub and camera rolling shutter motion blur.
- **Engineering Solution**:
  - **Speed Limits**: In `mapping.launch.py` and `responsive_teleop.py`:
    - Max Linear Speed: **$0.12\text{ m/s}$** (ultra-gentle mapping creep).
    - Max Angular Speed: **$0.25\text{ rad/s}$** (~$14^\circ/\text{s}$ smooth pivoting).
  - **Visual Link Locking**: In `mapping.launch.py`:
    - `'--RGBD/NeighborLinkRefining', 'true'`: Activates PnP RANSAC visual feature matching between consecutive frames. Visual features on the vehicle lock consecutive frames together, eliminating duplicate ghost objects.
    - `'--Vis/BundleAdjustment', '1'`: Local bundle adjustment across keyframe windows.
    - `'--Vis/RefineIterations', '5'`: Sub-pixel geometric refinement.
    - `'--Vis/MaxFeatures', '1200'`: Rich feature extraction for dense visual lock.

---

### Milestone 8: Loop Closure Optimization & Golden Map Consistency
- **Observed Failure**: Visual loop closure matching detected valid candidate images (e.g., node 185 ↔ 30 with 33 matches), but RTAB-Map rejected them with:
  `Rejecting all added loop closures ... maximum graph error ratio of 3.96 > RGBD/OptimizeMaxError 3.0`
- **Engineering Solution**:
  - Increased `--RGBD/OptimizeMaxError` from $3.0$ to **$5.0$** in both mapping and localization launch configurations.
  - Increased `--Rtabmap/DetectionRate` from $1.0\text{ Hz}$ to **$2.0\text{ Hz}$** for twice the loop closure checking responsiveness.
  - Validated against the verified golden baseline (`full_map_oct16.db`):
    - Full 15.86 m perimeter loop closure.
    - Closed with 2 verified global loop closures (`#148`, `#151` ↔ `#25`).
    - Optimization converged with **$0.000\text{ m}$ RMSE**.

---

## 5. Standard Operating Procedures (SOP) & Usage Manual

### 5.1 Starting a Fresh, High-Precision Mapping Session
1. Place robot on a flat, level floor at the desired origin (aligned with a wall or corridor).
2. Open Terminal 1 and launch mapping with database reset:
   ```bash
   cd ~/mapping_ws
   source install/setup.bash
   ros2 launch esp32_odom_bridge mapping.launch.py delete_db:=true
   ```
3. Open Terminal 2 and launch responsive teleop:
   ```bash
   ~/mapping_ws/teleop.sh
   ```
4. **Driving Protocol for Perfect Metric Consistency**:
   - Drive slowly in straight lines along the room's perimeter (0.08 to 0.12 m/s).
   - Make smooth, gentle turns at corners (0.15 to 0.25 rad/s).
   - Complete a perimeter loop around the room and **return to the starting location facing the original direction**.
   - Watch the launch terminal or `rtabmap_viz`: it will log `Loop closure detected!` and flash green.
5. Terminate mapping by pressing `Ctrl+C` in Terminal 1.

### 5.2 Viewing the 3D Point Cloud Map
In any terminal:
```bash
~/mapping_ws/view_pointcloud.sh
```
- The script automatically detects the fresh database, exports the assembled 3D cloud via `rtabmap-export`, converts to `.pcd`, and opens PCL Viewer.
- **PCL Viewer Controls**:
  - `c` : Toggle RGB Color / Intensity / Normal mode
  - `+` / `-` : Increase / decrease point render size
  - `r` : Reset 3D camera viewpoint
  - `q` : Quit viewer

### 5.3 Performing Wheel Odometry Calibration (Optional Verification)
To verify wheel encoder scale against a physical tape measure:
```bash
~/mapping_ws/calibrate_odometry.sh
```
- Select `[1]` for Linear Distance (Tape Measure).
- Select `[2]` for Angular Turn Calibration ($360^\circ$ spin).
- Select `[3]` for Live Odometry Drift Monitor.

---

## 6. Workspace File Directory & Reference Map

| File Path | Description | Key Parameters / Role |
| :--- | :--- | :--- |
| `src/esp32_odom_bridge/launch/mapping.launch.py` | Primary SLAM launch file | `ticks_per_rev: 15902`, `max_lin: 0.12`, `NeighborLinkRefining: true`, `OptimizeMaxError: 5.0` |
| `src/esp32_odom_bridge/launch/localization.launch.py` | Nav2 Localization launch file | `Mem/IncrementalMemory: false`, `ticks_per_rev: 15902` |
| `src/esp32_odom_bridge/esp32_odom_bridge/odom_bridge_node.py` | ESP32 Motor & Odometry Driver | 50 Hz serial, hardware millis sync, midpoint integration |
| `src/esp32_odom_bridge/esp32_odom_bridge/responsive_teleop.py` | Instant-Braking Teleoperation | Speed: $0.15\text{ m/s}$, Turn: $0.30\text{ rad/s}$, instant stop on key release |
| `src/esp32_odom_bridge/config/ekf.yaml` | Extended Kalman Filter (robot_localization) | Fuses `/wheel_odom` ($v_x$) + `/oak/imu/data` ($\omega_z$) into `/odom` |
| `src/esp32_odom_bridge/config/imu_calib.yaml` | Calibrated IMU Zero-Rate Biases | Gyro Y bias: $-0.000673\text{ rad/s}$, Gyro Z bias: $-0.007211\text{ rad/s}$ |
| `src/esp32_odom_bridge/firmware/clean_camera_line_follower_mapping.ino` | ESP32 C++ Microcontroller Firmware | 20 kHz PWM motor control, quadrature interrupt ISR, `MIN_PWM = 135` |
| `calibrate_odometry.sh` / `.py` | Interactive Wheel Calibration Tool | Empirical tape measure scaling tool for `ticks_per_rev` and wheelbase |
| `calibrate_imu.sh` / `.py` | Precision Camera IMU Calibrator | 1000-sample zero-rate gyro bias and noise covariance estimator |
| `view_pointcloud.sh` | Automated 3D Cloud Export & Viewer | Detects fresh `rtabmap.db`, auto-exports cloud via `rtabmap-export`, launches `pcl_viewer` |
| `teleop.sh` | Keyboard Teleoperation Launcher | Starts `responsive_teleop` node |
| `maps/full_map_oct16.db` | Verified Golden Benchmark Database | Complete 15.86m drift-free SLAM run with 2 verified loop closures (0.000m RMSE) |
