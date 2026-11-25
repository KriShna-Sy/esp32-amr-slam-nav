#!/home/krishna/ros_oak_env/bin/python3
"""
OAK Camera IMU Precision Calibration Tool
-------------------------------------------
Connects to Luxonis OAK camera, samples calibrated gyroscope and accelerometer
readings while stationary, computes zero-rate biases and sensor noise covariances,
and writes the calibration configuration to imu_calib.yaml for EKF fusion.
"""

import os
import sys
import time
import math
import yaml
import numpy as np

# Ensure DepthAI is imported from the environment if needed
try:
    import depthai as dai
except ImportError:
    print("Error: depthai not found in current Python path.")
    print("Please run this script using ~/ros_oak_env/bin/python3")
    sys.exit(1)


def main():
    print("=" * 65)
    print("      OAK CAMERA IMU PRECISION CALIBRATION TOOL")
    print("=" * 65)
    print("\n[INFO] Searching for connected Luxonis OAK camera...")

    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("[ERROR] No DepthAI devices found!")
        print("Please check that the OAK camera USB-3 cable is firmly plugged in.")
        sys.exit(1)

    print(f"[INFO] Found {len(devices)} device(s): {devices[0].getMxId()}")
    print("\n-------------------------------------------------------------")
    print("  CALIBRATION INSTRUCTIONS:")
    print("  1. Place the robot stationary on a flat, level floor.")
    print("  2. Do NOT touch, push, or vibrate the robot during calibration.")
    print("  3. Ensure motors are stopped (no teleop commands).")
    print("-------------------------------------------------------------\n")

    input("Press ENTER when the robot is completely stationary to start calibration...")

    # Build DepthAI IMU Pipeline
    pipeline = dai.Pipeline()
    try:
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        xout_imu = pipeline.create(dai.node.XLinkOut)
        xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)
    except Exception as e:
        print(f"[ERROR] Failed to configure DepthAI IMU pipeline: {e}")
        sys.exit(1)

    TOTAL_SAMPLES = 1000
    print(f"\n[INFO] Connecting to device and collecting {TOTAL_SAMPLES} samples (~10 seconds)...")

    gyro_samples_x = []
    gyro_samples_y = []
    gyro_samples_z = []
    accel_samples_x = []
    accel_samples_y = []
    accel_samples_z = []

    with dai.Device(pipeline) as device:
        q_imu = device.getOutputQueue("imu", maxSize=20, blocking=True)
        print("[INFO] Device connected. Warm-up 1 second...")
        time.sleep(1.0)
        print("[INFO] Collecting calibration data now...\n")

        start_time = time.time()
        while len(gyro_samples_z) < TOTAL_SAMPLES:
            in_imu = q_imu.get()
            for packet in in_imu.packets:
                g = packet.gyroscope
                a = packet.acceleroMeter
                gyro_samples_x.append(g.x)
                gyro_samples_y.append(g.y)
                gyro_samples_z.append(g.z)
                accel_samples_x.append(a.x)
                accel_samples_y.append(a.y)
                accel_samples_z.append(a.z)

                count = len(gyro_samples_z)
                if count % 100 == 0 or count == TOTAL_SAMPLES:
                    pct = int(count / TOTAL_SAMPLES * 100)
                    bar = "=" * (pct // 5) + "-" * (20 - (pct // 5))
                    print(f"\r  [{bar}] {pct:3d}% ({count}/{TOTAL_SAMPLES} samples)", end="", flush=True)

                if count >= TOTAL_SAMPLES:
                    break

        elapsed = time.time() - start_time
        print(f"\n\n[INFO] Data collection complete in {elapsed:.2f} seconds.")

    # Calculate statistics
    gx = np.array(gyro_samples_x)
    gy = np.array(gyro_samples_y)
    gz = np.array(gyro_samples_z)

    ax = np.array(accel_samples_x)
    ay = np.array(accel_samples_y)
    az = np.array(accel_samples_z)

    bias_gx = float(np.mean(gx))
    bias_gy = float(np.mean(gy))
    bias_gz = float(np.mean(gz))

    var_gx = float(np.var(gx))
    var_gy = float(np.var(gy))
    var_gz = float(np.var(gz))

    mean_ax = float(np.mean(ax))
    mean_ay = float(np.mean(ay))
    mean_az = float(np.mean(az))

    var_ax = float(np.var(ax))
    var_ay = float(np.var(ay))
    var_az = float(np.var(az))

    # Derived ROS Body Yaw Rate (ROS Z = -DepthAI Y)
    ros_yaw_rate_bias = -bias_gy
    ros_yaw_rate_std = math.sqrt(var_gy)

    # Inferred tilt from gravity vector:
    # In DepthAI frame, Gravity vector points in +Y direction (approx +9.81 m/s^2)
    # Pitch angle = atan2(az, -ay)
    total_accel = math.sqrt(mean_ax**2 + mean_ay**2 + mean_az**2)
    inferred_pitch_rad = math.atan2(mean_az, -mean_ay)
    inferred_pitch_deg = math.degrees(inferred_pitch_rad)

    print("\n" + "=" * 65)
    print("                 CALIBRATION RESULTS")
    print("=" * 65)
    print("DepthAI Frame Biases (rad/s):")
    print(f"  Gyro X (Pitch axis) : {bias_gx:+.6f} rad/s ({math.degrees(bias_gx):+.3f} deg/s)")
    print(f"  Gyro Y (Yaw axis)   : {bias_gy:+.6f} rad/s ({math.degrees(bias_gy):+.3f} deg/s)")
    print(f"  Gyro Z (Roll axis)  : {bias_gz:+.6f} rad/s ({math.degrees(bias_gz):+.3f} deg/s)")
    print("\nDepthAI Frame Variance (rad^2/s^2):")
    print(f"  Var X: {var_gx:.8f} (std: {math.sqrt(var_gx):.5f})")
    print(f"  Var Y: {var_gy:.8f} (std: {math.sqrt(var_gy):.5f})")
    print(f"  Var Z: {var_gz:.8f} (std: {math.sqrt(var_gz):.5f})")
    print("\nROS Body Yaw Verification (ROS Z = -DepthAI Y):")
    print(f"  Residual Yaw Rate Bias: {ros_yaw_rate_bias:+.6f} rad/s ({math.degrees(ros_yaw_rate_bias):+.3f} deg/s)")
    print(f"  Sensor Yaw Noise (1σ) : {ros_yaw_rate_std:.6f} rad/s ({math.degrees(ros_yaw_rate_std):.3f} deg/s)")
    print("\nAccelerometer & Tilt Check:")
    print(f"  Mean Accel: [{mean_ax:+.3f}, {mean_ay:+.3f}, {mean_az:+.3f}] m/s^2 (Total: {total_accel:.2f} m/s^2)")
    print(f"  Inferred Camera Pitch Angle: {inferred_pitch_deg:+.2f} deg ({inferred_pitch_rad:+.4f} rad)")
    print("=" * 65)

    calib_data = {
        'calibration_timestamp': time.time(),
        'samples_count': TOTAL_SAMPLES,
        'depthai_gyro_bias': {
            'x': bias_gx,
            'y': bias_gy,
            'z': bias_gz,
        },
        'depthai_gyro_variance': {
            'x': max(var_gx, 1e-5),
            'y': max(var_gy, 1e-5),
            'z': max(var_gz, 1e-5),
        },
        'depthai_accel_mean': {
            'x': mean_ax,
            'y': mean_ay,
            'z': mean_az,
        },
        'depthai_accel_variance': {
            'x': max(var_ax, 1e-4),
            'y': max(var_ay, 1e-4),
            'z': max(var_az, 1e-4),
        },
        'ros_body_mapping': {
            'yaw_rate_bias': ros_yaw_rate_bias,
            'yaw_rate_variance': max(var_gy, 1e-5),
            'inferred_pitch_rad': inferred_pitch_rad,
            'inferred_pitch_deg': inferred_pitch_deg,
        }
    }

    # Save to both package config and workspace root
    save_paths = [
        os.path.expanduser('~/mapping_ws/src/esp32_odom_bridge/config/imu_calib.yaml'),
        os.path.expanduser('~/mapping_ws/imu_calib.yaml'),
    ]

    for p in save_paths:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, 'w') as f:
            yaml.dump(calib_data, f, default_flow_style=False)
        print(f"[SUCCESS] Saved calibration to {p}")

    print("\n[READY] The calibrated IMU bias & variance are now saved and will be loaded automatically by oak_ros_bridge.py.")


if __name__ == '__main__':
    main()
