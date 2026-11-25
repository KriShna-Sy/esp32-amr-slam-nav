#!/usr/bin/env python3
"""
odom_bridge_node.py
Bridges the ESP32 firmware (clean_camera_line_follower_mapping.ino) to ROS 2:
  - forwards /cmd_vel to the ESP32 as "lin,ang\n"
  - reads "ODO,<left_ticks>,<right_ticks>,<millis>\n" telemetry
  - integrates differential-drive odometry and publishes nav_msgs/Odometry
    + the odom -> base_link TF, for RTAB-Map / ICP localization to consume.

All hardware-specific numbers are ROS 2 parameters (see launch/mapping.launch.py)
so this same package runs unchanged on the laptop now and the Jetson later.
"""
import math
import time
import glob
import os
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ESP32OdomBridge(Node):
    def __init__(self):
        super().__init__('esp32_odom_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheelbase', 0.5458)        # must match WHEELBASE in the .ino
        self.declare_parameter('wheel_diameter', 0.127)   # 5 inch (127 mm) wheel
        self.declare_parameter('ticks_per_rev', 47000)   # new motors: ~47000 ticks/wheel rev (hand-measured)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_linear_speed', 0.60)   # m/s max (full teleop speed range)
        self.declare_parameter('max_angular_speed', 1.00)  # rad/s max (full teleop turn range)
        self.declare_parameter('linear_scale', 1.0)        # 1.0 = direct velocity from Nav2 / teleop
        self.declare_parameter('angular_scale', 1.0)       # 1.0 = direct angular velocity
        self.declare_parameter('publish_tf', True)         # set False when visual odom owns odom→base_link
        self.declare_parameter('cmd_timeout', 0.25)        # 250ms watchdog: stop immediately when key released

        self.serial_port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        self.ser = None
        self._connect_serial()

        self.last_lt = 0
        self.last_rt = 0
        self.last_esp_ms = 0
        self.first_reading = True
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.tf_bcast = TransformBroadcaster(self)
        self.last_teleop_time = 0.0
        self.last_cmd_time = 0.0
        self.last_sent_lin = 0.0
        self.last_sent_ang = 0.0
        # Teleop manual drive (/cmd_vel) - Priority 1 (overrides Nav2)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_stamped_cb, 10)

        # Nav2 Autonomous Driving (/cmd_vel_safe from collision_monitor) - Priority 2
        # collision_monitor verifies scan at 20 Hz and guarantees E-stop before obstacles
        self.create_subscription(Twist, '/cmd_vel_safe', self.cmd_vel_nav_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_safe', self.cmd_vel_nav_stamped_cb, 10)
        self.create_timer(0.02, self.poll_serial)  # 50 Hz, matches firmware telemetry rate

        self.get_logger().info(
            f'esp32_odom_bridge up on {self.serial_port}. '
            f'meters_per_tick={self.meters_per_tick:.6f} '
            f'(wheel_dia={self.wheel_diameter}, ticks_per_rev={self.ticks_per_rev}) '
            f'-- slow speed mode: max_lin={self.max_linear_speed}m/s, max_ang={self.max_angular_speed}rad/s'
        )

    def _connect_serial(self):
        candidate_ports = [self.serial_port]
        found_ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        for p in found_ports:
            if p not in candidate_ports:
                candidate_ports.append(p)

        last_err = None
        for port in candidate_ports:
            if not os.path.exists(port):
                continue
            try:
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = serial.Serial()
                self.ser.port = port
                self.ser.baudrate = self.baud
                self.ser.timeout = 0.05
                self.ser.dtr = False
                self.ser.rts = False
                self.ser.open()
                self.ser.write(b'\n')
                self.ser.reset_input_buffer()
                self.ser.write(b'R\n')  # zero ticks at node start = clean run
                # ESP32 ticks are now 0 -> re-baseline. Without this, the first reading after a
                # USB reconnect is compared with the old (huge) count -> robot teleports.
                self.first_reading = True
                self.serial_port = port
                self.get_logger().info(f"Successfully connected to serial port {port}")
                return True
            except (serial.SerialException, OSError) as e:
                last_err = e
                self.ser = None

        self.get_logger().warn(
            f"No serial port available (tried {candidate_ports}): {last_err}. Check USB cable/power to ESP32!",
            throttle_duration_sec=3.0
        )
        return False

    def _send_velocity(self, vx: float, wz: float):
        if not self.ser or not self.ser.is_open:
            if not self._connect_serial():
                return

        # Boost angular velocity when turning in place (vx == 0) to overcome floor scrub friction
        eff_wz = wz
        if abs(vx) < 0.02 and abs(wz) > 0.02:
            # Firmware v3.3 (closed-loop + soft-start) turns reliably from 0.05 m/s per wheel
            # = 0.18 rad/s. The old 0.32 boost overshot the final heading and the robot
            # oscillated left/right at the goal instead of stopping.
            min_pivot_wz = 0.18
            if 0 < eff_wz < min_pivot_wz:
                eff_wz = min_pivot_wz
            elif -min_pivot_wz < eff_wz < 0:
                eff_wz = -min_pivot_wz

        lin = max(-self.max_linear_speed, min(self.max_linear_speed, vx * self.linear_scale))
        ang = max(-self.max_angular_speed, min(self.max_angular_speed, eff_wz * self.angular_scale))
        try:
            self.ser.write(f"{lin:.3f},{ang:.3f}\n".encode())
            self.last_sent_lin = lin
            self.last_sent_ang = ang
            self.get_logger().info(
                f"[CMD] lin={lin:.3f} m/s, ang={ang:.3f} rad/s (raw_vx={vx:.3f}, raw_wz={wz:.3f})",
                throttle_duration_sec=0.5
            )
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"Serial write error: {e}, attempting reconnect...", throttle_duration_sec=2.0)
            self._connect_serial()

    def cmd_vel_cb(self, msg: Twist):
        # Priority 1: User / Teleop direct control (Twist)
        self.last_teleop_time = time.time()
        self.last_cmd_time = time.time()
        self._send_velocity(msg.linear.x, msg.angular.z)

    def cmd_vel_stamped_cb(self, msg: TwistStamped):
        # Priority 1: User / Teleop direct control (TwistStamped)
        self.last_teleop_time = time.time()
        self.last_cmd_time = time.time()
        self._send_velocity(msg.twist.linear.x, msg.twist.angular.z)

    def cmd_vel_nav_cb(self, msg: Twist):
        # Priority 2: Nav2 Autonomous Driving (Twist)
        if (time.time() - self.last_teleop_time) > 0.5:
            self.last_cmd_time = time.time()
            self._send_velocity(msg.linear.x, msg.angular.z)

    def cmd_vel_nav_stamped_cb(self, msg: TwistStamped):
        # Priority 2: Nav2 Autonomous Driving (TwistStamped)
        if (time.time() - self.last_teleop_time) > 0.5:
            self.last_cmd_time = time.time()
            self._send_velocity(msg.twist.linear.x, msg.twist.angular.z)

    def _publish_hold(self):
        # No wheel data (USB lost / ESP32 rebooting): the ESP32's own 400 ms watchdog has stopped
        # the motors, so tell the EKF "standing still". Without this the EKF keeps extrapolating
        # the last velocity and the robot 'drives' metres away on the map (seen: 4.9 -> 10.7 m).
        now = self.get_clock().now()
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.th)
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.001
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.005
        odom.twist.covariance[0] = 0.002
        odom.twist.covariance[7] = odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.005
        self.odom_pub.publish(odom)
        self.last_odo_pub = time.time()

    def poll_serial(self):
        if time.time() - getattr(self, 'last_odo_pub', 0.0) > 0.2:
            self._publish_hold()
            self.get_logger().warn("No wheel odometry for >0.2 s - publishing zero velocity (robot assumed stopped)",
                                   throttle_duration_sec=5.0)
        # Watchdog: automatically stop motors if no velocity command has arrived recently (250ms for responsive braking)
        if self.last_cmd_time > 0 and (time.time() - self.last_cmd_time) > self.cmd_timeout:
            if abs(self.last_sent_lin) > 0.001 or abs(self.last_sent_ang) > 0.001:
                self._send_velocity(0.0, 0.0)

        if not self.ser or not self.ser.is_open:
            self._connect_serial()
            return

        try:
            while self.ser.in_waiting:
                raw = self.ser.readline().decode(errors='ignore').strip()
                if not raw.startswith("ODO,"):
                    continue
                parts = raw.split(",")
                if len(parts) != 4:
                    continue
                try:
                    lt, rt = int(parts[1]), int(parts[2])
                    esp_ms = int(parts[3])
                except (ValueError, IndexError):
                    continue

                if self.first_reading:
                    self.last_lt, self.last_rt = lt, rt
                    self.last_esp_ms = esp_ms
                    self.first_reading = False
                    continue

                # ESP32 rebooted (its millis() went backwards) -> re-baseline, no jump
                if esp_ms < self.last_esp_ms:
                    self.get_logger().warn("ESP32 reset detected (millis went backwards) - re-baselining odometry")
                    self.last_lt, self.last_rt = lt, rt
                    self.last_esp_ms = esp_ms
                    continue

                # Physically impossible step (> 1.5 m/s or > 4 rad/s) -> discard, re-baseline
                d_ms_chk = max(esp_ms - self.last_esp_ms, 1)
                step_m = max(abs(lt - self.last_lt), abs(rt - self.last_rt)) * self.meters_per_tick
                if step_m / (d_ms_chk / 1000.0) > 1.5 and step_m > 0.05:
                    self.get_logger().warn(
                        f"Rejected impossible odometry step ({step_m:.2f} m in {d_ms_chk} ms) - re-baselining")
                    self.last_lt, self.last_rt = lt, rt
                    self.last_esp_ms = esp_ms
                    continue

                # Invert signs because forward wheel rotation generates negative ticks
                dlt, drt = -(lt - self.last_lt), -(rt - self.last_rt)
                self.last_lt, self.last_rt = lt, rt

                dl = dlt * self.meters_per_tick
                dr = drt * self.meters_per_tick
                dcenter = (dl + dr) / 2.0
                dth = (dr - dl) / self.wheelbase

                now = self.get_clock().now()
                # Compute exact physical dt from ESP32 hardware millisecond clock
                # This prevents USB serial buffer burst jitter from inflating velocity
                d_ms = esp_ms - self.last_esp_ms
                if d_ms <= 0 or d_ms > 500:
                    dt = 0.020  # Nominal 50 Hz (20 ms) fallback
                else:
                    dt = d_ms / 1000.0
                self.last_esp_ms = esp_ms
                self.last_time = now

                # midpoint integration for better accuracy on curves
                self.x += dcenter * math.cos(self.th + dth / 2.0)
                self.y += dcenter * math.sin(self.th + dth / 2.0)
                self.th += dth

                q = yaw_to_quaternion(self.th)

                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.base_frame
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation = q
                odom.twist.twist.linear.x = dcenter / dt
                odom.twist.twist.angular.z = dth / dt

                # Diagonal covariances for 2D differential drive robot
                odom.pose.covariance = [
                    0.001, 0.0,   0.0,   0.0,   0.0,   0.0,
                    0.0,   0.001, 0.0,   0.0,   0.0,   0.0,
                    0.0,   0.0,   1e6,   0.0,   0.0,   0.0,
                    0.0,   0.0,   0.0,   1e6,   0.0,   0.0,
                    0.0,   0.0,   0.0,   0.0,   1e6,   0.0,
                    0.0,   0.0,   0.0,   0.0,   0.0,   0.005
                ]
                odom.twist.covariance = [
                    0.002, 0.0,   0.0,   0.0,   0.0,   0.0,
                    0.0,   1e6,   0.0,   0.0,   0.0,   0.0,
                    0.0,   0.0,   1e6,   0.0,   0.0,   0.0,
                    0.0,   0.0,   0.0,   1e6,   0.0,   0.0,
                    0.0,   0.0,   0.0,   0.0,   1e6,   0.0,
                    0.0,   0.0,   0.0,   0.0,   0.0,   0.005
                ]
                self.odom_pub.publish(odom)
                self.last_odo_pub = time.time()

                if self.publish_tf:
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = now.to_msg()
                    tf_msg.header.frame_id = self.odom_frame
                    tf_msg.child_frame_id = self.base_frame
                    tf_msg.transform.translation.x = self.x
                    tf_msg.transform.translation.y = self.y
                    tf_msg.transform.rotation = q
                    self.tf_bcast.sendTransform(tf_msg)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"Serial read error: {e}, reconnecting...", throttle_duration_sec=2.0)
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self._connect_serial()

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b'0.0,0.0\n')
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ESP32OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
