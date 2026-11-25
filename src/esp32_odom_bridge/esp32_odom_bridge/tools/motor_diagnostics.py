#!/usr/bin/env python3
"""
motor_diagnostics.py
Directly tests Left & Right motor drivers and encoder feedback on the ESP32 base.
Run:
  ros2 run esp32_odom_bridge motor_diagnostics [/dev/ttyUSB0]
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUD = 115200
WHEELBASE = 0.56

def main():
    print("=" * 65)
    print("      ESP32 MOTOR & ENCODER HARDWARE DIAGNOSTIC TOOL")
    print("=" * 65)
    print(f"Opening serial port {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] Could not open {PORT}: {e}")
        return

    time.sleep(1.0)
    ser.reset_input_buffer()

    def get_latest_odo(duration=0.5):
        start = time.time()
        last_lt, last_rt = None, None
        while time.time() - start < duration:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("ODO,"):
                parts = line.split(",")
                if len(parts) == 4:
                    try:
                        last_lt, last_rt = int(parts[1]), int(parts[2])
                    except ValueError:
                        pass
        return last_lt, last_rt

    lt0, rt0 = get_latest_odo(1.0)
    print(f"\n[CURRENT ENCODERS] Left Ticks: {lt0}, Right Ticks: {rt0}")
    print("\n-----------------------------------------------------------")
    print("TEST 1: Rotate RIGHT WHEEL by hand")
    print("Rotate the right wheel by hand now for 4 seconds...")
    print("-----------------------------------------------------------")
    time.sleep(4.0)
    lt1, rt1 = get_latest_odo(0.5)
    d_rt1 = (rt1 - rt0) if (rt1 is not None and rt0 is not None) else None
    print(f"Right Ticks before: {rt0} -> after: {rt1} (Delta: {d_rt1 if d_rt1 is not None else 'N/A'})")
    if d_rt1 is not None and abs(d_rt1) > 20:
        print(">>> [PASS] Right encoder is DETECTING motion correctly!")
    else:
        print(">>> [NOTE] Right encoder did not detect movement (0 delta or no signal).")

    print("\n-----------------------------------------------------------")
    print("TEST 2: Powering LEFT MOTOR ONLY (Forward at moderate speed, 1.5s)")
    print("-----------------------------------------------------------")
    # Command: lin=0.10, ang=-0.357 -> ls=0.20, rs=0.0
    start = time.time()
    while time.time() - start < 1.5:
        ser.write(b"0.100,-0.357\n")
        time.sleep(0.05)
    ser.write(b"0.000,0.000\n")
    time.sleep(0.5)
    lt2, rt2 = get_latest_odo(0.5)
    d_lt2 = (lt2 - lt1) if (lt2 is not None and lt1 is not None) else None
    print(f"Left Ticks delta: {d_lt2 if d_lt2 is not None else 'N/A'}")

    print("\n-----------------------------------------------------------")
    print("TEST 3: Powering RIGHT MOTOR at MODERATE speed (rs = +0.20 m/s, 1.5s)")
    print("-----------------------------------------------------------")
    # Command: lin=0.10, ang=0.357 -> ls=0.0, rs=0.20
    start = time.time()
    while time.time() - start < 1.5:
        ser.write(b"0.100,0.357\n")
        time.sleep(0.05)
    ser.write(b"0.000,0.000\n")
    time.sleep(0.5)
    lt3, rt3 = get_latest_odo(0.5)
    d_rt3 = (rt3 - rt2) if (rt3 is not None and rt2 is not None) else None
    print(f"Right Ticks delta (Moderate): {d_rt3 if d_rt3 is not None else 'N/A'}")

    print("\n-----------------------------------------------------------")
    print("TEST 4: Powering RIGHT MOTOR at MAXIMUM POWER (PWM 255 Forward, 1.5s)")
    print("-----------------------------------------------------------")
    # Command: lin=0.30, ang=1.071 -> ls=0.0, rs=+0.60 m/s (MAX_MPS = PWM 255 on CH_RR)
    start = time.time()
    while time.time() - start < 1.5:
        ser.write(b"0.300,1.071\n")
        time.sleep(0.05)
    ser.write(b"0.000,0.000\n")
    time.sleep(0.5)
    lt4, rt4 = get_latest_odo(0.5)
    d_rt4 = (rt4 - rt3) if (rt4 is not None and rt3 is not None) else None
    print(f"Right Ticks delta (Full Forward): {d_rt4 if d_rt4 is not None else 'N/A'}")

    print("\n-----------------------------------------------------------")
    print("TEST 5: Powering RIGHT MOTOR at MAXIMUM POWER (PWM 255 Reverse, 1.5s)")
    print("-----------------------------------------------------------")
    # Command: lin=-0.30, ang=-1.071 -> ls=0.0, rs=-0.60 m/s (PWM 255 on CH_RL)
    start = time.time()
    while time.time() - start < 1.5:
        ser.write(b"-0.300,-1.071\n")
        time.sleep(0.05)
    ser.write(b"0.000,0.000\n")
    time.sleep(0.5)
    lt5, rt5 = get_latest_odo(0.5)
    d_rt5 = (rt5 - rt4) if (rt5 is not None and rt4 is not None) else None
    print(f"Right Ticks delta (Full Reverse): {d_rt5 if d_rt5 is not None else 'N/A'}")

    print("\n" + "=" * 65)
    print("DIAGNOSTIC SUMMARY & HARDWARE CHECKLIST FOR RIGHT MOTOR:")
    print("=" * 65)
    print("Symptom: 'Resisting Torque with NO rotation':")
    print(" -> Resisting torque PROVES 12V power and ENABLE pins are ON (Dynamic Brake)!")
    print(" -> It stays locked in brake if RPWM / LPWM does not receive PWM signal.")
    print("\nCHECK THESE 3 ITEMS:")
    print("1. PIN MISMATCH (Most Common):")
    print("   In clean_camera_line_follower_mapping.ino:")
    print("     Pin 23 -> RIGHT_RPWM (Forward PWM)")
    print("     Pin 22 -> RIGHT_LPWM (Reverse PWM)")
    print("     Pin 25 -> RIGHT_REN  (Enable Forward)")
    print("     Pin 26 -> RIGHT_LEN  (Enable Reverse)")
    print("     Pin 18 -> RIGHT_ENC_A (Yellow wire)")
    print("     Pin 19 -> RIGHT_ENC_B (Green wire)")
    print("   *Note: In older diff_drive sketches, Pins 22 & 23 were the ENCODER!")
    print("   Ensure Pin 23 is physically connected to RPWM on the right IBT-2,")
    print("   and Pin 22 is connected to LPWM on the right IBT-2.")
    print("2. COMMON GROUND:")
    print("   Make sure ESP32 GND is connected to the Right IBT-2 GND pin.")
    print("3. MECHANICAL vs ELECTRONIC BRAKE CHECK:")
    print("   Turn off the 12V/24V battery power to the IBT-2.")
    print("   Does the right wheel turn freely by hand when power is OFF?")
    print("   - If YES: The lock is 100% electronic braking (IBT-2 PWM wiring).")
    print("   - If NO:  The gearbox/shaft is mechanically jammed.")
    print("=" * 65 + "\n")
    ser.close()

if __name__ == '__main__':
    main()
