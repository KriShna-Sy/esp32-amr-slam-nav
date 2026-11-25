#!/usr/bin/env python3
"""
calibrate_ticks.py
Calibrates encoder ticks per wheel revolution for the ESP32 robot base.
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUD = 115200
RUN_TIME = 4.0  # seconds to drive
SPEED = 0.1     # linear velocity (m/s)

def read_stable_odo(s):
    """Drain serial queue and read the newest valid ODO line."""
    time.sleep(0.1)
    lines = s.read(s.in_waiting).decode(errors='ignore').splitlines()
    for l in reversed(lines):
        if l.startswith("ODO,"):
            p = l.split(",")
            if len(p) == 4:
                try:
                    return int(p[1]), int(p[2])
                except ValueError:
                    pass
    # Fallback to readline
    for _ in range(10):
        l = s.readline().decode(errors='ignore').strip()
        if l.startswith("ODO,"):
            p = l.split(",")
            if len(p) == 4:
                try:
                    return int(p[1]), int(p[2])
                except ValueError:
                    pass
    return None, None

def run_calibration():
    print(f"Connecting to ESP32 on {PORT} at {BAUD} baud...")
    try:
        s = serial.Serial(PORT, BAUD, timeout=0.5)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    time.sleep(0.3)
    s.write(b'\n')
    time.sleep(0.1)
    s.reset_input_buffer()
    s.write(b'R\n')
    time.sleep(0.2)

    start_lt, start_rt = read_stable_odo(s)
    if start_lt is None:
        print("ERROR: Could not read encoder stream from ESP32.")
        s.close()
        sys.exit(1)

    print(f"Baseline reading : Left={start_lt}, Right={start_rt}")
    print(f"Starting motor test: running at {SPEED} m/s for {RUN_TIME} seconds...")
    print("Watch the marked wheel and count the rotations carefully!\n")

    t_start = time.time()
    while time.time() - t_start < RUN_TIME:
        s.write(f"{SPEED},0.0\n".encode())
        time.sleep(0.1)

    # Stop motors
    s.write(b'0.0,0.0\n')
    time.sleep(0.5)
    s.write(b'0.0,0.0\n')
    time.sleep(0.3)

    end_lt, end_rt = read_stable_odo(s)
    s.close()

    if end_lt is None:
        print("ERROR: Failed to capture final reading after stopping.")
        sys.exit(1)

    delta_lt = abs(end_lt - start_lt)
    delta_rt = abs(end_rt - start_rt)

    print("=" * 45)
    print("CALIBRATION RUN COMPLETE")
    print(f"Delta Left Ticks  : {delta_lt}  (from {start_lt} to {end_lt})")
    print(f"Delta Right Ticks : {delta_rt}  (from {start_rt} to {end_rt})")
    print("=" * 45)

    rotations_input = input("\nEnter the number of rotations you observed for the RIGHT wheel (e.g. 1.7): ").strip()
    try:
        rotations = float(rotations_input)
        if rotations > 0:
            ticks_per_rev = round(delta_rt / rotations, 1)
            print(f"\nCalculated ticks_per_rev = {delta_rt} / {rotations} = {ticks_per_rev}")
            print(f"Suggested integer parameter for launch file: {round(ticks_per_rev)}")
        else:
            print("Rotations must be > 0.")
    except ValueError:
        print("Invalid number entered.")

def main():
    run_calibration()


if __name__ == '__main__':
    main()
