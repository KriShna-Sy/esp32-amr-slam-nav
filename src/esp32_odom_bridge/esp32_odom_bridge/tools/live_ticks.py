#!/usr/bin/env python3
"""live_ticks: print live encoder ticks (hand-spin a wheel to check counting).
Usage: ros2 run esp32_odom_bridge live_ticks [/dev/ttyUSB0]
"""
import sys
import time
import serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    s = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(0.2)
    s.write(b'R\n')
    time.sleep(0.1)
    s.reset_input_buffer()
    print("Encoder reset to 0,0. Rotate a wheel by hand one full turn. Ctrl+C to stop.\n")
    try:
        while True:
            line = s.readline().decode(errors='ignore').strip()
            if line.startswith("ODO,"):
                p = line.split(",")
                if len(p) == 4:
                    print(f"\rLeft: {int(p[1]):8d}  |  Right: {int(p[2]):8d}", end="", flush=True)
            elif line.startswith(("FAULT", "DBG", "READY")):
                print(f"\n{line}")
    except KeyboardInterrupt:
        print("\nFinished.")
    finally:
        s.close()


if __name__ == '__main__':
    main()
