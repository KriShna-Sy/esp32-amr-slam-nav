#!/usr/bin/env python3
"""
dir_test: check which way the robot REALLY moves for forward / turn-left commands.
Robot ON THE FLOOR, 1 m clear. Stand BEHIND it (camera pointing away from you).
  ros2 run esp32_odom_bridge dir_test [/dev/ttyUSB0]
Saves ~/mapping_ws/logs/dir_test.txt
"""
import os
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
OUT = os.path.expanduser('~/mapping_ws/logs/dir_test.txt')


def drive(s, cmd, sec):
    end = time.time() + sec
    while time.time() < end:
        s.write((cmd + '\n').encode())
        time.sleep(0.1)
        s.read(s.in_waiting or 1)
    s.write(b'0,0\n')


def main():
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, 115200, 0.05
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(1.5)
    banner = [l for l in s.read(s.in_waiting or 1).decode(errors='ignore').splitlines() if 'READY' in l]
    s.write(b'R\n')
    time.sleep(0.3)
    s.reset_input_buffer()
    lines = [f"dir_test {time.strftime('%Y-%m-%d %H:%M:%S')}",
             f"firmware: {banner[-1] if banner else '(no READY line seen - board did not reset)'}"]
    print(lines[-1])

    input("\nStand BEHIND the robot (camera away from you). Press Enter -> it TURNS for 2 s...")
    drive(s, '0,0.3', 2.0)
    t = input("Did it turn to YOUR LEFT (counter-clockwise from above)? [y/n]: ").strip().lower()
    lines.append(f"turn-left command (0,0.3): turned left = {t}")

    input("\nPress Enter -> it drives FORWARD for 2 s...")
    drive(s, '0.15,0', 2.0)
    f = input("Did it move AWAY from you (forward, camera first)? [y/n]: ").strip().lower()
    lines.append(f"forward command (0.15,0): went forward = {f}")
    s.close()

    left_ok, fwd_ok = t.startswith('y'), f.startswith('y')
    if left_ok and fwd_ok:
        v = "ALL CORRECT"
    elif fwd_ok and not left_ok:
        v = "left/right swapped -> need firmware v2 (L/R swapped) flashed"
    elif not fwd_ok and left_ok:
        v = "both motors reversed AND sides swapped"
    else:
        v = "both motors reversed (forward and turns both wrong)"
    lines.append("VERDICT: " + v)
    print("\n" + "\n".join(lines[2:]))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"Saved {OUT}")


if __name__ == '__main__':
    main()
