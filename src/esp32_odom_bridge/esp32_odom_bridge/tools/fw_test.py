#!/usr/bin/env python3
"""
fw_test: automatic bench test for amr_base_pid firmware.  WHEELS OFF THE GROUND!
  ros2 run esp32_odom_bridge fw_test [/dev/ttyUSB0]
Nothing else may use the serial port (close miniterm / ROS launch first).
Result printed + saved to ~/mapping_ws/logs/fw_test.txt
"""
import os
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
OUT = os.path.expanduser('~/mapping_ws/logs/fw_test.txt')
TICKS_PER_M = 47000 / (3.14159265 * 0.127)

lines_out = []


def out(s=''):
    print(s)
    lines_out.append(s)


class FW:
    def __init__(self, port):
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, 115200, 0.02
        self.s.dtr = False
        self.s.rts = False
        self.s.open()
        self.buf = b''
        self.odo = []    # (t, lt, rt)
        self.dbg = []    # (t, tl, ml, pwl, tr, mr, pwr)
        self.msgs = []   # (t, text) everything that is not ODO/DBG

    def send(self, cmd):
        self.s.write((cmd + '\n').encode())

    def pump(self, sec, repeat=None, every=0.1):
        end = time.time() + sec
        nxt = 0.0
        while time.time() < end:
            if repeat and time.time() >= nxt:
                self.send(repeat)
                nxt = time.time() + every
            self.buf += self.s.read(4096)
            while b'\n' in self.buf:
                raw, self.buf = self.buf.split(b'\n', 1)
                ln = raw.decode(errors='ignore').strip()
                t = time.time()
                p = ln.split(',')
                try:
                    if p[0] == 'ODO' and len(p) == 4:
                        self.odo.append((t, int(p[1]), int(p[2])))
                    elif p[0] == 'DBG' and len(p) == 7:
                        self.dbg.append((t, *[float(x) for x in p[1:]]))
                    elif ln:
                        self.msgs.append((t, ln))
                        print(f"   << {ln}")
                except ValueError:
                    pass

    def odo_speed(self, t0, t1):
        s = [r for r in self.odo if t0 <= r[0] <= t1]
        if len(s) < 2:
            return float('nan'), float('nan')
        dt = s[-1][0] - s[0][0]
        # forward = negative ticks (bridge convention)
        vl = -(s[-1][1] - s[0][1]) / TICKS_PER_M / dt
        vr = -(s[-1][2] - s[0][2]) / TICKS_PER_M / dt
        return vl, vr

    def dbg_mean(self, t0, t1):
        s = [r for r in self.dbg if t0 <= r[0] <= t1]
        if not s:
            return None
        n = len(s)
        return [sum(r[i] for r in s) / n for i in range(1, 7)]


def save():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        f.write('\n'.join(lines_out) + '\n')
    print(f"Saved {OUT}")


def check(name, ok, detail):
    out(f"[{'PASS' if ok else 'FAIL'}] {name:<22} {detail}")
    return ok


def drive_test(fw, name, cmd, exp_l, exp_r, tol=0.05, dur=2.5):
    fw.send('C')
    t0 = time.time()
    fw.pump(dur, repeat=cmd)
    t1 = time.time()
    fw.pump(0.6, repeat='0,0')
    vl, vr = fw.odo_speed(t1 - 1.0, t1)
    d = fw.dbg_mean(t1 - 1.0, t1)
    pw = f" pwm L={d[2]:.0f} R={d[5]:.0f}" if d else ""
    ok = abs(vl - exp_l) < tol and abs(vr - exp_r) < tol
    faults = [m for t, m in fw.msgs if m.startswith('FAULT,') and t >= t0]
    if faults:
        ok = False
        pw += f"  {faults[0]}"
    return check(name, ok, f"cmd '{cmd}': L={vl:+.3f} R={vr:+.3f} m/s "
                           f"(want {exp_l:+.2f} {exp_r:+.2f}){pw}")


def main():
    out(f"fw_test on {PORT}  {time.strftime('%Y-%m-%d %H:%M:%S')}")
    try:
        fw = FW(PORT)
    except Exception as e:
        out(f"ERROR opening {PORT}: {e}  (close miniterm / ROS first)")
        save()
        return
    input("WHEELS OFF THE GROUND? Press Enter to start...")
    fw.pump(1.0)
    fw.send('R')
    fw.send('V')
    fw.pump(0.8)
    banner = [m for _, m in fw.msgs if 'READY' in m or 'RESET_OK' in m or 'DEBUG' in m]
    pid_fw = any('amr_base_pid' in m or 'DEBUG_ON' in m for _, m in fw.msgs)
    res = []
    res.append(check('firmware', pid_fw and len(fw.odo) > 10,
                     f"ODO lines={len(fw.odo)}, msgs={banner[-2:] if banner else 'none'}"))
    if not pid_fw:
        out("  -> amr_base_pid not detected (no DEBUG_ON reply). Reflash, then rerun.")

    res.append(drive_test(fw, 'forward 0.15', '0.15,0', 0.15, 0.15))
    res.append(drive_test(fw, 'backward 0.15', '-0.15,0', -0.15, -0.15))
    res.append(drive_test(fw, 'slow forward 0.05', '0.05,0', 0.05, 0.05, tol=0.03))
    res.append(drive_test(fw, 'pivot left 0.3', '0,0.3', -0.084, 0.084, tol=0.04))

    # watchdog: single command, must stop by itself
    fw.send('C')
    fw.send('0.15,0')
    t0 = time.time()
    fw.pump(2.0)
    vl, vr = fw.odo_speed(t0 + 1.2, t0 + 2.0)
    res.append(check('watchdog stop', abs(vl) < 0.01 and abs(vr) < 0.01,
                     f"speed 1.2-2.0 s after a single command: L={vl:+.3f} R={vr:+.3f}"))

    # Open-loop check (old firmware behaviour): separates firmware bug vs power/wiring
    fw.send('V')            # debug off
    fw.send('P')            # PID off -> open loop
    fw.pump(0.3)
    t0 = time.time()
    fw.pump(2.0, repeat='0.15,0')
    vl, vr = fw.odo_speed(time.time() - 1.0, time.time())
    fw.pump(0.6, repeat='0,0')
    ol = check('open-loop forward', vl > 0.03 and vr > 0.03,
               f"L={vl:+.3f} R={vr:+.3f} m/s (PWM ~165, any forward motion = pass)")
    res.append(ol)
    fw.send('P')            # PID back on
    fw.send('S')
    fw.pump(0.3)
    out("")
    out("Last DBG samples (tl, ml, pwmL, tr, mr, pwmR):")
    for r in fw.dbg[-5:]:
        out("   " + ", ".join(f"{x:.3f}" for x in r[1:]))
    out("Firmware messages:")
    for _, m in fw.msgs[-15:]:
        out("   " + m)
    moving = abs(vl) > 0.03 or abs(vr) > 0.03
    if not ol and not moving:
        out("DIAG: even open-loop does not move -> motor POWER / IBT-2 wiring / R_EN,L_EN, "
            "or ODO ticks not counting. Check battery switch + driver supply.")
    elif not ol:
        for side, v in (('LEFT', vl), ('RIGHT', vr)):
            if v < -0.03:
                out(f"DIAG: {side} wheel moves but reads BACKWARD (v={v:+.3f}). If that wheel "
                    f"physically turned FORWARD -> flip ENC_FWD_SIGN_{side[0]}; if it physically "
                    f"turned BACKWARD -> flip INVERT_{side}_MOTOR.")
            elif abs(v) < 0.03:
                out(f"DIAG: {side} wheel not moving/counting in open-loop (power, EN or encoder).")
    elif not res[1]:
        out("DIAG: open-loop moves but PID does not -> firmware/control issue (send me this file).")
    out("")
    out(f"RESULT: {sum(res)}/{len(res)} passed")
    fail_dir = [m for _, m in fw.msgs if 'ENC_DIR' in m]
    if fail_dir:
        out(f"  {fail_dir[0]} -> flip ENC_FWD_SIGN_{fail_dir[0][-1]} in amr_base_pid.ino and reflash")
    save()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        out("aborted by user")
        save()
