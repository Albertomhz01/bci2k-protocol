"""Protocol and safety tests of the firmware (runs the real .ino against the
simulated cuff). No BCI2000, no Tk.

    python firmware_protocol_test.py [out_dir]
"""

import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_link import SimArduino

OUT = sys.argv[1] if len(sys.argv) > 1 else "."
os.makedirs(OUT, exist_ok=True)

results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond), detail))
    print(f"[{'PASS' if cond else 'FAIL'}] {name}  {detail}")


class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.buf = ""
        self.lines = []      # (t, line) non-telemetry
        self.tel = []        # (t, pressure, target, desired, pwm)

    def pump(self, seconds):
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            self.poll()
            time.sleep(0.002)

    def poll(self):
        c = self.ser.read(4096)
        if c:
            self.buf += c.decode("ascii", "ignore")
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            line = line.strip()
            t = time.perf_counter()
            if line.startswith("P,"):
                p = line.split(",")
                if len(p) >= 6:
                    self.tel.append((t, float(p[1]), float(p[2]), float(p[3]), int(p[4])))
            elif line:
                self.lines.append((t, line))

    def send(self, s):
        self.ser.write((s + "\n").encode())
        return time.perf_counter()

    def wait_for(self, prefix, timeout):
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            self.poll()
            for t, l in self.lines:
                if l.startswith(prefix):
                    self.lines.remove((t, l))
                    return t, l
            time.sleep(0.002)
        return None, None

    def last_pressure(self):
        return self.tel[-1][1] if self.tel else None


sim = SimArduino(OUT, os.path.join(OUT, "protocol_truth.csv"))
try:
    link = Link(sim.port)
    t, l = link.wait_for("READY", 5)
    check("boot READY", l == "READY")
    link.send("PING")
    t, l = link.wait_for("READY", 2)
    check("PING -> READY", l == "READY")

    # 1. Out-of-range targets are refused and vent
    link.send("TRIAL,90,250,0,30000")
    t, l = link.wait_for("FAULT", 2)
    check("TRIAL target 250 mmHg refused", l == "FAULT,TARGET_OUT_OF_RANGE", l)
    link.send("TRIAL,91,50")
    t, l = link.wait_for("FAULT", 2)
    check("malformed TRIAL refused", l == "FAULT,BAD_TRIAL", l)

    # 2. HIGH-style trial: full speed, pump latched off at target, holds until DEFLATE
    t_cmd = link.send("TRIAL,1,111,0,40000")
    t, ack = link.wait_for("INFLATING,1,", 1)
    check("TRIAL ack", ack is not None, ack)
    t, pon = link.wait_for("PUMP_ON,1,", 1)
    check("full speed: PUMP_ON at t=0", pon is not None and pon.split(",")[2] == "0", pon)
    t_tr, tr = link.wait_for("TARGET_REACHED,1,", 15)
    check("TARGET_REACHED", tr is not None, tr)
    link.pump(3.0)
    held = [x for x in link.tel if x[0] > t_tr + 0.3]
    check("pump stays OFF while holding (no filling of the window)",
          all(x[4] == 0 for x in held), f"max pwm during hold {max(x[4] for x in held)}")
    # After the latch the reading may still climb briefly (motor spin-down +
    # sensor filter lag, same as the original firmware). It must not keep rising.
    p_settled = held[0][1]
    check("no pressure rise once the pump has stopped",
          max(x[1] for x in held) <= p_settled + 0.5,
          f"settled {p_settled:.1f}, max {max(x[1] for x in held):.1f} mmHg "
          f"(overshoot vs 111 target: {max(x[1] for x in held) - 111:+.1f})")
    check("no automatic deflation (Python owns HOLDING)",
          not any(l.startswith("DEFLATING") for _, l in link.lines))
    t_d = link.send("DEFLATE,1")
    t, dl = link.wait_for("DEFLATING,1,", 1)
    check("DEFLATE ack", dl is not None, dl)
    t, df = link.wait_for("DEFLATED,1,", 15)
    check("DEFLATED reported", df is not None, df)

    # 3. Duplicate TRIAL (same id) does not restart the ramp
    link.send("TRIAL,2,46,7500,40000")
    t, a1 = link.wait_for("INFLATING,2,", 1)
    link.pump(2.0)
    p_before = link.last_pressure()
    link.send("TRIAL,2,46,7500,40000")
    t, a2 = link.wait_for("INFLATING,2,", 1)
    check("duplicate TRIAL re-acked with the same start time", a1 == a2, f"{a1} / {a2}")
    link.pump(0.5)
    check("duplicate TRIAL did not reset the ramp", link.last_pressure() >= p_before - 1.0,
          f"{p_before:.1f} -> {link.last_pressure():.1f}")

    # 4. STOP during the ramp vents immediately
    t_s = link.send("STOP")
    t, st = link.wait_for("STOPPED", 1)
    check("STOP acknowledged fast", st is not None and (t - t_s) < 0.05, f"{(t - t_s)*1000:.1f} ms")
    link.pump(0.3)
    check("pump off after STOP", link.tel[-1][4] == 0)
    link.pump(4.0)
    check("cuff empties after STOP", link.last_pressure() < 3.0, f"{link.last_pressure():.1f} mmHg")

    # 5. Fail-safe: Python never sends DEFLATE
    t_f = link.send("TRIAL,3,46,2000,6000")
    t, _ = link.wait_for("TARGET_REACHED,3,", 10)
    t_fs, fs = link.wait_for("FAULT", 10)
    check("fail-safe vents without DEFLATE", fs == "FAULT,FAILSAFE_NO_DEFLATE", fs)
    check("fail-safe fires at failsafe_ms", fs and 5.9 < t_fs - t_f < 6.4, f"{t_fs - t_f:.3f} s")
    link.pump(5.0)
    check("cuff empty after fail-safe", link.last_pressure() < 3.0, f"{link.last_pressure():.1f}")

    # 6. Legacy START cycle still works end to end
    link.send("START,40")
    got = []
    for w in ("INFLATING", "HOLDING", "DEFLATING", "DONE"):
        t, l = link.wait_for(w, 20)
        got.append((t, l))
    check("legacy START -> INFLATING/HOLDING/DEFLATING/DONE", all(l for _, l in got),
          " ".join(str(l) for _, l in got))
    if all(l for _, l in got):
        check("legacy HOLD still 10 s", abs((got[2][0] - got[1][0]) - 10.0) < 0.1,
              f"{got[2][0] - got[1][0]:.3f} s")

    # 7. STOP latency while idle and while a trial holds pressure
    link.send("TRIAL,4,111,0,40000")
    link.wait_for("TARGET_REACHED,4,", 15)
    lat = []
    t_s = link.send("STOP")
    t, st = link.wait_for("STOPPED", 1)
    check("STOP while holding HIGH", st == "STOPPED", f"{(t - t_s)*1000:.1f} ms")
finally:
    sim.close()

n_fail = sum(1 for _, ok, _ in results if not ok)
print(f"\n{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
