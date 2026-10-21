"""Starts the native firmware simulator behind a pseudo-terminal so that
pyserial (and ArduinoPump) can open it exactly like a COM port."""

import os
import subprocess
import tty

HERE = os.path.dirname(os.path.abspath(__file__))
SRC  = os.path.join(HERE, "arduino_sim.cpp")
INO  = os.path.join(HERE, "..", "..", "arduino_pressure_controller", "arduino_pressure_controller.ino")


def build(out_dir):
    exe = os.path.join(out_dir, "arduino_sim")
    if (not os.path.exists(exe) or
            os.path.getmtime(exe) < max(os.path.getmtime(SRC), os.path.getmtime(INO))):
        subprocess.check_call(["g++", "-O2", "-std=c++17", "-o", exe, SRC])
    return exe


class SimArduino:
    def __init__(self, out_dir, truth_path, clock_error_ppm=3000.0, seed=1):
        exe = build(out_dir)
        master, slave = os.openpty()
        tty.setraw(master)
        tty.setraw(slave)
        self.port = os.ttyname(slave)
        self._slave = slave
        self.proc = subprocess.Popen(
            [exe, truth_path, str(clock_error_ppm), str(seed)],
            stdin=master, stdout=master, stderr=subprocess.DEVNULL, close_fds=True,
        )
        os.close(master)
        self.truth_path = truth_path

    def close(self):
        try:
            self.proc.terminate()
            self.proc.wait(2)
        except Exception:
            pass
        try:
            os.close(self._slave)
        except Exception:
            pass
