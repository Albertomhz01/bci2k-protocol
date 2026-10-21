"""Mock of the parts of BCI2000 that matter for event timing.

Runs as a separate PROCESS (like the real Operator/Source), so the Python
controller's busy-waiting cannot disturb its block clock.

What it reproduces (as observed in P001S001R02-R05.dat):
  * The amplifier delivers one block of SampleBlockSize samples every
    SampleBlockSize/SamplingRate seconds (small jitter).
  * A "SET STATE" command reaches the Source module after a short latency.
    The Source applies it to the NEXT block it sends, so every state change
    in the recording sits exactly on a block boundary, and which block it
    lands in depends on when the command arrives inside the block.
  * The Operator sees each block (OperatorBackLink) a few ms after the
    Source sent it; "GET STATE SourceTime" returns that value.

Client API = the subset of BCI2000Remote used by the controller:
    Execute(cmd) -> bool, Result, SetStateVariable, Start, Stop
plus Dump(path) which writes the recorded per-block states (the ".dat").
"""

import multiprocessing as mp
import random
import re
import threading
import time
from multiprocessing.connection import Client, Listener

AUTHKEY = b"mockbci"


def _server(address, fs, block, source_time_readable, seed, ready):
    rnd = random.Random(seed)
    T = block / fs
    lock = threading.Lock()
    src_states = {"StimulusPhase": 0, "PainLevel": 0, "PressureTarget": 0,
                  "TrialNumber": 0, "SourceTime": 0}
    pending = []                 # (apply_time, name, value)
    blocks = []                  # (index, send_time, states dict)
    op_view = []                 # (visible_time, SourceTime)
    running = {"on": False, "t0": None}
    params = {"SamplingRate": f"{fs:g}Hz", "SampleBlockSize": f"{block}"}

    def block_loop():
        k = 0
        t0 = time.perf_counter()
        running["t0"] = t0
        while running["on"]:
            t_send = t0 + (k + 1) * T + rnd.gauss(0.0, 0.0004)
            while True:
                d = t_send - time.perf_counter()
                if d <= 0:
                    break
                time.sleep(min(d, 0.002) if d > 0.0025 else 0.0001)
            now = time.perf_counter()
            with lock:
                keep = []
                for item in sorted(pending):
                    if item[0] <= now:
                        src_states[item[1]] = item[2]
                    else:
                        keep.append(item)
                pending[:] = keep
                src_states["SourceTime"] = int(now * 1000) & 0xFFFF
                blocks.append((k, now, dict(src_states)))
                # Operator gets the block after Source -> SigProc -> App -> back
                op_view.append((now + 0.003 + abs(rnd.gauss(0, 0.001)), src_states["SourceTime"]))
                if len(op_view) > 64:
                    del op_view[:32]
            k += 1

    listener = Listener(address, authkey=AUTHKEY)
    ready.set()
    conn = listener.accept()
    thread = None
    while True:
        try:
            cmd = conn.recv()
        except EOFError:
            break
        t_rx = time.perf_counter()
        ok, result = True, ""
        if cmd == "__START__":
            running["on"] = True
            thread = threading.Thread(target=block_loop, daemon=True)
            thread.start()
        elif cmd == "__STOP__":
            running["on"] = False
            if thread:
                thread.join(1)
        elif cmd.startswith("__DUMP__ "):
            import json
            with lock:
                data = {"fs": fs, "block": block, "t0": running["t0"],
                        "blocks": [[k, t, s] for k, t, s in blocks]}
            with open(cmd.split(" ", 1)[1], "w") as f:
                json.dump(data, f)
        else:
            for part in [p.strip() for p in cmd.split(";") if p.strip()]:
                m = re.match(r"SET STATE (\w+) (-?\d+)", part)
                if m:
                    # Operator -> Source message latency
                    lat = 0.0004 + abs(rnd.gauss(0, 0.0006))
                    with lock:
                        pending.append((t_rx + lat, m.group(1), int(m.group(2))))
                    continue
                m = re.match(r"GET STATE (\w+)", part)
                if m:
                    if m.group(1) == "SourceTime" and not source_time_readable:
                        ok, result = False, "State SourceTime not accessible"
                        continue
                    with lock:
                        vis = [v for t, v in op_view if t <= time.perf_counter()]
                    result = str(vis[-1] if vis else 0)
                    continue
                m = re.match(r"GET PARAMETER (\w+)", part)
                if m:
                    result = params.get(m.group(1), "")
                    ok = m.group(1) in params
                    continue
                # SET PARAMETER / others: accepted, ignored
        conn.send((ok, result))
    listener.close()


class MockBCI2000Remote:
    def __init__(self, fs=256, block=32, source_time_readable=True, seed=0):
        port = 52000 + random.randint(0, 5000)
        self.address = ("127.0.0.1", port)
        ready = mp.Event()
        self.proc = mp.Process(target=_server,
                               args=(self.address, fs, block, source_time_readable, seed, ready),
                               daemon=True)
        self.proc.start()
        ready.wait(5)
        self.conn = Client(self.address, authkey=AUTHKEY)
        self.Result = ""

    def _call(self, cmd):
        # Operator telnet round trip is ~1 ms on a local machine
        time.sleep(random.uniform(0.0003, 0.0010))
        self.conn.send(cmd)
        ok, self.Result = self.conn.recv()
        return ok

    def Execute(self, cmd):
        return self._call(cmd)

    def SetStateVariable(self, name, value):
        return self._call(f"SET STATE {name} {int(value)}")

    def Start(self):
        return self._call("__START__")

    def Stop(self):
        return self._call("__STOP__")

    def Dump(self, path):
        return self._call(f"__DUMP__ {path}")

    def close(self):
        try:
            self.conn.close()
        except Exception:
            pass
        self.proc.terminate()
