"""End-to-end timing simulation of experiment_controller_cruz.py.

Uses the REAL controller code (ArduinoPump, FixationCross, EventClock,
run_calibration, run_trial) against
  * the real firmware compiled natively with a simulated cuff (arduino_sim.cpp)
  * a mock BCI2000 that quantizes state changes to sample blocks.
No participant, no hardware. Needs a display for Tk (use xvfb-run on Linux).

    xvfb-run -a python run_timing_simulation.py --out sim_out --per-cond 5
    xvfb-run -a python run_timing_simulation.py --out sim_out_nolock --mode deadline

Then:  python analyze_simulation.py sim_out
"""

import argparse
import json
import os
import random
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, REPO)

from mock_bci2000 import MockBCI2000Remote   # noqa: E402
from sim_link import SimArduino              # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="sim_out")
    ap.add_argument("--per-cond", type=int, default=5)
    ap.add_argument("--iti", type=float, default=2.0, help="rest (s); 10 in the experiment")
    ap.add_argument("--mode", choices=("block", "deadline"), default="block")
    ap.add_argument("--fs", type=float, default=256)
    ap.add_argument("--block", type=int, default=32)
    ap.add_argument("--clock-ppm", type=float, default=3000.0,
                    help="Arduino resonator error to simulate")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--high", type=int, default=111)
    ap.add_argument("--low", type=int, default=46)
    ap.add_argument("--none", type=int, default=8)
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    random.seed(a.seed)

    # The controller imports bci_setup_cruz_con_ruta -> BCI2000Remote.
    fake = types.ModuleType("BCI2000Remote")
    fake.BCI2000Remote = MockBCI2000Remote
    sys.modules["BCI2000Remote"] = fake
    os.chdir(out)   # results/telemetry files land here
    import experiment_controller_cruz as ec
    ec.ITI_TIME = a.iti

    sim = SimArduino(out, os.path.join(out, "truth.csv"), a.clock_ppm, a.seed)
    bci = MockBCI2000Remote(a.fs, a.block, source_time_readable=(a.mode == "block"), seed=a.seed)
    pump = cross = telemetry = None
    record = {"args": vars(a), "trials": []}
    try:
        pump = ec.ArduinoPump(sim.port)
        cross = ec.FixationCross()
        cross.serial_drain = pump.drain
        bci.Start()
        time.sleep(1.0)
        clock = ec.EventClock(bci)
        print("EventClock:", clock.describe())
        record["sync"] = clock.info()
        telemetry = ec.TelemetryCSV(os.path.join(out, "telemetry.csv"), pump)
        pump.telemetry_sink = telemetry

        cal, timing = ec.run_calibration(bci, cross, pump, clock, a.high, a.none, a.low)
        record["calibration_trial"] = cal
        record["calibration"] = timing

        conds = ([(ec.NONE_LABEL, a.none)] * a.per_cond + [(ec.LOW_LABEL, a.low)] * a.per_cond
                 + [(ec.HIGH_LABEL, a.high)] * a.per_cond)
        random.shuffle(conds)
        for i, (c, p) in enumerate(conds, start=1):
            log = ec.run_trial(bci, cross, pump, clock, timing, i, c, p, 1)
            if log is None:
                raise RuntimeError("trial aborted")
            record["trials"].append(log)
        record["arduino_clock_ppm"] = pump.arduino_clock_ppm()
        record["align_failures"] = clock.align_failures
    finally:
        try:
            time.sleep(0.5)
            bci.Stop()
            bci.Dump(os.path.join(out, "eeg_states.json"))
        except Exception as e:
            print("dump failed", e)
        with open(os.path.join(out, "sim_results.json"), "w") as f:
            json.dump(record, f, indent=1, default=str)
        if telemetry:
            telemetry.close()
        if pump:
            pump.close()
        if cross:
            cross.close()
        sim.close()
        bci.close()
    print("done ->", out)


if __name__ == "__main__":
    main()
