# Timing simulation (no participant, no hardware)

Tests the fixed-duration event logic of `experiment_controller_cruz.py` and the
firmware `arduino_pressure_controller.ino` together:

* `arduino_sim.cpp` compiles the **unchanged** `.ino` for the PC, with a
  simulated pump, cuff, valve and MPX5050DP sensor. The model is fitted to the
  timing in P001S001R02–R05 (HIGH 111 mmHg ≈ 7 s, LOW 46 mmHg ≈ 3.6 s, HIGH
  deflation ≈ 4.5 s). It is not the real pump, only a plant with the right time scales.
* `mock_bci2000.py` stands in for BCI2000. It is a separate process that sends
  32-sample blocks at 256 Hz and applies `SET STATE` to the next block, which
  is what the real recordings show.
* `run_timing_simulation.py` runs the real controller functions (calibration,
  then trials) against both simulators.
* `analyze_simulation.py` checks the recorded states against ground truth.
* `firmware_protocol_test.py` checks the firmware's protocol and safety paths
  (fail-safe vent, STOP, duplicate TRIAL, legacy START).

Linux/macOS with `g++`, Python 3 with `pyserial`, `numpy`, `tkinter`,
`matplotlib`, and a display (`xvfb-run -a` on a headless machine):

```bash
cd tools/timing_sim
python firmware_protocol_test.py out_proto
xvfb-run -a python run_timing_simulation.py --out out_block --per-cond 5 --iti 2
python analyze_simulation.py out_block --plot out_block/pressure.png
xvfb-run -a python run_timing_simulation.py --out out_deadline --mode deadline --per-cond 3 --iti 2
python analyze_simulation.py out_deadline
```

To check a real recording after a session:

```bash
python tools/verify_event_timing.py dat_bci/P001/P001S001R06.dat --results participant_001_run06_results.json
```
