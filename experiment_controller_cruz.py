import csv
import json
import math
import random
import re
import time
import os
import tkinter as tk
from collections import deque
from datetime import datetime
from participant_interface import launch_interface
from bci_setup_cruz_con_ruta import setup_bci

import serial                       # pip install pyserial
import serial.tools.list_ports

# Control pressure is 8 mmHg so all conditions share the same pump noise and vibration.
NONE_PRESSURE    = 8
LOW_LABEL        = "LOW"
HIGH_LABEL       = "HIGH"
NONE_LABEL       = "NONE"

TRIALS_PER_BLOCK = 30
TRIALS_PER_COND  = 10

# SINGLE-RUN DESIGN: each run is a self-contained experiment
# (calibration, trials 1..TRIALS_PER_BLOCK, one .dat file).
# Re-running the same participant/session creates R02, R03, etc.
TRIALS_TOTAL     = TRIALS_PER_BLOCK

BLOCK_REST       = 30

CROSS_TIME       = 3    # Event 0 — Fixation Cross
ITI_TIME         = 10   # Event 4 — Rest (part of the same trial)

PAIN_MAP = {NONE_LABEL: 0, LOW_LABEL: 1, HIGH_LABEL: 2}

# --- Arduino (pressure pump) ---
# Serial port. None = auto-detect.
ARDUINO_PORT = None          # e.g. "COM3"
ARDUINO_BAUD = 115200        # Must match Serial.begin() in the .ino

PRESSURE_LOG_INTERVAL = 1.0

# HOLDING duration. It used to be timed by the Arduino (HOLD_DURATION in the
# .ino, 10000 ms) starting when the target was reached. It is now timed by
# Python from the HOLDING state onset, like INFLATING and DEFLATING.
HOLD_SECONDS = 10.0

# Maximum pressure Python can request; guards against bad participant JSON values.
MAX_SAFE_MMHG = 200.0

# Arduino error messages; abort the trial if received.
FAULT_WORDS = ("STOPPED", "FAULT")

# Protocol vocabulary and the number of comma separated fields each line must
# have. Anything else is a truncated line (serial overflow) and is dropped.
MESSAGE_FIELDS = {
    "READY":          (1,),
    "ZERO":           (3,),
    "INFLATING":      (1, 3),   # 1 = legacy START, 3 = TRIAL ack
    "HOLDING":        (1,),     # legacy START only
    "DEFLATING":      (1, 3),
    "DONE":           (1,),     # legacy START only
    "PUMP_ON":        (3,),
    "TARGET_REACHED": (4,),
    "DEFLATED":       (5,),
    "STOPPED":        (1,),
    "FAULT":          (2,),
}

# =============================================================================
# FIXED EXPERIMENTAL EVENT DURATIONS
#
# Two clocks exist in this experiment and are deliberately kept apart:
#
#   Experimental timing  -> when BCI2000's StimulusPhase changes. Decided ONLY
#                           by Python, with durations that are identical for
#                           HIGH, LOW and NONE.
#   Physical timing      -> when the cuff actually starts, reaches the target,
#                           and empties. Reported by the Arduino
#                           (PUMP_ON, TARGET_REACHED, DEFLATED), logged with
#                           its own timestamps, and NEVER used to end an
#                           experimental event.
#
# INFLATING_DURATION and DEFLATING_DURATION are measured in the calibration
# (trial 0, HIGH pressure) and rounded UP to whole seconds:
#     INFLATING_DURATION = ceil(measured HIGH inflation time + INFLATE_MARGIN_S)
#     DEFLATING_DURATION = ceil(measured HIGH deflation time + DEFLATE_MARGIN_S)
# With LOCK_TIMING_ACROSS_RUNS (below) this happens only in the FIRST run of a
# participant+session; later runs of that session reuse the same durations.
# =============================================================================
INFLATE_MARGIN_S  = 0.0     # extra margin added BEFORE ceil() (spec: none)
DEFLATE_MARGIN_S  = 0.0
MIN_EVENT_SECONDS = 1

# Sanity limit on the calibrated INFLATING/DEFLATING durations. A longer
# calibration means a leak or a loose hose, not a participant property.
MAX_CALIBRATED_EVENT_SECONDS = 30

# =============================================================================
# TIMING LOCK ACROSS RUNS
#
# Without the lock every run re-derives INFLATING/DEFLATING from its own
# calibration, so R02 could use 8 s and R03 12 s. With the lock, the FIRST
# run of a participant+session that finishes its calibration fixes the
# durations, and every later run of that same session reuses them (R03, R04,
# R05 get exactly the same INFLATING/HOLDING/DEFLATING as R02).
#
# Later runs still do the calibration (priming, sensor check, measurement),
# but only to CHECK that the locked window is still physically enough. If
# HIGH now needs longer than the locked INFLATING, a warning is shown before
# the trials start (usually the cuff was refitted looser: refit it).
#
# Where the lock lives: participant_XXX.json, key "timing_lock", one entry
# per session. It can be read and edited by hand, e.g.
#   "timing_lock": {"1": {"INFLATING_DURATION": 8, "DEFLATING_DURATION": 5,
#                         "reference_run": 2, ...}}
# Delete that session's entry to let the next run calibrate a new lock.
#
# If a session has no lock yet but earlier runs of it were already recorded
# (e.g. R02 before this change existed), the lock is taken from their
# participant_XXX_runNN_results.json ("calibration" section):
#   TIMING_REFERENCE_RUN = None -> the earliest such run of the session
#   TIMING_REFERENCE_RUN = 2    -> exactly run 2 (errors if it has no calibration)
# =============================================================================
LOCK_TIMING_ACROSS_RUNS = True
TIMING_REFERENCE_RUN    = None

# LOW (and NONE, see RAMP_NONE) follow a pressure ramp that ends this many
# seconds BEFORE the INFLATING window closes. The small margin absorbs the
# controller's tracking lag so the target is reached inside INFLATING instead
# of spilling into HOLDING.
RAMP_END_MARGIN_S = 0.5

# NONE keeps its 8 mmHg control pressure (it exists so that NONE also hears
# and feels the pump). With RAMP_NONE = True the 8 mmHg are delivered on the
# same slow ramp as LOW, so the pump is active across the INFLATING window in
# every condition instead of a ~0.2 s burst. Set to False to restore the old
# NONE behavior (full speed, reaches 8 mmHg almost immediately); the
# experimental INFLATING duration is identical either way.
RAMP_NONE = True

# The Arduino vents on its own if Python does not send DEFLATE within
# (INFLATING + HOLDING + FAILSAFE_GRACE_S) of the TRIAL command.
FAILSAFE_GRACE_S = 5.0

# Calibration inflation is event driven, limited by MAX_INFLATE_MS in the .ino.
CALIBRATION_MAX_INFLATE_S = 60.0

# PRIMING before the measured calibration. The very first inflation after the
# cuff is put on fills slack before the pressure rises (rig log 2026-09-21:
# stuck near 4 mmHg for 10 s, HIGH reached at 28.5 s in the calibration but at
# 16 s in trial 1). Measuring that first inflation made every INFLATING window
# far too long. So the calibration first inflates once to the participant's
# LOW pressure (not measured, never above LOW), vents, waits
# PRIME_SETTLE_S, and only then runs the measured HIGH inflation, from the
# same state the trials start from.
CALIBRATION_PRIMING = True
PRIME_SETTLE_S      = 5.0
PRIME_TRIAL_ID      = 999      # Arduino id of the priming cycle (not a trial)

# Cuff counts as empty below this (same as DEFLATE_DONE_MMHG in the .ino).
CUFF_EMPTY_MMHG = 3.0

# Before a trial starts, wait up to this long for the previous deflation to
# finish (normally it finished long before, during the rest).
CUFF_EMPTY_WAIT_S = 30.0

# TRIAL acknowledgment: resend (same id, harmless) and give up after.
ACK_RESEND_S  = 0.25
ACK_TIMEOUT_S = 1.0

# --- BCI2000 synchronization (see EventClock) ---
# Lock the start of every trial to BCI2000's block clock by watching the
# SourceTime state. Falls back to plain monotonic deadlines if unavailable.
BLOCK_LOCK = True
# Send transitions this fraction of a block after a block was seen by the
# Operator (far from the boundary where the Source applies state changes).
BLOCK_ALIGN_FRACTION = 0.25
# Stop Tkinter/serial work this long before a transition and spin, so a slow
# window redraw cannot delay the state change.
SPIN_BEFORE_DEADLINE_S = 0.020
# A transition sent later than this is flagged in the results.
LATE_TRANSITION_WARN_MS = 10.0


class PumpError(Exception):
    """Communication error with the pump Arduino."""
    pass


class PumpTimeoutError(PumpError):
    """Arduino did not report the next phase in time; STOP is sent."""
    pass


class PumpFaultError(PumpError):
    """Arduino reported FAULT/STOPPED and already vented."""
    pass


def _autodetect_arduino_port():
    """Returns the first port that looks like an Arduino/CH340, or None."""
    candidates = []
    for p in serial.tools.list_ports.comports():
        blob = f"{p.description} {p.manufacturer} {p.hwid}".lower()
        if any(k in blob for k in ("arduino", "ch340", "usb serial", "wchusb", "usb-serial")):
            candidates.append(p.device)
    if candidates:
        return candidates[0]
    return None


class ArduinoPump:
    """Serial interface for the Arduino.

    Protocol used by the experiment (fixed windows, Python owns the timeline):
        -> TRIAL,<id>,<mmHg>,<ramp_ms>,<failsafe_ms>
             <- INFLATING,<id>,<arduino_ms>   <- PUMP_ON,<id>,<t>
             <- TARGET_REACHED,<id>,<t>,<mmHg>
        -> DEFLATE,<id>
             <- DEFLATING,<id>,<t>   <- DEFLATED,<id>,<t>,<mmHg>,<t_since_deflate>
        -> STOP  <- STOPPED

    Legacy protocol (kept in the firmware for bench tests):
        -> START,<mmHg>  -> INFLATING -> HOLDING -> DEFLATING -> DONE

    Every received line is parsed by poll(). Telemetry updates the pressure
    fields; every other valid line is timestamped with time.perf_counter()
    on arrival and queued, so no caller (drain(), a wait loop) can lose a
    physical event any more.
    """

    def __init__(self, port=None, baud=ARDUINO_BAUD):
        if port is None:
            port = _autodetect_arduino_port()
        if port is None:
            raise PumpError(
                "No se encontró el puerto del Arduino automáticamente. "
                "Fija ARDUINO_PORT (p. ej. \"COM3\") arriba en el archivo."
            )

        self.port = port
        print(f"Abriendo Arduino en {port} @ {baud} baud...")
        # timeout=0 -> non-blocking reads so the Tkinter window doesn't freeze
        self.ser = serial.Serial(port, baud, timeout=0)

        # Opening the port resets the Arduino (DTR); wait for it to boot.
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self._buf = ""

        # Latest MPX5050DP reading (mmHg), used by the UI.
        self.pressure_mmhg = 0.0
        self.target_mmhg   = 0.0
        self.desired_mmhg  = 0.0    # ramp reference (new telemetry field)
        self.pwm           = 0      # pump PWM actually applied (new field)
        self.last_arduino_ms = None

        # Non-telemetry lines waiting to be consumed: (perf_counter, line)
        self._pending = deque()

        # (pc_rx_time, arduino_ms) pairs used to map Arduino millis() to
        # time.perf_counter(). The Uno runs from a ceramic resonator (up to
        # ~0.5 % off), so the mapping only uses samples near the event.
        self._clock_pairs = deque(maxlen=400)   # ~20 s at 20 Hz

        # Number of fields in the last telemetry line (6 = current firmware)
        self.telemetry_fields = None

        # Telemetry sink: called with a dict for each "P," line (set by main)
        self.telemetry_sink = None

        # Telemetry of the current trial: list of dicts (set/cleared by trial code)
        self.trial_telemetry = None
        # {"trial", "condition", "t0"} while a trial runs; used by the CSV sink
        self.telemetry_context = None

        self._handshake()

        print(f"Arduino listo en {port}. Presión actual: {self.pressure_mmhg:.1f} mmHg")

    def _handshake(self):
        """Sends PING and waits for READY to verify the firmware."""
        deadline = time.monotonic() + 5.0
        self.ser.write(b"PING\n")
        self.ser.flush()

        while time.monotonic() < deadline:
            line = self.read_line()

            if line and line.startswith("ZERO,"):
                # Arduino just measured the sensor zero offset.
                print(f"  [arduino] cero del sensor: {line}")

            if line == "READY":
                self._check_firmware_version()
                return

            time.sleep(0.01)

        raise PumpError(
            f"El Arduino en {self.port} no respondió READY. "
            "Revisa que tenga cargado arduino_pressure_controller.ino "
            "y que el baud sea 115200."
        )

    def _check_firmware_version(self):
        """The old firmware also answers READY/ZERO but silently ignores
        TRIAL/DEFLATE. The new one sends 6-field telemetry
        (P,actual,target,desired,pwm,ms); the old one sends 3 fields.
        Checked here so a stale upload fails at connection time, not in
        the middle of the calibration."""
        self.telemetry_fields = None
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and self.telemetry_fields is None:
            self.poll()
            time.sleep(0.01)

        if self.telemetry_fields is None:
            raise PumpError(
                f"El Arduino en {self.port} respondió READY pero no manda telemetría "
                "(líneas P,...). Revisa el firmware."
            )
        if self.telemetry_fields < 6:
            raise PumpError(
                f"El Arduino en {self.port} tiene el FIRMWARE VIEJO cargado "
                f"(telemetría de {self.telemetry_fields} campos, se esperan 6). "
                "No entiende TRIAL/DEFLATE. Sube la versión nueva de "
                "arduino_pressure_controller.ino con el IDE de Arduino y vuelve a correr."
            )

    # ------------------------------------------------------------------
    # Reading
    # ------------------------------------------------------------------

    def poll(self):
        """Reads everything available (non-blocking) and parses all complete
        lines. Telemetry updates the pressure fields; other valid lines are
        queued with their arrival time."""
        try:
            while True:
                chunk = self.ser.read(4096)
                if not chunk:
                    break
                self._buf += chunk.decode("ascii", errors="ignore")
                if len(chunk) < 4096:
                    break
        except Exception:
            raise PumpError("Se perdió la conexión serial con el Arduino.")

        if "\n" not in self._buf:
            return

        now = time.perf_counter()
        *lines, self._buf = self._buf.split("\n")
        for line in lines:
            line = line.strip()
            if line:
                self._process_line(line, now)

    def _process_line(self, line, rx_time):
        # --- Pressure telemetry: P,<actual>,<target>[,<desired>,<pwm>,<ms>] ---
        if line.startswith("P,"):
            parts = line.split(",")
            self.telemetry_fields = len(parts)
            if len(parts) >= 3:
                try:
                    self.pressure_mmhg = float(parts[1])
                    self.target_mmhg   = float(parts[2])
                except ValueError:
                    return
                ard_ms = None
                if len(parts) >= 6:
                    try:
                        self.desired_mmhg = float(parts[3])
                        self.pwm          = int(parts[4])
                        ard_ms            = int(parts[5])
                    except ValueError:
                        ard_ms = None
                if ard_ms is not None:
                    self.last_arduino_ms = ard_ms
                    self._clock_pairs.append((rx_time, ard_ms))
                row = {
                    "pc_rx": rx_time, "arduino_ms": ard_ms,
                    "pressure": self.pressure_mmhg, "target": self.target_mmhg,
                    "desired": self.desired_mmhg, "pwm": self.pwm,
                }
                if self.trial_telemetry is not None:
                    self.trial_telemetry.append(row)
                if self.telemetry_sink is not None:
                    try:
                        self.telemetry_sink(row)
                    except Exception:
                        pass   # logging must never stop the experiment
            return

        # ---- Protocol line ----
        # Only protocol words with the right number of fields are accepted;
        # anything else is a truncated line from a buffer overflow.
        parts = line.split(",")
        expected = MESSAGE_FIELDS.get(parts[0])
        if expected is None or len(parts) not in expected:
            return
        self._pending.append((rx_time, line))

    def read_line(self):
        """Non-blocking. Returns the next protocol line (READY, INFLATING,...)
        or None. Kept for the handshake and simple callers."""
        self.poll()
        if self._pending:
            return self._pending.popleft()[1]
        return None

    def take_events(self):
        """Returns and clears all queued protocol lines as (rx_time, line)."""
        self.poll()
        out = list(self._pending)
        self._pending.clear()
        return out

    def drain(self):
        """Drains the serial buffer so 20 Hz telemetry doesn't overflow it
        between trials, and keeps pressure_mmhg updated. Protocol lines are
        kept in the queue (and the trial code discards stale ones by id)."""
        try:
            self.poll()
        except Exception:
            pass   # A failed drain should not interrupt the experiment

    def arduino_ms_to_pc(self, arduino_ms):
        """Maps an Arduino millis() value to time.perf_counter() seconds.

        offset = min(pc_rx - arduino_ms/1000) over telemetry lines near that
        moment: the minimum is the line that waited the least in the serial
        buffers, so it approximates the true clock offset (+ ~1 ms USB).
        Only pairs within 2 s of the event are used, so resonator drift
        between the two clocks (up to ~5 ms/s) does not enter.
        Returns None if no telemetry is available."""
        near = [rx - ms / 1000.0 for rx, ms in self._clock_pairs
                if abs(ms - arduino_ms) <= 2000]
        if not near:
            near = [rx - ms / 1000.0 for rx, ms in list(self._clock_pairs)[-40:]]
        if not near:
            return None
        return arduino_ms / 1000.0 + min(near)

    def arduino_clock_ppm(self):
        """Rate error of the Arduino clock relative to the PC, in ppm
        (positive = Arduino runs slow), from the telemetry history. None if
        there is not enough history."""
        pairs = list(self._clock_pairs)
        if len(pairs) < 100:
            return None
        n = len(pairs) // 4
        a = min(pairs[:n],  key=lambda p: p[0] - p[1] / 1000.0)
        b = min(pairs[-n:], key=lambda p: p[0] - p[1] / 1000.0)
        d_ard = (b[1] - a[1]) / 1000.0
        if d_ard < 5.0:
            return None
        return ((b[0] - a[0]) / d_ard - 1.0) * 1e6

    # ------------------------------------------------------------------
    # Writing
    # ------------------------------------------------------------------

    def _write(self, text):
        self.ser.write(text.encode("ascii"))
        self.ser.flush()

    def send_trial(self, trial_id, mmhg, ramp_ms, failsafe_ms):
        """Starts a fixed-window trial. Returns perf_counter() at the write."""
        if mmhg <= 0 or mmhg > MAX_SAFE_MMHG:
            raise PumpError(
                f"Presión objetivo fuera de rango: {mmhg} mmHg "
                f"(el máximo permitido es {MAX_SAFE_MMHG:.0f})."
            )

        target = int(round(mmhg))

        # Set target locally first so the UI knows it immediately.
        self.target_mmhg = float(target)

        t = time.perf_counter()
        try:
            self._write(f"TRIAL,{int(trial_id)},{target},{int(ramp_ms)},{int(failsafe_ms)}\n")
        except Exception:
            raise PumpError("No se pudo escribir TRIAL al Arduino (conexión serial).")
        return t

    def send_deflate(self, trial_id):
        """Opens the vent (end of HOLDING). Returns perf_counter() at the write.
        If the write fails, a STOP is attempted too (it also vents)."""
        t = time.perf_counter()
        try:
            self._write(f"DEFLATE,{int(trial_id)}\n")
        except Exception:
            self.send_stop()
            raise PumpError("No se pudo escribir DEFLATE al Arduino (conexión serial).")
        return t

    def send_start(self, mmhg):
        """LEGACY self-timed cycle (not used by the experiment any more)."""
        if mmhg <= 0 or mmhg > MAX_SAFE_MMHG:
            raise PumpError(
                f"Presión objetivo fuera de rango: {mmhg} mmHg "
                f"(el máximo permitido es {MAX_SAFE_MMHG:.0f})."
            )
        target = int(round(mmhg))
        self.target_mmhg = float(target)
        self._write(f"START,{target}\n")

    def send_stop(self):
        try:
            self.ser.write(b"STOP\n")
            self.ser.flush()
        except Exception:
            pass  # Never let an error block an emergency STOP

    def close(self):
        """Sends STOP, then closes the port."""
        try:
            self.send_stop()
            time.sleep(0.1)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


# --- Simulated "brain" signal amplitude as a function of pressure ---
AMPLITUDE_BASE_UV = 50.0   # amplitude with no pressure / between phases
AMPLITUDE_GAIN_UV = 2.5    # amplitude increase per mmHg


def pressure_to_amplitude(pressure):
    return AMPLITUDE_BASE_UV + AMPLITUDE_GAIN_UV * pressure


class BCI2000DisconnectedError(Exception):
    """Stops the whole experiment if BCI2000 disconnects."""
    pass


def set_signal_amplitude(bci, amplitude_uv):
    """Updates SineAmplitude live (no SetConfig or run stop needed)."""
    try:
        ok = bci.Execute(f"SET PARAMETER SineAmplitude {amplitude_uv:.1f}")
        if not ok:
            print(f"    ⚠  No se pudo actualizar SineAmplitude: {bci.Result}")
        return ok
    except Exception:
        raise BCI2000DisconnectedError("Se perdió la conexión con BCI2000 (SineAmplitude).")


def safe_set_state(bci, name, value):
    """SetStateVariable; stops the experiment if BCI2000 disconnected."""
    try:
        bci.SetStateVariable(name, value)
        return True
    except Exception:
        raise BCI2000DisconnectedError(f"Se perdió la conexión con BCI2000 ({name}).")


def safe_set_states(bci, **states):
    """Sets several states in one BCI2000 call to minimize misalignment
    between states that should change together. Not perfectly aligned,
    since BCI2000 applies states per sample block."""
    cmd = "; ".join(f"SET STATE {name} {value}" for name, value in states.items())
    try:
        ok = bci.Execute(cmd)
        if not ok:
            print(f"    ⚠  No se pudieron actualizar estados: {bci.Result}")
        return ok
    except Exception:
        raise BCI2000DisconnectedError(f"Se perdió la conexión con BCI2000 ({cmd}).")


_NUMBER_RE = re.compile(r"[-+]?\d+(?:\.\d+)?")


def _bci_query_number(bci, command):
    """Runs a BCI2000 GET command and returns the first number in its result,
    or None. Never raises (reads are optional; the next SET detects a real
    disconnection)."""
    try:
        bci.Execute(command)
        m = _NUMBER_RE.search(str(getattr(bci, "Result", "")))
        return float(m.group(0)) if m else None
    except Exception:
        return None


# =============================================================================
# BCI2000 EVENT CLOCK
#
# How a state change reaches the EEG file (verified on P001S001R02-R05.dat):
#   Python -> Operator (telnet "SET STATE") -> Source module, which applies the
#   new value to the NEXT sample block it sends. All 575 StimulusPhase changes
#   in those four files fall exactly on a SampleBlockSize boundary. So a state
#   change is quantized to one block (32 samples = 125 ms at 256 Hz), and
#   WHICH block it lands in depends on where inside the block the command
#   arrives. time.sleep(8) cannot control that.
#
# What this class does:
#   1. Reads SamplingRate and SampleBlockSize from BCI2000.
#   2. Expresses every event duration as a whole number of blocks.
#   3. At the start of each trial, watches the SourceTime state (one new value
#      per block) until it changes, then waits BLOCK_ALIGN_FRACTION of a block.
#      That puts the command in the same place relative to the block clock
#      every time, far from the boundary.
#   4. All following transitions of the trial are absolute perf_counter()
#      deadlines at exact multiples of the block period from that point, so
#      they keep the same phase and land exactly N blocks apart.
#
# If SourceTime can't be read, it falls back to monotonic deadlines only:
# durations are still exact in PC time, but each transition may land in
# either of two adjacent blocks (±1 block in the EEG file).
# =============================================================================

class EventClock:

    def __init__(self, bci, block_lock=BLOCK_LOCK):
        self.bci = bci
        self.sampling_rate = _bci_query_number(bci, "GET PARAMETER SamplingRate")
        self.block_size    = _bci_query_number(bci, "GET PARAMETER SampleBlockSize")

        if self.sampling_rate and self.block_size and self.sampling_rate > 0:
            self.block_s = self.block_size / self.sampling_rate
        else:
            self.block_s = None

        self.block_locked = False
        if block_lock and self.block_s:
            self.block_locked = self._probe_source_time()

        self.mode = "block_locked" if self.block_locked else "deadline_only"
        self.align_failures = 0

    def describe(self):
        if self.block_s:
            per_s = 1.0 / self.block_s
            txt = (f"SamplingRate={self.sampling_rate:g} Hz, SampleBlockSize={self.block_size:g} "
                   f"-> 1 bloque = {self.block_s*1000:.1f} ms ({per_s:g} bloques/s)")
        else:
            txt = "SamplingRate/SampleBlockSize no disponibles"
        return f"{txt}; modo de sincronía: {self.mode}"

    def info(self):
        return {
            "sampling_rate_hz":  self.sampling_rate,
            "sample_block_size": self.block_size,
            "block_ms":          None if self.block_s is None else self.block_s * 1000.0,
            "sync_mode":         self.mode,
            "block_align_fraction": BLOCK_ALIGN_FRACTION if self.block_locked else None,
        }

    def _source_time(self):
        v = _bci_query_number(self.bci, "GET STATE SourceTime")
        return None if v is None else int(v)

    def _probe_source_time(self):
        """True if SourceTime can be read and advances once per block."""
        first = self._source_time()
        if first is None:
            return False
        changes = []
        last, last_t = first, time.perf_counter()
        end = last_t + 6 * self.block_s + 0.5
        while time.perf_counter() < end and len(changes) < 4:
            v = self._source_time()
            if v is None:
                return False
            if v != last:
                changes.append((v - last) % 65536)
                last = v
            time.sleep(0.001)
        if len(changes) < 3:
            return False
        block_ms = self.block_s * 1000.0
        # SourceTime is in ms (16 bit); consecutive blocks differ by ~block_ms.
        # A polling miss shows up as a multiple; accept those too.
        for d in changes:
            k = round(d / block_ms)
            if k < 1 or abs(d - k * block_ms) > 0.35 * block_ms:
                return False
        return True

    def duration(self, seconds):
        """(scheduled_seconds, n_blocks) for an event of `seconds`.
        Rounded UP to whole blocks. At 256 Hz / 32 samples a whole second is
        exactly 8 blocks, so nothing changes; with other settings the event
        can be slightly longer than `seconds` but it is identical in every
        trial, which is the requirement."""
        if not self.block_s:
            return float(seconds), None
        n = int(math.ceil(seconds / self.block_s - 1e-9))
        return n * self.block_s, n

    def is_whole_blocks(self, seconds):
        if not self.block_s:
            return True
        n = seconds / self.block_s
        return abs(n - round(n)) < 1e-9

    def align(self):
        """Returns (t, source_time): a perf_counter() instant placed
        BLOCK_ALIGN_FRACTION of a block after a block edge seen by the
        Operator, spinning until it. Without block lock returns (now, None)."""
        if not self.block_locked:
            return time.perf_counter(), None

        start = time.perf_counter()
        last = self._source_time()
        while last is not None and time.perf_counter() - start < 4 * self.block_s:
            v = self._source_time()
            if v is None:
                break
            if v != last:
                t = time.perf_counter() + BLOCK_ALIGN_FRACTION * self.block_s
                while time.perf_counter() < t:
                    pass
                return t, v
            time.sleep(0.0005)

        self.align_failures += 1
        print("    ⚠  No se detectó el borde de bloque (SourceTime); "
              "este trial usa solo deadlines monotónicos.")
        return time.perf_counter(), None

    def source_time(self):
        """SourceTime right now (for the log), or None."""
        if not self.block_locked:
            return None
        return self._source_time()
# -----------------------------------------------------------------------------


# Fixation Cross + pressure bar + rest
# **********************************************

class FixationCross:

    def __init__(
        self,
        bg_color: str = "black",
        cross_color: str = "white",
        arm_pct: float = 0.35,
        thickness_pct: float = 0.15,
        win_width: int = 800,
        win_height: int = 600,
    ):
        self.cross_color = cross_color
        self.arm_pct = arm_pct
        self.thickness_pct = thickness_pct
        self.win_width = win_width
        self.win_height = win_height
        self._running = True
        self._escape_pressed = False
        self._bar_fraction = 0.0
        self._rest_text_id = None
        self._rest_num_id = None
        self._cross_visible = True   # visible by default (rest / event 0)
        self._calibration_banner_id = None
        self._bar_visible = False        # whether to redraw the bar on resize
        self._current_bar_label = None   # current bar text (INFLATING/HOLDING/DEFLATING)

        # Set to pump.drain in main(). Called on every wait tick so the
        # serial buffer never overflows and phases aren't lost.
        self.serial_drain = None

        self.root = tk.Tk()
        self.root.title("Fixation Cross")
        self.root.configure(bg=bg_color)
        self.root.geometry(f"{win_width}x{win_height}")   # normal window, not fullscreen
        self.root.resizable(True, True)                   # can be maximized/moved to another monitor

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.bind("<Escape>", self._on_escape)

        self.canvas = tk.Canvas(self.root, bg=bg_color, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda e: self._on_configure())

        self.root.update()
        self.root.update_idletasks()
        self.root.after(100, self._draw_cross)

    def _on_configure(self):
        """On any window resize, redraws everything visible with the current geometry."""
        self._draw_cross()
        if self._bar_visible:
            self._redraw_bar_frame()
            self._draw_bar_fill()
            if self._current_bar_label:
                self.set_bar_label(self._current_bar_label)

    def get_win_size(self):
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w <= 1 or h <= 1:
            w, h = self.win_width, self.win_height
        return w, h

    def _on_close(self):
        self._running = False
        self._escape_pressed = True

    def _on_escape(self, event=None):
        self._running = False
        self._escape_pressed = True

    def _draw_cross(self):
        self.canvas.delete("cross")

        if not self._cross_visible:
            return

        w, h = self.get_win_size()

        lado = min(w, h)
        s = int(lado * self.arm_pct)
        t = int(lado * self.thickness_pct) // 2

        cx, cy = w // 2, h // 2

        self.canvas.create_rectangle(
            cx - s, cy - t, cx + s, cy + t,
            fill=self.cross_color, outline=self.cross_color, tags="cross",
        )
        self.canvas.create_rectangle(
            cx - t, cy - s, cx + t, cy + s,
            fill=self.cross_color, outline=self.cross_color, tags="cross",
        )

    def show_fixation_cross(self):
        """Shows the cross (event 0 / rest)."""
        self._cross_visible = True
        self._draw_cross()

    def hide_fixation_cross(self):
        """Hides the cross (start of event 1 / INFLATING)."""
        self._cross_visible = False
        self.canvas.delete("cross")

    def wait_for_start(self, message: str = "Presione ENTER para iniciar el protocolo"):
        """Shows a message and waits for the researcher to press ENTER."""
        w, h = self.get_win_size()
        msg_id = self.canvas.create_text(
            w // 2,
            h // 2 + 80,
            text=message,
            fill="gray70",
            font=("Helvetica", 16),
            tags="start_msg",
        )
        self.root.update()

        self._start_pressed = False

        def on_enter(event=None):
            self._start_pressed = True

        self.root.bind("<Return>", on_enter)

        while not self._start_pressed:
            if not self._running:
                break
            self._tick()
            time.sleep(0.01)

        self.canvas.delete("start_msg")
        self.root.unbind("<Return>")
        self.root.update()

    def _tick(self):
        """One wait tick: refreshes the window and drains the serial port."""
        if self.serial_drain is not None:
            self.serial_drain()
        self.root.update()

    def pump(self, seconds: float) -> bool:
        """Keeps the window responsive for `seconds`.
        Returns False if the user closed the window or pressed Escape."""
        end = time.time() + seconds
        while time.time() < end:
            if not self._running:
                return False
            self._tick()
            time.sleep(0.01)
        return True

    def update(self):
        """Single Tkinter event loop tick."""
        if self._running:
            self.root.update()

    def close(self):
        self._running = False
        try:
            self.root.destroy()
        except Exception:
            pass

    # --- Calibration banner (visible during all of trial 0) ---

    def show_calibration_banner(self, text="CALIBRACIÓN"):
        """Shows a fixed banner at the top during all calibration stages."""
        self.canvas.delete("calibration_banner")
        w, h = self.get_win_size()
        pad_x, pad_y = 24, 14
        text_id = self.canvas.create_text(
            w // 2, 40, text=text, fill="white",
            font=("Helvetica", 20, "bold"), tags="calibration_banner",
        )
        bbox = self.canvas.bbox(text_id)
        if bbox:
            self.canvas.create_rectangle(
                bbox[0] - pad_x, bbox[1] - pad_y, bbox[2] + pad_x, bbox[3] + pad_y,
                fill="#B91C1C", outline="", tags="calibration_banner",
            )
            self.canvas.tag_raise(text_id)
        self._calibration_banner_id = text_id
        self.root.update_idletasks()

    def hide_calibration_banner(self):
        self.canvas.delete("calibration_banner")
        self._calibration_banner_id = None

    # --- Pressure bar (INFLATING / HOLDING / DEFLATING) ---

    def _bar_geometry(self):
        w, h = self.get_win_size()
        bar_height = 60
        bar_width  = int(w * 0.75)
        x_left     = (w - bar_width) // 2
        x_right    = x_left + bar_width
        y_center   = h // 2
        y_top      = y_center - bar_height // 2
        y_bottom   = y_center + bar_height // 2
        return x_left, x_right, y_top, y_bottom, bar_width, bar_height

    def show_pressure_bar(self):
        self.canvas.delete("bar_frame")
        self.canvas.delete("bar_label")
        self._bar_visible = True
        self._current_bar_label = None
        self._redraw_bar_frame()
        self._bar_fraction = 0.0
        self._draw_bar_fill()

    def _redraw_bar_frame(self):
        self.canvas.delete("bar_frame")
        x_left, x_right, y_top, y_bottom, _, _ = self._bar_geometry()
        self.canvas.create_rectangle(
            x_left, y_top, x_right, y_bottom,
            outline="white", width=2, tags="bar_frame",
        )

    def set_bar_label(self, text: str):
        self._current_bar_label = text
        self.canvas.delete("bar_label")
        x_left, x_right, y_top, _, _, _ = self._bar_geometry()
        self.canvas.create_text(
            (x_left + x_right) // 2, y_top - 24,
            text=text, fill="white", font=("Helvetica", 16, "bold"),
            tags="bar_label",
        )
        self.root.update_idletasks()

    def _draw_bar_fill(self):
        self.canvas.delete("bar_fill")
        x_left, x_right, y_top, y_bottom, bar_width, _ = self._bar_geometry()
        fill_w = int(bar_width * self._bar_fraction)
        if fill_w > 0:
            self.canvas.create_rectangle(
                x_left, y_top, x_left + fill_w, y_bottom,
                fill="#DC2626", outline="", tags="bar_fill",
            )

    def set_bar_fraction(self, fraction: float):
        self._bar_fraction = max(0.0, min(1.0, fraction))
        self._draw_bar_fill()
        self.root.update_idletasks()

    def animate_bar_to(self, target_fraction: float, duration: float, fps: int = 30) -> bool:
        start_fraction = self._bar_fraction
        start_time = time.time()
        interval = 1.0 / fps

        while True:
            if not self._running:
                return False
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            t = elapsed / duration
            self.set_bar_fraction(start_fraction + (target_fraction - start_fraction) * t)
            self._tick()
            time.sleep(interval)

        self.set_bar_fraction(target_fraction)
        return True

    def hide_pressure_bar(self):
        self.canvas.delete("bar_frame")
        self.canvas.delete("bar_fill")
        self.canvas.delete("bar_label")
        self._bar_fraction = 0.0
        self._bar_visible = False
        self._current_bar_label = None

    # --- Rest: large label + countdown + emptying bar ---

    def _draw_rest_text(self, label: str, remaining: float):
        """Draws/updates the rest label and countdown just above the bar."""
        x_left, x_right, y_top, _, _, _ = self._bar_geometry()
        cx = (x_left + x_right) // 2
        y_word = y_top - 90
        y_num  = y_top - 40

        word_text = label
        num_text  = f"{remaining:0.0f}s"

        if self._rest_text_id and self.canvas.find_withtag("rest_label"):
            self.canvas.itemconfig(self._rest_text_id, text=word_text)
            self.canvas.coords(self._rest_text_id, cx, y_word)
            self.canvas.itemconfig(self._rest_num_id, text=num_text)
            self.canvas.coords(self._rest_num_id, cx, y_num)
        else:
            self._rest_text_id = self.canvas.create_text(
                cx, y_word, text=word_text, fill="white",
                font=("Helvetica", 54, "bold"), tags="rest_label",
            )
            self._rest_num_id = self.canvas.create_text(
                cx, y_num, text=num_text, fill="gray70",
                font=("Helvetica", 16), tags="rest_label",
            )

    def hide_rest_label(self):
        self.canvas.delete("rest_label")
        self._rest_text_id = None
        self._rest_num_id = None

    def countdown_rest(self, duration: float, label: str = "DESCANSO", fps: int = 10) -> bool:
        self.show_pressure_bar()
        self.set_bar_fraction(1.0)

        start = time.time()
        interval = 1.0 / fps

        while True:
            if not self._running:
                return False
            elapsed = time.time() - start
            remaining = duration - elapsed
            if remaining <= 0:
                break
            self.set_bar_fraction(max(0.0, remaining / duration))
            self._draw_rest_text(label, remaining)
            self._tick()
            time.sleep(interval)

        self.hide_rest_label()
        self.hide_pressure_bar()
        return True
# -----------------------------------------------------------------------------


def create_block(none, low, high):
    """Builds a shuffled block. All three conditions use the real pump;
    NONE is the lowest pressure (8 mmHg by default)."""
    trials = (
        [(NONE_LABEL, none)] * TRIALS_PER_COND
        + [(LOW_LABEL,  low)]  * TRIALS_PER_COND
        + [(HIGH_LABEL, high)] * TRIALS_PER_COND
    )
    random.shuffle(trials)
    return trials


def get_results_path(participant_data):
    p_num = str(participant_data["participant_number"]).zfill(3)
    run   = str(participant_data["run_number"]).zfill(2)
    return f"participant_{p_num}_run{run}_results.json"


def load_or_create_results(participant_data):
    path = get_results_path(participant_data)
    if os.path.exists(path):
        with open(path, "r") as f:
            return json.load(f)
    return {
        "participant_name":   participant_data["name"],
        "participant_number": participant_data["participant_number"],
        "run_number":         participant_data["run_number"],
        "session_number":     participant_data["session_number"],
        "start_timestamp":    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "blocks":             [],
    }


def save_results(participant_data, results):
    path = get_results_path(participant_data)
    with open(path, "w") as f:
        json.dump(results, f, indent=4)


def append_trial_to_results(participant_data, results, block_number, trial_data):
    block_entry = None
    for b in results["blocks"]:
        if b["block_number"] == block_number:
            block_entry = b
            break
    if block_entry is None:
        block_entry = {"block_number": block_number, "trials": []}
        results["blocks"].append(block_entry)

    block_entry["trials"].append(trial_data)

    save_results(participant_data, results)
    print(f"    ✔  Saved (Block {block_number}, Trial {trial_data['trial']})")


# =============================================================================
# Trial timing record
# =============================================================================

class TrialRecord:
    """Everything needed afterwards to check synchronization for one trial.

    All times are seconds relative to t0 = the scheduled onset of INFLATING
    (perf_counter clock).
      experimental[...]  scheduled / sent times of BCI2000 state changes
      physical[...]      Arduino events mapped to the PC clock
      physical_arduino_ms[...]  the same events in the Arduino's own clock
                                (ms since it received TRIAL), unmodified
    """

    def __init__(self, trial_id, condition, pressure):
        self.trial_id  = trial_id
        self.condition = condition
        self.pressure  = pressure
        self.t0        = None
        self.experimental = {}
        self.physical = {}
        self.physical_arduino_ms = {}
        self.physical_pressure = {}
        self.arduino_trial_start_ms = None
        self.notes = []
        self.stale_lines = []
        self.telemetry = []
        self.ramp_ms = 0

    def rel(self, t):
        return None if (t is None or self.t0 is None) else round(t - self.t0, 4)

    def mark_experimental(self, name, scheduled, sent, source_time=None):
        err_ms = None if scheduled is None else (sent - scheduled) * 1000.0
        self.experimental[name] = {
            "scheduled_s":    self.rel(scheduled),
            "sent_s":         self.rel(sent),
            "send_error_ms":  None if err_ms is None else round(err_ms, 2),
            "source_time_after": source_time,
        }
        if err_ms is not None and err_ms > LATE_TRANSITION_WARN_MS:
            self.notes.append(f"{name} sent {err_ms:.1f} ms late")
            print(f"    ⚠  {name} se envió {err_ms:.1f} ms tarde")

    def handle_line(self, pump, rx_time, line):
        """Records a physical event from the Arduino. Raises PumpFaultError on
        FAULT/STOPPED. Lines that belong to another trial id are kept as stale."""
        print(f"    [arduino] {line}")

        if line.startswith(FAULT_WORDS):
            raise PumpFaultError(
                f"El Arduino abortó el trial: '{line}'. Ya venteó el manguito."
            )

        parts = line.split(",")
        word = parts[0]
        if word not in ("INFLATING", "PUMP_ON", "TARGET_REACHED", "DEFLATING", "DEFLATED") \
                or len(parts) < 3:
            self.stale_lines.append(line)
            return
        try:
            line_id = int(parts[1])
            value   = int(parts[2])
        except ValueError:
            self.stale_lines.append(line)
            return
        if line_id != self.trial_id:
            self.stale_lines.append(line)
            return

        if word == "INFLATING":
            # ack; value = Arduino millis() at TRIAL receipt
            if self.arduino_trial_start_ms is None:
                self.arduino_trial_start_ms = value
                self.physical_arduino_ms["trial_command_received"] = 0
                self._map("trial_command_received", pump, 0, rx_time)
            return

        key = {"PUMP_ON": "pump_on",
               "TARGET_REACHED": "target_reached",
               "DEFLATING": "deflate_valve_open",
               "DEFLATED": "deflated"}[word]
        if key in self.physical:
            return
        self.physical_arduino_ms[key] = value
        if word in ("TARGET_REACHED", "DEFLATED") and len(parts) >= 4:
            try:
                self.physical_pressure[key] = float(parts[3])
            except ValueError:
                pass
        self._map(key, pump, value, rx_time)

    def _map(self, key, pump, t_ms, rx_time):
        """Arduino time -> PC time. Falls back to the arrival time if the
        mapping is not possible (then includes serial latency, ~1-10 ms)."""
        pc = None
        if self.arduino_trial_start_ms is not None:
            pc = pump.arduino_ms_to_pc(self.arduino_trial_start_ms + t_ms)
        if pc is None or pc > rx_time:
            pc = rx_time
        self.physical[key] = self.rel(pc)
        self.physical[key + "_rx"] = self.rel(rx_time)

    def summary(self):
        ph = self.physical
        ex = self.experimental
        out = {
            "trial_id":  self.trial_id,
            "condition": self.condition,
            "target_mmhg": self.pressure,
            # absolute time.perf_counter() of t0, to line this trial up with
            # the telemetry CSV (see results["sync"]["telemetry_origin_perf_counter"])
            "t0_perf_counter": self.t0,
            "experimental_s": ex,
            "physical_s": {k: v for k, v in ph.items() if not k.endswith("_rx")},
            "physical_rx_s": {k[:-3]: v for k, v in ph.items() if k.endswith("_rx")},
            "physical_arduino_ms": self.physical_arduino_ms,
            "physical_pressure_mmhg": self.physical_pressure,
            "notes": self.notes,
        }
        if self.stale_lines:
            out["stale_or_foreign_lines"] = self.stale_lines

        # Physical vs experimental boundaries (the whole point of the split)
        def s(name):
            e = ex.get(name)
            return None if e is None else e["sent_s"]
        tr, df = ph.get("target_reached"), ph.get("deflated")
        if tr is not None and s("HOLDING") is not None:
            out["target_reached_before_inflating_end"] = tr <= s("HOLDING")
            out["target_margin_before_holding_s"] = round(s("HOLDING") - tr, 4)
        if df is not None and s("REST") is not None:
            out["deflated_before_deflating_end"] = df <= s("REST")
        if ph.get("deflate_valve_open") is not None and df is not None:
            out["physical_deflation_s"] = round(df - ph["deflate_valve_open"], 4)
        if ph.get("pump_on") is not None and tr is not None:
            out["physical_inflation_s"] = round(tr - ph["pump_on"], 4)

        # Pressure summary from telemetry
        tel = self.telemetry
        if tel:
            p = [r["pressure"] for r in tel]
            out["max_pressure_mmhg"] = max(p)
            for name in ("HOLDING", "DEFLATING"):
                t_ev = s(name)
                if t_ev is not None:
                    before = [r for r in tel if self.rel(r["pc_rx"]) <= t_ev]
                    if before:
                        out[f"pressure_at_{name.lower()}_start_mmhg"] = before[-1]["pressure"]
            # pressure maintenance after the target (top-ups, +-5 mmHg limit)
            t_tr = ph.get("target_reached")
            t_def = s("DEFLATING")
            if t_tr is not None and t_def is not None:
                kept = [r for r in tel if t_tr + 0.3 <= self.rel(r["pc_rx"]) <= t_def]
                if kept:
                    out["maintained_min_mmhg"] = min(r["pressure"] for r in kept)
                    out["maintained_max_mmhg"] = max(r["pressure"] for r in kept)
                    out["topups"] = sum(1 for a, b in zip(kept, kept[1:])
                                        if a["pwm"] == 0 and b["pwm"] > 0)
                    worst = max(abs(out["maintained_min_mmhg"] - self.pressure),
                                abs(out["maintained_max_mmhg"] - self.pressure))
                    out["maintained_within_5mmhg"] = worst <= 5.0
            # ramp tracking error during INFLATING (LOW/NONE)
            t_end = s("HOLDING")
            ramp = [r for r in tel if r["desired"] and r["pwm"] is not None
                    and t_end is not None and 0 <= self.rel(r["pc_rx"]) <= t_end
                    and r["desired"] < self.pressure]
            if ramp:
                errs = [r["pressure"] - r["desired"] for r in ramp]
                out["ramp_tracking_error_mmhg"] = {
                    "mean": round(sum(errs) / len(errs), 2),
                    "min":  round(min(errs), 2),
                    "max":  round(max(errs), 2),
                }
        return out


# =============================================================================
# Waiting without drifting
# =============================================================================

def wait_until(deadline, pump, cross, rec, bar_target=None, label=None, check=None,
               stop_on=None):
    """Keeps the window and serial link alive until the absolute perf_counter()
    `deadline`. The last SPIN_BEFORE_DEADLINE_S are a pure spin (no Tk, no
    serial) so the caller can send the transition on time.

    stop_on: optional callable() -> bool; returns early (value "event") when
             true. Only used by the calibration, which is event driven.
    check:   optional callable() run every iteration (e.g. ack timeout).

    Returns True at the deadline, "event" if stop_on fired, None if the
    window was closed. Raises PumpFaultError on an Arduino FAULT.
    """
    last_log = time.perf_counter()
    prev_logged_p = pump.pressure_mmhg

    while True:
        now = time.perf_counter()
        remaining = deadline - now
        if remaining <= SPIN_BEFORE_DEADLINE_S:
            break

        if not cross._running:
            return None

        for rx_time, line in pump.take_events():
            rec.handle_line(pump, rx_time, line)

        if check is not None:
            check()

        if stop_on is not None and stop_on():
            return "event"

        current = pump.pressure_mmhg

        # Bar follows physical pressure. No numbers are shown to the participant
        # (would reveal the condition and bias pain reports/EEG); mmHg stay in the console.
        if bar_target and bar_target > 0:
            cross.set_bar_fraction(max(0.0, min(1.0, current / bar_target)))

        # Researcher-only console log: pressure, reference, PWM and rise rate.
        if bar_target and now - last_log >= PRESSURE_LOG_INTERVAL:
            rate = (current - prev_logged_p) / max(1e-6, now - last_log)
            t_rel = rec.rel(now)
            print(f"      [{label or '?'}] {current:5.1f} / {bar_target:.0f} mmHg   "
                  f"ref {pump.desired_mmhg:5.1f}  pwm {pump.pwm:3d}   "
                  f"({rate:+.1f} mmHg/s)   t={t_rel if t_rel is not None else 0:.1f}s")
            last_log      = now
            prev_logged_p = current

        cross.update()
        time.sleep(min(0.005, max(0.0, remaining - SPIN_BEFORE_DEADLINE_S)))

    while time.perf_counter() < deadline:
        pass
    return True


def wait_for_cuff_empty(pump, cross, timeout=CUFF_EMPTY_WAIT_S):
    """Before a new trial: the cuff must be empty. Normally returns at once
    (the previous deflation finished during the rest). Returns the seconds
    waited, or None if the window was closed."""
    start = time.perf_counter()
    pump.drain()
    if pump.pressure_mmhg < CUFF_EMPTY_MMHG:
        return 0.0
    print(f"    Esperando a que el manguito se vacíe ({pump.pressure_mmhg:.1f} mmHg)...")
    while time.perf_counter() - start < timeout:
        if not cross._running:
            return None
        pump.drain()
        if pump.pressure_mmhg < CUFF_EMPTY_MMHG:
            return time.perf_counter() - start
        cross.update()
        time.sleep(0.01)
    raise PumpTimeoutError(
        f"El manguito no se vació en {timeout:.0f} s "
        f"({pump.pressure_mmhg:.1f} mmHg). Revisa válvula y mangueras."
    )


def _ack_checker(pump, rec, resend):
    """Returns a check() that resends TRIAL (same id) until the Arduino
    acknowledges it, and raises PumpTimeoutError after ACK_TIMEOUT_S."""
    state = {"t0": time.perf_counter(), "last": time.perf_counter(), "n": 1}

    def check():
        if rec.arduino_trial_start_ms is not None:
            return
        now = time.perf_counter()
        if now - state["t0"] > ACK_TIMEOUT_S:
            raise PumpTimeoutError(
                f"El Arduino no confirmó TRIAL {rec.trial_id} en {ACK_TIMEOUT_S:.1f} s. "
                "Si la bomba no arrancó, lo más probable es que el Arduino tenga el "
                "firmware viejo: sube arduino_pressure_controller.ino otra vez."
            )
        if now - state["last"] >= ACK_RESEND_S:
            state["n"] += 1
            print(f"    (sin ack del Arduino; reenviando TRIAL — intento {state['n']})")
            resend()
            state["last"] = now
    return check


# =============================================================================
# Fixed-window trial: INFLATING -> HOLDING -> DEFLATING with standardized
# durations. Python/BCI2000 changes the state at the deadline, whatever the
# cuff is doing. Arduino messages are only logged.
# =============================================================================

def run_fixed_window_phases(pump, cross, bci, clock, timing, events, rec, t_inflating):
    """t_inflating: perf_counter() deadline of the INFLATING onset (already on
    the trial's block-locked grid). The REST state (4) is set here too, at
    the end of DEFLATING, so that boundary is also deadline driven.

    Returns True on success, None if the window was closed.
    """
    pressure  = rec.pressure
    condition = rec.condition

    d_inf  = timing["inflating_scheduled_s"]
    d_hold = timing["holding_scheduled_s"]
    d_def  = timing["deflating_scheduled_s"]

    t_holding   = t_inflating + d_inf
    t_deflating = t_holding + d_hold
    t_rest      = t_deflating + d_def

    use_ramp = condition == LOW_LABEL or (condition == NONE_LABEL and RAMP_NONE)
    ramp_ms  = int(round(max(0.5, d_inf - RAMP_END_MARGIN_S) * 1000)) if use_ramp else 0
    failsafe_ms = int(round((d_inf + d_hold + FAILSAFE_GRACE_S) * 1000))
    rec.ramp_ms = ramp_ms

    def log(phase, vector, t):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        events.append({"phase": phase, "vector": vector, "timestamp": ts,
                       "t_rel_s": rec.rel(t)})

    # ---------- INFLATING ----------
    # Arduino command first (a non-blocking serial write, microseconds), then
    # the BCI2000 state (a TCP round trip, ~1-3 ms). Same order in DEFLATING,
    # so the offset between the physical command and the state change is the
    # same in every trial and every condition.
    while time.perf_counter() < t_inflating:
        pass
    sent = pump.send_trial(rec.trial_id, pressure, ramp_ms, failsafe_ms)
    safe_set_states(bci, StimulusPhase=1, PressureTarget=pressure)
    rec.mark_experimental("INFLATING", t_inflating, sent, clock.source_time())
    log("INFLATING", 1, sent)

    cross.hide_fixation_cross()
    cross.show_pressure_bar()
    cross.set_bar_label("INFLATING")
    set_signal_amplitude(bci, pressure_to_amplitude(pressure))
    print(f"    INFLATING  ({d_inf:g} s fijo; {'rampa ' + str(ramp_ms) + ' ms' if ramp_ms else 'velocidad máxima'} -> {pressure} mmHg)")

    check = _ack_checker(pump, rec,
                         lambda: pump.send_trial(rec.trial_id, pressure, ramp_ms, failsafe_ms))
    got = wait_until(t_holding, pump, cross, rec, bar_target=pressure,
                     label="INFLATING", check=check)
    if got is None:
        return None
    if rec.arduino_trial_start_ms is None:
        raise PumpTimeoutError(f"El Arduino nunca confirmó TRIAL {rec.trial_id}.")

    # ---------- HOLDING ----------
    # Happens at the deadline even if TARGET_REACHED came earlier (normal) or
    # has not come yet (then the Arduino keeps inflating toward the target,
    # never above it, and the record flags it).
    safe_set_states(bci, StimulusPhase=2)
    sent = time.perf_counter()
    rec.mark_experimental("HOLDING", t_holding, sent, clock.source_time())
    log("HOLDING", 2, sent)
    cross.set_bar_label("HOLDING")
    if "target_reached" not in rec.physical:
        rec.notes.append("target not reached at INFLATING end")
        print("    ⚠  El objetivo aún no se alcanzaba al terminar INFLATING "
              "(la bomba sigue hasta el objetivo, nunca por encima).")

    got = wait_until(t_deflating, pump, cross, rec, bar_target=pressure, label="HOLDING")
    if got is None:
        return None

    # ---------- DEFLATING ----------
    sent = pump.send_deflate(rec.trial_id)
    safe_set_states(bci, StimulusPhase=3, PressureTarget=0)
    rec.mark_experimental("DEFLATING", t_deflating, sent, clock.source_time())
    log("DEFLATING", 3, sent)
    cross.set_bar_label("DEFLATING")
    set_signal_amplitude(bci, AMPLITUDE_BASE_UV)

    # The valve is open from here on. DEFLATED is logged when it arrives but
    # does not end the event; nothing ever holds pressure to fill the window.
    got = wait_until(t_rest, pump, cross, rec, bar_target=pressure, label="DEFLATING")
    if got is None:
        return None

    # ---------- end of DEFLATING -> REST (event 4) ----------
    safe_set_states(bci, StimulusPhase=4)
    sent = time.perf_counter()
    rec.mark_experimental("REST", t_rest, sent, clock.source_time())
    log("DESCANSO", 4, sent)
    if "deflated" not in rec.physical:
        rec.notes.append("cuff not yet below 3 mmHg at DEFLATING end (valve stays open)")
        print("    ⚠  El manguito aún no bajaba de 3 mmHg al terminar DEFLATING "
              "(la válvula sigue abierta).")

    cross.set_bar_fraction(0.0)
    return True


def run_trial(bci, cross, pump, clock, timing, trial_number, condition, pressure, block_number):
    print(f"\n  Trial {trial_number} | Run {block_number} | {condition} | {pressure} mmHg")

    events = []
    rec = TrialRecord(trial_number, condition, pressure)

    # The previous deflation normally ended long ago; this only waits if not.
    waited = wait_for_cuff_empty(pump, cross)
    if waited is None:
        return None

    # Initial trial states in one call to keep them aligned.
    safe_set_states(bci, TrialNumber=trial_number, PainLevel=PAIN_MAP[condition], PressureTarget=0)
    cross.pump(0.15)  # short pause so labels don't overlap in the Viewer

    cross.hide_rest_label()
    cross.hide_pressure_bar()
    cross.show_fixation_cross()
    cross.update()

    # Drop protocol lines that belong to earlier trials (already printed/logged).
    for rx_time, line in pump.take_events():
        rec.stale_lines.append(line)
    pump.trial_telemetry = rec.telemetry

    # Event 0 — Fixation cross. Its onset is locked to the BCI2000 block
    # clock; every later boundary of this trial is an exact multiple of the
    # block period from here.
    t_fix, st = clock.align()
    d_fix, _ = clock.duration(CROSS_TIME)
    rec.t0 = t_fix + d_fix          # t0 = scheduled INFLATING onset
    pump.telemetry_context = {"trial": trial_number, "condition": condition, "t0": rec.t0}
    safe_set_states(bci, StimulusPhase=0)
    sent = time.perf_counter()
    rec.mark_experimental("FIXATION", t_fix, sent, st)
    events.append({"phase": "FIXATION", "vector": 0,
                   "timestamp": datetime.now().strftime("%H:%M:%S.%f")[:-3],
                   "t_rel_s": rec.rel(sent)})
    print("    FIXATION (cruz)")
    if wait_until(rec.t0, pump, cross, rec) is None:
        pump.trial_telemetry = None
        return None

    trial_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    target_amplitude = pressure_to_amplitude(pressure)

    # All three conditions use the real pump (NONE target is 8 mmHg).
    if pressure <= 0:
        raise PumpError(
            f"Presión inválida en el trial {trial_number} ({condition}): "
            f"{pressure} mmHg. Las tres condiciones tienen que ser > 0."
        )

    # Events 1-3 — INFLATING / HOLDING / DEFLATING (fixed durations), then 4
    try:
        result = run_fixed_window_phases(pump, cross, bci, clock, timing, events, rec, rec.t0)
    except PumpError:
        pump.send_stop()
        pump.trial_telemetry = None
        raise
    if result is None:
        pump.send_stop()
        pump.trial_telemetry = None
        return None

    # Event 4 — Rest (10 s). The state was already set at the DEFLATING deadline.
    print("    DESCANSO")
    rest_ok = cross.countdown_rest(ITI_TIME, label="DESCANSO")

    # Collect anything that arrived during the rest (typically nothing; a
    # late DEFLATED would land here and still be recorded).
    for rx_time, line in pump.take_events():
        try:
            rec.handle_line(pump, rx_time, line)
        except PumpFaultError:
            raise
    pump.trial_telemetry = None
    pump.telemetry_context = None

    if not rest_ok:
        return None

    summary = rec.summary()
    summary["cuff_empty_wait_s"] = round(waited, 3)
    summary["ramp_ms"] = rec.ramp_ms
    _print_trial_summary(summary)

    return {
        "trial":     trial_number,
        "timestamp": trial_timestamp,
        "condition": condition,
        "pressure":  pressure,
        "signal_amplitude_uv": target_amplitude,
        "events":    events,
        "timing":    summary,
    }


def _print_trial_summary(s):
    ex, ph = s["experimental_s"], s["physical_s"]
    def g(d, k, f="{:.3f}"):
        v = d.get(k)
        if isinstance(v, dict):
            v = v.get("sent_s")
        return "—" if v is None else f.format(v)
    print(f"    tiempos (s desde INFLATING): exp HOLD {g(ex,'HOLDING')}  DEFL {g(ex,'DEFLATING')}  "
          f"REST {g(ex,'REST')} | fís. bomba {g(ph,'pump_on')}  objetivo {g(ph,'target_reached')} "
          f"({s['physical_pressure_mmhg'].get('target_reached', '—')} mmHg)  vacío {g(ph,'deflated')}")
    if "maintained_min_mmhg" in s:
        flag = "" if s["maintained_within_5mmhg"] else "   ⚠ FUERA de ±5 mmHg"
        print(f"    presión mantenida tras el objetivo: {s['maintained_min_mmhg']:.1f} a "
              f"{s['maintained_max_mmhg']:.1f} mmHg (objetivo {s['target_mmhg']}), "
              f"{s['topups']} re-inflados{flag}")


# =============================================================================
# Calibration (trial 0, HIGH pressure)
#
# Kept event driven like the original: INFLATING lasts until the cuff really
# reaches HIGH, HOLDING 10 s, DEFLATING until the cuff is really empty. That
# is precisely what it has to measure. The measurements then define the fixed
# INFLATING/DEFLATING durations of every trial of the run.
# =============================================================================

def run_calibration_phases(pump, cross, bci, clock, events, rec):
    pressure = rec.pressure
    failsafe_ms = int(round((CALIBRATION_MAX_INFLATE_S + HOLD_SECONDS + FAILSAFE_GRACE_S) * 1000))

    def log(phase, vector, t):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        events.append({"phase": phase, "vector": vector, "timestamp": ts,
                       "t_rel_s": rec.rel(t)})

    # ---------- INFLATING (until physical target) ----------
    rec.t0, st = clock.align()
    sent = pump.send_trial(rec.trial_id, pressure, 0, failsafe_ms)
    safe_set_states(bci, StimulusPhase=1, PressureTarget=pressure)
    rec.mark_experimental("INFLATING", rec.t0, sent, st)
    log("INFLATING", 1, sent)
    cross.hide_fixation_cross()
    cross.show_pressure_bar()
    cross.set_bar_label("INFLATING")
    set_signal_amplitude(bci, pressure_to_amplitude(pressure))

    check = _ack_checker(pump, rec,
                         lambda: pump.send_trial(rec.trial_id, pressure, 0, failsafe_ms))
    far = rec.t0 + CALIBRATION_MAX_INFLATE_S + 5.0   # the Arduino faults first
    got = wait_until(far, pump, cross, rec, bar_target=pressure, label="INFLATING",
                     check=check, stop_on=lambda: "target_reached" in rec.physical)
    if got is None:
        return None
    if got is True:
        raise PumpTimeoutError("La calibración no alcanzó la presión HIGH.")

    # ---------- HOLDING (HOLD_SECONDS from the state onset) ----------
    t_hold, st = clock.align()
    safe_set_states(bci, StimulusPhase=2)
    sent = time.perf_counter()
    rec.mark_experimental("HOLDING", t_hold, sent, st)
    log("HOLDING", 2, sent)
    cross.set_bar_label("HOLDING")
    d_hold, _ = clock.duration(HOLD_SECONDS)
    t_def = t_hold + d_hold
    if wait_until(t_def, pump, cross, rec, bar_target=pressure, label="HOLDING") is None:
        return None

    # ---------- DEFLATING (until physically empty) ----------
    sent = pump.send_deflate(rec.trial_id)
    safe_set_states(bci, StimulusPhase=3, PressureTarget=0)
    rec.mark_experimental("DEFLATING", t_def, sent, clock.source_time())
    log("DEFLATING", 3, sent)
    cross.set_bar_label("DEFLATING")
    set_signal_amplitude(bci, AMPLITUDE_BASE_UV)

    far = t_def + 3600.0   # like the original: no timeout, the valve is open
    got = wait_until(far, pump, cross, rec, bar_target=pressure, label="DEFLATING",
                     stop_on=lambda: "deflated" in rec.physical)
    if got is None:
        return None

    cross.set_bar_fraction(0.0)
    return True


def compute_standard_timing(rec, clock, none, low, high):
    """Turns the calibration measurements into the fixed event durations used
    by every trial of the run. Raises PumpError if they are not plausible."""
    ph = rec.physical
    if "pump_on" not in ph or "target_reached" not in ph:
        raise PumpError("La calibración no registró PUMP_ON/TARGET_REACHED.")
    if "deflate_valve_open" not in ph or "deflated" not in ph:
        raise PumpError("La calibración no registró DEFLATING/DEFLATED.")

    # Physical durations on the PC clock (ms resolution, then rounded to 1 ms
    # before ceil() so float noise like 7.0000001 cannot add a whole second).
    infl = round(ph["target_reached"] - ph["pump_on"], 3)
    defl = round(ph["deflated"] - ph["deflate_valve_open"], 3)

    inflating = max(MIN_EVENT_SECONDS, int(math.ceil(round(infl + INFLATE_MARGIN_S, 3))))
    deflating = max(MIN_EVENT_SECONDS, int(math.ceil(round(defl + DEFLATE_MARGIN_S, 3))))

    for name, v in (("INFLATING", inflating), ("DEFLATING", deflating)):
        if v > MAX_CALIBRATED_EVENT_SECONDS:
            raise PumpError(
                f"La calibración dio {name} = {v} s (> {MAX_CALIBRATED_EVENT_SECONDS} s). "
                "Probable fuga o manguera floja; revisa el hardware."
            )

    timing = {
        "HIGH_TARGET_PRESSURE":          high,
        "LOW_TARGET_PRESSURE":           low,
        "NONE_TARGET_PRESSURE":          none,
        "MEASURED_HIGH_INFLATION_TIME":  infl,
        "MEASURED_HIGH_DEFLATION_TIME":  defl,
        "MEASURED_HIGH_INFLATION_TIME_ARDUINO_MS": rec.physical_arduino_ms.get("target_reached"),
        "MEASURED_HIGH_DEFLATION_TIME_ARDUINO_MS": (
            None if "deflated" not in rec.physical_arduino_ms else
            rec.physical_arduino_ms["deflated"] - rec.physical_arduino_ms["deflate_valve_open"]),
        # What THIS run's calibration alone would have chosen (kept even when
        # a lock overrides INFLATING_DURATION/DEFLATING_DURATION below).
        "CALIBRATED_INFLATING_DURATION": inflating,
        "CALIBRATED_DEFLATING_DURATION": deflating,
        "none_uses_ramp":                RAMP_NONE,
        "arduino_clock_ppm":             None,   # filled in by run_calibration
        "timing_locked":                 False,
        "timing_reference_run":          None,
    }
    set_event_durations(timing, clock, inflating, deflating)
    timing.update(clock.info())
    return timing


def set_event_durations(timing, clock, inflating, deflating):
    """Writes the fixed INFLATING/HOLDING/DEFLATING durations (and everything
    derived from them: block counts, samples, LOW ramp) into `timing`.
    Used both for a fresh calibration and for a locked timing."""
    d_inf,  n_inf  = clock.duration(inflating)
    d_hold, n_hold = clock.duration(HOLD_SECONDS)
    d_def,  n_def  = clock.duration(deflating)

    timing.update({
        "INFLATING_DURATION":    inflating,
        "DEFLATING_DURATION":    deflating,
        "HOLDING_DURATION":      HOLD_SECONDS,
        "inflating_scheduled_s": d_inf,
        "holding_scheduled_s":   d_hold,
        "deflating_scheduled_s": d_def,
        "inflating_blocks":      n_inf,
        "holding_blocks":        n_hold,
        "deflating_blocks":      n_def,
        "inflating_samples":     None if n_inf  is None else int(n_inf  * clock.block_size),
        "holding_samples":       None if n_hold is None else int(n_hold * clock.block_size),
        "deflating_samples":     None if n_def  is None else int(n_def  * clock.block_size),
        "low_ramp_ms":           int(round(max(0.5, d_inf - RAMP_END_MARGIN_S) * 1000)),
    })


# =============================================================================
# Timing lock: storage, lookup and application
# =============================================================================

def _participant_json_path(participant_data):
    return f"participant_{str(participant_data['participant_number']).zfill(3)}.json"


def _lock_from_timing(timing, run_number, source):
    """The part of a run's timing that later runs of the session must reuse."""
    return {
        "INFLATING_DURATION":           int(timing["INFLATING_DURATION"]),
        "DEFLATING_DURATION":           int(timing["DEFLATING_DURATION"]),
        "HOLDING_DURATION":             timing.get("HOLDING_DURATION", HOLD_SECONDS),
        "reference_run":                run_number,
        "source":                       source,
        "MEASURED_HIGH_INFLATION_TIME": timing.get("MEASURED_HIGH_INFLATION_TIME"),
        "MEASURED_HIGH_DEFLATION_TIME": timing.get("MEASURED_HIGH_DEFLATION_TIME"),
        "HIGH_TARGET_PRESSURE":         timing.get("HIGH_TARGET_PRESSURE"),
        "LOW_TARGET_PRESSURE":          timing.get("LOW_TARGET_PRESSURE"),
        "NONE_TARGET_PRESSURE":         timing.get("NONE_TARGET_PRESSURE"),
        "locked_at":                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }


def _lock_from_earlier_results(participant_data):
    """Looks for a calibration in the results JSON of earlier runs of the same
    participant and session. Returns a lock dict or None."""
    p_num   = str(participant_data["participant_number"]).zfill(3)
    session = int(participant_data["session_number"])
    current = int(participant_data["run_number"])

    pattern = re.compile(rf"^participant_{p_num}_run(\d+)_results\.json$")
    candidates = []
    for filename in os.listdir("."):
        m = pattern.match(filename)
        if not m:
            continue
        run = int(m.group(1))
        if run >= current:
            continue
        try:
            with open(filename, "r") as f:
                res = json.load(f)
        except (OSError, ValueError):
            continue
        # Results filenames do not include the session: check it inside.
        if int(res.get("session_number", -1)) != session:
            continue
        cal = res.get("calibration") or {}
        if "INFLATING_DURATION" not in cal or "DEFLATING_DURATION" not in cal:
            continue
        candidates.append((run, cal))

    if TIMING_REFERENCE_RUN is not None:
        for run, cal in candidates:
            if run == TIMING_REFERENCE_RUN:
                return _lock_from_timing(cal, run, f"participant_{p_num}_run{str(run).zfill(2)}_results.json")
        raise PumpError(
            f"TIMING_REFERENCE_RUN = {TIMING_REFERENCE_RUN}, pero no encuentro una "
            f"calibración de ese run (sesión {session}) en "
            f"participant_{p_num}_run{str(TIMING_REFERENCE_RUN).zfill(2)}_results.json."
        )

    if not candidates:
        return None
    run, cal = min(candidates, key=lambda c: c[0])
    return _lock_from_timing(cal, run, f"participant_{p_num}_run{str(run).zfill(2)}_results.json")


def load_timing_lock(participant_data):
    """Returns the locked timing of this participant+session, or None if this
    run has to create it. A lock found in earlier results files is saved to
    the participant JSON right away so it never changes afterwards."""
    if not LOCK_TIMING_ACROSS_RUNS:
        return None

    path = _participant_json_path(participant_data)
    key  = str(int(participant_data["session_number"]))
    try:
        with open(path, "r") as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        cfg = {}

    lock = (cfg.get("timing_lock") or {}).get(key)
    if lock and "INFLATING_DURATION" in lock and "DEFLATING_DURATION" in lock:
        return lock

    lock = _lock_from_earlier_results(participant_data)
    if lock is not None:
        save_timing_lock(participant_data, lock)
    return lock


def save_timing_lock(participant_data, lock):
    """Stores the lock under participant_XXX.json -> timing_lock -> <session>.
    Other keys of the file (pressures, sessions list) are kept."""
    path = _participant_json_path(participant_data)
    key  = str(int(participant_data["session_number"]))
    try:
        with open(path, "r") as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        cfg = {}
    cfg.setdefault("timing_lock", {})[key] = lock
    with open(path, "w") as f:
        json.dump(cfg, f, indent=4)
    print(f"    ✔  Tiempos fijados para la sesión {key} en {path} "
          f"(INFLATING {lock['INFLATING_DURATION']} s, DEFLATING {lock['DEFLATING_DURATION']} s, "
          f"referencia R{str(lock['reference_run']).zfill(2)})")


def apply_timing_lock(timing, lock, clock):
    """Replaces this run's calibrated durations with the locked ones.
    Returns a list of warnings (empty = the locked windows still fit)."""
    inflating = int(lock["INFLATING_DURATION"])
    deflating = int(lock["DEFLATING_DURATION"])
    set_event_durations(timing, clock, inflating, deflating)
    timing["timing_locked"] = True
    timing["timing_reference_run"] = lock.get("reference_run")
    timing["timing_lock"] = lock

    warnings = []
    infl = timing["MEASURED_HIGH_INFLATION_TIME"]
    defl = timing["MEASURED_HIGH_DEFLATION_TIME"]
    usable_inf = timing["inflating_scheduled_s"]
    if infl > usable_inf:
        warnings.append(
            f"HIGH tardó {infl:.2f} s en inflarse en esta calibración, más que el "
            f"INFLATING fijado ({inflating} s). En los trials HIGH la presión se "
            f"alcanzará ya dentro de HOLDING. Reacomoda el manguito (más ajustado, "
            f"como en R{str(lock.get('reference_run')).zfill(2)}) antes de seguir."
        )
    if defl > timing["deflating_scheduled_s"]:
        warnings.append(
            f"El desinflado tardó {defl:.2f} s, más que el DEFLATING fijado "
            f"({deflating} s). El manguito terminará de vaciarse durante el descanso."
        )
    if timing.get("HIGH_TARGET_PRESSURE") != lock.get("HIGH_TARGET_PRESSURE") \
            and lock.get("HIGH_TARGET_PRESSURE") is not None:
        warnings.append(
            f"La presión HIGH cambió ({lock.get('HIGH_TARGET_PRESSURE')} -> "
            f"{timing.get('HIGH_TARGET_PRESSURE')} mmHg) desde el run de referencia."
        )
    timing["timing_lock_warnings"] = warnings
    return warnings


def run_calibration_priming(pump, cross, prime_mmhg):
    """Inflates once to prime_mmhg (LOW), vents, waits PRIME_SETTLE_S.
    Not measured and not an experimental event (StimulusPhase stays 0).
    Returns a dict for the results JSON, or None if the window was closed."""
    print(f"    PRIMING   (asentar el manguito: {prime_mmhg} mmHg, no se mide)")
    rec = TrialRecord(PRIME_TRIAL_ID, "PRIMING", prime_mmhg)
    for rx_time, line in pump.take_events():
        rec.stale_lines.append(line)
    rec.t0 = time.perf_counter()
    failsafe_ms = int(round((CALIBRATION_MAX_INFLATE_S + FAILSAFE_GRACE_S) * 1000))

    try:
        pump.send_trial(PRIME_TRIAL_ID, prime_mmhg, 0, failsafe_ms)
        check = _ack_checker(pump, rec,
                             lambda: pump.send_trial(PRIME_TRIAL_ID, prime_mmhg, 0, failsafe_ms))
        got = wait_until(rec.t0 + CALIBRATION_MAX_INFLATE_S + 5.0, pump, cross, rec,
                         check=check, stop_on=lambda: "target_reached" in rec.physical)
        if got is None:
            pump.send_stop()
            return None
        if got is True:
            raise PumpTimeoutError(f"El priming no alcanzó {prime_mmhg} mmHg.")

        pump.send_deflate(PRIME_TRIAL_ID)
        got = wait_until(time.perf_counter() + 60.0, pump, cross, rec,
                         stop_on=lambda: "deflated" in rec.physical)
        if got is None:
            return None
        if got is True:
            raise PumpTimeoutError("El manguito no se vació después del priming.")
    except PumpError:
        pump.send_stop()
        raise

    if not cross.pump(PRIME_SETTLE_S):
        return None

    info = {
        "target_mmhg": prime_mmhg,
        "time_to_target_s": (None if "pump_on" not in rec.physical else
                             round(rec.physical["target_reached"] - rec.physical["pump_on"], 3)),
        "deflation_s": (None if "deflate_valve_open" not in rec.physical else
                        round(rec.physical["deflated"] - rec.physical["deflate_valve_open"], 3)),
        "settle_s": PRIME_SETTLE_S,
    }
    print(f"    Priming listo: {prime_mmhg} mmHg en {info['time_to_target_s']} s "
          f"(esto NO define la duración de INFLATING)")
    return info


def run_calibration(bci, cross, pump, clock, pressure, none=None, low=None, lock=None):
    """Trial 0 — Calibration at HIGH pressure, with a red "CALIBRACIÓN"
    banner during all stages. No long rest at the end; returns to idle
    (StimulusPhase=0) before Trial 1.

    lock: locked timing of this participant+session (see TIMING LOCK). When
    given, the trials use its durations and this calibration only checks
    that they still fit.

    Returns (calibration_log, timing) or (None, None) if the window was closed.
    """
    print(f"\n{'='*50}")
    print(f"  CALIBRACIÓN (Trial 0) — presión: {pressure} mmHg")
    print(f"{'='*50}")

    events = []
    rec = TrialRecord(0, "CALIBRATION", pressure)

    def log_event(phase, vector):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        events.append({"phase": phase, "vector": vector, "timestamp": ts})
        safe_set_state(bci, "StimulusPhase", vector)

    safe_set_states(bci, TrialNumber=0, PainLevel=PAIN_MAP[HIGH_LABEL], PressureTarget=0)

    cross.show_calibration_banner("CALIBRACIÓN")

    waited = wait_for_cuff_empty(pump, cross)
    if waited is None:
        cross.hide_calibration_banner()
        return None, None

    # Priming: seat the cuff so the measured inflation is like a trial's.
    prime_info = None
    if CALIBRATION_PRIMING:
        prime_info = run_calibration_priming(pump, cross, low if low else pressure)
        if prime_info is None:
            cross.hide_calibration_banner()
            return None, None

    # Idle stage — fixation cross (3 s)
    cross.hide_rest_label()
    cross.hide_pressure_bar()
    cross.show_fixation_cross()
    print("    IDLE (cruz)")
    log_event("IDLE", 0)
    if not cross.pump(CROSS_TIME):
        cross.hide_calibration_banner()
        return None, None

    trial_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    target_amplitude = pressure_to_amplitude(pressure)

    for rx_time, line in pump.take_events():
        rec.stale_lines.append(line)
    pump.trial_telemetry = rec.telemetry
    pump.telemetry_context = {"trial": 0, "condition": "CALIBRATION", "t0": None}

    # INFLATING / HOLDING / DEFLATING with the real pump (event driven)
    print(f"    INFLATING  (bomba REAL -> {pressure} mmHg, se mide el tiempo)")
    try:
        result = run_calibration_phases(pump, cross, bci, clock, events, rec)
    except PumpError:
        pump.send_stop()
        pump.trial_telemetry = None
        cross.hide_calibration_banner()
        raise
    pump.trial_telemetry = None
    pump.telemetry_context = None
    if result is None:
        pump.send_stop()
        cross.hide_calibration_banner()
        return None, None

    # Back to idle, no long rest
    cross.hide_pressure_bar()
    log_event("IDLE", 0)
    cross.hide_calibration_banner()

    timing = compute_standard_timing(rec, clock, none, low, pressure)
    timing["arduino_clock_ppm"] = pump.arduino_clock_ppm()
    timing["priming"] = prime_info

    print("    Calibración completa.")
    print(f"    HIGH: inflado físico {timing['MEASURED_HIGH_INFLATION_TIME']:.3f} s "
          f"-> este run solo daría INFLATING = {timing['CALIBRATED_INFLATING_DURATION']} s")
    print(f"          desinflado físico {timing['MEASURED_HIGH_DEFLATION_TIME']:.3f} s "
          f"-> este run solo daría DEFLATING = {timing['CALIBRATED_DEFLATING_DURATION']} s")

    if lock is not None:
        warnings = apply_timing_lock(timing, lock, clock)
        print(f"    TIEMPOS FIJADOS por R{str(lock.get('reference_run')).zfill(2)} "
              f"(misma sesión): INFLATING = {timing['INFLATING_DURATION']} s, "
              f"DEFLATING = {timing['DEFLATING_DURATION']} s")
        for w in warnings:
            print(f"    ⚠  {w}")
    else:
        print(f"    Este run fija los tiempos de la sesión: "
              f"INFLATING = {timing['INFLATING_DURATION']} s, "
              f"DEFLATING = {timing['DEFLATING_DURATION']} s")
    if timing["inflating_blocks"] is not None:
        print(f"    En la señal: INFLATING {timing['inflating_samples']} muestras "
              f"({timing['inflating_blocks']} bloques), HOLDING {timing['holding_samples']}, "
              f"DEFLATING {timing['deflating_samples']}")
        for name, sec in (("INFLATING", timing["INFLATING_DURATION"]),
                          ("HOLDING", HOLD_SECONDS),
                          ("DEFLATING", timing["DEFLATING_DURATION"])):
            if not clock.is_whole_blocks(sec):
                print(f"    ⚠  {name} = {sec} s no es un número entero de bloques; "
                      f"se usa {clock.duration(sec)[0]:.3f} s (igual en todos los trials).")
    print()

    summary = rec.summary()
    return {
        "trial":     0,
        "timestamp": trial_timestamp,
        "condition": "CALIBRATION",
        "pressure":  pressure,
        "signal_amplitude_uv": target_amplitude,
        "events":    events,
        "timing":    summary,
    }, timing


def rest_between_blocks(bci, cross, block_number, total_blocks):
    print(f"\n{'='*50}")
    print(f"  END OF BLOCK {block_number}/{total_blocks}")
    print(f"  REST — {BLOCK_REST} seconds")
    print(f"{'='*50}")
    safe_set_state(bci, "StimulusPhase", 0)
    set_signal_amplitude(bci, AMPLITUDE_BASE_UV)
    if not cross.countdown_rest(BLOCK_REST, label="DESCANSO ENTRE BLOQUES"):
        raise BCI2000DisconnectedError("Ventana cerrada por el usuario durante el descanso entre bloques.")
    print("  Resuming...\n")


def get_telemetry_path(participant_data):
    p_num = str(participant_data["participant_number"]).zfill(3)
    run   = str(participant_data["run_number"]).zfill(2)
    return f"participant_{p_num}_run{run}_telemetry.csv"


class TelemetryCSV:
    """Every Arduino telemetry line of the run (20 Hz), one CSV row each:
    pressure, target, ramp reference and PWM, with the PC arrival time, the
    Arduino time, and the Arduino time mapped to the PC clock. t_rel_s is
    relative to the current trial's scheduled INFLATING onset (same origin as
    the 'timing' block of each trial in the results JSON)."""

    FIELDS = ["trial", "condition", "t_rel_s", "pc_rx_s", "pc_est_s", "arduino_ms",
              "pressure_mmhg", "target_mmhg", "desired_mmhg", "pwm"]

    def __init__(self, path, pump):
        self.path = path
        self.pump = pump
        self.t_origin = time.perf_counter()
        new = not os.path.exists(path)
        self.f = open(path, "a", newline="")
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(self.FIELDS)
        self.n = 0

    def __call__(self, row):
        ctx = self.pump.telemetry_context or {}
        t0 = ctx.get("t0")
        pc_est = None
        if row["arduino_ms"] is not None:
            pc_est = self.pump.arduino_ms_to_pc(row["arduino_ms"])
        ref = pc_est if pc_est is not None else row["pc_rx"]
        self.w.writerow([
            ctx.get("trial", ""), ctx.get("condition", ""),
            "" if t0 is None else f"{ref - t0:.4f}",
            f"{row['pc_rx'] - self.t_origin:.4f}",
            "" if pc_est is None else f"{pc_est - self.t_origin:.4f}",
            "" if row["arduino_ms"] is None else row["arduino_ms"],
            f"{row['pressure']:.1f}", f"{row['target']:.1f}",
            f"{row['desired']:.1f}", row["pwm"],
        ])
        self.n += 1
        if self.n % 200 == 0:
            self.f.flush()

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass


def main():
    participant_data = launch_interface()
    if participant_data is None:
        print("Experiment cancelled.")
        return

    print(f"\nStarting: {participant_data['name']} | Run {participant_data['run_number']}")

    p_num     = str(participant_data["participant_number"]).zfill(3)
    json_path = f"participant_{p_num}.json"

    with open(json_path, "r") as f:
        cfg = json.load(f)

    none = cfg.get("none_pressure", NONE_PRESSURE)   ## control: pump still runs
    low  = cfg.get("low_pressure",  46) ## Change the low
    high = cfg.get("high_pressure", 111) ## Change the high

    # Python-side range check (first line of defense; the .ino has its own limit).
    for label, value in (("none_pressure", none), ("low_pressure", low), ("high_pressure", high)):
        if value <= 0 or value > MAX_SAFE_MMHG:
            print(f"\n⚠  {label} = {value} mmHg está fuera de rango "
                  f"(1 a {MAX_SAFE_MMHG:.0f}). Revisa {json_path}. Abortando.")
            return

    if not (none < low < high):
        print(f"\n⚠  Las presiones tienen que ir en orden: "
              f"none ({none}) < low ({low}) < high ({high}). Abortando.")
        return

    print(f"Pressures — NONE: {none} mmHg | LOW: {low} mmHg | HIGH: {high} mmHg")
    print("Las tres condiciones inflan el manguito; solo cambia cuánto.")
    print("La barra llega al 100% en el objetivo de cada trial, y NO muestra "
          "números al participante (los mmHg solo salen aquí en la consola).\n")

    results = load_or_create_results(participant_data)

    # Timing lock of this participant+session, read BEFORE anything is
    # recorded so a bad TIMING_REFERENCE_RUN aborts without creating a .dat.
    try:
        timing_lock = load_timing_lock(participant_data)
    except PumpError as e:
        print(f"\n⚠  {e}\n   Abortando antes de grabar nada.")
        return
    if timing_lock is not None:
        print(f"Tiempos FIJADOS para la sesión {participant_data['session_number']} "
              f"(referencia R{str(timing_lock.get('reference_run')).zfill(2)}): "
              f"INFLATING {timing_lock['INFLATING_DURATION']} s, HOLDING {HOLD_SECONDS:g} s, "
              f"DEFLATING {timing_lock['DEFLATING_DURATION']} s.")
        print("La calibración de este run solo comprueba que sigan alcanzando.\n")
    elif LOCK_TIMING_ACROSS_RUNS:
        print(f"Sesión {participant_data['session_number']} sin tiempos fijados: la "
              "calibración de este run los fijará para los siguientes runs.\n")

    bci, dat_path = setup_bci(participant_data)
    if bci is None:
        print("No se pudo conectar/configurar BCI2000. Abortando.")
        return

    # --- Open the pump (Arduino) before starting the protocol ---
    try:
        pump = ArduinoPump(ARDUINO_PORT, ARDUINO_BAUD)
    except PumpError as e:
        print(f"\n⚠  No se pudo abrir el Arduino de la bomba: {e}")
        print("   Abortando (revisa el cable/puerto y ARDUINO_PORT).")
        try:
            bci.Stop()
        except Exception:
            pass
        return

    # bci.Start() is called only after ENTER: BCI2000 creates the .dat on
    # Start() and never overwrites, so aborting earlier would leave a short
    # file and push the next attempt to R02.

    cross = FixationCross()

    # CRITICAL: every wait tick drains the serial port. Without this the
    # buffer overflows and a later trial can hang waiting for a lost "INFLATING".
    cross.serial_drain = pump.drain

    print(f"\n{'='*50}")
    print("  Todo listo. Presiona ENTER en la ventana de la cruz para iniciar...")
    print("  (o Escape para cancelar el experimento)")
    print("  Nada se graba en disco hasta que presiones ENTER.")
    print(f"{'='*50}")
    cross.wait_for_start()

    if not cross._running:
        print("\n  Ventana cerrada antes de iniciar. Experimento cancelado.")
        print("  No se creó ningún .dat — el run sigue libre.")
        try:
            pump.close()
        except Exception:
            pass
        try:
            cross.close()
        except Exception:
            pass
        return

    print("  ¡Señal recibida! Iniciando protocolo...\n")

    # From here on the .dat file exists and is recording.
    bci.Start()
    time.sleep(2)
    set_signal_amplitude(bci, AMPLITUDE_BASE_UV)

    # Needs a running system: it watches SourceTime advance block by block.
    clock = EventClock(bci)
    print(f"  BCI2000: {clock.describe()}")
    if not clock.block_locked:
        print("  ⚠  Sin candado de bloque: cada cambio de estado puede caer en uno de")
        print("     dos bloques contiguos (±1 bloque en el .dat).")
    results["sync"] = clock.info()

    telemetry = TelemetryCSV(get_telemetry_path(participant_data), pump)
    pump.telemetry_sink = telemetry
    results["sync"]["telemetry_origin_perf_counter"] = telemetry.t_origin

    # All 4 states start at 0 (AddStateVariable default), so the recording
    # begins at a clean baseline. The Viewer only labels changes, not the initial 0.

    bci_disconnected = False
    try:
        # Each execution is a full run (R01, R02...); the number only names files.
        run_num = participant_data["run_number"]

        # Trial 0 — Calibration runs in every run (cuff refitted, sensor re-zeroed).
        # It also measures the HIGH inflation/deflation times that fix the
        # INFLATING/DEFLATING durations of every trial below.
        calibration_log, timing = run_calibration(bci, cross, pump, clock, high, none, low,
                                                  lock=timing_lock)
        if calibration_log is None:
            print("\n  Experimento interrumpido durante la calibración (ventana cerrada).")
            raise BCI2000DisconnectedError("Ventana de la cruz de fijación cerrada durante la calibración.")
        append_trial_to_results(participant_data, results, block_number=0, trial_data=calibration_log)
        results["calibration"] = timing
        save_results(participant_data, results)

        # First run of the session: its durations become the lock for the rest.
        if LOCK_TIMING_ACROSS_RUNS and timing_lock is None:
            save_timing_lock(participant_data, _lock_from_timing(
                timing, participant_data["run_number"],
                f"calibración de R{str(participant_data['run_number']).zfill(2)}"))

        lock_warnings = timing.get("timing_lock_warnings") or []

        # Wait for ENTER after calibration to allow checking the signal.
        print(f"\n{'='*50}")
        print("  Calibración terminada. Presiona ENTER en la ventana de la cruz")
        print("  para iniciar los trials...")
        print(f"  Cada trial: INFLATING {timing['INFLATING_DURATION']} s -> "
              f"HOLDING {HOLD_SECONDS:g} s -> DEFLATING {timing['DEFLATING_DURATION']} s "
              f"(iguales en HIGH, LOW y NONE"
              + (f", y en todos los runs de la sesión desde R{str(timing['timing_reference_run']).zfill(2)})"
                 if timing.get("timing_locked") else ")"))
        if lock_warnings:
            print()
            print("  ⚠  ATENCIÓN: los tiempos fijados quizá no alcancen en este run:")
            for w in lock_warnings:
                print(f"     - {w}")
            print("  ENTER = seguir con los tiempos fijados, Escape = detener aquí.")
        print(f"{'='*50}")
        start_msg = "Calibración completa. Presione ENTER para iniciar"
        if lock_warnings:
            start_msg = ("⚠ Los tiempos fijados quizá no alcancen (ver consola). "
                         "ENTER = seguir, Escape = detener")
        cross.wait_for_start(message=start_msg)

        if not cross._running:
            print("\n  Ventana cerrada después de la calibración. Experimento interrumpido.")
            raise BCI2000DisconnectedError("Ventana de la cruz de fijación cerrada tras la calibración.")

        print(f"\n{'='*50}")
        print(f"  RUN {run_num} — {TRIALS_TOTAL} trials")
        print(f"{'='*50}")

        block_trials = create_block(none, low, high)

        # Trials are numbered 1..30 in every run so each .dat is self-contained.
        global_trial = 1

        for trial_idx, (condition, pressure) in enumerate(block_trials, start=1):
            trial_log = run_trial(bci, cross, pump, clock, timing,
                                  global_trial, condition, pressure, run_num)

            if trial_log is None:
                print("\n  Experimento interrumpido (ventana cerrada por el usuario).")
                raise BCI2000DisconnectedError("Ventana de la cruz de fijación cerrada por el usuario.")

            append_trial_to_results(participant_data, results, run_num, trial_log)

            global_trial += 1

        print(f"\n  Run {run_num} terminado: {TRIALS_TOTAL} trials + calibración.")
        print(f"  Si quieres otro run con este participante, deja descansar y")
        print(f"  vuelve a correr el programa — se grabará solo como "
              f"R{str(run_num + 1).zfill(2)}.")

    except BCI2000DisconnectedError as e:
        bci_disconnected = True
        print(f"\n⚠  {e}")
        print("   El experimento se detuvo. Los trials ya completados quedaron guardados.")

    except PumpTimeoutError as e:
        bci_disconnected = True
        print(f"\n⚠  Watchdog de la bomba: {e}")
        print("   Se envió STOP al Arduino y se detuvo el experimento por seguridad.")

    except PumpFaultError as e:
        bci_disconnected = True
        print(f"\n⚠  Falla reportada por el Arduino: {e}")
        print("   El manguito ya se venteó. Revisa el hardware antes de volver a correr.")

    except PumpError as e:
        bci_disconnected = True
        print(f"\n⚠  Problema con el Arduino de la bomba: {e}")
        print("   Se detuvo el experimento. Los trials ya completados quedaron guardados.")

    except Exception as e:
        bci_disconnected = True
        print(f"\n⚠  Ocurrió un error inesperado y se detuvo el experimento ({type(e).__name__}).")
        print("   Los resultados de los trials ya completados se guardaron correctamente.")

    finally:
        # Always STOP and close the port, even on crash: the pump must never stay on.
        try:
            pump.close()
        except Exception:
            pass

        try:
            telemetry.close()
        except Exception:
            pass

        try:
            bci.Stop()
        except Exception:
            pass

        try:
            cross.close()
        except Exception:
            pass

        results["end_timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if bci_disconnected:
            results["ended_early_disconnected"] = True
        try:
            results.setdefault("sync", {})["arduino_clock_ppm_end"] = pump.arduino_clock_ppm()
            results["sync"]["block_align_failures"] = clock.align_failures
        except Exception:
            pass
        save_results(participant_data, results)

        print("\n" + "="*50)
        print("  EXPERIMENT FINISHED")
        print(f"  Results (JSON) saved → {get_results_path(participant_data)}")
        print(f"  Pressure telemetry (CSV) saved → {get_telemetry_path(participant_data)}")
        print(f"  Brain recording (.dat) saved → {dat_path}")
        print("="*50)


if __name__ == "__main__":
    main()
