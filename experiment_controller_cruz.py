import json
import random
import time
import os
import tkinter as tk
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

# Deflation ends when the MPX5050DP reads below DEFLATE_DONE_MMHG (3 mmHg).
# A trial only stops on an Arduino FAULT (overpressure or pump exceeding MAX_INFLATE_MS).
PRESSURE_LOG_INTERVAL = 1.0

# Must match HOLD_DURATION in the .ino (10000 ms).
HOLD_SECONDS = 10.0

# Maximum pressure Python can request; guards against bad participant JSON values.
MAX_SAFE_MMHG = 200.0

# Arduino error messages; abort the trial if received.
FAULT_WORDS = ("STOPPED", "FAULT")

# Protocol phase vocabulary.
PHASE_WORDS = ("READY", "INFLATING", "HOLDING", "DEFLATING", "DONE")


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

    Protocol:
        -> START,<mmHg>  -> INFLATING -> HOLDING -> DEFLATING -> DONE
        -> STOP          -> STOPPED
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

        self._handshake()

        print(f"Arduino listo en {port}. Presión actual: {self.pressure_mmhg:.1f} mmHg")

    def _handshake(self):
        """Sends PING and waits for READY to verify the firmware."""
        deadline = time.time() + 5.0
        self.ser.write(b"PING\n")
        self.ser.flush()

        while time.time() < deadline:
            line = self.read_line()

            if line and line.startswith("ZERO,"):
                # Arduino just measured the sensor zero offset.
                print(f"  [arduino] cero del sensor: {line}")

            if line == "READY":
                return

            time.sleep(0.01)

        raise PumpError(
            f"El Arduino en {self.port} no respondió READY. "
            "Revisa que tenga cargado arduino_pressure_controller.ino "
            "y que el baud sea 115200."
        )

    def drain(self):
        """Drains the serial buffer so 20 Hz telemetry doesn't overflow it
        between trials, and keeps pressure_mmhg updated."""
        try:
            while self.ser.in_waiting:
                self.read_line()   # Updates pressure_mmhg, discards old phase messages
        except Exception:
            pass   # A failed drain should not interrupt the experiment

    def send_start(self, mmhg):
        if mmhg <= 0 or mmhg > MAX_SAFE_MMHG:
            raise PumpError(
                f"Presión objetivo fuera de rango: {mmhg} mmHg "
                f"(el máximo permitido es {MAX_SAFE_MMHG:.0f})."
            )

        target = int(round(mmhg))

        # Clear old data so the next phase message belongs to this trial.
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._buf = ""

        # Set target locally first so the UI knows it immediately.
        self.target_mmhg = float(target)

        self.ser.write(f"START,{target}\n".encode("ascii"))
        self.ser.flush()

    def send_stop(self):
        try:
            self.ser.write(b"STOP\n")
            self.ser.flush()
        except Exception:
            pass  # Never let an error block an emergency STOP

    def read_line(self):
        """Non-blocking. Returns one phase line (INFLATING/HOLDING/...) or None.
        Telemetry lines ("P,<actual>,<target>") update pressure_mmhg and are
        not returned."""
        try:
            chunk = self.ser.read(256)
        except Exception:
            raise PumpError("Se perdió la conexión serial con el Arduino.")

        if chunk:
            self._buf += chunk.decode("ascii", errors="ignore")

        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()

            if not line:
                continue

            # --- Pressure telemetry ---
            if line.startswith("P,"):
                parts = line.split(",")
                if len(parts) >= 3:
                    try:
                        self.pressure_mmhg = float(parts[1])
                        self.target_mmhg   = float(parts[2])
                    except ValueError:
                        pass
                continue   # not a phase, keep looking

            # ---- Phase word ----
            # Only protocol words are accepted; anything else is a truncated
            # line from a buffer overflow and is discarded.
            if line in PHASE_WORDS or line.startswith(FAULT_WORDS) or line.startswith("ZERO,"):
                return line

            continue

        return None

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


def wait_for_inflating_ack(pump, cross, pressure, resend_every=2.0):
    """Waits for "INFLATING" confirming the START was received.
    Resends START every `resend_every` seconds until acked (resending is harmless).

    Returns True on ack, None if the window was closed.
    """
    last_send = time.time()
    attempts  = 1

    while True:
        if not cross._running:
            return None

        line = pump.read_line()
        if line:
            print(f"    [arduino] {line}")

            if line == "INFLATING":
                return True

            if line.startswith(FAULT_WORDS):
                raise PumpFaultError(
                    f"El Arduino abortó al arrancar: '{line}'. Ya venteó el manguito."
                )

        if time.time() - last_send >= resend_every:
            attempts += 1
            print(f"    (sin ack del Arduino; reenviando START — intento {attempts})")
            pump.send_start(pressure)
            last_send = time.time()

        cross.update()
        time.sleep(0.005)


def wait_for_phase(pump, cross, target_word, bar_target=None, label_prefix=None):
    """Keeps the Tkinter window alive while waiting for `target_word`.
    No timeout: a slow trial is still valid.

    bar_target: if given (mmHg), bar fill = actual pressure / bar_target.

    Returns True when the phase arrives, None if the window was closed.
    Raises PumpFaultError if the Arduino reports FAULT/STOPPED.
    """
    start = time.time()

    last_log      = 0.0                  # console log throttle
    prev_logged_p = pump.pressure_mmhg   # for rise-rate calculation

    while True:
        if not cross._running:
            return None

        elapsed = time.time() - start

        line = pump.read_line()   # also updates pump.pressure_mmhg
        if line:
            print(f"    [arduino] {line}")

            if line == target_word:
                return True

            # Arduino already vented on its own -> abort.
            if line.startswith(FAULT_WORDS):
                raise PumpFaultError(
                    f"El Arduino abortó el trial: '{line}' "
                    f"(esperábamos '{target_word}'). Ya venteó el manguito."
                )

        current = pump.pressure_mmhg

        # Bar follows physical pressure. No numbers are shown to the participant
        # (would reveal the condition and bias pain reports/EEG); mmHg stay in the console.
        if bar_target and bar_target > 0:
            cross.set_bar_fraction(max(0.0, min(1.0, current / bar_target)))

        # Researcher-only console log: pressure and rise rate.
        if bar_target and time.time() - last_log >= PRESSURE_LOG_INTERVAL:
            rate = (current - prev_logged_p) / max(1e-6, time.time() - last_log)
            phase_tag = label_prefix if label_prefix else "?"
            print(f"      [{phase_tag}] {current:5.1f} / {bar_target:.0f} mmHg   "
                  f"({rate:+.1f} mmHg/s)   t={elapsed:.0f}s")
            last_log      = time.time()
            prev_logged_p = current

        # No watchdog here, just wait.

        cross.update()
        time.sleep(0.005)


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
# -----------------------------------------------------------------------------


# **********************************************
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


def run_real_pressure_phases(pump, cross, bci, events, pressure):
    """Runs INFLATING -> HOLDING -> DEFLATING on the real pump, syncing the
    bar and BCI2000 states with Arduino reports. Timing is set by hardware.

    No software watchdogs; only an Arduino FAULT interrupts a trial.
    Returns True on success, None if the window was closed.
    """

    def log(phase, vector):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        events.append({"phase": phase, "vector": vector, "timestamp": ts})

    # ---------- INFLATING ----------
    cross.set_bar_label("INFLATING")
    # StimulusPhase and PressureTarget set together when inflation is commanded
    log("INFLATING", 1)
    safe_set_states(bci, StimulusPhase=1, PressureTarget=pressure)
    set_signal_amplitude(bci, pressure_to_amplitude(pressure))

    pump.send_start(pressure)   # <-- PUMP ACTUALLY STARTS HERE

    # Confirm the Arduino received START; resend if the ack is lost.
    try:
        ack = wait_for_inflating_ack(pump, cross, pressure)
    except PumpError:
        pump.send_stop()
        raise
    if ack is None:
        pump.send_stop()
        return None

    # Bar rises with real pressure; 100% = this trial's target.
    try:
        got = wait_for_phase(pump, cross, "HOLDING",
                             bar_target=pressure,
                             label_prefix="INFLATING")
    except PumpError:
        pump.send_stop()   # pump can't reach pressure -> stop
        raise
    if got is None:
        pump.send_stop()
        return None

    # ---------- HOLDING ----------
    cross.set_bar_label("HOLDING")
    log("HOLDING", 2)
    safe_set_state(bci, "StimulusPhase", 2)

    # Arduino holds for HOLD_DURATION (10 s), then sends "DEFLATING".
    # Bar keeps showing real pressure, so slow leaks are visible.
    try:
        got = wait_for_phase(pump, cross, "DEFLATING",
                             bar_target=pressure, label_prefix="HOLDING")
    except PumpError:
        pump.send_stop()
        raise
    if got is None:
        pump.send_stop()
        return None

    # ---------- DEFLATING ----------
    cross.set_bar_label("DEFLATING")
    log("DEFLATING", 3)
    safe_set_states(bci, StimulusPhase=3, PressureTarget=0)
    set_signal_amplitude(bci, AMPLITUDE_BASE_UV)

    # Bar falls with real pressure. "DONE" arrives when the sensor reads
    # below 3 mmHg (DEFLATE_DONE_MMHG in the .ino).
    try:
        got = wait_for_phase(pump, cross, "DONE",
                             bar_target=pressure, label_prefix="DEFLATING")
    except PumpError:
        pump.send_stop()
        raise
    if got is None:
        pump.send_stop()
        return None

    cross.set_bar_fraction(0.0)
    return True


def run_trial(bci, cross, pump, trial_number, condition, pressure, block_number):
    print(f"\n  Trial {trial_number} | Run {block_number} | {condition} | {pressure} mmHg")

    events = []

    def log_event(phase, vector):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        events.append({"phase": phase, "vector": vector, "timestamp": ts})
        safe_set_state(bci, "StimulusPhase", vector)

    # Initial trial states in one call to keep them aligned.
    safe_set_states(bci, TrialNumber=trial_number, PainLevel=PAIN_MAP[condition], PressureTarget=0)
    cross.pump(0.15)  # short pause so labels don't overlap in the Viewer

    # Event 0 — Fixation cross (3 s)
    cross.hide_rest_label()
    cross.hide_pressure_bar()
    cross.show_fixation_cross()
    print("    FIXATION (cruz)")
    log_event("FIXATION", 0)
    if not cross.pump(CROSS_TIME):
        return None

    trial_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    target_amplitude = pressure_to_amplitude(pressure)

    # Events 1-3 — INFLATING / HOLDING / DEFLATING
    cross.hide_fixation_cross()
    cross.show_pressure_bar()

    # All three conditions use the real pump (NONE target is 8 mmHg).
    if pressure <= 0:
        raise PumpError(
            f"Presión inválida en el trial {trial_number} ({condition}): "
            f"{pressure} mmHg. Las tres condiciones tienen que ser > 0."
        )

    print(f"    INFLATING  (bomba REAL -> {pressure} mmHg)")
    result = run_real_pressure_phases(pump, cross, bci, events, pressure)
    if result is None:
        return None

    # Event 4 — Rest (10 s)
    print("    DESCANSO")
    log_event("DESCANSO", 4)
    if not cross.countdown_rest(ITI_TIME, label="DESCANSO"):
        return None

    return {
        "trial":     trial_number,
        "timestamp": trial_timestamp,
        "condition": condition,
        "pressure":  pressure,
        "signal_amplitude_uv": target_amplitude,
        "events":    events,
    }


def run_calibration(bci, cross, pump, pressure):
    """Trial 0 — Calibration at HIGH pressure, with a red "CALIBRACIÓN"
    banner during all stages. No long rest at the end; returns to idle
    (StimulusPhase=0) before Trial 1.
    """
    print(f"\n{'='*50}")
    print(f"  CALIBRACIÓN (Trial 0) — presión: {pressure} mmHg")
    print(f"{'='*50}")

    events = []

    def log_event(phase, vector):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        events.append({"phase": phase, "vector": vector, "timestamp": ts})
        safe_set_state(bci, "StimulusPhase", vector)

    safe_set_states(bci, TrialNumber=0, PainLevel=PAIN_MAP[HIGH_LABEL], PressureTarget=0)

    cross.show_calibration_banner("CALIBRACIÓN")

    # Idle stage — fixation cross (3 s)
    cross.hide_rest_label()
    cross.hide_pressure_bar()
    cross.show_fixation_cross()
    print("    IDLE (cruz)")
    log_event("IDLE", 0)
    if not cross.pump(CROSS_TIME):
        cross.hide_calibration_banner()
        return None

    trial_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    target_amplitude = pressure_to_amplitude(pressure)

    # INFLATING / HOLDING / DEFLATING with the real pump
    cross.hide_fixation_cross()
    cross.show_pressure_bar()
    print(f"    INFLATING  (bomba REAL -> {pressure} mmHg)")
    try:
        result = run_real_pressure_phases(pump, cross, bci, events, pressure)
    except PumpError:
        cross.hide_calibration_banner()
        raise
    if result is None:
        cross.hide_calibration_banner()
        return None

    # Back to idle, no long rest
    cross.hide_pressure_bar()
    safe_set_state(bci, "StimulusPhase", 0)
    log_event("IDLE", 0)
    cross.hide_calibration_banner()

    print("    Calibración completa.\n")

    return {
        "trial":     0,
        "timestamp": trial_timestamp,
        "condition": "CALIBRATION",
        "pressure":  pressure,
        "signal_amplitude_uv": target_amplitude,
        "events":    events,
    }


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

    # All 4 states start at 0 (AddStateVariable default), so the recording
    # begins at a clean baseline. The Viewer only labels changes, not the initial 0.

    bci_disconnected = False
    try:
        # Each execution is a full run (R01, R02...); the number only names files.
        run_num = participant_data["run_number"]

        # Trial 0 — Calibration runs in every run (cuff refitted, sensor re-zeroed).
        calibration_log = run_calibration(bci, cross, pump, high)
        if calibration_log is None:
            print("\n  Experimento interrumpido durante la calibración (ventana cerrada).")
            raise BCI2000DisconnectedError("Ventana de la cruz de fijación cerrada durante la calibración.")
        append_trial_to_results(participant_data, results, block_number=0, trial_data=calibration_log)

        # Wait for ENTER after calibration to allow checking the signal.
        print(f"\n{'='*50}")
        print("  Calibración terminada. Presiona ENTER en la ventana de la cruz")
        print("  para iniciar los trials...")
        print(f"{'='*50}")
        cross.wait_for_start(message="Calibración completa. Presione ENTER para iniciar")

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
            trial_log = run_trial(bci, cross, pump, global_trial, condition, pressure, run_num)

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
        save_results(participant_data, results)

        print("\n" + "="*50)
        print("  EXPERIMENT FINISHED")
        print(f"  Results (JSON) saved → {get_results_path(participant_data)}")
        print(f"  Brain recording (.dat) saved → {dat_path}")
        print("="*50)


if __name__ == "__main__":
    main()