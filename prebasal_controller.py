"""
PREBASAL — estado basal previo al experimento de dolor.

Graba un .dat de BCI2000 con el participante en reposo, viendo una cruz de
fijación, durante PREBASAL_SECONDS (5 min por defecto). NO abre el puerto
serial, NO importa pyserial y NO toca el Arduino: si la bomba está
desconectada o el .ino no está cargado, este programa corre igual.

Por eso la clase FixationCross de aquí es una versión mínima y propia, y no
un import de experiment_controller_cruz.py: ese archivo hace `import serial`
en el encabezado, así que importarlo obligaría a tener la bomba lista para
grabar un basal donde no hay bomba. Lo único que se comparte son las dos
piezas que sí tienen que ser idénticas: participant_interface.launch_interface
y bci_setup_cruz_con_ruta.setup_bci.

CÓMO SE NUMERA ESTE RUN
-----------------------
El basal consume un número de run como cualquier otro (sale de los .dat que
ya existen en disco). O sea: si es lo primero que corres con el participante,
el basal queda en R01 y el experimento de dolor arranca en R02. Si prefieres
tenerlos separados, en la ventana de registro cambia el Session Number a mano
(p. ej. sesión 99 para basales) y el run se recalcula solo para esa sesión.

MARCADORES EN EL .dat
---------------------
    StimulusPhase = 0   pre-roll y post-roll (cruz, sin marcar)
    StimulusPhase = 5   PREBASAL — los 5 min de reposo
    PainLevel = PressureTarget = TrialNumber = 0   todo el run

El 5 es un código NUEVO, no reutiliza el 0 del experimento de dolor. Así, al
epocar en MNE, el segmento basal se recorta con un solo criterio
(StimulusPhase == 5) y nunca se confunde con las cruces de 3 s de los trials.
"""

import os
import json
import time
import tkinter as tk
from datetime import datetime

from participant_interface import launch_interface
from bci_setup_cruz_con_ruta import setup_bci


# --- Duraciones -------------------------------------------------------------
PREBASAL_SECONDS = 300      # 5 min de reposo marcados con StimulusPhase = 5

# Colchones antes y después del basal. El .dat empieza a escribirse unos
# instantes antes de que el estado cambie, y BCI2000 aplica los estados por
# bloques de muestras (SampleBlockSize=32 -> 125 ms). Estos segundos de
# sobra evitan que la primera o la última época del basal caigan cortadas
# contra el borde del archivo cuando filtres en el análisis.
PRE_ROLL_SECONDS  = 5
POST_ROLL_SECONDS = 5

PREBASAL_PHASE = 5

# Cada cuánto imprimir el tiempo restante EN LA CONSOLA. Nunca en la ventana
# del participante: igual que en el experimento, la pantalla del participante
# solo tiene la cruz, sin números ni cuenta regresiva que invite a mirar el
# reloj en vez de descansar.
CONSOLE_TICK = 15


class FixationCross:
    """Ventana negra con una cruz blanca centrada. Versión mínima: solo lo
    que el basal necesita (mostrar la cruz, esperar ENTER, aguantar N
    segundos sin congelarse, cerrar)."""

    def __init__(self, bg_color="black", cross_color="white",
                 arm_pct=0.35, thickness_pct=0.15,
                 win_width=800, win_height=600):
        self.cross_color = cross_color
        self.arm_pct = arm_pct
        self.thickness_pct = thickness_pct
        self.win_width = win_width
        self.win_height = win_height
        self._running = True
        self._start_pressed = False

        self.root = tk.Tk()
        self.root.title("Prebasal — Fixation Cross")
        self.root.configure(bg=bg_color)
        self.root.geometry(f"{win_width}x{win_height}")
        self.root.resizable(True, True)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.bind("<Escape>", self._on_escape)

        self.canvas = tk.Canvas(self.root, bg=bg_color, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self._on_configure)

        self.root.update()
        self.root.update_idletasks()
        self.root.after(100, self._draw_cross)

    # ---------- eventos de ventana ----------

    def _on_configure(self, event=None):
        self._draw_cross()

    def _on_close(self):
        self._running = False

    def _on_escape(self, event=None):
        self._running = False

    def _on_return(self, event=None):
        self._start_pressed = True

    # ---------- dibujo ----------

    def get_win_size(self):
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w <= 1 or h <= 1:
            w, h = self.win_width, self.win_height
        return w, h

    def _draw_cross(self):
        self.canvas.delete("cross")

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

    # ---------- espera ----------

    def wait_for_start(self, message="Presione ENTER para iniciar el basal"):
        w, h = self.get_win_size()
        self.canvas.create_text(
            w // 2, h // 2 + 80,
            text=message, fill="gray70",
            font=("Helvetica", 16), tags="start_msg",
        )
        self.root.update()

        self._start_pressed = False
        self.root.bind("<Return>", self._on_return)

        while not self._start_pressed:
            if not self._running:
                break
            self.root.update()
            time.sleep(0.01)

        self.canvas.delete("start_msg")
        self.root.unbind("<Return>")
        self.root.update()

    def hold(self, seconds):
        """Mantiene la ventana viva `seconds` segundos. Devuelve False si el
        investigador cerró la ventana o presionó Escape."""
        end = time.time() + seconds
        while time.time() < end:
            if not self._running:
                return False
            self.root.update()
            time.sleep(0.01)
        return True

    def close(self):
        self._running = False
        try:
            self.root.destroy()
        except Exception:
            pass


class BCI2000DisconnectedError(Exception):
    """Se perdió la conexión con BCI2000 a media grabación."""
    pass


def safe_set_states(bci, **states):
    """Fija varios estados en UNA sola llamada (mismo criterio que el
    controlador del experimento: menos viajes = menos riesgo de que dos
    estados que deben cambiar juntos caigan en bloques distintos)."""
    cmd = "; ".join(f"SET STATE {name} {value}" for name, value in states.items())
    try:
        ok = bci.Execute(cmd)
        if not ok:
            print(f"    ⚠  No se pudieron actualizar estados: {bci.Result}")
        return ok
    except Exception:
        raise BCI2000DisconnectedError(f"Se perdió la conexión con BCI2000 ({cmd}).")


def hold_with_console_countdown(cross, seconds, label):
    """Aguanta `seconds` mostrando el restante en CONSOLA cada CONSOLE_TICK.
    Devuelve los segundos que realmente transcurrieron (menos que `seconds`
    si se abortó)."""
    started = time.time()
    remaining = seconds

    while remaining > 0:
        step = CONSOLE_TICK
        if remaining < step:
            step = remaining

        if not cross.hold(step):
            return time.time() - started

        remaining -= step
        if remaining > 0:
            mins = int(remaining) // 60
            secs = int(remaining) % 60
            print(f"    {label} — restan {mins}:{str(secs).zfill(2)}")

    return time.time() - started


def get_prebasal_log_path(participant_data):
    subject = "P" + str(participant_data["participant_number"]).zfill(3)
    return f"prebasal_{subject}.json"


def save_prebasal_log(participant_data, record):
    path = get_prebasal_log_path(participant_data)

    if os.path.exists(path):
        with open(path, "r") as f:
            existing = json.load(f)
    else:
        existing = {
            "name":               participant_data["name"],
            "participant_number": participant_data["participant_number"],
            "prebasal_runs":      [],
        }

    existing["prebasal_runs"].append(record)

    with open(path, "w") as f:
        json.dump(existing, f, indent=4)

    print(f"✔  Bitácora del basal → {path}")
    return path


def main():
    print("=" * 60)
    print("  PREBASAL — reposo con cruz de fijación")
    print(f"  Duración marcada: {PREBASAL_SECONDS} s "
          f"({PREBASAL_SECONDS // 60} min)  |  sin bomba, sin Arduino")
    print("=" * 60)

    participant_data = launch_interface()
    if not participant_data:
        print("Registro cancelado. No se grabó nada.")
        return

    print(f"\nParticipante: {participant_data['name']}  "
          f"(P{str(participant_data['participant_number']).zfill(3)}, "
          f"S{str(participant_data['session_number']).zfill(3)}, "
          f"R{str(participant_data['run_number']).zfill(2)})")

    bci, dat_path = setup_bci(participant_data)
    if bci is None:
        print("No se pudo conectar/configurar BCI2000. Abortando.")
        return

    cross = FixationCross()

    print("\n" + "=" * 60)
    print("  Instrucciones al participante ANTES de presionar ENTER:")
    print("    - Ojos abiertos, mirando la cruz.")
    print("    - Quieto, sin hablar, sin apretar la mandíbula.")
    print("    - No cerrar los ojos ni buscar el reloj.")
    print()
    print("  Presiona ENTER en la ventana de la cruz para iniciar.")
    print("  (Escape o cerrar la ventana = abortar)")
    print("  Nada se graba en disco hasta que presiones ENTER.")
    print("=" * 60)

    cross.wait_for_start()

    if not cross._running:
        print("\n  Ventana cerrada antes de iniciar. Basal cancelado.")
        print("  No se creó ningún .dat — el run sigue libre.")
        cross.close()
        return

    print("\n  ¡Señal recibida! Grabando...\n")

    started_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    prebasal_elapsed = 0.0
    completed = False
    disconnected = False

    # A partir de aquí el .dat existe y está creciendo.
    bci.Start()
    time.sleep(2)

    try:
        # --- Pre-roll: cruz, estados en 0 ---------------------------------
        safe_set_states(bci, StimulusPhase=0, PainLevel=0,
                        PressureTarget=0, TrialNumber=0)
        print(f"  Pre-roll ({PRE_ROLL_SECONDS} s)...")
        if not cross.hold(PRE_ROLL_SECONDS):
            raise BCI2000DisconnectedError("Ventana cerrada durante el pre-roll.")

        # --- Basal --------------------------------------------------------
        safe_set_states(bci, StimulusPhase=PREBASAL_PHASE)
        print(f"  PREBASAL — StimulusPhase = {PREBASAL_PHASE} — "
              f"{PREBASAL_SECONDS} s")

        prebasal_elapsed = hold_with_console_countdown(
            cross, PREBASAL_SECONDS, "PREBASAL"
        )

        if not cross._running:
            safe_set_states(bci, StimulusPhase=0)
            raise BCI2000DisconnectedError(
                f"Ventana cerrada a los {prebasal_elapsed:.1f} s del basal."
            )

        completed = True

        # --- Post-roll ----------------------------------------------------
        safe_set_states(bci, StimulusPhase=0)
        print(f"  Post-roll ({POST_ROLL_SECONDS} s)...")
        cross.hold(POST_ROLL_SECONDS)

        print("\n  Basal completo.")

    except BCI2000DisconnectedError as e:
        disconnected = True
        print(f"\n⚠  {e}")
        print("   Se detuvo la grabación. El .dat conserva lo que alcanzó a grabarse.")

    except Exception as e:
        disconnected = True
        print(f"\n⚠  Error inesperado, se detuvo la grabación ({type(e).__name__}).")

    finally:
        try:
            bci.Stop()
        except Exception:
            pass

        try:
            cross.close()
        except Exception:
            pass

        record = {
            "session_number":     participant_data["session_number"],
            "run_number":         participant_data["run_number"],
            "start_timestamp":    started_at,
            "end_timestamp":      datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "planned_seconds":    PREBASAL_SECONDS,
            "actual_seconds":     round(prebasal_elapsed, 1),
            "stimulus_phase_code": PREBASAL_PHASE,
            "completed":          completed,
            "dat_path":           dat_path,
        }
        if disconnected:
            record["ended_early"] = True

        save_prebasal_log(participant_data, record)

        print("\n" + "=" * 60)
        print("  PREBASAL FINISHED")
        print(f"  Basal marcado: {prebasal_elapsed:.1f} s de "
              f"{PREBASAL_SECONDS} s planeados")
        print(f"  Brain recording (.dat) → {dat_path}")
        print("=" * 60)


if __name__ == "__main__":
    main()
