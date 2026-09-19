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

# NOTA: ya no usamos la librería "keyboard" (requería permisos de
# administrador en Windows para detectar teclas de forma confiable).
# En su lugar, el programa espera a que escribas algo en la misma
# terminal y presiones Enter — no necesita ninguna librería extra ni
# permisos especiales.

# Presión de la condición control. YA NO es 0: el manguito se infla también
# en NONE, solo que muy poco (8 mmHg). Así el participante siente el mismo
# ruido de bomba, la misma vibración y el mismo contacto en las tres
# condiciones, y lo único que cambia entre NONE/LOW/HIGH es la CANTIDAD de
# presión — que es justo lo que queremos que distinga el clasificador.
#
# Se puede sobrescribir desde participant_XXX.json con "none_pressure".
NONE_PRESSURE    = 8
LOW_LABEL        = "LOW"
HIGH_LABEL       = "HIGH"
NONE_LABEL       = "NONE"

TRIALS_PER_BLOCK = 30
TRIALS_PER_COND  = 10

# DISEÑO DE UN SOLO RUN.
#
# Antes el experimento eran 4 bloques = 4 ejecuciones (R01..R04), con la
# calibración solo en la primera y los trials numerados de corrido (el
# bloque 2 empezaba en el trial 31).
#
# Ahora cada ejecución es un experimento COMPLETO y autocontenido:
#   - calibración SIEMPRE
#   - trials numerados 1..TRIALS_PER_BLOCK, siempre
#   - un .dat que se cierra al terminar
#
# Volver a correr el programa con el mismo participante y sesión genera R02,
# luego R03, etc. (participant_interface.next_run_from_disk cuenta los .dat
# que ya existen). No hay un número total de runs previsto: se corre las
# veces que haga falta.
TRIALS_TOTAL     = TRIALS_PER_BLOCK

BLOCK_REST       = 30

CROSS_TIME       = 3    # Evento 0 — cruz de fijación
ITI_TIME         = 10   # Evento 4 — DESCANSO (dentro del propio trial)

# Estos tres YA NO controlan nada: ahora las tres condiciones usan la bomba
# real y quien marca el ritmo es el Arduino (llega a la presión -> HOLDING;
# pasan HOLD_DURATION ms -> DEFLATING; baja de 3 mmHg -> DONE). Se dejan
# como referencia de las duraciones esperadas, para el análisis.
INFLATING_TIME   = 7    # Evento 1 — INFLATING (referencia)
HOLDING_TIME     = 10   # Evento 2 — HOLDING   (referencia)
DEFLATING_TIME   = 3    # Evento 3 — DEFLATING (referencia)

PAIN_MAP = {NONE_LABEL: 0, LOW_LABEL: 1, HIGH_LABEL: 2}


# --- Arduino (bomba de presión real) ---------------------------------------
# Puerto serial del Arduino. En Windows suele ser "COM3", "COM4", etc.
# Déjalo en None para intentar autodetectarlo (busca un puerto Arduino/CH340).
ARDUINO_PORT = None          # p. ej. "COM3"  — pon el tuyo aquí si autodetección falla
ARDUINO_BAUD = 115200        # debe coincidir con Serial.begin() del .ino

# --- Sin watchdogs de software ----------------------------------------------
# ANTES había un detector de estancamiento (STALL_*) y cronómetros por fase
# (ACK/INFLATE/HOLD/DEFLATE_TIMEOUT). Abortaban trials que solo eran lentos.
#
# AHORA: Python NO aborta nada por tiempo. Espera a que el Arduino reporte la
# siguiente fase, tarde lo que tarde. El fin del desinflado lo decide UNA sola
# cosa, y vive en el .ino: el MPX5050DP tiene que marcar menos de
# DEFLATE_DONE_MMHG (3 mmHg). Ese es el único criterio.
#
# Lo único que sigue deteniendo un trial es un FAULT del Arduino, y el Arduino
# solo levanta FAULT si se disparó un seguro físico real (sobrepresión o bomba
# encendida más de MAX_INFLATE_MS). Eso no es un castigo por lentitud: es lo
# que evita que la bomba siga inflando el brazo del participante si se sale una
# manguera o el sensor deja de leer.

# Cada cuánto imprimir la presión en consola mientras infla (para que puedas
# VER qué tan rápido sube y diagnosticar la bomba con números reales).
PRESSURE_LOG_INTERVAL = 1.0

# HOLD_SECONDS tiene que ser IGUAL a HOLD_DURATION del .ino (10000 ms).
# Es el único número que vive duplicado en los dos programas, así que si
# cambias uno tienes que cambiar el otro.
HOLD_SECONDS = 10.0

# Presión máxima que este programa acepta pedirle al Arduino. El .ino tiene
# su propio tope (MAX_SAFE_MMHG); este es el filtro del lado de Python, para
# que un JSON de participante mal escrito no llegue nunca al hardware.
MAX_SAFE_MMHG = 200.0

# Palabras que el Arduino manda cuando algo salió mal. Si aparecen mientras
# esperamos una fase, el trial se aborta de inmediato en vez de quedarse
# esperando una fase que ya nunca va a llegar.
FAULT_WORDS = ("STOPPED", "FAULT")

# Vocabulario completo del protocolo. Todo lo que NO esté aquí (ni sea
# telemetría "P,...") es basura: una línea partida por un desborde de buffer.
PHASE_WORDS = ("READY", "INFLATING", "HOLDING", "DEFLATING", "DONE")


class PumpError(Exception):
    """Error de comunicación con el Arduino de la bomba."""
    pass


class PumpTimeoutError(PumpError):
    """El Arduino no reportó la siguiente fase a tiempo -> se manda STOP."""
    pass


class PumpFaultError(PumpError):
    """El Arduino reportó una falla (FAULT/STOPPED) y ya venteó por su cuenta."""
    pass


def _autodetect_arduino_port():
    """Busca un puerto que parezca un Arduino/clon CH340. Devuelve el device
    (p. ej. 'COM3') o None si no encuentra nada convincente."""
    candidates = []
    for p in serial.tools.list_ports.comports():
        blob = f"{p.description} {p.manufacturer} {p.hwid}".lower()
        if any(k in blob for k in ("arduino", "ch340", "usb serial", "wchusb", "usb-serial")):
            candidates.append(p.device)
    if candidates:
        return candidates[0]
    return None


class ArduinoPump:
    """Envoltura fina sobre el puerto serial del Arduino.

    Protocolo (definido en arduino_pressure_controller.ino):
        ->  START,<mmHg>   inicia inflado; responde  INFLATING
                           al llegar a la presión    HOLDING
                           tras HOLD_DURATION        DEFLATING
                           al bajar de 3 mmHg        DONE
        ->  STOP           corte de emergencia;      STOPPED
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
        # timeout=0 -> lecturas NO bloqueantes (para no congelar la ventana tkinter)
        self.ser = serial.Serial(port, baud, timeout=0)

        # Al abrir el puerto el Arduino se reinicia (DTR). Hay que esperar a
        # que arranque antes de mandarle nada, o el primer START se pierde.
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self._buf = ""

        # Última presión REAL leída del MPX5050DP (mmHg). La barra de la UI
        # se dibuja a partir de esto, no de un temporizador.
        self.pressure_mmhg = 0.0
        self.target_mmhg   = 0.0

        self._handshake()

        print(f"Arduino listo en {port}. Presión actual: {self.pressure_mmhg:.1f} mmHg")

    def _handshake(self):
        """Manda PING y espera READY. Confirma que del otro lado hay un
        Arduino con ESTE firmware y no cualquier otro puerto serial. De paso
        deja llegar la primera telemetría, para que la barra no arranque
        creyendo que la presión es 0 cuando el manguito ya trae algo."""
        deadline = time.time() + 5.0
        self.ser.write(b"PING\n")
        self.ser.flush()

        while time.time() < deadline:
            line = self.read_line()

            if line and line.startswith("ZERO,"):
                # El Arduino acaba de medir el cero real del MPX5050DP. Si el
                # offset es grande, aqui es donde te enteras.
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
        """Vacía el puerto serial SIN esperar ninguna fase.

        ESTA ES LA FUNCIÓN QUE FALTABA. El Arduino manda telemetría a 20 Hz
        SIEMPRE, esté inflando o en IDLE. Python solo leía el puerto dentro de
        wait_for_phase(), o sea: durante la cruz, los trials NONE y los
        descansos NADIE leía. Tres trials NONE seguidos = ~100 s sin leer, con
        ~280 bytes/s entrando. El buffer del driver de Windows (4 KB) se
        desborda y empieza a TIRAR bytes a media línea. Ahí se perdió el
        "INFLATING" del trial 5, y Python se quedó esperándolo para siempre.

        Llamando a esto en cada tick de la ventana, el buffer nunca se llena.
        De paso, pressure_mmhg se mantiene fresco entre trials.
        """
        try:
            while self.ser.in_waiting:
                self.read_line()   # actualiza pressure_mmhg; tira fases viejas
        except Exception:
            pass   # un drain que falla no debe tumbar el experimento

    def send_start(self, mmhg):
        if mmhg <= 0 or mmhg > MAX_SAFE_MMHG:
            raise PumpError(
                f"Presión objetivo fuera de rango: {mmhg} mmHg "
                f"(el máximo permitido es {MAX_SAFE_MMHG:.0f})."
            )

        target = int(round(mmhg))

        # Tirar TODO lo viejo antes de mandar el comando. Así el "INFLATING"
        # que leamos a continuación es forzosamente el de ESTE trial, y no
        # venimos arrastrando telemetría rancia de hace 100 segundos.
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._buf = ""

        # Fijamos el target localmente ANTES de mandar el comando. Así la
        # barra ya sabe cuál es su 100% desde el primer frame, sin tener que
        # esperar a que llegue la primera línea de telemetría.
        self.target_mmhg = float(target)

        self.ser.write(f"START,{target}\n".encode("ascii"))
        self.ser.flush()

    def send_stop(self):
        try:
            self.ser.write(b"STOP\n")
            self.ser.flush()
        except Exception:
            pass  # en un corte de emergencia no queremos que un error tape el STOP

    def read_line(self):
        """Devuelve UNA línea de FASE del Arduino (INFLATING/HOLDING/...), o
        None si todavía no hay ninguna. No bloquea.

        Las líneas de telemetría ("P,<actual>,<target>") se consumen aquí
        mismo: actualizan self.pressure_mmhg y NO se devuelven, para que el
        código de fases no tenga que filtrarlas."""
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

            # ---- Telemetría de presión real ----
            if line.startswith("P,"):
                parts = line.split(",")
                if len(parts) >= 3:
                    try:
                        self.pressure_mmhg = float(parts[1])
                        self.target_mmhg   = float(parts[2])
                    except ValueError:
                        pass
                continue   # no es una fase: seguir buscando

            # ---- Palabra de fase real ----
            # Solo se aceptan palabras del protocolo. Cualquier otra cosa es
            # una línea truncada por un desborde del buffer (p. ej. "0,0.0",
            # que es la cola de un "P,45,70.0" partido a la mitad) y se tira.
            # ANTES esto se devolvía como si fuera una fase y ensuciaba el log.
            if line in PHASE_WORDS or line.startswith(FAULT_WORDS) or line.startswith("ZERO,"):
                return line

            continue

        return None

    def close(self):
        """Cierra el puerto de forma segura, mandando STOP antes por si la
        bomba seguía encendida."""
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
    """Espera el "INFLATING" que confirma que el Arduino recibió el START.

    Si no llega en `resend_every` segundos, REENVÍA el START. No aborta: sigue
    reintentando. Un START perdido (cable, ruido, buffer) ya no congela el
    experimento — simplemente se vuelve a mandar hasta que el Arduino conteste.

    Mandar START dos veces es inofensivo: el .ino solo vuelve a fijar el target
    y reinicia phaseStart.

    Devuelve True cuando llega el ack, None si el usuario cerró la ventana.
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
    """Mantiene viva la ventana tkinter mientras espera a que el Arduino
    reporte `target_word` (p. ej. "HOLDING").

    NO hay cronómetro. Espera indefinidamente. Un trial lento sigue siendo un
    trial válido, así que el software no lo aborta.

    bar_target: si se pasa (mmHg), la barra se dibuja como
                presion_real / bar_target. O sea: el 100% de la barra ES la
                presión objetivo de ESTE trial. Si el target es 70 mmHg, 70
                llena la barra; si es 30, 30 la llena.

    Devuelve True si llegó la fase, None si el usuario cerró la ventana.
    Lanza PumpFaultError solo si el Arduino reporta FAULT/STOPPED (o sea, si
    se disparó un seguro físico y el manguito YA se venteó).
    """
    start = time.time()

    last_log      = 0.0                  # throttle del log de consola
    prev_logged_p = pump.pressure_mmhg   # para calcular la tasa de subida

    while True:
        if not cross._running:
            return None

        elapsed = time.time() - start

        line = pump.read_line()   # también actualiza pump.pressure_mmhg
        if line:
            print(f"    [arduino] {line}")

            if line == target_word:
                return True

            # El Arduino ya venteó por su cuenta (sobrepresión, timeout de
            # hardware, STOP). No tiene caso seguir esperando una fase que
            # nunca va a llegar: abortamos ya.
            if line.startswith(FAULT_WORDS):
                raise PumpFaultError(
                    f"El Arduino abortó el trial: '{line}' "
                    f"(esperábamos '{target_word}'). Ya venteó el manguito."
                )

        current = pump.pressure_mmhg

        # ---- Barra vinculada a la presión FÍSICA ----
        #
        # La barra SÍ se mueve con la presión real, pero la etiqueta de la
        # ventana NUNCA muestra números. El participante solo ve la palabra
        # de la fase (INFLATING/HOLDING/DEFLATING), que ya se puso una vez
        # en run_real_pressure_phases antes de entrar aquí.
        #
        # Enseñarle "42 / 70 mmHg" le decía en qué condición estaba y qué
        # tanto le faltaba: eso contamina el reporte subjetivo de dolor y
        # mete expectativa en el EEG. Los números se quedan del lado del
        # investigador, en la consola.
        if bar_target and bar_target > 0:
            cross.set_bar_fraction(max(0.0, min(1.0, current / bar_target)))

        # ---- Log en consola: presión real y qué tan rápido sube ----
        # (esto lo ve el investigador, no el participante)
        if bar_target and time.time() - last_log >= PRESSURE_LOG_INTERVAL:
            rate = (current - prev_logged_p) / max(1e-6, time.time() - last_log)
            phase_tag = label_prefix if label_prefix else "?"
            print(f"      [{phase_tag}] {current:5.1f} / {bar_target:.0f} mmHg   "
                  f"({rate:+.1f} mmHg/s)   t={elapsed:.0f}s")
            last_log      = time.time()
            prev_logged_p = current

        # Aquí NO hay watchdog. Ni de estancamiento ni de reloj. Se espera.

        cross.update()
        time.sleep(0.005)


# --- Modulación de la señal "cerebral" ficticia según la presión -----------
AMPLITUDE_BASE_UV = 50.0   # amplitud cuando no hay presión / entre fases
AMPLITUDE_GAIN_UV = 2.5    # cuánto sube la amplitud por cada mmHg de presión


def pressure_to_amplitude(pressure):
    return AMPLITUDE_BASE_UV + AMPLITUDE_GAIN_UV * pressure


class BCI2000DisconnectedError(Exception):
    """Se usa para detener el experimento por completo si BCI2000 se desconecta."""
    pass


def set_signal_amplitude(bci, amplitude_uv):
    """Cambia SineAmplitude en vivo. No requiere SetConfig ni detener el run."""
    try:
        ok = bci.Execute(f"SET PARAMETER SineAmplitude {amplitude_uv:.1f}")
        if not ok:
            print(f"    ⚠  No se pudo actualizar SineAmplitude: {bci.Result}")
        return ok
    except Exception:
        raise BCI2000DisconnectedError("Se perdió la conexión con BCI2000 (SineAmplitude).")


def safe_set_state(bci, name, value):
    """SetStateVariable. Si BCI2000 se desconectó, detiene el experimento
    en vez de seguir corriendo trials 'a ciegas' sin conexión real."""
    try:
        bci.SetStateVariable(name, value)
        return True
    except Exception:
        raise BCI2000DisconnectedError(f"Se perdió la conexión con BCI2000 ({name}).")


def safe_set_states(bci, **states):
    """Fija VARIOS estados en UNA sola llamada a BCI2000 (un solo viaje de
    red), en vez de una llamada separada por estado. Esto reduce mucho el
    margen de que dos estados que deberían cambiar juntos (ej. StimulusPhase
    y PressureTarget al iniciar INFLATING) caigan en bloques de muestra
    distintos por simple diferencia de timing entre llamadas separadas.
    No garantiza alineación perfecta al 100% (BCI2000 aplica los estados
    por bloques de muestras, eso es estructural del sistema), pero sí
    minimiza los casos de desalineación visual."""
    cmd = "; ".join(f"SET STATE {name} {value}" for name, value in states.items())
    try:
        ok = bci.Execute(cmd)
        if not ok:
            print(f"    ⚠  No se pudieron actualizar estados: {bci.Result}")
        return ok
    except Exception:
        raise BCI2000DisconnectedError(f"Se perdió la conexión con BCI2000 ({cmd}).")
# -----------------------------------------------------------------------------


# ──────────────────────────────────────────────
# Fixation Cross + barra de presión + descanso
# ──────────────────────────────────────────────

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
        self._cross_visible = True   # visible por defecto (estado de reposo/evento 0)
        self._calibration_banner_id = None
        self._bar_visible = False        # para saber si redibujar la barra al redimensionar
        self._current_bar_label = None   # texto actual de la barra (INFLATING/HOLDING/DEFLATING)

        # Se conecta en main() a pump.drain. Se llama en CADA tick de espera
        # (cruz, trials NONE, descansos) para que el puerto serial nunca se
        # quede sin leer. Sin esto, el buffer se desborda y se pierden fases.
        self.serial_drain = None

        self.root = tk.Tk()
        self.root.title("Fixation Cross")
        self.root.configure(bg=bg_color)
        self.root.geometry(f"{win_width}x{win_height}")   # ventana normal, no fullscreen
        self.root.resizable(True, True)                   # se puede maximizar/arrastrar a otro monitor

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.bind("<Escape>", self._on_escape)

        self.canvas = tk.Canvas(self.root, bg=bg_color, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda e: self._on_configure())

        self.root.update()
        self.root.update_idletasks()
        self.root.after(100, self._draw_cross)

    def _on_configure(self):
        """Se dispara en CUALQUIER cambio de tamaño de la ventana (maximizar,
        restaurar, arrastrar a otro monitor). Redibuja todo lo que esté
        visible en ese momento usando la geometría actual, para que nada
        quede desalineado (antes solo se redibujaba la cruz, y la barra se
        quedaba 'chueca' porque el marco no se recalculaba)."""
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
        """Muestra la cruz (usar durante el evento 0 / reposo)."""
        self._cross_visible = True
        self._draw_cross()

    def hide_fixation_cross(self):
        """Oculta la cruz (usar al iniciar el evento 1 / INFLATING)."""
        self._cross_visible = False
        self.canvas.delete("cross")

    def wait_for_start(self, message: str = "Presione ENTER para iniciar el protocolo"):
        """Muestra un mensaje y espera a que el investigador presione ENTER para continuar."""
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
        """Un tick de espera: refresca la ventana Y vacía el puerto serial.
        Todo bucle de espera de esta clase pasa por aquí."""
        if self.serial_drain is not None:
            self.serial_drain()
        self.root.update()

    def pump(self, seconds: float) -> bool:
        """Mantiene la ventana responsiva durante `seconds` segundos.
        Retorna False si el usuario cerró o presionó Escape."""
        end = time.time() + seconds
        while time.time() < end:
            if not self._running:
                return False
            self._tick()
            time.sleep(0.01)
        return True

    def update(self):
        """Un solo tick del event loop de tkinter."""
        if self._running:
            self.root.update()

    def close(self):
        self._running = False
        try:
            self.root.destroy()
        except Exception:
            pass

    # ---------- Banner de calibración (persiste durante todo el trial 0) ----------

    def show_calibration_banner(self, text="CALIBRACIÓN"):
        """Muestra un recuadro fijo arriba de la ventana indicando que se
        está corriendo la calibración. Se mantiene visible durante todas
        las etapas (idle/inflating/holding/deflating) del trial 0."""
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

    # ---------- Barra de presión (INFLATING / HOLDING / DEFLATING) ----------

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

    # ---------- Descanso: palabra grande + contador + barra que se vacía ----------

    def _draw_rest_text(self, label: str, remaining: float):
        """Dibuja/actualiza 'DESCANSO' en grande y el conteo debajo, justo
        arriba de la barra — así quedan agrupados visualmente en el centro
        de la ventana, sin depender del tamaño de la cruz."""
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
    """Las tres condiciones usan la bomba REAL. NONE ya no es 'sin presión':
    es la presión más baja de las tres (8 mmHg por defecto)."""
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
    """Ejecuta las fases INFLATING -> HOLDING -> DEFLATING usando la bomba
    REAL, sincronizando la barra visual y los estados de BCI2000 con lo que
    reporta el Arduino. Los tiempos ya NO son fijos: los marca el hardware.

    No hay watchdogs de software: si una fase tarda, se espera. Lo único que
    interrumpe un trial es un FAULT del Arduino (seguro físico ya disparado,
    manguito ya venteado).

    Devuelve True si terminó bien, o None si el usuario cerró la ventana.
    """

    def log(phase, vector):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        events.append({"phase": phase, "vector": vector, "timestamp": ts})

    # ---------- INFLATING ----------
    cross.set_bar_label("INFLATING")
    # StimulusPhase y PressureTarget juntos, en el instante en que ordenamos inflar
    log("INFLATING", 1)
    safe_set_states(bci, StimulusPhase=1, PressureTarget=pressure)
    set_signal_amplitude(bci, pressure_to_amplitude(pressure))

    pump.send_start(pressure)   # <-- LA BOMBA REALMENTE ARRANCA AQUÍ

    # Confirmar que el Arduino recibió el comando (imprime "INFLATING").
    # Si el ack se pierde, se reenvía el START en vez de esperar para siempre.
    try:
        ack = wait_for_inflating_ack(pump, cross, pressure)
    except PumpError:
        pump.send_stop()
        raise
    if ack is None:
        pump.send_stop()
        return None

    # La barra SUBE conforme sube la presión real del MPX. El 100% de la barra
    # es `pressure` (el target de ESTE trial): 70 mmHg llena la barra si el
    # target es 70; 30 mmHg la llena si el target es 30.
    # Sin watchdog: si la bomba tarda, se espera.
    try:
        got = wait_for_phase(pump, cross, "HOLDING",
                             bar_target=pressure,
                             label_prefix="INFLATING")
    except PumpError:
        pump.send_stop()   # la bomba no llega a la presión -> cortar
        raise
    if got is None:
        pump.send_stop()
        return None

    # ---------- HOLDING ----------
    cross.set_bar_label("HOLDING")
    log("HOLDING", 2)
    safe_set_state(bci, "StimulusPhase", 2)

    # El Arduino mantiene la presión HOLD_DURATION (10s) por su cuenta y luego
    # imprime "DEFLATING". La barra sigue mostrando la presión real: si hay una
    # fuga lenta durante el hold, la vas a VER bajar en la barra.
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

    # La barra BAJA sola, porque sigue reflejando la presión real mientras
    # el manguito se vacía. No hay animación inventada.
    #
    # El "DONE" llega cuando el MPX5050DP marca menos de 3 mmHg
    # (DEFLATE_DONE_MMHG en el .ino). Ese es el ÚNICO criterio de desinflado.
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

    # Los 3 estados iniciales del trial en UNA sola llamada (un solo viaje
    # de red), para que queden alineados entre sí lo más posible.
    safe_set_states(bci, TrialNumber=trial_number, PainLevel=PAIN_MAP[condition], PressureTarget=0)
    cross.pump(0.15)  # pequeña pausa: evita que las etiquetas se amontonen en el Viewer

    # Evento 0 — Cruz de fijación (3s)
    cross.hide_rest_label()
    cross.hide_pressure_bar()
    cross.show_fixation_cross()
    print("    FIXATION (cruz)")
    log_event("FIXATION", 0)
    if not cross.pump(CROSS_TIME):
        return None

    trial_timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    target_amplitude = pressure_to_amplitude(pressure)

    # Eventos 1-3 — INFLATING / HOLDING / DEFLATING
    cross.hide_fixation_cross()
    cross.show_pressure_bar()

    # Las TRES condiciones (NONE/LOW/HIGH) pasan por la bomba real. Ya no
    # existe la rama "simulada" de 0 mmHg con barra animada por temporizador:
    # en NONE el Arduino arranca igual que siempre, solo que el target es 8.
    if pressure <= 0:
        raise PumpError(
            f"Presión inválida en el trial {trial_number} ({condition}): "
            f"{pressure} mmHg. Las tres condiciones tienen que ser > 0."
        )

    print(f"    INFLATING  (bomba REAL -> {pressure} mmHg)")
    result = run_real_pressure_phases(pump, cross, bci, events, pressure)
    if result is None:
        return None

    # Evento 4 — DESCANSO (10s)
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
    """
    Trial 0 — Calibración. Corre UNA sola vez antes de todos los bloques.
    Usa las mismas duraciones que un trial normal (INFLATING 7s, HOLDING 10s,
    DEFLATING 3s) y la presión HIGH, con un recuadro rojo fijo 'CALIBRACIÓN'
    visible durante todas las etapas (idle/inflating/holding/deflating).
    No tiene fase de descanso larga al final: solo regresa a idle
    (StimulusPhase=0) para dar paso directo al Trial 1.
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

    # Etapa idle — cruz de fijación (3s)
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

    # Etapas INFLATING / HOLDING / DEFLATING con la BOMBA REAL
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

    # Regresa a idle — sin descanso largo, listo para pasar al Trial 1
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

    none = cfg.get("none_pressure", NONE_PRESSURE)   ## control: la bomba SÍ arranca
    low  = cfg.get("low_pressure",  46) ## Change the low
    high = cfg.get("high_pressure", 111) ## Change the high

    # Filtro del lado de Python: un JSON mal escrito no debe poder mandarle
    # una presión absurda al Arduino. El .ino tiene su propio tope, esta es
    # la primera línea de defensa.
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

    # --- Abrir la bomba (Arduino) ANTES de arrancar el protocolo ---
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

    # OJO con el orden: bci.Start() NO va aquí.
    #
    # BCI2000 crea y empieza a escribir el .dat en el instante en que recibe
    # Start(). Si arrancamos aquí y luego cierras la ventana de la cruz sin
    # presionar ENTER, ya quedó un .dat de 15-20 segundos en disco — y como
    # BCI2000 nunca sobrescribe, el siguiente intento se guarda como R02.
    # Así se acumularon P001S001R01/R02/R03 de 15 s, 23 s y 122 s.
    #
    # Ahora Start() va DESPUÉS de que confirmas con ENTER. Si abortas antes,
    # no se crea ningún archivo y el run sigue disponible.

    cross = FixationCross()

    # ENLACE CRÍTICO: a partir de aquí, cada tick de espera de la ventana
    # (cruz, trials NONE, descansos) vacía el puerto serial. Sin esta línea,
    # el buffer se desborda durante los trials NONE y el siguiente trial con
    # presión se queda congelado esperando un "INFLATING" que se perdió.
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

    # AQUÍ sí: a partir de este momento el .dat existe y está creciendo.
    bci.Start()
    time.sleep(2)
    set_signal_amplitude(bci, AMPLITUDE_BASE_UV)

    # Los 4 estados ya nacen en 0 por defecto (AddStateVariable), así que
    # la grabación empieza en un baseline de 0 real y limpio. El Viewer de
    # BCI2000 no dibuja una etiqueta de texto para ese valor inicial (solo
    # marca CAMBIOS), pero la traza sí está correctamente en 0 desde el
    # primer instante — no se fuerza ningún valor "dummy" que contamine
    # el .dat.

    bci_disconnected = False
    try:
        # Cada ejecución es un run completo: R01, R02, R03... El número solo
        # sirve para nombrar el .dat y el JSON de resultados, no cambia lo que
        # pasa adentro.
        run_num = participant_data["run_number"]

        # Trial 0 — Calibración. Ahora corre SIEMPRE, en todos los runs. Cada
        # run es una sesión de grabación independiente: el manguito se vuelve
        # a colocar, el sensor se vuelve a cerar y el participante lleva otro
        # rato sentado, así que la calibración del run anterior ya no aplica.
        calibration_log = run_calibration(bci, cross, pump, high)
        if calibration_log is None:
            print("\n  Experimento interrumpido durante la calibración (ventana cerrada).")
            raise BCI2000DisconnectedError("Ventana de la cruz de fijación cerrada durante la calibración.")
        append_trial_to_results(participant_data, results, block_number=0, trial_data=calibration_log)

        # Pausa después de la calibración: espera ENTER antes de arrancar
        # los trials. Permite revisar la señal/calibración con calma.
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

        # Los trials se numeran 1..30 en CADA run. Antes se numeraban de
        # corrido entre bloques (el bloque 2 empezaba en 31), pero ahora cada
        # .dat es un experimento completo y el TrialNumber tiene que poder
        # leerse solo, sin saber de qué run vino el archivo.
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
        # STOP + cierre del puerto SIEMPRE (aunque haya crash): la bomba nunca
        # debe quedar encendida al terminar.
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