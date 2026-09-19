"""
plot_recorded_signal.py  (versión BCI2kReader)

Grafica la señal "cerebral" DUMMY que BCI2000 ya grabó de verdad en el .dat
(generada por el SignalGenerator, con la amplitud modulada en vivo según
la presión de cada trial), junto con la fase del trial (StimulusPhase) y
la presión aplicada (PressureTarget).

NOTA: usamos BCI2kReader en vez de MNE porque MNE solo mapea
automáticamente el estado 'StimulusCode' a un canal de eventos; los
estados personalizados (StimulusPhase, PressureTarget, PainLevel,
TrialNumber) no los expone MNE como canales. BCI2kReader sí da acceso
directo a TODOS los estados como diccionario.

Requiere:
    pip install BCI2kReader matplotlib numpy --break-system-packages

Uso:
    python plot_recorded_signal.py "C:\\ruta\\a\\P018S004R05.dat"
    python plot_recorded_signal.py archivo.dat --channels 1 5 10
    python plot_recorded_signal.py archivo.dat --start 0 --end 60
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from BCI2kReader import BCI2kReader as b2k

PHASE_LABELS = {0: "NONE", 1: "INFLATING", 2: "HOLDING", 3: "DEFLATING", 4: "DONE"}


def main():
    parser = argparse.ArgumentParser(description="Grafica la señal dummy ya grabada en el .dat")
    parser.add_argument("dat_file", help="Ruta al archivo .dat")
    parser.add_argument("--channels", type=int, nargs="+", default=[1, 6, 11],
                         help="Índices de canales de señal a graficar (1-16). Default: 1 6 11")
    parser.add_argument("--start", type=float, default=None, help="Segundo inicial a mostrar")
    parser.add_argument("--end", type=float, default=None, help="Segundo final a mostrar")
    args = parser.parse_args()

    print(f"Cargando {args.dat_file} ...")
    with b2k.BCI2kReader(args.dat_file) as reader:
        sfreq = reader.samplingrate
        signals = reader.signals   # shape: (n_channels, n_samples)
        states = reader.states     # dict: {state_name: array}

    print(f"Frecuencia de muestreo: {sfreq} Hz")
    print(f"Canales de señal: {signals.shape[0]}")
    print(f"Estados disponibles: {list(states.keys())}")

    n_samples = signals.shape[1]
    times = np.arange(n_samples) / sfreq

    pressure = states["PressureTarget"].flatten()
    phase = states["StimulusPhase"].flatten()

    # Recortar rango de tiempo si se pidió
    if args.start is not None or args.end is not None:
        start = args.start or 0
        end = args.end or times[-1]
        mask = (times >= start) & (times <= end)
    else:
        mask = np.ones_like(times, dtype=bool)

    n_ch = len(args.channels)
    fig, ax = plt.subplots(n_ch + 2, 1, figsize=(13, 2.2 * (n_ch + 2)), sharex=True)

    # Presión
    ax[0].plot(times[mask], pressure[mask], color="tab:red")
    ax[0].set_ylabel("Presión\n(mmHg)")

    # Fase
    ax[1].step(times[mask], phase[mask], color="tab:green", where="post")
    ax[1].set_yticks(list(PHASE_LABELS.keys()))
    ax[1].set_yticklabels(list(PHASE_LABELS.values()))
    ax[1].set_ylabel("Fase")

    # Canales de señal dummy real (signals viene indexado desde 0)
    for i, ch_num in enumerate(args.channels):
        signal = signals[ch_num - 1]
        ax[2 + i].plot(times[mask], signal[mask], color="tab:blue", linewidth=0.5)
        ax[2 + i].set_ylabel(f"Ch{ch_num}\n(µV)")

    ax[-1].set_xlabel("Tiempo (s)")
    fig.suptitle("Señal cerebral dummy grabada (SignalGenerator) vs. fase y presión", fontsize=12)
    plt.tight_layout()

    out_path = args.dat_file.rsplit(".", 1)[0] + "_plot.png"
    plt.savefig(out_path, dpi=150)
    print(f"\nGráfica guardada en: {out_path}")

    plt.show()


if __name__ == "__main__":
    main()