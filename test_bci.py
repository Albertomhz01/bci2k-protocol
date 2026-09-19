from bci_setup_cruz_con_ruta import setup_bci
import time

print("Calling setup_bci()...")
bci = setup_bci()
print(f"setup_bci() returned: {bci}")

if bci is None:
    print("setup_bci() failed — revisa los mensajes de error arriba.")
else:
    # --- DIAGNÓSTICO: verificar parámetros del SignalGenerator ---
    # bci.Execute() regresa True/False (éxito), el valor real queda en bci.Result
    print("\n--- Checking SignalGenerator parameters ---")

    def check_param(name):
        ok = bci.Execute(f"GET PARAMETER {name}")
        print(f"{name}: ok={ok} | result={bci.Result!r}")

    check_param("SineAmplitude")
    check_param("SineFrequency")
    check_param("NoiseAmplitude")
    check_param("SourceCh")
    check_param("SignalType")
    print("--- End diagnostic ---\n")

    print("Calling bci.Start()...")
    bci.Start()
    print("bci.Start() succeeded!")

    time.sleep(5)
    print("Trying SetStateVariable...")
    bci.SetStateVariable("StimulusPhase", 1)
    print("SetStateVariable succeeded!")

    time.sleep(20)
    bci.Stop()