from BCI2000Remote import BCI2000Remote
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))

DATA_DIRECTORY = os.path.join(HERE, "dat_bci")
PARAMETER_FILE = os.path.join(HERE, "Parametros", "Parametros.prm")

BCI2000_ROOT_DEFAULT = r"C:\Users\L03579841\Desktop\bcipy\BCI2000\BCI2000 v3.6.beta.R7385\BCI2000.x64" # <- change to your BCI2000 location
BCI2000_ROOT  = os.environ.get("BCI2000_ROOT", BCI2000_ROOT_DEFAULT)
OPERATOR_PATH = os.path.join(BCI2000_ROOT, "prog", "Operator.exe")

# If True, WindowLeft/WindowTop and NumberOfSequences are forced after
# loading the .prm. Set to False to take them from the .prm instead.
FORZAR_AJUSTES_DE_VENTANA = True


def setup_bci(participant_data=None):
    """Connects to BCI2000, loads parameters and configures the .dat output.

    participant_data: optional dict from launch_interface() with
    'participant_number', 'session_number', 'run_number'; used to name the .dat.
    The fixation cross window is handled by FixationCross in
    experiment_controller_cruz.py.

    Returns (bci, expected_dat_path), or (None, None) on failure.
    """
    # --- Check paths before opening anything ---
    # A missing .prm or Operator.exe makes BCI2000 fail later with unclear errors.
    if not os.path.isfile(OPERATOR_PATH):
        print(f"No encuentro Operator.exe en:\n  {OPERATOR_PATH}")
        print("Ajusta BCI2000_ROOT_DEFAULT o define la variable de entorno BCI2000_ROOT.")
        return None, None

    if not os.path.isfile(PARAMETER_FILE):
        print(f"No encuentro el archivo de parámetros en:\n  {PARAMETER_FILE}")
        return None, None

    os.makedirs(DATA_DIRECTORY, exist_ok=True)

    bci = BCI2000Remote()

    bci.Quiet = False
    bci.TerminateOnError = False

    bci.OperatorPath = OPERATOR_PATH

    bci.WindowVisible = True

    print("Connecting to BCI2000...")

    # Retry connection (Operator takes a while to open its port)
    connect_result = False
    for attempt in range(8):
        connect_result = bci.Connect()
        if connect_result:
            break
        print(f"  Attempt {attempt + 1}/8 failed, retrying in 2s...")
        time.sleep(2)

    print(f"Connect() returned: {connect_result}")

    if not connect_result:
        print(f"CONNECT FAILED. Last error: {bci.Result}")
        return None, None

    print("Connected!")

    startup_result = bci.StartupModules(
    (
        "gHIampSource", #SignalGenerator #gHIampSource
        "DummySignalProcessing",
        "DummyApplication",
    )
)
    print(f"StartupModules() returned: {startup_result}")

    if not startup_result:
        print(f"STARTUP FAILED. Last error: {bci.Result}")
        return None, None

    print("Modules started!")

    bci.Execute("SET SYSTEM STATE Idle")

    bci.AddStateVariable("StimulusPhase", 8, 0)
    bci.AddStateVariable("PainLevel", 8, 0)
    bci.AddStateVariable("PressureTarget", 16, 0)
    bci.AddStateVariable("TrialNumber", 16, 0)

    # --- 1) Load parameter file ---
    # Loaded first because the .prm overwrites existing parameters
    # (ChannelNames, SamplingRate, SourceCh, filters, etc.).
    print(f"Cargando parámetros desde:\n  {PARAMETER_FILE}")
    params_result = bci.LoadParametersRemote(PARAMETER_FILE)
    print(f"LoadParametersRemote() returned: {params_result}")

    if not params_result:
        print(f"LOAD PARAMETERS FAILED. Last error: {bci.Result}")
        return None, None

    print("Parámetros cargados!")

    # --- 2) Settings that should not depend on the .prm ---
    if FORZAR_AJUSTES_DE_VENTANA:
        # Move the CursorTask window off screen
        bci.Execute("SET PARAMETER WindowLeft 99999")
        bci.Execute("SET PARAMETER WindowTop 99999")

        # Prevent CursorTask from stopping mid-experiment due to its sequence limit
        bci.Execute("SET PARAMETER NumberOfSequences 99999")

    # --- 3) .dat output location and name ---
    # Set after the .prm so it doesn't overwrite the path or subject name.
    bci.Execute(f'SET PARAMETER DataDirectory "{DATA_DIRECTORY}"')

    if participant_data:
        subject_name = f"P{str(participant_data['participant_number']).zfill(3)}"
        session_num  = str(participant_data['session_number']).zfill(3)
        run_num      = str(participant_data['run_number']).zfill(2)
        bci.Execute(f"SET PARAMETER SubjectName {subject_name}")
        bci.Execute(f"SET PARAMETER SubjectSession {session_num}")
        bci.Execute(f"SET PARAMETER SubjectRun {run_num}")
        expected_path = os.path.join(
            DATA_DIRECTORY,
            subject_name,
            f"{subject_name}S{session_num}R{run_num}.dat",
        )

        # --- Pre-flight: does the file already exist? ---
        # BCI2000 never overwrites a .dat; it silently saves to the next free
        # run number. Abort here so the recording goes to the expected file.
        if os.path.isfile(expected_path):
            print("\n" + "="*60)
            print("  YA EXISTE ese archivo:")
            print(f"    {expected_path}")
            print()
            print("  BCI2000 no lo va a sobrescribir: guardaría en el siguiente")
            print("  run libre sin decirte nada. Muévelo o bórralo antes de correr.")
            print("="*60)
            return None, None
    else:
        expected_path = os.path.join(
            DATA_DIRECTORY, "<SubjectName>", "<archivo>.dat"
        ) + "  (revisa SubjectName/Session/Run por defecto)"

    print(f"El .dat de esta corrida se guardará en:\n  {expected_path}")
    # --------------------------------------------------------------------------

    config_result = bci.SetConfig()
    print(f"SetConfig() returned: {config_result}")

    if not config_result:
        print(f"SETCONFIG FAILED. Last error: {bci.Result}")
        return None, None

    print("Config set!")

    return bci, expected_path