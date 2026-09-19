from BCI2000Remote import BCI2000Remote
import os
import time

# --- Rutas ------------------------------------------------------------------
# Se resuelven a partir de la carpeta donde vive ESTE archivo, no de un
# usuario de Windows escrito a mano. Así el mismo código corre en la máquina
# del laboratorio y en la tuya sin editar nada.
#
# Estructura que se asume (la misma que ya tienes):
#
#   Experiment-Chihiro/
#     bci_setup_cruz_con_ruta.py     <- este archivo
#     experiment_controller_cruz.py
#     dat_bci/                       <- se crea sola si no existe
#     Parametros/
#       Parametros.prm
#
# Si BCI2000 vive en otro lado, exporta la variable de entorno BCI2000_ROOT
# o edita BCI2000_ROOT_DEFAULT abajo.

HERE = os.path.dirname(os.path.abspath(__file__))

DATA_DIRECTORY = os.path.join(HERE, "dat_bci")
PARAMETER_FILE = os.path.join(HERE, "Parametros", "Parametros.prm")

BCI2000_ROOT_DEFAULT = r"C:\Users\L03579841\Desktop\bcipy\BCI2000\BCI2000 v3.6.beta.R7385\BCI2000.x64"
BCI2000_ROOT  = os.environ.get("BCI2000_ROOT", BCI2000_ROOT_DEFAULT)
OPERATOR_PATH = os.path.join(BCI2000_ROOT, "prog", "Operator.exe")

# Si es True, se fuerzan WindowLeft/WindowTop y NumberOfSequences DESPUÉS
# de cargar el .prm. Ponlo en False si prefieres que esos valores vengan
# del propio archivo de parámetros.
FORZAR_AJUSTES_DE_VENTANA = True


def setup_bci(participant_data=None):
    """
    participant_data: dict opcional con 'participant_number', 'session_number',
    'run_number' (lo que regresa launch_interface()). Si se pasa, el .dat se
    nombra y organiza según el participante/sesión/run.

    NOTA: este archivo ya NO lanza ninguna ventana de cruz de fijación —
    eso lo maneja por completo la clase FixationCross dentro de
    experiment_controller_cruz.py.
    """
    # --- Revisar las rutas ANTES de abrir nada -----------------------------
    # Un .prm que no existe o un Operator.exe mal ubicado son los dos errores
    # que más tiempo cuestan, porque BCI2000 falla varios pasos después y el
    # mensaje no dice nada útil. Mejor reventar aquí, con el path exacto.
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

    # Reintentar la conexión varias veces (el Operator tarda en abrir el puerto)
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

    # --- 1) Cargar TU diseño de parámetros ----------------------------------
    # Va primero, porque el .prm sobrescribe cualquier parámetro que ya esté
    # cargado. Aquí es donde entran los nombres de los canales (ChannelNames),
    # SamplingRate, SourceCh, filtros, etc.
    print(f"Cargando parámetros desde:\n  {PARAMETER_FILE}")
    params_result = bci.LoadParametersRemote(PARAMETER_FILE)
    print(f"LoadParametersRemote() returned: {params_result}")

    if not params_result:
        print(f"LOAD PARAMETERS FAILED. Last error: {bci.Result}")
        return None, None

    print("Parámetros cargados!")

    # --- 2) Ajustes que NO quieres que dependan del .prm ---------------------
    if FORZAR_AJUSTES_DE_VENTANA:
        # Mandar la ventana de CursorTask fuera de pantalla
        bci.Execute("SET PARAMETER WindowLeft 99999")
        bci.Execute("SET PARAMETER WindowTop 99999")

        # Evitar que CursorTask se autodetenga a medio experimento por su
        # propio límite interno de secuencias
        bci.Execute("SET PARAMETER NumberOfSequences 99999")

    # --- 3) Dónde y cómo se va a guardar el .dat -----------------------------
    # Va después del .prm para que el archivo de parámetros no pise la ruta
    # ni el nombre del sujeto.
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

        # --- Pre-flight: ¿ya existe ese archivo? ---------------------------
        # BCI2000 NUNCA sobrescribe un .dat. Si el archivo ya existe, al
        # recibir Start() busca el primer número de run libre y guarda ahí
        # SIN avisar. Por eso te aparecieron R02 y R03 aunque el Operator
        # reportaba "Run #1" las tres veces en el .applog.
        #
        # Preferimos reventar aquí, con el nombre exacto, a que se grabe una
        # hora de EEG en un archivo que no es el que esperabas.
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
