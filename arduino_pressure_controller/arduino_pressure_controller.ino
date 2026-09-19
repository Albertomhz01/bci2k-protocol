// =====================================================================
// PRESSURE CONTROLLER  --  MPX5050DP + L298N + solenoide de 3 vias
//
// Protocolo serial (115200 baud), este es el contrato con Python:
//
//   RECIBE:
//     START,<mmHg>   inicia el inflado hasta esa presion
//     STOP           corte de emergencia (apaga bomba y ventea)
//     ZERO           recalibra el cero del sensor (manguito SIN presion)
//     PING           responde READY (para verificar el enlace)
//
//   ENVIA:
//     READY                     al arrancar y al recibir PING
//     ZERO,OK,<offset>          cero medido y aplicado, en mmHg
//     ZERO,SKIPPED,<lectura>    habia presion al arrancar, no se auto cero
//     INFLATING                 al aceptar el START
//     HOLDING                   al alcanzar la presion objetivo
//     DEFLATING                 al terminar el HOLD
//     DONE                      cuando el sensor baja de DEFLATE_DONE_MMHG
//                               (3 mmHg). Ese es el UNICO criterio de fin.
//     STOPPED                   confirmacion de un STOP
//     FAULT,<motivo>            algo salio mal, ya se venteo por seguridad
//     P,<actual>,<objetivo>     TELEMETRIA, ~20 Hz, SIEMPRE, en mmHg
//
// La linea "P,..." es la que alimenta la barra de progreso en Python.
// El 100% de la barra es <objetivo>, asi que si el trial pide 70 mmHg,
// 70 llena la barra, y si pide 30 mmHg, 30 la llena. No hay ningun
// temporizador involucrado: la barra ES la presion del sensor.
// =====================================================================

const int enA       = 9;
const int In1       = 8;
const int In2       = 7;
const int solenoide = 2;
const int MPX       = A4;

// --- Calibracion del sensor -----------------------------------------
const float V_OFFSET    = 0.190;   // volts a 0 kPa
const float SENSITIVITY = 0.085;   // volts por kPa
const float KPA_TO_MMHG = 7.50062;

// --- Filtro -----------------------------------------------------------
const float ALPHA   = 0.15;
const int   SAMPLES = 32;          // antes 128: bajarlo agiliza el lazo

// --- Actuadores -------------------------------------------------------
const int SOLENOID_CLOSED = HIGH;  // retiene la presion en el manguito
const int SOLENOID_VENT   = LOW;   // abre la via de escape
const int PUMP_PWM        = 170;
const int PUMP_OFF        = 0;

// --- Tiempos ----------------------------------------------------------
const unsigned long HOLD_DURATION  = 10000UL;  // debe coincidir con HOLD_SECONDS de Python
const unsigned long TELEMETRY_MS   = 50UL;     // 20 Hz

// Unico watchdog que queda: tiempo maximo con la BOMBA ENCENDIDA.
// No es un castigo por inflar lento. Es el ultimo seguro fisico: si se sale
// una manguera o el sensor se muere leyendo bajo, esto es lo unico que impide
// que la bomba siga metiendo aire al brazo del participante para siempre.
// A ~5 mmHg/s, 70 mmHg se alcanzan en ~15 s. 60 s es enorme y no deberia
// dispararse nunca en un trial sano.
const unsigned long MAX_INFLATE_MS = 60000UL;

// --- Seguridad --------------------------------------------------------
const float MAX_SAFE_MMHG     = 200.0;  // por encima de esto se ventea sin preguntar

// UNICO criterio de fin de desinflado. El manguito esta vacio cuando el
// MPX5050DP marca menos de esto. No hay detector de meseta, ni cronometro,
// ni nada mas: se ventea hasta llegar aqui, tarde lo que tarde.
const float DEFLATE_DONE_MMHG = 3.0;    // mmHg

// --- Auto cero --------------------------------------------------------
// El offset real del MPX5050DP no es exactamente V_OFFSET. Al arrancar,
// con el manguito abierto al ambiente, medimos lo que el sensor cree que
// es "cero" y lo restamos de aqui en adelante. Sin esto, un offset de unos
// pocos cuentas de ADC hace que la presion nunca baje del umbral de DONE.
const float ZERO_MAX_ACCEPT_MMHG = 20.0;  // si al arrancar lee mas que esto, no auto ceramos

// --- Rechazo de picos electricos --------------------------------------
// El solenoide y el L298N conmutan corriente justo al lado de una linea
// analogica. Un transitorio puede rielar el ADC por una lectura y hacer que
// el sensor "vea" cientos de mmHg que nunca existieron.
//
// La presion en un manguito es un sistema LENTO: sube a unos 5 mmHg/s y baja
// a unos 5 mmHg/s. Un salto de mas de MAX_SLEW_MMHG entre dos lecturas
// consecutivas (separadas ~10 ms) equivale a miles de mmHg/s. Eso no es aire,
// es ruido, y se descarta.
//
// Pero si el salto PERSISTE muchas lecturas seguidas, ya no es un transitorio:
// el sensor se desconecto o cambio de verdad. Entonces si se acepta, y el
// resto de las protecciones actuan sobre el valor nuevo.
const float MAX_SLEW_MMHG          = 40.0;
const int   MAX_CONSECUTIVE_GLITCH = 25;

// La sobrepresion tampoco se declara con UNA lectura. Se exigen varias
// seguidas, porque una sobrepresion real dura (la bomba sigue metiendo aire),
// mientras que un pico electrico dura una sola vuelta del lazo.
const int OVERPRESSURE_HITS = 20;

int glitchCount       = 0;
int overpressureCount = 0;

float zeroOffsetMmhg = 0.0;

float targetMmhg   = 0.0;
float filteredMmhg = 0.0;
bool  emaSeeded    = false;

enum State
{
  IDLE,
  PUMPING,
  HOLDING,
  DEFLATING
};

State currentState = IDLE;

unsigned long phaseStart     = 0;
unsigned long lastTelemetry  = 0;

// ---------------------------------------------------------------------

void pumpOff()
{
  analogWrite(enA, PUMP_OFF);
}

void vent()
{
  analogWrite(enA, PUMP_OFF);
  digitalWrite(solenoide, SOLENOID_VENT);
}

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(20);   // readStringUntil no debe bloquear el lazo

  pinMode(enA,       OUTPUT);
  pinMode(In1,       OUTPUT);
  pinMode(In2,       OUTPUT);
  pinMode(solenoide, OUTPUT);

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);

  vent();

  currentState = IDLE;
  targetMmhg   = 0.0;

  zeroSensor();   // el manguito debe estar SIN presion al conectar el USB

  Serial.println("READY");
}

// Lectura CRUDA, sin corregir el cero y sin filtrar. Puede salir negativa,
// y asi tiene que ser: es lo que nos permite medir el offset real.
float readRawMmhg()
{
  long sum = 0;

  for (int i = 0; i < SAMPLES; i++)
    sum += analogRead(MPX);

  float adc     = sum / (float) SAMPLES;
  float voltage = 5.0 * adc / 1023.0;

  float kpa = (voltage - V_OFFSET) / SENSITIVITY;

  return kpa * KPA_TO_MMHG;   // TODO el sketch trabaja en mmHg
}

float readPressure()
{
  float mmhg = readRawMmhg() - zeroOffsetMmhg;

  if (mmhg < 0.0)
    mmhg = 0.0;

  if (!emaSeeded)
  {
    filteredMmhg = mmhg;
    emaSeeded    = true;
    glitchCount  = 0;

    return filteredMmhg;
  }

  float jump = mmhg - filteredMmhg;

  if (jump < 0.0)
    jump = -jump;

  if (jump > MAX_SLEW_MMHG)
  {
    glitchCount++;

    // Pico aislado: se tira la lectura y se conserva la anterior. El aire no
    // puede moverse asi de rapido, asi que no perdemos informacion real.
    if (glitchCount < MAX_CONSECUTIVE_GLITCH)
      return filteredMmhg;

    // Ya no es un pico: el sensor lleva mucho rato diciendo otra cosa.
    // Se acepta el valor nuevo y se resiembra el filtro.
    glitchCount  = 0;
    filteredMmhg = mmhg;

    return filteredMmhg;
  }

  glitchCount = 0;

  filteredMmhg = ALPHA * mmhg + (1.0 - ALPHA) * filteredMmhg;

  return filteredMmhg;
}

// Mide el cero del sensor con el manguito ABIERTO al ambiente. Se llama al
// arrancar y cada vez que Python manda ZERO. Si en ese momento hay presion
// de verdad en el manguito, NO auto cera (se estaria comiendo presion real).
void zeroSensor()
{
  vent();
  delay(300);   // dejar que se estabilice antes de medir

  float sum = 0.0;

  for (int i = 0; i < 20; i++)
  {
    sum += readRawMmhg();
    delay(20);
  }

  float avg = sum / 20.0;

  if (avg > ZERO_MAX_ACCEPT_MMHG || avg < -ZERO_MAX_ACCEPT_MMHG)
  {
    zeroOffsetMmhg = 0.0;

    Serial.print("ZERO,SKIPPED,");
    Serial.println(avg, 1);   // hay presion real, o el sensor esta mal cableado
  }
  else
  {
    zeroOffsetMmhg = avg;

    Serial.print("ZERO,OK,");
    Serial.println(avg, 1);   // este es el offset que estaba rompiendo el DONE
  }

  emaSeeded    = false;   // el filtro se resiembra con la escala ya corregida
  filteredMmhg = 0.0;
}

void sendTelemetry(float pressure)
{
  if (millis() - lastTelemetry < TELEMETRY_MS)
    return;

  lastTelemetry = millis();

  Serial.print("P,");
  Serial.print(pressure, 1);
  Serial.print(",");
  Serial.println(targetMmhg, 1);
}

void fault(const char *reason)
{
  vent();

  currentState      = IDLE;
  targetMmhg        = 0.0;
  glitchCount       = 0;
  overpressureCount = 0;

  Serial.print("FAULT,");
  Serial.println(reason);
}

void checkSerial()
{
  if (!Serial.available())
    return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("START"))
  {
    int comma = cmd.indexOf(',');

    if (comma < 0)
    {
      fault("BAD_START");
      return;
    }

    float requested = cmd.substring(comma + 1).toFloat();

    if (requested <= 0.0 || requested > MAX_SAFE_MMHG)
    {
      fault("TARGET_OUT_OF_RANGE");
      return;
    }

    targetMmhg = requested;   // ya viene en mmHg, no se convierte nada

    digitalWrite(solenoide, SOLENOID_CLOSED);
    analogWrite(enA, PUMP_PWM);

    currentState = PUMPING;
    phaseStart   = millis();

    glitchCount       = 0;
    overpressureCount = 0;

    Serial.println("INFLATING");
  }

  else if (cmd == "STOP")
  {
    vent();

    currentState = IDLE;
    targetMmhg   = 0.0;

    Serial.println("STOPPED");
  }

  else if (cmd == "ZERO")
  {
    if (currentState != IDLE)
    {
      fault("ZERO_WHILE_BUSY");
      return;
    }

    zeroSensor();
  }

  else if (cmd == "PING")
  {
    Serial.println("READY");
  }
}

void loop()
{
  checkSerial();

  float pressure = readPressure();

  sendTelemetry(pressure);

  // --- Seguridad, por encima de cualquier estado -----------------------
  //
  // La sobrepresion SOLO se vigila cuando la bomba puede meter aire, o sea
  // en PUMPING y HOLDING. Durante DEFLATING la bomba esta apagada y la valvula
  // esta abierta al ambiente: ahi una lectura de 200 mmHg no puede ser fisica,
  // es ruido electrico por definicion. Vigilarla ahi solo abortaba trials
  // buenos (y eso es exactamente lo que paso en el Trial 1, a 45 mmHg y
  // BAJANDO).
  if (currentState == PUMPING || currentState == HOLDING)
  {
    if (pressure > MAX_SAFE_MMHG)
    {
      overpressureCount++;

      if (overpressureCount >= OVERPRESSURE_HITS)
      {
        fault("OVERPRESSURE");
        return;
      }
    }
    else
    {
      overpressureCount = 0;
    }
  }
  else
  {
    overpressureCount = 0;
  }

  switch (currentState)
  {
    case PUMPING:

      if (pressure >= targetMmhg)
      {
        pumpOff();   // el solenoide sigue cerrado: el manguito retiene

        currentState = HOLDING;
        phaseStart   = millis();

        Serial.println("HOLDING");
      }
      else if (millis() - phaseStart > MAX_INFLATE_MS)
      {
        fault("INFLATE_TIMEOUT");
      }

      break;

    case HOLDING:

      if (millis() - phaseStart >= HOLD_DURATION)
      {
        digitalWrite(solenoide, SOLENOID_VENT);

        currentState = DEFLATING;
        phaseStart   = millis();

        Serial.println("DEFLATING");
      }

      break;

    case DEFLATING:

      // UNICO criterio de fin: el sensor marca menos de DEFLATE_DONE_MMHG.
      // La valvula queda abierta al ambiente y la bomba apagada. Si tarda,
      // tarda: no hay cronometro, no hay detector de meseta, no hay falla.
      if (pressure < DEFLATE_DONE_MMHG)
      {
        vent();

        currentState = IDLE;
        targetMmhg   = 0.0;

        Serial.println("DONE");
      }

      break;

    case IDLE:
      break;
  }

  delay(5);
}
