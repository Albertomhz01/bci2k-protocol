// =====================================================================
// PRESSURE CONTROLLER  --  MPX5050DP + L298N + 3-way solenoid
//
// Serial protocol (115200 baud), contract with Python:
//
//   RECEIVES:
//     START,<mmHg>   inflate to that pressure
//     STOP           emergency stop (pump off, vent)
//     ZERO           recalibrate sensor zero (cuff with NO pressure)
//     PING           replies READY (link check)
//
//   SENDS:
//     READY                     on boot and on PING
//     ZERO,OK,<offset>          zero measured and applied, in mmHg
//     ZERO,SKIPPED,<reading>    pressure present at boot, zero skipped
//     INFLATING                 START accepted
//     HOLDING                   target pressure reached
//     DEFLATING                 HOLD finished
//     DONE                      sensor below DEFLATE_DONE_MMHG (3 mmHg)
//     STOPPED                   STOP confirmed
//     FAULT,<reason>            error, already vented for safety
//     P,<actual>,<target>       telemetry, ~20 Hz, always, in mmHg
// =====================================================================

const int enA       = 9;
const int In1       = 8;
const int In2       = 7;
const int solenoide = 2;
const int MPX       = A4;

// --- Sensor calibration ---
const float V_OFFSET    = 0.190;   // volts at 0 kPa
const float SENSITIVITY = 0.085;   // volts per kPa
const float KPA_TO_MMHG = 7.50062;

// --- Filter ---
const float ALPHA   = 0.15;
const int   SAMPLES = 32;          // fewer samples = faster loop

// --- Actuators ---
const int SOLENOID_CLOSED = HIGH;  // holds pressure in the cuff
const int SOLENOID_VENT   = LOW;   // opens the vent path
const int PUMP_PWM        = 170;
const int PUMP_OFF        = 0;

// --- Timing ---
const unsigned long HOLD_DURATION  = 10000UL;  // must match HOLD_SECONDS in Python
const unsigned long TELEMETRY_MS   = 50UL;     // 20 Hz

// Only watchdog: max time with the pump ON. Last physical safety in case a
// hose disconnects or the sensor fails reading low. Should never trigger in
// a normal trial (~15 s to reach 70 mmHg).
const unsigned long MAX_INFLATE_MS = 60000UL;

// --- Safety ---
const float MAX_SAFE_MMHG     = 200.0;  // above this, vent immediately

// Only end-of-deflation criterion: cuff is empty when the sensor reads below
// this. No timer; vents until reached.
const float DEFLATE_DONE_MMHG = 3.0;    // mmHg

// --- Auto zero ---
// The real sensor offset differs from V_OFFSET. On boot (cuff vented) the
// "zero" reading is measured and subtracted; otherwise pressure may never
// drop below the DONE threshold.
const float ZERO_MAX_ACCEPT_MMHG = 20.0;  // skip auto zero if boot reading exceeds this

// --- Electrical spike rejection ---
// Solenoid/L298N switching can spike the ADC. Cuff pressure changes slowly
// (~5 mmHg/s), so a jump > MAX_SLEW_MMHG between consecutive readings is noise
// and is discarded. If the jump persists for MAX_CONSECUTIVE_GLITCH readings,
// it is accepted as a real change.
const float MAX_SLEW_MMHG          = 40.0;
const int   MAX_CONSECUTIVE_GLITCH = 25;

// Overpressure requires several consecutive readings (real overpressure
// persists; an electrical spike lasts one loop).
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
  Serial.setTimeout(20);   // readStringUntil must not block the loop

  pinMode(enA,       OUTPUT);
  pinMode(In1,       OUTPUT);
  pinMode(In2,       OUTPUT);
  pinMode(solenoide, OUTPUT);

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);

  vent();

  currentState = IDLE;
  targetMmhg   = 0.0;

  zeroSensor();   // cuff must have NO pressure when USB is connected

  Serial.println("READY");
}

// Raw reading: no zero correction, no filtering. Can be negative (needed to
// measure the real offset).
float readRawMmhg()
{
  long sum = 0;

  for (int i = 0; i < SAMPLES; i++)
    sum += analogRead(MPX);

  float adc     = sum / (float) SAMPLES;
  float voltage = 5.0 * adc / 1023.0;

  float kpa = (voltage - V_OFFSET) / SENSITIVITY;

  return kpa * KPA_TO_MMHG;   // the whole sketch works in mmHg
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

    // Isolated spike: discard reading, keep previous value.
    if (glitchCount < MAX_CONSECUTIVE_GLITCH)
      return filteredMmhg;

    // Persistent change: accept new value and reseed the filter.
    glitchCount  = 0;
    filteredMmhg = mmhg;

    return filteredMmhg;
  }

  glitchCount = 0;

  filteredMmhg = ALPHA * mmhg + (1.0 - ALPHA) * filteredMmhg;

  return filteredMmhg;
}

// Measures sensor zero with the cuff vented. Called on boot and on ZERO.
// Skipped if real pressure is present.
void zeroSensor()
{
  vent();
  delay(300);   // let it settle before measuring

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
    Serial.println(avg, 1);   // real pressure present, or sensor miswired
  }
  else
  {
    zeroOffsetMmhg = avg;

    Serial.print("ZERO,OK,");
    Serial.println(avg, 1);   // measured offset
  }

  emaSeeded    = false;   // reseed filter with corrected scale
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

    targetMmhg = requested;   // already in mmHg

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

  // --- Safety, above any state -----------------------------------------
  // Overpressure is only checked in PUMPING and HOLDING. During DEFLATING the
  // pump is off and the valve is open, so a high reading is electrical noise.
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
        pumpOff();   // solenoid stays closed: cuff holds pressure

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

      // Only end criterion: sensor below DEFLATE_DONE_MMHG. No timeout.
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
