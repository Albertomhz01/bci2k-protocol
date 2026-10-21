// =====================================================================
// PRESSURE CONTROLLER  --  MPX5050DP + L298N + 3-way solenoid
//
// Serial protocol (115200 baud), contract with Python:
//
//   RECEIVES:
//     TRIAL,<id>,<mmHg>,<ramp_ms>,<failsafe_ms>
//                    Fixed-window trial (used by experiment_controller_cruz.py).
//                    Inflate to <mmHg>, then keep it there (valve closed, short
//                    top-ups, never above the target) until Python sends
//                    DEFLATE. The Arduino does NOT decide when the
//                    experimental phases end; Python/BCI2000 does.
//                      ramp_ms == 0  -> full speed (PUMP_PWM) until target
//                                       (HIGH and calibration)
//                      ramp_ms  > 0  -> follow the reference trajectory
//                                       desired(t) = p0 + (target-p0)*t/ramp_ms
//                                       (LOW and NONE)
//                      failsafe_ms   -> vent on our own if DEFLATE has not
//                                       arrived this many ms after TRIAL
//                                       (Python crashed / link lost).
//                    A repeated TRIAL with the SAME id while that trial is
//                    active is only re-acknowledged (never restarts the ramp).
//     DEFLATE,<id>   open the vent now (end of HOLDING, decided by Python)
//     START,<mmHg>   LEGACY self-timed cycle (inflate, 10 s hold, deflate).
//                    Kept unchanged for bench tests.
//     STOP           emergency stop (pump off, vent)
//     ZERO           recalibrate sensor zero (cuff with NO pressure)
//     PING           replies READY (link check)
//
//   SENDS  (physical events: they describe the cuff, NOT the experiment):
//     READY                                  on boot and on PING
//     ZERO,OK,<offset>                       zero measured and applied, in mmHg
//     ZERO,SKIPPED,<reading>                 pressure present at boot, zero skipped
//     INFLATING,<id>,<ms>                    TRIAL accepted; <ms> = millis() at
//                                            receipt (clock reference for this trial)
//     PUMP_ON,<id>,<t>                       first time the pump ran in this trial
//     TARGET_REACHED,<id>,<t>,<mmHg>         target first reached; from here the
//                                            pressure is maintained (see below)
//     DEFLATING,<id>,<t>                     DEFLATE executed, valve open
//     DEFLATED,<id>,<t>,<mmHg>,<td>          sensor below DEFLATE_DONE_MMHG
//                                            (<td> = ms since DEFLATE)
//        <t> is always ms since this trial's TRIAL command was received.
//     INFLATING / HOLDING / DEFLATING / DONE legacy START cycle only
//     STOPPED                                STOP confirmed
//     FAULT,<reason>                         error, already vented for safety
//     P,<actual>,<target>,<desired>,<pwm>,<ms>
//                                            telemetry, ~20 Hz, always.
//                                            <ms> = millis(). The first two
//                                            fields are unchanged, so older
//                                            readers still work.
// =====================================================================

const int enA       = 9;     // L298N ENA -> PWM pin (analogWrite, ~490 Hz)
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
const int PUMP_PWM        = 255;   // full speed used by HIGH (unchanged). Also
                                   // the MAXIMUM PWM the ramp may ever use.
const int PUMP_OFF        = 0;

// --- Ramp controller (LOW / NONE) ---
// Proportional control with minimum/maximum PWM around the reference
// trajectory. The cuff only loses pressure through the vent, so with the
// valve closed the plant is close to an integrator: pumping raises
// pressure, stopping holds it. That is why a P-with-floor law is enough
// and no integral/derivative term is used.
//
//   err = desired - actual
//   err <= 0                -> pump OFF (at/above the trajectory: wait for it)
//   0 < err < RAMP_BAND     -> PWM between RAMP_PWM_MIN and PUMP_PWM
//   err >= RAMP_BAND        -> PWM = PUMP_PWM (falling behind: full speed)
//   actual >= target        -> TARGET_REACHED; the hold/top-up logic below
//                              takes over until DEFLATE
//
// RAMP_BAND_MMHG is a CONTROL band around the moving reference, not an
// acceptance window for the final pressure: the trial still stops only when
// the sensor reads >= target, exactly like HIGH.
//
// RAMP_PWM_MIN must be at or above the PWM at which the pump actually turns.
// If it is set too low the error simply grows and the P term raises the PWM
// until the pump moves, so a wrong guess makes tracking coarser but never
// unsafe. BENCH-TUNE THIS on the real pump (see README of the change).
const int   RAMP_PWM_MIN    = 100;
const float RAMP_BAND_MMHG  = 5.0;
const float RAMP_RESTART_MMHG = 0.5;  // hysteresis: restart pump only when this
                                      // far below the trajectory (no chatter)

// --- Pressure maintenance after the target (TRIAL only) ---
// Once the target is reached the cuff slowly loses pressure (tissue/cuff
// creep, small leaks: 111 -> ~96 mmHg over 22 s on the rig). From
// TARGET_REACHED until Python sends DEFLATE (the rest of INFLATING and all
// of HOLDING) the pump tops the cuff up:
//   pressure <  target - HOLD_TOPUP_MMHG  -> pump on (PUMP_PWM)
//   pressure >= target                    -> pump off
// So the pressure stays between target - 2 and target (+ <1 mmHg of sensor
// lag), well inside the +-5 mmHg limit. It never pumps above the target.
// A top-up that cannot restore the target within HOLD_TOPUP_MAX_MS means a
// real leak (hose off, valve open): vent and FAULT,HOLD_LEAK.
const float         HOLD_TOPUP_MMHG   = 2.0;
const unsigned long HOLD_TOPUP_MAX_MS = 5000UL;

// --- Timing ---
const unsigned long HOLD_DURATION  = 10000UL;  // legacy START only
const unsigned long TELEMETRY_MS   = 50UL;     // 20 Hz

// Max time with the pump ON. Last physical safety in case a hose
// disconnects or the sensor fails reading low. Applies to START and TRIAL.
const unsigned long MAX_INFLATE_MS = 60000UL;

// Fail-safe for TRIAL: the cuff is vented if DEFLATE has not arrived within
// failsafe_ms (sent by Python) of the TRIAL command. Clamped to this ceiling.
// This replaces the protection the old self-timed 10 s HOLD gave when Python
// dies: pressure can never be retained indefinitely.
const unsigned long MAX_TRIAL_MS   = 90000UL;

// --- Safety ---
const float MAX_SAFE_MMHG     = 200.0;  // above this, vent immediately

// End-of-deflation criterion: cuff is empty when the sensor reads below
// this. No timer; vents until reached.
const float DEFLATE_DONE_MMHG = 3.0;    // mmHg

// --- Auto zero ---
const float ZERO_MAX_ACCEPT_MMHG = 20.0;  // skip auto zero if boot reading exceeds this

// --- Electrical spike rejection ---
const float MAX_SLEW_MMHG          = 40.0;
const int   MAX_CONSECUTIVE_GLITCH = 25;

// Overpressure requires several consecutive readings.
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
  PUMPING,     // legacy START: full speed
  HOLDING,     // legacy START: self-timed hold
  DEFLATING,   // legacy START and TRIAL
  T_INFLATE,   // TRIAL: full speed or ramp, until target
  T_HOLD       // TRIAL: target reached, valve closed, waiting for DEFLATE
};

State currentState = IDLE;

unsigned long phaseStart     = 0;
unsigned long lastTelemetry  = 0;

// --- TRIAL bookkeeping ---
bool          trialMode      = false;  // true = TRIAL protocol, false = legacy START
long          trialId        = -1;
unsigned long trialStart     = 0;      // millis() when TRIAL was received
unsigned long rampMs         = 0;      // 0 = full speed
unsigned long failsafeMs     = MAX_TRIAL_MS;
unsigned long deflateStart   = 0;
float         rampStartMmhg  = 0.0;
float         desiredMmhg    = 0.0;    // reference trajectory (telemetry)
int           pumpPwm        = 0;      // PWM actually written (telemetry)
bool          pumpHasRun     = false;
bool          topupActive    = false;  // maintenance pump burst running
unsigned long topupStart     = 0;

// Prototypes (the Arduino IDE generates these; plain avr-g++ needs them)
void  zeroSensor();
float readPressure();
void  setPump(int pwm);

// ---------------------------------------------------------------------

void setPump(int pwm)
{
  if (pwm < 0)        pwm = 0;
  if (pwm > PUMP_PWM) pwm = PUMP_PWM;   // never faster than the HIGH speed

  pumpPwm = pwm;
  analogWrite(enA, pwm);
}

void pumpOff()
{
  setPump(PUMP_OFF);
}

void vent()
{
  setPump(PUMP_OFF);
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
  unsigned long now = millis();

  if (now - lastTelemetry < TELEMETRY_MS)
    return;

  lastTelemetry = now;

  Serial.print("P,");
  Serial.print(pressure, 1);
  Serial.print(",");
  Serial.print(targetMmhg, 1);
  Serial.print(",");
  Serial.print(desiredMmhg, 1);
  Serial.print(",");
  Serial.print(pumpPwm);
  Serial.print(",");
  Serial.println(now);
}

void fault(const char *reason)
{
  vent();

  currentState      = IDLE;
  trialMode         = false;
  topupActive       = false;
  targetMmhg        = 0.0;
  desiredMmhg       = 0.0;
  glitchCount       = 0;
  overpressureCount = 0;

  Serial.print("FAULT,");
  Serial.println(reason);
}

// "<event>,<id>,<t>" with t = ms since TRIAL receipt
void sendTrialEvent(const char *name, unsigned long t)
{
  Serial.print(name);
  Serial.print(",");
  Serial.print(trialId);
  Serial.print(",");
  Serial.println(t);
}

// Splits "A,B,C,..." (after the command word) into up to n float fields.
// Returns how many fields were found.
int parseFields(const String &cmd, float *out, int n)
{
  int count = 0;
  int pos   = cmd.indexOf(',');

  while (pos >= 0 && count < n)
  {
    int next = cmd.indexOf(',', pos + 1);
    String field = (next < 0) ? cmd.substring(pos + 1) : cmd.substring(pos + 1, next);
    out[count++] = field.toFloat();
    pos = next;
  }

  return count;
}

void startTrial(const String &cmd, float pressureNow)
{
  float f[4];

  if (parseFields(cmd, f, 4) < 4)
  {
    fault("BAD_TRIAL");
    return;
  }

  long  id        = (long) f[0];
  float requested = f[1];
  long  ramp      = (long) f[2];
  long  failsafe  = (long) f[3];

  // Duplicate of the trial already running (Python re-sent because the ack
  // was lost): acknowledge again, never restart the ramp or the timers.
  if (trialMode && id == trialId &&
      (currentState == T_INFLATE || currentState == T_HOLD))
  {
    Serial.print("INFLATING,");
    Serial.print(trialId);
    Serial.print(",");
    Serial.println(trialStart);
    return;
  }

  if (requested <= 0.0 || requested > MAX_SAFE_MMHG)
  {
    fault("TARGET_OUT_OF_RANGE");
    return;
  }

  if (ramp < 0 || (unsigned long) ramp > MAX_INFLATE_MS)
  {
    fault("RAMP_OUT_OF_RANGE");
    return;
  }

  trialMode     = true;
  trialId       = id;
  targetMmhg    = requested;
  rampMs        = (unsigned long) ramp;
  failsafeMs    = (failsafe <= 0 || (unsigned long) failsafe > MAX_TRIAL_MS)
                  ? MAX_TRIAL_MS : (unsigned long) failsafe;
  rampStartMmhg = (pressureNow < requested) ? pressureNow : requested;
  pumpHasRun    = false;
  topupActive   = false;

  digitalWrite(solenoide, SOLENOID_CLOSED);

  trialStart   = millis();
  phaseStart   = trialStart;
  currentState = T_INFLATE;

  glitchCount       = 0;
  overpressureCount = 0;

  Serial.print("INFLATING,");
  Serial.print(trialId);
  Serial.print(",");
  Serial.println(trialStart);

  // Full-speed trials start the pump right here, in the same loop pass as the
  // ack, so "INFLATING" and pump onset are the same instant (as before).
  if (rampMs == 0 && pressureNow < targetMmhg)
  {
    setPump(PUMP_PWM);
    pumpHasRun = true;
    sendTrialEvent("PUMP_ON", 0);
  }
}

void startDeflate()
{
  // Always vents, whatever the state: a DEFLATE must never be ignored.
  unsigned long now = millis();

  vent();

  deflateStart = now;
  topupActive  = false;
  phaseStart   = now;
  currentState = DEFLATING;
  desiredMmhg  = 0.0;

  if (trialMode)
    sendTrialEvent("DEFLATING", now - trialStart);
  else
    Serial.println("DEFLATING");
}

void checkSerial()
{
  if (!Serial.available())
    return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("TRIAL"))
  {
    startTrial(cmd, filteredMmhg);
  }

  else if (cmd.startsWith("DEFLATE"))
  {
    startDeflate();
  }

  else if (cmd.startsWith("START"))
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

    trialMode   = false;
    targetMmhg  = requested;   // already in mmHg
    desiredMmhg = requested;

    digitalWrite(solenoide, SOLENOID_CLOSED);
    setPump(PUMP_PWM);

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
    trialMode    = false;
    topupActive  = false;
    targetMmhg   = 0.0;
    desiredMmhg  = 0.0;

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

// One control step of a TRIAL inflation (full speed or ramp).
void trialInflateStep(float pressure, unsigned long now)
{
  unsigned long t = now - trialStart;

  // Physical target reached: latch the pump off. Never keep pumping above the
  // target to "fill" the INFLATING window; Python keeps the experimental
  // state as INFLATING until its own deadline.
  if (pressure >= targetMmhg)
  {
    pumpOff();

    desiredMmhg  = targetMmhg;
    currentState = T_HOLD;
    phaseStart   = now;

    Serial.print("TARGET_REACHED,");
    Serial.print(trialId);
    Serial.print(",");
    Serial.print(t);
    Serial.print(",");
    Serial.println(pressure, 1);
    return;
  }

  if (t > MAX_INFLATE_MS)
  {
    fault("INFLATE_TIMEOUT");
    return;
  }

  int pwm;

  if (rampMs == 0)
  {
    desiredMmhg = targetMmhg;
    pwm         = PUMP_PWM;
  }
  else
  {
    if (t >= rampMs)
      desiredMmhg = targetMmhg;
    else
      desiredMmhg = rampStartMmhg
                  + (targetMmhg - rampStartMmhg) * ((float) t / (float) rampMs);

    float err = desiredMmhg - pressure;

    // Hysteresis: once stopped at the trajectory, wait until we are
    // RAMP_RESTART_MMHG below it before running again.
    float onThreshold = (pumpPwm > 0) ? 0.0 : RAMP_RESTART_MMHG;

    if (err <= onThreshold)
      pwm = PUMP_OFF;
    else if (err >= RAMP_BAND_MMHG)
      pwm = PUMP_PWM;
    else
      pwm = RAMP_PWM_MIN + (int) ((PUMP_PWM - RAMP_PWM_MIN) * (err / RAMP_BAND_MMHG));
  }

  setPump(pwm);

  if (pwm > 0 && !pumpHasRun)
  {
    pumpHasRun = true;
    sendTrialEvent("PUMP_ON", t);
  }
}

void loop()
{
  checkSerial();

  float pressure = readPressure();
  unsigned long now = millis();

  sendTelemetry(pressure);

  // --- Safety, above any state -----------------------------------------
  // Overpressure is checked whenever the valve is closed. During DEFLATING
  // the pump is off and the valve is open, so a high reading is noise.
  if (currentState == PUMPING || currentState == HOLDING ||
      currentState == T_INFLATE || currentState == T_HOLD)
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

  // TRIAL fail-safe: Python must send DEFLATE in time. If it doesn't (crash,
  // cable, frozen PC) we vent on our own and report it.
  if ((currentState == T_INFLATE || currentState == T_HOLD) &&
      (now - trialStart >= failsafeMs))
  {
    fault("FAILSAFE_NO_DEFLATE");
    return;
  }

  switch (currentState)
  {
    // ---------------- TRIAL protocol ----------------
    case T_INFLATE:
      trialInflateStep(pressure, now);
      break;

    case T_HOLD:
      // Valve closed. Keep the pressure at the target until DEFLATE (or
      // the fail-safe above): short full-speed bursts when it drops more
      // than HOLD_TOPUP_MMHG below the target, off again at the target.
      if (topupActive)
      {
        if (pressure >= targetMmhg)
        {
          pumpOff();
          topupActive = false;
        }
        else if (now - topupStart > HOLD_TOPUP_MAX_MS)
        {
          fault("HOLD_LEAK");
          return;
        }
      }
      else if (pressure < targetMmhg - HOLD_TOPUP_MMHG)
      {
        setPump(PUMP_PWM);
        topupActive = true;
        topupStart  = now;
      }
      break;

    // ---------------- legacy START protocol ----------------
    case PUMPING:

      if (pressure >= targetMmhg)
      {
        pumpOff();   // solenoid stays closed: cuff holds pressure

        currentState = HOLDING;
        phaseStart   = now;

        Serial.println("HOLDING");
      }
      else if (now - phaseStart > MAX_INFLATE_MS)
      {
        fault("INFLATE_TIMEOUT");
      }

      break;

    case HOLDING:

      if (now - phaseStart >= HOLD_DURATION)
      {
        startDeflate();
      }

      break;

    // ---------------- shared ----------------
    case DEFLATING:

      // Only end criterion: sensor below DEFLATE_DONE_MMHG. No timeout; the
      // valve stays open whatever happens.
      if (pressure < DEFLATE_DONE_MMHG)
      {
        vent();

        currentState = IDLE;
        targetMmhg   = 0.0;
        desiredMmhg  = 0.0;

        if (trialMode)
        {
          Serial.print("DEFLATED,");
          Serial.print(trialId);
          Serial.print(",");
          Serial.print(now - trialStart);
          Serial.print(",");
          Serial.print(pressure, 1);
          Serial.print(",");
          Serial.println(now - deflateStart);
          trialMode = false;
        }
        else
        {
          Serial.println("DONE");
        }
      }

      break;

    case IDLE:
      break;
  }

  delay(5);
}
