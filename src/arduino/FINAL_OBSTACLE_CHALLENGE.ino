/*
  WRO 2025 – Obstacle Challenge (Arduino)
  ========================================
  Dual-processor architecture: Raspberry Pi 3B+ handles vision & high-level FSM;
  Arduino Uno handles real-time actuation & proximity safety at 50 Hz.

  Fixes applied vs original repo:
    [FIX-1] REQUIRE_HEARTBEAT = true  (watchdog now active, matches paper §3.2 & S5)
    [FIX-2] Steering law: δ = KP*(o + Fy) + KD*Fy_dot  (paper Eq.12, always fused)
            Previously used a winner-takes-all latch that discarded Fy when Pi active.
    [FIX-3] BASE_SPEED = 230 (matches paper Table, §5.5)
    [FIX-4] Removed undocumented EXIT_PARK / POST_EXIT_AVOID states; parking FSM
            now matches the five-state specification in paper §6 exactly.

  Protocol (Pi → Arduino, one-directional ASCII @ 115200 baud):
    GO              – acknowledge start after Arduino prints RDY
    MODE RACE       – enter normal lap-running mode
    MODE PARK       – trigger parking FSM
    DRIVE o s       – lane offset o∈[-1,1], speed hint s∈[0,255]
    KEEP LEFT       – 450 ms left-bias (green pillar avoidance)
    KEEP RIGHT      – 450 ms right-bias (red pillar avoidance)
    HB              – heartbeat (resets 400 ms RX watchdog)
    STOP            – immediate emergency stop
*/

#include <Servo.h>
#include <AFMotor.h>

// ─── Pin Map ──────────────────────────────────────────────────────────────────
#define TRIG_FC  13   // Front-Centre TRIG
#define ECHO_FC  A5   // Front-Centre ECHO
#define TRIG_FLD  3   // Front-Left Diagonal TRIG
#define ECHO_FLD A1
#define TRIG_FRD  5   // Front-Right Diagonal TRIG
#define ECHO_FRD A2
#define TRIG_L    6   // Left TRIG
#define ECHO_L   A3
#define TRIG_R   10   // Right TRIG
#define ECHO_R   A4
#define TRIG_B    2   // Rear TRIG
#define ECHO_B   A0
#define SERVO_PIN 9   // Steering servo

// ─── Hardware ─────────────────────────────────────────────────────────────────
AF_DCMotor motor(1);  // M1 on L293D shield v1
Servo steering;

// ─── Timing ───────────────────────────────────────────────────────────────────
const uint16_t      ECHO_TIMEOUT_US   = 20000;  // ~3.4 m max range
const unsigned long LOOP_PERIOD_MS    = 20;      // 50 Hz control loop (paper §3.1)

// ─── Steering dynamics (paper §5.4, Eq.12) ───────────────────────────────────
const int   MAX_STEER_DEG = 60;
const float KP_STEER      = 1.9f;   // proportional gain
const float KD_STEER      = 0.25f;  // derivative gain

// ─── Speed policy (paper §5.5, Eq.14) ────────────────────────────────────────
// [FIX-3] BASE_SPEED corrected to 230 (was 240 in open-challenge sketch)
const int BASE_SPEED   = 230;   // smax, PWM units (≈20.8 cm/s calibrated)
const int MIN_SPEED    = 130;   // smin, PWM units (≈7.8 cm/s)
const int REVERSE_SPEED = 160;
const int CLEAR_FAST_CM = 90;   // cfast: full speed above this
const int CLEAR_SLOW_CM = 20;   // cslow: min speed below this

// ─── Safety thresholds (paper §5.2) ──────────────────────────────────────────
const int FRONT_STOP_CM = 15;   // hard stop if FC < 15 cm
const int HARD_BLOCK_CM = 14;   // hard block if FC+diagonals < 14 cm

// ─── Potential-field parameters (paper Table 3, §5.3, Eq.10-11) ──────────────
const int   SIDE_PUSH_START = 60;   // activation radius r for all sensors (cm)
const float K_FRONT         = 1.9f; // front-centre: speed regulation only
const float K_DIAG          = 1.7f; // front-left/right diagonal
const float K_SIDE          = 1.6f; // left/right side
const float K_BACK          = 0.9f; // rear (supplementary damping)

// ─── Sensor smoothing (paper §5.1, Eq.9, α=0.20) ─────────────────────────────
const float ALPHA_SMOOTH = 0.20f;

// ─── Serial / Protocol ────────────────────────────────────────────────────────
// [FIX-1] Watchdog now ENABLED to match paper §3.2 & simulation S5
const bool          REQUIRE_HEARTBEAT = true;
const unsigned long RX_WATCHDOG_MS   = 400;   // motor halts if no byte in 400 ms
const unsigned long HINT_BIAS_MS     = 450;   // KEEP LEFT/RIGHT duration (paper §5.4)
const int           SERBUF_LEN       = 64;

// ─── Motor direction ──────────────────────────────────────────────────────────
const bool INVERT_MOTOR = false;

// ─── Parking FSM timeouts (paper Table 5, §6.3) ──────────────────────────────
const unsigned long T_ALIGN      = 6000UL;  // Align phase timeout (ms)
const unsigned long T_ENTER      = 5000UL;  // Enter phase timeout (ms)
const unsigned long T_STRAIGHTEN = 3000UL;  // Straighten phase timeout (ms)
const unsigned long T_CENTRE     = 4000UL;  // Centre phase timeout (ms)
// Total worst-case = 18 s (paper §6.3)

// Parking sensor thresholds (paper Table 4)
const int ALIGN_TOL_CM     = 5;   // Δtol: |dL – dR| < 5 cm → parallel
const int ENTER_BACK_CM    = 15;  // d*_B: rear wall threshold
const int ENTER_SIDE_CM    = 12;  // d*_R: right side threshold during Enter
const int PROX_GUARD_CM    = 10;  // proximity guard: re-enter Align if FC < 10 cm
const int PARK_NUDGE_SPEED = 110; // slow nudge speed for Align/Centre phases

// ─── State ────────────────────────────────────────────────────────────────────
float dFCf=200, dFLDf=200, dFRDf=200, dLf=200, dRf=200, dBf=200;
float prevFy = 0.0f;

// Pi hints (updated by serial)
float g_offset = 0.0f;   // lane offset o ∈ [-1, 1] (paper §4.5.1)
int   g_speed  = BASE_SPEED;

int           g_keepBias       = 0;   // -1=left, +1=right, 0=none
unsigned long g_keepBias_until = 0;

// Serial bookkeeping
char          lineBuf[SERBUF_LEN];
int           lineLen  = 0;
unsigned long lastRxMs = 0;

// Mode enum – exactly five parking states from paper §6.1
enum Mode {
  IDLE,
  RACE,
  PARK_ALIGN,
  PARK_ENTER,
  PARK_STRAIGHTEN,
  PARK_CENTRE,
  PARK_HOLD,
  EMERGENCY_STOP
};
Mode mode = IDLE;

unsigned long phaseStartMs = 0;

// ─── Motor helpers ────────────────────────────────────────────────────────────
int currentSpeed = 0;
int currentDir   = 0;

void applyMotor(int speed, int dir) {
  if (speed <= 0 || dir == 0) {
    motor.setSpeed(0); motor.run(RELEASE);
    currentSpeed = 0; currentDir = 0;
    return;
  }
  motor.setSpeed(constrain(speed, 0, 255));
  bool fwd = (dir > 0) ^ INVERT_MOTOR;
  motor.run(fwd ? FORWARD : BACKWARD);
  currentSpeed = speed; currentDir = dir;
}

void driveForward(int spd)  { applyMotor(spd,  1); }
void driveBackward(int spd) { applyMotor(spd, -1); }
void driveStop()            { applyMotor(0,    0); }

// ─── Servo helper ─────────────────────────────────────────────────────────────
void setSteerDeg(int delta) {
  delta = constrain(delta, -MAX_STEER_DEG, MAX_STEER_DEG);
  steering.writeMicroseconds(1500 + (int)(delta * 11.0f));
}

// ─── Sensor reading ───────────────────────────────────────────────────────────
long readOnce(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  return (dur == 0) ? 400L : (long)(dur / 58);
}

long readMedian3(uint8_t trig, uint8_t echo) {
  long a = readOnce(trig, echo);
  long b = readOnce(trig, echo);
  long c = readOnce(trig, echo);
  if (a > b) { long t=a; a=b; b=t; }
  if (b > c) { long t=b; b=c; c=t; }
  if (a > b) { long t=a; a=b; b=t; }
  return b;
}

// ─── Speed mapping (paper Eq.14) ──────────────────────────────────────────────
int mapToSpeed(long clear_cm) {
  if (clear_cm <= CLEAR_SLOW_CM) return MIN_SPEED;
  if (clear_cm >= CLEAR_FAST_CM) return BASE_SPEED;
  float t = float(clear_cm - CLEAR_SLOW_CM) / float(CLEAR_FAST_CM - CLEAR_SLOW_CM);
  return constrain((int)(MIN_SPEED + t*(BASE_SPEED - MIN_SPEED)), MIN_SPEED, BASE_SPEED);
}

// ─── Repulsion function (paper Eq.10) ─────────────────────────────────────────
// ρ(d, r, k) = k · max(0, 1 – d/r)   [linear, matches paper exactly]
float repulse(float d_cm, float range_cm, float k) {
  if (d_cm >= range_cm) return 0.0f;
  return k * max(0.0f, 1.0f - (d_cm / range_cm));
}

// ─── Safety reverse manoeuvre (paper §5.2) ────────────────────────────────────
void safetyReverse() {
  bool freerRight = (dRf > dLf);
  driveStop();
  setSteerDeg(freerRight ? +MAX_STEER_DEG : -MAX_STEER_DEG);
  delay(120);
  driveBackward(REVERSE_SPEED);
  delay(300);
  driveForward(MIN_SPEED);
  delay(350);
  setSteerDeg(0);
}

// ─── Phase transition helper ──────────────────────────────────────────────────
void enterPhase(Mode m) {
  mode = m;
  phaseStartMs = millis();
}

// ─── Serial parsing ───────────────────────────────────────────────────────────
void handleLine(char *s) {
  lastRxMs = millis();

  // Strip trailing whitespace
  int n = strlen(s);
  while (n > 0 && (s[n-1] == '\r' || s[n-1] == '\n' || s[n-1] == ' '))
    s[--n] = 0;
  if (n == 0) return;

  if (strcmp(s, "GO") == 0) return;

  if (strncmp(s, "MODE ", 5) == 0) {
    char *arg = s + 5;
    if (strcmp(arg, "RACE") == 0) {
      mode = RACE;
      setSteerDeg(0);
      g_keepBias = 0;
      g_keepBias_until = 0;
    } else if (strcmp(arg, "PARK") == 0) {
      g_keepBias = 0;
      g_keepBias_until = 0;
      enterPhase(PARK_ALIGN);
    }
    return;
  }

  if (strncmp(s, "DRIVE ", 6) == 0) {
    float o; int sp;
    if (sscanf(s + 6, "%f %d", &o, &sp) == 2) {
      g_offset = constrain(o, -1.0f, 1.0f);
      g_speed  = constrain(sp, 0, 255);
    }
    return;
  }

  if (strcmp(s, "KEEP LEFT") == 0) {
    g_keepBias = -1;
    g_keepBias_until = millis() + HINT_BIAS_MS;
    return;
  }
  if (strcmp(s, "KEEP RIGHT") == 0) {
    g_keepBias = +1;
    g_keepBias_until = millis() + HINT_BIAS_MS;
    return;
  }

  if (strcmp(s, "HB") == 0) return;  // heartbeat, lastRxMs already updated

  if (strcmp(s, "STOP") == 0) {
    mode = EMERGENCY_STOP;
    driveStop();
    setSteerDeg(0);
    return;
  }
}

void pumpSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < SERBUF_LEN - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // overflow: discard
    }
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(50);

  // Configure sensor pins
  uint8_t trigs[] = {TRIG_FC, TRIG_FLD, TRIG_FRD, TRIG_L, TRIG_R, TRIG_B};
  uint8_t echos[] = {ECHO_FC, ECHO_FLD, ECHO_FRD, ECHO_L, ECHO_R, ECHO_B};
  for (int i = 0; i < 6; i++) {
    pinMode(trigs[i], OUTPUT);
    pinMode(echos[i], INPUT);
  }

  steering.attach(SERVO_PIN);
  setSteerDeg(0);

  motor.setSpeed(0);
  motor.run(RELEASE);
  currentSpeed = 0; currentDir = 0;

  // Seed smoothing filters with real readings
  dFCf  = readMedian3(TRIG_FC,  ECHO_FC);
  dFLDf = readMedian3(TRIG_FLD, ECHO_FLD);
  dFRDf = readMedian3(TRIG_FRD, ECHO_FRD);
  dLf   = readMedian3(TRIG_L,   ECHO_L);
  dRf   = readMedian3(TRIG_R,   ECHO_R);
  dBf   = readMedian3(TRIG_B,   ECHO_B);

  Serial.println("RDY");
  lastRxMs = millis();
}

// ─── Main Loop (50 Hz target) ────────────────────────────────────────────────
void loop() {
  unsigned long loopStart = millis();

  // ── 0. Serial pump & watchdog ──────────────────────────────────────────────
  pumpSerial();

  // [FIX-1] Watchdog active: halt motor + centre servo if no byte in 400 ms
  if (REQUIRE_HEARTBEAT && mode != EMERGENCY_STOP) {
    if (millis() - lastRxMs > RX_WATCHDOG_MS) {
      driveStop();
      setSteerDeg(0);  // actively centred, not held at last angle (paper §5.2)
      // Keep looping so we can recover when serial resumes
      unsigned long elapsed = millis() - loopStart;
      if (elapsed < LOOP_PERIOD_MS) delay(LOOP_PERIOD_MS - elapsed);
      return;
    }
  }

  // ── 1. Sensor acquisition + exponential smoothing (paper Eq.9, α=0.20) ─────
  long dFC  = readMedian3(TRIG_FC,  ECHO_FC);
  long dFLD = readMedian3(TRIG_FLD, ECHO_FLD);
  long dFRD = readMedian3(TRIG_FRD, ECHO_FRD);
  long dL   = readMedian3(TRIG_L,   ECHO_L);
  long dR   = readMedian3(TRIG_R,   ECHO_R);
  long dB   = readMedian3(TRIG_B,   ECHO_B);

  dFCf  = ALPHA_SMOOTH * dFC  + (1.0f - ALPHA_SMOOTH) * dFCf;
  dFLDf = ALPHA_SMOOTH * dFLD + (1.0f - ALPHA_SMOOTH) * dFLDf;
  dFRDf = ALPHA_SMOOTH * dFRD + (1.0f - ALPHA_SMOOTH) * dFRDf;
  dLf   = ALPHA_SMOOTH * dL   + (1.0f - ALPHA_SMOOTH) * dLf;
  dRf   = ALPHA_SMOOTH * dR   + (1.0f - ALPHA_SMOOTH) * dRf;
  dBf   = ALPHA_SMOOTH * dB   + (1.0f - ALPHA_SMOOTH) * dBf;

  // ── 2. Safety arbitration (paper §5.2) – not suppressible by Pi ────────────
  if (mode != EMERGENCY_STOP && mode != PARK_HOLD) {
    bool frontClose = (dFCf < FRONT_STOP_CM);
    bool hardBlock  = (dFCf < HARD_BLOCK_CM) && (dFLDf < HARD_BLOCK_CM) && (dFRDf < HARD_BLOCK_CM);
    if (frontClose || hardBlock) {
      safetyReverse();
      unsigned long elapsed = millis() - loopStart;
      if (elapsed < LOOP_PERIOD_MS) delay(LOOP_PERIOD_MS - elapsed);
      return;
    }
  }

  // ── 3. Mode behaviours ─────────────────────────────────────────────────────
  switch (mode) {

    // ──────────────────────────────────────────────────────────────────────────
    case IDLE: {
      driveStop();
      setSteerDeg(0);
      break;
    }

    // ──────────────────────────────────────────────────────────────────────────
    case RACE: {
      /*
       * [FIX-2] Steering law now correctly implements paper Eq.12:
       *   δ = KP * (o + Fy) + KD * Fy_dot
       *
       * Previously the code had a winner-takes-all latch: if a DRIVE command
       * arrived within 500 ms it used o alone; otherwise it used Fy alone.
       * The paper always sums them. Fixed here: Fy is always computed from
       * sensors; o from the latest Pi DRIVE hint (default 0 until first DRIVE).
       */

      // Compute lateral repulsion field Fy (paper Eq.11)
      // Positive Fy pushes nose right; negative pushes left.
      float fFLD = repulse(dFLDf, SIDE_PUSH_START, K_DIAG);
      float fFRD = repulse(dFRDf, SIDE_PUSH_START, K_DIAG);
      float fL   = repulse(dLf,   SIDE_PUSH_START, K_SIDE);
      float fR   = repulse(dRf,   SIDE_PUSH_START, K_SIDE);
      // Front-centre does NOT contribute lateral force; rear has reduced role
      float Fy = fFLD - fFRD + fL - fR;  // Eq.11

      // PD derivative term
      float Fy_dot = (Fy - prevFy) / (LOOP_PERIOD_MS / 1000.0f);
      prevFy = Fy;

      // Fuse Pi lane offset + potential field (paper Eq.12)
      float delta = KP_STEER * (g_offset + Fy) + KD_STEER * Fy_dot;

      // KEEP bias: additive ±8° for 450 ms (paper §5.4)
      if (g_keepBias != 0 && (long)(millis() - g_keepBias_until) < 0) {
        delta += (float)(g_keepBias > 0 ? +8 : -8);
      } else {
        g_keepBias = 0;
      }

      int steerDeg = constrain((int)delta, -MAX_STEER_DEG, MAX_STEER_DEG);
      setSteerDeg(steerDeg);

      // Speed: min of field-based and Pi hint (paper §5.5)
      int spd_sensor = mapToSpeed((long)dFCf);
      int spd = min(spd_sensor, g_speed);
      spd = constrain(spd, MIN_SPEED, BASE_SPEED);

      // Reduce speed in high-repulsion environments
      float totalRepulse = fFLD + fFRD + fL + fR;
      if (totalRepulse > 1.5f) spd = max(spd - 40, MIN_SPEED);

      driveForward(spd);
      break;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // PARKING FSM – five states matching paper §6.1 / Table 4 exactly
    // ──────────────────────────────────────────────────────────────────────────

    case PARK_ALIGN: {
      /*
       * State: Align
       * Action: nudge forward/backward to balance dL ≈ dR
       * Exit:   |dL - dR| < ALIGN_TOL or timeout T_ALIGN → Enter
       * Proximity guard: if dFC < PROX_GUARD_CM → re-enter Align (reset phase timer)
       */
      if (dFCf < PROX_GUARD_CM) {
        // Proximity guard: emergency reverse, reset phase timer only
        driveStop();
        driveBackward(REVERSE_SPEED);
        delay(300);
        driveStop();
        enterPhase(PARK_ALIGN);  // resets phase timer, not global elapsed
        break;
      }

      int diff = (int)(dLf - dRf);
      // Nudge direction: if left > right, vehicle is left of bay centre → move right
      int steerBias = (diff > 0) ? -1 : +1;
      int mag = constrain(abs(diff) / 2, 4, 12);
      setSteerDeg(steerBias * mag);

      if (dFCf > 20) driveForward(PARK_NUDGE_SPEED);
      else if (dBf > 20) driveBackward(PARK_NUDGE_SPEED - 10);
      else driveStop();

      // Sensor-based exit
      if (abs(diff) <= ALIGN_TOL_CM) {
        driveStop(); setSteerDeg(0);
        enterPhase(PARK_ENTER);
        break;
      }
      // Timeout exit
      if (millis() - phaseStartMs > T_ALIGN) {
        driveStop(); setSteerDeg(0);
        enterPhase(PARK_ENTER);
      }
      break;
    }

    case PARK_ENTER: {
      /*
       * State: Enter
       * Action: reverse at +60° steering
       * Exit:   dB < d*_B OR dR < d*_R (or timeout T_ENTER) → Straighten
       */
      setSteerDeg(+MAX_STEER_DEG);

      if (dBf > ENTER_BACK_CM && dRf > ENTER_SIDE_CM) {
        driveBackward(REVERSE_SPEED);
      } else {
        driveStop();
        enterPhase(PARK_STRAIGHTEN);
        break;
      }

      if (millis() - phaseStartMs > T_ENTER) {
        driveStop();
        enterPhase(PARK_STRAIGHTEN);
      }
      break;
    }

    case PARK_STRAIGHTEN: {
      /*
       * State: Straighten
       * Action: reverse with counter-steer until δ* → 0
       * Exit:   |steer error| < 5° or timeout T_STRAIGHTEN → Centre
       */
      setSteerDeg(-MAX_STEER_DEG);

      if (dBf > 16) driveBackward(MIN_SPEED);
      else driveStop();

      int diff = (int)(dLf - dRf);
      bool straight = (abs(diff) <= ALIGN_TOL_CM);
      if (straight || millis() - phaseStartMs > T_STRAIGHTEN) {
        driveStop(); setSteerDeg(0);
        enterPhase(PARK_CENTRE);
      }
      break;
    }

    case PARK_CENTRE: {
      /*
       * State: Centre
       * Action: nudge fore/aft so dF ≈ dB (within Δtol = ALIGN_TOL_CM)
       * Exit:   |dF - dB| < Δtol or timeout T_CENTRE → Hold
       * Proximity guard applied at top of case
       */
      if (dFCf < PROX_GUARD_CM) {
        // Proximity guard: emergency reverse, re-enter Align
        driveStop();
        driveBackward(REVERSE_SPEED);
        delay(300);
        driveStop();
        enterPhase(PARK_ALIGN);  // paper Table 4: any non-terminal → Align on guard
        break;
      }

      setSteerDeg(0);
      int diff = (int)(dFCf - dBf);

      if (abs(diff) <= ALIGN_TOL_CM) {
        driveStop();
        enterPhase(PARK_HOLD);
        break;
      }

      // Nudge toward balance point
      if (diff > 0 && dBf > 14) {
        driveBackward(PARK_NUDGE_SPEED);
      } else if (diff < 0 && dFCf > 14) {
        driveForward(PARK_NUDGE_SPEED);
      } else {
        driveStop();
      }

      if (millis() - phaseStartMs > T_CENTRE) {
        driveStop();
        enterPhase(PARK_HOLD);
      }
      break;
    }

    case PARK_HOLD: {
      /*
       * State: Hold (terminal accepting state)
       * Action: motor stop, servo centred
       */
      driveStop();
      setSteerDeg(0);
      break;
    }

    // ──────────────────────────────────────────────────────────────────────────
    case EMERGENCY_STOP: {
      driveStop();
      setSteerDeg(0);
      break;
    }
  }

  // ── 4. Maintain 50 Hz loop cadence ────────────────────────────────────────
  unsigned long elapsed = millis() - loopStart;
  if (elapsed < LOOP_PERIOD_MS) delay(LOOP_PERIOD_MS - elapsed);
}
