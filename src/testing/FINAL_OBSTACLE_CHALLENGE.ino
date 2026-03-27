/*
  WRO 2025 – Obstacle Challenge (Arduino)
  - Motors & steering on Arduino
  - Six HC-SR04 ultrasonics for safety + parking + 5s post-exit avoid
  - High-level hints from Raspberry Pi over Serial (one-shot friendly):
      GO, MODE RACE|PARK, DRIVE <offset> <speed>, KEEP LEFT|RIGHT, HB, STOP

  Behavior:
  - EXIT_PARK: NO REVERSING. Immediately pick freer side (L/R) and hold ±60° (TURN_HOLD_MS), then POST_EXIT_AVOID (5s OA) → RACE.
  - RACE: Open-Challenge obstacle avoidance (repulsion fields). If a DRIVE arrived in the last 0.5 s, latch it and override OA briefly.
*/

#include <Servo.h>
#include <AFMotor.h>

// ---------- Pin Map ----------
#define TRIG_FC 13    // Front-Center TRIG
#define ECHO_FC A5    // Front-Center ECHO
#define TRIG_FLD 3    // Front-Left Diag TRIG
#define ECHO_FLD A1
#define TRIG_FRD 5    // Front-Right Diag TRIG
#define ECHO_FRD A2
#define TRIG_L 6      // Left TRIG
#define ECHO_L A3
#define TRIG_R 10     // Right TRIG
#define ECHO_R A4
#define TRIG_B 2      // Back TRIG
#define ECHO_B A0
#define SERVO_PIN 9   // Steering servo (D9)

// ---------- Hardware ----------
AF_DCMotor motor(1);    // M1 on L293D shield v1
Servo steering;

// ---------- Tunables ----------
const uint16_t ECHO_TIMEOUT_US   = 20000;  // ~5 m

// Steering dynamics
const int      MAX_STEER_DEG     = 60;
const float    KP_STEER          = 1.9f;
const float    KD_STEER          = 0.25f;

// Speed policy
const int      BASE_SPEED        = 230;    // normal speed (0..255)
const int      MIN_SPEED         = 130;    // minimum forward speed when narrow
const int      REVERSE_SPEED     = 160;    // reverse speed (used in safety routine)
const int      CLEAR_FAST_CM     = 90;     // >= this ahead -> allow BASE_SPEED
const int      CLEAR_SLOW_CM     = 20;     // <= this ahead -> clamp to MIN_SPEED

// Stops & safety
const int      FRONT_STOP_CM     = 15;     // stop/avoid if FC < 15 cm
const int      HARD_BLOCK_CM     = 14;     // reverse failsafe threshold
const int      SIDE_PUSH_START   = 60;     // start side repulsion within this range

// Repulsion strengths
const float    K_FRONT           = 1.9f;
const float    K_DIAG            = 1.7f;
const float    K_SIDE            = 1.6f;
const float    K_BACK            = 0.9f;

// Smoothing
const float    ALPHA_SMOOTH      = 0.20f;

// Optional: flip motor polarity without rewiring
const bool     INVERT_MOTOR      = false;

// ---------- Serial/Protocol ----------
const bool          REQUIRE_HEARTBEAT = false;   // set true to force stop if no serial
const unsigned long RX_WATCHDOG_MS    = 400;     // used only if REQUIRE_HEARTBEAT==true
const unsigned long HINT_BIAS_MS      = 450;     // KEEP LEFT/RIGHT bias duration
const int           SERBUF_LEN        = 64;

// ---------- One-shot latch (for DRIVE) ----------
const unsigned long LATCH_MS = 500;      // latch DRIVE for 0.5 s
unsigned long lastCmdMs = 0;             // millis of last DRIVE

// ---------- State (smoothed distances) ----------
float dFCf=200, dFLDf=200, dFRDf=200, dLf=200, dRf=200, dBf=200;

// PD steering memory
float prevFy = 0.0f;

// DRIVE hint from Pi (latched)
float g_offset = 0.0f; // -1..+1
int   g_speed  = 0;    // 0..255

// Hint bias (KEEP LEFT/RIGHT): -1=left bias, +1=right bias, 0=none
int   g_keepBias = 0;
unsigned long g_keepBias_until = 0;

// Serial RX tracking
char lineBuf[SERBUF_LEN];
int  lineLen = 0;
unsigned long lastRxMs = 0;

// ---------- Modes ----------
enum Mode {
  IDLE,
  EXIT_PARK,          // NOW: decide side & hold ±60°, no reversing
  RACE,
  PARK_ALIGN,
  PARK_ENTER,
  PARK_STRAIGHTEN,
  PARK_CENTER,
  PARK_HOLD,
  EMERGENCY_STOP,
  POST_EXIT_AVOID     // 5s field-only avoidance before RACE
};
Mode mode = IDLE;

// ----- Exit-Park parameters -----
const unsigned long TURN_HOLD_MS    = 600;    // hold ±60° steering

// EXIT_PARK substates
enum ExitStage { EP_DECIDE_SIDE, EP_TURN_60, EP_DONE };
ExitStage epStage = EP_DECIDE_SIDE;
bool epPreferRight = true;
unsigned long epStageStartMs = 0;

// ----- Post-exit avoidance timing -----
const unsigned long POST_EXIT_MS = 5000UL;
unsigned long postExitStartMs = 0;

// ---------- Motor stability layer ----------
int currentMotorSpeed = 0;   // 0..255 absolute magnitude
int currentMotorDir = 0;     // -1 = backward, 0 = stopped, +1 = forward

void applyMotorHardware(int speed, int dir) {
  if (speed <= 0 || dir == 0) {
    motor.setSpeed(0);
    motor.run(RELEASE);
    currentMotorSpeed = 0;
    currentMotorDir = 0;
    return;
  }
  motor.setSpeed(constrain(speed, 0, 255));
  if (INVERT_MOTOR) dir = -dir;
  if (dir > 0) motor.run(FORWARD);
  else motor.run(BACKWARD);
  currentMotorSpeed = speed;
  currentMotorDir = dir;
}

void setMotorState(int targetSpeedSigned) {
  int targetDir = 0;
  int targetSpeed = 0;
  if (targetSpeedSigned > 0) { targetDir = +1; targetSpeed = targetSpeedSigned; }
  else if (targetSpeedSigned < 0) { targetDir = -1; targetSpeed = -targetSpeedSigned; }
  else { targetDir = 0; targetSpeed = 0; }
  if (targetDir != currentMotorDir || targetSpeed != currentMotorSpeed) {
    applyMotorHardware(targetSpeed, targetDir);
  }
}

void driveForward_direct(uint8_t spd) { setMotorState(spd); }
void driveBackward_direct(uint8_t spd) { setMotorState(-int(spd)); }
void driveStop_direct() { setMotorState(0); }

// ---------- Helpers ----------
template<typename T>
T clampT(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
inline int   clampI(int v, int lo, int hi)       { return clampT<int>(v, lo, hi); }
inline float clampF(float v, float lo, float hi) { return clampT<float>(v, lo, hi); }

long readDistanceOnce(uint8_t trig, uint8_t echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) return 400;       // treat as far
  return (long)(dur / 58);        // us -> cm
}

long readDistanceMedian3(uint8_t trig, uint8_t echo) {
  long a = readDistanceOnce(trig, echo);
  long b = readDistanceOnce(trig, echo);
  long c = readDistanceOnce(trig, echo);
  if (a > b) { long t=a; a=b; b=t; }
  if (b > c) { long t=b; b=c; c=t; }
  if (a > b) { long t=a; a=b; b=t; }
  return b;
}

// Microsecond-precision steering
void setSteerDeg(int delta) {
  delta = clampI(delta, -MAX_STEER_DEG, MAX_STEER_DEG);
  const int center_us = 1500;
  const float us_per_deg = 11.0f;
  int pulse = center_us + (int)(delta * us_per_deg);
  steering.writeMicroseconds(clampI(pulse, 1000, 2000));
}

// map speed by clearance
int mapToSpeed(long clear_cm) {
  if (clear_cm <= CLEAR_SLOW_CM) return MIN_SPEED;
  if (clear_cm >= CLEAR_FAST_CM) return BASE_SPEED;
  float t = float(clear_cm - CLEAR_SLOW_CM) / float(CLEAR_FAST_CM - CLEAR_SLOW_CM);
  int spd = (int)(MIN_SPEED + t * (BASE_SPEED - MIN_SPEED));
  return clampI(spd, MIN_SPEED, BASE_SPEED);
}

// Compute repulsion magnitude
float repulse(float d_cm, float range_cm, float k) {
  if (d_cm >= range_cm) return 0.0f;
  float s = 1.0f - (d_cm / range_cm);
  if (s < 0) s = 0;
  return k * s;
}

// Stop → steer away → avoidance maneuver (safety)
void stopSteerAndAvoid(bool preferRight) {
  driveStop_direct();
  setSteerDeg(preferRight ? +MAX_STEER_DEG : -MAX_STEER_DEG);
  delay(120);
  driveBackward_direct(REVERSE_SPEED);
  delay(300);
  driveForward_direct(MIN_SPEED);
  delay(350);
  setSteerDeg(0);
}

// ---------- EXIT_PARK control (no reversing version) ----------
void enterExitPark() {
  mode = EXIT_PARK;
  epStage = EP_DECIDE_SIDE;        // start by deciding side immediately
  epStageStartMs = millis();
  setSteerDeg(0);
  driveStop_direct();
}

void runExitPark() {
  switch (epStage) {
    case EP_DECIDE_SIDE: {
      // Choose side with more clearance immediately (no reversing)
      epPreferRight = (dRf >= dLf);
      epStage = EP_TURN_60;
      epStageStartMs = millis();
      break;
    }

    case EP_TURN_60: {
      // Hold ±60° steering, motors stopped
      setSteerDeg(epPreferRight ? +60 : -60);
      driveStop_direct();
      if (millis() - epStageStartMs >= TURN_HOLD_MS) {
        setSteerDeg(0);
        epStage = EP_DONE;
      }
      break;
    }

    case EP_DONE: {
      // Enter 5s obstacle avoidance before RACE
      setSteerDeg(0);
      mode = POST_EXIT_AVOID;
      postExitStartMs = millis();
      break;
    }
  }
}

// ---------- Serial parsing ----------
void handleLine(char *s) {
  lastRxMs = millis();

  int n = 0;
  while (s[n]) n++;
  while (n > 0 && (s[n-1] == '\r' || s[n-1] == '\n' || s[n-1] == ' ')) { s[n-1] = 0; n--; }
  if (n == 0) return;

  if (strcmp(s, "GO") == 0) { return; }

  if (strncmp(s, "MODE ", 5) == 0) {
    char *arg = s + 5;

    if (strcmp(arg, "RACE") == 0) {
      if (mode == IDLE || mode == PARK_ALIGN || mode == PARK_ENTER ||
          mode == PARK_STRAIGHTEN || mode == PARK_CENTER || mode == PARK_HOLD ||
          mode == EMERGENCY_STOP) {
        enterExitPark();
      } else {
        mode = RACE;
        setSteerDeg(0);
      }
      g_keepBias = 0; g_keepBias_until = 0;
      return;
    }

    if (strcmp(arg, "PARK") == 0) {
      g_keepBias = 0; g_keepBias_until = 0;
      mode = PARK_ALIGN;
      return;
    }
  }

  if (strncmp(s, "DRIVE ", 6) == 0) {
    float o; int sp;
    if (sscanf(s + 6, "%f %d", &o, &sp) == 2) {
      g_offset = clampF(o, -1.0f, 1.0f);
      g_speed  = clampI(sp, 0, 255);
      lastCmdMs = millis();                 // latch timestamp
    }
    return;
  }

  if (strcmp(s, "KEEP LEFT") == 0) {
    g_keepBias = -1; g_keepBias_until = millis() + HINT_BIAS_MS;
    return;
  }
  if (strcmp(s, "KEEP RIGHT") == 0) {
    g_keepBias = +1; g_keepBias_until = millis() + HINT_BIAS_MS;
    return;
  }

  if (strcmp(s, "HB") == 0) { return; }

  if (strcmp(s, "STOP") == 0) {
    mode = EMERGENCY_STOP;
    driveStop_direct();
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
    } else {
      if (lineLen < SERBUF_LEN - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;
      }
    }
  }
}

// ---------- Parking helpers (kept, unused unless you use PARK modes) ----------
const int ALIGN_TOL_CM       = 3;   // |L-R| below this => parallel
const int ALIGN_SPEED        = 120;

const int ENTER_BACK_NEAR_CM = 22;  // when back gets this close, start straighten
const int ENTER_SIDE_NEAR_CM = 18;  // when right side close during angle-in

const int STRAIGHT_TOL_CM    = 3;   // |L-R| small again => straight
const int CENTER_FRONT_CM    = 18;  // try to center between front/back ~ this
const int CENTER_BACK_CM     = 18;

unsigned long phaseStartMs = 0;

void enterPhase(Mode m) {
  mode = m;
  phaseStartMs = millis();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(50);

  steering.attach(SERVO_PIN);
  setSteerDeg(0);

  currentMotorSpeed = 0;
  currentMotorDir = 0;
  motor.setSpeed(0);
  motor.run(RELEASE);

  dFCf  = readDistanceMedian3(TRIG_FC,  ECHO_FC);
  dFLDf = readDistanceMedian3(TRIG_FLD, ECHO_FLD);
  dFRDf = readDistanceMedian3(TRIG_FRD, ECHO_FRD);
  dLf   = readDistanceMedian3(TRIG_L,   ECHO_L);
  dRf   = readDistanceMedian3(TRIG_R,   ECHO_R);
  dBf   = readDistanceMedian3(TRIG_B,   ECHO_B);

  Serial.println("RDY");
  lastRxMs = millis();
}

// ---------- Main Loop ----------
void loop() {
  // 0) Serial & optional watchdog
  pumpSerial();

  if (REQUIRE_HEARTBEAT) {
    if (mode != EMERGENCY_STOP && !(mode == EXIT_PARK && epStage == EP_TURN_60)) {
      if (millis() - lastRxMs > RX_WATCHDOG_MS) {
        driveStop_direct();
        setSteerDeg(0);
      }
    }
  }

  // 1) Read & smooth ultrasonics
  long dFC  = readDistanceMedian3(TRIG_FC,  ECHO_FC);
  long dFLD = readDistanceMedian3(TRIG_FLD, ECHO_FLD);
  long dFRD = readDistanceMedian3(TRIG_FRD, ECHO_FRD);
  long dL   = readDistanceMedian3(TRIG_L,   ECHO_L);
  long dR   = readDistanceMedian3(TRIG_R,   ECHO_R);
  long dB   = readDistanceMedian3(TRIG_B,   ECHO_B);

  dFCf  = ALPHA_SMOOTH * dFC  + (1 - ALPHA_SMOOTH) * dFCf;
  dFLDf = ALPHA_SMOOTH * dFLD + (1 - ALPHA_SMOOTH) * dFLDf;
  dFRDf = ALPHA_SMOOTH * dFRD + (1 - ALPHA_SMOOTH) * dFRDf;
  dLf   = ALPHA_SMOOTH * dL   + (1 - ALPHA_SMOOTH) * dLf;
  dRf   = ALPHA_SMOOTH * dR   + (1 - ALPHA_SMOOTH) * dRf;
  dBf   = ALPHA_SMOOTH * dB   + (1 - ALPHA_SMOOTH) * dBf;

  // 2) Global immediate safety — works in any mode (except during the brief turn)
  if (!(mode == EXIT_PARK && epStage == EP_TURN_60) && mode != EMERGENCY_STOP) {
    if (dFCf < FRONT_STOP_CM) {
      bool freerRight = (dRf > dLf);
      stopSteerAndAvoid(freerRight);
      delay(20);
      return;
    }
    if (dFCf < HARD_BLOCK_CM || (dFLDf < HARD_BLOCK_CM && dFRDf < HARD_BLOCK_CM)) {
      bool freerRight = (dRf > dLf);
      stopSteerAndAvoid(freerRight);
      delay(20);
      return;
    }
  }

  // 3) Mode behaviors
  switch (mode) {
    case IDLE: {
      driveStop_direct();
      setSteerDeg(0);
      break;
    }

    case EXIT_PARK: {
      runExitPark();
      break;
    }

    case POST_EXIT_AVOID: {
      // Field-only obstacle avoidance for 5s (ignores Pi DRIVE offset)
      float Fx = 0, Fy = 0;

      float fFC  = repulse(dFCf,  CLEAR_FAST_CM, K_FRONT);   Fx += -fFC;
      float fFLD = repulse(dFLDf, CLEAR_FAST_CM, K_DIAG);
      float fFRD = repulse(dFRDf, CLEAR_FAST_CM, K_DIAG);
      const float c45 = 0.70710678f;
      Fx += -fFLD * c45;   Fy += -fFLD * c45;
      Fx += -fFRD * c45;   Fy += +fFRD * c45;

      float fL = repulse(dLf, SIDE_PUSH_START, K_SIDE);
      float fR = repulse(dRf, SIDE_PUSH_START, K_SIDE);
      Fy += -fL;  Fy += +fR;

      float fB = repulse(dBf, SIDE_PUSH_START, K_BACK);      Fx += +fB;

      float Fy_dot = Fy - prevFy;  prevFy = Fy;
      float steer_from_fields = (KP_STEER * Fy * 30.0f) + (KD_STEER * Fy_dot * 30.0f);
      int steerDeg = (int)clampF(steer_from_fields, -(float)MAX_STEER_DEG, (float)MAX_STEER_DEG);
      setSteerDeg(steerDeg);

      long diagAvg = (long)((dFLDf + dFRDf) * 0.5f);
      long forwardClear = min((long)dFCf, diagAvg);
      int spd = mapToSpeed(forwardClear);
      if (Fx < -1.2f) spd = max(spd - 50, MIN_SPEED);

      if (spd <= 0) driveStop_direct();
      else driveForward_direct(spd);

      // Seamless handoff to RACE
      if (millis() - postExitStartMs >= POST_EXIT_MS) {
        prevFy = 0.0f;      // avoid PD spike on the first RACE tick
        mode = RACE;
      }
      break;
    }

    case RACE: {
      // ---- Open-Challenge obstacle avoidance baseline ----
      float Fx = 0, Fy = 0;
      float fFC  = repulse(dFCf,  CLEAR_FAST_CM, K_FRONT);
      Fx += -fFC;
      float fFLD = repulse(dFLDf, CLEAR_FAST_CM, K_DIAG);
      float fFRD = repulse(dFRDf, CLEAR_FAST_CM, K_DIAG);
      const float c45 = 0.70710678f;
      Fx += -fFLD * c45;   Fy += -fFLD * c45;
      Fx += -fFRD * c45;   Fy += +fFRD * c45;
      float fL = repulse(dLf, SIDE_PUSH_START, K_SIDE);
      float fR = repulse(dRf, SIDE_PUSH_START, K_SIDE);
      Fy += -fL;
      Fy += +fR;
      float fB = repulse(dBf, SIDE_PUSH_START, K_BACK);
      Fx += +fB;

      float Fy_dot = Fy - prevFy; prevFy = Fy;
      float steer_from_fields = (KP_STEER * Fy * 30.0f) + (KD_STEER * Fy_dot * 30.0f);
      int   steerDeg_OA = clampI((int)steer_from_fields, -MAX_STEER_DEG, MAX_STEER_DEG);

      long diagAvg = (long)((dFLDf + dFRDf) * 0.5f);
      long forwardClear = min((long)dFCf, diagAvg);
      int spd_OA = mapToSpeed(forwardClear);
      if (Fx < -1.2f) spd_OA = max(spd_OA - 50, MIN_SPEED);

      // ---- Pi latch override (0.5 s after DRIVE) ----
      bool latchActive = (millis() - lastCmdMs) <= LATCH_MS;
      float steer_from_offset = g_offset * MAX_STEER_DEG;

      int bias = 0;
      if (g_keepBias != 0 && (long)(millis() - g_keepBias_until) < 0) {
        bias = (g_keepBias > 0) ? +8 : -8;
      } else {
        g_keepBias = 0;
      }

      int steerDeg = steerDeg_OA;
      int spd      = spd_OA;

      if (latchActive) {
        steerDeg = (int)clampF(steer_from_offset + (float)bias, -(float)MAX_STEER_DEG, (float)MAX_STEER_DEG);
        spd      = clampI(g_speed, 0, 255);
      } else {
        steerDeg = clampI(steerDeg_OA + bias, -MAX_STEER_DEG, MAX_STEER_DEG);
        // spd stays from OA
      }

      setSteerDeg(steerDeg);
      if (spd <= 0) driveStop_direct();
      else          driveForward_direct(spd);
      break;
    }

    case PARK_ALIGN: {
      int diff = (int)(dLf - dRf);
      int sgn = (diff > 0) ? -1 : +1;
      int mag = min(12, max(4, abs(diff)));
      setSteerDeg(sgn * mag);

      int spd = ALIGN_SPEED;
      if (dFCf > 25) driveForward_direct(spd);
      else if (dBf > 25) driveBackward_direct(spd-10);
      else driveStop_direct();

      if (abs(diff) <= ALIGN_TOL_CM) {
        driveStop_direct();
        setSteerDeg(0);
        enterPhase(PARK_ENTER);
      }

      if (millis() - phaseStartMs > 5000UL) {
        enterPhase(PARK_ENTER);
      }
      break;
    }

    case PARK_ENTER: {
      setSteerDeg(+MAX_STEER_DEG);
      if (dBf > ENTER_BACK_NEAR_CM && dRf > ENTER_SIDE_NEAR_CM) {
        driveBackward_direct(REVERSE_SPEED);
      } else {
        driveStop_direct();
        enterPhase(PARK_STRAIGHTEN);
      }
      if (millis() - phaseStartMs > 4000UL) {
        enterPhase(PARK_STRAIGHTEN);
      }
      break;
    }

    case PARK_STRAIGHTEN: {
      setSteerDeg(-MAX_STEER_DEG);
      if (dBf > 16) driveBackward_direct(MIN_SPEED);
      else driveStop_direct();

      int diff = (int)(dLf - dRf);
      if (abs(diff) <= STRAIGHT_TOL_CM || millis() - phaseStartMs > 2000UL) {
        driveStop_direct();
        setSteerDeg(0);
        enterPhase(PARK_CENTER);
      }
      break;
    }

    case PARK_CENTER: {
      bool front_ok = (dFCf >= CENTER_FRONT_CM - 2) && (dFCf <= CENTER_FRONT_CM + 6);
      bool back_ok  = (dBf >= CENTER_BACK_CM - 2)  && (dBf <= CENTER_BACK_CM + 6);

      if (!front_ok && dFCf > CENTER_FRONT_CM + 6 && dFCf > 14) {
        setSteerDeg(0);
        driveForward_direct(MIN_SPEED);
      } else if (!back_ok && dBf > CENTER_BACK_CM + 6 && dBf > 14) {
        setSteerDeg(0);
        driveBackward_direct(MIN_SPEED);
      } else {
        driveStop_direct();
      }

      if (front_ok && back_ok) {
        enterPhase(PARK_HOLD);
      }
      if (millis() - phaseStartMs > 3000UL) {
        enterPhase(PARK_HOLD);
      }
      break;
    }

    case PARK_HOLD: {
      driveStop_direct();
      setSteerDeg(0);
      break;
    }

    case EMERGENCY_STOP: {
      driveStop_direct();
      setSteerDeg(0);
      break;
    }
  }

  // Loop timing ~20ms for smooth servo updates
  delay(20);
}
