/*
  WRO 2025 – Open Challenge
*/

#include <Servo.h>
#include <AFMotor.h>

// ---------- Pin Map (saved reference) ----------
#define TRIG_FC 2     // Front-Center TRIG
#define ECHO_FC A0
#define TRIG_FLD 3    // Front-Left Diag TRIG
#define ECHO_FLD A1
#define TRIG_FRD 5    // Front-Right Diag TRIG
#define ECHO_FRD A2
#define TRIG_L 6      // Left TRIG
#define ECHO_L A3
#define TRIG_R 10     // Right TRIG
#define ECHO_R A4
#define TRIG_B 13     // Back TRIG
#define ECHO_B A5

#define SERVO_PIN 9   // Steering servo (D9)

// ---------- Hardware ----------
AF_DCMotor motor(1);    // M1 on L293D shield v1
Servo steering;

// ---------- Tunables ----------
const uint16_t ECHO_TIMEOUT_US   = 30000;  // ~5 m

// (1) Faster steering response
const int      MAX_STEER_DEG     = 55;     // was 35
const float    KP_STEER          = 1.9f;   // was 1.2f (P gain)
const float    KD_STEER          = 0.25f;  // (4) D gain (start 0.15–0.35)

// Speed policy
const int      BASE_SPEED        = 240;    // normal speed (0..255) — adjust if needed
const int      MIN_SPEED         = 120;    // minimum forward speed when narrow
const int      REVERSE_SPEED     = 160;    // reverse speed
const int      CLEAR_FAST_CM     = 90;     // >= this ahead -> allow BASE_SPEED
const int      CLEAR_SLOW_CM     = 20;     // <= this ahead -> clamp to MIN_SPEED

// Stops & safety
const int      FRONT_STOP_CM     = 15;     // NEW: stop if FC < 15 cm
const int      HARD_BLOCK_CM     = 14;     // reverse failsafe threshold
const int      SIDE_PUSH_START   = 60;     // start side repulsion within this range

// Repulsion strengths
const float    K_FRONT           = 1.9f;
const float    K_DIAG            = 1.7f;
const float    K_SIDE            = 1.6f;
const float    K_BACK            = 0.9f;

// (2) Faster reaction (less smoothing) & loop timing
const float    ALPHA_SMOOTH      = 0.25f;  // was 0.35f

// Optional: flip motor polarity without rewiring
const bool     INVERT_MOTOR      = false;

// ---------- State (smoothed distances) ----------
float dFCf=200, dFLDf=200, dFRDf=200, dLf=200, dRf=200, dBf=200;

// PD steering state
float prevFy = 0.0f;

// ---------- Helpers ----------
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

template<typename T>
T clamp(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

// (3) Microsecond-precision steering for snappier control
void setSteerDeg(int delta) { // -MAX..+MAX relative to center (90°)
  delta = clamp(delta, -MAX_STEER_DEG, MAX_STEER_DEG);
  const int center_us = 1500;
  const float us_per_deg = 11.0f;     // tune if needed (10–12 typical)
  int pulse = center_us + (int)(delta * us_per_deg);
  steering.writeMicroseconds(clamp(pulse, 1000, 2000));
}

void driveForward(uint8_t spd) {
  motor.setSpeed(spd);
  if (INVERT_MOTOR) motor.run(BACKWARD);
  else              motor.run(FORWARD);
}

void driveBackward(uint8_t spd) {
  motor.setSpeed(spd);
  if (INVERT_MOTOR) motor.run(FORWARD);
  else              motor.run(BACKWARD);
}

void driveStop() {
  motor.setSpeed(0);
  motor.run(RELEASE);
}

// Stop → steer away → avoidance maneuver (reverse+arc forward)
void stopSteerAndAvoid(bool preferRight) {
  driveStop();
  setSteerDeg(preferRight ? +MAX_STEER_DEG : -MAX_STEER_DEG);
  delay(120);                          // short settle while stopped

  // Back out a bit
  driveBackward(REVERSE_SPEED);
  delay(300);

  // Arc forward away from obstacle
  driveForward(MIN_SPEED);
  delay(350);

  // Straighten for resume
  setSteerDeg(0);
}

// Linear map with clamp for speed
int mapToSpeed(long clear_cm) {
  if (clear_cm <= CLEAR_SLOW_CM) return MIN_SPEED;
  if (clear_cm >= CLEAR_FAST_CM) return BASE_SPEED;
  float t = float(clear_cm - CLEAR_SLOW_CM) / float(CLEAR_FAST_CM - CLEAR_SLOW_CM);
  int spd = (int)(MIN_SPEED + t * (BASE_SPEED - MIN_SPEED));
  return clamp(spd, MIN_SPEED, BASE_SPEED);
}

// Compute repulsion magnitude given distance and range
float repulse(float d_cm, float range_cm, float k) {
  if (d_cm >= range_cm) return 0.0f;
  float s = 1.0f - (d_cm / range_cm);
  if (s < 0) s = 0;
  return k * s; // smooth, bounded
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  steering.attach(SERVO_PIN);
  setSteerDeg(0);

  motor.setSpeed(0);
  motor.run(RELEASE);

  // Initial reads to seed filters
  dFCf  = readDistanceMedian3(TRIG_FC,  ECHO_FC);
  dFLDf = readDistanceMedian3(TRIG_FLD, ECHO_FLD);
  dFRDf = readDistanceMedian3(TRIG_FRD, ECHO_FRD);
  dLf   = readDistanceMedian3(TRIG_L,   ECHO_L);
  dRf   = readDistanceMedian3(TRIG_R,   ECHO_R);
  dBf   = readDistanceMedian3(TRIG_B,   ECHO_B);
}

// ---------- Main Loop ----------
void loop() {
  // 1) Read & smooth distances
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

  // 2) Immediate STOP zone: if very close in front, stop then avoid
  if (dFCf < FRONT_STOP_CM) {
    bool freerRight = (dRf > dLf);     // turn toward open side
    stopSteerAndAvoid(freerRight);
    delay(20);
    return;
  }

  // 3) Failsafe: if front + both diagonals are tight, reverse+avoid
  if (dFCf < HARD_BLOCK_CM || (dFLDf < HARD_BLOCK_CM && dFRDf < HARD_BLOCK_CM)) {
    bool freerRight = (dRf > dLf);
    stopSteerAndAvoid(freerRight);
    delay(20);
    return;
  }

  // 4) Potential-field style repulsion vectors
  // Bearings: FC=0°, FLD=+45°, FRD=-45°, L=+90°, R=-90°, B=180°
  float Fx = 0, Fy = 0;

  float fFC  = repulse(dFCf,  CLEAR_FAST_CM, K_FRONT);
  Fx += -fFC;

  float fFLD = repulse(dFLDf, CLEAR_FAST_CM, K_DIAG);
  float fFRD = repulse(dFRDf, CLEAR_FAST_CM, K_DIAG);
  const float c45 = 0.70710678f;
  Fx += -fFLD * c45;   Fy += -fFLD * c45;   // away from +45°
  Fx += -fFRD * c45;   Fy += +fFRD * c45;   // away from -45°

  float fL = repulse(dLf, SIDE_PUSH_START, K_SIDE);
  float fR = repulse(dRf, SIDE_PUSH_START, K_SIDE);
  Fy += -fL;  // left sensor pushes right
  Fy += +fR;  // right sensor pushes left

  float fB = repulse(dBf, SIDE_PUSH_START, K_BACK);
  Fx += +fB;

  // 5) PD Steering: P on lateral repulsion (Fy), D on its change
  float Fy_dot = Fy - prevFy;    // discrete derivative
  prevFy = Fy;

  float steerCmd = (KP_STEER * Fy * 30.0f) + (KD_STEER * Fy_dot * 30.0f);
  int   steerDeg = (int)clamp(steerCmd, (float)-MAX_STEER_DEG, (float)MAX_STEER_DEG);
  setSteerDeg(steerDeg);

  // 6) Proportional speed based on forward clearance (min FC, diag mean)
  long diagAvg = (long)((dFLDf + dFRDf) * 0.5f);
  long forwardClear = min((long)dFCf, diagAvg);
  int spd = mapToSpeed(forwardClear);

  // If net field is strongly pushing back, bias speed down a bit
  if (Fx < -1.2f) spd = max(spd - 50, MIN_SPEED);

  // 7) Drive
  driveForward(spd);

  // (2) Faster loop — ~20 ms matches typical servo update frame
  delay(20);

  // --- Debug (optional) ---
  // Serial.print("FC:"); Serial.print(dFCf);
  // Serial.print(" FLD:"); Serial.print(dFLDf);
  // Serial.print(" FRD:"); Serial.print(dFRDf);
  // Serial.print(" L:"); Serial.print(dLf);
  // Serial.print(" R:"); Serial.print(dRf);
  // Serial.print(" B:"); Serial.print(dBf);
  // Serial.print(" | Fx:"); Serial.print(Fx,2);
  // Serial.print(" Fy:"); Serial.print(Fy,2);
  // Serial.print(" dFy:"); Serial.print(Fy_dot,2);
  // Serial.print(" steer:"); Serial.print(steerDeg);
  // Serial.print(" spd:"); Serial.println(spd);
}
