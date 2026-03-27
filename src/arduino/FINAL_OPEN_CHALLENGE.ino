/*
  WRO 2025 – Open Challenge with Lap Counting
*/

#include <Servo.h>
#include <AFMotor.h>

// ---------- Pin Map (updated) ----------
#define TRIG_FC 13    // Front-Center TRIG moved to D13
#define ECHO_FC A5    // Front-Center ECHO moved to A5
#define TRIG_FLD 3    // Front-Left Diag TRIG
#define ECHO_FLD A1
#define TRIG_FRD 5    // Front-Right Diag TRIG
#define ECHO_FRD A2
#define TRIG_L 6      // Left TRIG
#define ECHO_L A3
#define TRIG_R 10     // Right TRIG
#define ECHO_R A4

#define SWITCH_PIN A0 // Start switch: one leg to A0, other to GND
#define SERVO_PIN 9   // Steering servo (D9)

// ---------- Hardware ----------
AF_DCMotor motor(1);    // M1 on L293D shield v1
Servo steering;

// ---------- Tunables ----------
const uint16_t ECHO_TIMEOUT_US = 30000;  // ~5 m

const int      MAX_STEER_DEG   = 60;
const float    KP_STEER        = 1.9f;
const float    KD_STEER        = 0.25f;

const int      BASE_SPEED      = 240;
const int      MIN_SPEED       = 120;
const int      REVERSE_SPEED   = 160;
const int      CLEAR_FAST_CM   = 90;
const int      CLEAR_SLOW_CM   = 20;

const int      FRONT_REVERSE_CM= 10;     // reverse if FC < 10 cm
const int      HARD_BLOCK_CM   = 14;     // reverse if tight ahead/diagonals
const int      SIDE_PUSH_START = 60;

const float    K_FRONT         = 1.9f;
const float    K_DIAG          = 1.7f;
const float    K_SIDE          = 1.6f;

const float    ALPHA_SMOOTH    = 0.20f;
const bool     INVERT_MOTOR    = false;

// ---------- State (smoothed distances) ----------
float dFCf=200, dFLDf=200, dFRDf=200, dLf=200, dRf=200;
float prevFy = 0.0f;

// ---------- Lap counting ----------
unsigned long lastLapTime = 0;
int lapCount = 0;
const int maxLaps = 3;
const unsigned long lapCooldownMs = 20000; // 20s (your lap ~35s)

// ---------- Helpers ----------
long readDistanceOnce(uint8_t trig, uint8_t echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) return 400;       // treat as far (no echo)
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

// Reverse 1.5s -> steer to freer side 60° -> forward 0.4s -> straighten
void reverseAndAdjust(bool steerRight) {
  driveBackward(REVERSE_SPEED);
  delay(1500);
  driveStop();

  setSteerDeg(steerRight ? +MAX_STEER_DEG : -MAX_STEER_DEG);
  delay(120);
  driveForward(MIN_SPEED);
  delay(400);
  driveStop();
  setSteerDeg(0);
}

int mapToSpeed(long clear_cm) {
  if (clear_cm <= CLEAR_SLOW_CM) return MIN_SPEED;
  if (clear_cm >= CLEAR_FAST_CM) return BASE_SPEED;
  float t = float(clear_cm - CLEAR_SLOW_CM) / float(CLEAR_FAST_CM - CLEAR_SLOW_CM);
  int spd = (int)(MIN_SPEED + t * (BASE_SPEED - MIN_SPEED));
  return clamp(spd, MIN_SPEED, BASE_SPEED);
}

float repulse(float d_cm, float range_cm, float k) {
  if (d_cm >= range_cm) return 0.0f;
  float s = 1.0f - (d_cm / range_cm);
  if (s < 0) s = 0;
  return k * s; // smooth, bounded
}

void waitForSwitch() {
  // Button wired: A0 <-> GND (active LOW). Use internal pull-up.
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  // Wait until pressed (LOW) then released (HIGH) to start
  while (digitalRead(SWITCH_PIN) == HIGH) { delay(10); } // wait press
  delay(150); // debounce
  while (digitalRead(SWITCH_PIN) == LOW) { delay(10); }  // wait release
  delay(150); // debounce
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  steering.attach(SERVO_PIN);
  setSteerDeg(0);

  motor.setSpeed(0);
  motor.run(RELEASE);

  // Wait for start switch on A0
  waitForSwitch();

  // Seed filters
  dFCf  = readDistanceMedian3(TRIG_FC,  ECHO_FC);
  dFLDf = readDistanceMedian3(TRIG_FLD, ECHO_FLD);
  dFRDf = readDistanceMedian3(TRIG_FRD, ECHO_FRD);
  dLf   = readDistanceMedian3(TRIG_L,   ECHO_L);
  dRf   = readDistanceMedian3(TRIG_R,   ECHO_R);
}

// ---------- Main Loop ----------
void loop() {
  // 1) Read & smooth distances
  long dFC  = readDistanceMedian3(TRIG_FC,  ECHO