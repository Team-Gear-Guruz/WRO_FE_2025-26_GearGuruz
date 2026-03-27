#include <Servo.h>
#include <AFMotor.h>

// ---------- Pin Map (updated) ----------
#define TRIG_FC 13    // Front-Center TRIG
#define ECHO_FC A5   // Front-Center ECHO
#define TRIG_FLD 3    // Front-Left Diag TRIG
#define ECHO_FLD A1
#define TRIG_FRD 5    // Front-Right Diag TRIG
#define ECHO_FRD A2
#define TRIG_L 6      // Left TRIG
#define ECHO_L A3
#define TRIG_R 10     // Right TRIG
#define ECHO_R A4
#define SERVO_PIN 9   // Steering servo (D9)
#define SWITCH_PIN A0 // Program start switch: A0 -> button -> GND (active-LOW)

// ---------- Hardware ----------
AF_DCMotor motor(1);    
Servo steering;

// ---------- Tunables ----------
const uint16_t ECHO_TIMEOUT_US   = 27000;  
const int      MAX_STEER_DEG     = 60;
const float    KP_STEER          = 2.0f;
const float    KD_STEER          = 0.25f;
const int      BASE_SPEED        = 255;
const int      MIN_SPEED         = 140;
const int      REVERSE_SPEED     = 160;
const int      CLEAR_FAST_CM     = 40;
const int      CLEAR_SLOW_CM     = 20;
const int      FRONT_REVERSE_CM  = 10;
const int      HARD_BLOCK_CM     = 14;
const int      SIDE_PUSH_START   = 60;
const float    K_FRONT           = 1.9f;
const float    K_DIAG            = 1.7f;
const float    K_SIDE            = 1.6f;
const float    ALPHA_SMOOTH      = 0.25f;
const bool     INVERT_MOTOR      = false;

// ---------- State ----------
float dFCf=200, dFLDf=200, dFRDf=200, dLf=200, dRf=200;
float prevFy = 0.0f;

// ---------- Lap counting ----------
unsigned long lastLapTime = 0;
int lapCount = 0;
const int MAX_LAPS = 22;
const unsigned long LAP_COOLDOWN_MS = 10000;  

// ---------- Helpers ----------
long readDistanceOnce(uint8_t trig, uint8_t echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) return 400;
  return (long)(dur / 58);
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

void setSteerDeg(int delta) {
  delta = clamp(delta, -MAX_STEER_DEG, MAX_STEER_DEG);
  const int center_us = 1500;
  const float us_per_deg = 11.0f;
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

void reverseAndReadjust(bool preferRight) {
  Serial.println(F("[ACTION] Reverse & Readjust"));
  driveBackward(REVERSE_SPEED);
  delay(1500);
  driveStop();
  int steerCmd = preferRight ? +MAX_STEER_DEG : -MAX_STEER_DEG;
  setSteerDeg(steerCmd);
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
  return clamp((int)(MIN_SPEED + t * (BASE_SPEED - MIN_SPEED)), MIN_SPEED, BASE_SPEED);
}

float repulse(float d_cm, float range_cm, float k) {
  if (d_cm >= range_cm) return 0.0f;
  float s = 1.0f - (d_cm / range_cm);
  if (s < 0) s = 0;
  return k * s;
}

// ---- Program start switch (A0 -> GND) ----
void waitForStartSwitch() {
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  Serial.println(F("[SW] Hold… press & release START (A0->GND)"));
  // Wait for press (LOW)
  while (digitalRead(SWITCH_PIN) == HIGH) { delay(5); }
  delay(150);
  // Wait for release (HIGH)
  while (digitalRead(SWITCH_PIN) == LOW) { delay(5); }
  delay(150);
  Serial.println(F("[SW] START!"));
  delay(500);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // Wait here until button is tapped
  waitForStartSwitch();

  steering.attach(SERVO_PIN);
  setSteerDeg(0);
  motor.setSpeed(0);
  motor.run(RELEASE);

  // Initial reads
  dFCf  = readDistanceMedian3(TRIG_FC,  ECHO_FC);
  dFLDf = readDistanceMedian3(TRIG_FLD, ECHO_FLD);
  dFRDf = readDistanceMedian3(TRIG_FRD, ECHO_FRD);
  dLf   = readDistanceMedian3(TRIG_L,   ECHO_L);
  dRf   = readDistanceMedian3(TRIG_R,   ECHO_R);

  Serial.println(F("=== Robot Ready ==="));
}

// ---------- Main Loop ----------
void loop() {
  // Raw distances
  long dFC  = readDistanceMedian3(TRIG_FC,  ECHO_FC);
  long dFLD = readDistanceMedian3(TRIG_FLD, ECHO_FLD);
  long dFRD = readDistanceMedian3(TRIG_FRD, ECHO_FRD);
  long dL   = readDistanceMedian3(TRIG_L,   ECHO_L);
  long dR   = readDistanceMedian3(TRIG_R,   ECHO_R);

  // Smoothed
  dFCf  = ALPHA_SMOOTH * dFC  + (1 - ALPHA_SMOOTH) * dFCf;
  dFLDf = ALPHA_SMOOTH * dFLD + (1 - ALPHA_SMOOTH) * dFLDf;
  dFRDf = ALPHA_SMOOTH * dFRD + (1 - ALPHA_SMOOTH) * dFRDf;
  dLf   = ALPHA_SMOOTH * dL   + (1 - ALPHA_SMOOTH) * dLf;
  dRf   = ALPHA_SMOOTH * dR   + (1 - ALPHA_SMOOTH) * dRf;

  // ----- Sensor Debug -----
  Serial.print(F("[US] FC:"));  Serial.print(dFC);  Serial.print(F(" (sm ")); Serial.print(dFCf,1); Serial.print(")");
  Serial.print(F(" FLD:"));     Serial.print(dFLD); Serial.print(F(" (sm ")); Serial.print(dFLDf,1); Serial.print(")");
  Serial.print(F(" FRD:"));     Serial.print(dFRD); Serial.print(F(" (sm ")); Serial.print(dFRDf,1); Serial.print(")");
  Serial.print(F(" L:"));       Serial.print(dL);   Serial.print(F(" (sm ")); Serial.print(dLf,1);  Serial.print(")");
  Serial.print(F(" R:"));       Serial.print(dR);   Serial.print(F(" (sm ")); Serial.print(dRf,1);  Serial.print(")");

  // Obstacle handling
  if (dFCf < FRONT_REVERSE_CM) {
    Serial.println(F(" | EVENT: FC<10cm -> ReverseAdjust"));
    bool freerRight = (dRf > dLf);
    reverseAndReadjust(freerRight);
    return;
  }
  if (dFCf < HARD_BLOCK_CM || (dFLDf < HARD_BLOCK_CM && dFRDf < HARD_BLOCK_CM)) {
    Serial.println(F(" | EVENT: HardBlock -> ReverseAdjust"));
    bool freerRight = (dRf > dLf);
    reverseAndReadjust(freerRight);
    return;
  }

  float Fx = 0, Fy = 0;
  float fFC  = repulse(dFCf,  CLEAR_FAST_CM, K_FRONT); Fx += -fFC;
  float fFLD = repulse(dFLDf, CLEAR_FAST_CM, K_DIAG);
  float fFRD = repulse(dFRDf, CLEAR_FAST_CM, K_DIAG);
  const float c45 = 0.70710678f;
  Fx += -fFLD * c45;   Fy += -fFLD * c45;
  Fx += -fFRD * c45;   Fy += +fFRD * c45;
  float fL = repulse(dLf, SIDE_PUSH_START, K_SIDE);
  float fR = repulse(dRf, SIDE_PUSH_START, K_SIDE);
  Fy += -fL;  Fy += +fR;

  float Fy_dot = Fy - prevFy; prevFy = Fy;
  int steerDeg = (int)clamp((KP_STEER * Fy * 30.0f) + (KD_STEER * Fy_dot * 30.0f),
                            (float)-MAX_STEER_DEG, (float)MAX_STEER_DEG);
  setSteerDeg(steerDeg);

  long diagAvg = (long)((dFLDf + dFRDf) * 0.5f);
  long forwardClear = min((long)dFCf, diagAvg);
  int spd = mapToSpeed(forwardClear);
  if (Fx < -1.2f) spd = max(spd - 50, MIN_SPEED);

  // Lap counting
  if (dFCf > 160 && (millis() - lastLapTime > LAP_COOLDOWN_MS)) {
    lapCount++;
    lastLapTime = millis();
    Serial.print(F(" | LAP++ -> ")); Serial.println(lapCount);
    if (lapCount >= MAX_LAPS) {
      Serial.println(F("[FINISH] Max laps reached -> Stop"));
      driveStop();
      setSteerDeg(0);
      while (true) { }
    }
  }

  // Drive debug
  Serial.print(F(" | Fx:"));   Serial.print(Fx,2);
  Serial.print(F(" Fy:"));     Serial.print(Fy,2);
  Serial.print(F(" steer:"));  Serial.print(steerDeg);
  Serial.print(F(" spd:"));    Serial.print(spd);
  Serial.print(F(" laps:"));   Serial.println(lapCount);

  driveForward(spd);
  delay(20);
}
