/*
  WRO 2025 – Open Challenge (Uno + L293D Shield v1 + Servo + 6x HC-SR04)
  - Left-wall following with proportional steering.
  - Failsafe: reverse + steer away, then continue.
*/

#include <Servo.h>
#include <AFMotor.h>   // Adafruit Motor Shield v1 (L293D + 74HC595)

// ---------- Pin Map (from photo) ----------
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

#define SERVO_PIN 9   // Steering
// Motor M1 is on PWM D11 internally via the shield

// ---------- Objects ----------
AF_DCMotor motor(1);   // M1 on the L293D shield
Servo steering;

// ---------- Tunables ----------
const int   TARGET_CM        = 22;   // desired distance to left wall
const float KP               = 2.0;  // deg per cm error
const int   MAX_STEER_DEG    = 30;   // servo limit from center
const int   BASE_SPEED       = 180;  // 0..255
const int   SLOW_SPEED       = 140;
const int   REVERSE_SPEED    = 150;
const int   FRONT_BLOCK_CM   = 14;   // reverse trigger
const int   FRONT_CAUTION_CM = 25;   // slow down
const int   LAP_GATE_FC_CM   = 20;   // lap gate: wall ahead
const int   LAP_GATE_L_CM    = 50;   // lap gate: open left
const uint16_t ECHO_TIMEOUT  = 30000; // 30ms -> ~5m

// ---------- State ----------
int lapCount = 0;
bool inStartZone = false;
unsigned long lastLapStamp = 0;
unsigned long lastMoveStamp = 0;

// ---------- Helpers ----------
long readDistanceOnce(uint8_t trig, uint8_t echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT);
  if (dur == 0) return 400;         // no echo
  return (long)(dur / 58);          // us -> cm
}

long readDistance(uint8_t trig, uint8_t echo) {
  // median of 3 for robustness
  long a = readDistanceOnce(trig, echo);
  long b = readDistanceOnce(trig, echo);
  long c = readDistanceOnce(trig, echo);
  // sort a,b,c quickly
  if (a > b) { long t=a; a=b; b=t; }
  if (b > c) { long t=b; b=c; c=t; }
  if (a > b) { long t=a; a=b; b=t; }
  return b; // median
}

void setSteerDeg(int delta) { // -MAX..+MAX relative to center
  if (delta < -MAX_STEER_DEG) delta = -MAX_STEER_DEG;
  if (delta >  MAX_STEER_DEG) delta =  MAX_STEER_DEG;
  steering.write(90 + delta);
}

void driveForward(uint8_t spd) {
  motor.setSpeed(spd);
  motor.run(FORWARD);
}

void driveBackward(uint8_t spd) {
  motor.setSpeed(spd);
  motor.run(BACKWARD);
}

void driveStop() {
  motor.setSpeed(0);
  motor.run(RELEASE);
}

void reverseAndTurn(bool preferRight) {
  // Step 1: reverse a bit
  driveBackward(REVERSE_SPEED);
  setSteerDeg(preferRight ? +MAX_STEER_DEG : -MAX_STEER_DEG);
  delay(280);

  // Step 2: swing forward away from obstacle
  driveForward(SLOW_SPEED);
  delay(320);

  // Step 3: straighten and resume
  setSteerDeg(0);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_FC, OUTPUT);  pinMode(ECHO_FC, INPUT);
  pinMode(TRIG_FLD, OUTPUT); pinMode(ECHO_FLD, INPUT);
  pinMode(TRIG_FRD, OUTPUT); pinMode(ECHO_FRD, INPUT);
  pinMode(TRIG_L, OUTPUT);   pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);   pinMode(ECHO_R, INPUT);
  pinMode(TRIG_B, OUTPUT);   pinMode(ECHO_B, INPUT);

  steering.attach(SERVO_PIN);
  setSteerDeg(0);

  motor.setSpeed(0);
  motor.run(RELEASE);

  delay(300);
  lastMoveStamp = millis();
}

// ---------- Main Loop ----------
void loop() {
  // Read key distances
  long dFC = readDistance(TRIG_FC, ECHO_FC);
  long dL  = readDistance(TRIG_L,  ECHO_L);
  long dR  = readDistance(TRIG_R,  ECHO_R);

  // ----- Lap gate (simple pattern: wall ahead + open left) -----
  if (dFC < LAP_GATE_FC_CM && dL > LAP_GATE_L_CM && (millis() - lastLapStamp > 3000)) {
    if (!inStartZone) {
      lapCount++;
      Serial.print("Lap completed: "); Serial.println(lapCount);
      lastLapStamp = millis();
      inStartZone = true;
    }
  } else if (dFC > (LAP_GATE_FC_CM + 10)) {
    inStartZone = false;
  }

  // ----- Failsafe: reverse + turn (do NOT stop and wait) -----
  if (dFC < FRONT_BLOCK_CM) {
    bool roomOnRight = (dR > dL);  // choose freer side
    reverseAndTurn(roomOnRight);
  }

  // ----- Proportional steering on left wall -----
  int error = TARGET_CM - (int)dL;                   // +ve if too far from wall
  int steerDeg = (int)(KP * error);                  // convert to degrees
  if (steerDeg >  MAX_STEER_DEG) steerDeg =  MAX_STEER_DEG;
  if (steerDeg < -MAX_STEER_DEG) steerDeg = -MAX_STEER_DEG;
  setSteerDeg(steerDeg);

  // Speed policy: slow if close to front wall
  int spd = (dFC < FRONT_CAUTION_CM) ? SLOW_SPEED : BASE_SPEED;
  driveForward(spd);

  // ----- Mission end -----
  if (lapCount >= 3) {
    driveStop();
    setSteerDeg(0);
    // Keep printing but do not block; judges can see it's done
    static bool once = false;
    if (!once) { Serial.println("Mission complete (3 laps)."); once = true; }
    delay(200);
    return;
  }

  delay(50);
}
