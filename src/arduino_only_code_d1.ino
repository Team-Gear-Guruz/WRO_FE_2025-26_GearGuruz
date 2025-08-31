/* -----------------------------------------------------------
   Arduino Motor + Servo bridge for Raspberry Pi vision brain
   - Motor: L293/TB6612 "M1" (ENA/PWM + IN1 + IN2)
   - Servo: D9 (standard hobby servo for steering)
   - Serial protocol (newline-terminated):
        M <int>      : motor speed -255..255 (sign = direction)
        S <int>      : servo angle 0..180 (optional)
        SUS <int>    : servo microseconds 1000..2000 (preferred)
        STOP         : motor stop (PWM=0)
        PING         : respond "PONG"
   ----------------------------------------------------------- */

#include <Servo.h>

// ====== PIN MAP (edit these three if your shield differs) ======
#define PIN_M_PWM   5     // ENA / PWMA (must be PWM-capable)
#define PIN_M_IN1   11    // IN1
#define PIN_M_IN2   13    // IN2
// If you have a TB6612 with STBY pin, uncomment and set it HIGH
// #define PIN_STBY 12

#define PIN_SERVO   9     // Steering servo on D9

// ====== SERVO LIMITS (tune if needed) ======
const int SERVO_US_MIN = 1000;   // microseconds
const int SERVO_US_MID = 1500;
const int SERVO_US_MAX = 2000;

Servo steer;

void motorStop() {
  analogWrite(PIN_M_PWM, 0);
  digitalWrite(PIN_M_IN1, LOW);
  digitalWrite(PIN_M_IN2, LOW);
}

void motorWrite(int val) {
  // val: -255..255  (sign = direction)
  int pwm = val;
  if (pwm < -255) pwm = -255;
  if (pwm >  255) pwm =  255;

  if (pwm == 0) {
    motorStop();
    return;
  }

  if (pwm > 0) {
    digitalWrite(PIN_M_IN1, HIGH);
    digitalWrite(PIN_M_IN2, LOW);
    analogWrite(PIN_M_PWM, pwm);
  } else {
    digitalWrite(PIN_M_IN1, LOW);
    digitalWrite(PIN_M_IN2, HIGH);
    analogWrite(PIN_M_PWM, -pwm);
  }
}

void setup() {
  pinMode(PIN_M_PWM, OUTPUT);
  pinMode(PIN_M_IN1, OUTPUT);
  pinMode(PIN_M_IN2, OUTPUT);
#ifdef PIN_STBY
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
#endif

  steer.attach(PIN_SERVO, SERVO_US_MIN, SERVO_US_MAX);
  steer.writeMicroseconds(SERVO_US_MID);

  Serial.begin(115200);
  motorStop();
  delay(200);
  Serial.println("ARDUINO_BRIDGE_READY");
}

String line;

void loop() {
  // Read newline-terminated line
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length()) handleCommand(line);
      line = "";
    } else {
      line += c;
    }
  }
}

void handleCommand(const String& cmd) {
  // Commands:
  // "M <int>", "S <int>", "SUS <int>", "STOP", "PING"
  if (cmd == "STOP") {
    motorStop();
    Serial.println("OK");
    return;
  }
  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  if (cmd.startsWith("M ")) {
    long v = cmd.substring(2).toInt();
    if (v < -255) v = -255;
    if (v >  255) v =  255;
    motorWrite((int)v);
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("SUS ")) {
    long us = cmd.substring(4).toInt();
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    steer.writeMicroseconds((int)us);
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("S ")) {
    long ang = cmd.substring(2).toInt();
    if (ang < 0) ang = 0;
    if (ang > 180) ang = 180;
    steer.write((int)ang);
    Serial.println("OK");
    return;
  }

  Serial.println("ERR");
}