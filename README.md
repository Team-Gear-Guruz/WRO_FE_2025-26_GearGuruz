Engineering materials
====


<p align="center">
  <img src="https://raw.githubusercontent.com/ayan-atm/WRO_FE_2025-26/main/other/TeamBanner.png" alt="Team Banner" width="900"/>
</p>

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2025.

## Content of Repository

| Folder     | Description |
|------------|-------------|
| `models/`  | 3D-printable mechanical parts (sensor mounts, chassis, etc.) |
| `schemes/` | Electrical schematics and wiring diagrams |
| `src/`     | Python (Raspberry Pi) → Vision, lane following, obstacle avoidance, parking logic.<br>Arduino → Motor + servo bridge (serial protocol for PWM + steering). |
| `t-photos/`| Team photos (includes one official group photo and one funny team photo) |
| `v-photos/`| Vehicle photos (6 angles: front, back, sides, top, bottom) |
| `video/`   | Driving demo link or video files |
| `other/`   | Documentation, datasets, hardware specs, and supporting materials |

---

##  Parts List

| Component                | Quantity | Description / Role                                     |
|--------------------------|----------|--------------------------------------------------------|
| Raspberry Pi 4 (or 3B+)  | 1        | Main controller, runs Python + OpenCV vision + logic   |
| Arduino Uno              | 1        | Low-level controller for motor + steering (PWM + servo)|
| L293D / TB6612 Driver    | 1        | Motor driver IC for DC motor control                   |
| DC Gear Motor            | 1        | Rear wheel drive motor                                 |
| Servo Motor (MG996R)     | 1        | Steering actuator (mounted on front axle)              |
| HC-SR04 Ultrasonic Sensor| 6        | Obstacle detection (front, back, left, right, FL, FR)  |
| Li-on Battery 12V        | 1        | Power source for electronics and motors                |
| Motor Mounts / Chassis   | 1 set    | 3D-printed chassis parts + mounts                      |
| Wheels + Tires           | 2–4      | Drive + steering wheels                                |
| Wires, Jumper cables     | —        | Connections between Pi, Arduino, sensors, and driver   |
| Breadboard / PCB         | 1        | For stable wiring and connections                      |
| Camera (Pi Camera / USB) | 1        | Used by Raspberry Pi for lane + obstacle detection     |
| Motor 12v DC             | 1        | For Moving the robot (connected to single axle)     |

<!-- Parts Gallery -->
<h2> Photos of parts</h2>

<table>
  <tr>
    <td align="center">
      <img src="other/PCB.jpeg" width="220" alt="PCB"><br/>
      <sub><b>PCB</b></sub>
    </td>
    <td align="center">
      <img src="other/Ball%20bearing%2020mmx40mm.jpg" width="220" alt="Ball bearing 1"><br/>
      <sub><b>Ball Bearing 1</b></sub>
    </td>
    <td align="center">
      <img src="other/Ball%20bearing%202.jpg" width="220" alt="Ball bearing 2"><br/>
      <sub><b>Ball Bearing 2</b></sub>
    </td>
    <td align="center">
      <img src="other/Motor%20Shield.webp" width="220" alt="Motor Shield"><br/>
      <sub><b>Motor Shield</b></sub>
    </td>
  </tr>
</table>

## Team members

Ayan Atmakuri, age 16, atmakuriayan@gmail.com 
(Raspberry Pi Programming)

Kushal Khemani, age 16, kushal.khemani@gmail.com
(Arduino Programming, 3D Modeling, Building, PCB)

## Team Photos

<img src="https://github.com/ayan-atm/WRO_FE_2025-26/raw/main/t-photos/Official%20Picture.jpeg" width="40%" height="40%"> <img src="https://github.com/ayan-atm/WRO_FE_2025-26/raw/main/t-photos/Funny%20Picture.png" width="40%" height="40%">

Ayan Atmakuri (Purple shirt), Kushal Khemani (Grey shirt with green collar)

##  Quick Overview

A vision-guided mini-vehicle using:

- **Raspberry Pi**: Computer vision (OpenCV), control logic (PD + FSM)  
- **Arduino Uno**: Motor and servo actuation, obstacle feedback  
- **Simple Serial Protocol**: Commands like `M <int>`, `SUS <us>`, `STOP`, `PING`  
- **Features**: Lane following, obstacle avoidance, auto-parking, HSV tuning, simulator.

# Introduction

This repository hosts the software and wiring for an autonomous robot car built for the **World Robot Olympiad 2025 – Future Engineers Challenge**.

The design leverages a **Raspberry Pi** for high-level perception and decision-making—handling lane detection, obstacle avoidance, color-coded behavior (e.g., pass red on the right, green on the left), and vision-guided parking—while an **Arduino Uno** handles real-time actuation of the DC drive motor and steering servo based on those decisions. This documentation provided a step-by-step process on how you can implement your own robot.

### System Workflow - We use this to set ground rules

1. **Vision & Perception (Raspberry Pi)**  
   - Captures camera frames and processes them for road lanes, colored markers, and parking bays.  
   - Runs a state machine with PD control logic, lap counting, and parking alignment commands.

2. **Control Communication**  
   - Sends steering (e.g., servo angle) and throttle (motor PWM) commands via serial to the Arduino.

3. **Motion Execution (Arduino)**  
   - Receives commands and outputs PWM for the motor driver (L293D/TB6612) and servo control for steering.

4. **Real World Performance**  
   - Car autonomously navigates laps, avoids obstacles, obeys color-based passing rules, executes turnarounds, and completes vision-guided parallel parking.



## 1. Mobility Management - finding what suits best 

Our vehicle’s mobility system has been engineered for robust, stable navigation across both Open and Obstacle Challenges in WRO 2025. The setup balances power, control precision, and sensor integration using reliable, competition-grade components.

### Motor Selection and Implementation

We selected a **12V DC gear motor rated at 300 RPM** for propulsion. The decision was based on:

* **High torque output**, necessary for low-speed maneuvers and quick directional changes
* Sufficient RPM to maintain speed while tracking lines or navigating walls
* Compatibility with the **L293D motor driver shield**, which allows:

  * PWM-based speed control via **digital pin D11**
  * Direction control via built-in 74HC595 shift register (internal use of D4, D7, D8, D12)

Only **Motor M1** on the shield is used, simplifying wiring and reducing interference with ultrasonic TRIG/ECHO pins.

### Steering Mechanism

Steering is handled by an **MG996R high-torque servo motor**, chosen for its:

* **Superior torque (up to 10kg.cm)**, ideal for precise control of front-wheel pivot steering
* Robust metal gears suitable for extended use and minor shocks
* Faster response time compared to standard 9g hobby servos

The servo is powered via a **dedicated buck converter set at 5V**, ensuring:

* Stable power supply without drawing current from the Arduino
* Protection against brownout during high-load movements

The servo signal is connected to **digital pin D9** and is controlled via the `Servo.h` library.

### Chassis and Component Mounting

The chassis is a **custom design** with cutouts and supports for:

* Arduino Uno + L293D Shield (stacked)
* Servo motor mounted at the front axle for steering
* Power bank or Li-ion battery for 12V supply
* Buck converter securely fixed for servo
* Ultrasonic sensors (6x) around the perimeter

The mounting design ensures:

* **Center of gravity remains low**, aiding stability
* **Balanced front-rear weight** to avoid drift or oversteer
* **Mechanical isolation** for sensor mounts to avoid vibration interference

### Engineering Principles Applied

* **Torque over speed**: The 300 RPM motor offers sufficient torque for all obstacle scenarios and allows smooth lap completion.
* **Proportional control via PWM** for both speed and steering adjustments.
* **Power isolation**: Servo draws from a separate buck, preventing logic voltage drops on Arduino.

Failsafe routines in the software monitor obstacle proximity. If all sensor readings drop below safe limits, the car **automatically reverses and adjusts angle** to prevent collision or deadlock.


## 2. Power and Sense Management - trying to be energy efficient

To enable robust autonomous operation across both the Open and Obstacle Challenges, our power and sensing systems were designed to be modular, efficient, and fail-safe. Key design goals included isolating high-power and logic domains, ensuring consistent voltage supply for sensitive components, and providing wide-angle situational awareness through sensor fusion.

---

### Power Management

#### Power Source:

The entire vehicle is powered by a **12V Li-ion battery**, chosen for its:

* High energy density
* Lightweight form factor
* Rechargeability
* Ability to sustain current surges required by motors and electronics

The battery feeds both high-power components and regulated subsystems via a **step-down (buck) converter**.

#### Voltage Regulation:

* **DC Motor (12V)**: Powered directly from the battery through an L293D motor shield.
* **Servo Motor (MG996R, 5V)**: Powered via a dedicated buck converter to ensure stable current and prevent voltage dips under load.
* **Arduino Uno & Logic Circuits**: Supplied via the 5V rail, either from USB or regulated output from the buck.
* **Raspberry Pi** (Obstacle Challenge): Powered separately or through a second buck converter, ensuring isolation from motor spikes.

This separation of power rails helps avoid brownouts, particularly during servo actuation or rapid motor speed changes. A common ground is maintained to ensure reliable sensor readings.

#### Safety Features:

* On/off switch to isolate the main supply
* Fuse protection inline with the Li-ion output
* Reverse polarity and surge protection via onboard diode


###  Sense Management

#### Sensor System:

The vehicle uses **six ultrasonic distance sensors (HC-SR04)** strategically positioned to cover:

* Forward path detection
* Side wall following and obstacle clearance
* Rear proximity awareness

These sensors were selected based on:

* Proven reliability and accuracy (2–400 cm range)
* Low cost and easy integration with Arduino
* Minimal power consumption (\~15 mA per sensor)

#### Sensor Strategy:

* **Open Challenge**: The sensors guide wall-following behavior and lap-count detection using distance thresholds and orientation cues.
* **Obstacle Challenge**: The sensors serve as secondary proximity detection, while the **Raspberry Pi + camera** handles traffic signs, colored zones, and advanced navigation through computer vision.

#### Power Consumption Considerations:

All sensors operate on 5V logic. Since each sensor draws minimal current, they are powered directly from the Arduino 5V output. The servo's power-hungry nature warranted a dedicated 5V supply via buck converter to avoid interference.


## 3. Obstacle Management - the main challenge

Obstacle management in our self-driving vehicle focuses on real-time detection, decision-making, and recovery strategies to navigate both static and dynamic obstacles with minimal latency and high reliability.

The only solution - a **hybrid approach**:

* **Ultrasonic sensors (Arduino-controlled)** for reactive proximity-based obstacle avoidance
* **Camera + Raspberry Pi (OpenCV)** for computer vision tasks such as detecting stop signs, colored zones, and “No Entry” paths

---

### Strategy Overview - what we found after a few long discussions

#### **Open Challenge (Arduino-only):**

* Use ultrasonic sensors to detect walls and gaps
* Follow the left wall to maintain direction
* Avoid close-range collisions using threshold values
* Perform a lap-count using sensor fusion
* Maintain consistent motion unless a fail-safe triggers

#### **Obstacle Challenge (Raspberry Pi + Arduino):**

* Raspberry Pi processes camera input for traffic sign recognition
* On detecting a STOP sign, Pi sends a signal to Arduino to pause
* If a colored zone or arrow sign is detected, Pi sends directional decisions
* Arduino reacts with real-time motor and steering commands
* Failsafes include “no echo” situations, timeout recovery, and realignment

---

### Flow Diagram - easy to interpret

```
[Start]
   |
   v
[Sensor Calibration]
   |
   v
[Wall Following Enabled] ←-------------------------\
   |                                              |
   v                                              |
[Obstacle Detected?] --Yes--> [Decision: Stop/Turn/Slow]
   |                                              |
  No                                              v
   |                                  [Resume Wall Following]
   v                                              |
[Zone Entry Detected?] --Yes--> [Trigger Zone Handling Logic]
   |                                              |
  No                                              v
   |                                  [Lap Count or Finish?]
   v                                              |
[Continue Movement] -----------------------------/
```

---

### Pseudocode (Obstacle Challenge) - we learnt this last year

```plaintext
Start
→ Initialize Arduino and Pi communication
→ Start camera stream and ultrasonic sensors
→ While lap count < 3:
    - Use ultrasonic sensors for wall following
    - If front obstacle distance < 15 cm:
        - Stop or steer away
    - If Pi detects STOP sign:
        - Pause 3 seconds
    - If Pi detects “turn right”:
        - Send servo signal to turn
    - If Pi detects color zone:
        - Change speed / behavior
→ After 3 laps, stop car
```

---

### Arduino Code Snippet (Obstacle Handling Logic) - OOP

// Inside loop()

```cpp
long distFC = readDistance(TRIG_FC, ECHO_FC);
long distL = readDistance(TRIG_L, ECHO_L);
long distR = readDistance(TRIG_R, ECHO_R);

// Wall following logic
if (distL < 15) {
  turnRight();
} else if (distL > 30) {
  turnLeft();
} else {
  goStraight();
}

// Obstacle ahead
if (distFC < 12) {
  stopCar();
  delay(800);
  turnRight();  // Basic evasive action
  delay(400);
}
```

---

### Raspberry Pi Role (Computer Vision)

The Pi runs OpenCV-based code to:

* Detect red signs
* Recognize arrows for directional instructions
* Identify colored zones using HSV masks

After classification, the Pi sends command signals to Arduino via serial:

* `"STOP"`, `"LEFT"`, `"RIGHT"`, etc.

The Arduino interprets these and adjusts movement accordingly.

---

### Failsafe Mechanisms

* **No echo recovery**: If no echo is received from a sensor for more than 3 reads, the car slows down and centers.
* **Stuck detection**: If the car hasn't moved in distance (based on rear sensors), it reverses for 1 second and retries.
* **Soft timeout**: If a lap takes longer than 2 minutes, the system logs an alert.
* **Reversing**: The car reverses if it is less than 15 cm from an obstacle.



# Arduino Pin Mapping for Self-Driving Car (WRO 2025) 

**This is what we decided upon...**

### 🔧 Motor (M1 via L293D Shield)
- **Speed (PWM):** `D11`  
- **Direction:** Handled internally by the shield’s 74HC595  
  *(Uses internal pins D4, D7, D8, D12 — do not control directly)*

---

### Servo Motor
- **Signal Pin:** `D9`

---

### Ultrasonic Sensors (6 Total)
> Echo lines are placed on analog pins `A0–A5` for cleaner signal and to free digital pins.  
> Trigger lines are assigned to available digital pins.


# Unit Testing 

These are standalone Arduino sketches to test individual subsystems (servo, motor, ultrasonic sensors) before integrating them together.  
Each can be copied into the Arduino IDE and uploaded separately.



### 1) Servo Sweep (D9)

Moves the steering **servo** smoothly from 0°→180°→0°.  
⚠️ **Important:** MG996R needs its own 5–6 V supply (≥3 A). Do not power from the Arduino 5 V pin.
Errors I made - did not use a buck converter to supply constant voltage and amps. This caused stuttering in the steering.

```cpp
#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);   // Servo signal on D9 (power from external 5–6 V)
}

void loop() {
  // Sweep 0° -> 180°
  for (int pos = 0; pos <= 180; pos += 5) {
    myServo.write(pos);
    delay(50);
  }
  // Sweep back 180° -> 0°
  for (int pos = 180; pos >= 0; pos -= 5) {
    myServo.write(pos);
    delay(50);
  }
}

```
### 2) DC Motor on L293D Shield (M1)

Drives the M1 motor channel forward, stop, reverse, stop.

Note: generic L293D shields vary. This code assumes M1_DIR on D12 and M1_PWM on D3.
If your shield uses different pins, update the defines.
Errors I made - used the pins on the arduino that the motor shield was dependent on.
```cpp
// Test Motor at M1 on L293D Shield
int M1_DIR = 12;  // Direction pin for M1 (check your shield!)
int M1_PWM = 3;   // PWM pin for M1 speed

void setup() {
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
}

void loop() {
  // Forward
  digitalWrite(M1_DIR, HIGH);
  analogWrite(M1_PWM, 200);  // Speed (0-255)
  delay(2000);

  // Stop
  analogWrite(M1_PWM, 0);
  delay(1000);

  // Reverse
  digitalWrite(M1_DIR, LOW);
  analogWrite(M1_PWM, 200);
  delay(2000);

  // Stop
  analogWrite(M1_PWM, 0);
  delay(1000);
}
```  
### 3) Six Ultrasonic Sensors (HC-SR04)

Reads Front, Back, Left, Right, Diagonal-Front-Left, Diagonal-Front-Right.
All sensors share VCC → 5 V and GND → GND, each has its own TRIG/ECHO pins.
Errors I made - connecting the trig pins in one digital pin (for sequential reading). This made the program complex and led to crashing.
```cpp
// ------------------- Pin mappings -------------------
#define TRIG_F   2
#define ECHO_F   4

#define TRIG_B   5
#define ECHO_B   6

#define TRIG_L   7
#define ECHO_L   8

#define TRIG_R   A0   // D14
#define ECHO_R   A1   // D15

#define TRIG_DFL A2   // D16
#define ECHO_DFL A3   // D17

#define TRIG_DFR A4   // D18
#define ECHO_DFR A5   // D19
// -----------------------------------------------------

long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout
  if (duration == 0) return 400; // no echo → max range
  return duration / 58;          // convert µs to cm
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_F, OUTPUT);   pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT);   pinMode(ECHO_B, INPUT);
  pinMode(TRIG_L, OUTPUT);   pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);   pinMode(ECHO_R, INPUT);
  pinMode(TRIG_DFL, OUTPUT); pinMode(ECHO_DFL, INPUT);
  pinMode(TRIG_DFR, OUTPUT); pinMode(ECHO_DFR, INPUT);

  Serial.println("Ultrasonic Test: F, B, L, R, DFL, DFR");
}

void loop() {
  long distF   = readDistance(TRIG_F,   ECHO_F);
  long distB   = readDistance(TRIG_B,   ECHO_B);
  long distL   = readDistance(TRIG_L,   ECHO_L);
  long distR   = readDistance(TRIG_R,   ECHO_R);
  long distDFL = readDistance(TRIG_DFL, ECHO_DFL);
  long distDFR = readDistance(TRIG_DFR, ECHO_DFR);

  Serial.print("F: ");   Serial.print(distF);   Serial.print("  ");
  Serial.print("B: ");   Serial.print(distB);   Serial.print("  ");
  Serial.print("L: ");   Serial.print(distL);   Serial.print("  ");
  Serial.print("R: ");   Serial.print(distR);   Serial.print("  ");
  Serial.print("DFL: "); Serial.print(distDFL); Serial.print("  ");
  Serial.print("DFR: "); Serial.println(distDFR);

  delay(100);
}
```  
⚠️ Gotchas

Servo power: MG996R is high-torque; use a separate 5–6 V buck (≥3–5 A).

Motor shield pinouts: Check your shield silkscreen; some use D11/D3 for M1.

Grounding: Arduino GND, motor shield GND, buck GND, servo GND, and sensor GND must all be connected together.

Ultrasonic max range: 400 cm in the serial monitor means “no object detected.”

### 4) Modular Testing Code - because why not.
This code will involve all components required to run the robot in the first round.

It’s menu-driven over the Serial Monitor:

Press 1 → Servo sweep test

Press 2 → Motor forward/stop/reverse test

Press 3 → Live ultrasonic scan (all six)

Press 4 → Simple safety drive (forward, stop if front < 25 cm)

Press s → Stop motor immediately

Press h → Help menu

```cpp
/*
  Modular Test Suite: Motor (L293D M1) + Servo (D10) + 6x Ultrasonic
  - Board: Arduino Uno + Generic L293D Motor Shield
  - Motor: M1 (AFMotor abstracts shield pins)
  - Servo: MG996R signal on D10 (POWER FROM 5–6 V BUCK, not Arduino 5V!)
  - Ultrasonic: 6x HC-SR04, shared 5V/GND, individual TRIG/ECHO pins

  Controls (Serial Monitor @115200, "No line ending"):
    1 : Servo sweep
    2 : Motor test (FWD, STOP, REV)
    3 : Ultrasonic live scan (all 6)
    4 : Safety drive (forward; stop if front < 25 cm)
    s : Stop motor now
    h : Help
*/

#include <AFMotor.h>
#include <Servo.h>

// ------------------- Motor (L293D shield, M1) -------------------
AF_DCMotor motor1(1);   // M1 port on the shield

// ------------------- Servo (signal on D10) ----------------------
Servo steering;
const int SERVO_PIN = 10;
const int SERVO_CENTER = 90;
const int SERVO_SWEEP = 40; // ±40° from center

// ------------------- Ultrasonic pins ----------------------------
#define TRIG_F   2
#define ECHO_F   4

#define TRIG_B   5
#define ECHO_B   6

#define TRIG_L   7
#define ECHO_L   8

#define TRIG_R   A0   // D14
#define ECHO_R   A1   // D15

#define TRIG_DFL A2   // D16
#define ECHO_DFL A3   // D17

#define TRIG_DFR A4   // D18
#define ECHO_DFR A5   // D19

// ------------------- Modes --------------------------------------
enum Mode : uint8_t {
  IDLE = 0,
  SERVO_SWEEP_MODE,
  MOTOR_TEST_MODE,
  ULTRA_SCAN_MODE,
  SAFETY_DRIVE_MODE
};

Mode mode = IDLE;

// ------------------- Helpers ------------------------------------
long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // timeout 30ms
  if (duration == 0) return 400; // no echo within window
  return duration / 58;          // µs → cm
}

struct Distances {
  long F, B, L, R, DFL, DFR;
};

Distances readAll() {
  Distances d;
  d.F   = readDistanceCM(TRIG_F,   ECHO_F);
  d.B   = readDistanceCM(TRIG_B,   ECHO_B);
  d.L   = readDistanceCM(TRIG_L,   ECHO_L);
  d.R   = readDistanceCM(TRIG_R,   ECHO_R);
  d.DFL = readDistanceCM(TRIG_DFL, ECHO_DFL);
  d.DFR = readDistanceCM(TRIG_DFR, ECHO_DFR);
  return d;
}

void printAll(const Distances& d) {
  Serial.print("F: ");   Serial.print(d.F);   Serial.print("  ");
  Serial.print("B: ");   Serial.print(d.B);   Serial.print("  ");
  Serial.print("L: ");   Serial.print(d.L);   Serial.print("  ");
  Serial.print("R: ");   Serial.print(d.R);   Serial.print("  ");
  Serial.print("DFL: "); Serial.print(d.DFL); Serial.print("  ");
  Serial.print("DFR: "); Serial.println(d.DFR);
}

// ------------------- Motor control wrappers ---------------------
void motorStop() {
  motor1.run(RELEASE);
}

void motorForward(uint8_t speed255) {
  motor1.setSpeed(speed255);
  motor1.run(FORWARD);
}

void motorBackward(uint8_t speed255) {
  motor1.setSpeed(speed255);
  motor1.run(BACKWARD);
}

// ------------------- UI -----------------------------------------
void printHelp() {
  Serial.println(F("\n=== Test Suite Controls ==="));
  Serial.println(F("1 : Servo sweep"));
  Serial.println(F("2 : Motor test (forward/stop/reverse)"));
  Serial.println(F("3 : Ultrasonic live scan"));
  Serial.println(F("4 : Safety drive (stop if front < 25 cm)"));
  Serial.println(F("s : Stop motor"));
  Serial.println(F("h : Help\n"));
}

void setMode(Mode m) {
  mode = m;
  switch (mode) {
    case IDLE:               Serial.println(F("[MODE] IDLE")); break;
    case SERVO_SWEEP_MODE:   Serial.println(F("[MODE] SERVO SWEEP")); break;
    case MOTOR_TEST_MODE:    Serial.println(F("[MODE] MOTOR TEST")); break;
    case ULTRA_SCAN_MODE:    Serial.println(F("[MODE] ULTRASONIC SCAN")); break;
    case SAFETY_DRIVE_MODE:  Serial.println(F("[MODE] SAFETY DRIVE")); break;
  }
}

// ------------------- Setup --------------------------------------
void setup() {
  Serial.begin(115200);

  // Servo
  steering.attach(SERVO_PIN);
  steering.write(SERVO_CENTER);

  // Motor
  motorStop();           // ensure stopped
  motor1.setSpeed(0);

  // Ultrasonic pinModes
  pinMode(TRIG_F, OUTPUT);   pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT);   pinMode(ECHO_B, INPUT);
  pinMode(TRIG_L, OUTPUT);   pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);   pinMode(ECHO_R, INPUT);
  pinMode(TRIG_DFL, OUTPUT); pinMode(ECHO_DFL, INPUT);
  pinMode(TRIG_DFR, OUTPUT); pinMode(ECHO_DFR, INPUT);

  printHelp();
  setMode(IDLE);
}

// ------------------- Mode state vars ----------------------------
unsigned long t0 = 0;
int sweepDir = +1;
int servoPos = SERVO_CENTER;

// For motor test steps
uint8_t motorStep = 0;
unsigned long motorTimer = 0;

// ------------------- Loop ---------------------------------------
void loop() {
  // ---- Serial command handling ----
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') setMode(SERVO_SWEEP_MODE);
    else if (c == '2') { motorStep = 0; motorTimer = 0; setMode(MOTOR_TEST_MODE); }
    else if (c == '3') setMode(ULTRA_SCAN_MODE);
    else if (c == '4') setMode(SAFETY_DRIVE_MODE);
    else if (c == 's') { motorStop(); Serial.println(F("[MOTOR] STOP")); }
    else if (c == 'h') printHelp();
  }

  // ---- Mode behaviors ----
  switch (mode) {
    case IDLE: {
      // idle: keep things safe
      motorStop();
      steering.write(SERVO_CENTER);
    } break;

    case SERVO_SWEEP_MODE: {
      // Non-blocking sweep: update every 20 ms
      unsigned long now = millis();
      if (now - t0 >= 20) {
        t0 = now;
        servoPos += sweepDir * 2;  // speed of sweep
        if (servoPos >= SERVO_CENTER + SERVO_SWEEP) { servoPos = SERVO_CENTER + SERVO_SWEEP; sweepDir = -1; }
        if (servoPos <= SERVO_CENTER - SERVO_SWEEP) { servoPos = SERVO_CENTER - SERVO_SWEEP; sweepDir = +1; }
        steering.write(servoPos);
      }
    } break;

    case MOTOR_TEST_MODE: {
      // Step through: FWD 2s → STOP 1s → REV 2s → STOP 1s → repeat
      unsigned long now = millis();
      if (motorStep == 0) {
        motorForward(200);
        Serial.println(F("[MOTOR] FORWARD @200"));
        motorTimer = now;
        motorStep = 1;
      } else if (motorStep == 1 && now - motorTimer >= 2000) {
        motorStop();
        Serial.println(F("[MOTOR] STOP"));
        motorTimer = now;
        motorStep = 2;
      } else if (motorStep == 2 && now - motorTimer >= 1000) {
        motorBackward(200);
        Serial.println(F("[MOTOR] BACKWARD @200"));
        motorTimer = now;
        motorStep = 3;
      } else if (motorStep == 3 && now - motorTimer >= 2000) {
        motorStop();
        Serial.println(F("[MOTOR] STOP"));
        motorTimer = now;
        motorStep = 4;
      } else if (motorStep == 4 && now - motorTimer >= 1000) {
        motorStep = 0; // loop again
      }
    } break;

    case ULTRA_SCAN_MODE: {
      // Print all sensor distances ~10 Hz
      static unsigned long lastPrint = 0;
      unsigned long now = millis();
      if (now - lastPrint >= 100) {
        lastPrint = now;
        Distances d = readAll();
        printAll(d);
      }
    } break;

    case SAFETY_DRIVE_MODE: {
      // Drive forward slowly; stop if front < 25 cm
      static unsigned long lastChk = 0;
      unsigned long now = millis();
      if (now - lastChk >= 100) {
        lastChk = now;
        long f = readDistanceCM(TRIG_F, ECHO_F);
        Serial.print(F("Front(cm): ")); Serial.println(f);
        if (f < 25) {
          motorStop();
          Serial.println(F("[SAFETY] Obstacle close → STOP"));
        } else {
          motorForward(140); // gentle cruise
        }
      }
      // Keep servo centered during this simple test
      steering.write(SERVO_CENTER);
    } break;
  }
}


```

  Wiring Notes:
  - Motor: connect to M1 on L293D shield; supply 6–12 V to shield VM/EXT PWR; GND common with Arduino.
  - Servo: D10 signal; POWER from separate 5–6 V buck (≥3–5 A). Tie buck GND to Arduino GND.
  - Ultrasonic: All VCC → 5 V, all GND → GND. TRIG/ECHO as defined above.
  - Common Ground: Battery –, shield GND, Arduino GND, both buck GNDs, sensor GND, servo GND MUST be common.



# Arduino Code – Square Track 3-Lap Challenge

This Arduino code powers a self-driving robot car to autonomously complete **3 laps on a square track** that contains both **interior and exterior walls**. It uses **6 ultrasonic sensors** to detect obstacles and implements a **left-wall-following algorithm** for navigation.

---

## Navigation Strategy

- **Wall Following Rule:** Left-hand rule – always keep the left wall within a specific distance range.
- **Obstacle Avoidance:** Uses `Front-Center`, `Left`, and `Right` sensors to detect and steer around walls.
- **Lap Detection:** When the robot re-enters a specific sensor pattern near the start zone (e.g., wall in front but no wall on left), a lap is counted.
- **Goal:** Complete 3 full laps and stop.

---

## Pin Configuration

| Component             | TRIG (Digital) | ECHO (Analog) |
|----------------------|----------------|---------------|
| Front-Center (FC)    | D2             | A0            |
| Front-Left Diagonal  | D3             | A1            |
| Front-Right Diagonal | D5             | A2            |
| Left (L)             | D6             | A3            |
| Right (R)            | D10            | A4            |
| Back (B)             | D13            | A5            |

- **Motor (M1):** PWM on D11 (Direction handled by L293D shield)
- **Servo:** Signal on D9

---

## Arduino Code with Explanation

### 1. Pin Mapping and Includes

We define all the pin assignments and include the Servo library to control the steering mechanism.

```cpp
#define TRIG_FC 2
#define ECHO_FC A0
#define TRIG_FLD 7
#define ECHO_FLD A1
#define TRIG_FRD 8
#define ECHO_FRD A2
#define TRIG_L 10
#define ECHO_L A3
#define TRIG_R 12
#define ECHO_R A4
#define TRIG_B 13
#define ECHO_B A5

#define MOTOR_PWM 11
#define SERVO_PIN 9
#define STBY 4 // used internally by the shield

#include <Servo.h>
````

---

### 2. Global Variables and Distance Reader

* `lapCount`: Tracks completed laps.
* `inStartZone`: Used to avoid double-counting laps.
* `readDistance()`: Generic ultrasonic reader using TRIG/ECHO logic.

```cpp
Servo steering;

int lapCount = 0;
bool inStartZone = false;
unsigned long lastLapTime = 0;

long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 400;
  return duration / 58;
}
```

---

### 3. Basic Movement Commands

These functions control the car's direction and speed using the motor and servo.

```cpp
void moveForward() {
  analogWrite(MOTOR_PWM, 180);  // Set motor speed
}

void turnLeft() {
  steering.write(120);  // Servo turn left
}

void turnRight() {
  steering.write(60);   // Servo turn right
}

void goStraight() {
  steering.write(90);   // Servo center
}

void stopCar() {
  analogWrite(MOTOR_PWM, 0);  // Stop motor
}
```

---

### 4. Setup Routine

Sets pin modes and prepares the motor and servo.

```cpp
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_FC, OUTPUT); pinMode(ECHO_FC, INPUT);
  pinMode(TRIG_FLD, OUTPUT); pinMode(ECHO_FLD, INPUT);
  pinMode(TRIG_FRD, OUTPUT); pinMode(ECHO_FRD, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);

  pinMode(MOTOR_PWM, OUTPUT);
  steering.attach(SERVO_PIN);
  goStraight();  // Default orientation
}
```

---

### 5. Main Navigation Loop

The loop reads distances and makes steering decisions. It also monitors lap count based on a “start zone” condition.

```cpp
void loop() {
  long distFC = readDistance(TRIG_FC, ECHO_FC);
  long distL = readDistance(TRIG_L, ECHO_L);
  long distR = readDistance(TRIG_R, ECHO_R);
```

#### 🚩 Lap Detection Logic

This logic ensures that a lap is only counted when the robot passes through a specific condition: wall ahead but no left wall.

```cpp
  if (distFC < 20 && distL > 50 && (millis() - lastLapTime > 3000)) {
    if (!inStartZone) {
      lapCount++;
      Serial.print("Lap Completed: "); Serial.println(lapCount);
      lastLapTime = millis();
      inStartZone = true;
    }
  } else if (distFC > 30) {
    inStartZone = false;
  }
```

#### 🧭 Wall-Following Steering

Uses distance from the left wall to steer the car.

```cpp
  if (distL < 15) {
    turnRight();  // Too close to wall
  } else if (distL > 30) {
    turnLeft();   // Too far from wall
  } else {
    goStraight();
  }

  moveForward();  // Keep moving
```

#### 🛑 Stop After 3 Laps

```cpp
  if (lapCount >= 3) {
    stopCar();
    while (true) {
      Serial.println("Mission Complete!");
      delay(1000);
    }
  }

  delay(100);  // Loop pacing
}
```

---

## 🛠️ Customization Tips

| Goal                       | What to Adjust                         |
| -------------------------- | -------------------------------------- |
| More precise turning       | Tune `steering.write()` values         |
| Better lap detection       | Modify `distFC` and `distL` thresholds |
| Change wall-following side | Replace `distL` logic with `distR`     |
| Avoid diagonal corners     | Use FLD / FRD for early turns          |

---

## Optional Enhancements

* Add line sensors or camera for finish line detection
* Use PID for better steering control
* Switch to right-wall following by updating logic

---
FULL CODE (With Revisons)- 
```cpp
/*
  WRO 2025 – Open Challenge (Uno + L293D Shield v1 + Servo + 6x HC-SR04)
  - Pin map follows the user's photo.
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


```
## **Now we shall be using a Raspberry Pi too.**

_**NOTE**_ From Kushal Khemani - This robot was made in less than 3 weeks (due to school examinations and SAT prep for both team members).

# Getting Started

Ayan and I had absolutely no idea how to use a Raspberry Pi when we first started working with it. At the beginning, even basic tasks like flashing the OS onto an SD card or connecting it to a monitor felt overwhelming. To figure things out, we relied on a few YouTube tutorials that broke the process down step by step. Slowly, by following along with these resources, we gained confidence and were able to start experimenting with the Pi ourselves.

**Helpful videos we used:**

1. [Guide on setting up a Raspberry Pi 4](https://www.youtube.com/watch?v=ntaXWS8Lk34)
2. [Installing Raspberry Pi OS](https://www.youtube.com/watch?v=ntLJmHOJ0ME)
3. [Beginner-friendly tutorial on GPIO pins](https://www.youtube.com/watch?v=H1lxZweM52U)

Since I had just moved houses and all my electronics were still at my old place, I had to figure out how to use the Raspberry Pi without a keyboard or monitor. I ended up setting it up for headless operation, which means accessing it remotely from my laptop over Wi-Fi. By enabling SSH and using a tool like PuTTY or the built-in terminal, I could control the Pi entirely from my computer. This allowed me to run commands, install software, and experiment with projects without needing the physical peripherals. It was a lifesaver while I waited to get my setup back from the old house.

This is what we did - 

### **Step 1: Download Raspberry Pi OS**

1. Go to the [Raspberry Pi Software page](https://www.raspberrypi.com/software/).
2. Download **Raspberry Pi OS Lite** (no desktop required, smaller and faster for headless use).

---

### **Step 2: Flash the OS to the SD Card**

1. Download and install **Raspberry Pi Imager** or **Balena Etcher** on your computer.
2. Insert your SD card.
3. Open the imager/etcher → select the OS image → select the SD card → flash.
4. Wait until the process finishes.

---

### **Step 3: Enable SSH**

1. After flashing, open the SD card folder on your computer.
2. In the **root directory** (the main folder), create a blank file named:

```
ssh
```

(no extension, all lowercase).
3\. This will allow you to remotely access the Pi using SSH.

---

### **Step 4: Connect to Wi-Fi**

1. In the same SD card root directory, create a file called:

```
wpa_supplicant.conf
```

2. Open it with a text editor and add:

```conf
country=IN
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="YOUR_WIFI_SSID"
    psk="YOUR_WIFI_PASSWORD"
    key_mgmt=WPA-PSK
}
```

3. Replace `YOUR_WIFI_SSID` and `YOUR_WIFI_PASSWORD` with your network info.
4. Save the file. This will let your Pi connect to Wi-Fi on boot.

---

### **Step 5: Boot the Raspberry Pi**

1. Insert the SD card into the Pi.
2. Connect power.
3. Wait 2–3 minutes for it to boot and connect to Wi-Fi.

---

### **Step 6: Find the Pi’s IP Address**

* Option 1: Log into your router and check connected devices.
* Option 2: Use a phone app like **Fing** to scan your Wi-Fi network.
* Look for a device named `raspberrypi`. Note the IP address (e.g., `192.168.1.5`).

---

### **Step 7: Connect via SSH**

* **On Windows:** Use [PuTTY](https://www.putty.org/)

  1. Open PuTTY → enter the Pi’s IP → Port = 22 → SSH → Open.
  2. Login with username: `pi`, password: `raspberry`.

* **On Mac/Linux:** Open Terminal and type:

```bash
ssh pi@<Pi_IP_address>
```

(e.g., `ssh pi@192.168.1.5`)

* Enter password: `raspberry`.

---

### **Step 8: Update Your Pi**

Once connected, run:

```bash
sudo apt update
sudo apt upgrade -y
```

This ensures your Pi has the latest software.

## Setting up VSCode


### **Step 1: Install VS Code and Remote SSH Extension**

1. Make sure **Visual Studio Code** is installed on your computer: [VS Code Download](https://code.visualstudio.com/).
2. Open VS Code → go to **Extensions (Ctrl+Shift+X)** → search for **Remote - SSH** → install it.

---

### **Step 2: Connect VS Code to Raspberry Pi via SSH**

1. Press `F1` → type **Remote-SSH: Connect to Host…** → select **Add New SSH Host**.
2. Enter your SSH connection:

```bash
ssh pi@<Pi_IP_address>
```

Example: `ssh pi@192.168.1.5`
3\. Choose the default SSH config file location when prompted.
4\. Once added, select the host → VS Code will open a new window connected to your Pi.
5\. Enter password: `raspberry`.

Now your VS Code is **directly running on the Pi**, even though it’s headless.

---

### **Step 3: Open or Create a Project**

1. In the new VS Code window, go to **File → Open Folder** → navigate to a folder on your Pi (e.g., `/home/pi/projects`).
2. You can now **create new Python, C++, or Arduino files** here.

---

### **Step 4: Upload/Run Code**

* Any file you save in VS Code is automatically on your Pi.
* To run Python code, open the terminal in VS Code (\`Ctrl+\`\`) and run:

```bash
python3 myfile.py
```

* For C++ or other languages, you can compile/run the code directly in the terminal.

---

### **Step 5 (Optional): Sync Local Folder to Pi**

If you want to edit files locally and upload automatically:

1. Install **SFTP extension** in VS Code.
2. Configure it with your Pi’s IP, username `pi`, password `raspberry`.
3. Every time you save locally, it uploads to the Pi automatically.





```bash
cd src/
python3 wro.py sim                      # Run in simulator
python3 wro.py hsv_tuner                # Adjust HSV thresholds
python3 wro.py cam --dry-run            # Camera mode without motors
python3 wro.py cam --port /dev/ttyACM0 --baud 115200  # Full deployment with Arduino
