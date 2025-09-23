Engineering material
====


<p align="center">
  <img src="other/teambanner.png" alt="Team Banner" width="900"/>
</p>

This repository contains engineering material of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2025.

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
| Raspberry Pi 3B+ (or 4)  | 1        | Main controller, runs Python + OpenCV vision + logic   |
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
      <img src="other/PCB_New.jpeg" width="220" alt="PCB"><br/>
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

Kushal Khemani, age 16, kushal.khemani@gmail.com

## Team Photos


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


# Unit Testing For Arduino

These are standalone Arduino sketches to test individual subsystems (servo, motor, ultrasonic sensors) before integrating them together.  
Each can be copied into the Arduino IDE and uploaded separately.



### 1) Servo Sweep (D9)

Moves the steering **servo** smoothly from 0°→180°→0°.  
**Important:** MG996R needs its own 5–6 V supply (≥3 A). Do not power from the Arduino 5 V pin.
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
// ---- Ultrasonic Sensors ----
struct Ultrasonic {
  uint8_t trigPin;
  uint8_t echoPin;
  const char* name;
};

// Define all sensors with mapping
Ultrasonic sensors[] = {
  {2,  A0, "Front-Center"},
  {3,  A1, "Front-Left Diagonal"},
  {5,  A2, "Front-Right Diagonal"},
  {6,  A3, "Left"},
  {10, A4, "Right"},
  {13, A5, "Back"}
};

long getDistance(uint8_t trigPin, uint8_t echoPin) {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo time
  long duration = pulseIn(echoPin, HIGH, 20000); // timeout = 20ms (~3.4m range)

  // Convert to distance in cm
  long distance = duration * 0.034 / 2;
  return distance;
}

void setup() {
  Serial.begin(9600);

  // Initialize pins
  for (auto &s : sensors) {
    pinMode(s.trigPin, OUTPUT);
    pinMode(s.echoPin, INPUT);
  }

  Serial.println("Ultrasonic Sensor Test Started...");
}

void loop() {
  for (auto &s : sensors) {
    long d = getDistance(s.trigPin, s.echoPin);
    Serial.print(s.name);
    Serial.print(": ");
    if (d == 0) Serial.println("Out of range");
    else Serial.print(d), Serial.println(" cm");
  }

  Serial.println("--------------------");
  delay(500); // wait before next reading
}

```  
🥳 Gotchas

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

#### Lap Detection Logic

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

#### Wall-Following Steering

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

#### Stop After 3 Laps

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

## Customization Tips

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
  WRO 2025 – Open Challenge
  Full Obstacle Avoidance (Potential-Field Style) + PD Steering & Proportional Speed
  Hardware: Arduino UNO + Adafruit Motor Shield v1 (L293D) + 1x DC Motor (M1) + Servo (D9) 

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


```
## **Now we shall be using a Raspberry Pi too.**

# Getting Started

I had absolutely no idea how to use a Raspberry Pi when we first started working with it. At the beginning, even basic tasks like flashing the OS onto an SD card or connecting it to a monitor felt overwhelming. To figuring things out, relying on a few YouTube tutorials that broke the process down step by step. Slowly, by following along with these resources, I gained confidence and were able to start experimenting with the Pi ourselves.

**Helpful videos we used:**

1. [Guide on setting up a Raspberry Pi 4](https://www.youtube.com/watch?v=ntaXWS8Lk34)
2. [Installing Raspberry Pi OS](https://www.youtube.com/watch?v=ntLJmHOJ0ME)
3. [Beginner-friendly tutorial on GPIO pins](https://www.youtube.com/watch?v=H1lxZweM52U)

Since I had just moved houses and all my electronics were still at my old place, I had to figure out how to use the Raspberry Pi without a keyboard or monitor. I ended up setting it up for headless operation, which means accessing it remotely from my laptop over Wi-Fi. By enabling SSH and using a tool like PuTTY or the built-in terminal, I could control the Pi entirely from my computer. This allowed me to run commands, install software, and experiment with projects without needing the physical peripherals. It was a lifesaver while I waited to get my setup back from the old house.

This is what I did - 

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

# Unit Testing For Raspberry Pi
### 1) Camera Testing File
```py
from picamera2 import Picamera2
import cv2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")

picam2.start()

while True:
    frame = picam2.capture_array()
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

```
### 2) Using Raspberry Pi to output to Arduino

```cpp
#include <AFMotor.h>   // For L293D motor shield
#include <Servo.h>

AF_DCMotor motor(1);   // Motor connected to M1 on L293D shield
Servo steering;        // Steering servo

// Servo center values (adjust for your robot’s alignment)
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 60;
const int SERVO_RIGHT  = 120;

void setup() {
  Serial.begin(115200);
  motor.setSpeed(0);
  motor.run(RELEASE);

  steering.attach(9);       // Servo signal pin
  steering.write(SERVO_CENTER);  // Center on startup
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');  // Read line
    cmd.trim();

    // --- Motor control ---
    if (cmd.startsWith("M")) {
      int spd = cmd.substring(2).toInt();  // Example: "M 180"
      spd = constrain(spd, 0, 255);
      motor.setSpeed(spd);
      motor.run(FORWARD);
    }

    else if (cmd == "STOP") {
      motor.setSpeed(0);
      motor.run(RELEASE);
    }

    // --- Steering control ---
    else if (cmd.startsWith("S")) {
      int angle = cmd.substring(2).toInt();  // Example: "S 120"
      angle = constrain(angle, 60, 120);     // protect servo range
      steering.write(angle);
    }

    else if (cmd == "LEFT") {
      steering.write(SERVO_LEFT);
    }

    else if (cmd == "RIGHT") {
      steering.write(SERVO_RIGHT);
    }
  }
}

```
# WRO 2025 – Vision → Arduino High-Level Control (Obstacle Challenge)

> Raspberry Pi = vision + strategy hints.
> Arduino = motors/servo + safety + **all** reversing + **full ultrasonic parking**.

This repo contains a single Python script for Raspberry Pi that detects the mat line (orange/blue), red/green pillars, and the magenta start section. It streams **high-level commands** to an Arduino over USB serial. Your Arduino sketch executes motion, avoidance, and the complete parking maneuver using six ultrasonic sensors.

---

## Features

* **Line tracking** (orange/blue) → normalized lane offset `[-1…+1]`
* **Pillar detection** (red/green) → keep-side hints (`KEEP LEFT/RIGHT`)
* **Lap counting** by magenta start section → 3 laps then `MODE PARK`
* **Robust colors**: auto-detects camera channel order (RGB vs BGR) & locks
* **WRO start flow**: waits for a **Start button** (GPIO 17) or Enter
* **Serial handshake**: waits for Arduino `RDY`, then sends `GO`, `MODE RACE`
* **Rate-limited serial**: dedupes lines to avoid flooding the Arduino
* **Heartbeat** in Park mode so Arduino watchdog stays happy
* **Graceful shutdown**: on quit/Ctrl-C sends `STOP` and cleans up

---

## System Architecture

```
PiCamera2 + OpenCV (Pi)
     │
     ├─ Detect line / pillars / magenta
     │
     ├─ Compute lane offset & lap state
     │
USB  │  ASCII protocol (newline-terminated)
     ▼
Arduino (AFMotor + Servo + 6x HC-SR04)
     ├─ State machine: RACE → PARK_* → HOLD
     ├─ Reversing & safety bubbles
     └─ Full parallel parking (ultrasonic only)
```


## 🔌 Serial Protocol (Pi → Arduino)

| Command      | Args                      | Purpose                                            |
| ------------ | ------------------------- | -------------------------------------------------- |
| `GO`         | –                         | Start session after Arduino prints `RDY`           |
| `MODE RACE`  | –                         | Normal lap running                                 |
| `MODE PARK`  | –                         | Enter parking state machine (Arduino-only control) |
| `DRIVE o s`  | `o∈[-1..1]`, `s∈[0..255]` | Lane offset (left− / right+), speed hint           |
| `KEEP LEFT`  | –                         | Temporary bias away from a **green** pillar        |
| `KEEP RIGHT` | –                         | Temporary bias away from a **red** pillar          |
| `HB`         | –                         | Heartbeat (keeps Arduino watchdog alive)           |
| `STOP`       | –                         | Emergency stop on exit                             |

> The Arduino should **clamp** speeds by local clearance, and **own** all reversing/safety.


## ⚙️ Config You’ll Touch First

In the Python file:

```python
HEADLESS = False        # True if running over SSH without a GUI
LINE_MODE = "orange"    # or "blue" depending on your mat
DEBUG_OVERLAY = True    # on-screen HUD (disable for max performance)
START_PIN = 17          # GPIO start button (falls back to Enter if missing)
```

**HSV thresholds** live in `color_ranges` and are already tuned wide for venue lighting.

---

## Quick Start

1. **Pi packages**

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-serial
```

2. **Wire Start button** (optional, WRO-friendly)
   BCM **GPIO 17 (pin 11)** → button → **GND**.

3. **Upload Arduino sketch** (the one you built from our previous step).
   It must print `RDY` on boot and implement the protocol above.

4. **Run the Pi script**

```bash
python3 pi_vision_obstacle_highlevel.py
```

* Window opens (unless `HEADLESS=True`)
* Waits for Start (button or Enter)
* Waits for Arduino `RDY`, then sends `GO`, `MODE RACE`

---

## How the Vision Works (Pi)

### 1) Channel-order auto-detect

Some PiCamera2 pipelines deliver memory as RGB, others as BGR. Feeding the wrong order to HSV makes orange look blue, etc.
**Solution**: for \~18 frames, the script tries **both** mappings—“as-is” vs “RGB→BGR swap”—and **scores** which one makes your selected **line color** (orange or blue) dominate. It then **locks** the winner for the rest of the run.

```text
frame (RGB888 ndarray)
 ├─ treat as BGR → score mapping
 └─ convert RGB→BGR → score mapping
Lock better score → use consistently (no flicker)
```

> If the preview looks mirrored/upside-down, uncomment a `cv2.flip(...)` line where indicated.

### 2) Color masks

* Light blur → HSV
* CLAHE on V channel → lighting robustness
* Wide HSV bands for: `red`, `green`, `magenta`, `orange`, `blue`, `black`
* Morph open/close for clean blobs

### 3) Line offset

* Find largest orange/blue blob → centroid `cx`
* Normalize offset: `offset = (cx - center) / (w/2)` → `[-1..+1]`
* Speed hint: slower if pillar is “near” (area threshold)

### 4) Lap counting

* When **magenta** area exceeds a threshold → you’re in the **start section**
* Count **rising edges** (entering the section)
* After seeing it **4th time** (start pass + 3 laps) → send `MODE PARK`


## Troubleshooting

**“Colors swapped (orange ↔ blue / red ↔ cyan)”**
The auto-detector handles this. If it still misclassifies in your lighting:

* Confirm `LINE_MODE` matches your mat.
* Ensure magenta and pillar colors are visible during the first \~1 second (helps the score).
* As a last resort, set `SWAP_RB = True` (legacy manual override).

**“Left/right seems reversed”**
Uncomment a horizontal flip:

```python
# frame_bgr = cv2.flip(frame_bgr, 1)  # mirror
```

**“Serial port not found”**

* Check with `ls /dev/ttyACM* /dev/ttyUSB*`
* Add user to dialout: `sudo usermod -a -G dialout $USER && sudo reboot`

**“Laggy preview”**

* Set `DEBUG_OVERLAY = False`
* Or run headless: `HEADLESS = True` (no window)

**“Arduino doesn’t start”**

* Confirm it prints `RDY` at 115200
* Close Arduino Serial Monitor (only one program can own the port)

---

### Field Tuning Cheatsheet

* **Line window**: If the bot weaves, reduce the acceptable band a bit by tightening how you map offset → speed on the **Arduino** side (PID gain & saturation).
* **Pillar near thresholds**:

  * `min_area` \~ `0.002 * (w*h)`
  * `near_area` \~ `0.004 * (w*h)`
    Increase if false positives; decrease if it reacts too late.
* **Magenta start threshold**: `start_mag_area = int(0.0035*(w*h))`
  Raise if double-counts; lower if it misses the section.

> All reversing/parking tuning is on the **Arduino** (ultrasonic thresholds and timings).

### Code Map

* **`camera_open()`** – PiCamera2 in `RGB888`, AWB warmup then lock.
* **Auto-detect block** – decides once between as-is vs `RGB→BGR`.
* **`detect_colors()`** – masks + areas (HSV, CLAHE, morph).
* **`track_line()`** – centroid + grayscale fallback.
* **`serial_open()/send_line()`** – port scan, dedupe/rate-limit writes.
* **Start button** – waits on GPIO 17 (fallback Enter).
* **Main loop** – capture → color mapping → detect → protocol out → HUD.

### Safety Notes (WRO)

* Start in a **waiting state** (one Start button).
* **No RF/BLE/Wi-Fi** during runs; USB serial is fine.
* The Arduino must ensure **no touching** magenta boundaries in parking and must keep signs within their circles (your ultrasonic parking logic should err on the safe side).
* After parking, **stop** and ignore further motion hints.


### Extending

* Add a **“mode override”** key to force `MODE PARK` manually during testing.
* Log per-frame decisions (offset, areas) to CSV for offline tune.
* Add a “cooldown” window around magenta to prevent double lap counts.


### Need a minimal Arduino simulator?

You can bench the Pi without a robot using a tiny Python script that prints `RDY`, then echos back whatever the Pi sends. Ask and I’ll drop it in as `tools/serial_echo.py`.


# 🏁 Raspberry Pi Code -  Obstacle Challenge 
```python
#!/usr/bin/env python3
"""
WRO 2025 – Vision → Arduino High-Level Control (Webcam + Parking Steering)

- Webcam via OpenCV V4L2 (/dev/video*)
- Locks exposure & white balance (v4l2-ctl) for stable HSV colors
- RACE: line-follow (orange/blue), red/green pillar hints
- START zone (magenta) lap counting with guard if starting inside zone
- PARK ALIGN: compute steering from magenta (fallback black) to align parallel; does NOT drive inside bay
- Robust: auto-fallback to NO_SERIAL when Arduino not connected
- Flags:
    --test-vision   : run camera + color detection only (skip Arduino/serial)
    --no-arduino    : force NO_SERIAL even if a port exists
    --video-index N : choose /dev/videoN (default 0)
"""
import os, sys, time, glob, subprocess, argparse, math
import cv2
import numpy as np
import serial

# ---------------------- Headless / Qt ------------------------------------------
HEADLESS = False
if HEADLESS:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
if not os.environ.get("DISPLAY"):
    HEADLESS = True
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

# ---------------------- Serial --------------------------------------------------
BAUD = 115200
ser = None
_last_sent = ""
_last_tx = 0.0
NO_SERIAL = False  # set by flags or auto-detection

def serial_open(auto_ports=None, retries=6, wait_between=1.0):
    """Try to open Arduino serial; on failure, enable NO_SERIAL and continue."""
    global ser, NO_SERIAL
    if NO_SERIAL:
        print("[Pi] NO_SERIAL=True → skip serial.")
        return None
    if auto_ports is None:
        auto_ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
        auto_ports = auto_ports or ["/dev/ttyACM0", "/dev/ttyUSB0"]
    for attempt in range(1, retries + 1):
        for p in auto_ports:
            try:
                print(f"[Pi] Trying {p} @ {BAUD}…")
                s = serial.Serial(p, BAUD, timeout=0.02)
                time.sleep(2.0)  # Arduino auto-reset
                ser = s
                print(f"[Pi] Connected: {p}")
                return p
            except Exception as e:
                print(f"[Pi] {p} failed: {e}")
        print(f"[Pi] Retry {attempt}/{retries} in {wait_between}s")
        time.sleep(wait_between)
    print("[Pi] WARN: No serial ports found. Using NO_SERIAL=True.")
    NO_SERIAL = True
    return None

def wait_for_arduino_ready(max_wait=17.0):
    if NO_SERIAL:
        print("[Pi] NO_SERIAL: skip RDY wait.")
        return False
    print("[Pi] Waiting for Arduino RDY…")
    t0 = time.time()
    while time.time() - t0 < max_wait:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            line = ""
        if line == "RDY":
            print("[Pi] Arduino RDY")
            return True
        time.sleep(0.01)
    print("[Pi] No RDY seen; continuing.")
    return False

def send_line(s: str, min_interval=0.08, dedupe=True):
    global _last_sent, _last_tx
    now = time.time()
    if dedupe and s == _last_sent and (now - _last_tx) < min_interval:
        return
    if NO_SERIAL or ser is None:
        print(f"[TX:TEST] {s}")
        _last_sent, _last_tx = s, now
        return
    try:
        ser.write((s + "\n").encode("ascii", errors="ignore"))
    except Exception as e:
        print(f"[Pi] Serial write failed: {e}")
    _last_sent, _last_tx = s, now

# ---------------------- Start Button -------------------------------------------
HAS_GPIO = False
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

START_PIN = 17

def wait_for_start_button():
    if NO_SERIAL or not HAS_GPIO:
        print("[Pi] Press Enter to START…")
        try: input()
        except EOFError: pass
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(START_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("[Pi] Waiting for Start button…")
    while True:
        if GPIO.input(START_PIN) == 0:
            print("[Pi] START pressed!")
            break
        time.sleep(0.01)

# ---------------------- Webcam (V4L2) ------------------------------------------
def wait_for_camera_device(timeout=15.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if glob.glob("/dev/video*"): return True
        time.sleep(0.1)
    return False

def free_camera_nodes():
    nodes = glob.glob("/dev/video*")
    if not nodes: return
    try:
        subprocess.run(["/usr/bin/fuser", "-k"] + nodes,
                       check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

def configure_logitech_uvc(dev="/dev/video0", exposure_abs=120, wb_temp=4000, gain=0):
    cmds = [
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_auto=1"],
        ["v4l2-ctl", "-d", dev, f"--set-ctrl=exposure_absolute={exposure_abs}"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=white_balance_temperature_auto=0"],
        ["v4l2-ctl", "-d", dev, f"--set-ctrl=white_balance_temperature={wb_temp}"],
    ]
    if gain is not None:
        cmds.append(["v4l2-ctl", "-d", dev, f"--set-ctrl=gain={gain}"])
    for c in cmds:
        try:
            subprocess.run(c, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

def camera_open_webcam(index=0, width=640, height=480, target_fps=30):
    if not wait_for_camera_device(): raise RuntimeError("[Pi] No /dev/video* found.")
    free_camera_nodes(); time.sleep(0.2)
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened(): raise RuntimeError(f"[Pi] Failed to open /dev/video{index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS,         target_fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,  2)
    t0 = time.time()
    while time.time() - t0 < 1.5:
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            print(f"[Pi] Webcam ready at {frame.shape[1]}x{frame.shape[0]}")
            return cap
        time.sleep(0.05)
    raise RuntimeError("[Pi] Webcam warmup failed")

# ---------------------- Vision (HSV masks) -------------------------------------
LINE_MODE = "orange"   # or "blue"
DEBUG_OVERLAY = False

color_ranges = {
    "red1":     ((  0,  80,  70), ( 12, 255, 255)),
    "red2":     ((168,  80,  70), (179, 255, 255)),
    "green":    (( 40,  70,  70), ( 85, 255, 255)),
    "magenta":  ((135,  80,  80), (170, 255, 255)),
    "orange":   ((  5, 120, 110), ( 22, 255, 255)),
    "blue":     (( 95, 110,  80), (135, 255, 255)),
    "black":    ((  0,   0,   0), (179,  80,  80)),
}

def preprocess_hsv(frame_bgr):
    b = cv2.GaussianBlur(frame_bgr, (5,5), 0)
    return cv2.cvtColor(b, cv2.COLOR_BGR2HSV)

def clean_mask(mask):
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=1)
    return mask

def detect_colors(frame_bgr):
    hsv = preprocess_hsv(frame_bgr)
    r1 = cv2.inRange(hsv, color_ranges["red1"][0], color_ranges["red1"][1])
    r2 = cv2.inRange(hsv, color_ranges["red2"][0], color_ranges["red2"][1])
    masks = {"red": clean_mask(cv2.bitwise_or(r1, r2))}
    for c in ["green", "magenta", "orange", "blue", "black"]:
        masks[c] = clean_mask(cv2.inRange(hsv, color_ranges[c][0], color_ranges[c][1]))
    areas = {k: int(cv2.countNonZero(v)) for k, v in masks.items()}
    return masks, areas

def centroid_from_mask(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] <= 0: return None
    return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

def track_line(frame_bgr, masks, mode):
    chosen = "orange" if mode != "blue" else "blue"
    mask = masks[chosen]
    area = cv2.countNonZero(mask)
    h, w = frame_bgr.shape[:2]
    cx = w // 2
    if area > 800:
        cen = centroid_from_mask(mask)
        if cen: cx = cen[0]
    return cx

# ---------------------- Parking Steering ---------------------------------------
def _median_angle_deg(lines):
    if lines is None or len(lines) == 0: return 0.0, 0
    angs = []
    for x1,y1,x2,y2 in lines.reshape(-1,4):
        a = math.degrees(math.atan2(y2 - y1, x2 - x1))
        while a <= -90: a += 180
        while a >   90: a -= 180
        angs.append(a)
    if not angs: return 0.0, 0
    return float(np.median(angs)), len(angs)

def parking_steer(frame_bgr, masks):
    """Return (steer [-1..1], angle_err_deg, lateral_norm) for parallel align."""
    mag = masks.get("magenta", None)
    blk = masks.get("black", None)
    use_mag = mag is not None and cv2.countNonZero(mag) > 500
    mask = mag if use_mag else blk
    if mask is None or cv2.countNonZero(mask) < 200:
        return 0.0, 0.0, 0.0
    h, w = mask.shape[:2]
    edges = cv2.Canny(mask, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=40, minLineLength=40, maxLineGap=12)
    ang_err, _ = _median_angle_deg(lines)
    cen = centroid_from_mask(mask)
    lateral = 0.0
    if cen: lateral = (cen[0] - (w//2)) / (w/2.0)
    k_lat, k_ang = 0.8, 0.02
    steer = float(np.clip(k_lat*lateral + k_ang*ang_err, -1.0, 1.0))
    return steer, float(ang_err), float(lateral)

# ---------------------- Main ----------------------------------------------------
def main():
    global NO_SERIAL
    ap = argparse.ArgumentParser()
    ap.add_argument("--test-vision", action="store_true")
    ap.add_argument("--no-arduino", action="store_true")
    ap.add_argument("--video-index", type=int, default=0)
    args = ap.parse_args()
    if args.test_vision or args.no_arduino: NO_SERIAL = True

    dev = f"/dev/video{args.video_index}"
    configure_logitech_uvc(dev, exposure_abs=120, wb_temp=4000, gain=0)
    cap = camera_open_webcam(index=args.video_index, width=640, height=480, target_fps=30)

    serial_open()

    if not HEADLESS:
        cv2.namedWindow("WRO FE – Vision (Webcam)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("WRO FE – Vision (Webcam)", 900, 600)

    wait_for_start_button()

    if not NO_SERIAL:
        wait_for_arduino_ready(17.0)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            send_line("GO", dedupe=False)
            send_line("MODE RACE", dedupe=False)
            time.sleep(0.25)

    laps_seen = 0
    magenta_seen_last = False
    STATE = "RACE"
    left_start_zone = False
    last_hb = 0.0
    HB_PERIOD = 0.2

    PARK_ALIGN_DURATION = 4.0   # seconds of gentle parallel align driving
    PARK_SPEED = 90             # low forward speed to avoid entering bay
    park_t0 = None

    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok or frame_bgr is None or frame_bgr.size == 0:
                time.sleep(0.01); continue

            h, w = frame_bgr.shape[:2]
            center_x = w // 2

            masks, areas = detect_colors(frame_bgr)
            min_area = int(0.002 * (w*h))
            near_area = int(0.004 * (w*h))
            start_mag_area = int(0.0035 * (w*h))

            # RACE: pillar hints
            if STATE == "RACE" and not NO_SERIAL:
                if areas["red"]   > min_area:  send_line("KEEP RIGHT")
                if areas["green"] > min_area:  send_line("KEEP LEFT")

            # Lap counting with guard
            in_start = areas["magenta"] > start_mag_area
            if not in_start: left_start_zone = True
            if STATE == "RACE" and left_start_zone and in_start and not magenta_seen_last:
                laps_seen += 1
                if laps_seen >= 3 and not NO_SERIAL:
                    STATE = "PARK"
                    park_t0 = time.time()
                    send_line("MODE PARK")
            magenta_seen_last = in_start

            # Line tracking during RACE
            cx = track_line(frame_bgr, masks, LINE_MODE)
            offset_px = cx - center_x
            offset = max(-1.0, min(1.0, offset_px / (w/2.0)))
            near_pillar = (areas["red"] > near_area) or (areas["green"] > near_area)
            speed = 120 if near_pillar else 180
            if STATE == "RACE" and not NO_SERIAL:
                send_line(f"DRIVE {offset:.3f} {int(speed)}")

            # PARK: parallel alignment only (no driving inside)
            if STATE == "PARK":
                steer, ang_err, lat = parking_steer(frame_bgr, masks)
                if not NO_SERIAL:
                    now = time.time()
                    if park_t0 is None: park_t0 = now
                    if (now - park_t0) < PARK_ALIGN_DURATION:
                        send_line(f"DRIVE {steer:.3f} {int(PARK_SPEED)}")
                    else:
                        send_line("DRIVE 0.000 0")
                else:
                    # test-vision: show alignment numbers
                    print(f"[PARK] steer={steer:+.3f} ang={ang_err:+.1f}° lat={lat:+.3f}")

            # Heartbeat
            now = time.time()
            if (now - last_hb) > HB_PERIOD and not NO_SERIAL:
                send_line("HB"); last_hb = now

            # Optional preview
            if not HEADLESS:
                if DEBUG_OVERLAY:
                    cv2.putText(frame_bgr, f"STATE: {STATE}", (10, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.imshow("WRO FE – Vision (Webcam)", frame_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("[Pi] Ctrl-C")
    finally:
        try: cap.release()
        except Exception: pass
        if ser:
            try: ser.close()
            except Exception: pass
        if HAS_GPIO:
            try: GPIO.cleanup()
            except Exception: pass
        if not HEADLESS:
            try: cv2.destroyAllWindows()
            except Exception: pass
        print("[Pi] Clean exit.")

if _name_ == "_main_":
    main()
```
# 🏁 Arduino Code -  Obstacle Challenge

Here’s the TL;DR of what that Arduino sketch does:

### Roles & I/O

* **Drives the robot**: controls the DC motor (Adafruit Motor Shield v1, M1) and the steering **servo on D9**.
* **Reads 6 ultrasonics** (HC-SR04):
  FC(2/A0), FLD(3/A1), FRD(5/A2), L(6/A3), R(10/A4), B(13/A5).
* **Talks to the Raspberry Pi over serial (115200)** and follows its high-level hints.

### Serial protocol (what it accepts)

* `GO` – acknowledge start (stays idle until a MODE arrives).
* `MODE RACE` – race/obstacle driving.
* `MODE PARK` – take over and execute the full parking sequence.
* `DRIVE <offset> <speed>` – lane offset `[-1..+1]` and speed hint `0..255`.
* `KEEP LEFT` / `KEEP RIGHT` – temporary bias (≈450 ms) to the left/right.
* `HB` – heartbeat (used only to keep watchdog happy).
* `STOP` – immediate emergency stop.

On boot it **prints `RDY`** so the Pi knows it’s ready.

### Safety & watchdog

* **RX watchdog** (≈400 ms): if the Pi goes silent, it stops the motor and centers the steering.
* **Immediate obstacle stops**:

  * If **front-center** < 15 cm → stop, reverse briefly, and arc away.
  * If **hard block** (<14 cm) ahead or both diagonals are very close → same avoidance.
* All reversing/avoid is done **locally** (Arduino), independent of the Pi.

### RACE mode (line following + obstacle avoidance)

* Uses the Pi’s `DRIVE offset` as the **base steering** (offset × max-steer).
* Adds a **potential-field** term from ultrasonics:

  * FC/diagonals/side/back distances repel the robot away from obstacles.
  * PD control on the lateral field for smoother steering (KP/KD tunables).
* Applies a brief **KEEP bias** (±8°) when `KEEP LEFT/RIGHT` is received.
* Speed:

  * Starts from Pi’s `speed` hint.
  * **Clamped by forward clearance** (maps clearance to min/base speed).
  * Extra slow-down if the field strongly “pushes back”.

### PARK mode (fully on Arduino, ultrasonic only)

State machine:

1. **PARK\_ALIGN** – make the car **parallel to the side wall**
   (balance L vs R distances using slow nudges and small steering).
2. **PARK\_ENTER** – **reverse with right lock** to slip into the lot
   (until back or right side gets “near”).
3. **PARK\_STRAIGHTEN** – counter-steer while reversing a bit to straighten.
4. **PARK\_CENTER** – small forward/back nudges to center between **front** and **back** walls.
5. **PARK\_HOLD** – stop, steering centered; ignore further DRIVE hints.

Each phase has **timeouts** and conservative thresholds to avoid touching boundaries.

### Steering & drive details

* **Servo**: microsecond control around 1500 µs; ±60° limit; \~11 µs/° (tunable).
* **Motor**: forward/back via Motor Shield M1; polarity invert option if wiring flips direction.
* **Loop cadence** ≈ 20 ms for smooth servo updates and responsive control.
* **Filtering**: exponential smoothing (α=0.2) on all six distance readings.

### Tuning knobs (quick ones)

* Steering gains: `KP_STEER`, `KD_STEER`, `MAX_STEER_DEG`.
* Speed policy: `BASE_SPEED`, `MIN_SPEED`, `CLEAR_*_CM`.
* Safety thresholds: `FRONT_STOP_CM`, `HARD_BLOCK_CM`.
* Parking thresholds: `ALIGN_TOL_CM`, `ENTER_*_CM`, `CENTER_*_CM`.
* Hints: `HINT_BIAS_MS` (duration) and bias magnitude (±8° in code).

### Minimal console behavior

* On boot: prints **`RDY`**.
* (Optional debug prints are in the code, commented out; you can enable them to see distances, mode, and commands.)


FULL CODE ( WITH TWEEKS) - 
```cpp
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


```
```bash
cd src/
python3 wro.py sim                      # Run in simulator
python3 wro.py hsv_tuner                # Adjust HSV thresholds
python3 wro.py cam --dry-run            # Camera mode without motors
python3 wro.py cam --port /dev/ttyACM0 --baud 115200  # Full deployment with Arduino
