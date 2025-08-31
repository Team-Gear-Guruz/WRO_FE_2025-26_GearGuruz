Engineering materials
====

## Team Banner

<p align="center">
  <img src="https://raw.githubusercontent.com/ayan-atm/WRO_FE_2025-26/main/other/TeamBanner.png" alt="Team Banner" width="900"/>
</p>

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2022.

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
| Servo Motor (SG90/MG90S) | 1        | Steering actuator (mounted on front axle)              |
| HC-SR04 Ultrasonic Sensor| 6        | Obstacle detection (front, back, left, right, FL, FR)  |
| Li-on Battery 12V| 1     | Power source for electronics and motors                |
| Motor Mounts / Chassis   | 1 set    | 3D-printed chassis parts + mounts                      |
| Wheels + Tires           | 2–4      | Drive + steering wheels                                |
| Wires, Jumper cables     | —        | Connections between Pi, Arduino, sensors, and driver   |
| Breadboard / PCB         | 1        | For stable wiring and connections                      |
| Camera (Pi Camera / USB) | 1        | Used by Raspberry Pi for lane + obstacle detection     |
| Motor 12v DC | 1        | For Moving the robot (connected to single axle)     |

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

Kushal Khemani, age 16, kushal.khemani@gmail.com

## Team Photos

<img src="https://github.com/ayan-atm/WRO_FE_2025-26/raw/main/t-photos/Official%20Picture.jpeg" width="40%" height="40%"> <img src="https://github.com/ayan-atm/WRO_FE_2025-26/raw/main/t-photos/Funny%20Picture.png" width="40%" height="40%">

Ayan Atmakuri (Purple shirt), Kushal Khemani (Grey and green shirt)

##  Quick Overview

A vision-guided mini-vehicle using:

- **Raspberry Pi**: Computer vision (OpenCV), control logic (PD + FSM)  
- **Arduino Uno**: Motor and servo actuation, obstacle feedback  
- **Simple Serial Protocol**: Commands like `M <int>`, `SUS <us>`, `STOP`, `PING`  
- **Features**: Lane following, obstacle avoidance, auto-parking, HSV tuning, simulator.

## Introduction

This repository hosts the software and wiring for an autonomous robot car built for the **World Robot Olympiad 2025 – Future Engineers Challenge**.

The design leverages a **Raspberry Pi** for high-level perception and decision-making—handling lane detection, obstacle avoidance, color-coded behavior (e.g., pass red on the right, green on the left), and vision-guided parking—while an **Arduino Uno** handles real-time actuation of the DC drive motor and steering servo based on those decisions.

### System Workflow

1. **Vision & Perception (Raspberry Pi)**  
   - Captures camera frames and processes them for road lanes, colored markers, and parking bays.  
   - Runs a state machine with PD control logic, lap counting, and parking alignment commands.

2. **Control Communication**  
   - Sends steering (e.g., servo angle) and throttle (motor PWM) commands via serial to the Arduino.

3. **Motion Execution (Arduino)**  
   - Receives commands and outputs PWM for the motor driver (L293D/TB6612) and servo control for steering.

4. **Real World Performance**  
   - Car autonomously navigates laps, avoids obstacles, obeys color-based passing rules, executes turnarounds, and completes vision-guided parallel parking.


##  Running the System

```bash
cd src/
python3 wro.py sim                      # Run in simulator
python3 wro.py hsv_tuner                # Adjust HSV thresholds
python3 wro.py cam --dry-run            # Camera mode without motors
python3 wro.py cam --port /dev/ttyACM0 --baud 115200  # Full deployment with Arduino
