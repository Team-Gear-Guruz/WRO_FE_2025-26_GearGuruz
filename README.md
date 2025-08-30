Engineering materials
====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2022.

## Content

* `t-photos` Team photos (includes one official group photo and one funny team photo).
* `v-photos` Vehicle photos (6 angles: front, back, sides, top, bottom).
* `video` contains a link to our driving demonstration video
* `schemes` Schematic diagrams of the electromechanical system (motors, sensors, controllers, wiring).
* `src` Source code for the vehicle:
* 
        Python (Raspberry Pi) → Vision, lane following, obstacle avoidance, parking logic.
        Arduino → Motor + servo bridge (serial protocol for PWM and steering).
* `models` 3D printing, laser cutting, and CNC files for chassis and mechanical components.
* `other` Documentation, datasets, hardware specs, and supporting materials for preparing the vehicle.

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
| Li-Po Battery (3.7V 2000mAh)| 1     | Power source for electronics and motors                |
| Motor Mounts / Chassis   | 1 set    | 3D-printed chassis parts + mounts                      |
| Wheels + Tires           | 2–4      | Drive + steering wheels                                |
| Wires, Jumper cables     | —        | Connections between Pi, Arduino, sensors, and driver   |
| Breadboard / PCB         | 1        | For stable wiring and connections                      |
| Camera (Pi Camera / USB) | 1        | Used by Raspberry Pi for lane + obstacle detection     |

## Team members

Ayan Atmakuri, age 16, atmakuriayan@gmail.com 

Kushal Khemani, age 16, kushal.khemani@gmail.com

## Team Photos

https://github.com/ayan-atm/WRO_FE_2025-26/blob/main/t-photos/Official%20Picture.jpeg

https://github.com/ayan-atm/WRO_FE_2025-26/blob/main/t-photos/Funny%20Picture.png

##  Quick Overview

A vision-guided mini-vehicle using:

- **Raspberry Pi**: Computer vision (OpenCV), control logic (PD + FSM)  
- **Arduino Uno**: Motor and servo actuation, obstacle feedback  
- **Simple Serial Protocol**: Commands like `M <int>`, `SUS <us>`, `STOP`, `PING`  
- **Features**: Lane following, obstacle avoidance, auto-parking, HSV tuning, simulator.

## Introduction

Our vehicle is driven by a Raspberry Pi (vision and AI brain) and an Arduino (motor + servo controller).

The Raspberry Pi runs Python code for perception (lane detection, obstacle recognition, parking detection) and decision-making (PD control, lap tracking, state machines).

The Arduino receives high-level commands (motor PWM, steering servo microseconds) over serial and translates them into signals for the motor driver (L293/TB6612) and the steering servo.

The workflow:

Raspberry Pi captures camera feed → processes lane + obstacle data.

Control logic generates steering and throttle signals.

Arduino executes commands → drives motors and steering servo.

This integration allowed the vehicle to complete laps, handle obstacles, turn around when needed, and finish with vision-guided parking.

##  Running the System

```bash
cd src/
python3 wro.py sim                      # Run in simulator
python3 wro.py hsv_tuner                # Adjust HSV thresholds
python3 wro.py cam --dry-run            # Camera mode without motors
python3 wro.py cam --port /dev/ttyACM0 --baud 115200  # Full deployment with Arduino
