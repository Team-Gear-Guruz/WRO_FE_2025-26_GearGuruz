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

## Team members

Ayan Atmakuri, age 16, atmakuriayan@gmail.com 
Kushal Khemani, age 16, kushal.khemani@gmail.com

## Introduction

Our vehicle is driven by a Raspberry Pi (vision and AI brain) and an Arduino (motor + servo controller).

The Raspberry Pi runs Python code for perception (lane detection, obstacle recognition, parking detection) and decision-making (PD control, lap tracking, state machines).

The Arduino receives high-level commands (motor PWM, steering servo microseconds) over serial and translates them into signals for the motor driver (L293/TB6612) and the steering servo.

The workflow:

Raspberry Pi captures camera feed → processes lane + obstacle data.

Control logic generates steering and throttle signals.

Arduino executes commands → drives motors and steering servo.

This integration allowed the vehicle to complete laps, handle obstacles, turn around when needed, and finish with vision-guided parking.
