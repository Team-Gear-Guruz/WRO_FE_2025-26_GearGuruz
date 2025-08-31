# ⚡ Electromechanical Diagrams

This folder contains the **electrical and wiring schematics** of our WRO Future Engineers vehicle, designed with **Fritzing**.  
It documents how the **Raspberry Pi (vision + logic)** communicates with the **Arduino (actuation + sensors)** to create a fully autonomous robot.

---

## 🔑 Key Components

| Component | Role |
|-----------|------|
| **Arduino Uno** | Low-level controller for motor & servo control, ultrasonic sensors |
| **Raspberry Pi 3/4** | Vision (OpenCV) + decision-making (FSM, PD control) |
| **Motor Driver (L293D / TB6612)** | Controls DC motor speed & direction |
| **DC Gear Motor + Servo** | Rear drive + steering system |
| **Ultrasonic Sensors (6× HC-SR04)** | Distance sensing: front, back, sides, diagonals |
| **Li-Po Battery (3.7V, 2200mAh)** | Power source for electronics & motors |

---

## 🔁 Signal Flow

```mermaid
flowchart TD
    Camera -->|OpenCV Vision| RaspberryPi
    RaspberryPi -->|Serial Commands| Arduino
    Arduino -->|PWM| MotorDriver --> DCMotor
    Arduino --> Servo
    Ultrasonics --> Arduino

<img src="Schematic%20Diagram.jpeg" width="600" alt="Electromechanical Schematic">
