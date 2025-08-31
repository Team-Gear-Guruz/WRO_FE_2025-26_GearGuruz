# ⚡ Electromechanical Diagrams

This folder contains the **schematic diagrams** and electrical wiring of our WRO Future Engineers vehicle, created using **Fritzing**.

---

## 🔑 Key Components

| Component | Role |
|-----------|------|
| **Arduino Uno** | Low-level controller for motor & servo control, ultrasonic sensors |
| **Raspberry Pi 3B+** | Vision (OpenCV) + decision-making (FSM, PD control) |
| **Motor Driver (L293D / TB6612)** | Controls DC motor speed & direction |
| **DC Gear Motor + Servo** | Rear drive + steering system |
| **Ultrasonic Sensors (6x HC-SR04)** | Distance sensing: front, back, sides, diagonals |
| **Li-Po Battery (3.7V, 2200mAh)** | Power source for electronics & motors |

---

## 🔄 Signal Flow

```mermaid
flowchart TD
    Camera -->|OpenCV Vision| RaspberryPi
    RaspberryPi -->|Serial Commands| Arduino
    Arduino -->|PWM| MotorDriver --> DCMotor
    Arduino --> Servo
    Ultrasonics --> Arduino
