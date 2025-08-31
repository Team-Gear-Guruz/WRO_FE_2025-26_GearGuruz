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

## ⚙️ Control Logic

Our robot’s behavior is powered by two core control strategies: **PD Control** and a **Finite State Machine (FSM).**

---

### PD Control (Proportional–Derivative Control)
PD control is used to keep the robot stable and responsive while following lanes or steering.

- **Proportional (P):** Reacts to the size of the error (e.g., how far the robot is from the center of the lane).  
- **Derivative (D):** Reacts to the rate of change of the error (e.g., how quickly the robot is drifting).  
- **Result:** Smooth steering that avoids overshooting and oscillations.

 *Formula:*  
  
- `u(t)` → Control output (steering correction)  
- `e(t)` → Error at time *t* (e.g., distance from lane center)  
- `Kp` → Proportional gain (responsiveness)  
- `Kd` → Derivative gain (stability)  

---

### FSM (Finite State Machine)
The FSM manages the robot’s high-level behavior by switching between states based on sensor inputs.

- **Lane Following** → Default driving state using camera input.  
- **Obstacle Avoidance** → Triggered when ultrasonic sensors detect a nearby object.  
- **Turn Around** → Activated when blocked completely.  
- **Parking** → Entered when the magenta parking bars are detected.  

📊 *State Transition Diagram:*

```mermaid
stateDiagram-v2
    [*] --> LaneFollowing
    LaneFollowing --> ObstacleAvoidance: Obstacle detected
    ObstacleAvoidance --> LaneFollowing: Path clear
    LaneFollowing --> Parking: Parking zone detected
    LaneFollowing --> TurnAround: Blocked path
    TurnAround --> LaneFollowing: Path clear
    Parking --> [*]

## 🔄 Signal Flow

```mermaid
flowchart TD
    Camera -->|OpenCV Vision| RaspberryPi
    RaspberryPi -->|Serial Commands| Arduino
    Arduino -->|PWM| MotorDriver --> DCMotor
    Arduino --> Servo
    Ultrasonics --> Arduino
