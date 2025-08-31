# ⚡ Electromechanical Diagrams

This folder documents the **wiring, control logic, and behavior** of our WRO Future Engineers vehicle.  
The diagrams show how the **Raspberry Pi (vision + decision-making)**, **Arduino (actuator control)**, and **sensors/motors** all work together.  

At a high level:
- The **Raspberry Pi** runs computer vision and calculates corrections using **PD Control**.  
- A **Finite State Machine (FSM)** decides when to switch between lane following, obstacle avoidance, turning, and parking.  
- The **Arduino** executes commands from the Pi, controlling motors, servo steering, and ultrasonic sensors.  

---

## 🔑 Key Components

| Component                        | Role                                                                 |
|----------------------------------|----------------------------------------------------------------------|
| **Arduino Uno 🤖**               | Low-level controller for motor & servo control, ultrasonic sensors   |
| **Raspberry Pi 3/4 🍓**          | Runs OpenCV vision + high-level decision-making (FSM + PD control)   |
| **Motor Driver (L293D / TB6612) ⚡** | Controls DC motor speed & direction                               |
| **DC Gear Motor + Servo 🌀**      | Provides rear drive + steering                                       |
| **Ultrasonic Sensors (6× HC-SR04) 📡** | Distance sensing: front, back, sides, diagonals                 |
| **Li-Po Battery (3.7V, 2200mAh) 🔋** | Power source for electronics & motors                            |

---

## ⚙️ PD Control (Steering)

Our vehicle uses a **Proportional–Derivative (PD) Controller** to keep itself centered in the lane.

\[
u(t) = K_p \cdot e(t) + K_d \cdot \frac{de(t)}{dt}
\]

- **e(t)** → Lane error (distance from the center of the track)  
- **Kp (Proportional Gain)** → Corrects based on how far the car is off-center  
- **Kd (Derivative Gain)** → Corrects based on how fast the error is changing (prevents zig-zagging)  
- **u(t)** → Steering correction applied to the servo  

👉 This ensures smooth, stable lane following without overshooting.

---

## 🧩 FSM (Finite State Machine)

High-level behavior is managed by an **FSM**, which switches states depending on sensor and camera input.

**States:**
- **Lane Following** → Default, uses camera + PD control  
- **Obstacle Avoidance** → Triggered when obstacles detected  
- **Turn Around** → Used if the path is completely blocked  
- **Parking** → Activated when magenta parking zone is detected  

### State Transition Diagram
```mermaid
stateDiagram-v2
    [*] --> LaneFollowing
    LaneFollowing --> ObstacleAvoidance: Obstacle detected
    ObstacleAvoidance --> LaneFollowing: Path clear
    LaneFollowing --> Parking: Parking zone detected
    LaneFollowing --> TurnAround: Blocked path
    TurnAround --> LaneFollowing: Path clear
    Parking --> [*]

flowchart TD
    Camera["📷 Camera"] -->|Lane Detection (OpenCV)| RaspberryPi["🍓 Raspberry Pi"]
    RaspberryPi -->|Serial Commands| Arduino["🤖 Arduino Uno"]
    Arduino -->|PWM| MotorDriver["⚡ Motor Driver (L293D/TB6612)"]
    MotorDriver --> DCMotor["🌀 DC Motor"]
    Arduino --> Servo["⚙️ Steering Servo"]
    Ultrasonics["📡 Ultrasonic Sensors"] --> Arduino


---

✅ This is **the single README.md** you need — just copy this whole block into your `schemes/README.md`.  
Both diagrams will now show up, because each one is wrapped in its own ` ```mermaid ` block.  

Do you want me to also embed your **actual schematic JPEG** (so it appears under the “Signal Flow” section alongside the mermaid diagram)?

