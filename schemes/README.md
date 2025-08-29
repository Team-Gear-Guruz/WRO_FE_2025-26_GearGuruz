Electromechanical diagrams
====

This schematic diagram was made using the help of the software Fritzing.

This folder contains the electrical schematic of our WRO Future Engineers vehicle.

🔧 Key Components
Arduino Uno – Motor & servo control, ultrasonic sensors.
Raspberry Pi – Vision + decision-making (camera + OpenCV).
Motor Driver (L293/TB6612) – DC motor speed & direction.
DC Motor & Servo – Drive + steering system.
Ultrasonic Sensors (6x HC-SR04) – Distance sensing (front, back, sides, diagonals).
Li-Po Battery (3.7V, 2200mAh) – Power source.

⚙️ Signal Flow
Camera → Raspberry Pi (Vision + FSM) → Serial → Arduino → Motors & Servo
  ↳ Ultrasonic sensors → Arduino (safety & obstacle detection)
