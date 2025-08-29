Electromechanical diagrams
====

This schematic diagram was made using the help of the software Fritzing.
This directory contains the electromechanical schematic of our WRO Future Engineers vehicle.

🔧 Components Used

Arduino Uno – main microcontroller for motor + servo + ultrasonic sensors.

Raspberry Pi – vision & decision-making (camera + OpenCV).

L293D / TB6612 Motor Driver – controls DC motor speed & direction.

DC Motor – drives the vehicle forward/backward.

Servo Motor – controls steering mechanism.

Ultrasonic Sensors (HC-SR04) – 6 sensors for obstacle detection:

Front, Back, Left, Right, Front-Left, Front-Right.

Li-Po Battery (3.7V, 2200mAh) – powers Arduino and actuators.

⚙️ Connections Overview

DC Motor → Motor Driver → Arduino PWM pins.

Servo Motor → Arduino Digital Pin D9.

Ultrasonic Sensors → Arduino digital pins (Trig/Echo mapped individually).

Arduino ↔ Raspberry Pi → Serial communication (USB/TTL).

Battery → Supplies power to Arduino, motor driver, and sensors.

🧩 System Flow

Raspberry Pi processes camera feed → detects lanes, pillars, parking.

Pi sends motor/servo commands (M <int>, SUS <int>, STOP) via Serial.

Arduino executes commands → controls DC motor + servo.

Ultrasonic sensors provide distance feedback for obstacle detection & safety.
