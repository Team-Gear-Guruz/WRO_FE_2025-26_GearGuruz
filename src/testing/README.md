# Source Code

This directory contains all the code and other files used in the process of programming our WRO Future Engineers vehicle using a Raspberry Pi (vision + control) and an Arduino (motor + servo bridge).

- **wro.py** – Main Python program running on Raspberry Pi. Handles lane following, obstacle detection (red/green pillars), lap counting, and vision-guided parking between magenta bars. Supports simulation (`sim`), camera mode (`cam`), and an HSV threshold tuner (`hsv_tuner`).
- **Vision module (inside wro.py)** – Uses OpenCV to process frames, detect white lanes, colored pillars, and magenta parking bars.
- **ObstacleLogic (inside wro.py)** – Finite state machine for lane following, obstacle passing, turn-around logic, and automated parking.
- **Simulator (inside wro.py)** – Renders a virtual lane, pillars, and parking bay for testing logic without hardware. Includes keyboard controls for placing/moving obstacles.
- **HSV Tuner (inside wro.py)** – Interactive HSV slider tool to fine-tune color detection masks for white, red, green, and magenta.
- **arduino_bridge.ino** – Arduino sketch that receives serial commands from the Pi and drives:  
  - DC motor via L293/TB6612 driver (`M <int>`).  
  - Steering servo on D9 (`SUS <int>` or `S <int>`).  
  - Includes safety commands like `STOP` and a connectivity check with `PING → PONG`.

This modular structure separates **vision + decision making (Raspberry Pi)** from **low-level motor actuation (Arduino)**, making the code easier to test, debug, and extend.
