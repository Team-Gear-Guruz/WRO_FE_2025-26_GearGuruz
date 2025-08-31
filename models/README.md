
## 3D Models Used

The design of the self-driving vehicle includes several custom-designed and 3D-printed components to ensure optimal component placement, secure mounting, ease of assembly, and overall mechanical stability. These models were designed using CAD tools (e.g., Creo or Fusion 360) with attention to chassis balance, sensor positioning, and modularity.

---

### 1. **Main Chassis Baseplate**

* **Description**: Flat, rectangular base with predrilled holes and slots for mounting the motor shield, Arduino Uno, Raspberry Pi, and buck converters.
* **Purpose**:

  * Central structure to hold all electronics
  * Provides wire management channels
  * Elevates the board from the ground to avoid damage
* **Design Considerations**:

  * Lightweight PLA for easy prototyping
  * Designed with symmetry for easy reassembly
  * Reinforced edges near wheel-mount zones for rigidity

---

### 2. **Motor and Wheel Mounts**

* **Description**: Custom motor brackets to hold the 12V 300 RPM DC gear motor securely.
* **Purpose**:

  * Maintain precise motor alignment for consistent straight motion
  * Absorb minor vibrations
* **Design Considerations**:

  * Mounted with screws and vibration-dampening washers
  * Shaft opening designed to match motor diameter snugly
  * Sufficient clearance provided for wire routing

---

### 3. **Ultrasonic Sensor Brackets**

* **Description**: Compact front-facing and side-mounted holders for HC-SR04 sensors.
* **Purpose**:

  * Precisely orient sensors for accurate wall-following and obstacle detection
  * Avoid blind spots due to misalignment
* **Design Considerations**:

  * 15° angle offsets for diagonal sensors
  * Cable slot integrated to keep wiring clean
  * Removable clip-on design for quick sensor replacement

---

### 4. **Raspberry Pi and Camera Mount**

* **Description**: Vertical mount structure to hold the Pi and camera module at an appropriate viewing height and angle.
* **Purpose**:

  * Ensure stable and elevated camera field-of-view for obstacle challenge vision tasks
  * Protect the Raspberry Pi from accidental impact
* **Design Considerations**:

  * Tiltable design with pre-set notches (30°, 45°, 60°) for camera angle tuning
  * Ventilation slots for passive cooling
  * Holes for M2.5 screws to secure Pi directly

---

### Design Files & Printing

* **Material**: PLA or PETG (depending on need for flexibility vs rigidity)
* **Printing Specs**:

  * Layer height: 0.2 mm
  * Infill: 25–35%
  * Supports: Enabled for Pi mount and diagonal brackets
* **Design Tools**: Creo Parametric (Primary), Fusion 360 (Prototypes)

