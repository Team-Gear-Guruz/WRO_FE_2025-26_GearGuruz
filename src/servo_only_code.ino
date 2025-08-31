#include <Servo.h>

Servo myServo;  

void setup() {
  myServo.attach(9);   // Servo connected to D9
}

void loop() {
  // Sweep from 0° to 180°
  for (int pos = 0; pos <= 180; pos += 5) {
    myServo.write(pos);
    delay(50);
  }

  // Sweep back from 180° to 0°
  for (int pos = 180; pos >= 0; pos -= 5) {
    myServo.write(pos);
    delay(50);
  }
}
