// ------------------- Pin mappings -------------------
#define TRIG_F   2
#define ECHO_F   4

#define TRIG_B   5
#define ECHO_B   6

#define TRIG_L   7
#define ECHO_L   8

#define TRIG_R   A0   // D14
#define ECHO_R   A1   // D15

#define TRIG_DFL A2   // D16
#define ECHO_DFL A3   // D17

#define TRIG_DFR A4   // D18
#define ECHO_DFR A5   // D19
// -----------------------------------------------------

// Generic distance read function
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // 30ms timeout
  if (duration == 0) return 400; // no echo detected (return max ~4m)
  return duration / 58;          // µs → cm
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_F, OUTPUT);   pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT);   pinMode(ECHO_B, INPUT);
  pinMode(TRIG_L, OUTPUT);   pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);   pinMode(ECHO_R, INPUT);
  pinMode(TRIG_DFL, OUTPUT); pinMode(ECHO_DFL, INPUT);
  pinMode(TRIG_DFR, OUTPUT); pinMode(ECHO_DFR, INPUT);

  Serial.println("6x Ultrasonic Test: Front, Back, Left, Right, DiagFL, DiagFR");
}

void loop() {
  long distF   = readDistance(TRIG_F,   ECHO_F);
  long distB   = readDistance(TRIG_B,   ECHO_B);
  long distL   = readDistance(TRIG_L,   ECHO_L);
  long distR   = readDistance(TRIG_R,   ECHO_R);
  long distDFL = readDistance(TRIG_DFL, ECHO_DFL);
  long distDFR = readDistance(TRIG_DFR, ECHO_DFR);

  Serial.print("F: "); Serial.print(distF);   Serial.print(" cm  ");
  Serial.print("B: "); Serial.print(distB);   Serial.print(" cm  ");
  Serial.print("L: "); Serial.print(distL);   Serial.print(" cm  ");
  Serial.print("R: "); Serial.print(distR);   Serial.print(" cm  ");
  Serial.print("DFL: "); Serial.print(distDFL); Serial.print(" cm  ");
  Serial.print("DFR: "); Serial.print(distDFR); Serial.println(" cm");

  delay(100); // refresh rate
}
