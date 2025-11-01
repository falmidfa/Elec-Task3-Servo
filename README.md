# Elec-Task3-Servo

## Task1-Code

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

void setup() {
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(10);
  servo5.attach(11);

  unsigned long startTime = millis();

  // Sweep motion for 2 seconds
  while (millis() - startTime < 2000) {
    for (int i = 0; i <= 180; i += 10) {
      servo1.write(i);
      servo2.write(i);
      servo3.write(i);
      servo4.write(i);
      servo5.write(i);
      delay(50);

      // Stop sweep if 2 seconds passed
      if (millis() - startTime >= 2000) break;
    }
  }

  // Hold all servos at 90 degrees
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
}

void loop() {
  // Do nothing — servos stay at 90°
}

---

## Task2-Code

#define DIR_PIN 12 
#define STEP_PIN 14 
#define STEPS_PER_REV 200  // typical for NEMA 17

// ramp parameters (microseconds per half-cycle)
const int delayStart = 1500;  
const int delayEnd   = 200;   
const int delayStep  = 50;     
const int pulsesPerDelay = 200; 

// helper to compute RPM from delay_us
double computeRPM(long delay_us) {
  // period for one full step (HIGH+LOW) = 2*delay_us microseconds
  double freqHz = 1e6 / (2.0 * delay_us); // steps per second
  double rpm = (freqHz * 60.0) / STEPS_PER_REV;
  return rpm;
}

void pulseOnce(int stepPin, unsigned long delay_us) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delay_us);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delay_us);
}

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  Serial.begin(115200);
  delay(100);
}

void loop() {
  // 1) One revolution clockwise
  digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    pulseOnce(STEP_PIN, 1000); // fixed test delay
  }
  Serial.println("One rev CW done");
  delay(1000);

  // 2) One revolution counterclockwise
  digitalWrite(DIR_PIN, LOW);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    pulseOnce(STEP_PIN, 1000);
  }
  Serial.println("One rev CCW done");
  delay(1000);

  // 3) Accelerate (ramp up)
  Serial.println("Accelerating...");
  digitalWrite(DIR_PIN, HIGH);
  for (int d = delayStart; d >= delayEnd; d -= delayStep) {
    double rpm = computeRPM(d);
    Serial.print("delay_us=");
    Serial.print(d);
    Serial.print("  RPM=");
    Serial.println(rpm, 2);

   
    for (int p = 0; p < pulsesPerDelay; p++) {
      pulseOnce(STEP_PIN, d);
    }
  }

  // hold at top speed for 2 seconds
  Serial.println("Hold top speed 2s");
  unsigned long holdStart = millis();
  while (millis() - holdStart < 2000) {
    pulseOnce(STEP_PIN, delayEnd);
  }

  // decelerate (ramp down)
  Serial.println("Decelerating...");
  for (int d = delayEnd; d <= delayStart; d += delayStep) {
    double rpm = computeRPM(d);
    Serial.print("delay_us=");
    Serial.print(d);
    Serial.print("  RPM=");
    Serial.println(rpm, 2);

    for (int p = 0; p < pulsesPerDelay; p++) {
      pulseOnce(STEP_PIN, d);
    }
  }

  Serial.println("Ramp cycle complete");
  while (true) { delay(1000); } // stop here
}
