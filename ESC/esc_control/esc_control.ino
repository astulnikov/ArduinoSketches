#include <Servo.h>

Servo esc;
int throttlePin = 0;
int throttle = 0;

void setup() {
  Serial.begin(9600);
  esc.attach(6);
  esc.write(90);
}

void loop() {
  
  if (Serial.available() > 0) {
    throttle = Serial.parseInt();
    Serial.print("I received: ");
    Serial.println(throttle);
    esc.writeMicroseconds(throttle);
  }
}
