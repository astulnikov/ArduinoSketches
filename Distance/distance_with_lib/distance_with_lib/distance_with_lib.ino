#include <Arduino.h>

#include "Ultrasonic.h"

//distance sensor
const int TRIG_PIN = 28;
const int ECHO_PIN = 29;

Ultrasonic mFrontUltrasonic(TRIG_PIN, ECHO_PIN);

void setup() {
  // initialize serial communication
  Serial.begin(9600);
  Serial.setTimeout(3000);
}

void loop() {
  int currentDistance = mFrontUltrasonic.Ranging(CM);
  Serial.print("Distance: ");
  Serial.println(currentDistance, DEC);
  // Serial.println("message: " + Serial.readStringUntil('l'));
  delay(1000);
}
