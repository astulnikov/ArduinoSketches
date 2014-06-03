#include "Ultrasonic.h" 

//distance sensor
const int TRIG_PIN = 33;
const int ECHO_PIN = 32;

Ultrasonic mFrontUltrasonic(TRIG_PIN, ECHO_PIN);

void setup() {
  // initialize serial communication
  Serial.begin(9600);

}

void loop() {
  int currentDistance = mFrontUltrasonic.Ranging(CM);
  Serial.print("Distance: ");
  Serial.println(currentDistance, DEC);
  delay(1000);
}


