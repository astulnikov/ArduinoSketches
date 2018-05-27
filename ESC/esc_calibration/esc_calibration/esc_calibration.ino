#include <Arduino.h>

#include <Servo.h>

#define MAX_SIGNAL 2000
#define STOP_SIGNAL 1500
#define MIN_SIGNAL 700
#define MOTOR_PIN 6

Servo motor;

void setup() {
  Serial.begin(9600);
  Serial.println("Program begin...");
  Serial.println("This program will calibrate the ESC.");
  Serial.println("Press any key to continue");

  motor.attach(MOTOR_PIN);

  // Wait for input
  while (!Serial.available());
  Serial.read();

  Serial.println("Calibrating Maximum Signal");
  Serial.println("Turn on power source, then wait 2 seconds and press any key");
  motor.writeMicroseconds(MAX_SIGNAL);

  // Wait for input
  while (!Serial.available());
  Serial.read();

  // Send min output
  Serial.println("Calibrating Minimum Signal");
  Serial.println("Press any key to finish");
  motor.writeMicroseconds(MIN_SIGNAL);

  // Wait for input
  while (!Serial.available());
  Serial.read();
  motor.writeMicroseconds(STOP_SIGNAL);
}

void loop() {

}
