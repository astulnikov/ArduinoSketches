#include <AFMotor.h>
#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"
#include "Ultrasonic.h"

const long ONE_MINUTE = 60000;

AF_DCMotor motorLF(1); // create motor #1
AF_DCMotor motorLR(2); // create motor #2
AF_DCMotor motorRF(4); // create motor #4
AF_DCMotor motorRR(3); // create motor #3

volatile int mTicksLeft;
volatile int mTicksRight;
long mTimeLeft;
long mTimeRight;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  Serial.println("Prepare Motors!");
  motorLF.setSpeed(100);     // set the speed to 200/255
  motorLR.setSpeed(100);     // set the speed to 200/255
  motorRF.setSpeed(100);     // set the speed to 200/255
  motorRR.setSpeed(100);     // set the speed to 200/255

  Serial.println("Attaching Interrupts for speed measure...");
  attachInterrupt(4, diskInterruptLeft, RISING);
  attachInterrupt(5, diskInterruptRight, RISING);
}

void loop() {
  runForward();
  delay(1000);
  Serial.print("RPM left: ");
  int rpmLeft = getRPMLeft();
  Serial.println(rpmLeft);
  Serial.print("RPM right: ");
  int rpmRight = getRPMRight();
  Serial.println(rpmRight);
  stop();
  delay(1000);
  Serial.print("RPM left: ");
  rpmLeft = getRPMLeft();
  Serial.println(rpmLeft);
  Serial.print("RPM right: ");
  rpmRight = getRPMRight();
  Serial.println(rpmRight);
  runBackward();
  delay(1000);
  Serial.print("RPM left: ");
  rpmLeft = getRPMLeft();
  Serial.println(rpmLeft);
  Serial.print("RPM right: ");
  rpmRight = getRPMRight();
  Serial.println(rpmRight);
  stop();
  delay(1000);
}

void runForward(){
  Serial.println("Forward");
  motorLF.run(FORWARD);
  motorLR.run(FORWARD);
  motorRF.run(FORWARD);
  motorRR.run(FORWARD);
}

void stop(){
  Serial.println("Stop");
  motorLF.run(RELEASE);
  motorLR.run(RELEASE);
  motorRF.run(RELEASE);
  motorRR.run(RELEASE);
}

void runBackward(){
  Serial.println("Backward");
  motorLF.run(BACKWARD);
  motorLR.run(BACKWARD);
  motorRF.run(BACKWARD);
  motorRR.run(BACKWARD);
}

int getRPMLeft(){
  float revolutions = mTicksLeft / 20.0;
  mTicksLeft = 0;
  int rpm = revolutions * ONE_MINUTE / (millis() - mTimeLeft);
  mTimeLeft = millis();
  return rpm;
}

int getRPMRight(){
  float revolutions = mTicksRight / 20.0;
  mTicksRight = 0;
  int rpm = revolutions * ONE_MINUTE / (millis() - mTimeRight);
  mTimeRight = millis();
  return rpm;
}

//Capture The left IR Break-Beam Interrupt
void diskInterruptLeft(){
  mTicksLeft++;
}

//Capture The right IR Break-Beam Interrupt
void diskInterruptRight(){
  mTicksRight++;
}
