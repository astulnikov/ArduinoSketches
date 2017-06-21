#include <Arduino.h>

#include <Servo.h>
#include "Ultrasonic.h"

const char CHECK_SYMBOL = 'c';
const char END_LINE_SYMBOL = 'l';

const int MAX_ANGLE = 26;
const int START_ANGLE = 4;
const int ZERO_ANGLE = 90 + START_ANGLE;

const int STOP_DISTANCE = 60; //In centimeters
const int STOP_BACK_DISTANCE = 30; //In centimeters
const int FREE_DISTANCE = 130; //In centimeters
const int DECISION_DELAY = 100;
const int SPEED_MEASURE_DELAY = 200;
const int FAST_DRIVE_DELAY = 300;
const int BRAKE_APPLY_INTERVAL = 30;
const int BRAKE_APPLY_DELAY = 80;
const int DRIVE_BACK_DELAY = 1500;

const int READ_MESSAGE_DELAY = 1000;
const int INFO_DELAY = 500;

const int DIRECTION_NO = 0;
const int DIRECTION_FORWARD = 1;
const int DIRECTION_BACKWARD = 2;

const int WAY_BLOCKED = 3;

const int STOP_ESC_VALUE = 1500;
const int BACK_ESC_VALUE = 1650;
const int RUN_ESC_VALUE = 1200;
const int RUN_MAX_ESC_VALUE = 710;

//Steering servo
const int SERVO_STEERING_PIN = 9;
//Engine servo
const int SERVO_ESC_PIN = 6;

//distance sensor
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

//rear distance sensor
const int REAR_TRIG_PIN = 7;
const int REAR_ECHO_PIN = 8;

Servo esc;
Servo steering;

Ultrasonic mFrontUltrasonic(TRIG_PIN, ECHO_PIN);
Ultrasonic mRearUltrasonic(REAR_TRIG_PIN, REAR_ECHO_PIN);

unsigned long mLastSentInfoTimeStamp;
unsigned long mDriveStartTime;
unsigned long mDriveBackStartTime;
unsigned long mDecisionTimeStamp;

unsigned long mSpeedMeasureTimeStamp;
int mSpeed;

boolean mIsRunning;
int mIsWayBlocked;
int mDirection;
float mCurrentPower;

int mPreviousFrontDistance;
long mPrevFDistanceTimeStamp;

void setup() {
  // initialize serial communication
  Serial.begin(9600);
  Serial.setTimeout(1000);

  esc.attach(SERVO_ESC_PIN);
  esc.write(90);

  steering.attach(SERVO_STEERING_PIN);
  turnToAngle(ZERO_ANGLE);

  delay(3000);
}

void loop() {
  // Serial.println("message: " + Serial.readStringUntil('l'));

    if (mDecisionTimeStamp < (millis() - DECISION_DELAY)) {

    int frontDistance = (measureDistance(mFrontUltrasonic) +
      measureDistance(mFrontUltrasonic)) / 2;
    long frontTimestamp = millis();
    int rearDistance = measureDistance(mRearUltrasonic);

    if(mSpeedMeasureTimeStamp < (millis() - SPEED_MEASURE_DELAY) ||
  mSpeed < 0){
      mSpeed = calculateSpeed(mPreviousFrontDistance, frontDistance,
      frontTimestamp - mPrevFDistanceTimeStamp);
      sendMessage("speed " + String(mSpeed));
      mPreviousFrontDistance = frontDistance;
      mPrevFDistanceTimeStamp = frontTimestamp;
      mSpeedMeasureTimeStamp = millis();
    }

    if(millis() - INFO_DELAY > mLastSentInfoTimeStamp) {
      sendMessage("F: " + String(frontDistance));
      sendMessage("R: " + String(rearDistance));
      mLastSentInfoTimeStamp = millis();
    }

    mDecisionTimeStamp = millis();
    chooseDirection(frontDistance, rearDistance);
  }
}

int measureDistance(Ultrasonic ultrasonic){
  int firstMeasure = ultrasonic.Ranging(CM);
  int secondMeasure = ultrasonic.Ranging(CM);
  if(firstMeasure == 0){
    return secondMeasure;
  } else if(secondMeasure == 0){
    return firstMeasure;
  } else {
    return (firstMeasure + secondMeasure) / 2;
  }
}

int calculateSpeed(int s1, int s2, long timeDiff){
  if(s1 == 0){
    s1 = 300;
  }
  int speed = (int) (s1 - s2) / (timeDiff / 1000.0f);
  if(speed < 0){
    speed = 0;
  }
  sendMessage("s1: " + String(s1) + " s2: " + String(s2) + " timeDiff: " + String(timeDiff));
  return speed;
}

void sendMessage(String message) {
  Serial.println(String(CHECK_SYMBOL) + message + String(END_LINE_SYMBOL));
}

void sendMessage(int message) {
  sendMessage(String(message));
}

void turnToAngle(int angle) {
  sendMessage("turnToAngle " + String(angle));
  if (steering.read() != angle) {
    steering.write(angle);
  }
}

void chooseDirection(int frontDistance, int rearDistance) {
  if(frontDistance > FREE_DISTANCE || frontDistance == 0){
    turnToAngle(ZERO_ANGLE);
    runForward(40);
  } else if(frontDistance < STOP_DISTANCE){
    stopRun();
    checkBreaks();
  } else {
    reactOnFrontDistance(frontDistance);
  }
}

void checkBreaks(){
  if(mSpeed > 40){
    sendMessage("STOP");
    apllyBrakesBack();
    if(mSpeed > 100){
      sendMessage("---------------- apllyBrakesBack again");
      apllyBrakesBack();
    }
    if(mSpeed > 300){
      sendMessage("---------------- apllyBrakesBack For Sure");
      apllyBrakesBack();
    }
    mSpeed = -1;
  }
}

void reactOnFrontDistance(int distance){
  if(distance - STOP_DISTANCE > 0){
    int power = (distance - STOP_DISTANCE) / 2.5f;
    if(power < 25){
      power = 25;
    }
    runForward(power);
  }
}

void runForward(int percent) {
  sendMessage("Run Forward Power" + String(percent));
  mCurrentPower = percent;
  int power = STOP_ESC_VALUE - (STOP_ESC_VALUE - RUN_MAX_ESC_VALUE) / 100.0 * percent;
  if(!mIsRunning){
    esc.writeMicroseconds(power);
    delay(50);
    esc.writeMicroseconds(STOP_ESC_VALUE);
    delay(50);
  }
  mIsRunning = true;
  mDirection = DIRECTION_FORWARD;
  esc.writeMicroseconds(power);
}

void runBack(int percent) {
  sendMessage("Run Back Power" + String(percent));
  mIsRunning = true;
  mDirection = DIRECTION_BACKWARD;
  moveBack(percent);
}

void moveBack(int percent){
  int power = STOP_ESC_VALUE + (BACK_ESC_VALUE - STOP_ESC_VALUE) / 100.0 * percent;
  esc.writeMicroseconds(power);
}

void stopRun() {
  sendMessage("Stop Run");
  mIsRunning = false;
  esc.writeMicroseconds(STOP_ESC_VALUE);
}

void apllyBrakesBack(){
  sendMessage("Apply Brakes");
  delay(BRAKE_APPLY_INTERVAL);
  moveBack(100);
  delay(BRAKE_APPLY_DELAY);
  stopRun();
}
