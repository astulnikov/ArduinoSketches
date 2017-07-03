#include <Arduino.h>

#include <Servo.h>
#include "Ultrasonic.h"

const char CHECK_SYMBOL = 'c';
const char END_LINE_SYMBOL = 'l';

const int MAX_ANGLE = 26;
const int START_ANGLE = 4;
const int ZERO_ANGLE = 90 + START_ANGLE;

const int STOP_DISTANCE = 90; //In centimeters
const int REVERSE_DISTANCE = 20; //In centimeters
const int STOP_BACK_DISTANCE = 30; //In centimeters
const int FREE_DISTANCE = 130; //In centimeters
const int DISTANCE_THRESHOLD = 250; //In centimeters

const int DECISION_DELAY = 0;
const int SPEED_MEASURE_DELAY = 0;
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
const int RUN_MIN_PERCENT = 25;
const int MINIMAL_SPEED = 5;
const int AFTER_STOP_PAUSE = 1000;

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
boolean mIsWayBlocked;
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
  if(mIsWayBlocked && mDecisionTimeStamp > millis() - AFTER_STOP_PAUSE){
    return;
  }

  mIsWayBlocked = false;

  if (mDecisionTimeStamp < (millis() - DECISION_DELAY)) {
    int frontDistance = measureDistance(mFrontUltrasonic);
    long frontTimestamp = millis();

    int rearDistance = measureDistance(mRearUltrasonic);


      mSpeed = calculateSpeed(mPreviousFrontDistance, frontDistance,
    frontTimestamp - mPrevFDistanceTimeStamp);
      mPreviousFrontDistance = frontDistance;
      mPrevFDistanceTimeStamp = frontTimestamp;
      mSpeedMeasureTimeStamp = millis();


    if(mLastSentInfoTimeStamp < (millis() - INFO_DELAY)) {
      sendMessage("F: " + String(frontDistance));
      sendMessage("R: " + String(rearDistance));
      sendMessage("Speed " + String(mSpeed));
      mLastSentInfoTimeStamp = millis();
    }

    mDecisionTimeStamp = millis();
    chooseDirection(frontDistance, rearDistance);
  }
}

int measureDistance(Ultrasonic ultrasonic){
  int firstMeasure = ultrasonic.Ranging(CM);
  delay(5);
  int secondMeasure = ultrasonic.Ranging(CM);
  if(firstMeasure == 0){
    firstMeasure = 300;
  }
  if(secondMeasure == 0){
    secondMeasure = 300;
  }

  if(firstMeasure - secondMeasure > DISTANCE_THRESHOLD ||
secondMeasure - firstMeasure > DISTANCE_THRESHOLD){
    sendMessage("Recursion");
    return measureDistance(ultrasonic);
  }
  return (firstMeasure + secondMeasure) / 2;
}

int calculateSpeed(int s1, int s2, long timeDiff){
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
  if(frontDistance > FREE_DISTANCE){
    turnToAngle(ZERO_ANGLE);
    runForward(40);
  } else if(frontDistance < STOP_DISTANCE){

    checkBreaks();

    if(mSpeed == 0 &&
      frontDistance <= REVERSE_DISTANCE &&
      rearDistance > REVERSE_DISTANCE){
      turnToAngle(70);
      runBack(80);
    } else if(mSpeed < MINIMAL_SPEED &&
    frontDistance > REVERSE_DISTANCE){
      turnToAngle(ZERO_ANGLE);
      runForward(25);
    } else{
      stopRun();
    }
  } else {
    reactOnFrontDistance(frontDistance);
  }
}

void checkBreaks(){
  if(mSpeed > MINIMAL_SPEED){
    mIsWayBlocked = true;
    stopRun();
    if(mSpeed > 15){
      sendMessage("---------------- STOP");
      apllyBrakesBack();
    }
    if(mSpeed > 50){
      sendMessage("---------------- apllyBrakesBack again");
      apllyBrakesBack();
    }
    if(mSpeed > 180){
      sendMessage("---------------- apllyBrakesBack For Sure");
      apllyBrakesBack();
    }
    mSpeed = -1;
  }
}

void reactOnFrontDistance(int distance){
  turnToAngle(ZERO_ANGLE);
  if(distance - STOP_DISTANCE > 0){
    int power = (distance - STOP_DISTANCE) / 2.5f;
    if(power < RUN_MIN_PERCENT){
      power = RUN_MIN_PERCENT;
    }
    runForward(power);
  }
}

void runForward(int percent) {
  percent = 25;
  sendMessage("Run Forward Power" + String(percent));
  mCurrentPower = percent;
  int power = STOP_ESC_VALUE - (STOP_ESC_VALUE - RUN_MAX_ESC_VALUE) / 100.0 * percent;
  if(!mIsRunning || mDirection == DIRECTION_BACKWARD){
    esc.writeMicroseconds(power);
    delay(60);
    esc.writeMicroseconds(STOP_ESC_VALUE);
    delay(60);
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
