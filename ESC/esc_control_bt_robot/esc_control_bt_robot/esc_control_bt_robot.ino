#include <Arduino.h>

#include <Servo.h>
#include "Ultrasonic.h"

const char CHECK_SYMBOL = 'c';
const char END_LINE_SYMBOL = 'l';
const char APPROVE_SYMBOL = 'a';
const char STEERING_SYMBOL = 's';
const char DRIVE_SYMBOL = 'd';
const char ROBOT_SYMBOL = 'r';

const char RUN_FORWARD_SYMBOL = '1';
const char RUN_FAST_FORWARD_SYMBOL = '3';
const char RUN_FORWARD_PERCENT_SYMBOL = '4';
const char RUN_BACK_SYMBOL = '2';
const char STOP_SYMBOL = '0';

const int READ_MESSAGE_DELAY = 1000;
const int COMMAND_DELAY = 1000;
const int INFO_DELAY = 500;

const int STOP_DISTANCE = 20; //In centimeters
const int FREE_DISTANCE = 200; //In centimeters
const int DISTANCE_DELAY = 150;
const int FAST_DRIVE_DELAY = 500;
const int BRAKE_APPLY_DELAY = 100;

const int MAX_ANGLE = 26;
const int START_ANGLE = 4;
const int ZERO_ANGLE = 90 + START_ANGLE;

const int MAX_SPEED_FORWARD = 170;
const int MAX_SPEED_BACKWARD = 120;

const int DIRECTION_NO = 0;
const int DIRECTION_FORWARD = 1;
const int DIRECTION_BACKWARD = 2;

const int WAY_BLOCKED = 5;

const int STOP_ESC_VALUE = 1500;
const int BACK_ESC_VALUE = 1650;
const int RUN_ESC_VALUE = 1200;
const int RUN_MAX_ESC_VALUE = 710;

//Steering servo
const int SERVO_PIN = 9;
//Engine servo
const int SERVO_ESC_PIN = 6;

//distance sensor
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

//rear distance sensor
const int REAR_TRIG_PIN = 7;
const int REAR_ECHO_PIN = 8;

Servo esc;
Servo myServo;

Ultrasonic mFrontUltrasonic(TRIG_PIN, ECHO_PIN);
Ultrasonic mRearUltrasonic(REAR_TRIG_PIN, REAR_ECHO_PIN);

char incomingByte;

unsigned long mDriveStartTime;
unsigned long mTime;
unsigned long mLastCommandTimeStamp;
unsigned long mLastSentInfoTimeStamp;
boolean mIsRobotMode;
boolean mIsRunning;
int mDirection;
int mIsWayBlocked;
float mCurrentPower;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(READ_MESSAGE_DELAY);
  esc.attach(SERVO_ESC_PIN);
  esc.write(90);
  myServo.attach(SERVO_PIN);
  turnToAngle(ZERO_ANGLE);
}

void loop() {
  if (!mIsRobotMode){
    if (Serial.available() > 0) {
      incomingByte = Serial.read();
      if (incomingByte == CHECK_SYMBOL) {
        readMessage();
      }
    }

    if(millis() - COMMAND_DELAY > mLastCommandTimeStamp) {
      stopRun();
    }
    if(millis() - INFO_DELAY > mLastSentInfoTimeStamp) {

    sendMessage("F: " + String(mFrontUltrasonic.Ranging(CM)));
    sendMessage("R: " + String(mRearUltrasonic.Ranging(CM)));
    mLastSentInfoTimeStamp = millis();
    }

  } else {
    if (mTime < (millis() - DISTANCE_DELAY)) {
      mTime = millis();
      makeDecision();
    }
  }
}

void readMessage() {
  String message = Serial.readStringUntil(END_LINE_SYMBOL);

  chooseAction(message);

  // String approveString = String(APPROVE_SYMBOL);
  // String approveMessage = String(approveString + message);
  // sendMessage(message);
}

void chooseAction(String data) {
  char command = data.charAt(0);
  if (command == STEERING_SYMBOL) {
    String angle = data.substring(1);
    turnToAngle(START_ANGLE + angle.toInt());
    sendMessage("steer " + angle);
  } else if (command == DRIVE_SYMBOL) {
    driveControl(data.substring(1));
  } else if (command == ROBOT_SYMBOL) {
    setRobotMode(!mIsRobotMode);
  }
  mLastCommandTimeStamp = millis();
}

void driveControl(String command) {
  char driveCommand = command.charAt(0);
  if (driveCommand == RUN_FORWARD_SYMBOL) {
    runForward();
    sendMessage("drive");
  } else if (driveCommand == RUN_FAST_FORWARD_SYMBOL) {
    runFastForward();
    sendMessage("drive fast");
  } else if (driveCommand == RUN_BACK_SYMBOL) {
    if(mIsRunning){
      runBack();
      mIsRunning = false;
    }
    runBack();
    sendMessage("drive back");
  } else if (driveCommand == RUN_FORWARD_PERCENT_SYMBOL){
    String power = command.substring(1);
    runForward(power.toInt());
    sendMessage("power " + power);
  } else {
    stopRun();
    sendMessage("stop drive");
  }
}

void runForward() {
  mIsRunning = true;
  esc.writeMicroseconds(RUN_ESC_VALUE);
}

void runFastForward() {
  esc.writeMicroseconds(RUN_MAX_ESC_VALUE);
}

void runForward(int percent) {
  int power = STOP_ESC_VALUE - (STOP_ESC_VALUE - RUN_MAX_ESC_VALUE) / 100.0 * percent;
  esc.writeMicroseconds(power);
}

void runBack() {
  esc.writeMicroseconds(BACK_ESC_VALUE);
}

void stopRun() {
  esc.writeMicroseconds(STOP_ESC_VALUE);
}

void turnToAngle(int angle) {
  if (myServo.read() != angle) {
    myServo.write(angle);
  }
}

void sendMessage(String message) {
  Serial.println(String(CHECK_SYMBOL) + message + String(END_LINE_SYMBOL));
}

void sendMessage(int message) {
  sendMessage(String(message));
}

void setRobotMode(boolean robotMode) {
  mIsRobotMode = robotMode;
  if (robotMode) {
    runF();
  }
  else {
    stopF();
  }
}

void makeDecision() {
  if (mDirection != DIRECTION_BACKWARD && mIsWayBlocked < WAY_BLOCKED) {
    int currentDistance = mFrontUltrasonic.Ranging(CM);
    sendMessage(currentDistance);
    reactOnDistance(currentDistance);
  }
  else if (mDirection == DIRECTION_BACKWARD || mIsWayBlocked == WAY_BLOCKED) {
    mIsWayBlocked = 0;
    int currentDistance = mRearUltrasonic.Ranging(CM);
    sendMessage(currentDistance);
    tryGoBack(currentDistance);
  }
}

void reactOnDistance(int distance) {
  if (distance < STOP_DISTANCE && distance > 0) {
    stopF();
  }
  else if (distance < FREE_DISTANCE && distance > 0) {
    float degreePerCentimeter = (float) MAX_ANGLE / (float) FREE_DISTANCE;
    int steeringAngle = ZERO_ANGLE + MAX_ANGLE - degreePerCentimeter * distance;
    turnToAngle(steeringAngle);
    float power = (float) (0.4 / (FREE_DISTANCE - STOP_DISTANCE)) * distance + 0.6;
    runF(power);
  }
  else {
    turnToAngle(ZERO_ANGLE);
    runF();
  }
}

void tryGoBack(int distance) {
  if (distance < STOP_DISTANCE) {
    stopB();
  }
  else {
    turnToAngle(ZERO_ANGLE - MAX_ANGLE); //turn max to right
    runB();
  }
}

void runF() {
  if (mCurrentPower < 1) {
    mIsRunning = true;
    mCurrentPower = 1.0;
    mDirection = DIRECTION_FORWARD;
    runForward();
  } else if (mDriveStartTime < (millis() - FAST_DRIVE_DELAY)) {
    Serial.println("MAX POWER");
    runFastForward();
  }
}

void runF(float power) {
  if (mCurrentPower != power) {
    mIsRunning = true;
    mCurrentPower = power;
    mDirection = DIRECTION_FORWARD;
    Serial.println(power);
    esc.writeMicroseconds((int) RUN_ESC_VALUE * power);
  }
}

void stopF() {
  if (mIsRunning) {
    stopRun();

    delay(15);

    runBack();

    delay(BRAKE_APPLY_DELAY);
  }

  stopRun();
  mIsRunning = false;
  mDirection = DIRECTION_NO;
  mIsWayBlocked++;
}

void runB() {
  if (!mIsRunning) {
    mIsRunning = true;
    mDirection = DIRECTION_BACKWARD;
    runBack();
  }
}

void stopB() {
  if (mIsRunning) {
    stopRun();

    delay(15);

    runForward();

    delay(BRAKE_APPLY_DELAY);
  }

  stopRun();
  mIsRunning = false;
  mDirection = DIRECTION_NO;
}
