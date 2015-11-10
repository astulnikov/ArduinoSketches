#include <Servo.h>
#include "Ultrasonic.h"

const char CHECK_SYMBOL = 'c';
const char APPROVE_SYMBOL = 'a';
const char STEERING_SYMBOL = 's';
const char DRIVE_SYMBOL = 'd';
const char ROBOT_SYMBOL = 'r';

const char RUN_FORWARD_SYMBOL = '1';
const char RUN_FAST_FORWARD_SYMBOL = '3';
const char RUN_BACK_SYMBOL = '2';
const char STOP_SYMBOL = '0';

const int READ_MESSAGE_DELAY = 5;

const int STOP_DISTANCE = 20; //In centimeters
const int FREE_DISTANCE = 200; //In centimeters
const int DISTANCE_DELAY = 150;
const int FAST_DRIVE_DELAY = 500;
const int BRAKE_APPLY_DELAY = 100;

const int MAX_ANGLE = 26;
const int START_ANGLE = 6;
const int ZERO_ANGLE = 90 + START_ANGLE;

const int MAX_SPEED_FORWARD = 170;
const int MAX_SPEED_BACKWARD = 120;

const int DIRECTION_NO = 0;
const int DIRECTION_FORWARD = 1;
const int DIRECTION_BACKWARD = 2;

const int WAY_BLOCKED = 5;

const int STOP_ESC_VALUE = 1500;
const int BACK_ESC_VALUE = 1200;
const int RUN_ESC_VALUE = 1600;
const int RUN_MAX_ESC_VALUE = 1750;

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
int attemptCount;
String bufferString = "";

unsigned long mDriveStartTime;
unsigned long mTime;
boolean mIsRobotMode;
boolean mIsRunning;
int mDirection;
int mIsWayBlocked;
float mCurrentPower;

void setup() {
  Serial.begin(9600);
  esc.attach(SERVO_ESC_PIN);
  esc.write(90);
  myServo.attach(SERVO_PIN);
}

void loop() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == CHECK_SYMBOL) {
      delay(READ_MESSAGE_DELAY);
      readMessage();
    }
  }
  if (mIsRobotMode && mTime < (millis() - DISTANCE_DELAY)) {
    mTime = millis();
    makeDecision();
  }
}

void readMessage() {
  String message = "";
  while (Serial.available()) {
    incomingByte = Serial.read();
    message.concat(incomingByte);
  }

  chooseAction(message);

  String checkString = String(CHECK_SYMBOL);
  String approveString = String(APPROVE_SYMBOL);
  String approveMessage = String(checkString + approveString);
  sendMessage(approveMessage);
}

void chooseAction(String data) {
  if (data.charAt(0) == STEERING_SYMBOL) {
    String angle = data.substring(1);
    myServo.write(angle.toInt());
  } else if (data.charAt(0) == DRIVE_SYMBOL) {
    driveControl(data.charAt(1));
  } else if (data.charAt(0) == ROBOT_SYMBOL) {
    setRobotMode(!mIsRobotMode);
  }
}

void driveControl(char command) {
  if (command == RUN_FORWARD_SYMBOL) {
    runForward();
  } else if (command == RUN_FAST_FORWARD_SYMBOL) {
    runFastForward();
  } else if (command == RUN_BACK_SYMBOL) {
    runBack();
  } else {
    stopRun();
  }
}

void runForward() {
  esc.writeMicroseconds(RUN_ESC_VALUE);
}

void runFastForward() {
  esc.writeMicroseconds(RUN_MAX_ESC_VALUE);
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
  Serial.println(message);
}

void sendMessage(int message) {
  Serial.println(String(message));
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
  } else if(mDriveStartTime < (millis() - FAST_DRIVE_DELAY)){
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