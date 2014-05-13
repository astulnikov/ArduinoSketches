#include <Servo.h> 
#include "Ultrasonic.h" 

const char CHECK_SYMBOL = 'c';
const char APPROVE_SYMBOL= 'a';
const char STEERING_SYMBOL= 's';
const char DRIVE_SYMBOL= 'd';
const char ROBOT_SYMBOL= 'r';

const char RUN_FORWARD_SYMBOL= '1';
const char RUN_BACK_SYMBOL= '2';
const char STOP_SYMBOL= '0';

const int PAUSE_DELAY = 5;
const int READ_MESSAGE_DELAY = 5;
const int WAIT_FOR_APPROVE_DELAY = 100;
const int ATTEMPTS_TOTAL = 3; 
const int STOP_DISTANCE = 30; //In centimeters
const int FREE_DISTANCE = 200; //In centimeters

//Motor sounds...
const int MOTOR_START_DISTANCE = 20;
const int MOTOR_START_HIGH_DISTANCE = 30;
const int MOTOR_RUN_DISTANCE = 50;
const int MOTOR_RUN_HIGH_DISTANCE = 60;

const int DISTANCE_DELAY = 150;

const int ZERO_ANGLE = 90; 
const int MAX_ANGLE = 26; 
const int START_ANGLE = 0; 

const int SERVO_PIN = 9;
const int LED = 13; 

//distance sensor
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

//Motor driver section
const int R_A_IA = 5; // A-IA
const int R_A_IB = 6; // A-IB

Servo myServo;  
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

char incomingByte;
int attemptCount;
String bufferString = "";

unsigned long time;
int mDistance;
boolean mIsRobotMode = true; //Only for tests
boolean mIsRunning;

void setup() { 
  Serial.begin(9600);
  pinMode(LED, OUTPUT);

  //motors init
  pinMode(R_A_IA,OUTPUT);
  digitalWrite(R_A_IA, LOW);
  pinMode(R_A_IB,OUTPUT);
  digitalWrite(R_A_IB, LOW);

  //servo init
  myServo.attach(SERVO_PIN);
} 


void loop() { 
  if (Serial.available() > 0) {  
    incomingByte = Serial.read();
    if(incomingByte == CHECK_SYMBOL) {
      delay(READ_MESSAGE_DELAY);
      readMessage();
    }
  }
  if(mIsRobotMode && time < (millis() - DISTANCE_DELAY)) {
    time = millis();
    int currentDistance = ultrasonic.Ranging(CM);
    if(currentDistance < MOTOR_START_DISTANCE ||
      (currentDistance > MOTOR_START_HIGH_DISTANCE &&
      currentDistance < MOTOR_RUN_DISTANCE) ||
      currentDistance > MOTOR_RUN_HIGH_DISTANCE) {
      mDistance = currentDistance;
      Serial.println(mDistance);
      reactOnDistance(mDistance);
    }
  }
} 

void readMessage() {
  String message = "";
  while(Serial.available()) {
    incomingByte = Serial.read();
    message.concat(incomingByte);
  }

  chooseAction(message);

  String checkString = String(CHECK_SYMBOL);
  String approveString = String(APPROVE_SYMBOL);
  String approveMessage = String(checkString + approveString);
  //  sendMessage(approveMessage);
  //  delay(PAUSE_DELAY);
  message = CHECK_SYMBOL + message;
  sendMessage(message);
}

void chooseAction(String data) {
  if(data.charAt(0) == STEERING_SYMBOL){
    String angle = data.substring(1);
    turnToAngle(angle.toInt() + START_ANGLE);
  } 
  else if(data.charAt(0) == DRIVE_SYMBOL){
    driveControl(data.charAt(1));
  } 
  else if(data.charAt(0) == ROBOT_SYMBOL) {
    setRobotMode(!mIsRobotMode);
    //TODO begin robot program
  }
}

void turnToAngle(int angle) { 
  if(myServo.read() != angle) {
    myServo.write(angle);
  }
}

void driveControl(char command){
  if(command == RUN_FORWARD_SYMBOL){ 
    runForward();
  } 
  else if(command == RUN_BACK_SYMBOL){
    runBack();
  } 
  else{
    stopRun(); 
  }
}

void runForward(){
  analogWrite(R_A_IA, 200);
  digitalWrite(R_A_IB, LOW);
}

void runBack(){
  analogWrite(R_A_IA, 70);
  digitalWrite(R_A_IB, HIGH);
}

void stopRun(){
  digitalWrite(R_A_IA, LOW);
  digitalWrite(R_A_IB, LOW);
  mIsRunning = false;
}

void sendMessage(String message){
  Serial.println(message);
}

void safeSend(String message) {
  digitalWrite(LED, HIGH);
  attemptCount++;
  bufferString = message;
  sendMessage(bufferString);
  delay(WAIT_FOR_APPROVE_DELAY);
  digitalWrite(LED, LOW);
  if(!readApproveMessage() && attemptCount < ATTEMPTS_TOTAL){
    safeSend(bufferString);
  } 
  else {
    bufferString = "";
    attemptCount = 0; 
  }
}

boolean readApproveMessage(){
  if (Serial.available() > 1) {  
    incomingByte = Serial.read();
    if(incomingByte == CHECK_SYMBOL) {
      incomingByte = Serial.read();
      if(incomingByte == APPROVE_SYMBOL){
        bufferString = "";
        attemptCount = 0; 
        return true;
      }
    }
  }
  return false;
}

void setRobotMode(boolean robotMode) {
  mIsRobotMode = robotMode;
  if(robotMode) {
    runF();
  } 
  else {
    stopF(); 
  }
}

void reactOnDistance(int distance) {
  if(distance < STOP_DISTANCE) {
    stopF();
  } 
  else if(distance < FREE_DISTANCE) {
    float degreePerCentimeter = (float) MAX_ANGLE / (float) FREE_DISTANCE;
    int steeringAngle = ZERO_ANGLE + MAX_ANGLE - degreePerCentimeter * distance;
    turnToAngle(steeringAngle);  
    runF();
  } 
  else {
    turnToAngle(ZERO_ANGLE);  
    runF();
  }
}

void runF(){
  if(!mIsRunning) {
    mIsRunning = true;
    analogWrite(R_A_IA, 170);
    digitalWrite(R_A_IB, LOW);
  }
}

void stopF(){
  if(mIsRunning) {
    digitalWrite(R_A_IA, LOW);
    digitalWrite(R_A_IB, LOW);

    delay(10);

    analogWrite(R_A_IA, 70);
    digitalWrite(R_A_IB, HIGH);

    delay(70);
  }

  digitalWrite(R_A_IA, LOW);
  digitalWrite(R_A_IB, LOW);
  mIsRunning = false;
}













