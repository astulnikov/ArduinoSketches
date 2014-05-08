#include <Servo.h> 

const char CHECK_SYMBOL = 'c';
const char APPROVE_SYMBOL= 'a';
const char STEERING_SYMBOL= 's';
const char DRIVE_SYMBOL= 'd';

const char RUN_FORWARD_SYMBOL= '1';
const char RUN_BACK_SYMBOL= '2';
const char STOP_SYMBOL= '0';

const int PAUSE_DELAY = 5;
const int READ_MESSAGE_DELAY = 5;
const int WAIT_FOR_APPROVE_DELAY = 100;
const int ATTEMPTS_TOTAL = 3; 
const int STOP_DISTANCE = 5;

const int START_ANGLE = 5; 

const int SERVO_PIN = 9;
const int LED = 13; 

//distance sensor
const int trigPin = 10;
const int echoPin = 11;

//Motor driber section
const int R_A_IA = 5; // A-IA
const int R_A_IB = 6; // A-IB

Servo myServo;  

char incomingByte;
int attemptCount;
String bufferString = "";

boolean isRobotMode;

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
  reactOnDistance(getDistance());
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
    myServo.write(angle.toInt() + START_ANGLE);
  } 
  else{
    driveControl(data.charAt(1));
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

long getEchoTiming() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin,HIGH);
  return duration;
} 

long getDistance() {
  long distacne_cm = getEchoTiming()/29/2;
  return distacne_cm;
}

void setRobotMode(boolean robotMode) {
  isRobotMode = robotMode;
}

void reactOnDistance(int distance) {
  if(distance < STOP_DISTANCE) {
    stopRun();
  }
}










