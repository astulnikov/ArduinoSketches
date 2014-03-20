#include <Servo.h> 

const char CHECK_SYMBOL = 'c';
const char APPROVE_SYMBOL=  'a';
const int PAUSE_DELAY = 5;
const int READ_MESSAGE_DELAY = 5;
const int WAIT_FOR_APPROVE_DELAY = 100;
const int ATTEMPTS_TOTAL = 3; 
const int LED = 13; 

Servo myServo;  
char incomingByte;
int attemptCount;
String bufferString = "";

void setup() { 
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  myServo.attach(9); 
} 


void loop() { 
  if (Serial.available() > 0) {  
    incomingByte = Serial.read();
    if(incomingByte == CHECK_SYMBOL) {
      delay(READ_MESSAGE_DELAY);
      readMessage();
    }
  }
} 

void readMessage() {
  String message = "";
  while(Serial.available()) {
    incomingByte = Serial.read();
    message.concat(incomingByte);
  }
  myServo.write(message.toInt());
  String checkString = String(CHECK_SYMBOL);
  String approveString = String(APPROVE_SYMBOL);
  String approveMessage = String(checkString + approveString);
  sendMessage(approveMessage);
//  delay(PAUSE_DELAY);
//  message = CHECK_SYMBOL + message;
//  safeSend(message);
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










