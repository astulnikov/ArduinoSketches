#include <Servo.h>

const char CHECK_SYMBOL = 'c';
const char APPROVE_SYMBOL = 'a';
const char STEERING_SYMBOL = 's';
const char DRIVE_SYMBOL = 'd';

const char RUN_FORWARD_SYMBOL = '1';
const char RUN_FAST_FORWARD_SYMBOL = '3';
const char RUN_BACK_SYMBOL = '2';
const char STOP_SYMBOL = '0';

const int PAUSE_DELAY = 5;
const int READ_MESSAGE_DELAY = 5;
const int WAIT_FOR_APPROVE_DELAY = 100;
const int ATTEMPTS_TOTAL = 3;

const int STOP_ESC_VALUE = 1500;
const int BACK_ESC_VALUE = 1200;
const int RUN_ESC_VALUE = 1600;
const int RUN_MAX_ESC_VALUE = 1750;

const int SERVO_PIN = 9;
const int SERVO_ESC_PIN = 6;

Servo esc;
Servo myServo;

char incomingByte;
int attemptCount;
String bufferString = "";

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
  }
  else {
    driveControl(data.charAt(1));
  }
}

void driveControl(char command) {
  if (command == RUN_FORWARD_SYMBOL) {
    runForward();
  } else if (command == RUN_FAST_FORWARD_SYMBOL) {
    runFastForvard();
  } else if (command == RUN_BACK_SYMBOL) {
    runBack();
  } else {
    stopRun();
  }
}

void runForward() {
  esc.writeMicroseconds(RUN_ESC_VALUE);
}

void runFastForvard() {
  esc.writeMicroseconds(RUN_MAX_ESC_VALUE);
}

void runBack() {
  esc.writeMicroseconds(BACK_ESC_VALUE);
}

void stopRun() {
  esc.writeMicroseconds(STOP_ESC_VALUE);
}

void sendMessage(String message) {
  Serial.println(message);
}
