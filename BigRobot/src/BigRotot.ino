#include <AFMotor.h>
#include "Wire.h"

#include "GY_85.h"
#include "SharpIR.h"
#include "Ultrasonic.h"
#include <Servo.h>

#define IR A8
#define IR_MODEL 1080

const char CHECK_SYMBOL = 'c';
const char END_LINE_SYMBOL = 'l';
const char STEERING_SYMBOL = 's';
const char DRIVE_SYMBOL = 'd';

const String HEADING_SYMBOL = "h";
const String TEMP_SYMBOL = "t";
const String SPEED_SYMBOL = "s";
const String DISTANCE_SYMBOL = "d";
const String REAR_DISTANCE_SYMBOL = "r";

const char RUN_FORWARD_SYMBOL = '1';
const char RUN_FORWARD_PERCENT_SYMBOL = '3';
const char RUN_BACK_SYMBOL = '2';
const char STOP_SYMBOL = '0';

const float WHEEL_RADIUS = 0.032; //meters
const long ONE_MINUTE = 60000;
const int MAX_SPEED_FORWARD = 255;
//distance servo
const int SERVO_PIN = 10;

AF_DCMotor motorLF(4); // create motor #4
AF_DCMotor motorLR(3); // create motor #3
AF_DCMotor motorRF(1); // create motor #1
AF_DCMotor motorRR(2); // create motor #2

volatile int mTicksLeft;
volatile int mTicksRight;
long mTimeLeft;
long mTimeRight;

GY_85 GY85;
SharpIR SharpIR(IR, IR_MODEL);
Servo mDistanceServo;
int mDistanceServoAngle;
int mDistanceServoAngleDiff = 5;

void setup() {
  Wire.begin();
  delay(10);

  // set up Serial library at 9600 bps
  Serial.begin(9600);
  Serial2.begin(9600);

  Serial.println("Prepare Motors!");
  motorLF.setSpeed(MAX_SPEED_FORWARD);     // set the speed up to 255
  motorLR.setSpeed(MAX_SPEED_FORWARD);     // set the speed up to 255
  motorRF.setSpeed(MAX_SPEED_FORWARD);     // set the speed up to 255
  motorRR.setSpeed(MAX_SPEED_FORWARD);     // set the speed up to 255

  Serial.println("Attaching Interrupts for speed measure...");
  attachInterrupt(4, diskInterruptLeft, RISING);
  attachInterrupt(5, diskInterruptRight, RISING);

  Serial.println("Init Accelerometr!");
  GY85.init();

  Serial.println("Attach Servo");
  mDistanceServo.attach(SERVO_PIN);
  mDistanceServo.write(65);

  delay(2000);
}

void loop() {
  if (Serial.available() > 0) {
    byte incomingByte = Serial.read();
    if (incomingByte == CHECK_SYMBOL) {
      readMessage();
    }
  }

  delay(3000);

  printRpms();
  stop();

  printSensorsData();
  // scan();

  // Serial.print("Distance: ");
  // Serial.println(getIRDistance());
}

void readMessage() {
  String message = Serial.readStringUntil(END_LINE_SYMBOL);
  chooseAction(message);
}

void chooseAction(String data) {
  char command = data.charAt(0);
  if (command == STEERING_SYMBOL) {
    String angle = data.substring(1);
    turnToAngle(angle.toInt());
    Serial.println("steer " + angle);
  } else if (command == DRIVE_SYMBOL) {
    driveControl(data.substring(1));
  }
}

void sendMessage(String message) {
  Serial.println(String(CHECK_SYMBOL) + message + String(END_LINE_SYMBOL));
}

void sendMessage(int message) {
  sendMessage(String(message));
}

void sendDistanceArray(byte bytes[], int size) {
    Serial.print(CHECK_SYMBOL);
    Serial.print(DISTANCE_SYMBOL);
    Serial.write(bytes, size);
    Serial.print(END_LINE_SYMBOL);
}

void driveControl(String command) {
  char driveCommand = command.charAt(0);
  if (driveCommand == RUN_FORWARD_SYMBOL) {
    runForward(80);
    Serial.println("drive");
  } else if (driveCommand == RUN_BACK_SYMBOL) {
    runBackward();
    Serial.println("drive back");
  } else if (driveCommand == RUN_FORWARD_PERCENT_SYMBOL){
    String power = command.substring(1);
    runForward(power.toInt());
    Serial.println("power " + power);
  } else {
    stop();
    Serial.println("stop drive");
  }
}

void runForward(int power){
  Serial.print("Forward ");
  Serial.println(power);
  motorLF.setSpeed(MAX_SPEED_FORWARD * power / 100);
  motorLR.setSpeed(MAX_SPEED_FORWARD * power / 100);
  motorRF.setSpeed(MAX_SPEED_FORWARD * power / 100);
  motorRR.setSpeed(MAX_SPEED_FORWARD * power / 100);
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
  stop();
  motorLF.run(BACKWARD);
  motorLR.run(BACKWARD);
  motorRF.run(BACKWARD);
  motorRR.run(BACKWARD);
}

void turnRight(){
  Serial.println("Right");
  stop();
  motorLF.run(FORWARD);
  motorLR.run(FORWARD);
  motorRF.run(BACKWARD);
  motorRR.run(BACKWARD);
}

void turnLeft(){
  Serial.println("Left");
  stop();
  motorLF.run(BACKWARD);
  motorLR.run(BACKWARD);
  motorRF.run(FORWARD);
  motorRR.run(FORWARD);
}

void turnToAngle(int angle){
  Serial.print("----- Turn to Angle: ");
  Serial.println(angle);
  float currentAngle = getHeading();
  float targetAngle = currentAngle + angle;
  if(angle < 0){
    while(currentAngle >= targetAngle){
      turnLeft();
      delay(50);
      currentAngle = getHeading();
      Serial.print("Current Angle: ");
      Serial.println(currentAngle);
    }
  } else if(angle > 0){
    while(currentAngle <= targetAngle){
      turnRight();
      delay(50);
      currentAngle = getHeading();
      Serial.print("Current Angle: ");
      Serial.println(currentAngle);
    }
  }
}

void printRpms(){
  Serial.print("RPM left: ");
  int rpmLeft = getRPMLeft();
  Serial.println(rpmLeft);
  Serial.print("RPM right: ");
  int rpmRight = getRPMRight();
  Serial.println(rpmRight);
  float speed = getSpeed(rpmLeft, rpmRight);
  sendMessage(SPEED_SYMBOL + speed);
}

float getSpeed(int rpmLeft, int rpmRight){
  int averageRpm = (rpmLeft + rpmRight) / 2;
  return averageRpm * WHEEL_RADIUS * 0.10472;
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

void printSensorsData(){
  int ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  int ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  int az = GY85.accelerometer_z( GY85.readFromAccelerometer() );

  int cx = GY85.compass_x( GY85.readFromCompass() );
  int cy = GY85.compass_y( GY85.readFromCompass() );
  int cz = GY85.compass_z( GY85.readFromCompass() );

  float gx = GY85.gyro_x( GY85.readGyro() );
  float gy = GY85.gyro_y( GY85.readGyro() );
  float gz = GY85.gyro_z( GY85.readGyro() );
  float gt = GY85.temp  ( GY85.readGyro() );

  Serial.print  ( "accelerometer" );
  Serial.print  ( " x:" );
  Serial.print  ( ax );
  Serial.print  ( " y:" );
  Serial.print  ( ay );
  Serial.print  ( " z:" );
  Serial.print  ( az );

  Serial.print  ( "  compass" );
  Serial.print  ( " x:" );
  Serial.print  ( cx );
  Serial.print  ( " y:" );
  Serial.print  ( cy );
  Serial.print  (" z:");
  Serial.print  ( cz );

  Serial.print  ( "  gyro" );
  Serial.print  ( " x:" );
  Serial.print  ( gx );
  Serial.print  ( " y:" );
  Serial.print  ( gy );
  Serial.print  ( " z:" );
  Serial.println  ( gz );

  Serial.print  ( " Temp:" );
  Serial.println( gt );
  sendMessage(TEMP_SYMBOL + gt);

  Serial.print  ( " Heading:" );
  Serial.println( getHeading() );
  sendMessage(HEADING_SYMBOL + getHeading());

  Serial.print  ( " Compensated Heading:" );
  Serial.println(getCompensatedHeading());
}

float getHeading(){
  float cx = GY85.compass_x( GY85.readFromCompass() );
  float cy = GY85.compass_y( GY85.readFromCompass() );
  double heading = atan2(cy, cx);
  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
  return heading * RAD_TO_DEG; //radians to degrees
}

float getCompensatedHeading(){
  float cx = GY85.compass_x(GY85.readFromCompass());
  float cy = GY85.compass_y(GY85.readFromCompass());
  float cz = GY85.compass_z(GY85.readFromCompass());

  int ax = GY85.accelerometer_x(GY85.readFromAccelerometer());
  int ay = GY85.accelerometer_y(GY85.readFromAccelerometer());

  float xh,yh,ayf,axf;
  ayf=ay/57.0;//Convert to rad
  axf=ax/57.0;//Convert to rad
  xh=cx*cos(ayf)+cy*sin(ayf)*sin(axf)-cz*cos(axf)*sin(ayf);
  yh=cy*cos(axf)+cz*sin(axf);

  double var_compass=atan2((double)yh,(double)xh) * (180 / PI) -90;
  if (var_compass>0){
    var_compass=var_compass-360;
  }
  var_compass=360+var_compass;
  return var_compass;
}

int getIRDistance(){
  // unsigned long pepe1=millis();  // takes the time before the loop on the library begins
  int dis=SharpIR.distance();  // this returns the distance to the object you're measuring
  // unsigned long pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement

  // Serial.print("Time taken (ms): ");
  // Serial.println(pepe2);
  return dis;
}

void scan(){
  byte distances[110];

  for (int pos = 10; pos <= 120; pos++) {
    mDistanceServo.write(pos);
    delay(3);
    distances[pos - 10] = getIRDistance();

    Serial.print("Distance: ");
    Serial.println(distances[pos - 10]);
  }

  sendDistanceArray(distances, 110);

  mDistanceServo.write(10);

  // int lowestDistance = 100;
  // for(int i = 35; i <= 55; i++){
  //     if(lowestDistance > distances[i]){
  //       lowestDistance = distances[i];
  //     }
  // }
  // if(lowestDistance < 20){
  //   Serial.print("Distance: ");
  //   Serial.println(lowestDistance);
  //   runBackward();
  //   return;
  // }
  //
  // if(lowestDistance > 70){
  //   runForward(100);
  //   return;
  // }
  //
  // int largestAngle = 0;
  // int largestDistance = 0;
  // for(int i = 0; i <= 90; i++){
  //     if(largestDistance < distances[i]){
  //       largestDistance = distances[i];
  //       largestAngle = i;
  //     }
  // }
  // turnToAngle(largestAngle - 45);
  // if(largestDistance > 20){
  //   runForward(70);
  // }
}
