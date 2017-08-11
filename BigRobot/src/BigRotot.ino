#include <AFMotor.h>
#include "Wire.h"

#include "GY_85.h"
#include "SharpIR.h"
#include "Ultrasonic.h"
#include <Servo.h>

#define IR A8
#define IR_MODEL 1080

const long ONE_MINUTE = 60000;
const int MAX_SPEED_FORWARD = 255;
//distance servo
const int SERVO_PIN = 10;

AF_DCMotor motorLF(1); // create motor #1
AF_DCMotor motorLR(2); // create motor #2
AF_DCMotor motorRF(4); // create motor #4
AF_DCMotor motorRR(3); // create motor #3

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

  Serial.begin(9600);           // set up Serial library at 9600 bps

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
}

void loop() {
  delay(500);

  printRpms();
  stop();

  printSensorsData();
  scan();
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

void printRpms(){
  Serial.print("RPM left: ");
  int rpmLeft = getRPMLeft();
  Serial.println(rpmLeft);
  Serial.print("RPM right: ");
  int rpmRight = getRPMRight();
  Serial.println(rpmRight);
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

  Serial.print  ( " Heading:" );
  Serial.println( getHeading() );

  Serial.println(getCompensatedHeading());

  // Serial.print("Distance: ");
  // Serial.println(getIRDistance());
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
  unsigned long pepe1=millis();  // takes the time before the loop on the library begins
  int dis=SharpIR.distance();  // this returns the distance to the object you're measuring
  unsigned long pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement

  // Serial.print("Time taken (ms): ");
  // Serial.println(pepe2);
  return dis;
}

void scan(){
  int distances[90];

  for (int pos = 20; pos <= 110; pos++) {
    mDistanceServo.write(pos);
    delay(5);
    distances[pos - 20] = getIRDistance();
  }

  mDistanceServo.write(20);

  int lowestAngle = 0;
  int lowestDistance = 0;
  for(int i = 60; i <= 70; i++){
      if(lowestDistance > distances[i]){
        lowestDistance = distances[i];
        lowestAngle = i;
      }
  }
  turnToAngle(lowestAngle - 65);
  // int lowestCenter = 100;
  // for(int i = 60; i <= 70; i++){
  //     if(lowestCenter > distances[i]){
  //       lowestCenter = distances[i];
  //     }
  // }
  //
  // int lowestLeft = 100;
  // for(int i = 20; i < 60; i++){
  //     if(lowestLeft > distances[i]){
  //       lowestLeft = distances[i];
  //     }
  // }
  //
  // int lowestRight = 100;
  // for(int i = 71; i <= 110; i++){
  //     if(lowestRight > distances[i]){
  //       lowestRight = distances[i];
  //     }
  // }
  // Serial.print("Left: ");
  // Serial.println(lowestLeft);
  // Serial.print("Center: ");
  // Serial.println(lowestCenter);
  // Serial.print("Right: ");
  // Serial.println(lowestRight);
  // if(lowestLeft < 10 || lowestRight < 10 || lowestRight < 10){
  //   runBackward();
  // } else if(lowestCenter > 40){
  //   runForward(100);
  // } else if(lowestCenter >= lowestLeft && lowestCenter >= lowestRight){
  //   runForward(80);
  // } else if(lowestLeft >= lowestRight){
  //   turnLeft();
  // } else {
  //   turnRight();
  // }
}

void turnToAngle(int angle){
  float currentAngle = getCompensatedHeading();
  float targetAngle = currentAngle + angle;
  if(angle < 0){
    while(currentAngle >= targetAngle){
      turnLeft();
      delay(100);
      currentAngle = getCompensatedHeading();
      Serial.print("Current Angle: ");
      Serial.println(currentAngle);
    }
  } else if(angle > 0){
    while(currentAngle <= targetAngle){
      turnRight();
      delay(100);
      currentAngle = getCompensatedHeading();
      Serial.print("Current Angle: ");
      Serial.println(currentAngle);
    }
  }
}
