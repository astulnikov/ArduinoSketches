#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"
#include "Ultrasonic.h" 

const long ONE_MINUTE = 60000;

const int MAX_SPEED_FORWARD = HIGH; //200 for regulating
const int MAX_SPEED_BACKWARD = LOW; //50

//distance sensor
const int TRIG_PIN = 33;
const int ECHO_PIN = 32;

//Motor driver section Left
const int R_A_IA = 12; // A-IA
const int R_A_IB = 13; // A-IB

//Motor driver section Right
const int R_B_IA = 10; // B-IA
const int R_B_IB = 11; // B-IB

Ultrasonic mFrontUltrasonic(TRIG_PIN, ECHO_PIN);

HMC5883L mCompass;
ITG3200 mGyro;
ADXL345 mAccel;

int16_t gx, gy, gz;
int16_t ax, ay, az;

volatile int mTicksLeft;
volatile int mTicksRight;
long mTimeLeft;
long mTimeRight;

//Test var for different movies
int mCurrntIteration;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(9600);

  // initialize device
  Serial.println("Initializing I2C devices....");
  mAccel.initialize();
  mGyro.initialize();

  mCompass = HMC5883L(); //new instance of HMC5883L library
  setupHMC5883L(); //setup the HMC5883L

    // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mAccel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  Serial.println("Testing device connections...");
  Serial.println(mGyro.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  //Digital Pin 2 Set As An Interrupt
  Serial.println("Attaching Interrupts for speed measure...");
  attachInterrupt(0, diskInterruptLeft, RISING);
  attachInterrupt(1, diskInterruptRight, RISING);

  //left motor init
  pinMode(R_A_IA,OUTPUT);
  digitalWrite(R_A_IA, LOW);
  pinMode(R_A_IB,OUTPUT);
  digitalWrite(R_A_IB, LOW);

  //right motors init
  pinMode(R_B_IA,OUTPUT);
  digitalWrite(R_B_IA, LOW);
  pinMode(R_B_IB,OUTPUT);
  digitalWrite(R_B_IB, LOW);
}

void loop() {
  Serial.print("Compass: ");
  float heading = getHeading();
  Serial.println(heading);

  Serial.print("RPM left: ");
  int rpmLeft = getRPMLeft();
  Serial.println(rpmLeft);
  Serial.print("RPM right: ");
  int rpmRight = getRPMRight();
  Serial.println(rpmRight);

  int currentDistance = mFrontUltrasonic.Ranging(CM);
  Serial.print("Distance: ");
  Serial.println(currentDistance);

  goMove();

  delay(1000); 
}

void setupHMC5883L(){
  //Setup the HMC5883L, and check for errors
  int error; 
  error = mCompass.SetScale(1.3); //Set the scale of the compass.
  if(error != 0) Serial.println(mCompass.GetErrorText(error)); //check if there is an error, and print if so

  error = mCompass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) Serial.println(mCompass.GetErrorText(error)); //check if there is an error, and print if so
}

float getHeading(){
  //Get the reading from the HMC5883L and calculate the heading
  MagnetometerScaled scaled = mCompass.ReadScaledAxis(); //scaled values from compass.
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;

  return heading * RAD_TO_DEG; //radians to degrees
}

//int getRPM(volatile int *ticks, long *time){
//  int ticksValue = *ticks;
//  long timeValue = *time;
//  float revolutions = ticksValue / 20.0;
//  *ticks = 0;
//  int rpm = revolutions * ONE_MINUTE / (millis() - timeValue);
//  *time = millis();
//}

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

void goMove(){
  //different movies
  mCurrntIteration++;
  switch(mCurrntIteration) {
  case 1:
    runForward();
    break; 
  case 2:
    stopRunning();
    break; 
  case 3:
    runBackward();
    break;
  case 4:
    stopRunning();
    break;
  case 5:
    turnLeft();
    break;  
  case 6:
    stopRunning();
    break;  
  case 7:
    turnRight();
    break;  
  case 8:
    stopRunning();
    mCurrntIteration = 0;
    break;  
  } 
}

void runForward(){
  Serial.println("RUN FORWARD");
  analogWrite(R_A_IA, MAX_SPEED_FORWARD);
  analogWrite(R_B_IA, MAX_SPEED_FORWARD);
  digitalWrite(R_A_IB, LOW);
  digitalWrite(R_B_IB, LOW);
}

void runBackward(){
  Serial.println("RUN BACKWARD");
  analogWrite(R_A_IA, MAX_SPEED_BACKWARD);
  analogWrite(R_B_IA, MAX_SPEED_BACKWARD);
  digitalWrite(R_A_IB, HIGH);
  digitalWrite(R_B_IB, HIGH);
}

void stopRunning(){
  Serial.println("STOP");
  digitalWrite(R_A_IA, LOW);
  digitalWrite(R_B_IA, LOW);
  digitalWrite(R_A_IB, LOW);
  digitalWrite(R_B_IB, LOW);
}

void turnLeft(){
  Serial.println("TURN LEFT");
  analogWrite(R_A_IA, MAX_SPEED_FORWARD);
  analogWrite(R_B_IA, MAX_SPEED_BACKWARD);
  digitalWrite(R_A_IB, LOW);
  digitalWrite(R_B_IB, HIGH);
}

void turnRight(){
  Serial.println("TURN RIGHT");
  analogWrite(R_A_IA, MAX_SPEED_BACKWARD);
  analogWrite(R_B_IA, MAX_SPEED_FORWARD);
  digitalWrite(R_A_IB, HIGH);
  digitalWrite(R_B_IB, LOW);
}

//Capture The left IR Break-Beam Interrupt
void diskInterruptLeft(){
  mTicksLeft++;
}

//Capture The right IR Break-Beam Interrupt
void diskInterruptRight(){
  mTicksRight++;
}
















