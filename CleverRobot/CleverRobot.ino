#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

const long ONE_MINUTE = 60000;

HMC5883L compass;
ITG3200 gyro;
ADXL345 accel;

int16_t gx, gy, gz;
int16_t ax, ay, az;

volatile int ticksLeft;
volatile int ticksRight;
long timeLeft;
long timeRight;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(9600);

  // initialize device
  Serial.println("Initializing I2C devices....");
  accel.initialize();
  gyro.initialize();

  compass = HMC5883L(); //new instance of HMC5883L library
  setupHMC5883L(); //setup the HMC5883L

    // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  Serial.println("Testing device connections...");
  Serial.println(gyro.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  //Digital Pin 2 Set As An Interrupt
  Serial.println("Attaching Interrupts for speed measure...");
  attachInterrupt(0, diskInterruptLeft, RISING);
  attachInterrupt(1, diskInterruptRight, RISING);
}

void loop() {
  Serial.print("Compass: ");
  float heading = getHeading();
  Serial.println(heading);

  Serial.print("RPM left: ");
  Serial.println(getRPM(&ticksLeft, &timeLeft), DEC);
  Serial.print("RPM right: ");
  Serial.println(getRPM(&ticksRight, &timeRight), DEC);

  delay(1000); 
}

void setupHMC5883L(){
  //Setup the HMC5883L, and check for errors
  int error; 
  error = compass.SetScale(1.3); //Set the scale of the compass.
  if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
}

float getHeading(){
  //Get the reading from the HMC5883L and calculate the heading
  MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;

  return heading * RAD_TO_DEG; //radians to degrees
}

int getRPM(volatile int *ticks, long *time){
  int ticksValue = *ticks;
  long timeValue = *time;
  float revolutions = ticksValue / 20.0;
  *ticks = 0;
  int rpm = revolutions * ONE_MINUTE / (millis() - timeValue);
  *time = millis();
}

//Capture The IR Break-Beam Interrupt
void diskInterruptLeft(){
  ticksLeft++;
}

//Capture The IR Break-Beam Interrupt
void diskInterruptRight(){
  ticksRight++;
}






