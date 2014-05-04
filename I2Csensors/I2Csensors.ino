#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

#define LED_PIN 13
bool blinkState = false;

HMC5883L compass;
ITG3200 gyro;
ADXL345 accel;

int16_t gx, gy, gz;
int16_t ax, ay, az;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(9600);

  // initialize device
  Serial.println("Initializing I2C devices....");
  accel.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  
  gyro.initialize();
  
  compass = HMC5883L(); //new instance of HMC5883L library
  setupHMC5883L(); //setup the HMC5883L

  Serial.println("Testing device connections...");
  Serial.println(gyro.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
}

void loop() {
  // read raw accel measurements from device
  accel.getAcceleration(&ax, &ay, &az);
  gyro.getRotation(&gx, &gy, &gz);

  //Serial.print(az);
  // display tab-separated accel x/y/z values
  Serial.print("accelga ");
  Serial.print(ax); 
  Serial.print(" ");
  Serial.print(ay); 
  Serial.print(" ");
  Serial.print(az); 
  Serial.print(" ");

  Serial.print("gyroska ");
  Serial.print(gx); 
  Serial.print(" ");
  Serial.print(gy); 
  Serial.print(" ");
  Serial.print(az); 
  Serial.print(" ");
  Serial.println();

  Serial.print("Compass: ");
  float heading = getHeading();
  Serial.println(heading);

  delay(1000);

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
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



