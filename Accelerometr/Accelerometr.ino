#define ADC_ref 5.0
#define analog_resolution 1024.0

unsigned int value_x;
unsigned int value_y;
unsigned int value_z;

void setup()   {
  Serial.begin(9600);
}
 
void loop() {
  value_x = analogRead(0);
  value_y = analogRead(1);
  value_z = analogRead(2);
  
  Serial.print(value_x/analog_resolution*ADC_ref, 5);
  Serial.print(" ");
  Serial.print(value_y/analog_resolution*ADC_ref, 5);
  Serial.print(" ");
  Serial.println(value_z/analog_resolution*ADC_ref, 5);
  
  delay(1000);
}
