volatile int ticks;
int rpmilli;
float revolutions;
long timeold;

void setup(){
  //Digital Pin 2 Set As An Interrupt
  attachInterrupt(0, diskInterrupt, RISING);
  Serial.begin(9600);
}

void loop(){
  delay(500);
  revolutions = ticks / 20.0;
  ticks = 0;
  rpmilli = revolutions * 60000 / (millis() - timeold);
  timeold = millis();

  Serial.println(revolutions, DEC);
  Serial.print("RPM: ");
  Serial.println(rpmilli, DEC);
}

//Capture The IR Break-Beam Interrupt
void diskInterrupt(){
  ticks++;
}








