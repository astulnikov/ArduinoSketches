int ledPin = 13;
int trigPin = 10;
int echoPin = 11;

long distance;
int ledLevel; 

void setup()  {
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  Serial.begin(9600);
}

void loop()  { 
  distance = getDistance();
  Serial.print(distance);
  Serial.print(", ");
  ledBlink(distance);
} 

void ledBlink(int time) {
  digitalWrite(ledPin, HIGH);
  delay(time);
  digitalWrite(ledPin, LOW);
  delay(time);
}

long getEchoTiming() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin,HIGH);
  return duration;
} 

long getDistance() {
  long distacne_cm = getEchoTiming()/29/2;
  return distacne_cm;
}

