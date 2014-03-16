#include <Servo.h> 

Servo myservo;

int ledPin = 13;
int trigPin = 10;
int echoPin = 11;

long distance;
int count;
int pos;

void setup()  {
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  myservo.attach(9);
  Serial.begin(9600);
}

void loop()  { 
  distance = getDistance();
  Serial.print(distance);
  Serial.print(", ");
  ledBlink(distance);
  rotateServo(distance);
} 

void ledBlink(int time) {
  digitalWrite(ledPin, HIGH);
  delay(time);
  digitalWrite(ledPin, LOW);
  delay(time);
}

void rotateServo(int angle) {
  count++;
  pos += angle;
  pos = pos / 2;
  if(count > 5) {
    if(angle <= 180) {
      myservo.write(angle); 
    } 
    else {
      myservo.write(1); 
    }
    count = 0;
  }
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



