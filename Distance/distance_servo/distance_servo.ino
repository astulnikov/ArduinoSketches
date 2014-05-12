#include <Servo.h> 

Servo myServo;

int ledPin = 13;
int trigPin = 11;
int echoPin = 12;

long distance;
int count;
int pos;

void setup()  {
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  myServo.attach(9);
  Serial.begin(9600);
}

void loop()  { 
  distance = getDistance();
  Serial.print(distance);
  Serial.print(", ");  
  ledBlink(distance);
  reactOnDistance(distance);
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
      myServo.write(angle); 
    } 
    else {
      myServo.write(1); 
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

void reactOnDistance(int distance) {
  if(distance < 110) {
    float degreePerCentimeter = 26.0 / 110.0;
    Serial.print(degreePerCentimeter);
    Serial.print(", "); 
    int steeringAngle = 90 + 26 - degreePerCentimeter * distance;
    myServo.write(steeringAngle);
    Serial.print(steeringAngle);
    Serial.println(", ");  
  } 
  else {
    myServo.write(90);  
  }
}





