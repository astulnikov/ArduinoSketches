#include <Servo.h> 

//Motor driber section
int R_A_IA = 5; // A-IA
int R_A_IB = 6; // A-IB

const int LED = 13; 

Servo myServo; 

void setup() { 
  pinMode(LED, OUTPUT);

  //motors init
  pinMode(R_A_IA,OUTPUT);
  digitalWrite(R_A_IA, LOW);
  pinMode(R_A_IB,OUTPUT);
  digitalWrite(R_A_IB, LOW);
  
  myServo.attach(9); 
}

void loop() { 
  digitalWrite(LED, HIGH);
  analogWrite(R_A_IA, 150);
  digitalWrite(R_A_IB, LOW);

  delay(1000);
  
  digitalWrite(LED, LOW);
  digitalWrite(R_A_IA, LOW);
  digitalWrite(R_A_IB, LOW);
  
  delay(1000);
  
  digitalWrite(LED, HIGH);
  analogWrite(R_A_IA, 150);
  digitalWrite(R_A_IB, HIGH);

  delay(1000);
  
  digitalWrite(LED, LOW);
  digitalWrite(R_A_IA, LOW);
  digitalWrite(R_A_IB, LOW);
  
  delay(1000);
}

