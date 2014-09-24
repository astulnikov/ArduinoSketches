const int MAX_SPEED_FORWARD = 250;
const int MAX_SPEED_BACKWARD = 10;

//Motor driver section Left-Front
const int L_R_A_IA = 11; // A-IA
const int L_R_A_IB = 12; // A-IB

//Motor driver section Left-Rear
const int L_R_B_IA = 9;  // B-IA
const int L_R_B_IB = 10; // B-IB

//Motor driver section Right-Front
const int R_R_A_IA = 6;  // A-IA
const int R_R_A_IB = 7; // A-IB

//Motor driver section Right-Rear
const int R_R_B_IA = 4; // B-IA
const int R_R_B_IB = 5; // B-IB

void setup() {

  // initialize serial communication
  Serial.begin(9600);

  //left front motor init
  pinMode(L_R_A_IA,OUTPUT);
  digitalWrite(L_R_A_IA, LOW);
  pinMode(L_R_A_IB,OUTPUT);
  digitalWrite(L_R_A_IB, LOW);

  //left rear motors init
  pinMode(L_R_B_IA,OUTPUT);
  digitalWrite(L_R_B_IA, LOW);
  pinMode(L_R_B_IB,OUTPUT);
  digitalWrite(L_R_B_IB, LOW);

  //right front motor init
  pinMode(R_R_A_IA,OUTPUT);
  digitalWrite(R_R_A_IA, LOW);
  pinMode(R_R_A_IB,OUTPUT);
  digitalWrite(R_R_A_IB, LOW);

  //right rear motors init
  pinMode(R_R_B_IA,OUTPUT);
  digitalWrite(R_R_B_IA, LOW);
  pinMode(R_R_B_IB,OUTPUT);
  digitalWrite(R_R_B_IB, LOW);
}

void loop() {
  makeMove();
  delay(500);
  doStop(); 
}

void makeMove(){
  //  for(int i = 80; i >= 1; i--){
  //    goForward(i);
  //    delay(500);
  //  }
  goForward(80);
}

void goForward(int percent){
  Serial.print("RUN FORWARD ");
  int speed = MAX_SPEED_FORWARD / 100.0 * percent;
  Serial.println(speed);

  digitalWrite(L_R_A_IA, HIGH);
  analogWrite(L_R_A_IB, speed);

  digitalWrite(L_R_B_IA, HIGH); 
  analogWrite(L_R_B_IB, speed);

  digitalWrite(R_R_A_IA, HIGH); 
  analogWrite(R_R_A_IB, speed);

  digitalWrite(R_R_B_IA, HIGH); 
  analogWrite(R_R_B_IB, speed);
}

void doStop(){
  Serial.println("STOP");
  digitalWrite(L_R_A_IA, LOW);
  digitalWrite(L_R_A_IB, LOW);
  digitalWrite(L_R_B_IA, LOW);
  digitalWrite(L_R_B_IB, LOW);

  digitalWrite(R_R_A_IA, LOW);
  digitalWrite(R_R_A_IB, LOW);
  digitalWrite(R_R_B_IA, LOW);
  digitalWrite(R_R_B_IB, LOW);
}






