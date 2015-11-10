void setup() {     
  Serial.begin(57600);  
  pinMode(13, OUTPUT);
}
 
int f = 0, pos;
void loop() {
  boolean has_request = false;
  String in = "";
  if (Serial.available()) {
    in = "";
    while (true) {  // should add time out here
      while (Serial.available() == false) {}
      in += (char)(Serial.read());
      if (in.endsWith("\r\n\r\n")) {
        has_request = true;  break;
      }
    }
  }
  if (has_request) {
    int i1 = in.indexOf("GET /blink?f="), i2;
    if (i1 != -1) {
      i2 = in.indexOf(" ", i1+13);
      f = in.substring(i1+13, i2).toInt();
    }
    Serial.println("HTTP/1.1 200 OK\nContent-Type: text/html\nConnection: close");
    String sr = "<!DOCTYPE HTML>\n";
    sr += "<html>\n";
    sr += "LED frequency: ";
    sr += f;
    sr += "Hz.</html>";
    Serial.print("Content-Length: ");
    Serial.print(sr.length());
    Serial.print("\r\n\r\n");
    Serial.print(sr);
    has_request = false;
  }
  if (f>0) {
    static unsigned long t = millis();
    if (millis() > t + 1000/f) {
      digitalWrite(13, 1-digitalRead(13));
      t = millis();
    }
  }
}
