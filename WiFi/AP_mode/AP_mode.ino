#include <WiFiRM04.h>

char ssid[] = "shamm_arduino";   

int status = WL_IDLE_STATUS;
//WiFiRM04Server server(80);

void setup() {
  Serial.begin(9600);      // initialize serial communication

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    while(true);        // don't continue
  } 
  bool result = false;
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to create Network named: ");
    Serial.println(ssid); 
    result = WiFi.beginAsAp(ssid);
    Serial.print("Result: ");
    Serial.println(result); 
    delay(10000);
  }
}

void loop() {
  printWifiStatus();
  delay(10000);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}



