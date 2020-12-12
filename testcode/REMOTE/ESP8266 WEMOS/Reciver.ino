#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char *ssid = "PWM_TEST";
const char *password = "Polybius";

ESP8266WebServer server(80);

void handleSentVar() {
  if (server.hasArg("joystick")) { // this is the variable sent from the client
    int readingInt = server.arg("joystick").toInt();
    //char readingToPrint[5];
    server.send(200, "text/html", "OK");
    if (readingInt>0) {Serial.println(readingInt);}
  }
}

void setup() {
  delay(1000);

  Serial.begin(9600);

  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();

  server.on("/data/", HTTP_GET, handleSentVar); // when the server receives a request with /data/ in the string then run the handleSentVar function
  server.begin();
}

void loop() {
  server.handleClient();
}
