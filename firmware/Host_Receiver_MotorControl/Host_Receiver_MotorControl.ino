

#include <WiFi.h>

const char* ssid = "eduvalor";
const char* password = "12345678"; // must be at least 8 characters

WiFiServer server(3333); // TCP server on port 3333

void setup() {
  Serial.begin(115200);

  // Start WiFi in Access Point mode
  WiFi.softAP(ssid, password);

  Serial.print("ESP32 AP started. IP: ");
  Serial.println(WiFi.softAPIP());

  // Start TCP server
  server.begin();
  Serial.println("TCP server started on port 3333");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected!");

    while (client.connected()) {
      while (client.available()) {
        char c = client.read();
        Serial.print("Received: ");
        Serial.println(c);
      }
      delay(10);
    }

    client.stop();
    Serial.println("Client disconnected.");
  }
}