/*
  ESP32-S3 TCP Client
  - Connects to Wi-Fi
  - Connects to a TCP host (another ESP32-S3 acting as server)
  - Auto-reconnects to Wi-Fi and server
  - Sends heartbeat every 2s
  - Reads and prints incoming data
*/

#include <WiFi.h>

//////////////////// USER SETTINGS ////////////////////
const char* WIFI_SSID     = "eduvalor";
const char* WIFI_PASSWORD = "12345678";

// Server (the other ESP32-S3 running a TCP server)
const char* HOST_IP   = "192.168.4.1";  // <-- put the server's IP here
const uint16_t HOST_PORT = 3333;         // <-- must match server port
///////////////////////////////////////////////////////

WiFiClient client;

// Timing
unsigned long lastWifiCheck   = 0;
unsigned long lastSrvCheck    = 0;
unsigned long lastHeartbeatMs = 0;

// Intervals (tune to taste)
const unsigned long WIFI_CHECK_INTERVAL_MS = 3000;
const unsigned long SRV_CHECK_INTERVAL_MS  = 2000;
const unsigned long HEARTBEAT_INTERVAL_MS  = 2000;

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("[WiFi] Connecting to "); Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Failed to connect.");
  }
}

bool connectToServer() {
  if (client.connected()) return true;

  Serial.printf("[TCP] Connecting to %s:%u ...\n", HOST_IP, HOST_PORT);

  // Optional: tweak client before connect
  client.setNoDelay(true);              // disable Nagle for lower latency
  client.setTimeout(2000);              // read timeout

  if (client.connect(HOST_IP, HOST_PORT)) {
    Serial.println("[TCP] Connected to server.");
    // Optional: send an initial hello
    client.println("HELLO_FROM_CLIENT");
    return true;
  } else {
    Serial.println("[TCP] Connection failed.");
    client.stop();
    return false;
  }
}

void sendHeartbeat() {
  if (!client.connected()) return;

  static uint32_t count = 0;
  String msg = "HEARTBEAT " + String(++count);
  client.println(msg);
  Serial.println("[TCP] >> " + msg);
}

void readIncoming() {
  // Read lines if available
  while (client.connected() && client.available()) {
    String line = client.readStringUntil('\n'); // respects client.setTimeout
    line.trim();
    if (line.length() > 0) {
      Serial.println("[TCP] << " + line);
      // TODO: parse and act on commands from server here
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== ESP32-S3 TCP Client ===");
  ensureWiFi();
  connectToServer();
}

void loop() {
  // Keep Wi-Fi alive
  if (millis() - lastWifiCheck >= WIFI_CHECK_INTERVAL_MS) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      ensureWiFi();
    }
  }

  // Keep TCP connection alive
  if (millis() - lastSrvCheck >= SRV_CHECK_INTERVAL_MS) {
    lastSrvCheck = millis();
    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
      connectToServer();
    }
  }

  // If connected, communicate
  if (client.connected()) {
    readIncoming();

    // Heartbeat every 2 seconds
    if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
      lastHeartbeatMs = millis();
      sendHeartbeat();
    }
  } else {
    // Not connected to server: short nap before next attempt cycle
    delay(100);
  }
}
