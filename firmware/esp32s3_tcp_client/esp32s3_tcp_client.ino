/*
  ESP32-S3 TCP Client (Joystick via shared array)
  - Connects to Wi-Fi
  - Connects to TCP host
  - Sends ONLY when any of [X, Y, SW] changes
  - Frame: AA BB | X_hi X_lo | Y_hi Y_lo | SW | CC DD
*/

#include <WiFi.h>

//////////////////// USER SETTINGS ////////////////////
const char* WIFI_SSID     = "eduvalor";
const char* WIFI_PASSWORD = "12345678";
const char* HOST_IP       = "192.168.4.1";
const uint16_t HOST_PORT  = 3333;
///////////////////////////////////////////////////////

// -------- Shared data (UPDATED ELSEWHERE) --------
// [0] = X (int16), [1] = Y (int16), [2] = SW (0/1)
volatile int16_t last_joystick_value[3] = {0, 0, 0};
// -------------------------------------------------

WiFiClient client;

// Frame constants
constexpr uint8_t START1 = 0xAA;
constexpr uint8_t START2 = 0xBB;
constexpr uint8_t END1   = 0xCC;
constexpr uint8_t END2   = 0xDD;

// Timing
unsigned long lastWifiCheck = 0;
unsigned long lastSrvCheck  = 0;
unsigned long lastPollMs    = 0;

// Intervals
const unsigned long WIFI_CHECK_INTERVAL_MS = 3000;
const unsigned long SRV_CHECK_INTERVAL_MS  = 2000;
const unsigned long POLL_INTERVAL_MS       = 20;   // 50 Hz poll of shared array

// Last sent snapshot
bool firstSend = true;
int16_t last_sent[3] = {0, 0, 0};  // [X, Y, SW]

// ---- Net helpers ----
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
    Serial.print("[WiFi] Connected. IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Failed to connect.");
  }
}

bool connectToServer() {
  if (client.connected()) return true;
  Serial.printf("[TCP] Connecting to %s:%u ...\n", HOST_IP, HOST_PORT);
  client.setNoDelay(true);
  client.setTimeout(2000);
  if (client.connect(HOST_IP, HOST_PORT)) {
    Serial.println("[TCP] Connected.");
    return true;
  }
  Serial.println("[TCP] Connection failed.");
  client.stop();
  return false;
}

// ---- Packet TX ----
// Build + send: AA BB | X_hi X_lo | Y_hi Y_lo | SW | CC DD
inline void sendJoystick9(int16_t x, int16_t y, uint8_t sw) {
  uint8_t pkt[9];
  pkt[0] = START1; pkt[1] = START2;
  pkt[2] = uint8_t((x >> 8) & 0xFF);
  pkt[3] = uint8_t( x       & 0xFF);
  pkt[4] = uint8_t((y >> 8) & 0xFF);
  pkt[5] = uint8_t( y       & 0xFF);
  pkt[6] = (sw & 0x01);               // 1 byte switch
  pkt[7] = END1;   pkt[8] = END2;

  if (client.connected()) client.write(pkt, sizeof(pkt));
}

// Check shared array, send only if changed
inline void maybeSendFromShared() {
  // Snapshot the shared values (briefly block ISRs if they might update it)
  int16_t x, y, sw16;
  noInterrupts();
  x    = last_joystick_value[0];
  y    = last_joystick_value[1];
  sw16 = last_joystick_value[2];
  interrupts();

  uint8_t sw = (uint8_t)(sw16 & 0x01);

  // Skip if no change since last send
  if (!firstSend &&
      x == last_sent[0] &&
      y == last_sent[1] &&
      sw == (uint8_t)(last_sent[2] & 0x01)) {
    return;
  }

  // Send and update snapshot
  sendJoystick9(x, y, sw);
  firstSend     = false;
  last_sent[0]  = x;
  last_sent[1]  = y;
  last_sent[2]  = sw;  // store as 0/1 for comparison
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP32-S3 TCP Client (Joystick via shared array) ===");

  ensureWiFi();
  connectToServer();
}

void loop() {
  // Keep Wi-Fi alive
  if (millis() - lastWifiCheck >= WIFI_CHECK_INTERVAL_MS) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) ensureWiFi();
  }

  // Keep TCP connection alive
  if (millis() - lastSrvCheck >= SRV_CHECK_INTERVAL_MS) {
    lastSrvCheck = millis();
    if (WiFi.status() == WL_CONNECTED && !client.connected()) connectToServer();
  }

  // Poll the shared array and send only on change
  if (client.connected() && (millis() - lastPollMs >= POLL_INTERVAL_MS)) {
    lastPollMs = millis();
    maybeSendFromShared();
  }

  if (!client.connected()) delay(50);
}
