#include <Arduino.h>
#include <WiFi.h>

/*
  ESP32-S3 TCP Client (Wi-Fi/TCP starts only after 3s hold)
  - After trigger: buzzer beeps twice (fixed)
  - Sends: AA BB | X_hi X_lo | Y_hi Y_lo | SW | CC DD
*/

// ---------- Pins (S3-safe) ----------
const int VRx = 4;     // ADC1 (GPIO1–10 are valid on S3)
const int VRy = 5;     // ADC1
const int SW  = 7;    // joystick push button (to GND), use INPUT_PULLUP
const int touchPin  = 2;   // digital: HIGH while held (e.g., TTP223 OUT)
const int buzzerPin = 10;  // active buzzer (HIGH = on)
const int R = 13;
const int G =12;
const int B =14;

// ---------- Wi-Fi/TCP ----------
const char* WIFI_SSID     = "eduvalor";
const char* WIFI_PASSWORD = "12345678";
const char* HOST_IP       = "192.168.4.1";
const uint16_t HOST_PORT  = 3333;

constexpr uint8_t START1 = 0xAA;
constexpr uint8_t START2 = 0xBB;
constexpr uint8_t END1   = 0xCC;
constexpr uint8_t END2   = 0xDD;

// ---------- Timing ----------
unsigned long lastWifiCheck = 0, lastSrvCheck = 0, lastPollMs = 0;
const unsigned long WIFI_CHECK_INTERVAL_MS = 3000;
const unsigned long SRV_CHECK_INTERVAL_MS  = 2000;
const unsigned long POLL_INTERVAL_MS       = 20;   // 50 Hz

// Touch-hold trigger
const unsigned long HOLD_MS   = 3000;
const int  BEEP_ON_MS  = 200;
const int  BEEP_OFF_MS = 150;

unsigned long touchStartTime = 0;
bool isTouching   = false;
bool triggeredNet = false;

WiFiClient client;

// ---------- Helpers ----------
void beepTwice() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(BEEP_ON_MS);
    digitalWrite(buzzerPin, LOW);
    if (i == 0) delay(BEEP_OFF_MS);
  }
}

void ensureWiFi() {
  analogWrite(B, 0);
  delay(5);
  analogWrite(R, 255);
  delay(5);
  analogWrite(G, 255);
  delay(5);
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
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
    beepTwice();
    analogWrite(B, 0);
    delay(5);
    analogWrite(R, 0);
    delay(5);
    analogWrite(G, 255);
    delay(5);
    
    Serial.println("[TCP] Connected.");
    return true;
  }
  Serial.println("[TCP] Connection failed.");
  analogWrite(R, 255);
  delay(5);
  analogWrite(G, 0);
  delay(5);
  analogWrite(B, 0);
  delay(5);
  client.stop();
  return false;
}

inline void sendJoystick9(int16_t x, int16_t y, uint8_t sw) {
  uint8_t pkt[9];
  pkt[0] = START1; pkt[1] = START2;
  pkt[2] = uint8_t((x >> 8) & 0xFF);
  pkt[3] = uint8_t( x       & 0xFF);
  pkt[4] = uint8_t((y >> 8) & 0xFF);
  pkt[5] = uint8_t( y       & 0xFF);
  pkt[6] = (sw & 0x01);
  pkt[7] = END1;   pkt[8] = END2;
  if (client.connected()) client.write(pkt, sizeof(pkt));
}

void setup() {
  // Make sure you opened the correct USB-CDC port in Serial Monitor
  Serial.begin(115200);

  delay(500);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\n=== ESP32-S3: WiFi starts after 3s touch ===");

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(touchPin, INPUT);     // use INPUT_PULLDOWN if your sensor idles LOW
  pinMode(SW, INPUT_PULLUP);    // button to GND
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  delay(500);
  analogWrite(G, 0);
  analogWrite(R, 0); 
  analogWrite(R, 255); 

}

void loop() {
  // 1) 3s hold to unlock/start networking
  int touchValue = digitalRead(touchPin);  // HIGH while held (invert if needed)
  if (touchValue == HIGH) {
    
    if (!isTouching) { isTouching = true; touchStartTime = millis(); }
    else if (!triggeredNet && (millis() - touchStartTime >= HOLD_MS)) {
      ensureWiFi();
      analogWrite(B, 0);
      delay(5);
      analogWrite(R, 255);
      delay(5);
      analogWrite(G, 255);
      delay(5);
      if (WiFi.status() == WL_CONNECTED) connectToServer();
      triggeredNet = true;  // don’t re-trigger while still held
    }
  } else {
    isTouching = false;

  }

  // 2) Keep Wi-Fi/TCP alive once unlocked
  if (triggeredNet) {
    if (millis() - lastWifiCheck >= WIFI_CHECK_INTERVAL_MS) {
      lastWifiCheck = millis();
      if (WiFi.status() != WL_CONNECTED) ensureWiFi();
    }
    if (millis() - lastSrvCheck >= SRV_CHECK_INTERVAL_MS) {
      lastSrvCheck = millis();
      if (WiFi.status() == WL_CONNECTED && !client.connected()) connectToServer();
    }
  }

  // 3) Send joystick at 50 Hz if connected
  if (triggeredNet && client.connected() && (millis() - lastPollMs >= POLL_INTERVAL_MS)) {
    lastPollMs = millis();
    int16_t x  = analogRead(VRx);
    int16_t y  = analogRead(VRy);
    uint8_t sw = (digitalRead(SW) == LOW) ? 1 : 0;
    sendJoystick9(x, y, sw);
  }

  if (!client.connected())
    
    delay(50);
}
