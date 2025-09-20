#include <WiFi.h>

/// ================== Motor Pins ==================
#define ENA 21
#define ENB 22
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26

int last_x_value = 0;
int last_y_value = 0;

//////////////////// USER SETTINGS ////////////////////
const char* WIFI_SSID     = "eduvalor";
const char* WIFI_PASSWORD = "12345678";
const uint16_t HOST_PORT  = 3333;
///////////////////////////////////////////////////////

volatile int16_t last_joystick_value[3] = {0, 0, 0};

WiFiServer server(HOST_PORT);
WiFiClient client;

constexpr uint8_t START1 = 0xAA;
constexpr uint8_t START2 = 0xBB;
constexpr uint8_t END1   = 0xCC;
constexpr uint8_t END2   = 0xDD;

uint8_t rxBuf[9];
uint8_t rxPos = 0;

// Timeout settings
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT_MS = 1000; // 1 second

void resetJoystickValues() {
  noInterrupts();
  last_joystick_value[0] = 0;
  last_joystick_value[1] = 0;
  last_joystick_value[2] = 0;
  interrupts();
  Serial.println("[DATA] Reset to X=0, Y=0, SW=0");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP32 TCP Server ===");

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] AP started. IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  server.setNoDelay(true);

  resetJoystickValues();
}

void setMotorA(int speed) {
  if (speed < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(speed));
}

void setMotorB(int speed) {
  if (speed < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, abs(speed));
}

void loop() {
  // Accept a new client if none connected
  if (!client || !client.connected()) {
    if (client) {
      Serial.println("[TCP] Client disconnected!");
      client.stop();
      resetJoystickValues();
    }
    client = server.available();
    if (client) {
      Serial.println("[TCP] Client connected!");
      rxPos = 0;
      lastReceiveTime = millis();
    }
    delay(50);
    return;
  }

  // Read incoming data
  while (client.available()) {
    uint8_t b = client.read();
    lastReceiveTime = millis();  // update last received time

    // 9-byte frame parser
    if (rxPos == 0 && b != START1) continue;
    if (rxPos == 1 && b != START2) { rxPos = 0; continue; }

    rxBuf[rxPos++] = b;

    if (rxPos == 9) {
      if (rxBuf[7] == END1 && rxBuf[8] == END2) {
        int16_t x = (rxBuf[2] << 8) | rxBuf[3];
        int16_t y = (rxBuf[4] << 8) | rxBuf[5];
        uint8_t sw = rxBuf[6] & 0x01;

        noInterrupts();
        last_joystick_value[0] = x;
        last_joystick_value[1] = y;
        last_joystick_value[2] = sw;
        interrupts();


        Serial.printf("[DATA] X=%d, Y=%d, SW=%u\n", x, y, sw);
        int xMapped = map(x, 0, 4095, -100, 100);
        int yMapped = map(y, 0, 4095, -100, 100);
        // Arcade drive mixing
        
        int rightMotor  = yMapped + xMapped;  // forward/back + turn
        int leftMotor = yMapped - xMapped;
        // Constrain to PWM range
        leftMotor  = constrain(leftMotor, -100, 100);
        rightMotor = constrain(rightMotor, -100, 100);

        // Set motors
        if ((xMapped>-30 && xMapped <30) && (yMapped>-30 && yMapped <30)){
          setMotorA(0);
          setMotorB(0);
        }
        else{
          if ((x != last_x_value) || (y != last_y_value) )
            {
            setMotorA(leftMotor);
            setMotorB(rightMotor);
            // Debug
            Serial.print("X:");
            Serial.print(xMapped);
            Serial.print(" Y:");
            Serial.print(yMapped);
            Serial.print(" L:");
            Serial.print(leftMotor);
            Serial.print(" R:");
            Serial.println(rightMotor);
            }
        }

        last_x_value = x;
        last_y_value = y;


      }
      rxPos = 0;
    }
  }

  // Check for timeout (client idle)
  if (millis() - lastReceiveTime > TIMEOUT_MS) {
    if (client && client.connected()) {
      Serial.println("[TCP] Client idle timeout, disconnecting...");
      client.stop();
      resetJoystickValues();
    }
  }
}