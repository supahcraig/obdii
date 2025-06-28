// ESP32 + ELM327 Bluetooth + I2C LCD Example
// Wiring:
//   ESP32  <-> LCD (I2C)
//     GPIO 21 -> SDA
//     GPIO 22 -> SCL
//     3.3V     -> VCC
//     GND      -> GND
//
//   ESP32 <-> ELM327 (BT Classic SPP)
//     No wiring: uses built-in Bluetooth
//
// IMPORTANT: BluetoothSerial.connect() only supports MAC addresses.
// You must obtain your ELM327's Bluetooth MAC. For example:
//   • On iOS: Settings > Bluetooth > tap ⓘ next to the device
//   • On Android: Settings > Bluetooth > gear icon > Advanced
//   • On Linux: run `hcitool scan` in a terminal

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "BluetoothSerial.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);
BluetoothSerial SerialBT;

// Replace with your ELM327's actual MAC address
const char* elmMac = "OBDII";
const char* elmPin = "1234"; // Default PIN for most ELM327 modules

static const uint8_t BLUE_LED_PIN = 2;

// Interval between PID requests (ms)
const unsigned long requestInterval = 1000;
unsigned long lastRequestTime = 0;

bool connectToBTDevice(const char* mac) {
  // ensure LED is off while we try
  digitalWrite(BLUE_LED_PIN, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting...");
  Serial.printf("Trying to connect to %s …\n", mac);

  if (SerialBT.connect(mac)) {
    lcd.setCursor(1, 0);
    lcd.print("Connected!");
    Serial.println("✓ Connected!");
    digitalWrite(BLUE_LED_PIN, HIGH);
    return true;
  } else {
    Serial.println("✗ Connection failed");
    return false;
  }
}

// Send ELM command with CR, return response up to '>'
String sendELM(const String &cmd, unsigned long timeout = 5000) {
  while (SerialBT.available()) SerialBT.read();
  Serial.print("<< " + cmd + "\n");
  SerialBT.print(cmd);
  SerialBT.write('\r');

  String resp;
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') break;
      resp += c;
    }
  }
  return resp;
}



void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  //initialize LED
  pinMode(2, OUTPUT);

  // Initialize Bluetooth in master mode
  SerialBT.setPin(elmPin);
  if (!SerialBT.begin("ESP32_OBD_Client", true)) {
    lcd.setCursor(0, 0);
    lcd.print("BT init failed");
    while (true) delay(1000);
  }

  bool conn = connectToBTDevice(elmMac);
  delay(2000);

    // Verify and auto-fallback protocol for 1997 Jeep
  String proto = sendELM("ATDP", 2000);
  Serial.println("Protocol: " + proto);
  if (proto.indexOf("CAN") >= 0) {
    Serial.println("Switching to ISO9141-2 for 1997 Jeep");
    sendELM("ATSP3", 2000);  // ISO9141-2
    proto = sendELM("ATDP", 2000);
    Serial.println("New Protocol: " + proto);
  }
  lcd.clear(); lcd.print("Proto:");
  lcd.setCursor(0,1);
  lcd.print(proto.substring(0,16));
  delay(2000);

  lcd.clear(); lcd.print("ELM ready"); delay(500);
  lcd.clear(); lcd.print("Raw inspect");

}


void loop() {
  if (!SerialBT.connected()) {
    // reconnect?
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connection lost");
    digitalWrite(BLUE_LED_PIN, LOW);
    delay(2000);

    lcd.setCursor(1, 0);
    lcd.print("Reconnecting....");
    connectToBTDevice(elmMac);

    return;
  }

  ///////////////

    unsigned long now = millis();
  if (now - lastRequestTime < requestInterval) {
    return; // skip until interval elapsed
  }
  lastRequestTime = now;

  // Request RPM (Service 01 PID 0C)
  String raw = sendELM("010C", 2000);
  Serial.println("--- RAW START ---");
  Serial.println(raw);
  Serial.println("---- RAW END ----");

  // Parse response '410C' frame
  int idx = raw.indexOf("410C");
  if (idx >= 0) {
    String payload = raw.substring(idx + 4);
    payload.replace(" ", "");
    if (payload.length() >= 4) {
      int A = strtol(payload.substring(0,2).c_str(), NULL, 16);
      int B = strtol(payload.substring(2,4).c_str(), NULL, 16);
      int rpm = ((A << 8) | B) / 4;
      Serial.printf("Parsed RPM: %d rpm\n", rpm);
      lcd.clear(); lcd.print("RPM:"); lcd.print(rpm);
    } else {
      Serial.println("Err: data too short");
    }
  } else {
    Serial.println("Err: no 410C frame");
  }

}
