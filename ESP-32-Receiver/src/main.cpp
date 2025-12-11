#include <WiFi.h>
#include <esp_now.h>

//For LCD Control -Arduino LiquidCrystal I2C Library-
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


typedef struct struct_message {
  char msg[32];
} struct_message;

struct_message incomingData;

// Multicast MAC address (broadcast)
uint8_t mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

LiquidCrystal_I2C lcd(0x27, 16, 2);

void lcd_setup() {
    lcd.begin();
    lcd.backlight();
}

void lcd_message() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Rear bike connected:");
    lcd.setCursor(0, 1);
    lcd.print("Crash Detected!");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  Serial.print("Received from MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" - Message: ");
  Serial.println(incomingData.msg);



  

}

void setup() {
  Serial.begin(115200);
  Serial.println(WiFi.macAddress());

  
  // Set device as Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing needed here; data is handled in callback

}
