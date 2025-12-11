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

// Timer variables for LCD clear
unsigned long lastMessageTime = 0;
const unsigned long LCD_CLEAR_TIME = 10000; // 10 seconds in milliseconds
bool messageLcdActive = false;

void lcd_setup() {
    lcd.begin();
    lcd.backlight();
}

void lcd_message(const char* message) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bike connected:");
    lcd.setCursor(0, 1);
    lcd.print(message);
}

void lcd_message_clear() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Road Clear");
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
  lcd_setup();
  lcd_message(incomingData.msg);
  
  // Record the time when message was displayed
  lastMessageTime = millis();
  messageLcdActive = true;
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
  
  // Initialize LCD with default message
  lcd_setup();
  lcd_message_clear();
}

void loop() {
  // Check if LCD message should be cleared
  if (messageLcdActive && (millis() - lastMessageTime) >= LCD_CLEAR_TIME) {
    lcd_message_clear();
    messageLcdActive = false;
    Serial.println("LCD displaying Road Clear after 10 seconds");
  }
}
