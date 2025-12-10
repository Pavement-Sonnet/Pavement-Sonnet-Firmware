// #include <Arduino.h>
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>

// LiquidCrystal_I2C lcd(0x27, 16, 2);

// void setup() {
//   lcd.begin();
//   lcd.backlight();
//   lcd.setCursor(0, 0);
//   lcd.print("Hello ESP32!");
//   lcd.setCursor(0, 1);
//   lcd.print("Qapass LCD OK");
// }

// void loop() {}

#include <WiFi.h>
#include <esp_now.h>

typedef struct struct_message {
  char msg[32];
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  
  // Set device as Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback
  esp_now_register_send_cb(OnDataSent);

  // Define receiver MAC address
  uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}; // Replace with receiver MAC

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  //if (!esp_now_is_peer_exist(broadcastAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  //}
}

void loop() {
  strcpy(myData.msg, "Hello ESP32 Receiver!");
  uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}; // Replace with receiver MAC
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  Serial.println(result == ESP_OK ? "Sent!" : "Error");
  delay(2000);
}

