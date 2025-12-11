// #include <Arduino.h>
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>



// void loop() {}

#include <WiFi.h>
#include <esp_now.h>

typedef struct struct_message {
  char msg[32];
} struct_message;

struct_message myData;

uint8_t bike_break = 0;
uint8_t bike_speed = 0;

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

  // Define broadcast MAC address (FF:FF:FF:FF:FF:FF)
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    return;
  }
}

void loop() {
  bike_break = random(0, 2); // Simulate brake status (0 or 1)
  bike_speed = random(0, 51); // Simulate speed (0 to 50)
  if (bike_break) {
    snprintf(myData.msg, sizeof(myData.msg), "Brake ON");
    // Broadcast MAC address
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    Serial.println(result == ESP_OK ? "Sent!" : "Error");
  } 
  else if (bike_speed <= 15) {
    snprintf(myData.msg, sizeof(myData.msg), "Bump! Slow down");
    // Broadcast MAC address
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    Serial.println(result == ESP_OK ? "Sent!" : "Error");
  }
  
  delay(2000);
}

