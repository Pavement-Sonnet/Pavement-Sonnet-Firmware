#include <WiFi.h>
#include <esp_now.h>

typedef struct struct_message {
  char msg[32];
} struct_message;

struct_message incomingData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  Serial.print("Received: ");
  Serial.println(incomingData.msg);
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

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing needed here; data is handled in callback
}
