#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

typedef struct struct_message {
  char msg[32];
} struct_message;

struct_message myData;

uint8_t bike_break = 0;
uint8_t bike_speed = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

volatile bool recvDisplayActive = false;
volatile bool recvMsgReady = false;
char recvMsg[32] = {0};
char recvMsgBuffer[32] = {0};  // Double buffer to prevent corruption
unsigned long recvDisplayUntil = 0;
SemaphoreHandle_t recvMsgMutex = NULL;
TaskHandle_t displayTaskHandle = NULL;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.printf("Received %d bytes from %02X:%02X:%02X:%02X:%02X:%02X: ", len, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  struct_message recv;
  size_t copyLen = len < sizeof(recv) ? len : sizeof(recv);
  memcpy(&recv, incomingData, copyLen);
  recv.msg[sizeof(recv.msg) - 1] = '\0';
  Serial.println(recv.msg);

  // Copy to buffer atomically
  if (xSemaphoreTake(recvMsgMutex, 0) == pdTRUE) {
    strncpy(recvMsgBuffer, recv.msg, sizeof(recvMsgBuffer) - 1);
    recvMsgBuffer[sizeof(recvMsgBuffer) - 1] = '\0';
    recvDisplayUntil = millis() + 5000;
    recvMsgReady = true;
    recvDisplayActive = true;
    xSemaphoreGive(recvMsgMutex);
  }
  
  // Notify display task to update LCD immediately
  if (displayTaskHandle != NULL) {
    xTaskNotifyFromISR(displayTaskHandle, 1, eSetBits, NULL);
  }
}

static void setIdleScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Road clear");
}

static void sendBroadcastAndShow(const char* text) {
  snprintf(myData.msg, sizeof(myData.msg), "%s", text);
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
  Serial.println(result == ESP_OK ? "Sent!" : "Error");
}

void setup() {
  Serial.begin(115200);
  
  // Create mutex for thread-safe message buffer
  recvMsgMutex = xSemaphoreCreateMutex();
  
  // Store the current task handle so ESP-NOW callback can notify us
  displayTaskHandle = xTaskGetCurrentTaskHandle();
  
  // Set device as Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init LCD
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP-NOW Ready");
  delay(1000);
  setIdleScreen();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

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
  // Check for notifications immediately without blocking
  uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, 0);
  
  // Check if a new receive message needs to be displayed
  if (recvMsgReady || notificationValue) {
    recvMsgReady = false;
    // Copy from buffer to display variable safely
    if (xSemaphoreTake(recvMsgMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      strncpy(recvMsg, recvMsgBuffer, sizeof(recvMsg) - 1);
      recvMsg[sizeof(recvMsg) - 1] = '\0';
      xSemaphoreGive(recvMsgMutex);
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Warning:");
      lcd.setCursor(0, 1);
      lcd.print(recvMsg);
    }
  }
  
  // If receive priority display expired, clear it and return to idle
  if (recvDisplayActive && millis() >= recvDisplayUntil) {
    recvDisplayActive = false;
    setIdleScreen();
  }

  bike_break = random(0, 2); // Simulate brake status (0 or 1)
  bike_speed = random(0, 51); // Simulate speed (0 to 50)
  if (bike_break) {
    sendBroadcastAndShow("Brake ON");
  } 
  else if (bike_speed <= 15) {
    sendBroadcastAndShow("Bump! Slow down");
  }
  
  delay(5000);
}

