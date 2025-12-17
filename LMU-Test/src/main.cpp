// Arduino Libraries
#include <Arduino.h>

// Accelerometer, GPS, Temperature Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Module Headers
#include "config.h"
#include "mqtt_config.h"
#include "wifi_manager.h"
#include "mqtt_client.h"
#include "ble_manager.h"
#include "power_manager.h"
#include "mpu_manager.h"
#include "sensor_manager.h"

// 全域變數
static unsigned long lastPublish = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("\n=== ESP32 WiFi & BLE Test ===");

  // 1. 電源軌初始化 (Power Rails Init)
  power_init();

  // 2. 按鈕腳位設定
  pinMode(buttonPin, INPUT_PULLUP);

  // 3. 初始化 BLE
  ble_init();

  // 4. WiFi 與 MQTT 設定
  if (ENABLE_WIFI) {
    wifi_connect();
    mqtt_init();
  }

  // 5. 感測器初始化
  sensors_init();

  // 6. MPU 初始化
  mpu_init();
}

void loop() {
  mqtt_loop();

  // Deep Sleep Check
  if (millis() > RUN_TIME_MS) {
    Serial.println("Time's up! Sleep...");

    // 1. Rail 1 處理: 透過 I2C 指令進入低功耗模式
    if (ENABLE_MPU) mpu_setup_wom_low_power();
    
    // 2. 切斷其他電源軌
    power_shutdown_rails();

    // Check Later:
    // 注意: GPIO 在 Deep Sleep 時會變成高阻抗(Floating)狀態
    // 如果你的 MOSFET 需要持續的 LOW 來保持關閉，你可能需要在 Gate 端加一個下拉電阻(Pull-down resistor)
    // 這樣當 ESP32 睡著(腳位放開)時，電阻會把 Gate 拉低，確保 MOSFET 關閉。

    Serial.println("[System] Entering Deep Sleep");
    Serial.flush(); // 確保訊息印完

    esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)mpuIntPin, 0); 
    esp_deep_sleep_start();
  }

  // ================= BLE Button Trigger Logic =================
  ble_handle_button();

  // Data Collection & Print
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    SensorData data = sensors_read_all();
    
    Serial.printf("[Sensors] Temp: %.2f | BLE: %s | Sound: %d | AirQuality: %d | GPS: %.6f, %.6f\n", 
      data.temperature, ble_get_status_string(), data.soundLevel, data.airQuality, 
      data.latitude, data.longitude);
    lastPrint = millis();
  }

  // MQTT Publish
  unsigned long now = millis();
  if (now - lastPublish > 5000) {
    lastPublish = now;
    
    SensorData data = sensors_read_all();
    
    char gpsBuf[256];
    char accelFrag[128];
    char accelPayload[256];
    char sensorBuf[256];

    // 加速度數據
    if(ENABLE_MPU) {
      sensors_event_t a, g, tempEvent;
      mpu_read_accel(&a, &g, &tempEvent);
      snprintf(accelFrag, sizeof(accelFrag), "\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f",
        a.acceleration.x, a.acceleration.y, a.acceleration.z);
    } else {
      strcpy(accelFrag, "\"accel_x\":null,\"accel_y\":null,\"accel_z\":null");
    }
    
    // 準備 MQTT Payloads
    snprintf(gpsBuf, sizeof(gpsBuf), "{\"latitude\":%.6f,\"longitude\":%.6f}", 
      data.latitude, data.longitude);
    snprintf(accelPayload, sizeof(accelPayload), "{%s}", accelFrag);
    snprintf(sensorBuf, sizeof(sensorBuf), "{\"temperature\":%.2f,\"sound_level\":%d,\"air_quality\":%d}", 
      data.temperature, data.soundLevel, data.airQuality);

    // 發布 GPS、加速度和感測器到不同主題
    bool ok1 = mqtt_publish(MQTT_TOPIC_GPS, gpsBuf);
    bool ok2 = mqtt_publish(MQTT_TOPIC_ACCEL, accelPayload);
    bool ok3 = mqtt_publish(MQTT_TOPIC_SENSORS, sensorBuf);

    Serial.print("Published GPS -> "); Serial.print(MQTT_TOPIC_GPS); Serial.print(" : "); Serial.println(gpsBuf);
    Serial.print("Publish result: "); Serial.println(ok1 ? "OK" : "FAILED");
    Serial.print("Published Accel -> "); Serial.print(MQTT_TOPIC_ACCEL); Serial.print(" : "); Serial.println(accelPayload);
    Serial.print("Publish result: "); Serial.println(ok2 ? "OK" : "FAILED");
    Serial.print("Published Sensors -> "); Serial.print(MQTT_TOPIC_SENSORS); Serial.print(" : "); Serial.println(sensorBuf);
    Serial.print("Publish result: "); Serial.println(ok3 ? "OK" : "FAILED");
  }
  
  delay(10);
}