// Arduino Libraries
#include <Arduino.h>
#include <math.h>

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
#include "mock_data.h"

// 全域變數
static unsigned long lastPublish = 0;

// 連續加速度採樣與路況分析設定
static const unsigned long SENSOR_PUBLISH_INTERVAL_MS = 15000; // 15 秒上報一次
static const unsigned long ACCEL_SAMPLE_INTERVAL_MS   = 20;    // 50 Hz 採樣率
static const size_t ACCEL_BUF_SIZE = 750;                      // 約 15 秒窗口 (50 Hz * 15 s)
static float accelBuf[ACCEL_BUF_SIZE];
static size_t accelIndex = 0;
static bool accelFilled = false;
static unsigned long lastAccelSample = 0;
static const float POTHOLE_TRIGGER_G = 3.5f; // 簡易坑洞門檻 (g)
static bool potholeDetected = false;
static float potholeLat = 0.0f;
static float potholeLon = 0.0f;
static unsigned long lastPotholeMark = 0;

struct RoadState {
  uint8_t score;
  bool potholeFlag;
  float lat;
  float lon;
};

// 將最新加速度樣本寫入循環緩衝
static void push_accel_sample(float magnitude) {
  accelBuf[accelIndex] = magnitude;
  accelIndex = (accelIndex + 1) % ACCEL_BUF_SIZE;
  if (accelIndex == 0) accelFilled = true;
}

// 簡易 DFT，計算 3~15Hz 頻帶能量並映射成 1~255 分數
static uint8_t compute_road_score(float sampleRateHz) {
  const size_t N = accelFilled ? ACCEL_BUF_SIZE : accelIndex;
  if (N < 16) return 1; // 資料不足

  // 去除偏移 (含重力)
  float mean = 0.0f;
  for (size_t i = 0; i < N; ++i) mean += accelBuf[i];
  mean /= N;

  // 頻帶能量 (naive DFT)
  const float bandLow = 3.0f;
  const float bandHigh = 15.0f;
  float bandEnergy = 0.0f;
  for (size_t k = 1; k < N / 2; ++k) {
    float freq = (sampleRateHz * k) / N;
    if (freq < bandLow || freq > bandHigh) continue;
    float real = 0.0f;
    float imag = 0.0f;
    for (size_t n = 0; n < N; ++n) {
      float v = accelBuf[(accelFilled ? (n + accelIndex) % ACCEL_BUF_SIZE : n)] - mean;
      float angle = -2.0f * PI * k * n / N;
      real += v * cosf(angle);
      imag += v * sinf(angle);
    }
    bandEnergy += (real * real + imag * imag) / N;
  }

  // 以經驗值進行簡單映射
  const float REF_ENERGY = 8000.0f; // 可視路況微調
  float norm = bandEnergy / REF_ENERGY;
  if (norm > 1.0f) norm = 1.0f;
  if (norm < 0.0f) norm = 0.0f;
  return static_cast<uint8_t>(1 + norm * 254.0f);
}

static RoadState analyze_road_condition() {
  RoadState state;
  state.potholeFlag = potholeDetected;
  state.lat = potholeLat;
  state.lon = potholeLon;
  state.score = compute_road_score(1000.0f / ACCEL_SAMPLE_INTERVAL_MS);
  // 重置坑洞旗標，待下次偵測
  potholeDetected = false;
  return state;
}

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

  // 6. MPU 初始化 (skip if using mock data to avoid I2C errors)
  if (!USE_MOCK_DATA) {
    mpu_init();
  }
}

void loop() {
  mqtt_loop();

  // 連續收集加速度並即時檢查坑洞
  unsigned long now = millis();
  if ((USE_MOCK_DATA || ENABLE_MPU) && (now - lastAccelSample >= ACCEL_SAMPLE_INTERVAL_MS)) {
    lastAccelSample = now;
    sensors_event_t a, g, tempEvent;
    bool readSuccess = false;
    
    if (USE_MOCK_DATA) {
      readSuccess = mpu_read_accel_mock(&a, &g, &tempEvent);
    } else if (ENABLE_MPU) {
      readSuccess = mpu_read_accel(&a, &g, &tempEvent);
    }
    
    if (readSuccess) {
      float magG = sqrtf(a.acceleration.x * a.acceleration.x +
                        a.acceleration.y * a.acceleration.y +
                        a.acceleration.z * a.acceleration.z) / 9.80665f; // 轉為 g 值
      push_accel_sample(magG);

      // 坑洞偵測：瞬時大加速度
      if (magG >= POTHOLE_TRIGGER_G && (now - lastPotholeMark) > 500) {
        lastPotholeMark = now;
        potholeDetected = true;
        sensors_read_gps(&potholeLat, &potholeLon);
        Serial.printf("[Road] Pothole spike detected (%.2fg) @ GPS: %.6f, %.6f\n", magG, potholeLat, potholeLon);
      }
    }
  }

  // Deep Sleep Check
  if (millis() > RUN_TIME_MS) {
    Serial.println("Time's up! Sleep...");

    // 1. Rail 1 處理: 透過 I2C 指令進入低功耗模式
    if (!USE_MOCK_DATA && ENABLE_MPU) mpu_setup_wom_low_power();
    
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

  // MQTT Publish + 路況分析，每 15 秒
  if (now - lastPublish > SENSOR_PUBLISH_INTERVAL_MS) {
    lastPublish = now;

    SensorData data = sensors_read_all();
    RoadState road = analyze_road_condition();

    char gpsBuf[256];
    char sensorBuf[320];
    char accelFrag[128];
    char roadBuf[256];
    char potholeLatBuf[32];
    char potholeLonBuf[32];

    // 取最後一次加速度向量供觀察
    if (USE_MOCK_DATA) {
      sensors_event_t a, g, tempEvent;
      mpu_read_accel_mock(&a, &g, &tempEvent);
      snprintf(accelFrag, sizeof(accelFrag), "\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f",
        a.acceleration.x, a.acceleration.y, a.acceleration.z);
    } else if (ENABLE_MPU) {
      sensors_event_t a, g, tempEvent;
      mpu_read_accel(&a, &g, &tempEvent);
      snprintf(accelFrag, sizeof(accelFrag), "\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f",
        a.acceleration.x, a.acceleration.y, a.acceleration.z);
    } else {
      strcpy(accelFrag, "\"accel_x\":null,\"accel_y\":null,\"accel_z\":null");
    }

    snprintf(gpsBuf, sizeof(gpsBuf), "{\"latitude\":%.6f,\"longitude\":%.6f}",
      data.latitude, data.longitude);

    if (road.potholeFlag) {
      snprintf(potholeLatBuf, sizeof(potholeLatBuf), "%.6f", road.lat);
      snprintf(potholeLonBuf, sizeof(potholeLonBuf), "%.6f", road.lon);
    } else {
      strcpy(potholeLatBuf, "null");
      strcpy(potholeLonBuf, "null");
    }

    snprintf(sensorBuf, sizeof(sensorBuf),
      "{\"temperature\":%.2f,\"sound_level\":%d,\"air_quality\":%d,\"road_score\":%u,"
      "\"pothole\":%s,\"pothole_lat\":%s,\"pothole_lon\":%s}",
      data.temperature, data.soundLevel, data.airQuality, road.score,
      road.potholeFlag ? "true" : "false",
      potholeLatBuf,
      potholeLonBuf);

    snprintf(roadBuf, sizeof(roadBuf),
      "{\"score\":%u,\"pothole\":%s,\"pothole_lat\":%s,\"pothole_lon\":%s}",
      road.score,
      road.potholeFlag ? "true" : "false",
      potholeLatBuf,
      potholeLonBuf);

    char accelPayload[256];
    snprintf(accelPayload, sizeof(accelPayload), "{%s}", accelFrag);

    bool okGps = mqtt_publish(MQTT_TOPIC_GPS, gpsBuf);
    bool okAccel = mqtt_publish(MQTT_TOPIC_ACCEL, accelPayload);
    bool okSensors = mqtt_publish(MQTT_TOPIC_SENSORS, sensorBuf);

    Serial.printf("Published GPS -> %s : %s (ok=%d)\n", MQTT_TOPIC_GPS, gpsBuf, okGps);
    Serial.printf("Published Accel -> %s : %s (ok=%d)\n", MQTT_TOPIC_ACCEL, accelPayload, okAccel);
    Serial.printf("Published Sensors -> %s : %s (ok=%d)\n", MQTT_TOPIC_SENSORS, sensorBuf, okSensors);
    Serial.printf("[Road] score=%u pothole=%s @ %.6f, %.6f\n",
      road.score, road.potholeFlag ? "true" : "false", road.lat, road.lon);
  }
  
  delay(10);
}
