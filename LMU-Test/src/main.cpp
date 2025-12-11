// Arduino Libraries
#include <Arduino.h>

// Accelerometer, GPS, Temperature Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS.h>
#include <HardwareSerial.h>

// BLE Headers
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Module Headers
#include "mqtt_config.h"
#include "wifi_manager.h"
#include "mqtt_client.h"

// ==========================================
//           功能開關 (Power Flags)
// ==========================================
const bool ENABLE_WIFI    = true;   
const bool ENABLE_POST    = false; 
const bool ENABLE_MPU     = true;   
const bool ENABLE_DS18B20 = true;   
const bool ENABLE_GPS     = true;   
const bool ENABLE_SOUND   = true;   
const bool ENABLE_MQ135   = true;   
const bool ENABLE_BLE     = true;   

// ==========================================
//        電源軌控制設定 (Power Rails)
// ==========================================
// Rail 1: Always On (3.3V) -> MPU6050 (軟體控制睡眠)
// Rail 2: GPIO Direct Drive -> Temp, Sound (低功耗)
// Rail 3: MOSFET/Relay -> GPS (高功耗 3.3V)
// Rail 4: MOSFET/Relay -> MQ135 (高功耗 5V)

const int PIN_RAIL_2 = 25; // Control for Temp & Sound
const int PIN_RAIL_3 = 26; // Control for GPS
const int PIN_RAIL_4 = 27; // Control for MQ135

// 感測器通電後的穩定時間(毫秒)，GPS冷啟動可能需要更久，但至少給硬體一點時間過電
const int POWER_STABILIZE_DELAY = 500; 

// ==========================================
//           BLE 設定 (UUID)
// ==========================================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ==========================================
//           硬體腳位與設定(Sleep Time)
// ==========================================
const unsigned long RUN_TIME_MS = 100000; //Sleep after 100 seconds
const uint64_t SLEEP_TIME_SEC   = 100; // Sleep for 10 seconds   
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds           

#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
#define tempPin 32
#define soundPin 35
#define mq135Pin 34
#define mpuIntPin 14
#define buttonPin 0 // GPIO0 as button input


static unsigned long lastPublish = 0;

// 物件宣告
HardwareSerial gpsSerial(2);
TinyGPS gps;
Adafruit_MPU6050 mpu;
OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

// BLE 回調函數
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("[BLE] Client Connected");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("[BLE] Client Disconnected");
    }
};

// Funtion to write to MPU register
void writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
uint8_t readMPURegister(uint8_t reg) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

// Wake on Motion Register Patch
void setupMPU_WOM_LowPower() {
  writeMPURegister(0x6B, 0x00); delay(10);
  writeMPURegister(0x38, 0x00);
  writeMPURegister(0x1C, 0x01); 
  writeMPURegister(0x1F, 2); 
  writeMPURegister(0x20, 20); 
  writeMPURegister(0x37, 0xA0);
  writeMPURegister(0x38, 0x40);
  writeMPURegister(0x6C, 0x47);
  writeMPURegister(0x6B, 0x20);
  Serial.println("[MPU] Configured for WOM Cycle Mode");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("\n=== ESP32 WiFi & BLE Test ===");

  // ================================================
  // 1. 電源軌初始化 (Power Rails Init) - 最優先執行
  // ================================================
  Serial.println("[Power] Enabling Power Rails...");
  
  // 設定腳位模式
  pinMode(PIN_RAIL_2, OUTPUT);
  pinMode(PIN_RAIL_3, OUTPUT);
  pinMode(PIN_RAIL_4, OUTPUT);

  // 開啟電源
  // Rail 1 是硬體常開，不用程式控制
  digitalWrite(PIN_RAIL_2, HIGH); // Temp, Sound ON
  digitalWrite(PIN_RAIL_3, HIGH); // GPS ON
  digitalWrite(PIN_RAIL_4, HIGH); // MQ135 ON

  // 等待電壓穩定與感測器啟動
  Serial.printf("[Power] Waiting %d ms for sensors to stabilize...\n", POWER_STABILIZE_DELAY);
  delay(POWER_STABILIZE_DELAY); 

  pinMode(buttonPin, INPUT_PULLUP); // Button Pin Setup

  // 1. 初始化 BLE (Init BLE)
  if (ENABLE_BLE) {
    BLEDevice::init("ESP32_Vibe_Trigger");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising started...");
  }

  // 2. WiFi Setup
  if (ENABLE_WIFI) {
    wifi_connect();
    mqtt_init();
  }

  // 3. Sensors Setup
  if (ENABLE_GPS) gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  if (ENABLE_DS18B20) tempSensor.begin();
  if (ENABLE_SOUND) pinMode(soundPin, INPUT);
  if (ENABLE_MQ135) pinMode(mq135Pin, INPUT);

  // 4. MPU Setup
  if (ENABLE_MPU) {
    Wire.begin();
    if (mpu.begin()) {
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      pinMode(mpuIntPin, INPUT_PULLUP); 
    }
  }
}

// Button debounce variables
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void loop() {
  mqtt_loop();

  // Deep Sleep Check
  if (millis() > RUN_TIME_MS) {
    Serial.println("Time's up! Sleep...");

    // 1. Rail 1 處理: 透過 I2C 指令進入低功耗模式
    if (ENABLE_MPU) setupMPU_WOM_LowPower();
    
    // 2. 切斷其他電源軌
    Serial.println("[Power] Cutting power to Rail 2, 3, 4");
    digitalWrite(PIN_RAIL_2, LOW); // 切斷 Temp/Sound
    digitalWrite(PIN_RAIL_3, LOW); // 切斷 GPS
    digitalWrite(PIN_RAIL_4, LOW); // 切斷 MQ135

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
  if (ENABLE_BLE) {
    int reading = digitalRead(buttonPin);
    if (reading != lastButtonState) lastDebounceTime = millis();

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // Detect button press (LOW)
      if (reading == LOW && deviceConnected) {
         Serial.println("[Button] Pressed! Sending VIB notify...");
         pCharacteristic->setValue("VIB");
         pCharacteristic->notify();
         delay(200); // Simple debounce to prevent multiple triggers
      }
    }
    lastButtonState = reading;
    
    // Disconnection and reconnection handling
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); 
        pServer->startAdvertising(); 
        Serial.println("[BLE] Restart advertising");
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
  }
  // ===================================================

  // Data Collection & Print
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    float temp = 0;
    int soundLevel = 0;
    int airQuality = 0;
    float latitude = 0.0, longitude = 0.0;
    if(ENABLE_DS18B20) {
        tempSensor.requestTemperatures();
        temp = tempSensor.getTempCByIndex(0);
    }
    if(ENABLE_MPU) {
      sensors_event_t a, g, tempEvent;
      mpu.getEvent(&a, &g, &tempEvent);
    }
    if(ENABLE_GPS) {
      while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
      }
      gps.f_get_position(&latitude, &longitude);
    }
    if (ENABLE_SOUND)
    {
      soundLevel = analogRead(soundPin);
    }
    if (ENABLE_MQ135)
    {
      airQuality = analogRead(mq135Pin);
    }
    
    Serial.printf("[Sensors] Temp: %.2f | BLE: %s | Sound: %d | AirQuality: %d | GPS: %.6f, %.6f\n", 
      temp, deviceConnected ? "CONN" : "WAIT", soundLevel, airQuality, latitude, longitude);
    lastPrint = millis();
  }

  unsigned long now = millis();
  if (now - lastPublish > 5000) {
    lastPublish = now;
    char gpsBuf[256];
    char accelFrag[128];
    char accelPayload[256];
    char sensorBuf[256];

    // Fetch sensor data
    float temp = 0;
    int soundLevel = 0;
    int airQuality = 0;
    float latitude = 0.0, longitude = 0.0;
    if(ENABLE_DS18B20) {
        tempSensor.requestTemperatures();
        temp = tempSensor.getTempCByIndex(0);
    }
    if(ENABLE_MPU) {
      sensors_event_t a, g, tempEvent;
      mpu.getEvent(&a, &g, &tempEvent);
      snprintf(accelFrag, sizeof(accelFrag), "\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f",
        a.acceleration.x, a.acceleration.y, a.acceleration.z);
    } else {
      strcpy(accelFrag, "\"accel_x\":null,\"accel_y\":null,\"accel_z\":null");
    }
    if(ENABLE_GPS) {
      while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
      }
      gps.f_get_position(&latitude, &longitude);
    }
    if (ENABLE_SOUND)
    {
      soundLevel = analogRead(soundPin);
    }
    if (ENABLE_MQ135)
    {
      airQuality = analogRead(mq135Pin);
    }
    // Prepare MQTT Payloads
    snprintf(gpsBuf, sizeof(gpsBuf), "{\"latitude\":%.6f,\"longitude\":%.6f}", latitude, longitude);
    snprintf(accelPayload, sizeof(accelPayload), "{%s}", accelFrag);
    snprintf(sensorBuf, sizeof(sensorBuf), "{\"temperature\":%.2f,\"sound_level\":%d,\"air_quality\":%d}", 
      temp, soundLevel, airQuality);

    // Publish GPS, accel, and sensors to separate topics
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