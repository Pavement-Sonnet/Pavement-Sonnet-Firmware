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

// WiFi Libraries
#include <WiFi.h>
#include <HTTPClient.h>

// BLE Headers
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
const uint64_t SLEEP_TIME_SEC   = 10; // Sleep for 10 seconds   
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds           

#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
#define tempPin 32
#define soundPin 35
#define mq135Pin 34
#define mpuIntPin 14
#define buttonPin 0 // GPIO0 as button input

// WiFi
const char ssid[] = "JustWiFi_IoT";
const char pwd[]  = "66966696";
const char* serverUrl = "http://YOUR_SERVER_IP_OR_URL/api/data"; 

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
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pwd);
    // Non-blocking wait to allow BLE to run first
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
  // Deep Sleep Check
  if (millis() > RUN_TIME_MS) {
    Serial.println("Time's up! Sleep...");
    if (ENABLE_WIFI) WiFi.disconnect(true);
    if (ENABLE_MPU) setupMPU_WOM_LowPower();
    
    
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
    
    Serial.printf("[Sensors] Temp: %.2f | WiFi: %d | BLE: %s | Sound: %d | AirQuality: %d | GPS: %.6f, %.6f\n", 
      temp, WiFi.status() == WL_CONNECTED, deviceConnected ? "CONN" : "WAIT", soundLevel, airQuality, latitude, longitude);
    lastPrint = millis();
  }
  
  
  
  delay(10);
}