#include "sensor_manager.h"
#include "config.h"
#include <Arduino.h>
#include <OneWire.h>

// 感測器物件
static HardwareSerial gpsSerial(2);
static TinyGPS gps;
static OneWire oneWire(tempPin);
static DallasTemperature tempSensor(&oneWire);

// GPS 緩存與狀態
static float lastGpsLat = 0.0f;
static float lastGpsLon = 0.0f;
static bool hasGpsFix = false;
static unsigned long lastGpsLog = 0;
static int lastCharsRead = 0;
static int lastSatCount = 0;
static unsigned long lastFixAge = 0;

void sensors_init() {
    // GPS 初始化
    if (ENABLE_GPS) {
        // 先嘗試 9600，如失敗改試 115200
        gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
        delay(100);
        Serial.printf("[Sensors] GPS initialized at %d baud (RXD2=%d, TXD2=%d)\n", GPS_BAUD, RXD2, TXD2);
    }
    
    // DS18B20 溫度感測器初始化
    if (ENABLE_DS18B20) {
        tempSensor.begin();
        Serial.println("[Sensors] DS18B20 temperature sensor initialized");
    }
    
    // 聲音感測器初始化
    if (ENABLE_SOUND) {
        pinMode(soundPin, INPUT);
        Serial.println("[Sensors] Sound sensor initialized");
    }
    
    // MQ135 空氣品質感測器初始化
    if (ENABLE_MQ135) {
        pinMode(mq135Pin, INPUT);
        Serial.println("[Sensors] MQ135 air quality sensor initialized");
    }
}

float sensors_read_temperature() {
    if (!ENABLE_DS18B20) return 0.0;
    
    tempSensor.requestTemperatures();
    return tempSensor.getTempCByIndex(0);
}

int sensors_read_sound() {
    if (!ENABLE_SOUND) return 0;
    
    return analogRead(soundPin);
}

int sensors_read_air_quality() {
    if (!ENABLE_MQ135) return 0;
    
    return analogRead(mq135Pin);
}

void sensors_read_gps(float* latitude, float* longitude) {
    if (!ENABLE_GPS) {
        *latitude = 0.0;
        *longitude = 0.0;
        return;
    }

    // 讀取一段時間的 NMEA 資料，避免只讀到零星字元
    // 增加到 500ms 以確保獲取完整的 NMEA 句子
    const unsigned long readWindowMs = 500;
    unsigned long start = millis();
    int charsRead = 0;
    char firstChars[32] = {0}; // 儲存前幾個字元用於診斷
    
    while (millis() - start < readWindowMs) {
        while (gpsSerial.available() > 0) {
            uint8_t c = gpsSerial.read();
            gps.encode(c);
            if (charsRead < 31) {
                firstChars[charsRead] = (c >= 32 && c < 127) ? c : '?';
            }
            charsRead++;
        }
    }

    unsigned long fixAge = 0;
    float lat = 0.0f;
    float lon = 0.0f;
    gps.f_get_position(&lat, &lon, &fixAge);

    // 解析衛星數和其他診斷資訊
    int numSats = gps.satellites();
    unsigned long hdop = gps.hdop();

    bool validFix = (fixAge != TinyGPS::GPS_INVALID_AGE) && (fixAge < 2000) && (numSats >= 3);
    if (validFix) {
        lastGpsLat = lat;
        lastGpsLon = lon;
        hasGpsFix = true;
    }

    if (hasGpsFix) {
        *latitude = lastGpsLat;
        *longitude = lastGpsLon;
    } else {
        *latitude = 0.0f;
        *longitude = 0.0f;
    }

    // 限速輸出診斷訊息
    if (millis() - lastGpsLog > 3000) {
        lastGpsLog = millis();
        Serial.printf("[GPS] chars=%d first='%.30s' sats=%d hdop=%lu age=%lu lat=%.6f lon=%.6f fix=%d\n",
                      charsRead, firstChars, numSats, hdop, fixAge, lat, lon, hasGpsFix ? 1 : 0);
        
        // 如果有數據但無衛星，可能需要更多時間鎖定或位置不夠開闊
        if (charsRead > 0 && numSats == 0) {
            Serial.println("[GPS] INFO: Module receiving data but no satellite lock yet. This is normal on cold start. Need 5-15 min outdoors with clear sky.");
        }
    }
    
    // 儲存診斷資訊供 LCD 使用
    lastCharsRead = charsRead;
    lastSatCount = numSats;
    lastFixAge = fixAge;
}

GpsDiagnostics sensors_get_gps_diagnostics() {
    GpsDiagnostics diag;
    diag.satellites = lastSatCount;
    diag.fixAge = lastFixAge;
    diag.latitude = lastGpsLat;
    diag.longitude = lastGpsLon;
    diag.hasFix = hasGpsFix;
    diag.charsRead = lastCharsRead;
    return diag;
}

SensorData sensors_read_all() {
    SensorData data;
    
    data.temperature = sensors_read_temperature();
    data.soundLevel = sensors_read_sound();
    data.airQuality = sensors_read_air_quality();
    sensors_read_gps(&data.latitude, &data.longitude);
    
    return data;
}
