#include "sensor_manager.h"
#include "config.h"
#include <Arduino.h>
#include <OneWire.h>

// 感測器物件
static HardwareSerial gpsSerial(2);
static TinyGPS gps;
static OneWire oneWire(tempPin);
static DallasTemperature tempSensor(&oneWire);

void sensors_init() {
    // GPS 初始化
    if (ENABLE_GPS) {
        gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
        Serial.println("[Sensors] GPS initialized");
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
    
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    gps.f_get_position(latitude, longitude);
}

// Read GPS speed in km/h
float sensors_read_gps_speed() {
    if (!ENABLE_GPS) return 0.0;
    
    // Process incoming GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    
    // TinyGPS returns speed in knots, convert to km/h (1 knot ≈ 1.852 km/h)
    return gps.f_speed_kmph();
}

SensorData sensors_read_all() {
    SensorData data;
    
    data.temperature = sensors_read_temperature();
    data.soundLevel = sensors_read_sound();
    data.airQuality = sensors_read_air_quality();
    sensors_read_gps(&data.latitude, &data.longitude);
    
    return data;
}
