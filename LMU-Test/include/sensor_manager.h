#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <TinyGPS.h>
#include <HardwareSerial.h>
#include <DallasTemperature.h>

// 感測器數據結構
struct SensorData {
    float temperature;
    int soundLevel;
    int airQuality;
    float latitude;
    float longitude;
};

// 初始化所有感測器
void sensors_init();

// 讀取所有感測器數據
SensorData sensors_read_all();

// 讀取溫度
float sensors_read_temperature();

// 讀取聲音等級
int sensors_read_sound();

// 讀取空氣品質
int sensors_read_air_quality();

// 讀取 GPS 數據
void sensors_read_gps(float* latitude, float* longitude);

// 讀取 GPS 速度 (km/h)
float sensors_read_gps_speed();

#endif // SENSOR_MANAGER_H
