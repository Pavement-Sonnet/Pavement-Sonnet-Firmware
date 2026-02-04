#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 功能開關
const bool ENABLE_WIFI     = true; 
const bool ENABLE_POST     = false;
const bool ENABLE_MPU      = true; 
const bool ENABLE_DS18B20  = true; 
const bool ENABLE_GPS      = true; 
const bool ENABLE_SOUND    = true; 
const bool ENABLE_MQ135    = true; 
const bool ENABLE_BLE      = true; 

// 電源軌腳位
const int PIN_RAIL_2 = 25; 
const int PIN_RAIL_3 = 26; 
const int PIN_RAIL_4 = 27; 
const int POWER_STABILIZE_DELAY = 500;

// 感測器腳位
#define RXD2 17
#define TXD2 16
#define GPS_BAUD 9600
#define tempPin 32
#define soundPin 35
#define mq135Pin 34
#define mpuIntPin 14
#define buttonPin 0

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// 時間參數
const unsigned long RUN_TIME_MS = 100000;
const uint64_t SLEEP_TIME_SEC   = 100;
#define uS_TO_S_FACTOR 1000000ULL

#endif