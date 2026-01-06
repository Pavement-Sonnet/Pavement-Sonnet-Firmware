#include "mpu_manager.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

static Adafruit_MPU6050 mpu;

// 寫入 MPU 暫存器
static void writeMPURegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// 讀取 MPU 暫存器
static uint8_t readMPURegister(uint8_t reg) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 1);
    if (Wire.available()) return Wire.read();
    return 0;
}

bool mpu_init() {
    if (!ENABLE_MPU) return false;
    
    Wire.begin();
    if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        pinMode(mpuIntPin, INPUT_PULLUP);
        Serial.println("[MPU] Initialized successfully");
        return true;
    } else {
        Serial.println("[MPU] Failed to initialize");
        return false;
    }
}

void mpu_setup_wom_low_power() {
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

bool mpu_read_accel(sensors_event_t* a, sensors_event_t* g, sensors_event_t* temp) {
    if (!ENABLE_MPU) return false;
    mpu.getEvent(a, g, temp);
    return true;
}

Adafruit_MPU6050* mpu_get_instance() {
    return &mpu;
}
