#ifndef MPU_MANAGER_H
#define MPU_MANAGER_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// 初始化 MPU6050
bool mpu_init();

// 設定 MPU6050 為 Wake on Motion 低功耗模式
void mpu_setup_wom_low_power();

// 讀取 MPU6050 加速度數據
bool mpu_read_accel(sensors_event_t* a, sensors_event_t* g, sensors_event_t* temp);

// 獲取 MPU6050 物件指標（用於直接操作）
Adafruit_MPU6050* mpu_get_instance();

#endif // MPU_MANAGER_H
