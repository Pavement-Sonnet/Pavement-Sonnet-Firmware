#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// put function declarations here:


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  //Serial.println("MPU6050 test!");

  Wire.begin();
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //Serial.println("MPU6050 Found!");

  // Set accelerometer range to +/-2G
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  //Serial.print("加速度計範圍設定為: ");
  // switch (mpu.getAccelerometerRange()) {
  //   case MPU6050_RANGE_2_G:   Serial.println("+-2G");   break;
  //   case MPU6050_RANGE_4_G:   Serial.println("+-4G");   break;
  //   case MPU6050_RANGE_8_G:   Serial.println("+-8G");   break;
  //   case MPU6050_RANGE_16_G:  Serial.println("+-16G");  break;
  // }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Serial.print("陀螺儀範圍設定為: ");
  // switch (mpu.getGyroRange()) {
  //   case MPU6050_RANGE_250_DEG:   Serial.println("+-250 deg/s");   break;
  //   case MPU6050_RANGE_500_DEG:   Serial.println("+-500 deg/s");   break;
  //   case MPU6050_RANGE_1000_DEG:  Serial.println("+-1000 deg/s");  break;
  //   case MPU6050_RANGE_2000_DEG:  Serial.println("+-2000 deg/s");  break;
  // }
  // Serial.println("---------------------------------");

  Serial.println(); Serial.println("X\tY\tZ");
  delay(100);


}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  
  Serial.printf("%.3f %.3f %.3f %.3f %.3f %.3f\n", 
                a.acceleration.x, 
                a.acceleration.y, 
                a.acceleration.z,
                g.gyro.x,
                g.gyro.y,
                g.gyro.z);
  
  delay(100);
}

