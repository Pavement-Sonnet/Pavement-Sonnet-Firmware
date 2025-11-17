#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
const int tempPin = 32;
const int soundPin = 35;

HardwareSerial gpsSerial(2);

Adafruit_MPU6050 mpu;

OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

// WiFi Configuration
const char ssid[]="Justin_[object Object]";
const char pwd[]="66966696";

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); //設置WiFi模式
  WiFi.begin(ssid,pwd);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  // Activate the DS18B20 sensor
  tempSensor.begin();
  
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
  tempSensor.requestTemperatures();
  int tempValue = tempSensor.getTempCByIndex(0);
  int soundValue = analogRead(soundPin);

  
  Serial.printf("%.3f %.3f %.3f %.3f %.3f %.3f %d %d\n", 
                a.acceleration.x, 
                a.acceleration.y, 
                a.acceleration.z,
                g.gyro.x,
                g.gyro.y,
                g.gyro.z,
                tempValue,
                soundValue
              );

  while (gpsSerial.available() > 0){
    // get the byte data from the GPS
    char gpsData = gpsSerial.read();
    Serial.print(gpsData);
  }
  
  delay(100);
}

