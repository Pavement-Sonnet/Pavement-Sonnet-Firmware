#include "mock_data.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

// ========== Accelerometer Mock ==========
void accel_get_mock_fragment(char* buf, size_t len) {
  static float t = 0.0f;
  // simple oscillating values to simulate motion
  float ax = 0.02f * sinf(t * 1.0f);
  float ay = 0.03f * sinf(t * 1.5f + 0.5f);
  float az = 0.04f + 0.05f * sinf(t * 0.7f + 1.0f);
  t += 0.2f;
  // produce fragment without outer braces
  // ensure buffer length is sufficient (we expect < 128)
  snprintf(buf, len, "{\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f}", ax, ay, az);
}

// ========== Sensor Mock ==========
void sensor_get_mock_json(char* buf, size_t len) {
  static float t = 0.0f;
  static int count = 0;

  // Simulate temperature oscillating around 22°C
  float temp = 22.0f + 3.0f * sinf(t * 0.5f);

  // Simulate sound level varying between 45-75 dB
  int sound = 60 + (int)(15.0f * sinf(t * 0.3f));

  // Simulate air quality (0-100, higher is worse)
  int air_quality = 40 + (int)(20.0f * sinf(t * 0.4f + 1.5f));

  t += 0.1f;
  count++;

  unsigned long ts = (unsigned long)time(NULL);

  snprintf(buf, len,
           "{\"temperature\":%.2f,\"sound_level\":%d,\"air_quality\":%d}", 
           temp, sound, air_quality);
}

// ========== GPS Mock ==========
void gps_get_mock_json(char* buf, size_t len) {
  static double lat = 48.262798; // starting lat
  static double lon = 11.668381; // starting lon
  static double alt = 10.0; // meters
  static int count = 0;

  // Slightly move the position each call
  lat += 0.00001 * (count % 5);
  lon += 0.00001 * ((count+2) % 5);
  alt += 0.01 * ((count+3) % 3);
  count++;

  // Timestamp (simple epoch seconds)
  time_t t = time(NULL);
  unsigned long ts = (unsigned long)t;

  // JSON payload
  snprintf(buf, len,"{\"latitude\":%.6f,\"longitude\":%.7f}", lat, lon);
}

// ========== Individual Sensor Mock Functions ==========
float sensors_mock_temperature() {
  static float t = 0.0f;
  // Simulate temperature oscillating around 22°C
  float temp = 22.0f + 3.0f * sinf(t * 0.5f);
  t += 0.1f;
  return temp;
}

int sensors_mock_sound() {
  static float t = 0.0f;
  // Simulate sound level varying between 45-75 dB
  int sound = 60 + (int)(15.0f * sinf(t * 0.3f));
  t += 0.1f;
  return sound;
}

int sensors_mock_air_quality() {
  static float t = 0.0f;
  // Simulate air quality (0-100, higher is worse)
  int air_quality = 40 + (int)(20.0f * sinf(t * 0.4f + 1.5f));
  t += 0.1f;
  return air_quality;
}

void sensors_mock_gps(float* latitude, float* longitude) {
  static double lat = 48.262798; // starting lat
  static double lon = 11.668381; // starting lon
  static int count = 0;

  // Slightly move the position each call
  lat += 0.00001 * (count % 5);
  lon += 0.00001 * ((count+2) % 5);
  count++;

  *latitude = (float)lat;
  *longitude = (float)lon;
}

// ========== Mock Accelerometer Function ==========
bool mpu_read_accel_mock(sensors_event_t* a, sensors_event_t* g, sensors_event_t* temp) {
  static float t = 0.0f;
  
  // Generate oscillating mock accelerometer values
  float ax = 0.02f * sinf(t * 1.0f);
  float ay = 0.03f * sinf(t * 1.5f + 0.5f);
  float az = 9.81f + 0.05f * sinf(t * 0.7f + 1.0f); // Include gravity
  t += 0.2f;
  
  // Fill accelerometer event
  if (a) {
    a->acceleration.x = ax;
    a->acceleration.y = ay;
    a->acceleration.z = az;
    a->type = SENSOR_TYPE_ACCELEROMETER;
  }
  
  // Fill gyroscope event (zeros for simplicity)
  if (g) {
    g->gyro.x = 0.0f;
    g->gyro.y = 0.0f;
    g->gyro.z = 0.0f;
    g->type = SENSOR_TYPE_GYROSCOPE;
  }
  
  // Fill temperature event
  if (temp) {
    temp->temperature = 22.0f + 3.0f * sinf(t * 0.5f);
    temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  }
  
  return true;
}
