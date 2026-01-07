#pragma once
#include <stddef.h>
#include <Adafruit_Sensor.h>

// Fill `buf` with a JSON fragment (no outer braces) representing accelerometer data.
// Example output: {"accel_x":0.01,"accel_y":-0.02,"accel_z":9.81}
// `buf` will be NUL-terminated.
void accel_get_mock_fragment(char* buf, size_t len);

// Fill `buf` with a JSON object containing temperature, sound level, and air quality.
// Fields: temperature (float, °C), sound_level (int, dB), air_quality (int, 0-100)
void sensor_get_mock_json(char* buf, size_t len);

// Fill `buf` with a JSON-formatted mock GPS sample.
// `buf` will be NUL-terminated. Keep `len` reasonably large (>=128).
// Each call will slightly change the position to simulate movement.
void gps_get_mock_json(char* buf, size_t len);

// Individual sensor mock functions matching real sensor interfaces
float sensors_mock_temperature();
int sensors_mock_sound();
int sensors_mock_air_quality();
void sensors_mock_gps(float* latitude, float* longitude);

// Mock accelerometer function matching mpu_read_accel signature
bool mpu_read_accel_mock(sensors_event_t* a, sensors_event_t* g, sensors_event_t* temp);
