#include "sensor_mock.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

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
