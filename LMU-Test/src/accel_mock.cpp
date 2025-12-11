#include "accel_mock.h"
#include <stdio.h>
#include <math.h>

void accel_get_mock_fragment(char* buf, size_t len) {
  static float t = 0.0f;
  // simple oscillating values to simulate motion
  float ax = 0.02f * sinf(t * 1.0f);
  float ay = 0.03f * sinf(t * 1.5f + 0.5f);
  float az = 9.81f + 0.05f * sinf(t * 0.7f + 1.0f);
  t += 0.2f;
  // produce fragment without outer braces
  // ensure buffer length is sufficient (we expect < 128)
  snprintf(buf, len, "\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f", ax, ay, az);
}
