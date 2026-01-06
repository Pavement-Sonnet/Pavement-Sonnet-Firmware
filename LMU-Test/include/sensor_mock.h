#pragma once
#include <stddef.h>

// Fill `buf` with a JSON object containing temperature, sound level, and air quality.
// Fields: temp (float, C), sound (int, dB), airquality (int, 0-100)
void sensor_get_mock_json(char* buf, size_t len);
