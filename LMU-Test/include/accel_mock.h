#pragma once
#include <stddef.h>

// Fill `buf` with a JSON fragment (no outer braces) representing accelerometer data.
// Example output: "accel":{"ax":0.01,"ay":-0.02,"az":9.81}
// `buf` will be NUL-terminated.
void accel_get_mock_fragment(char* buf, size_t len);
