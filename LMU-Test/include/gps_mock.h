#pragma once
#include <stddef.h>

// Fill `buf` with a JSON-formatted mock GPS sample.
// `buf` will be NUL-terminated. Keep `len` reasonably large (>=128).
// Each call will slightly change the position to simulate movement.
void gps_get_mock_json(char* buf, size_t len);
