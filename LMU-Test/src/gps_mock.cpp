#include "gps_mock.h"
#include <stdio.h>
#include <time.h>

void gps_get_mock_json(char* buf, size_t len) {
  static double lat = 37.4219999; // starting lat
  static double lon = -122.0840575; // starting lon
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
  // Example: {"ts":169...,"lat":37.42,"lon":-122.08,"alt":10.0,"hdop":0.9,"sats":8}
  snprintf(buf, len,
           "{\"ts\":%lu,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.2f,\"hdop\":%.2f,\"sats\":%d}",
           ts, lat, lon, alt, 0.9 + ((count%3)*0.1), 6 + (count%5));
}
