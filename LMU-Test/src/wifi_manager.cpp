#include "wifi_manager.h"
#include <WiFi.h>
#include "mqtt_config.h"

void wifi_connect() {
  Serial.print("Connecting to WiFi '");
  Serial.print(WIFI_SSID);
  Serial.println("'...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    attempts++;
    if (attempts % 80 == 0) Serial.println();
  }

  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());
}
