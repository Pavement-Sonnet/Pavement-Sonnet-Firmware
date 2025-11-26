#include <Arduino.h>
#include "mqtt_config.h"
#include "wifi_manager.h"
#include "mqtt_client.h"

static unsigned long lastPublish = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting ESP32 MQTT example");
  wifi_connect();
  mqtt_init();
}

void loop() {
  mqtt_loop();

  unsigned long now = millis();
  if (now - lastPublish > 5000) {
    lastPublish = now;
    char payload[64];
    snprintf(payload, sizeof(payload), "hello %lu", now / 1000);
    bool ok = mqtt_publish(MQTT_TOPIC_PUB, payload);
    Serial.print("Published: "); Serial.print(payload);
    Serial.print(" -> "); Serial.println(ok ? "OK" : "FAILED");
  }
}