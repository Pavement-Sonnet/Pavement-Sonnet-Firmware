#include <Arduino.h>
#include "mqtt_config.h"
#include "wifi_manager.h"
#include "mqtt_client.h"
#include "gps_mock.h"
#include "accel_mock.h"
#include "sensor_mock.h"

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
  if (now - lastPublish > 1000) {
    lastPublish = now;
    char gpsBuf[256];
    char accelBuf[256];
    char sensorBuf[256];

    // Fetch mock data
    gps_get_mock_json(gpsBuf, sizeof(gpsBuf));
    accel_get_mock_fragment(accelBuf, sizeof(accelBuf));
    sensor_get_mock_json(sensorBuf, sizeof(sensorBuf));

    // Prepare accel payload as its own JSON object with timestamp
    // unsigned long ts = (unsigned long)time(NULL);
    // snprintf(accelPayload, sizeof(accelPayload), "{\"ts\":%lu,%s}", ts, accelFrag);

    // Publish GPS, accel, and sensors to separate topics
    bool ok1 = mqtt_publish(MQTT_TOPIC_GPS, gpsBuf);
    bool ok2 = mqtt_publish(MQTT_TOPIC_ACCEL, accelBuf);
    bool ok3 = mqtt_publish(MQTT_TOPIC_SENSORS, sensorBuf);

    Serial.print("Published GPS -> "); Serial.print(MQTT_TOPIC_GPS); Serial.print(" : "); Serial.println(gpsBuf);
    Serial.print("Publish result: "); Serial.println(ok1 ? "OK" : "FAILED");
    Serial.print("Published Accel -> "); Serial.print(MQTT_TOPIC_ACCEL); Serial.print(" : "); Serial.println(accelBuf);
    Serial.print("Publish result: "); Serial.println(ok2 ? "OK" : "FAILED");
    Serial.print("Published Sensors -> "); Serial.print(MQTT_TOPIC_SENSORS); Serial.print(" : "); Serial.println(sensorBuf);
    Serial.print("Publish result: "); Serial.println(ok3 ? "OK" : "FAILED");
  }
}