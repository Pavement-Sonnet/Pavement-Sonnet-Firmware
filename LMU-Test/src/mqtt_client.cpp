#include "mqtt_client.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "mqtt_config.h"

static WiFiClient espClient;
static PubSubClient mqttClient(espClient);
static unsigned long lastReconnectAttempt = 0;

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT msg received ["); Serial.print(topic); Serial.print("] ");
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println(msg);
}

void mqtt_init() {
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
  mqttClient.setCallback(mqttCallback);
}

static bool mqtt_reconnect() {
  Serial.print("Attempting MQTT connection...");
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
    Serial.println("connected");
    mqttClient.subscribe(MQTT_TOPIC_SUB);
    return true;
  } else {
    Serial.print("failed, rc="); Serial.print(mqttClient.state());
    Serial.println(" will retry in 5 seconds");
    return false;
  }
}

void mqtt_loop() {
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (mqtt_reconnect()) lastReconnectAttempt = 0;
    }
  } else {
    mqttClient.loop();
  }
}

bool mqtt_publish(const char* topic, const char* payload) {
  if (!mqttClient.connected()) return false;
  return mqttClient.publish(topic, payload);
}
