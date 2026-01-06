// MQTT and WiFi configuration - update these values for your network
#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

// Fill these fields
#define WIFI_SSID ""
#define WIFI_PASS ""

// Broker can be your PC running Mosquitto (use PC IP) or a remote broker
#define MQTT_BROKER_IP "test.mosquitto.org"
#define MQTT_BROKER_PORT 1883

// Client ID and optional credentials (leave empty strings for no auth)
#define MQTT_CLIENT_ID "esp32_client"
#define MQTT_USER ""
#define MQTT_PASS ""

// Topics
#define MQTT_TOPIC_SUB "esp32/server/in"
#define MQTT_TOPIC_GPS "esp32/device2/sensors/gps"
#define MQTT_TOPIC_ACCEL "esp32/device2/sensors/accel"
#define MQTT_TOPIC_SENSORS "esp32/device2/sensors/data"

#endif // MQTT_CONFIG_H
