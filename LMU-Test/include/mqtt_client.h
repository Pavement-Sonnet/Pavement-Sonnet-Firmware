#pragma once

// Initialize MQTT, handle loop and publish helper
void mqtt_init();
void mqtt_loop();
bool mqtt_publish(const char* topic, const char* payload);
