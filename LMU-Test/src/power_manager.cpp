#include "power_manager.h"
#include "config.h"
#include <Arduino.h>

void power_init() {
    Serial.println("[Power] Enabling Power Rails...");
    
    // 設定腳位模式
    pinMode(PIN_RAIL_2, OUTPUT);
    pinMode(PIN_RAIL_3, OUTPUT);
    pinMode(PIN_RAIL_4, OUTPUT);

    // 開啟電源
    // Rail 1 是硬體常開，不用程式控制
    digitalWrite(PIN_RAIL_2, HIGH); // Temp, Sound ON
    digitalWrite(PIN_RAIL_3, HIGH); // GPS ON
    digitalWrite(PIN_RAIL_4, HIGH); // MQ135 ON

    // 等待電壓穩定與感測器啟動
    Serial.printf("[Power] Waiting %d ms for sensors to stabilize...\n", POWER_STABILIZE_DELAY);
    delay(POWER_STABILIZE_DELAY);
}

void power_shutdown_rails() {
    Serial.println("[Power] Cutting power to Rail 2, 3, 4");
    digitalWrite(PIN_RAIL_2, LOW); // 切斷 Temp/Sound
    digitalWrite(PIN_RAIL_3, LOW); // 切斷 GPS
    digitalWrite(PIN_RAIL_4, LOW); // 切斷 MQ135
}
