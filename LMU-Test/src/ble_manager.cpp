#include "ble_manager.h"
#include "config.h"
#include <Arduino.h>

// BLE 全域變數
static BLEServer* pServer = NULL;
static BLECharacteristic* pCharacteristic = NULL;
static bool deviceConnected = false;
static bool oldDeviceConnected = false;

// 按鈕防彈跳變數
static int lastButtonState = HIGH;
static unsigned long lastDebounceTime = 0;
static const unsigned long debounceDelay = 50;

// BLE 回調函數
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("[BLE] Client Connected");
    }
    
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("[BLE] Client Disconnected");
    }
};

void ble_init() {
    if (!ENABLE_BLE) return;
    
    BLEDevice::init("ESP32_Vibe_Trigger");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  
    BLEDevice::startAdvertising();
    
    Serial.println("[BLE] Advertising started...");
}

void ble_handle_button() {
    if (!ENABLE_BLE) return;
    
    int reading = digitalRead(buttonPin);
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        // 偵測按鈕按下 (LOW)
        if (reading == LOW && deviceConnected) {
            Serial.println("[Button] Pressed! Sending VIB notify...");
            pCharacteristic->setValue("VIB");
            pCharacteristic->notify();
            delay(200); // 簡單的防彈跳延遲
        }
    }
    lastButtonState = reading;
    
    // 斷線與重連處理
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); 
        pServer->startAdvertising(); 
        Serial.println("[BLE] Restart advertising");
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}

bool ble_is_connected() {
    return deviceConnected;
}

void ble_notify(const char* message) {
    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue(message);
        pCharacteristic->notify();
    }
}

const char* ble_get_status_string() {
    return deviceConnected ? "CONN" : "WAIT";
}
