#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE 初始化
void ble_init();

// BLE 處理按鈕觸發邏輯
void ble_handle_button();

// 檢查 BLE 是否已連接
bool ble_is_connected();

// 發送 BLE 通知
void ble_notify(const char* message);

// 獲取連接狀態字串（用於調試輸出）
const char* ble_get_status_string();

#endif // BLE_MANAGER_H
