#include "lcd_manager.h"
#include "config.h"
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// LCD 物件 (I2C 地址 0x27, 16x2 顯示器)
static LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
static bool lcdInitialized = false;

// I2C 掃描函數
static void scanI2C() {
    Serial.println("[LCD] Scanning I2C bus...");
    byte count = 0;
    
    for (byte i = 8; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[LCD] Found I2C device at address 0x%02X\n", i);
            count++;
        }
    }
    
    if (count == 0) {
        Serial.println("[LCD] ERROR: No I2C devices found! Check wiring.");
    } else {
        Serial.printf("[LCD] Found %d device(s)\n", count);
    }
}

void lcd_init() {
    if (!ENABLE_LCD) return;
    
    // 使用自訂 I2C 腳位初始化
    Serial.printf("[LCD] Initializing I2C on SDA=%d, SCL=%d\n", LCD_SDA, LCD_SCL);
    Wire.begin(LCD_SDA, LCD_SCL);
    delay(100);
    
    // 掃描 I2C 匯流排
    scanI2C();
    
    // 初始化 LCD
    Serial.printf("[LCD] Attempting to initialize LCD at address 0x%02X\n", LCD_I2C_ADDR);
    lcd.init();
    delay(100);
    lcd.backlight();
    delay(100);
    lcd.clear();
    delay(100);
    
    // 顯示啟動訊息
    lcd.setCursor(0, 0);
    lcd.print("GPS Tracker");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");
    
    lcdInitialized = true;
    Serial.println("[LCD] Display initialized successfully");
    delay(2000);
    lcd.clear();
}

void lcd_update_gps(int sats, unsigned long age, float lat, float lon, bool hasFix) {
    if (!ENABLE_LCD || !lcdInitialized) return;
    
    lcd.clear();
    
    // 第一行：衛星數量和鎖定狀態
    lcd.setCursor(0, 0);
    if (hasFix) {
        lcd.print("GPS:LOCK Sat:");
        lcd.print(sats);
    } else if (sats == 0 || sats == 255) {
        lcd.print("GPS:Searching..");
    } else {
        lcd.print("GPS:Wait Sat:");
        lcd.print(sats);
    }
    
    // 第二行：座標或等待訊息
    lcd.setCursor(0, 1);
    if (hasFix) {
        // 顯示座標（簡化格式）
        char coordBuf[17];
        snprintf(coordBuf, sizeof(coordBuf), "%.4f,%.4f", lat, lon);
        lcd.print(coordBuf);
    } else {
        lcd.print("No Fix Yet");
    }
}

void lcd_clear() {
    if (!ENABLE_LCD || !lcdInitialized) return;
    lcd.clear();
}

void lcd_print(int row, const char* message) {
    if (!ENABLE_LCD || !lcdInitialized) return;
    if (row >= LCD_ROWS) return;
    
    lcd.setCursor(0, row);
    lcd.print(message);
}
