#ifndef LCD_MANAGER_H
#define LCD_MANAGER_H

// 初始化 LCD 顯示器
void lcd_init();

// 更新 LCD 顯示 GPS 資訊
void lcd_update_gps(int sats, unsigned long age, float lat, float lon, bool hasFix);

// 清除 LCD 顯示
void lcd_clear();

// 顯示自訂訊息
void lcd_print(int row, const char* message);

#endif // LCD_MANAGER_H
