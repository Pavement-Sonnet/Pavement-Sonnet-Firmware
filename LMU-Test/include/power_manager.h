#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

// 初始化所有電源軌
void power_init();

// 關閉所有可控電源軌（用於深度睡眠前）
void power_shutdown_rails();

#endif // POWER_MANAGER_H
