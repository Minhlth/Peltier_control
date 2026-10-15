#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool system_enable;         // Cờ cho phép hệ thống hoạt động
    float temp_on_c;            // Ngưỡng nhiệt độ bật (mặc định 25.0 C)
    float temp_off_c;           // Ngưỡng nhiệt độ tắt (mặc định 20.0 C)
    float hum_on_rh;            // Ngưỡng độ ẩm bật (mặc định 65.0 %)
    float hum_off_rh;           // Ngưỡng độ ẩm tắt (mặc định 60.0 %)
    uint32_t min_on_off_sec;    // Thời gian giữ trạng thái tối thiểu (10 - 30s)
    float ntc_limit_c;          // Ngưỡng cắt quá nhiệt NTC (mặc định 80.0 C)
    float ntc_recover_c;        // Ngưỡng phục hồi an toàn NTC (mặc định 65.0 C)
} app_config_t;

esp_err_t app_config_init(app_config_t *cfg);
esp_err_t app_config_save(const app_config_t *cfg);

#endif // APP_CONFIG_H