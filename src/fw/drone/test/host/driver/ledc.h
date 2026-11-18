#pragma once

#include <stdint.h>
#include "esp_err.h"

#define LEDC_HIGH_SPEED_MODE 0
#define LEDC_AUTO_CLK 0
typedef struct { int speed_mode, duty_resolution, timer_num, freq_hz, clk_cfg; } ledc_timer_config_t;
typedef struct { int gpio_num, speed_mode, channel, timer_sel; uint32_t duty; } ledc_channel_config_t;
esp_err_t ledc_timer_config(const ledc_timer_config_t *config);
esp_err_t ledc_channel_config(const ledc_channel_config_t *config);
