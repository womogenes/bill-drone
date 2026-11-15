#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define I2C_NUM_0 0
#define I2C_MODE_MASTER 1
#define GPIO_PULLUP_ENABLE 1
typedef struct {
    int mode, sda_io_num, scl_io_num, sda_pullup_en, scl_pullup_en;
    struct { uint32_t clk_speed; } master;
} i2c_config_t;
esp_err_t i2c_param_config(int port, const i2c_config_t *config);
esp_err_t i2c_driver_install(int port, int mode, size_t rx, size_t tx, int flags);
esp_err_t i2c_driver_delete(int port);
esp_err_t i2c_master_write_to_device(int port, uint8_t address, const uint8_t *data, size_t length, TickType_t wait);
esp_err_t i2c_master_write_read_device(int port, uint8_t address, const uint8_t *write, size_t write_length, uint8_t *read, size_t read_length, TickType_t wait);
