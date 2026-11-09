// The barometer (baro.h): a Bosch BMP388 on its own I2C bus, polled from core 0, so the control loop's IMU reads never wait on it and a fault on its wires cannot stop the IMU.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "baro.h"

#define PORT I2C_NUM_1
#define SDA 17
#define SCL 15
static const uint8_t addresses[] = {0x76, 0x77};
#define BMP388_ID 0x50
#define BMP390_ID 0x60
enum { CHIP_ID = 0x00, ERR_REG = 0x02, STATUS = 0x03, PWR_CTRL = 0x1b, OSR = 0x1c, ODR = 0x1d, CONFIG = 0x1f, CALIBRATION = 0x31, CMD = 0x7e };
// Pressure x8 and temperature x1 oversampling (about 19 ms a measurement, inside the 20 ms period), 50 Hz output, the IIR filter at coefficient 3 (register value 2), then normal mode with both measurements on.
#define OSR_VALUE 0x03
#define ODR_VALUE 0x02
#define CONFIG_VALUE (2 << 1)
#define NORMAL 0x33
#define SOFT_RESET 0xb6
// Status bits: a new pressure and a new temperature since the data was last read.
#define READY 0x60
// The task polls every POLL_MS, searches again every SEARCH_MS while there is no chip, and after FAILURES failed reads in a row or QUIET_US without a new measurement (a brownout puts the chip back to sleep, unconfigured). Transfers wait up to two FreeRTOS ticks: at the 100 Hz tick a shorter timeout rounds to none (flight_rt.c read_fifo).
#define POLL_MS 10
#define SEARCH_MS 1000
#define FAILURES 10
#define QUIET_US 200000
#define TIMEOUT pdMS_TO_TICKS(20)
// A reading is kept only inside the chip's range (Pa), and only within JUMP (m) of the last one kept unless JUMPS in a row agree with each other: 2 m in one 20 ms period is 100 m/s, a corrupt read rather than the drone moving.
#define LOWEST 30000.0
#define HIGHEST 125000.0
#define JUMP 2.0
#define JUMPS 5

// The calibration coefficients as the BMP388 datasheet's floating-point compensation takes them.
typedef struct {
    double t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
} Calibration;

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static BaroStats stats = {.sample = {.read_us = -1}};

static esp_err_t write_register(uint8_t address, uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    return i2c_master_write_to_device(PORT, address, data, sizeof(data), TIMEOUT);
}

static esp_err_t read_registers(uint8_t address, uint8_t reg, uint8_t *data, size_t length) {
    return i2c_master_write_read_device(PORT, address, &reg, 1, data, length, TIMEOUT);
}

static void count_error(void) {
    portENTER_CRITICAL(&lock);
    stats.errors++;
    portEXIT_CRITICAL(&lock);
}

// The NVM's 21 bytes from 0x31, little-endian, scaled as the datasheet's floating-point compensation takes them.
static void calibration_from(const uint8_t *n, Calibration *c) {
    uint16_t t1 = n[0] | n[1] << 8, t2 = n[2] | n[3] << 8, p5 = n[11] | n[12] << 8, p6 = n[13] | n[14] << 8;
    int16_t p1 = (int16_t)(n[5] | n[6] << 8), p2 = (int16_t)(n[7] | n[8] << 8), p9 = (int16_t)(n[17] | n[18] << 8);
    c->t1 = t1 * 256.0;
    c->t2 = t2 / 1073741824.0;
    c->t3 = (int8_t)n[4] / 281474976710656.0;
    c->p1 = (p1 - 16384) / 1048576.0;
    c->p2 = (p2 - 16384) / 536870912.0;
    c->p3 = (int8_t)n[9] / 4294967296.0;
    c->p4 = (int8_t)n[10] / 137438953472.0;
    c->p5 = p5 * 8.0;
    c->p6 = p6 / 64.0;
    c->p7 = (int8_t)n[15] / 256.0;
    c->p8 = (int8_t)n[16] / 32768.0;
    c->p9 = p9 / 281474976710656.0;
    c->p10 = (int8_t)n[19] / 281474976710656.0;
    c->p11 = (int8_t)n[20] / 36893488147419103232.0;
}

// The datasheet's floating-point compensation, in double: the temperature (°C) first, which the pressure (Pa) depends on.
static void compensate(const Calibration *c, uint32_t raw_pressure, uint32_t raw_temperature, double *pressure, double *temperature) {
    double d = raw_temperature - c->t1;
    double t = d * c->t2 + d * d * c->t3;
    double p = raw_pressure;
    double out1 = c->p5 + c->p6 * t + c->p7 * t * t + c->p8 * t * t * t;
    double out2 = p * (c->p1 + c->p2 * t + c->p3 * t * t + c->p4 * t * t * t);
    double out3 = p * p * (c->p9 + c->p10 * t) + p * p * p * c->p11;
    *pressure = out1 + out2 + out3;
    *temperature = t;
}

// Finds the chip at either address and configures it; its address, or 0 if none answers or it refuses the configuration. A failed search prints what each address answered on the serial console whenever that changes (no answer: wiring or power; another chip ID: not a BMP388/390, e.g. 0x58 a BMP280, 0x60 also a BME280; refused: the configuration did not take).
static uint8_t search(Calibration *c) {
    static char reported[96];
    char outcome[96] = "";
    size_t length = 0;
    for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
        uint8_t address = addresses[i], id = 0, nvm[21], check[2];
        if (read_registers(address, CHIP_ID, &id, 1) != ESP_OK) {
            length += snprintf(outcome + length, sizeof(outcome) - length, " 0x%02x no answer;", address);
            continue;
        }
        if (id != BMP388_ID && id != BMP390_ID) {
            length += snprintf(outcome + length, sizeof(outcome) - length, " 0x%02x chip ID 0x%02x;", address, id);
            continue;
        }
        length += snprintf(outcome + length, sizeof(outcome) - length, " 0x%02x chip ID 0x%02x refused the configuration;", address, id);
        if (write_register(address, CMD, SOFT_RESET) != ESP_OK) continue;
        vTaskDelay(pdMS_TO_TICKS(20));
        // In sleep after the reset: the oversampling, rate and filter first, then normal mode, which the chip refuses (conf_err) if the measurement does not fit the period.
        if (read_registers(address, CALIBRATION, nvm, sizeof(nvm)) != ESP_OK || write_register(address, OSR, OSR_VALUE) != ESP_OK ||
            write_register(address, ODR, ODR_VALUE) != ESP_OK || write_register(address, CONFIG, CONFIG_VALUE) != ESP_OK ||
            write_register(address, PWR_CTRL, NORMAL) != ESP_OK)
            continue;
        vTaskDelay(pdMS_TO_TICKS(20));
        if (read_registers(address, ERR_REG, &check[0], 1) != ESP_OK || read_registers(address, PWR_CTRL, &check[1], 1) != ESP_OK ||
            (check[0] & 7) || (check[1] & NORMAL) != NORMAL)
            continue;
        calibration_from(nvm, c);
        portENTER_CRITICAL(&lock);
        stats.address = address;
        stats.chip = id;
        portEXIT_CRITICAL(&lock);
        return address;
    }
    portENTER_CRITICAL(&lock);
    stats.address = 0;
    stats.searches++;
    portEXIT_CRITICAL(&lock);
    if (strcmp(outcome, reported)) {
        printf("Barometer not found (SDA %d, SCL %d):%s\n", SDA, SCL, outcome);
        strcpy(reported, outcome);
    }
    return 0;
}

static void baro_task(void *arg) {
    static Calibration calibration;
    while (true) {
        uint8_t address = search(&calibration);
        if (!address) {
            vTaskDelay(pdMS_TO_TICKS(SEARCH_MS));
            continue;
        }
        int failures = 0, jumps = 0;
        double kept = NAN, pending = NAN;
        int64_t heard = esp_timer_get_time();
        TickType_t deadline = xTaskGetTickCount();
        while (failures < FAILURES && esp_timer_get_time() - heard < QUIET_US) {
            vTaskDelayUntil(&deadline, pdMS_TO_TICKS(POLL_MS));
            uint8_t data[7];
            if (read_registers(address, STATUS, data, sizeof(data)) != ESP_OK) {
                failures++;
                count_error();
                continue;
            }
            failures = 0;
            if ((data[0] & READY) != READY) continue;
            int64_t now = heard = esp_timer_get_time();
            double pressure, temperature;
            compensate(&calibration, data[1] | data[2] << 8 | (uint32_t)data[3] << 16, data[4] | data[5] << 8 | (uint32_t)data[6] << 16, &pressure, &temperature);
            double altitude = 44330.0 * (1 - pow(pressure / 101325.0, 1 / 5.255));
            bool ranged = pressure >= LOWEST && pressure <= HIGHEST;
            bool jumped = ranged && isfinite(kept) && fabs(altitude - kept) > JUMP;
            if (jumped) {
                jumps = isfinite(pending) && fabs(altitude - pending) <= JUMP ? jumps + 1 : 1;
                pending = altitude;
                if (jumps >= JUMPS) jumped = false;
            }
            if (!ranged || jumped) {
                portENTER_CRITICAL(&lock);
                stats.rejected++;
                portEXIT_CRITICAL(&lock);
                continue;
            }
            jumps = 0;
            pending = NAN;
            kept = altitude;
            portENTER_CRITICAL(&lock);
            stats.samples++;
            stats.sample = (BaroSample){.read_us = now, .pressure = pressure, .temperature = temperature, .altitude = altitude};
            portEXIT_CRITICAL(&lock);
        }
        portENTER_CRITICAL(&lock);
        stats.address = 0;
        portEXIT_CRITICAL(&lock);
    }
}

void baro_start(void) {
    i2c_config_t bus = {.mode = I2C_MODE_MASTER, .sda_io_num = SDA, .scl_io_num = SCL,
                        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
                        .master.clk_speed = 400000};
    // Not fatal: the drone flies without a barometer.
    if (i2c_param_config(PORT, &bus) == ESP_OK && i2c_driver_install(PORT, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK)
        xTaskCreatePinnedToCore(baro_task, "baro", 4096, NULL, 2, NULL, 0);
}

bool baro_sample(BaroSample *sample, uint32_t max_age_us) {
    portENTER_CRITICAL(&lock);
    *sample = stats.sample;
    portEXIT_CRITICAL(&lock);
    return sample->read_us >= 0 && esp_timer_get_time() - sample->read_us <= max_age_us;
}

void baro_stats(BaroStats *out) {
    portENTER_CRITICAL(&lock);
    *out = stats;
    portEXIT_CRITICAL(&lock);
}
