// Thrust stand: one ESC on GPIO25 and up to two HX711 load cells (A: DT GPIO32, SCK GPIO33; B: DT GPIO39, SCK GPIO17), driven from the laptop by sw/stand.py over USB at 921600 baud. The ESC pulse is the drone's (fw/drone/main/flight_rt.c esc_us): 400 Hz, 1000 us plus the throttle (1-1200), and 950 us for throttle 0 (stopped, below the calibrated bottom so the ESC arms and stays off), so a throttle here is a throttle there.
// Laptop to stand, one command per line: "T <throttle>" sets the throttle, "P <us>" sets a raw pulse for the ESC's range calibration (1000-2200 us), "C <mask>" chooses the cells read (bit 0 A, bit 1 B; A alone at start, since an unwired B on GPIO39, which has no pull-up, floats). T and P are held for WATCHDOG_US; without a fresh one the pulse falls back to 950 us (motor stopped).
// Stand to laptop, one line each: "R <reason>" once at boot, why the board last reset (esp_reset_reason: 1 power-on, 3 software, 4 panic, 9 brownout, ...), "F <cell> <us> <raw>" a load cell's reading and when its conversion finished (this board's clock, us), "E <us> <pulse>" the ESC pulse from that moment on, "H <us>" a heartbeat every 100 ms.
// The HX711 converts at 10 Hz unless its RATE pin is high (80 Hz); step timing wants 80 Hz.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define BAUD 921600
#define ESC_PIN 25
#define ESC_HZ 400
#define PULSE_STOP 950
#define PULSE_MIN 1000
#define PULSE_MAX 2200
#define THROTTLE_MAX 1200
#define WATCHDOG_US 300000
#define CELLS 2
#define LED GPIO_NUM_2

static const int dout_pins[CELLS] = {32, 39}, sck_pins[CELLS] = {33, 17};
static portMUX_TYPE transfer = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE command = portMUX_INITIALIZER_UNLOCKED;
static int64_t commanded_us;
static volatile int cells = 1;
static SemaphoreHandle_t esc;

static void write_line(const char *line) {
    uart_write_bytes(UART_NUM_0, line, strlen(line));
}

// The ESC pulse width (us) at 400 Hz: a 2500 us period in 14-bit duty.
static void set_pulse(int pulse) {
    static int current = -1;
    if (pulse < PULSE_STOP) pulse = PULSE_STOP;
    if (pulse > PULSE_MAX) pulse = PULSE_MAX;
    xSemaphoreTake(esc, portMAX_DELAY);
    if (pulse == current) {
        xSemaphoreGive(esc);
        return;
    }
    current = pulse;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)pulse * ESC_HZ * (1 << 14) / 1000000);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    char line[48];
    snprintf(line, sizeof(line), "E %lld %d\n", (long long)esp_timer_get_time(), pulse);
    write_line(line);
    xSemaphoreGive(esc);
}

// A finished conversion, if the cell has one (DOUT low): 24 bits, then a 25th pulse for channel A at gain 128. A clock-high pause over 60 us powers the HX711 down, so the transfer runs with interrupts off.
static bool read_cell(int cell, int32_t *value) {
    if (gpio_get_level(dout_pins[cell])) return false;
    int32_t raw = 0;
    portENTER_CRITICAL(&transfer);
    for (int i = 0; i < 24; i++) {
        gpio_set_level(sck_pins[cell], 1);
        esp_rom_delay_us(1);
        raw = raw << 1 | gpio_get_level(dout_pins[cell]);
        gpio_set_level(sck_pins[cell], 0);
        esp_rom_delay_us(1);
    }
    gpio_set_level(sck_pins[cell], 1);
    esp_rom_delay_us(1);
    gpio_set_level(sck_pins[cell], 0);
    portEXIT_CRITICAL(&transfer);
    *value = raw & 0x800000 ? raw - 0x1000000 : raw;
    return true;
}

static void mark_command(void) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&command);
    commanded_us = now;
    portEXIT_CRITICAL(&command);
}

// Every millisecond: each chosen cell's finished conversion, the watchdog, the heartbeat.
static void sample_task(void *arg) {
    int64_t heartbeat_us = 0;
    while (true) {
        for (int cell = 0; cell < CELLS; cell++) {
            int32_t raw;
            int64_t now = esp_timer_get_time();
            if (cells >> cell & 1 && read_cell(cell, &raw)) {
                char line[48];
                snprintf(line, sizeof(line), "F %d %lld %ld\n", cell, (long long)now, (long)raw);
                write_line(line);
            }
        }
        int64_t now = esp_timer_get_time();
        // The command time is 64-bit and written on the other core: read under the lock so it can't tear.
        portENTER_CRITICAL(&command);
        bool fresh = now - commanded_us <= WATCHDOG_US;
        portEXIT_CRITICAL(&command);
        if (!fresh) set_pulse(PULSE_STOP);
        if (now - heartbeat_us >= 100000) {
            heartbeat_us = now;
            char line[32];
            snprintf(line, sizeof(line), "H %lld\n", (long long)now);
            write_line(line);
            gpio_set_level(LED, fresh);
        }
        vTaskDelay(1);
    }
}

void app_main(void) {
    uart_config_t uart = {.baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart));

    char reason[16];
    snprintf(reason, sizeof(reason), "R %d\n", (int)esp_reset_reason());
    write_line(reason);
    esc = xSemaphoreCreateMutex();
    ledc_timer_config_t timer = {.speed_mode = LEDC_HIGH_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT, .timer_num = LEDC_TIMER_0, .freq_hz = ESC_HZ, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t channel = {.gpio_num = ESC_PIN, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    set_pulse(PULSE_STOP);

    gpio_config_t led = {.pin_bit_mask = 1ULL << LED, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&led));
    for (int cell = 0; cell < CELLS; cell++) {
        gpio_config_t sck = {.pin_bit_mask = 1ULL << sck_pins[cell], .mode = GPIO_MODE_OUTPUT};
        ESP_ERROR_CHECK(gpio_config(&sck));
        gpio_set_level(sck_pins[cell], 0);
        gpio_config_t dout = {.pin_bit_mask = 1ULL << dout_pins[cell], .mode = GPIO_MODE_INPUT, .pull_up_en = dout_pins[cell] < 34};
        ESP_ERROR_CHECK(gpio_config(&dout));
    }
    commanded_us = -WATCHDOG_US;
    xTaskCreatePinnedToCore(sample_task, "sample", 4096, NULL, 10, NULL, 1);

    char line[64];
    size_t length = 0;
    while (true) {
        uint8_t byte;
        if (uart_read_bytes(UART_NUM_0, &byte, 1, portMAX_DELAY) != 1) continue;
        if (byte != '\n' && length < sizeof(line) - 1) {
            line[length++] = byte;
            continue;
        }
        line[length] = 0;
        length = 0;
        int value;
        if (sscanf(line, "T %d", &value) == 1 && value >= 0 && value <= THROTTLE_MAX) {
            mark_command();
            set_pulse(value ? PULSE_MIN + value : PULSE_STOP);
        } else if (sscanf(line, "P %d", &value) == 1 && value >= PULSE_MIN && value <= PULSE_MAX) {
            mark_command();
            set_pulse(value);
        } else if (sscanf(line, "C %d", &value) == 1 && value >= 0 && value < 1 << CELLS) {
            cells = value;
        }
    }
}
