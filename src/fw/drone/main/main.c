// The drone: settings from NVS, outputs at zero, the radio, control on core 1, then the camera receiver on core 0. This loop reads the load cell and battery every 50 ms and keeps the recording metadata current.
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "camera.h"
#include "flight_math.h"
#include "runtime.h"
#include "settings.h"

// HX711 channel A, gain 128: DT/DOUT on GPIO32, SCK on GPIO33, VCC to 3V3.
#define LOAD_DOUT 32
#define LOAD_SCK 33
// The dev board's blue LED: blinks at 1 Hz until the controller is heard, then stays on.
#define LED 2
// Battery on GPIO35 (ADC1 channel 7) through a 100k/10k divider.
#define BATTERY ADC_CHANNEL_7
#define DIVIDER ((100000 + 10000) / 10000)
#define PERIOD_MS 50

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE transfer = portMUX_INITIALIZER_UNLOCKED;
static int32_t force;
static int64_t force_us = -1;
static uint32_t force_sequence;
static uint16_t adc_raw;
static float voltage;
static char meta[META];

void sensors_status(Status *status) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&lock);
    bool valid = force_us >= 0;
    status->force = force;
    status->force_age_ms = valid ? (now - force_us) / 1000 : 0;
    status->force_sequence = force_sequence;
    status->adc_raw = adc_raw;
    status->voltage = voltage;
    portEXIT_CRITICAL(&lock);
    status->flags |= valid << 7;
}

// Reads a finished conversion without waiting for one: while DOUT is high the HX711 is still converting.
static void read_load_cell(void) {
    if (gpio_get_level(LOAD_DOUT)) return;
    int32_t value = 0;
    // A clock-high pause over 60 us powers the HX711 down, so the transfer runs with interrupts off.
    portENTER_CRITICAL(&transfer);
    for (int i = 0; i < 24; i++) {
        gpio_set_level(LOAD_SCK, 1);
        esp_rom_delay_us(1);
        int bit = gpio_get_level(LOAD_DOUT);
        gpio_set_level(LOAD_SCK, 0);
        value = value << 1 | bit;
        esp_rom_delay_us(1);
    }
    // The 25th pulse selects channel A, gain 128 for the next sample.
    gpio_set_level(LOAD_SCK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(LOAD_SCK, 0);
    portEXIT_CRITICAL(&transfer);
    if (value & 0x800000) value -= 0x1000000;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&lock);
    force = value;
    force_us = now;
    force_sequence++;
    portEXIT_CRITICAL(&lock);
}

// The recording's description (sw/flight_log.py reads it); the radio resends it after each start and whenever it changes.
static size_t metadata(void) {
    char settings[SETTINGS_JSON_SIZE];
    if (!settings_json(settings, sizeof(settings))) abort();
    size_t at = snprintf(meta, META,
                         "{\"type\":\"metadata\",\"version\":1,\"sample_format\":\"<II7h2h2H2h3hHHHH3hH12h\",\"record_size\":%d,"
                         "\"target_hz\":%d,\"batch_ms\":%d,\"imu_dlpf\":%d,\"pwm_hz\":{\"servo\":%d,\"esc\":%d},"
                         "\"control_core\":1,\"network_core\":0,\"sensor_axis\":[%d,%d,%d],\"sensor_to_body\":[%d,%d,%d],"
                         "\"servo_offsets\":{\"left\":%.9g,\"right\":%.9g},\"settings\":%s,\"thrust_table\":[",
                         RECORD_SIZE, SAMPLE_HZ, 1000 * LOG_BATCH / SAMPLE_HZ, IMU_DLPF, SERVO_HZ, ESC_HZ,
                         sensor_axis[0], sensor_axis[1], sensor_axis[2], sensor_to_body[0], sensor_to_body[1], sensor_to_body[2],
                         settings_get(SETTING_servo_offset_left), settings_get(SETTING_servo_offset_right), settings);
    for (size_t i = 0; i < flight_thrust_count && at < META; i++) {
        at += snprintf(meta + at, META - at, "%s[%d,%.9g]", i ? "," : "", (int)flight_thrust[i][0], flight_thrust[i][1]);
    }
    if (at < META) at += snprintf(meta + at, META - at, "]}");
    if (at >= META) abort();
    return at;
}

void app_main(void) {
    // A full or newer-format NVS is an error to fix by hand: erasing it would lose the calibration and tuning.
    ESP_ERROR_CHECK(nvs_flash_init());
    settings_load();
    flight_runtime_outputs();
    radio_start();
    flight_runtime_start();
    camera_start();

    gpio_config_t led = {.pin_bit_mask = 1ULL << LED, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&led));
    gpio_config_t sck = {.pin_bit_mask = 1ULL << LOAD_SCK, .mode = GPIO_MODE_OUTPUT};
    gpio_config_t dout = {.pin_bit_mask = 1ULL << LOAD_DOUT, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&sck));
    ESP_ERROR_CHECK(gpio_set_level(LOAD_SCK, 0));
    ESP_ERROR_CHECK(gpio_config(&dout));

    adc_oneshot_unit_handle_t adc;
    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
    adc_oneshot_chan_cfg_t channel = {.atten = ADC_ATTEN_DB_6, .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, BATTERY, &channel));
    adc_cali_handle_t calibration;
    adc_cali_line_fitting_config_t fitting = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_6, .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&fitting, &calibration));

    uint32_t described = settings_version() - 1;
    size_t pending = 0;
    TickType_t deadline = xTaskGetTickCount();
    while (true) {
        if (described != settings_version()) {
            described = settings_version();
            pending = metadata();
        }
        // Refused while the previous metadata is still being sent; the same text is offered again.
        if (pending && radio_log_meta(meta, pending)) pending = 0;
        read_load_cell();
        gpio_set_level(LED, radio_connected() || esp_timer_get_time() / 500000 % 2);
        int raw, millivolts;
        ESP_ERROR_CHECK(adc_oneshot_read(adc, BATTERY, &raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calibration, raw, &millivolts));
        flight_runtime_set_voltage(millivolts * DIVIDER);
        portENTER_CRITICAL(&lock);
        // The 12-bit reading on a 16-bit scale.
        adc_raw = raw << 4 | raw >> 8;
        voltage = millivolts * DIVIDER / 1000.0f;
        portEXIT_CRITICAL(&lock);
        vTaskDelayUntil(&deadline, pdMS_TO_TICKS(PERIOD_MS));
    }
}
