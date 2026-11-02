// The GPS receiver (gps.h): UBX over UART1, configured into the RAM layer at every start, since the module keeps no settings without its backup supply.
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "gps.h"

#define PORT UART_NUM_1
#define RX 39
#define TX 17
#define GPS_BAUD 115200
// A fresh M10 talks 38400 (Beitian ships some at 9600 or 115200); 115200 first, for a module that kept its baud across a drone restart.
static const uint32_t bauds[] = {115200, 38400, 9600, 230400};
// How long each baud gets to show a NAV-PVT, and how long without one sends the task back to searching.
#define LISTEN_MS 1500
#define SILENCE_US 3000000

static const UbxSetting configuration[] = {
    {0x40520001, GPS_BAUD},  // CFG-UART1-BAUDRATE
    {0x10740001, 1},         // CFG-UART1OUTPROT-UBX
    {0x10740002, 0},         // CFG-UART1OUTPROT-NMEA
    {0x20910007, 1},         // CFG-MSGOUT-UBX_NAV_PVT_UART1: every solution
    {0x30210001, 100},       // CFG-RATE-MEAS: 100 ms
    {0x20110021, 8},         // CFG-NAVSPG-DYNMODEL: airborne < 4 g
};

static QueueHandle_t events;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static GpsStats stats = {.fix = {.received_us = -1}};

// Waits up to wait_ms for the driver's next event (the line idle for about a byte after a burst, so within a millisecond of a frame's end) and parses what is buffered; returns whether a NAV-PVT came.
static bool receive(UbxParser *parser, uint32_t wait_ms) {
    static uint8_t bytes[256];
    uart_event_t event;
    if (xQueueReceive(events, &event, pdMS_TO_TICKS(wait_ms)) != pdTRUE) return false;
    if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) uart_flush_input(PORT);
    size_t buffered = 0;
    uart_get_buffered_data_len(PORT, &buffered);
    int n = buffered ? uart_read_bytes(PORT, bytes, buffered < sizeof(bytes) ? buffered : sizeof(bytes), 0) : 0;
    if (n <= 0) return false;
    bool solution = false;
    const uint8_t *data = bytes;
    size_t length = n;
    UbxFrame frame;
    while (true) {
        uint32_t skipped = parser->skipped, corrupt = parser->corrupt;
        bool complete = ubx_parse(parser, &data, &length, &frame);
        int64_t now = esp_timer_get_time();
        UbxPvt pvt;
        bool decoded = complete && ubx_pvt(&frame, &pvt);
        portENTER_CRITICAL(&lock);
        stats.skipped += parser->skipped - skipped;
        stats.corrupt += parser->corrupt - corrupt;
        if (decoded) {
            stats.messages++;
            stats.fix.pvt = pvt;
            stats.fix.received_us = now;
        }
        portEXIT_CRITICAL(&lock);
        solution |= decoded;
        if (!complete) break;
    }
    return solution;
}

static void gps_task(void *arg) {
    static UbxParser parser;
    static uint8_t request[96];
    size_t request_length = ubx_valset(configuration, sizeof(configuration) / sizeof(configuration[0]), request, sizeof(request));
    int next = 0;
    while (true) {
        // Configure at a candidate baud (the module switches after acknowledging), then listen at ours.
        uint32_t baud = bauds[next];
        next = (next + 1) % (sizeof(bauds) / sizeof(bauds[0]));
        uart_set_baudrate(PORT, baud);
        uart_flush_input(PORT);
        uart_write_bytes(PORT, request, request_length);
        uart_wait_tx_done(PORT, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(50));
        uart_set_baudrate(PORT, GPS_BAUD);
        uart_flush_input(PORT);
        xQueueReset(events);
        memset(&parser, 0, sizeof(parser));
        int64_t until = esp_timer_get_time() + LISTEN_MS * 1000LL, heard = -1;
        while (esp_timer_get_time() < until)
            if (receive(&parser, 100)) {
                heard = esp_timer_get_time();
                break;
            }
        if (heard < 0) continue;
        portENTER_CRITICAL(&lock);
        stats.configured = true;
        portEXIT_CRITICAL(&lock);
        next = 0;
        while (esp_timer_get_time() - heard < SILENCE_US)
            if (receive(&parser, 100)) heard = esp_timer_get_time();
        portENTER_CRITICAL(&lock);
        stats.configured = false;
        portEXIT_CRITICAL(&lock);
    }
}

void gps_start(void) {
    uart_config_t link = {.baud_rate = GPS_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                          .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(PORT, 1024, 0, 16, &events, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT, &link));
    // GPIO39 has no pull-up; without a module the line floats and the parser skips the noise.
    ESP_ERROR_CHECK(uart_set_pin(PORT, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (xTaskCreatePinnedToCore(gps_task, "gps", 3072, NULL, 3, NULL, 0) != pdPASS) abort();
}

bool gps_fix(GpsFix *fix, uint32_t max_age_us) {
    portENTER_CRITICAL(&lock);
    *fix = stats.fix;
    portEXIT_CRITICAL(&lock);
    return fix->received_us >= 0 && esp_timer_get_time() - fix->received_us <= max_age_us;
}

void gps_stats(GpsStats *out) {
    portENTER_CRITICAL(&lock);
    *out = stats;
    portEXIT_CRITICAL(&lock);
}
