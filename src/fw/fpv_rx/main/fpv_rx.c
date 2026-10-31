// FPV receiver: hears the camera's raw 802.11 video (action frames) (fw/esp32cam/src/fpv.h) and passes each chunk to the laptop over USB at 1 Mbaud (this board's CP2102 stops there, about 98 KB/s of chunks) as [length u16][rssi i8][chunk], the shard starting with its FPV2 header. Until it hears a chunk it steps through channels 1-13 every 150 ms; it stays on a channel while chunks arrive and resumes searching after a second without one.
#include <string.h>
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "nvs_flash.h"

#define BAUD 1000000
#define HEADER 24 // a management frame's 802.11 header
#define ACTION "\x7f" "FPV" // vendor-specific action category and marker, ahead of the chunk
#define FCS 4
#define SEARCH_US 150000
#define LOST_US 1000000
#define CHUNK 1400 // the camera's largest chunk payload

// The camera sends broadcast action frames with this BSSID.
static const uint8_t bssid[6] = {'F', 'P', 'V', 'R', 'A', 'W'};
static RingbufHandle_t chunks;
static volatile int64_t heard_us;
static uint16_t admitted;

static void received(void *buffer, wifi_promiscuous_pkt_type_t type) {
    const wifi_promiscuous_pkt_t *packet = buffer;
    const uint8_t *frame = packet->payload;
    const int length = (int)packet->rx_ctrl.sig_len - HEADER - 4 - FCS;
    if (type != WIFI_PKT_MGMT || length < 20 || memcmp(frame + 16, bssid, 6) || memcmp(frame + HEADER, ACTION, 4) || memcmp(frame + HEADER + 4, "FPV2", 4)) return;
    heard_us = esp_timer_get_time();
    // The camera sends each frame as shards, any data of which rebuild it (fw/esp32cam/src/fec.h): the first that arrives admits the frame only if the buffer has room for that many, and the first data distinct shards go on. With the USB side behind, whole frames are dropped rather than shards of many.
    const uint8_t *chunk = frame + HEADER + 4;
    const uint16_t number = chunk[4] | chunk[5] << 8;
    static uint16_t seen_frame;
    static uint64_t seen;
    static int passed;
    static bool started;
    if (!started || number != seen_frame) {
        started = true;
        seen_frame = number;
        seen = 0;
        passed = 0;
        admitted = xRingbufferGetCurFreeSize(chunks) >= chunk[8] * (3 + 20 + CHUNK) ? number : number - 1;
    }
    const uint64_t bit = 1ull << (chunk[6] & 63);
    if (number != admitted || (seen & bit) || passed >= chunk[8]) return;
    seen |= bit;
    ++passed;
    // The Wi-Fi task is the only writer, so one static buffer serves.
    static uint8_t out[3 + 20 + CHUNK];
    out[0] = length & 0xff;
    out[1] = length >> 8;
    out[2] = (uint8_t)packet->rx_ctrl.rssi;
    memcpy(out + 3, chunk, length);
    xRingbufferSend(chunks, out, 3 + length, 0);
}

static void forward(void *unused) {
    for (;;) {
        size_t size;
        uint8_t *item = xRingbufferReceiveUpTo(chunks, &size, portMAX_DELAY, 4096);
        uart_write_bytes(UART_NUM_0, item, size);
        vRingbufferReturnItem(chunks, item);
    }
}

void app_main(void) {
    // A byte buffer, whose free size is all of its free space: a no-split buffer reports only the largest contiguous piece, which near its end refused frames that would have fitted.
    chunks = xRingbufferCreate(32 * 1024, RINGBUF_TYPE_BYTEBUF);
    uart_config_t uart = {.baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreatePinnedToCore(forward, "forward", 4096, NULL, 5, NULL, 1);

    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(received));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    int channel = 1;
    for (;;) {
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(SEARCH_US / 1000));
        while (esp_timer_get_time() - heard_us < LOST_US) vTaskDelay(pdMS_TO_TICKS(100));
        channel = channel % 13 + 1;
    }
}
