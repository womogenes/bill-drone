// FPV receiver: hears the camera's raw 802.11 video (action frames) (fw/esp32cam/src/fpv.h) and passes each chunk to the laptop over USB as [length u16][rssi i8][chunk], the shard starting with its FPV2 header. On an ESP32 the USB is its board's CP2102 at 1 Mbaud, about 98 KB/s of chunks (a 12 KB frame took 120 ms to cross); on an ESP32-S3 (make s3) it is the chip's own USB (USB-Serial-JTAG, /dev/ttyACM*), with no baud rate to stop it. While no chunks arrive it sends FPVR (ESP32) or FPVS (S3, so the laptop gives the camera the larger budget) twice a second, by which the laptop knows this port from the other boards' (every CP2102 here has the same USB serial number). Every camera sends with the same BSSID and its own MAC as the frame's sender, so once the laptop names its camera (FPVF and the six bytes of its MAC, written to this port) only that camera's frames pass and steady the channel search; until then every camera's do. Until it hears a chunk it steps through channels 1-13 every 150 ms; it stays on a channel while chunks arrive and resumes searching after a second without one.
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "driver/usb_serial_jtag.h"
#endif
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

#define BAUD 1000000
#define HEADER 24 // a management frame's 802.11 header
#define ACTION "\x7f" "FPV" // vendor-specific action category and marker, ahead of the chunk
#define FCS 4
#define SEARCH_US 150000
#define LOST_US 1000000
#define CHUNK 1400 // the camera's largest chunk payload
#if CONFIG_IDF_TARGET_ESP32S3
// The S3 camera board's GPIO3 drives its flash LEDs, so no LED; its buffer holds a whole frame of the larger budget.
#define BEACON "FPVS"
#define BUFFER (96 * 1024)
#else
#define LED GPIO_NUM_2 // the board's blue LED
#define BEACON "FPVR"
#define BUFFER (32 * 1024)
#define UART_TX_BUFFER 8192
#endif

// The camera sends broadcast action frames with this BSSID.
static const uint8_t bssid[6] = {'F', 'P', 'V', 'R', 'A', 'W'};
static RingbufHandle_t chunks;
static uint8_t camera[6];
static volatile bool chosen;
static volatile int64_t heard_us;
// Camera updates (fw/esp32cam/src/air_update.h, fw/tools/camera_air_ota.py): the laptop's FPVE messages go to the chosen camera over ESP-NOW, and the camera's go back as FPVE, the length (u16) and the message. FPVC and a channel holds the radio there while they flow (held_us), where the channel search would wander off with no video coming. The last send's outcome comes from the send callback.
static volatile int64_t held_us;
static SemaphoreHandle_t send_done;
static volatile bool send_ok;
#define HOLD_US 5000000
static uint16_t admitted;
// Where frames go missing, sent to the laptop every second as FPVT and eight u32 (since boot): frames heard (their first shard), frames declined for want of buffer room for all their data shards, frames admitted but short of shards when the next began (the air), shards the buffer refused, the longest a USB write took in the last second (us; long means the laptop is not reading) and the bytes written, then the shards the heard frames had (data and parity) and the distinct ones of them heard, whose shortfall is the air's loss.
static volatile uint32_t frames_heard, frames_declined, frames_short, shards_refused, usb_longest_us, usb_bytes, shards_sent, shards_heard;
// With several receivers on the laptop (FPVK, a first, a count and a modulus) each owns the shards whose index modulo the modulus falls in [first, first + count) and passes only its share of each frame's data shards (count / modulus of them) from those, so their USB links add up instead of all carrying the same first shards. Any data distinct shards rebuild a frame, and a frame's shards are about half parity, so each receiver's class holds about twice its share: it can lose half of them in the air. Beyond its share it passes others' shards only while its USB has nothing waiting (FILL_BYTES), which covers a receiver that missed its own when there is room and costs nothing when there is not. Alone (0, 1, 1, the default) every shard is its own and its share is all the data shards, as before.
static volatile uint8_t slot_first = 0, slot_count = 1, slot_modulus = 1;
#define FILL_BYTES (3 + 20 + CHUNK)

// Bytes waiting for the USB: in the buffer and, on the ESP32, in the UART driver's own transmit buffer, which the forward task fills at once.
static int backlog(void) {
    int waiting = BUFFER - (int)xRingbufferGetCurFreeSize(chunks);
#if !CONFIG_IDF_TARGET_ESP32S3
    size_t free = 0;
    uart_get_tx_buffer_free_size(UART_NUM_0, &free);
    waiting += UART_TX_BUFFER - (int)free;
#endif
    return waiting;
}

static void received(void *buffer, wifi_promiscuous_pkt_type_t type) {
    const wifi_promiscuous_pkt_t *packet = buffer;
    const uint8_t *frame = packet->payload;
    const int length = (int)packet->rx_ctrl.sig_len - HEADER - 4 - FCS;
    if (type != WIFI_PKT_MGMT || length < 20 || memcmp(frame + 16, bssid, 6) || memcmp(frame + HEADER, ACTION, 4) || memcmp(frame + HEADER + 4, "FPV2", 4)) return;
    if (chosen && memcmp(frame + 10, camera, 6)) return;
    heard_us = esp_timer_get_time();
    // The camera sends each frame as shards, any data of which rebuild it (fw/esp32cam/src/fec.h): the first that arrives admits the frame only if the buffer has room for that many, and the first data distinct shards go on. With the USB side behind, whole frames are dropped rather than shards of many.
    const uint8_t *chunk = frame + HEADER + 4;
    const uint16_t number = chunk[4] | chunk[5] << 8;
    static uint16_t seen_frame;
    static uint64_t seen, heard_bits;
    static int passed, data_wanted, own_passed, share;
    static bool started;
    if (!started || number != seen_frame) {
        if (started && admitted == seen_frame && passed < data_wanted) ++frames_short;
        started = true;
        seen_frame = number;
        seen = heard_bits = 0;
        passed = own_passed = 0;
        share = (chunk[8] * slot_count + slot_modulus - 1) / slot_modulus;
        shards_sent += chunk[7];
        data_wanted = chunk[8];
        ++frames_heard;
        admitted = xRingbufferGetCurFreeSize(chunks) >= share * (3 + 20 + CHUNK) ? number : number - 1;
        if (admitted != number) ++frames_declined;
    }
    const uint64_t bit = 1ull << (chunk[6] & 63);
    if (!(heard_bits & bit)) {
        heard_bits |= bit;
        ++shards_heard;
    }
    if (number != admitted || (seen & bit) || passed >= chunk[8]) return;
    const bool own = (uint8_t)(chunk[6] % slot_modulus - slot_first) < slot_count && own_passed < share;
    if (!own && backlog() >= FILL_BYTES) return;
    seen |= bit;
    ++passed;
    own_passed += own;
    // The Wi-Fi task is the only writer, so one static buffer serves.
    static uint8_t out[3 + 20 + CHUNK];
    out[0] = length & 0xff;
    out[1] = length >> 8;
    out[2] = (uint8_t)packet->rx_ctrl.rssi;
    memcpy(out + 3, chunk, length);
    if (xRingbufferSend(chunks, out, 3 + length, 0) != pdTRUE) ++shards_refused;
}

static void write_out(const void *data, size_t size) {
#if CONFIG_IDF_TARGET_ESP32S3
    usb_serial_jtag_write_bytes(data, size, portMAX_DELAY);
#else
    uart_write_bytes(UART_NUM_0, data, size);
#endif
}

static int read_in(uint8_t *data, size_t size) {
#if CONFIG_IDF_TARGET_ESP32S3
    return usb_serial_jtag_read_bytes(data, size, portMAX_DELAY);
#else
    // A short wait, not forever: uart_read_bytes waits for all size bytes, so with portMAX_DELAY the commands (FPVF and its camera, FPVE) were taken only once 512 bytes had come, which the laptop's occasional FPVF never made: the ESP32 receiver passed every camera's frames (found 2026-11-14).
    return uart_read_bytes(UART_NUM_0, data, size, pdMS_TO_TICKS(10));
#endif
}

static void forward(void *unused) {
    for (;;) {
        size_t size;
        uint8_t *item = xRingbufferReceiveUpTo(chunks, &size, portMAX_DELAY, 4096);
        const int64_t began = esp_timer_get_time();
        write_out(item, size);
        const uint32_t took = esp_timer_get_time() - began;
        if (took > usb_longest_us) usb_longest_us = took;
        usb_bytes += size;
        vRingbufferReturnItem(chunks, item);
    }
}

// The LED (ESP32): solid while a camera's chunks arrive (within LOST_US), blinking at 1 Hz while none do, with the beacon to the laptop at each change.
static void blink(void *unused) {
#ifdef LED
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
#endif
    for (bool lit = false, report = false;; vTaskDelay(pdMS_TO_TICKS(500))) {
        const bool hearing = esp_timer_get_time() - heard_us < LOST_US;
        if (!hearing) xRingbufferSend(chunks, BEACON, 4, 0);
        if ((report = !report)) {
            uint32_t stats[9] = {0, frames_heard, frames_declined, frames_short, shards_refused, usb_longest_us, usb_bytes, shards_sent, shards_heard};
            memcpy(stats, "FPVT", 4);
            usb_longest_us = 0;
            xRingbufferSend(chunks, stats, sizeof(stats), 0);
        }
        lit = hearing || !lit;
#ifdef LED
        gpio_set_level(LED, lit);
#endif
    }
}

static void espnow_received(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
    if (!chosen || memcmp(info->src_addr, camera, 6) || length > ESP_NOW_MAX_DATA_LEN) return;
    uint8_t out[6 + ESP_NOW_MAX_DATA_LEN];
    memcpy(out, "FPVE", 4);
    out[4] = length & 0xff;
    out[5] = length >> 8;
    memcpy(out + 6, data, length);
    xRingbufferSend(chunks, out, 6 + length, 0);
}

static void espnow_sent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    send_ok = status == ESP_NOW_SEND_SUCCESS;
    xSemaphoreGive(send_done);
}

// One message to the camera, tried until it is acknowledged (up to 5 times); FPVX goes back to the laptop if it never is.
static void to_camera(const uint8_t *data, int length) {
    static uint8_t peer[6];
    if (memcmp(peer, camera, 6)) {
        if (esp_now_is_peer_exist(peer)) esp_now_del_peer(peer);
        esp_now_peer_info_t info = {.ifidx = WIFI_IF_STA};
        memcpy(info.peer_addr, camera, 6);
        if (esp_now_add_peer(&info) != ESP_OK) return;
        memcpy(peer, camera, 6);
        // 24 Mbps, as the video comes: at the default 1 Mbps a 4 KB sector took 40 ms of air.
        esp_now_rate_config_t rate = {.phymode = WIFI_PHY_MODE_11G, .rate = WIFI_PHY_RATE_24M};
        esp_now_set_peer_rate_config(peer, &rate);
    }
    held_us = esp_timer_get_time();
    for (int tries = 0; tries < 5; ++tries) {
        xSemaphoreTake(send_done, 0);
        if (esp_now_send(camera, data, length) == ESP_OK && xSemaphoreTake(send_done, pdMS_TO_TICKS(100)) == pdTRUE && send_ok) return;
    }
    xRingbufferSend(chunks, "FPVX", 4, 0);
}

// The laptop's commands, looked for anywhere in what arrives: FPVF and a camera's MAC (only its frames pass from then on), FPVC and a channel (held while updates flow), FPVE, a length (u16) and a message for the camera, FPVK and this receiver's shards (first, count, modulus; acknowledged with FPVA, so the laptop knows this firmware took them).
static void commands(void *unused) {
    static uint8_t got[512];
    int have = 0;
    for (;;) {
        const int n = read_in(got + have, sizeof(got) - have);
        if (n <= 0) continue;
        have += n;
        for (;;) {
            // Not the start of a command: drop bytes from the front until what is left could be.
            int skip = 0;
            while (skip < have && memcmp(got + skip, "FPV", have - skip < 3 ? have - skip : 3)) ++skip;
            if (skip) memmove(got, got + skip, have -= skip);
            if (have < 5) break;
            const uint8_t kind = got[3];
            int used = 1;
            if (kind == 'F') {
                if (have < 10) break;
                memcpy(camera, got + 4, 6);
                chosen = true;
                used = 10;
            } else if (kind == 'C') {
                if (got[4] >= 1 && got[4] <= 13) {
                    held_us = esp_timer_get_time();
                    esp_wifi_set_channel(got[4], WIFI_SECOND_CHAN_NONE);
                }
                used = 5;
            } else if (kind == 'K') {
                if (have < 7) break;
                if (got[6] >= 1 && got[5] >= 1 && got[4] + got[5] <= got[6]) {
                    slot_first = got[4];
                    slot_count = got[5];
                    slot_modulus = got[6];
                    // Acknowledged as FPVA, not FPVK: a port left echoing (a terminal's defaults) sent an FPVK echo back, which was echoed again, thousands a second (2026-11-14).
                    uint8_t ack[7] = {'F', 'P', 'V', 'A', got[4], got[5], got[6]};
                    xRingbufferSend(chunks, ack, 7, 0);
                }
                used = 7;
            } else if (kind == 'E') {
                if (have < 6) break;
                const int length = got[4] | got[5] << 8;
                if (length <= ESP_NOW_MAX_DATA_LEN) {
                    if (have < 6 + length) break;
                    if (chosen) to_camera(got + 6, length);
                    used = 6 + length;
                }
            }
            memmove(got, got + used, have -= used);
        }
        // A buffer full of nothing usable starts over.
        if (have == sizeof(got)) have = 0;
    }
}

void app_main(void) {
    // A byte buffer, whose free size is all of its free space: a no-split buffer reports only the largest contiguous piece, which near its end refused frames that would have fitted.
    chunks = xRingbufferCreate(BUFFER, RINGBUF_TYPE_BYTEBUF);
#if CONFIG_IDF_TARGET_ESP32S3
    usb_serial_jtag_driver_config_t usb = {.tx_buffer_size = 16384, .rx_buffer_size = 8192};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
#else
    uart_config_t uart = {.baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, UART_TX_BUFFER, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif
    xTaskCreatePinnedToCore(forward, "forward", 4096, NULL, 5, NULL, 1);
    send_done = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(commands, "commands", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(blink, "blink", 2048, NULL, 1, NULL, 1);

    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    // A station joined to nothing, for ESP-NOW; the video comes by promiscuous receive either way.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // No power save: a sleeping station hears nothing.
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_received));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_sent));
    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(received));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    int channel = 1;
    for (;;) {
        if (esp_timer_get_time() - held_us >= HOLD_US) esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(SEARCH_US / 1000));
        while (esp_timer_get_time() - heard_us < LOST_US || esp_timer_get_time() - held_us < HOLD_US) vTaskDelay(pdMS_TO_TICKS(100));
        channel = channel % 13 + 1;
    }
}
