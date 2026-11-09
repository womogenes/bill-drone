// USB-to-ESP-NOW relay. The laptop supplies every controller heartbeat.

#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_now.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "../../../pairs.h"
#include "../../../link.h"

// The channel, the rate this relay sends to the drone at (link.h link_rates; 1 is long range 500 kbps) and its maximum transmit power (dBm, 0 = the driver's own) at power-up. The laptop can change all three until the next reset with an RCF1 frame (bridge.py POST /radio), so a mismatch is undone by resetting or replugging this board.
#define CHANNEL 6
#define RATE 1
#define POWER 0
#define PACKET_MAX 250
#define HEADER_SIZE 18
#define PENDING_MAX 260
#define WINDOW 4
#define OUTGOING_MAX 8
// The board's blue LED: blinks at 1 Hz until the link works both ways, then stays on while in the last 2 s the drone was heard and acknowledged a packet from the laptop. The drone's LED goes by the laptop's packets reaching it, so the controller's is not solid while the drone's blinks (only a packet the drone receives but rejects as stale can still differ).
#define LED 2
#define HEARD_US 2000000

// The paired drone's station MAC (fw/pairs.h).
static uint8_t drone_mac[6];

typedef struct {
    uint8_t length;
    uint8_t data[PACKET_MAX];
} packet_t;

static QueueHandle_t outgoing;
static QueueHandle_t incoming;
static QueueHandle_t uart_events;
static SemaphoreHandle_t window;
static volatile int64_t heard_us;
static volatile int64_t delivered_us;
// Updates may switch the link to the fast rate: an update request F (fw/drone/main/radio.c) that this relay forwards also sets its own rate, after the request is sent, so the drone hears it at the old one. Each further update request keeps it fast for FAST_HOLD_US; after that it goes back to long range by itself, as the drone does.
#define FAST_HOLD_US 3000000
static const esp_now_rate_config_t fast_rate = {.phymode = WIFI_PHY_MODE_11B, .rate = WIFI_PHY_RATE_11M_L};
static bool fast;
static int64_t fast_until;
// The radio as set (RATE, POWER, CHANNEL or an RCF1 frame's), power in 0.25 dBm; the driver's power from boot, which power 0 restores.
static uint8_t usual_rate = RATE, channel_now = CHANNEL;
static int8_t power_now, power_boot;

static void set_rate(bool on)
{
    esp_now_rate_config_t rate = on ? fast_rate : link_rates[usual_rate];
    if (esp_now_set_peer_rate_config(drone_mac, &rate) == ESP_OK) {
        fast = on;
    }
}

// The link report (link.h): this relay's counts, the drone's session from its newest packet (so the report reads as the drone's link), and an RCF1 change waiting for the radio task (0xff: unchanged).
static LinkCounter link;
static volatile uint32_t drone_session;
static uint32_t report_sequence;
static portMUX_TYPE config_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t requested[3] = {0xff, 0xff, 0xff};
static volatile bool configured, report_now;

static void set_power(int dbm)
{
    int8_t wanted = dbm ? (dbm * 4 < 8 ? 8 : dbm * 4 > 84 ? 84 : dbm * 4) : power_boot;
    if (esp_wifi_set_max_tx_power(wanted) == ESP_OK) {
        esp_wifi_get_max_tx_power(&power_now);
    }
}

// An RCF1 change: rate index, power (dBm, 0 the driver's own) and channel, each 0xff to keep. From the radio task, which owns the rate.
static void apply_config(void)
{
    portENTER_CRITICAL(&config_lock);
    uint8_t rate = requested[0], power = requested[1], channel = requested[2];
    memset(requested, 0xff, sizeof(requested));
    configured = false;
    portEXIT_CRITICAL(&config_lock);
    if (rate < LINK_RATES && rate != usual_rate) {
        usual_rate = rate;
        if (!fast) {
            set_rate(false);
        }
    }
    if (power <= 21) {
        set_power(power);
    }
    if (channel >= 1 && channel <= 11 && channel != channel_now && esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) == ESP_OK) {
        channel_now = channel;
    }
    report_now = true;
}

// Matches wire.unpack: size, magic, channel, fragment index and count.
static bool valid_header(const uint8_t *packet, size_t length)
{
    return length >= HEADER_SIZE && length <= PACKET_MAX && memcmp(packet, "DC1", 3) == 0
        && packet[3] <= LINK_CHANNEL && packet[16] < packet[17] && packet[17] <= 32;
}

static void sent(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    const int64_t now = esp_timer_get_time();
    if (status == ESP_NOW_SEND_SUCCESS) {
        delivered_us = now;
    }
    link_sent(&link, now, status == ESP_NOW_SEND_SUCCESS);
    xSemaphoreGive(window);
}

static void received(const esp_now_recv_info_t *info, const uint8_t *data, int length)
{
    if (memcmp(info->src_addr, drone_mac, 6) || length > PACKET_MAX) {
        return;
    }
    heard_us = esp_timer_get_time();
    uint32_t number = 0;
    if (length >= HEADER_SIZE) {
        uint32_t session;
        memcpy(&session, data + 4, 4);
        memcpy(&number, data + 8, 4);
        drone_session = session;
    }
    // Every drone packet takes a number from one counter (fw/drone/main/radio.c next_sequence), so skipped numbers are lost packets or ones the drone could not send (its report's refused).
    link_heard(&link, heard_us, info, number, length >= HEADER_SIZE);
    packet_t packet = {.length = length};
    memcpy(packet.data, data, length);
    xQueueSend(incoming, &packet, 0);
}

static void led_task(void *argument)
{
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    for (;;) {
        const int64_t now = esp_timer_get_time();
        const bool connected = heard_us && now - heard_us < HEARD_US && delivered_us && now - delivered_us < HEARD_US;
        gpio_set_level(LED, connected || now / 500000 % 2 == 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// This relay's link report on channel 7 (link.h), under the drone's newest session so the bridge reads it as this drone's link.
static void report_packet(packet_t *packet, const LinkReport *report)
{
    uint32_t session = drone_session, sequence = ++report_sequence, token = 0;
    memcpy(packet->data, "DC1", 3);
    packet->data[3] = LINK_CHANNEL;
    memcpy(packet->data + 4, &session, 4);
    memcpy(packet->data + 8, &sequence, 4);
    memcpy(packet->data + 12, &token, 4);
    packet->data[16] = 0;
    packet->data[17] = 1;
    memcpy(packet->data + HEADER_SIZE, report, sizeof(*report));
    packet->length = HEADER_SIZE + sizeof(*report);
}

// The link report once a second to the laptop (and at once after an RCF1 change), and every BEACON_MS to the drone as well: a drone following its controller (radio_follow) stays on the channel where it hears it, even while the laptop sends nothing.
#define BEACON_MS 250
static void report_task(void *argument)
{
    LinkReport report;
    packet_t packet;
    int64_t reported_us = esp_timer_get_time();
    bool have = false;
    for (int beat = 0;; beat++) {
        vTaskDelay(pdMS_TO_TICKS(BEACON_MS / 5));
        const int64_t now = esp_timer_get_time();
        if (report_now || now - reported_us >= LINK_REPORT_US) {
            report_now = false;
            reported_us = now;
            link_report(&link, now, "RLY1", &report);
            report.channel = channel_now;
            report.rate = fast ? 5 : usual_rate;
            report.power = power_now;
            have = true;
            report_packet(&packet, &report);
            xQueueSend(incoming, &packet, 0);
        }
        // Into the queue only while it has room: a beacon never displaces the laptop's packets.
        if (have && beat % 5 == 0) {
            report_packet(&packet, &report);
            xQueueSend(outgoing, &packet, 0);
        }
    }
}

static void radio_task(void *argument)
{
    packet_t packet;
    for (;;) {
        if (configured) {
            apply_config();
        }
        // Hold a send slot before taking a packet so waiting packets stay droppable.
        xSemaphoreTake(window, portMAX_DELAY);
        if (xQueueReceive(outgoing, &packet, pdMS_TO_TICKS(100)) != pdTRUE) {
            xSemaphoreGive(window);
            if (fast && esp_timer_get_time() > fast_until) {
                set_rate(false);
            }
            continue;
        }
        link_sending(&link, esp_timer_get_time());
        if (esp_now_send(drone_mac, packet.data, packet.length) != ESP_OK) {
            link_unsent(&link);
            xSemaphoreGive(window);
        }
        // Channel 6 carries requests: id, then op (first fragment only).
        if (packet.data[3] == 6) {
            if (fast) {
                fast_until = esp_timer_get_time() + FAST_HOLD_US;
            }
            if (packet.data[16] == 0 && packet.length >= HEADER_SIZE + 6 && packet.data[HEADER_SIZE + 4] == 'F') {
                set_rate(packet.data[HEADER_SIZE + 5]);
                fast_until = esp_timer_get_time() + FAST_HOLD_US;
            }
        }
    }
}

static void usb_write_task(void *argument)
{
    packet_t packet;
    uint8_t encoded[PACKET_MAX + 4 + 3];
    for (;;) {
        xQueueReceive(incoming, &packet, portMAX_DELAY);
        uint32_t crc = esp_rom_crc32_le(0, packet.data, packet.length);
        uint8_t data[PACKET_MAX + 4];
        memcpy(data, packet.data, packet.length);
        memcpy(data + packet.length, &crc, 4);
        size_t length = packet.length + 4;
        size_t position = 0;
        size_t size = 1;
        uint8_t code = 1;
        for (size_t i = 0; i < length; i++) {
            if (data[i] == 0) {
                encoded[position] = code;
                position = size++;
                code = 1;
            } else {
                encoded[size++] = data[i];
                if (++code == 255) {
                    encoded[position] = code;
                    position = size++;
                    code = 1;
                }
            }
        }
        encoded[position] = code;
        encoded[size++] = 0;
        uart_write_bytes(UART_NUM_0, encoded, size);
    }
}

static void decode_frame(const uint8_t *pending, size_t length)
{
    packet_t packet;
    uint8_t decoded[PENDING_MAX];
    size_t size = 0;
    size_t offset = 0;
    while (offset < length) {
        uint8_t code = pending[offset];
        size_t end = offset + code;
        if (end > length) {
            return;
        }
        memcpy(decoded + size, pending + offset + 1, code - 1);
        size += code - 1;
        offset = end;
        if (code < 255 && offset < length) {
            decoded[size++] = 0;
        }
    }
    if (size < HEADER_SIZE + 4) {
        link_rejected(&link);
        return;
    }
    size -= 4;
    uint32_t crc;
    memcpy(&crc, decoded + size, 4);
    if (esp_rom_crc32_le(0, decoded, size) != crc || !valid_header(decoded, size)) {
        link_rejected(&link);
        return;
    }
    uint8_t channel = decoded[3];
    // Channel 7 from the laptop is for this relay: RCF1, then rate, power and channel (0xff keeps one), applied by the radio task.
    if (channel == LINK_CHANNEL) {
        if (size >= HEADER_SIZE + 7 && !memcmp(decoded + HEADER_SIZE, "RCF1", 4)) {
            portENTER_CRITICAL(&config_lock);
            memcpy(requested, decoded + HEADER_SIZE + 4, 3);
            configured = true;
            portEXIT_CRITICAL(&config_lock);
        }
        return;
    }
    if (channel != 1 && channel != 4 && channel != 5 && channel != 6) {
        return;
    }
    packet.length = size;
    memcpy(packet.data, decoded, size);
    if (xQueueSend(outgoing, &packet, 0) != pdTRUE) {
        // The oldest waiting packet makes room and counts as dropped.
        packet_t oldest;
        xQueueReceive(outgoing, &oldest, 0);
        xQueueSend(outgoing, &packet, 0);
        link_rejected(&link);
    }
}

static void usb_read_task(void *argument)
{
    static uint8_t buffer[512];
    static uint8_t pending[PENDING_MAX + 1];
    size_t length = 0;
    bool discard = false;
    uart_event_t event;
    for (;;) {
        xQueueReceive(uart_events, &event, portMAX_DELAY);
        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
            uart_flush_input(UART_NUM_0);
            link_rejected(&link);
            xQueueReset(uart_events);
            length = 0;
            discard = true;
            continue;
        }
        if (event.type != UART_DATA) {
            continue;
        }
        int count;
        while ((count = uart_read_bytes(UART_NUM_0, buffer, sizeof(buffer), 0)) > 0) {
            for (int i = 0; i < count; i++) {
                uint8_t value = buffer[i];
                if (value) {
                    if (!discard) {
                        pending[length++] = value;
                        if (length > PENDING_MAX) {
                            length = 0;
                            discard = true;
                        }
                    }
                    continue;
                }
                if (!discard && length) {
                    decode_frame(pending, length);
                }
                length = 0;
                discard = false;
            }
        }
    }
}

void app_main(void)
{
    // Before the callbacks that count into it are registered.
    link_init(&link, esp_timer_get_time());
    outgoing = xQueueCreate(OUTGOING_MAX, sizeof(packet_t));
    incoming = xQueueCreate(32, sizeof(packet_t));
    window = xSemaphoreCreateCounting(WINDOW, WINDOW);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // Long range as well as 802.11b/g/n: the drone's default-rate frames still arrive, and this relay sends to the drone at the 500 kbps long-range rate, several dB more margin against the other networks on channel 6 (the drone receives long range since 2026-10-29). With both protocol sets on both ends each side hears every rate in link.h, so either may change its sending rate alone; an LR-only protocol set would gain no sensitivity and would deafen a side to the other's 802.11b frames.
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(received));
    uint8_t own[6];
    pair_find(false, own, drone_mac);
    esp_now_peer_info_t peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
    memcpy(peer.peer_addr, drone_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    esp_now_rate_config_t rate = link_rates[RATE];
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(drone_mac, &rate));
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power_boot));
    power_now = power_boot;
    if (POWER) {
        set_power(POWER);
    }

    uart_config_t uart = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 4096, 16, &uart_events, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(usb_read_task, "usb_read", 4096, NULL, 12, NULL);
    xTaskCreate(usb_write_task, "usb_write", 4096, NULL, 11, NULL);
    xTaskCreate(radio_task, "radio", 4096, NULL, 11, NULL);
    xTaskCreate(led_task, "led", 2048, NULL, 1, NULL);
    xTaskCreate(report_task, "report", 3072, NULL, 5, NULL);
}
