// USB-to-ESP-NOW relay. The laptop supplies every controller heartbeat.

#include <string.h>
#include "driver/uart.h"
#include "esp_now.h"
#include "esp_rom_crc.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

#define CHANNEL 6
#define PACKET_MAX 250
#define HEADER_SIZE 18
#define PENDING_MAX 260
#define WINDOW 4
#define OUTGOING_MAX 8

static const uint8_t drone_mac[6] = {0xb0, 0x3f, 0xd3, 0x52, 0x5c, 0x60};

typedef struct {
    uint8_t length;
    uint8_t data[PACKET_MAX];
} packet_t;

static QueueHandle_t outgoing;
static QueueHandle_t incoming;
static QueueHandle_t uart_events;
static SemaphoreHandle_t window;

// Matches wire.unpack: size, magic, channel, fragment index and count.
static bool valid_header(const uint8_t *packet, size_t length)
{
    return length >= HEADER_SIZE && length <= PACKET_MAX && memcmp(packet, "DC1", 3) == 0
        && packet[3] <= 6 && packet[16] < packet[17] && packet[17] <= 32;
}

static void sent(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    xSemaphoreGive(window);
}

static void received(const esp_now_recv_info_t *info, const uint8_t *data, int length)
{
    if (memcmp(info->src_addr, drone_mac, 6) || length > PACKET_MAX) {
        return;
    }
    packet_t packet = {.length = length};
    memcpy(packet.data, data, length);
    xQueueSend(incoming, &packet, 0);
}

static void radio_task(void *argument)
{
    packet_t packet;
    for (;;) {
        // Hold a send slot before taking a packet so waiting packets stay droppable.
        xSemaphoreTake(window, portMAX_DELAY);
        xQueueReceive(outgoing, &packet, portMAX_DELAY);
        if (esp_now_send(drone_mac, packet.data, packet.length) != ESP_OK) {
            xSemaphoreGive(window);
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
        return;
    }
    size -= 4;
    uint32_t crc;
    memcpy(&crc, decoded + size, 4);
    if (esp_rom_crc32_le(0, decoded, size) != crc || !valid_header(decoded, size)) {
        return;
    }
    uint8_t channel = decoded[3];
    if (channel != 1 && channel != 4 && channel != 5 && channel != 6) {
        return;
    }
    packet.length = size;
    memcpy(packet.data, decoded, size);
    if (xQueueSend(outgoing, &packet, 0) != pdTRUE) {
        packet_t oldest;
        xQueueReceive(outgoing, &oldest, 0);
        xQueueSend(outgoing, &packet, 0);
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
    outgoing = xQueueCreate(OUTGOING_MAX, sizeof(packet_t));
    incoming = xQueueCreate(32, sizeof(packet_t));
    window = xSemaphoreCreateCounting(WINDOW, WINDOW);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(received));
    esp_now_peer_info_t peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = false};
    memcpy(peer.peer_addr, drone_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

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
}
