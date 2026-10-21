// TAG2 pose and flow packets from the ESP32-CAM over SPI (fw/esp32cam/README.md). The camera raises READY when a packet is queued; its rising edge wakes the receiver, which clocks the 100 bytes at once and adds how long READY was high to the camera's capture-to-ready age.
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "camera.h"

#define SCLK 27
#define MOSI 26
#define MISO 34
#define CS 23
#define READY 4
// The camera's own limit is 150 ms capture-to-ready; anything older on arrival is dropped.
#define MAX_AGE_US 150000

typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t sequence, capture_us, age_us;
    uint16_t tag_id, valid;
    float position[3], rotation[9], margin, reprojection_px, flow[2];
    uint32_t flow_interval_us;
    float flow_residual_px;
    uint16_t flow_inliers;
    int16_t flow_offset_us;
    uint16_t reserved, checksum;
} Packet;
_Static_assert(sizeof(Packet) == 100, "TAG2 layout");

static DMA_ATTR Packet packet;
static spi_device_handle_t device;
static TaskHandle_t task;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static CameraPose latest;
static CameraFlow latest_flow;
static bool have_pose, have_flow;
static CameraStats stats = {.frame_us = -1};

// CRC-16/CCITT-FALSE, as in the camera's packet.h.
static uint16_t crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i] << 8;
        for (int bit = 0; bit < 8; bit++) crc = crc & 0x8000 ? crc << 1 ^ 0x1021 : crc << 1;
    }
    return crc;
}

// The edge time travels in the notification value: the low 32 bits of esp_timer, compared by unsigned difference. This ISR runs from flash, so during a flash write (only while disarmed) the edge is stamped late and the age reads low.
static void ready_edge(void *arg) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(task, (uint32_t)esp_timer_get_time(), eSetValueWithOverwrite, &woken);
    portYIELD_FROM_ISR(woken);
}

static void receive_task(void *arg) {
    spi_transaction_t transfer = {.length = sizeof(packet) * 8, .rx_buffer = &packet};
    uint32_t last_sequence = 0;
    bool have_sequence = false;
    while (true) {
        uint32_t edge_us;
        bool edge = xTaskNotifyWait(0, 0, &edge_us, pdMS_TO_TICKS(100)) == pdTRUE;
        // Without a read the camera never queues another packet, so a READY left high is drained.
        if (!edge && !gpio_get_level(READY)) continue;
        ESP_ERROR_CHECK(spi_device_polling_transmit(device, &transfer));
        int64_t now = esp_timer_get_time();
        bool intact = !memcmp(packet.magic, "TAG2", 4) &&
                      crc16((const uint8_t *)&packet, offsetof(Packet, checksum)) == packet.checksum;
        bool repeat = intact && have_sequence && packet.sequence == last_sequence;
        uint32_t age = packet.age_us + ((uint32_t)now - edge_us);
        CameraPose pose = {.sequence = packet.sequence, .capture_us = now - age, .tag_id = packet.tag_id, .valid = packet.valid,
                           .margin = packet.margin, .reprojection_px = packet.reprojection_px};
        memcpy(pose.position_m, packet.position, sizeof(pose.position_m));
        memcpy(pose.rotation, packet.rotation, sizeof(pose.rotation));
        CameraFlow flow = {.sequence = packet.sequence, .capture_us = now - age + packet.flow_offset_us,
                           .interval_us = packet.flow_interval_us, .shift_px = {packet.flow[0], packet.flow[1]},
                           .residual_px = packet.flow_residual_px, .inliers = packet.flow_inliers};
        portENTER_CRITICAL(&lock);
        stats.packets++;
        if (!intact) {
            stats.corrupt++;
        } else if (!edge) {
            stats.unsynced++;
        } else if (repeat) {
            stats.repeats++;
        } else {
            // The newest frame decides: one without the tag, or too old, clears the previous pose.
            stats.frames++;
            stats.frame_us = now;
            have_pose = packet.valid && age <= MAX_AGE_US;
            stats.stale += packet.valid && !have_pose;
            stats.poses += have_pose;
            if (have_pose) latest = pose;
            have_flow = packet.flow_interval_us && age <= MAX_AGE_US;
            stats.flows += have_flow;
            if (have_flow) latest_flow = flow;
        }
        portEXIT_CRITICAL(&lock);
        // A restarted camera counts from zero again, so only an exact repeat is refused.
        if (intact) {
            last_sequence = packet.sequence;
            have_sequence = true;
        }
    }
}

void camera_start(void) {
    spi_bus_config_t bus = {.mosi_io_num = MOSI, .miso_io_num = MISO, .sclk_io_num = SCLK,
                            .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = sizeof(Packet)};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    // ESP32 slave DMA needs mode 1 or 3; the camera uses 1.
    spi_device_interface_config_t camera = {.mode = 1, .clock_speed_hz = 4000000, .spics_io_num = CS, .queue_size = 1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &camera, &device));
    // The pull-down also keeps the camera's GPIO2 strap low.
    gpio_config_t ready = {.pin_bit_mask = 1ULL << READY, .mode = GPIO_MODE_INPUT,
                           .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_POSEDGE};
    ESP_ERROR_CHECK(gpio_config(&ready));
    if (xTaskCreatePinnedToCore(receive_task, "camera", 3072, NULL, 4, &task, 0) != pdPASS) abort();
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(READY, ready_edge, NULL));
}

bool camera_pose(CameraPose *pose, uint32_t max_age_us) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&lock);
    bool fresh = have_pose && now - latest.capture_us <= max_age_us;
    if (fresh) *pose = latest;
    portEXIT_CRITICAL(&lock);
    return fresh;
}

bool camera_flow(CameraFlow *flow, uint32_t max_age_us) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&lock);
    bool fresh = have_flow && now - latest_flow.capture_us <= max_age_us;
    if (fresh) *flow = latest_flow;
    portEXIT_CRITICAL(&lock);
    return fresh;
}

void camera_stats(CameraStats *stats_out) {
    portENTER_CRITICAL(&lock);
    *stats_out = stats;
    portEXIT_CRITICAL(&lock);
}
