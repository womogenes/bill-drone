// TAG3 pose and flow packets from the ESP32-CAM over UART (fw/esp32cam/README.md). The camera pulses SYNC as it starts each packet, at the moment it measured the packet's capture-to-send age, so the capture time on this clock is the SYNC edge minus that age, whatever the UART's latency. Packets are found in the byte stream by their magic and CRC, so the camera's boot messages on the same line are skipped.
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "camera.h"

#define LINK_UART UART_NUM_2
#define RX 34
#define TX 26
#define SYNC 4
#define BAUD 2000000
// A packet takes 0.8 ms on the wire; one completed more than this after the last SYNC edge has no edge of its own.
#define MAX_EDGE_TO_PACKET_US 5000
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
    float tag_position[3], alt_position[3], alt_rotation[9], alt_reprojection_px;
    uint16_t trusted, checksum;
} Packet;
_Static_assert(sizeof(Packet) == 164, "TAG3 layout");

static Packet packet;
// Low 32 bits of esp_timer at the last SYNC rising edge, compared by unsigned difference.
static volatile uint32_t sync_us;
static volatile bool have_sync;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static CameraPose latest, latest_candidate;
static CameraFlow latest_flow;
static bool have_pose, have_candidate, have_flow;
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

// This ISR runs from flash, so during a flash write (only while disarmed) the edge is stamped late and the age reads low.
static void sync_edge(void *arg) {
    sync_us = (uint32_t)esp_timer_get_time();
    have_sync = true;
}

// Fills the packet from the UART stream: aligns on the "TAG3" magic, then reads the rest. Returns false on a timeout.
static bool read_packet(void) {
    uint8_t *bytes = (uint8_t *)&packet;
    int have = 0;
    while (have < 4) {
        if (uart_read_bytes(LINK_UART, bytes + have, 1, pdMS_TO_TICKS(100)) != 1) return false;
        if (bytes[have] == (uint8_t)"TAG3"[have]) have++;
        else have = bytes[have] == 'T' ? (bytes[0] = 'T', 1) : 0;
    }
    int rest = sizeof(Packet) - 4;
    return uart_read_bytes(LINK_UART, bytes + 4, rest, pdMS_TO_TICKS(20)) == rest;
}

static void receive_task(void *arg) {
    uint32_t last_sequence = 0;
    bool have_sequence = false;
    while (true) {
        if (!read_packet()) continue;
        int64_t now = esp_timer_get_time();
        uint32_t edge_us = sync_us;
        // Without a recent edge (SYNC unwired or missed) the send time is estimated from the arrival.
        bool edge = have_sync && (uint32_t)now - edge_us <= MAX_EDGE_TO_PACKET_US;
        if (!edge) edge_us = (uint32_t)now - (sizeof(Packet) * 10 * 1000000ULL / BAUD);
        have_sync = false;
        bool intact = crc16((const uint8_t *)&packet, offsetof(Packet, checksum)) == packet.checksum;
        bool repeat = intact && have_sequence && packet.sequence == last_sequence;
        uint32_t age = packet.age_us + ((uint32_t)now - edge_us);
        CameraPose pose = {.sequence = packet.sequence, .capture_us = now - age, .tag_id = packet.tag_id, .valid = packet.valid,
                           .margin = packet.margin, .reprojection_px = packet.reprojection_px,
                           .alt_reprojection_px = packet.alt_reprojection_px};
        memcpy(pose.position_m, packet.position, sizeof(pose.position_m));
        memcpy(pose.rotation, packet.rotation, sizeof(pose.rotation));
        memcpy(pose.tag_position_m, packet.tag_position, sizeof(pose.tag_position_m));
        memcpy(pose.alt_position_m, packet.alt_position, sizeof(pose.alt_position_m));
        memcpy(pose.alt_rotation, packet.alt_rotation, sizeof(pose.alt_rotation));
        CameraFlow flow = {.sequence = packet.sequence, .capture_us = now - age + packet.flow_offset_us,
                           .interval_us = packet.flow_interval_us, .shift_px = {packet.flow[0], packet.flow[1]},
                           .residual_px = packet.flow_residual_px, .inliers = packet.flow_inliers};
        portENTER_CRITICAL(&lock);
        stats.packets++;
        if (!intact) {
            stats.corrupt++;
        } else if (repeat) {
            stats.repeats++;
        } else {
            // The newest frame decides: one without the tag, or too old, clears the previous pose.
            stats.frames++;
            stats.unsynced += !edge;
            stats.frame_us = now;
            have_pose = packet.valid && packet.trusted && age <= MAX_AGE_US;
            stats.stale += packet.valid && !have_pose;
            stats.poses += have_pose;
            if (have_pose) latest = pose;
            have_candidate = packet.trusted && age <= MAX_AGE_US;
            if (have_candidate) latest_candidate = pose;
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
    uart_config_t link = {.baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                          .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &link));
    // TX (to the camera's RX) is wired for later use, such as gyro rates for the camera; nothing is sent yet.
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_config_t sync = {.pin_bit_mask = 1ULL << SYNC, .mode = GPIO_MODE_INPUT,
                          .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_POSEDGE};
    ESP_ERROR_CHECK(gpio_config(&sync));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SYNC, sync_edge, NULL));
    if (xTaskCreatePinnedToCore(receive_task, "camera", 3072, NULL, 4, NULL, 0) != pdPASS) abort();
}

bool camera_pose(CameraPose *pose, uint32_t max_age_us) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&lock);
    bool fresh = have_pose && now - latest.capture_us <= max_age_us;
    if (fresh) *pose = latest;
    portEXIT_CRITICAL(&lock);
    return fresh;
}

bool camera_candidates(CameraPose *pose, uint32_t max_age_us) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&lock);
    bool fresh = have_candidate && now - latest_candidate.capture_us <= max_age_us;
    if (fresh) *pose = latest_candidate;
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
