// TAG3 pose and flow packets from the ESP32-CAM over UART (fw/esp32cam/README.md). The camera raises SYNC as it queues each packet and sends it at 2 Mbaud; the rising edge's time minus the packet's age is the capture time on this clock, whatever the UART and task latency. The byte stream is searched for the magic and checked by CRC, so the camera's boot messages on the same wire (at 115200) are skipped.
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "camera.h"
#include "camera_link.h"

#define PORT UART_NUM_2
#define RX 34
#define TX 27
#define SYNC 13
#define BAUD 2000000
// The camera's own limit is 150 ms capture-to-queued; anything older on arrival is dropped.
#define MAX_AGE_US 150000

static QueueHandle_t events;
static pcnt_unit_handle_t rx_counter;
static volatile uint32_t edges, edge_us;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static CameraPose latest, latest_candidate;
static CameraFlow latest_flow;
static bool have_pose, have_candidate, have_flow;
static CameraStats stats = {.frame_us = -1};
// The update request waiting for the camera's answer (camera_update), and that answer.
static SemaphoreHandle_t answered;
static uint32_t awaited;
static CameraUpdateReply answer;

// The edge time is the low 32 bits of esp_timer, compared by unsigned difference. This ISR runs from flash, so during a flash write (only while disarmed) the edge is stamped late and the age reads low.
static void sync_edge(void *arg) {
    portENTER_CRITICAL_ISR(&lock);
    edge_us = (uint32_t)esp_timer_get_time();
    edges++;
    portEXIT_CRITICAL_ISR(&lock);
}

static void accept(const CameraPacket *packet, bool synced, uint32_t since_edge_us, int64_t now) {
    static uint32_t last_sequence;
    static bool have_sequence;
    bool repeat = have_sequence && packet->sequence == last_sequence;
    uint32_t age = packet->age_us + since_edge_us;
    CameraPose pose = {.sequence = packet->sequence, .capture_us = now - age, .tag_id = packet->tag_id, .valid = packet->valid,
                       .margin = packet->margin, .reprojection_px = packet->reprojection_px,
                       .alt_reprojection_px = packet->alt_reprojection_px};
    memcpy(pose.position_m, packet->position, sizeof(pose.position_m));
    memcpy(pose.rotation, packet->rotation, sizeof(pose.rotation));
    memcpy(pose.tag_position_m, packet->tag_position, sizeof(pose.tag_position_m));
    memcpy(pose.alt_position_m, packet->alt_position, sizeof(pose.alt_position_m));
    memcpy(pose.alt_rotation, packet->alt_rotation, sizeof(pose.alt_rotation));
    CameraFlow flow = {.sequence = packet->sequence, .capture_us = now - age + packet->flow_offset_us,
                       .interval_us = packet->flow_interval_us, .shift_px = {packet->flow[0], packet->flow[1]},
                       .residual_px = packet->flow_residual_px, .inliers = packet->flow_inliers};
    portENTER_CRITICAL(&lock);
    if (!synced) {
        stats.unsynced++;
    } else if (repeat) {
        stats.repeats++;
    } else {
        // The newest frame decides: one without the tag, or too old, clears the previous pose.
        stats.frames++;
        stats.frame_us = now;
        stats.sequence = packet->sequence;
        stats.capture_us = now - age;
        stats.valid = packet->valid;
        stats.trusted = packet->trusted;
        have_pose = packet->valid && packet->trusted && age <= MAX_AGE_US;
        stats.stale += packet->valid && !have_pose;
        stats.poses += have_pose;
        if (have_pose) latest = pose;
        have_candidate = packet->trusted && age <= MAX_AGE_US;
        if (have_candidate) latest_candidate = pose;
        have_flow = packet->flow_interval_us && age <= MAX_AGE_US;
        stats.flows += have_flow;
        if (have_flow) latest_flow = flow;
    }
    portEXIT_CRITICAL(&lock);
    // A restarted camera counts from zero again, so only an exact repeat is refused.
    last_sequence = packet->sequence;
    have_sequence = true;
}

static void receive_task(void *arg) {
    static CameraParser parser;
    static CameraReplyParser replies;
    static uint8_t bytes[256];
    uint32_t used_edges = 0;
    while (true) {
        uart_event_t event;
        if (xQueueReceive(events, &event, portMAX_DELAY) != pdTRUE) continue;
        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
            // Bytes were lost; the packet they belonged to fails its CRC and the parser resumes at the next "TAG3". What is already buffered is intact and still read: flushing it here used to throw away the good packets around each loss.
            portENTER_CRITICAL(&lock);
            stats.overflows++;
            portEXIT_CRITICAL(&lock);
        }
        if (event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR || event.type == UART_BREAK) {
            portENTER_CRITICAL(&lock);
            stats.line_errors++;
            portEXIT_CRITICAL(&lock);
        }
        size_t buffered = 0;
        uart_get_buffered_data_len(PORT, &buffered);
        while (buffered) {
            int n = uart_read_bytes(PORT, bytes, buffered < sizeof(bytes) ? buffered : sizeof(bytes), 0);
            if (n <= 0) break;
            buffered -= n;
            const uint8_t *data = bytes;
            size_t length = n;
            // Update replies share the wire with the packets; each parser skips the other's bytes.
            const uint8_t *reply_data = bytes;
            size_t reply_length = n;
            CameraUpdateReply reply;
            while (camera_reply_parse(&replies, &reply_data, &reply_length, &reply)) {
                portENTER_CRITICAL(&lock);
                bool wanted = awaited && reply.id == awaited;
                if (wanted) {
                    answer = reply;
                    awaited = 0;
                }
                portEXIT_CRITICAL(&lock);
                if (wanted) xSemaphoreGive(answered);
            }
            CameraPacket packet;
            while (true) {
                uint32_t skipped = parser.skipped, corrupt = parser.corrupt;
                bool complete = camera_parse(&parser, &data, &length, &packet);
                portENTER_CRITICAL(&lock);
                stats.skipped_bytes += parser.skipped - skipped;
                stats.corrupt += parser.corrupt - corrupt;
                stats.packets += complete;
                uint32_t edge_count = edges, edge_time = edge_us;
                portEXIT_CRITICAL(&lock);
                if (!complete) break;
                int64_t now = esp_timer_get_time();
                uint32_t since_edge_us;
                bool synced = camera_sync((uint32_t)now, edge_count, edge_time, &used_edges, &since_edge_us);
                accept(&packet, synced, since_edge_us, now);
            }
        }
    }
}

void camera_start(void) {
    uart_config_t link = {.baud_rate = BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                          .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    // A 2 KB receive buffer is a dozen packets; the transmit side (GPIO27, to the camera) carries update requests, written straight to the FIFO by the caller.
    answered = xSemaphoreCreateBinary();
    if (!answered) abort();
    ESP_ERROR_CHECK(uart_driver_install(PORT, 2048, 0, 16, &events, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT, &link));
    // The driver empties the 128-byte hardware FIFO when it holds 120 bytes, which at 2 Mbaud leaves 8 bytes (40 us) for the interrupt to run before bytes are lost; the camera's packets overflowed it on 10-40% of frames (2026-10-26). At 32 bytes the interrupt has 96 bytes (480 us) of slack, for a few more interrupts per packet.
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(PORT, 32));
    // A pulse counter shares the RX pin through the GPIO matrix and counts its falling edges, several hundred per packet; it accumulates past its 16-bit limit at each watch point.
    pcnt_unit_config_t unit = {.low_limit = -1, .high_limit = 30000, .flags.accum_count = 1};
    ESP_ERROR_CHECK(pcnt_new_unit(&unit, &rx_counter));
    pcnt_chan_config_t edges_on_rx = {.edge_gpio_num = RX, .level_gpio_num = -1};
    pcnt_channel_handle_t channel;
    ESP_ERROR_CHECK(pcnt_new_channel(rx_counter, &edges_on_rx, &channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(rx_counter, 30000));
    ESP_ERROR_CHECK(pcnt_unit_enable(rx_counter));
    ESP_ERROR_CHECK(pcnt_unit_start(rx_counter));
    // GPIO34 has no internal pull-up; with the camera unpowered the line floats and the parser skips the noise.
    ESP_ERROR_CHECK(uart_set_pin(PORT, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // The pull-down holds SYNC low while the camera is off or booting.
    gpio_config_t sync = {.pin_bit_mask = 1ULL << SYNC, .mode = GPIO_MODE_INPUT,
                          .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_POSEDGE};
    ESP_ERROR_CHECK(gpio_config(&sync));
    if (xTaskCreatePinnedToCore(receive_task, "camera", 3072, NULL, 4, NULL, 0) != pdPASS) abort();
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SYNC, sync_edge, NULL));
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
    stats_out->sync_edges = edges;
    portEXIT_CRITICAL(&lock);
    int rx_edges = 0;
    pcnt_unit_get_count(rx_counter, &rx_edges);
    stats_out->rx_edges = rx_edges;
}

const char *camera_update(uint8_t op, const uint8_t *body, size_t length, uint32_t timeout_ms, char *text, size_t size) {
    static uint8_t frame[sizeof(CameraUpdateRequest) + CAMERA_UPDATE_BODY + 2];
    static uint32_t next_id;
    static char failure[64];
    if (length > CAMERA_UPDATE_BODY) return "Camera request too long";
    if (!next_id) next_id = esp_random() | 1;
    CameraUpdateRequest header = {.magic = {'C', 'U', 'P', '1'}, .id = next_id++, .op = op, .length = length};
    if (!header.id) header.id = next_id++;
    memcpy(frame, &header, sizeof(header));
    memcpy(frame + sizeof(header), body, length);
    size_t total = sizeof(header) + length;
    uint16_t crc = camera_crc16(frame, total);
    frame[total++] = crc & 0xff;
    frame[total++] = crc >> 8;
    xSemaphoreTake(answered, 0);
    portENTER_CRITICAL(&lock);
    awaited = header.id;
    portEXIT_CRITICAL(&lock);
    uart_write_bytes(PORT, frame, total);
    bool done = xSemaphoreTake(answered, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
    portENTER_CRITICAL(&lock);
    awaited = 0;
    CameraUpdateReply reply = answer;
    portEXIT_CRITICAL(&lock);
    if (!done) return "The camera did not answer";
    size_t shown = reply.length < sizeof(reply.text) ? reply.length : sizeof(reply.text);
    if (reply.status) {
        snprintf(failure, sizeof(failure), "Camera: %.*s", (int)shown, reply.text);
        return failure;
    }
    snprintf(text, size, "%.*s", (int)shown, reply.text);
    return NULL;
}
