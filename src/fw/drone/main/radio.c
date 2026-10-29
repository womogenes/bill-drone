// Direct ESP-NOW link: status messages, streamed targets, stop, and the request service for updates and transactions. Framing: src/fw/controller/wire.py.
//
// Channels: 0 status (drone, every 50 ms), 1 targets (laptop, newest wins; each one is the controller heartbeat),
// 2/3 binary/JSON flight log (one message in flight, resent until acknowledged), 4 recorder subscription,
// 5 log acknowledgement, 6 requests and replies (id-tagged, fragmented).
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_ota_ops.h"
#include "miniz.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mbedtls/sha256.h"
#include "runtime.h"
#include "settings.h"
#include "arming.h"
#include "camera.h"

// The controller's station MAC; the drone answers nothing else.
static const uint8_t peer[ESP_NOW_ETH_ALEN] = {0x04, 0xb2, 0x47, 0x54, 0x80, 0x4c};
#define WIFI_CHANNEL 6

#define HEADER 18
#define PAYLOAD 232
#define CHANNELS 7
#define WINDOW 4
#define REQUEST (18 * PAYLOAD)
// Big enough for the longest settings JSON after the 5-byte reply header, and at least 8 fragments for a schema page and the other replies.
#define REPLY_FRAGMENTS ((5 + SETTINGS_JSON_SIZE + PAYLOAD - 1) / PAYLOAD)
#define REPLY ((REPLY_FRAGMENTS > 8 ? REPLY_FRAGMENTS : 8) * PAYLOAD)
#define SECTOR 4096
#define HASH 8
// Target floats: base, roll, pitch, yaw rate, two servos, two throttles, then the hover mode (hover.h), which older senders leave out.
#define TARGETS 12
// An image that is still unconfirmed this long after boot restarts, so the bootloader rolls back to the previous one.
#define CONFIRM_US 90000000
// Most records per FLG1 batch (controller/log_format.py MAX_BATCH) and the batch header size: a full batch fits two fragments, so one goes out per tick.
#define LOG_MAX 5
#define LOG_HEADER 12

// Request states, each with one owner of the request and reply buffers: the receive callback fills IDLE, the update task handles READY and sends REPLIED.
enum { IDLE, READY, REPLIED };

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t session, sequence, ticks;
static int inflight;
static uint32_t latest[CHANNELS];
static bool seen[CHANNELS];
static uint32_t target_sequence;
static uint8_t stop_id[4];
static bool stop_pending;
static bool log_requested;
static int64_t log_us;
static uint32_t log_acks[CHANNELS];
// The flight-log streamer runs in tick(); meta is the latest metadata JSON (main.c).
static uint8_t log_message[LOG_HEADER + LOG_MAX * RECORD_SIZE];
static const uint8_t *log_data;
static size_t log_length, log_records;
static uint8_t log_channel;
static uint32_t log_number, streamed_session;
static int log_fragment, log_count;
static int64_t log_sent_us;
static bool log_busy, session_open;
static uint8_t meta[META];
static size_t meta_length;
static bool meta_dirty, meta_sending, meta_writing;
static uint8_t request[REQUEST] __attribute__((aligned(4)));
static uint32_t request_sequence, request_mask, request_length;
// The runtime's stop count when the request arrived: an arm request is refused after any later stop.
static uint32_t request_generation;
static ArmHistory arm_history;
static int request_state;
static uint8_t reply[REPLY];
static size_t reply_length;
static bool has_reply;
static TaskHandle_t update_task;
// The running image awaits confirmation (V) after an update.
static bool unconfirmed;
// Updates may switch the link to FAST_RATE (request F; the controller's relay follows the same request) while requests keep coming: with none for FAST_HOLD_US it goes back to its usual rate by itself, so a lost switch-back cannot leave it fast.
#define FAST_HOLD_US 3000000
// This drone's usual rate is the default 1 Mbps, not long range: its 100 Hz log stream needs the airtime, and at 500 kbps retries on a busy channel filled the log ring (10-30% of samples dropped on 2026-10-29); the controller still sends to it at long range, so the targets keep the margin.
static const esp_now_rate_config_t usual_rate = {.phymode = WIFI_PHY_MODE_11B, .rate = WIFI_PHY_RATE_1M_L};
static const esp_now_rate_config_t fast_rate = {.phymode = WIFI_PHY_MODE_11B, .rate = WIFI_PHY_RATE_11M_L};
static bool fast;
static int64_t fast_until;
// Compressed sector writes (request Y): raw deflate, expanded by the ROM's inflater into a sector.
static tinfl_decompressor inflater;
static uint8_t inflated[SECTOR];

static void set_rate(bool on) {
    esp_now_rate_config_t rate = on ? fast_rate : usual_rate;
    if (esp_now_set_peer_rate_config(peer, &rate) == ESP_OK) fast = on;
}

static uint32_t now_ms(void) {
    return esp_timer_get_time() / 1000;
}

static uint32_t next_sequence(void) {
    portENTER_CRITICAL(&lock);
    if (!++sequence) sequence = 1;
    uint32_t value = sequence;
    portEXIT_CRITICAL(&lock);
    return value;
}

// Never waits: ESP-NOW keeps at most WINDOW frames queued, and a refused frame is the caller's to retry or drop.
static bool transmit(uint8_t channel, uint32_t number, const uint8_t *data, size_t length, uint8_t index, uint8_t count) {
    uint8_t packet[HEADER + PAYLOAD];
    uint32_t token = now_ms();
    memcpy(packet, "DC1", 3);
    packet[3] = channel;
    memcpy(packet + 4, &session, 4);
    memcpy(packet + 8, &number, 4);
    memcpy(packet + 12, &token, 4);
    packet[16] = index;
    packet[17] = count;
    memcpy(packet + HEADER, data, length);
    portENTER_CRITICAL(&lock);
    bool open = inflight < WINDOW;
    if (open) inflight++;
    portEXIT_CRITICAL(&lock);
    if (!open) return false;
    // The send callback may run before esp_now_send returns, so the slot is taken first.
    bool ok = esp_now_send(peer, packet, HEADER + length) == ESP_OK;
    if (!ok) {
        portENTER_CRITICAL(&lock);
        inflight--;
        portEXIT_CRITICAL(&lock);
    }
    return ok;
}

static void on_sent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    portENTER_CRITICAL(&lock);
    inflight--;
    portEXIT_CRITICAL(&lock);
}

static void collect(uint32_t number, uint8_t index, uint8_t count, const uint8_t *data, size_t length) {
    size_t start = index * PAYLOAD;
    // Every fragment but the last is full, so a request has no gaps.
    if ((index + 1 < count && length != PAYLOAD) || start + length > REQUEST) return;
    portENTER_CRITICAL(&lock);
    bool idle = request_state == IDLE;
    portEXIT_CRITICAL(&lock);
    // A request arriving while the previous one is handled is dropped; the laptop retries it.
    if (!idle) return;
    if (request_mask == 0 || request_sequence != number) {
        request_sequence = number;
        request_mask = 0;
    }
    memcpy(request + start, data, length);
    request_mask |= 1U << index;
    if (index + 1 == count) request_length = start + length;
    if (request_mask != (count == 32 ? 0xffffffffU : (1U << count) - 1)) return;
    request_mask = 0;
    // Read after any stop handled earlier in this same receive task.
    uint32_t generation = flight_runtime_stop_generation();
    portENTER_CRITICAL(&lock);
    request_generation = generation;
    request_state = READY;
    portEXIT_CRITICAL(&lock);
    xTaskNotifyGive(update_task);
}

// When a packet from the paired controller last passed the session and freshness checks, for radio_connected().
static volatile int64_t heard_us = INT64_MIN / 2;

bool radio_connected(void) { return esp_timer_get_time() - heard_us < 2000000; }

// Runs in the Wi-Fi task: parse, check freshness and order, and queue; never sends.
static void on_received(const esp_now_recv_info_t *info, const uint8_t *packet, int size) {
    if (memcmp(info->src_addr, peer, ESP_NOW_ETH_ALEN) || size < HEADER || size > HEADER + PAYLOAD ||
        memcmp(packet, "DC1", 3) || packet[3] >= CHANNELS || packet[17] > 32 || packet[16] >= packet[17])
        return;
    uint8_t channel = packet[3], index = packet[16], count = packet[17];
    uint32_t packet_session, number, token;
    memcpy(&packet_session, packet + 4, 4);
    memcpy(&number, packet + 8, 4);
    memcpy(&token, packet + 12, 4);
    const uint8_t *data = packet + HEADER;
    size_t length = size - HEADER;
    int32_t age = (int32_t)(now_ms() - token);
    if (packet_session != session || age < 0 || age > 200) return;
    heard_us = esp_timer_get_time();
    if (channel == 6) {
        // Stop preempts everything, including a request in progress; the laptop repeats it until answered.
        if (count == 1 && length == 5 && data[4] == 'X') {
            flight_runtime_command_stop(number);
            portENTER_CRITICAL(&lock);
            memcpy(stop_id, data, 4);
            stop_pending = true;
            portEXIT_CRITICAL(&lock);
        } else {
            collect(number, index, count, data, length);
        }
        return;
    }
    if (count != 1) return;
    if (channel == 5) {
        if (length == 1 && data[0] < CHANNELS) {
            portENTER_CRITICAL(&lock); log_acks[data[0]] = number; portEXIT_CRITICAL(&lock);
        }
        return;
    }
    if ((channel != 1 && channel != 4) || !length) return;
    // Older laptops send 8 targets (no mode), 9 (no hold-point nudge) or 11 (no height nudge); the rest stay zero.
    if (channel == 1 && ((length != 1 + TARGETS * 4 && length != 1 + 11 * 4 && length != 1 + 9 * 4 && length != 1 + 8 * 4) || data[0] != 'T')) return;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&lock);
    bool newer = !seen[channel] || (uint32_t)(number - latest[channel]) - 1 < 0x7fffffffU;
    if (newer) {
        seen[channel] = true;
        latest[channel] = number;
        if (channel == 4 && data[0] == 0) {
            log_requested = false;
        } else if (channel == 4 && (data[0] == 1 || data[0] == 'H')) {
            log_us = now;
            // A new subscriber gets the metadata again.
            if (data[0] == 1 && !log_requested) meta_dirty = meta_length > 0;
            if (data[0] == 1) log_requested = true;
        }
    }
    portEXIT_CRITICAL(&lock);
    if (!newer || channel != 1) return;
    // A late or reordered target packet is dropped: the newer one already superseded it.
    float targets[TARGETS] = {0};
    memcpy(targets, data + 1, length - 1);
    if (flight_runtime_targets(targets)) {
        portENTER_CRITICAL(&lock); target_sequence = number; portEXIT_CRITICAL(&lock);
    }
}

static bool subscribed(int64_t now) {
    return log_requested && now - log_us < 3000000;
}

static void put32(uint8_t *at, uint32_t value) {
    memcpy(at, &value, 4);
}

// Picks the next log message: a session's start, then metadata, then sample batches, then its stop record.
static void next_log_message(void) {
    LogState state;
    flight_runtime_log(&state);
    portENTER_CRITICAL(&lock);
    bool send_meta = meta_dirty && !meta_writing;
    if (send_meta && !(state.session && state.session != streamed_session)) {
        meta_dirty = false;
        meta_sending = true;
    }
    portEXIT_CRITICAL(&lock);
    log_channel = 2;
    log_data = log_message;
    log_records = 0;
    if (state.session && state.session != streamed_session) {
        memcpy(log_message, "LGS1", 4);
        put32(log_message + 4, state.session);
        put32(log_message + 8, state.started_us);
        memcpy(log_message + 12, state.bias, 12);
        memcpy(log_message + 24, state.pose_zero, 8);
        log_message[32] = state.calibrated;
        log_length = 33;
        portENTER_CRITICAL(&lock);
        streamed_session = state.session;
        session_open = true;
        // Every recording gets the metadata, even when the laptop service restarted since it last arrived.
        meta_dirty = meta_length > 0;
        portEXIT_CRITICAL(&lock);
    } else if (send_meta) {
        log_channel = 3;
        log_data = meta;
        log_length = meta_length;
    } else if (session_open && (state.pending >= LOG_BATCH || (state.pending && !state.active))) {
        log_records = flight_runtime_read_log(log_message + LOG_HEADER, LOG_MAX);
        memcpy(log_message, "FLG1", 4);
        put32(log_message + 4, state.session);
        log_message[8] = log_records;
        log_message[9] = 0;
        log_message[10] = RECORD_SIZE;
        log_message[11] = 0;
        log_length = LOG_HEADER + log_records * RECORD_SIZE;
    } else if (session_open && !state.active) {
        memcpy(log_message, "LGE1", 4);
        put32(log_message + 4, state.session);
        log_message[8] = state.reason;
        put32(log_message + 9, state.samples);
        put32(log_message + 13, state.dropped);
        put32(log_message + 17, state.elapsed_us);
        log_length = 21;
        portENTER_CRITICAL(&lock);
        session_open = false;
        portEXIT_CRITICAL(&lock);
    } else {
        return;
    }
    log_number = next_sequence();
    log_count = (log_length + PAYLOAD - 1) / PAYLOAD;
    log_fragment = 0;
    log_busy = true;
}

// Background traffic: at most two log fragments per tick, after the status message and the stop reply.
static void stream_log(int64_t now) {
    portENTER_CRITICAL(&lock);
    bool active = subscribed(now);
    uint32_t acked = log_acks[log_channel];
    portEXIT_CRITICAL(&lock);
    if (log_busy && log_fragment == log_count) {
        if (acked == log_number) {
            if (log_records) flight_runtime_consume_log(log_records);
            if (log_channel == 3) {
                portENTER_CRITICAL(&lock); meta_sending = false; portEXIT_CRITICAL(&lock);
            }
            log_busy = false;
        } else if (now - log_sent_us > 100000) {
            log_fragment = 0;
        }
    }
    if (!active) return;
    if (!log_busy) next_log_message();
    for (int sent_now = 0; log_busy && log_fragment < log_count && sent_now < 2; sent_now++) {
        size_t offset = log_fragment * PAYLOAD;
        size_t length = log_length - offset < PAYLOAD ? log_length - offset : PAYLOAD;
        if (!transmit(log_channel, log_number, log_data + offset, length, log_fragment, log_count)) break;
        if (++log_fragment == log_count) log_sent_us = now;
    }
}

static void camera_status(Status *status, int64_t now) {
    CameraStats camera;
    camera_stats(&camera);
    status->camera_packets = camera.packets;
    status->camera_frames = camera.frames;
    status->camera_poses = camera.poses;
    status->camera_flows = camera.flows;
    status->camera_corrupt = camera.corrupt;
    status->camera_repeats = camera.repeats;
    status->camera_stale = camera.stale;
    status->camera_unsynced = camera.unsynced;
    status->camera_skipped_bytes = camera.skipped_bytes;
    status->camera_overflows = camera.overflows;
    status->camera_line_errors = camera.line_errors;
    status->camera_sync_edges = camera.sync_edges;
    status->camera_rx_edges = camera.rx_edges;
    status->camera_sequence = camera.sequence;
    status->camera_age_ms = camera.frame_us < 0 ? UINT32_MAX : (now - camera.capture_us) / 1000;
    status->camera_valid = camera.valid;
    status->camera_trusted = camera.trusted;
}

static void tick(void *arg) {
    portENTER_CRITICAL(&lock);
    bool stopping = stop_pending;
    uint8_t answer[5] = {stop_id[0], stop_id[1], stop_id[2], stop_id[3], 0};
    portEXIT_CRITICAL(&lock);
    if (stopping && transmit(6, next_sequence(), answer, sizeof(answer), 0, 1)) {
        portENTER_CRITICAL(&lock);
        if (!memcmp(stop_id, answer, 4)) stop_pending = false;
        portEXIT_CRITICAL(&lock);
    }
    if (++ticks % 5) {
        stream_log(esp_timer_get_time());
        return;
    }
    Status status = {.magic = {'S', 'T', 'A', '1'}};
    flight_runtime_status(&status);
    sensors_status(&status);
    int64_t now = esp_timer_get_time();
    status.time_ms = now / 1000;
    status.settings_version = settings_version();
    status.flags |= settings_calibration_unsaved() << 1;
    camera_status(&status, now);
    portENTER_CRITICAL(&lock);
    status.target_sequence = target_sequence;
    status.flags |= subscribed(now) << 5;
    portEXIT_CRITICAL(&lock);
    transmit(0, next_sequence(), (const uint8_t *)&status, sizeof(status), 0, 1);
    stream_log(now);
}

static bool hash_matches(const uint8_t *data, const uint8_t *expected) {
    uint8_t digest[32];
    mbedtls_sha256(data, SECTOR, digest, 0);
    return !memcmp(digest, expected, HASH);
}

static const char *write_sector(const esp_partition_t *target, uint32_t sector, const uint8_t *data) {
    if ((sector + 1) * SECTOR > target->size) return "Sector outside the slot";
    if (!flight_runtime_begin_maintenance()) return "Disable control and zero throttle before updating";
    // Writing over the fallback slot requires the running image to stay selected.
    esp_ota_mark_app_valid_cancel_rollback();
    unconfirmed = false;
    esp_err_t error = esp_partition_erase_range(target, sector * SECTOR, SECTOR);
    if (!error) error = esp_partition_write(target, sector * SECTOR, data, SECTOR);
    const char *failure = error ? esp_err_to_name(error) : NULL;
    uint8_t check[256];
    for (size_t offset = 0; !failure && offset < SECTOR; offset += sizeof(check)) {
        error = esp_partition_read(target, sector * SECTOR + offset, check, sizeof(check));
        if (error) failure = esp_err_to_name(error);
        else if (memcmp(check, data + offset, sizeof(check))) failure = "Sector readback mismatch";
    }
    flight_runtime_end_maintenance();
    return failure;
}

static void describe(char *text, size_t size, const esp_partition_t *partition) {
    esp_app_desc_t app;
    if (!partition || esp_ota_get_partition_description(partition, &app)) {
        snprintf(text, size, "null");
        return;
    }
    snprintf(text, size, "\"%02x%02x%02x%02x%02x%02x%02x%02x\"", app.app_elf_sha256[0], app.app_elf_sha256[1],
             app.app_elf_sha256[2], app.app_elf_sha256[3], app.app_elf_sha256[4], app.app_elf_sha256[5],
             app.app_elf_sha256[6], app.app_elf_sha256[7]);
}

// Requests: A arm, I info, W write sector, C copy sector from the running slot, B boot, V confirm, R reset, S read settings, T tune (JSON object), D schema page, G calibrate IMU, Z zero a servo, X servo bench test (start or read), P load the test pattern (flight_runtime_pattern), Q set the accelerometer correction (offset x, y, z then gain x, y, z as float32), Y write a deflate-compressed sector, F the fast update rate on (1) or off (0), K a camera firmware update request (its op, then its body; camera_update). Fills reply after the id and status bytes; returns an error or NULL.
static const char *handle(uint8_t op, const uint8_t *body, size_t length, size_t *size) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    char *text = (char *)reply + 5;
    static char error[96];
    if (op == 'S' || op == 'D') {
        *size = op == 'S' ? settings_json(text, REPLY - 5) : settings_schema(text, REPLY - 5, length ? body[0] : 0);
        return *size ? NULL : "Settings do not fit the reply";
    }
    if (op == 'X') {
        // Servo bench test: a float32 amplitude (degrees) and a byte of cycles starts it; one byte (any) reads the results.
        if (length == 5) {
            float amp;
            memcpy(&amp, body, 4);
            const char *failure = flight_runtime_servo_test(amp, body[4]);
            if (failure) return failure;
        } else if (length != 1) {
            return "Expected float32 amplitude and a byte of cycles, or one byte to read";
        }
        *size = flight_runtime_servo_result(text, REPLY - 5);
        return *size ? NULL : "Result does not fit the reply";
    }
    if (op == 'P') {
        const char *failure = flight_runtime_pattern(body, length, text, REPLY - 5);
        if (failure) return failure;
        *size = strlen(text);
        return NULL;
    }
    if (op == 'T' || op == 'G' || op == 'Z' || op == 'Q' || op == 'L') {
        // Each of these ends in an NVS write, which is only allowed while disarmed.
        if (!flight_runtime_idle()) return "Disarm and zero throttle first";
        int64_t now = esp_timer_get_time();
        if (op == 'T') {
            const char *failure = settings_tune((const char *)body, length, now, error, sizeof(error));
            if (failure) return failure;
        } else if (op == 'G') {
            const char *failure = flight_runtime_calibrate();
            if (failure) return failure;
        } else if (op == 'L') {
            if (length != 8) return "Expected two float32 values";
            // A new zero pose makes the old pitch trim meaningless.
            int indices[] = {SETTING_pose_zero_roll, SETTING_pose_zero_pitch, SETTING_pitch_trim};
            float values[3] = {0};
            memcpy(values, body, 8);
            for (int i = 0; i < 2; i++) {
                const char *failure = settings_validate(indices[i], values[i], error, sizeof(error));
                if (failure) return failure;
            }
            settings_change(indices, values, 3, now);
        } else if (op == 'Q') {
            if (length != 24) return "Expected six float32 values";
            int indices[6];
            float values[6];
            memcpy(values, body, sizeof(values));
            for (int i = 0; i < 6; i++) {
                indices[i] = (i < 3 ? SETTING_accel_offset_x : SETTING_accel_gain_x) + i % 3;
                const char *failure = settings_validate(indices[i], values[i], error, sizeof(error));
                if (failure) return failure;
            }
            settings_change(indices, values, 6, now);
        } else {
            if (length != 1 || body[0] > 1) return "Expected side 0 or 1";
            float offset;
            const char *failure = flight_runtime_zero_servo(body[0], &offset, error, sizeof(error));
            if (failure) return failure;
            int index = body[0] ? SETTING_servo_offset_right : SETTING_servo_offset_left;
            settings_change(&index, &offset, 1, now);
        }
        flight_runtime_apply_settings();
        *size = settings_json(text, REPLY - 5);
        return NULL;
    }
    if (op == 'K') {
        if (!length) return "Expected a camera request";
        if (!flight_runtime_idle()) return "Disable control and zero throttle before updating the camera";
        // Selecting the new image reads all of it to check its hash.
        const char *failure = camera_update(body[0], body + 1, length - 1, body[0] == 'B' ? 5000 : 2000, text, REPLY - 5);
        if (failure) return failure;
        *size = strlen(text);
        return NULL;
    }
    if (op == 'A') {
        // The recorder must be subscribed and the previous recording closed: each armed session is one recording.
        int64_t now = esp_timer_get_time();
        portENTER_CRITICAL(&lock);
        bool recorder = subscribed(now) && !session_open;
        portEXIT_CRITICAL(&lock);
        int earlier = arm_history_find(&arm_history, request);
        if (earlier >= 0) return earlier ? NULL : "Arm request was refused earlier";
        // request_sequence is the packet sequence all of the request's fragments share.
        const char *failure = flight_runtime_arm(recorder, settings_calibration_unsaved(), request_generation,
                                                 request_sequence);
        arm_history_add(&arm_history, request, !failure);
        return failure;
    }
    if (op == 'I') {
        char running_sha[24], target_sha[24];
        describe(running_sha, sizeof(running_sha), running);
        describe(target_sha, sizeof(target_sha), target);
        // reset is esp_reset_reason_t: 1 power-on, 3 software, 4 panic, 5-7 watchdogs, 9 brownout.
        *size = snprintf(text, REPLY - 5, "{\"running\":\"%s\",\"target\":%s%s%s,\"size\":%lu,\"running_sha\":%s,\"target_sha\":%s,\"reset\":%d}",
                         running->label, target ? "\"" : "", target ? target->label : "null", target ? "\"" : "",
                         target ? (unsigned long)target->size : 0UL, running_sha, target_sha, esp_reset_reason());
        return NULL;
    }
    if (op == 'V') {
        esp_err_t error = esp_ota_mark_app_valid_cancel_rollback();
        if (!error) unconfirmed = false;
        return error ? esp_err_to_name(error) : NULL;
    }
    if (op == 'R') return flight_runtime_idle() ? NULL : "Disable control and zero throttle before resetting";
    if (op == 'W') {
        if (length != 2 + HASH + SECTOR) return "Malformed sector write";
        if (!hash_matches(body + 2 + HASH, body + 2)) return "Sector arrived corrupted";
        return write_sector(target, body[0] | body[1] << 8, body + 2 + HASH);
    }
    if (op == 'Y') {
        if (length < 2 + HASH + 1) return "Malformed compressed sector";
        size_t in = length - 2 - HASH, out = SECTOR;
        tinfl_init(&inflater);
        tinfl_status status = tinfl_decompress(&inflater, body + 2 + HASH, &in, inflated, inflated, &out, TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
        if (status != TINFL_STATUS_DONE || out != SECTOR) return "Compressed sector did not expand to a sector";
        if (!hash_matches(inflated, body + 2)) return "Sector arrived corrupted";
        return write_sector(target, body[0] | body[1] << 8, inflated);
    }
    if (op == 'F') {
        if (length != 1) return "Expected 1 or 0";
        set_rate(body[0]);
        fast_until = esp_timer_get_time() + FAST_HOLD_US;
        return NULL;
    }
    if (op == 'C') {
        if (length != 2 + HASH) return "Malformed sector copy";
        uint32_t sector = body[0] | body[1] << 8;
        // The request is short, so the end of its buffer holds the sector.
        uint8_t *data = request + REQUEST - SECTOR;
        if ((sector + 1) * SECTOR > running->size) return "Sector outside the slot";
        esp_err_t error = esp_partition_read(running, sector * SECTOR, data, SECTOR);
        if (error) return esp_err_to_name(error);
        if (!hash_matches(data, body + 2)) return "Running slot differs";
        return write_sector(target, sector, data);
    }
    if (op == 'B') {
        if (!flight_runtime_begin_maintenance()) return "Disable control and zero throttle before updating";
        // ESP-IDF checks the whole image and its SHA-256 before selecting it.
        esp_err_t error = esp_ota_set_boot_partition(target);
        flight_runtime_end_maintenance();
        return error ? esp_err_to_name(error) : NULL;
    }
    return "Unknown update request";
}

static void send_reply(void) {
    uint32_t number = next_sequence();
    int count = (reply_length + PAYLOAD - 1) / PAYLOAD;
    for (int index = 0, attempt = 0; index < count && attempt < 200; attempt++) {
        size_t offset = index * PAYLOAD, length = reply_length - offset < PAYLOAD ? reply_length - offset : PAYLOAD;
        if (transmit(6, number, reply + offset, length, index, count)) index++;
        else vTaskDelay(1);
    }
}

// Stores a finished calibration, then commits changed settings while disarmed: flash writes never run under the control lock or in the control task.
static void store_settings(void) {
    float bias[3], pose_zero[2], sensor_frame;
    int64_t now = esp_timer_get_time();
    if (flight_runtime_take_calibration(bias, pose_zero, &sensor_frame)) {
        int indices[] = {SETTING_gyro_bias_x, SETTING_gyro_bias_y, SETTING_gyro_bias_z, SETTING_pose_zero_roll,
                         SETTING_pose_zero_pitch, SETTING_calibration_frame, SETTING_calibrated};
        float changes[] = {bias[0], bias[1], bias[2], pose_zero[0], pose_zero[1], sensor_frame, 1};
        settings_change(indices, changes, 7, now);
    }
    float trimmed[2];
    float hover_trim[2];
    if (flight_runtime_take_hover_trim(&hover_trim[0], &hover_trim[1])) {
        int indices[] = {SETTING_hover_trim_start_x, SETTING_hover_trim_start_y};
        if (settings_change(indices, hover_trim, 2, now)) flight_runtime_apply_settings();
    }
    if (flight_runtime_take_trim(&trimmed[0], &trimmed[1])) {
        int indices[] = {SETTING_roll_bias, SETTING_pitch_bias};
        if (settings_change(indices, trimmed, 2, now)) flight_runtime_apply_settings();
    }
    if (settings_save_due(now) && flight_runtime_begin_maintenance()) {
        settings_commit(now);
        flight_runtime_end_maintenance();
    }
}

static void update_loop(void *arg) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
        if (unconfirmed && esp_timer_get_time() > CONFIRM_US && flight_runtime_idle()) esp_restart();
        if (fast && esp_timer_get_time() > fast_until) set_rate(false);
        store_settings();
        portENTER_CRITICAL(&lock);
        int state = request_state;
        portEXIT_CRITICAL(&lock);
        if (state == READY) {
            if (request_length < 5) {
                portENTER_CRITICAL(&lock); request_state = IDLE; portEXIT_CRITICAL(&lock);
                continue;
            }
            uint8_t op = request[4];
            if (has_reply && !memcmp(reply, request, 4)) {
                // A retry: replay the reply instead of repeating the request.
                state = REPLIED;
            } else {
                size_t size = 0;
                if (fast) fast_until = esp_timer_get_time() + FAST_HOLD_US;
                const char *error = handle(op, request + 5, request_length - 5, &size);
                memcpy(reply, request, 4);
                reply[4] = error != NULL;
                if (error) size = snprintf((char *)reply + 5, REPLY - 5, "%s", error);
                reply_length = 5 + (size < REPLY - 5 ? size : REPLY - 5);
                has_reply = true;
                state = REPLIED;
            }
        }
        if (state != REPLIED) continue;
        send_reply();
        bool reset = reply[4] == 0 && request[4] == 'R';
        portENTER_CRITICAL(&lock); request_state = IDLE; portEXIT_CRITICAL(&lock);
        if (reset) {
            vTaskDelay(pdMS_TO_TICKS(300));
            esp_restart();
        }
    }
}

void radio_start(void) {
    esp_now_peer_info_t peer_info = {.ifidx = WIFI_IF_STA};
    memcpy(peer_info.peer_addr, peer, sizeof(peer));
    session = esp_random() | 1;
    // A slot written over USB with blank OTA data has no state.
    esp_ota_img_states_t image;
    unconfirmed = esp_ota_get_state_partition(esp_ota_get_running_partition(), &image) == ESP_OK &&
                  image == ESP_OTA_IMG_PENDING_VERIFY;
    if (xTaskCreatePinnedToCore(update_loop, "update", 4096, NULL, 2, &update_task, 0) != pdPASS) abort();
    wifi_init_config_t wifi = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&wifi));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // Espressif long range as well as 802.11b/g/n: the controller sends at the 500 kbps long-range rate, which survives a busy channel 6 better (fw/controller/README.md), and both ends hear either.
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_received));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_sent));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    esp_now_rate_config_t rate = usual_rate;
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer, &rate));
    const esp_timer_create_args_t timer_args = {.callback = tick, .name = "radio"};
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 10000));
    printf("Radio on channel %d; reset reason %d\n", WIFI_CHANNEL, esp_reset_reason());
}

bool radio_log_meta(const char *data, size_t length) {
    if (length < 1 || length > META) abort();
    portENTER_CRITICAL(&lock);
    bool sending = meta_sending;
    if (!sending) meta_writing = true;
    portEXIT_CRITICAL(&lock);
    if (sending) return false;
    // tick() starts sending meta only while nobody writes it, and nobody writes it while it is sent.
    memcpy(meta, data, length);
    portENTER_CRITICAL(&lock);
    meta_length = length;
    meta_dirty = true;
    meta_writing = false;
    portEXIT_CRITICAL(&lock);
    return true;
}
