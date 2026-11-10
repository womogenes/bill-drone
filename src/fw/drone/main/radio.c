// Direct ESP-NOW link: status messages, streamed targets, stop, and the request service for updates and transactions. Framing: src/fw/controller/wire.py.
// Channels: 0 status (drone, every 50 ms), 1 targets (laptop, newest wins; each one is the controller heartbeat), 2/3 binary/JSON flight log (one message in flight, resent until acknowledged), 4 recorder subscription, 5 log acknowledgement, 6 requests and replies (id-tagged, fragmented).
#include <math.h>
#include <stddef.h>
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
#include "gps.h"
#include "baro.h"
#include "runtime.h"
#include "arming.h"
#include "camera.h"
#include "../../pairs.h"
#include "../../link.h"

// The paired controller's station MAC (fw/pairs.h); the drone answers nothing else. own is this drone's, its ID.
static uint8_t peer[ESP_NOW_ETH_ALEN], own[ESP_NOW_ETH_ALEN];
// The channel at boot; with radio_follow on, a disarmed drone that hears nothing from its controller for FOLLOW_AFTER_US tries the others in link.h link_channels, FOLLOW_DWELL_US each, and stays where it hears it (the relay beacons its link report every 250 ms), so a controller moved to another channel takes the drone along and one back on 6 brings it back.
#define WIFI_CHANNEL 6
#define FOLLOW_AFTER_US 2000000
#define FOLLOW_DWELL_US 750000
// This radio's counts for the LNK1 report (link.h), the channel it is on and its transmit power (0.25 dBm) as set, and the driver's power from boot, which radio_power 0 keeps.
static LinkCounter link;
static uint8_t channel_now = WIFI_CHANNEL;
static int8_t power_now, power_boot;
static bool following, hopping;
// When any packet from the controller last arrived, stale or not, for following it.
static volatile int64_t peer_us;
static int64_t hopped_us;
static uint32_t radio_version;
// Set once ESP-NOW is up: the update task starts before it.
static volatile bool radio_ready;

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
// Target floats: base, roll, pitch, yaw rate, two servos, two throttles, the hover mode (hover.h), then the hold-point nudge (forward, left, up); older senders leave out the tail.
#define TARGETS 12
// An image that is still unconfirmed this long after boot restarts, so the bootloader rolls back to the previous one.
#define CONFIRM_US 90000000
// Most records per FLG1 batch (controller/log_format.py MAX_BATCH) and the batch header size: a full batch fits two fragments, so one goes out per tick.
#define LOG_MAX 4
#define LOG_HEADER 12
_Static_assert(LOG_HEADER + LOG_MAX * RECORD_SIZE <= 2 * PAYLOAD, "A full log batch fits two fragments");

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
// This drone's usual rate (radio_rate, link.h link_rates) defaults to 1 Mbps, not long range: its 100 Hz log stream needs the airtime, and at 500 kbps retries on a busy channel filled the log ring (10-30% of samples dropped on 2026-10-29); the controller still sends to it at long range, so the targets keep the margin. Outdoors on 2026-11-08 the status stream (this direction) lost 25-77% of its packets beyond 15 m from the laptop, so long range here is the first option to try.
static uint8_t usual_rate;
static const esp_now_rate_config_t fast_rate = {.phymode = WIFI_PHY_MODE_11B, .rate = WIFI_PHY_RATE_11M_L};
static bool fast;
static int64_t fast_until;
// Compressed sector writes (request Y): raw deflate, expanded by the ROM's inflater into a sector.
static tinfl_decompressor inflater;
static uint8_t inflated[SECTOR];

static void set_rate(bool on) {
    esp_now_rate_config_t rate = on ? fast_rate : link_rates[usual_rate];
    if (esp_now_set_peer_rate_config(peer, &rate) == ESP_OK) fast = on;
}

// The radio settings (settings.def radio_rate, radio_power, radio_follow), whenever the settings change; from the update task only.
static void apply_radio_settings(void) {
    if (!radio_ready || radio_version == settings_version()) return;
    radio_version = settings_version();
    int rate = settings_get(SETTING_radio_rate);
    if (rate >= 0 && rate < (int)LINK_RATES && rate != usual_rate) {
        usual_rate = rate;
        if (!fast) set_rate(false);
    }
    int power = settings_get(SETTING_radio_power) * 4;
    int8_t wanted = power ? (power < 8 ? 8 : power > 84 ? 84 : power) : power_boot;
    if (wanted != power_now && esp_wifi_set_max_tx_power(wanted) == ESP_OK) esp_wifi_get_max_tx_power(&power_now);
    following = settings_get(SETTING_radio_follow) > 0.5f;
}

// radio_follow: while disarmed and not hearing the controller, step through link_channels until it is heard again.
static void follow(int64_t now) {
    hopping = radio_ready && following && flight_runtime_idle() && now - peer_us > FOLLOW_AFTER_US;
    if (!hopping || now - hopped_us < FOLLOW_DWELL_US) return;
    size_t index = 0;
    while (index < LINK_CHANNELS && link_channels[index] != channel_now) index++;
    uint8_t next = link_channels[(index + 1) % LINK_CHANNELS];
    if (esp_wifi_set_channel(next, WIFI_SECOND_CHAN_NONE) == ESP_OK) channel_now = next;
    hopped_us = now;
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

// The radio quiet test (request M): nothing is transmitted until this time (esp_timer µs), while the GPS's own polls of its RF health and satellites (gps.c, every second) collect in the tunnel, so their readings with the drone's transmitter silent can be compared.
static volatile int64_t quiet_until;

static void quiet_task(void *arg) {
    int seconds = (int)(intptr_t)arg;
    // The tunnel keeps frames through the silence and 5 s after it, for the N that takes them.
    gps_tunnel_hold(seconds * 1000 + 5300);
    // Time for this request's reply to leave first.
    vTaskDelay(pdMS_TO_TICKS(300));
    quiet_until = esp_timer_get_time() + seconds * 1000000LL;
    vTaskDelete(NULL);
}

// The GPS report (gps_report) goes out once a second while the laptop's recorder subscription asks for it ('G' after its first byte, bridge.py since 2026-11-09), so a bridge that would read it as a status never gets one.
static int64_t gps_report_us = INT64_MIN / 2;
static bool gps_report_due;

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
    if (esp_timer_get_time() < quiet_until) return false;
    portENTER_CRITICAL(&lock);
    bool open = inflight < WINDOW;
    if (open) inflight++;
    portEXIT_CRITICAL(&lock);
    if (!open) {
        link_refused(&link);
        return false;
    }
    // The send callback may run before esp_now_send returns, so the slot is taken first.
    link_sending(&link, esp_timer_get_time());
    bool ok = esp_now_send(peer, packet, HEADER + length) == ESP_OK;
    if (!ok) {
        link_unsent(&link);
        portENTER_CRITICAL(&lock);
        inflight--;
        portEXIT_CRITICAL(&lock);
    }
    return ok;
}

static void on_sent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    link_sent(&link, esp_timer_get_time(), status == ESP_NOW_SEND_SUCCESS);
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
        memcmp(packet, "DC1", 3) || (packet[3] >= CHANNELS && packet[3] != LINK_CHANNEL) || packet[17] > 32 || packet[16] >= packet[17])
        return;
    uint8_t channel = packet[3], index = packet[16], count = packet[17];
    uint32_t packet_session, number, token;
    memcpy(&packet_session, packet + 4, 4);
    memcpy(&number, packet + 8, 4);
    memcpy(&token, packet + 12, 4);
    const uint8_t *data = packet + HEADER;
    size_t length = size - HEADER;
    int64_t now = esp_timer_get_time();
    peer_us = now;
    // Channel 5 acknowledgements carry this drone's numbers and channel 7 the relay's own, so only the rest count the laptop's gaps.
    link_heard(&link, now, info, number, channel != 5 && channel != LINK_CHANNEL);
    // The relay's beacon (its link report) only keeps a following drone on this channel.
    if (channel == LINK_CHANNEL) return;
    int32_t age = (int32_t)(now_ms() - token);
    if (packet_session != session || age < 0 || age > 200) {
        link_rejected(&link);
        return;
    }
    heard_us = now;
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
        if (channel == 4 && length >= 2 && data[1] == 'G') gps_report_us = now;
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
    // The thirteen counters, packets to rx_edges, are in the same order in both.
    _Static_assert(offsetof(CameraStats, rx_edges) == 12 * sizeof(uint32_t), "CameraStats counters");
    _Static_assert(offsetof(Status, camera_rx_edges) - offsetof(Status, camera_packets) == 12 * sizeof(uint32_t), "Status camera counters");
    memcpy((uint8_t *)status + offsetof(Status, camera_packets), &camera.packets, 13 * sizeof(uint32_t));
    status->camera_sequence = camera.sequence;
    status->camera_age_ms = camera.frame_us < 0 ? UINT32_MAX : (now - camera.capture_us) / 1000;
    status->camera_valid = camera.valid;
    status->camera_trusted = camera.trusted;
}

static void gps_status(Status *status, int64_t now) {
    GpsStats gps;
    gps_stats(&gps);
    const UbxPvt *pvt = &gps.fix.pvt;
    status->gps_messages = gps.messages;
    status->gps_corrupt = gps.corrupt;
    status->gps_skipped = gps.skipped;
    status->gps_age_ms = gps.fix.received_us < 0 ? UINT32_MAX : (now - gps.fix.received_us) / 1000;
    status->gps_itow_ms = pvt->itow_ms;
    status->gps_fix = pvt->fix_type;
    status->gps_flags = pvt->flags;
    status->gps_satellites = pvt->satellites;
    status->gps_configured = gps.configured;
    status->gps_latitude = pvt->latitude;
    status->gps_longitude = pvt->longitude;
    status->gps_height_mm = pvt->height_mm;
    memcpy(status->gps_velocity, pvt->velocity_mm_s, sizeof(status->gps_velocity));
    status->gps_horizontal_accuracy_mm = pvt->horizontal_accuracy_mm;
    status->gps_vertical_accuracy_mm = pvt->vertical_accuracy_mm;
    status->gps_speed_accuracy = pvt->speed_accuracy_mm_s;
}

static void baro_status(Status *status, int64_t now) {
    BaroStats baro;
    baro_stats(&baro);
    bool fresh = baro.sample.read_us >= 0 && now - baro.sample.read_us <= 200000;
    float altitude = fminf(fmaxf(baro.sample.altitude * 10, INT16_MIN), INT16_MAX);
    status->baro_altitude_dm = fresh ? lroundf(altitude) : 0;
    status->baro_flags = (baro.address != 0) | fresh << 1 | (baro.address == 0x77) << 2;
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
        int64_t now = esp_timer_get_time();
        // Between status messages; a refused send (the window full) is retried on the next tick.
        portENTER_CRITICAL(&lock);
        if (ticks % 100 == 3) gps_report_due = now - gps_report_us < 3000000;
        bool gps_due = gps_report_due;
        portEXIT_CRITICAL(&lock);
        if (gps_due) {
            uint8_t message[PAYLOAD];
            size_t length = gps_report(message, sizeof(message), now);
            if (!length || transmit(0, next_sequence(), message, length, 0, 1)) gps_report_due = false;
        }
        // The link report once a second (link.h), on a tick between status messages; a refused one goes out on a later tick.
        static LinkReport report;
        static bool report_pending;
        if (ticks % 100 == 2) {
            link_report(&link, now, "LNK1", &report);
            report.channel = channel_now;
            report.rate = fast ? 5 : usual_rate;
            report.power = power_now;
            report.flags = following | hopping << 1;
            report_pending = true;
        }
        if (report_pending && transmit(LINK_CHANNEL, next_sequence(), (const uint8_t *)&report, sizeof(report), 0, 1)) report_pending = false;
        stream_log(now);
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
    gps_status(&status, now);
    baro_status(&status, now);
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

// Scratch space for request error messages.
static char error[96];

// Changes all of the values, or none and says why.
static const char *change_settings(const int *indices, const float *values, int count, int64_t now) {
    for (int i = 0; i < count; i++) {
        const char *failure = settings_validate(indices[i], values[i], error, sizeof(error));
        if (failure) return failure;
    }
    settings_change(indices, values, count, now);
    return NULL;
}

// Requests: A arm, O record while disarmed (1) or stop recording (0), U raw bytes to the camera's wire (recovery), J reset the camera (1: into download mode), I info, W write sector, C copy sector from the running slot, B boot, V confirm, R reset, S read settings, T tune (JSON object), D schema page, G calibrate IMU, Z zero a servo, X servo bench test (start or read), P load the test pattern (flight_runtime_pattern), Q set the accelerometer correction (offset x, y, z then gain x, y, z as float32), Y write a deflate-compressed sector, F the fast update rate on (1) or off (0), K a camera firmware update request (its op, then its body; camera_update), M the radio quiet test, N the GPS tunnel (gps_send, gps_take). Fills reply after the id and status bytes; returns an error or NULL.
static const char *handle(uint8_t op, const uint8_t *body, size_t length, size_t *size) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    char *text = (char *)reply + 5;
    if (op == 'S' || op == 'D') {
        *size = op == 'S' ? settings_json(text, REPLY - 5) : settings_schema(text, REPLY - 5, length ? body[0] : 0);
        return *size ? NULL : "Settings do not fit the reply";
    }
    if (op == 'E') {
        // ESC calibration (props off): the stage byte, as flight_runtime_esc_calibration takes it.
        if (length < 1) return "Malformed ESC calibration request";
        return flight_runtime_esc_calibration(body[0]);
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
        const char *failure = NULL;
        if (op == 'T') {
            failure = settings_tune((const char *)body, length, now, error, sizeof(error));
        } else if (op == 'G') {
            failure = flight_runtime_calibrate();
        } else if (op == 'L') {
            if (length != 8) return "Expected two float32 values";
            // A new zero pose makes the old pitch trim meaningless.
            int indices[] = {SETTING_pose_zero_roll, SETTING_pose_zero_pitch, SETTING_pitch_trim};
            float values[3] = {0};
            memcpy(values, body, 8);
            failure = change_settings(indices, values, 3, now);
        } else if (op == 'Q') {
            if (length != 24) return "Expected six float32 values";
            int indices[6];
            float values[6];
            memcpy(values, body, sizeof(values));
            for (int i = 0; i < 6; i++) indices[i] = (i < 3 ? SETTING_accel_offset_x : SETTING_accel_gain_x) + i % 3;
            failure = change_settings(indices, values, 6, now);
        } else {
            if (length != 1 || body[0] > 1) return "Expected side 0 or 1";
            float offset;
            int index = body[0] ? SETTING_servo_offset_right : SETTING_servo_offset_left;
            failure = flight_runtime_zero_servo(body[0], &offset, error, sizeof(error));
            if (!failure) settings_change(&index, &offset, 1, now);
        }
        if (failure) return failure;
        flight_runtime_apply_settings();
        *size = settings_json(text, REPLY - 5);
        return NULL;
    }
    if (op == 'K') {
        if (!length) return "Expected a camera request";
        // Only what is safe in flight passes while armed: the tag preview over the radio (P), status (S), slots (I) and log (L). Writing, booting or restarting the camera (W, B, X), its access point (A) and FPV (V, which stops tag tracking) wait for the drone to be disarmed with zero throttle. All of them used to, which stopped the dashboard's preview in hover (2026-10-31).
        if (!flight_runtime_idle() && !strchr("PSIL", body[0])) return "Disable control and zero throttle first: only P, S, I and L reach the camera in flight";
        // Selecting the new image reads all of it to check its hash.
        const char *failure = camera_update(body[0], body + 1, length - 1, body[0] == 'B' ? 5000 : 2000, text, REPLY - 5);
        if (failure) return failure;
        *size = strlen(text);
        return NULL;
    }
    if (op == 'J') {
        // Camera reset through its EN line (1: into the ROM's download mode), only disarmed: a hung camera came back only from a power cycle by hand (2026-10-31).
        if (!flight_runtime_idle()) return "Disarm and zero throttle first";
        // 2, pin, ms: pull one free pin low instead, to find which one the camera's EN or IO0 is wired to.
        if (length >= 3 && body[0] == 2) return camera_probe_pin(body[1], body[2] * 10) ? NULL : "Not a free pin";
        // 3: the camera restarts itself with IO0 held low (no EN wire needed), toward the ROM's download mode.
        if (length >= 1 && body[0] == 3) {
            camera_restart_holding_io0(500);
            return NULL;
        }
        // 4: hold the camera in reset (off) until 5 releases it or the drone restarts; anything else the reset pulse.
        if (length >= 1 && (body[0] == 4 || body[0] == 5)) {
            camera_hold(body[0] == 4);
            *size = snprintf(text, REPLY - 5, "camera %s", body[0] == 4 ? "held in reset" : "released");
            return NULL;
        }
        camera_reset(length >= 1 && body[0] == 1);
        return NULL;
    }
    if (op == 'U') {
        // Raw bytes to and from the camera's wire (camera_raw), only disarmed: baud (u32), wait ms (u16), then the bytes. For waking a camera stuck in its ROM bootloader (2026-10-31: silent from 01:23 after line errors, no remote power switch).
        if (!flight_runtime_idle()) return "Disarm and zero throttle first";
        if (length < 6) return "Malformed raw camera request";
        uint32_t baud = body[0] | body[1] << 8 | body[2] << 16 | (uint32_t)body[3] << 24;
        uint16_t wait = body[4] | body[5] << 8;
        if (baud < 9600 || baud > 2000000 || wait > 3000) return "Raw camera request out of range";
        *size = camera_raw(baud, body + 6, length - 6, wait, (uint8_t *)text, REPLY - 5);
        return NULL;
    }
    if (op == 'M') {
        // The radio quiet test: seconds (1-15) without transmitting, only disarmed, the GPS's MON-RF and NAV-SAT every second into the tunnel (take them with N after; its 4 KB holds about 6 s of them).
        if (!flight_runtime_idle()) return "Disarm and zero throttle first";
        if (length < 1 || body[0] < 1 || body[0] > 15) return "Seconds 1-15";
        if (xTaskCreatePinnedToCore(quiet_task, "quiet", 3072, (void *)(intptr_t)body[0], 2, NULL, 0) != pdPASS) return "No memory for the quiet test";
        *size = snprintf(text, REPLY - 5, "quiet for %d s", body[0]);
        return NULL;
    }
    if (op == 'N') {
        // The GPS tunnel: the body's bytes go to the module (only disarmed: a configuration could change what the hover's fusion gets), and the reply is the frames other than NAV-PVT received since the last N, after a u32 count of frames dropped for want of room.
        if (length && !flight_runtime_idle()) return "Disarm and zero throttle first";
        if (length) gps_send(body, length);
        uint32_t dropped;
        size_t taken = gps_take((uint8_t *)text + 4, REPLY - 9, &dropped);
        memcpy(text, &dropped, 4);
        *size = 4 + taken;
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
    if (op == 'O') {
        // A recording while disarmed (1) or its end (0), as arming's needs the recorder and the previous one closed.
        int64_t now = esp_timer_get_time();
        portENTER_CRITICAL(&lock);
        bool recorder = subscribed(now) && !session_open;
        portEXIT_CRITICAL(&lock);
        return length >= 1 ? flight_runtime_record(body[0], recorder) : "Malformed record request";
    }
    if (op == 'I') {
        char running_sha[24], target_sha[24];
        describe(running_sha, sizeof(running_sha), running);
        describe(target_sha, sizeof(target_sha), target);
        // reset is esp_reset_reason_t: 1 power-on, 3 software, 4 panic, 5-7 watchdogs, 9 brownout.
        *size = snprintf(text, REPLY - 5, "{\"drone\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"running\":\"%s\",\"target\":%s%s%s,\"size\":%lu,\"running_sha\":%s,\"target_sha\":%s,\"reset\":%d}",
                         own[0], own[1], own[2], own[3], own[4], own[5], running->label, target ? "\"" : "", target ? target->label : "null", target ? "\"" : "",
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
        apply_radio_settings();
        follow(esp_timer_get_time());
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
    pair_find(true, own, peer);
    esp_now_peer_info_t peer_info = {.ifidx = WIFI_IF_STA};
    memcpy(peer_info.peer_addr, peer, sizeof(peer));
    session = esp_random() | 1;
    link_init(&link, esp_timer_get_time());
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
    esp_now_rate_config_t rate = link_rates[usual_rate];
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer, &rate));
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power_boot));
    power_now = power_boot;
    radio_ready = true;
    const esp_timer_create_args_t timer_args = {.callback = tick, .name = "radio"};
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 10000));
    printf("Radio on channel %d, transmit power %d (0.25 dBm); reset reason %d\n", WIFI_CHANNEL, power_boot, esp_reset_reason());
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
