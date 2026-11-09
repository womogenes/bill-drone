// Radio link quality and options shared by the drone (fw/drone/main/radio.c) and the controller's relay (fw/controller/native/main/relay.c): the once-a-second link report each sends on channel 7, the PHY rates either can send at, and the channels the drone may follow its controller to. The laptop decodes the reports in fw/controller/bridge.py (LINK_FORMAT).
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_now.h"
#include "freertos/FreeRTOS.h"

// Channel 7 carries link reports: LNK1 from the drone, RLY1 from the relay (to the laptop over USB, and to the drone as its beacon), and RCF1 from the laptop to the relay alone (radio settings, never forwarded). Firmware from before 2026-11-09 drops channel 7: the drone and wire.unpack accept channels 0-6, the relay forwards nothing from USB above 6.
#define LINK_CHANNEL 7
#define LINK_REPORT_US 1000000

// The rates a radio_rate setting (drone) or RCF1 rate (relay) picks, by index; 0 is the drone's usual 1 Mbps and 1 the relay's usual 500 kbps long range (both since 2026-10-29), 5 the 11 Mbps update rate (request F).
static const esp_now_rate_config_t link_rates[] = {
    {.phymode = WIFI_PHY_MODE_11B, .rate = WIFI_PHY_RATE_1M_L},
    {.phymode = WIFI_PHY_MODE_LR, .rate = WIFI_PHY_RATE_LORA_500K},
    {.phymode = WIFI_PHY_MODE_LR, .rate = WIFI_PHY_RATE_LORA_250K},
    {.phymode = WIFI_PHY_MODE_11B, .rate = WIFI_PHY_RATE_2M_L},
    {.phymode = WIFI_PHY_MODE_11B, .rate = WIFI_PHY_RATE_5M_L},
    {.phymode = WIFI_PHY_MODE_11B, .rate = WIFI_PHY_RATE_11M_L},
    {.phymode = WIFI_PHY_MODE_11G, .rate = WIFI_PHY_RATE_6M},
};
#define LINK_RATES (sizeof(link_rates) / sizeof(link_rates[0]))

// The 2.4 GHz channels that do not overlap; the drone tries them in this order while it follows its controller (radio_follow). The camera's FPV video uses 11 (fw/esp32cam/README.md).
static const uint8_t link_channels[] = {6, 1, 11};
#define LINK_CHANNELS (sizeof(link_channels) / sizeof(link_channels[0]))

// One interval's counts, as sent. rx: packets from the partner (MAC-matched, header-valid); rejected: of those, the drone's refused as stale or from another session, the relay's USB frames dropped (bad frame or a full queue); gaps: the partner's sequence numbers that never arrived (the laptop's on the drone, the drone's on the relay; numbers the sender used without transmitting count too, see refused); silence_ms: the longest time without a partner packet, including the one still running; RSSI of the partner's packets (mean and lowest, dBm, -128 with none) and the receiver's noise floor; tx_ok and tx_fail: the send callbacks (fail: no acknowledgement after the MAC's retries); refused: sends that never went out (window full or esp_now_send failing); latency_ms: the longest send-to-callback time; then the radio's channel, rate index (link_rates), maximum transmit power (0.25 dBm) and flags (drone: 1 radio_follow on, 2 hopping channels).
typedef struct __attribute__((packed)) {
    char magic[4];
    uint16_t interval_ms, rx, rejected, gaps, silence_ms;
    int8_t rssi_mean, rssi_min, noise_floor;
    uint16_t tx_ok, tx_fail, refused, latency_ms;
    uint8_t channel, rate;
    int8_t power;
    uint8_t flags;
} LinkReport;
_Static_assert(sizeof(LinkReport) == 29, "LinkReport layout");

// Counts for one LinkReport (unpacked: the report's packed fields would be unaligned), written from the Wi-Fi task's callbacks and the senders; every function takes the lock.
typedef struct {
    uint32_t rx, rejected, gaps, tx_ok, tx_fail, refused;
    int64_t silence_us, latency_us, rssi_sum;
    int rssi_min, noise_floor;
} LinkCounts;

typedef struct {
    portMUX_TYPE lock;
    LinkCounts counts;
    int64_t start_us, heard_us;
    uint32_t sequence;
    bool sequenced;
    // When each send still awaiting its callback was handed to ESP-NOW, oldest first (callbacks come in send order for one peer).
    int64_t sent_us[8];
    uint8_t sent_first, sent_count;
} LinkCounter;

static inline void link_clear(LinkCounter *link) {
    memset(&link->counts, 0, sizeof(link->counts));
    link->counts.rssi_min = 127;
}

static inline void link_init(LinkCounter *link, int64_t now) {
    memset(link, 0, sizeof(*link));
    link->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    link_clear(link);
    link->start_us = link->heard_us = now;
}

static inline uint16_t link_saturate(int64_t value) {
    return value < 0 ? 0 : value > UINT16_MAX ? UINT16_MAX : value;
}

// A packet from the partner; sequenced says whether number belongs to the partner's one counter (repeats and older numbers are not gaps).
static inline void link_heard(LinkCounter *link, int64_t now, const esp_now_recv_info_t *info, uint32_t number, bool sequenced) {
    portENTER_CRITICAL(&link->lock);
    if (now - link->heard_us > link->counts.silence_us) link->counts.silence_us = now - link->heard_us;
    link->heard_us = now;
    link->counts.rx++;
    if (info && info->rx_ctrl) {
        int rssi = info->rx_ctrl->rssi;
        link->counts.rssi_sum += rssi;
        if (rssi < link->counts.rssi_min) link->counts.rssi_min = rssi;
        link->counts.noise_floor = info->rx_ctrl->noise_floor;
    }
    if (sequenced) {
        uint32_t step = number - link->sequence;
        if (link->sequenced && step > 1 && step < 0x10000) link->counts.gaps += step - 1;
        if (!link->sequenced || (step && step < 0x80000000U)) link->sequence = number;
        link->sequenced = true;
    }
    portEXIT_CRITICAL(&link->lock);
}

static inline void link_rejected(LinkCounter *link) {
    portENTER_CRITICAL(&link->lock);
    link->counts.rejected++;
    portEXIT_CRITICAL(&link->lock);
}

static inline void link_refused(LinkCounter *link) {
    portENTER_CRITICAL(&link->lock);
    link->counts.refused++;
    portEXIT_CRITICAL(&link->lock);
}

// Just before esp_now_send: its callback can run before the call returns.
static inline void link_sending(LinkCounter *link, int64_t now) {
    portENTER_CRITICAL(&link->lock);
    if (link->sent_count < 8) link->sent_us[(link->sent_first + link->sent_count++) % 8] = now;
    portEXIT_CRITICAL(&link->lock);
}

// esp_now_send refused the packet link_sending announced.
static inline void link_unsent(LinkCounter *link) {
    portENTER_CRITICAL(&link->lock);
    if (link->sent_count) link->sent_count--;
    link->counts.refused++;
    portEXIT_CRITICAL(&link->lock);
}

static inline void link_sent(LinkCounter *link, int64_t now, bool delivered) {
    portENTER_CRITICAL(&link->lock);
    if (delivered) link->counts.tx_ok++;
    else link->counts.tx_fail++;
    if (link->sent_count) {
        int64_t latency = now - link->sent_us[link->sent_first];
        if (latency > link->counts.latency_us) link->counts.latency_us = latency;
        link->sent_first = (link->sent_first + 1) % 8;
        link->sent_count--;
    }
    portEXIT_CRITICAL(&link->lock);
}

// The interval's report (the radio fields are the caller's), and a fresh interval.
static inline void link_report(LinkCounter *link, int64_t now, const char *magic, LinkReport *report) {
    portENTER_CRITICAL(&link->lock);
    LinkCounts counts = link->counts;
    int64_t silence = now - link->heard_us, interval = now - link->start_us;
    link_clear(link);
    link->start_us = now;
    portEXIT_CRITICAL(&link->lock);
    if (counts.silence_us > silence) silence = counts.silence_us;
    memset(report, 0, sizeof(*report));
    memcpy(report->magic, magic, 4);
    report->interval_ms = link_saturate(interval / 1000);
    report->rx = link_saturate(counts.rx);
    report->rejected = link_saturate(counts.rejected);
    report->gaps = link_saturate(counts.gaps);
    report->silence_ms = link_saturate(silence / 1000);
    report->rssi_mean = counts.rx ? counts.rssi_sum / (int64_t)counts.rx : -128;
    report->rssi_min = counts.rx ? counts.rssi_min : -128;
    report->noise_floor = counts.noise_floor;
    report->tx_ok = link_saturate(counts.tx_ok);
    report->tx_fail = link_saturate(counts.tx_fail);
    report->refused = link_saturate(counts.refused);
    report->latency_ms = link_saturate(counts.latency_us / 1000);
}
