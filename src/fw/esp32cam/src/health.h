#pragma once
#include <Preferences.h>
#include <WiFi.h>
#include "esp_system.h"

// Reset and Wi-Fi history for /status. Each boot's reset reason is counted in NVS, so a power loss (reason 1, which also scrambles the RTC crumbs), a brownout (9), a panic (4) or a watchdog (5 to 7) stays visible across later resets. The Wi-Fi side counts disconnections since boot, with the last reason (ESP-IDF wifi_err_reason_t) and when it happened.
namespace health {
constexpr int reasonSlots = 16;
uint32_t resets[reasonSlots] = {};
uint32_t boots = 0;
int resetReason = 0;
volatile uint32_t wifiDrops = 0, wifiConnects = 0, lastDropMs = 0, lastConnectMs = 0;
volatile uint16_t lastDropReason = 0;

void countBoot() {
    resetReason = int(esp_reset_reason());
    Preferences store;
    if (!store.begin("health", false))
        return;
    store.getBytes("resets", resets, sizeof(resets));
    boots = store.getUInt("boots", 0) + 1;
    store.putUInt("boots", boots);
    ++resets[resetReason >= 0 && resetReason < reasonSlots ? resetReason : 0];
    store.putBytes("resets", resets, sizeof(resets));
    store.end();
}

void watchWifi() {
    WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t info) {
        ++wifiDrops;
        lastDropReason = info.wifi_sta_disconnected.reason;
        lastDropMs = millis();
    }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t) {
        ++wifiConnects;
        lastConnectMs = millis();
    }, ARDUINO_EVENT_WIFI_STA_GOT_IP);
}

// JSON members (no braces) for /status: the reset history by esp_reset_reason_t and the Wi-Fi link.
int status(char *out, size_t size) {
    return snprintf(out, size,
                    "\"boots\":%u,\"brownouts\":%u,\"power_ons\":%u,\"panics\":%u,\"watchdogs\":%u,\"resets_by_reason\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
                    "\"rssi\":%d,\"wifi_connects\":%u,\"wifi_drops\":%u,\"wifi_last_drop_reason\":%u,\"wifi_last_drop_ms\":%u,\"wifi_last_connect_ms\":%u,",
                    unsigned(boots), unsigned(resets[ESP_RST_BROWNOUT]), unsigned(resets[ESP_RST_POWERON]), unsigned(resets[ESP_RST_PANIC]),
                    unsigned(resets[ESP_RST_INT_WDT] + resets[ESP_RST_TASK_WDT] + resets[ESP_RST_WDT]), unsigned(resets[0]), unsigned(resets[1]),
                    unsigned(resets[2]), unsigned(resets[3]), unsigned(resets[4]), unsigned(resets[5]), unsigned(resets[6]), unsigned(resets[7]),
                    unsigned(resets[8]), unsigned(resets[9]), unsigned(resets[10]), int(WiFi.RSSI()), unsigned(wifiConnects), unsigned(wifiDrops),
                    unsigned(lastDropReason), unsigned(lastDropMs), unsigned(lastConnectMs));
}
}
