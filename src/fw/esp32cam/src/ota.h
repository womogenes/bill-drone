#pragma once
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "esp_wifi.h"
#include "esp_camera.h"
extern bool cameraReady;
extern volatile bool cameraStopRequested;
struct WifiNetwork {
    const char *ssid, *password;
};
#if __has_include("wifi.h")
#include "wifi.h"
#else
#include "wifi.example.h"
#endif
// Away from every known network the station joins nothing and parks on radioChannel, clear of the drone's ESP-NOW link on channel 6, ready for FPV's raw frames (fpv.h). The access point (apSsid) starts there only when asked (the drone's K A request, update.h), for Wi-Fi updates, until the camera restarts: switching between the two reallocated the Wi-Fi driver's internal RAM, and JPEG capture then lacked its 32 KB block of DMA memory (2026-10-30, 12 bytes short), and with the access point up no raw frame went out.
constexpr int radioChannel = 11;
constexpr uint32_t parkAfterMs = 30000;
const WifiNetwork *wifiChosen = nullptr;
bool apStarted = false, parked = false;

inline bool wifiConfigured() { return wifiNetworks[0].ssid[0] || apSsid[0]; }

// The network joined, the access point or the parked station, its address, and the camera's state, as the drone's K S request reports them (update.h).
inline void wifiState(char *text, size_t size) {
    uint8_t channel = 0;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&channel, &second);
    snprintf(text, size, "%s %s %s ch%u", apStarted ? "ap" : parked ? "radio" : WiFi.status() == WL_CONNECTED ? "joined" : wifiChosen ? "joining" : "off",
             apStarted ? apSsid : wifiChosen ? wifiChosen->ssid : "-", (apStarted ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(), unsigned(channel));
}

inline void park() {
    wifiChosen = nullptr;
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA);
    // The driver refuses a channel while the station is still connecting ("sta is connecting"), and the camera then sent on its network's channel.
    esp_wifi_disconnect();
    for (int tries = 0; tries < 50 && esp_wifi_set_channel(radioChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK; ++tries) {
        esp_wifi_disconnect();
        delay(20);
    }
    apStarted = false;
    parked = true;
}

inline void startAp() {
    if (!apSsid[0] || apStarted)
        return;
    WiFi.mode(WIFI_AP);
    apStarted = WiFi.softAP(apSsid, apPassword[0] ? apPassword : nullptr, radioChannel);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    parked = !apStarted;
}

inline void startWifi() {
    // Nothing is printed: UART0 is the drone link. Without wifi.h there is no Wi-Fi and no OTA.
    if (!wifiConfigured())
        return;
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("bill-esp32cam");
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    // Full transmit power (about 20 dBm) draws current bursts that browned this board out on its 3.3 V regulator (2026-10-26, "Brownout detector was triggered" at Wi-Fi start); 8.5 dBm keeps the bursts smaller and still links at about -46 dBm received indoors.
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    // One scan at boot picks the first listed network in range.
    const int found = wifiNetworks[0].ssid[0] ? WiFi.scanNetworks() : 0;
    for (const WifiNetwork &network : wifiNetworks)
        for (int i = 0; i < found && !wifiChosen; ++i)
            if (WiFi.SSID(i) == network.ssid)
                wifiChosen = &network;
    WiFi.scanDelete();
    if (wifiChosen)
        WiFi.begin(wifiChosen->ssid, wifiChosen->password);
    else
        park();
    ArduinoOTA.setHostname("bill-esp32cam");
    // No OTA password, as requested. Wi-Fi credentials are separate.
    ArduinoOTA.setTimeout(10000);
    ArduinoOTA.onStart([]() {
        // The detection loop stops the camera after its current frame and keeps it stopped until the restart.
        cameraStopRequested = true;
        while (cameraReady) delay(10);
    });
    ArduinoOTA.onError([](ota_error_t) { ESP.restart(); });
}

inline void serviceOta() {
    static bool started = false;
    static uint32_t lastRetry = 0;
    static bool joined = false;
    if (!wifiChosen && !apStarted) return;
    joined |= WiFi.status() == WL_CONNECTED;
    // A network in range that never lets the camera join (a changed password) parks the station too.
    if (!joined && wifiChosen && millis() > parkAfterMs) {
        park();
        return;
    }
    if (!apStarted && WiFi.status() != WL_CONNECTED) {
        // The core's auto-reconnect retries most losses at once; this catches the rest (a failed association, a missing access point) every 5 s, which leaves an association in progress time to finish.
        if (millis() - lastRetry >= 5000) {
            lastRetry = millis();
            WiFi.reconnect();
        }
        return;
    }
    if (!started) {
        ArduinoOTA.begin();
        started = true;
    }
    ArduinoOTA.handle();
}
