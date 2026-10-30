#pragma once
#include <WiFi.h>
#include <ArduinoOTA.h>
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
// The access point's channel, clear of the drone's ESP-NOW link on channel 6.
constexpr int apChannel = 11;
constexpr uint32_t apAfterMs = 30000;
const WifiNetwork *wifiChosen = nullptr;
bool apStarted = false;

inline bool wifiConfigured() { return wifiNetworks[0].ssid[0] || apSsid[0]; }

// The network joined or the access point, its address, and the camera's state, as the drone's K S request reports them (update.h).
inline void wifiState(char *text, size_t size) {
    snprintf(text, size, "%s %s %s", apStarted ? "ap" : WiFi.status() == WL_CONNECTED ? "joined" : wifiChosen ? "joining" : "off",
             apStarted ? apSsid : wifiChosen ? wifiChosen->ssid : "-", (apStarted ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str());
}

// Without a known network the camera is its own access point until it restarts: retrying networks would move its radio off the access point's channel.
inline void startAp() {
    WiFi.mode(WIFI_AP);
    apStarted = WiFi.softAP(apSsid, apPassword[0] ? apPassword : nullptr, apChannel);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
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
    if (wifiChosen) {
        WiFi.begin(wifiChosen->ssid, wifiChosen->password);
    } else if (apSsid[0]) {
        startAp();
    }
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
    // A network in range that never lets the camera join (a changed password) falls back to the access point too.
    if (!joined && !apStarted && apSsid[0] && millis() > apAfterMs) {
        startAp();
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
