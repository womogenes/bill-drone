#pragma once
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
extern bool cameraReady;
extern volatile bool cameraStopRequested;
#if __has_include("wifi.h")
#include "wifi.h"
#else
#include "wifi.example.h"
#endif

inline void startWifi() {
    // Nothing is printed: UART0 is the drone link. Without wifi.h there is no Wi-Fi and no OTA.
    if (!wifiSsid[0])
        return;
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("bill-esp32cam");
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.begin(wifiSsid, wifiPassword);
    // Full transmit power (about 20 dBm) draws current bursts that browned this board out on its 3.3 V regulator (2026-10-26, "Brownout detector was triggered" at Wi-Fi start); 8.5 dBm keeps the bursts smaller and still links at about -46 dBm received indoors.
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
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
    if (!wifiSsid[0]) return;
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - lastRetry >= 10000) {
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
