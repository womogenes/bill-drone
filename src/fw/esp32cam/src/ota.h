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
    if (!wifiSsid[0]) {
        Serial.println("OTA disabled: copy wifi.example.h to wifi.h and set the network.");
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("bill-esp32cam");
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.begin(wifiSsid, wifiPassword);
    ArduinoOTA.setHostname("bill-esp32cam");
    // No OTA password, as requested. Wi-Fi credentials are separate.
    ArduinoOTA.setTimeout(10000);
    ArduinoOTA.onStart([]() {
        // The detection loop stops the camera after its current frame and keeps it stopped until the restart.
        cameraStopRequested = true;
        while (cameraReady) delay(10);
        Serial.println("OTA update started; camera stopped.");
    });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA error: %u; restarting camera.\n", error); ESP.restart(); });
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
        Serial.print("OTA ready at ");
        Serial.println(WiFi.localIP());
    }
    ArduinoOTA.handle();
}
