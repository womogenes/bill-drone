#pragma once
#include <WebServer.h>
#include <Preferences.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include "cJSON.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "config.h"
#include "packet.h"

WebServer web(80);
Preferences settings;
SemaphoreHandle_t previewMutex;
constexpr size_t previewCapacity = 128 * 1024;
uint8_t *previewJpeg;
size_t previewLength = 0;
volatile bool previewActive = false;
uint32_t previewSequence = 0;
volatile uint32_t captureCount = 0, poseCount = 0, previewCount = 0;
PosePacket lastPose;

bool calibrated() { return config::fx > 0 && config::fy > 0 && config::tagSizeM > 0; }

bool applyCalibration(const String &body) {
    cJSON *json = cJSON_Parse(body.c_str());
    if (!json)
        return false;
    const char *names[] = {"fx", "fy", "cx", "cy", "tag_size_mm", "tag_id"};
    float values[6];
    bool valid = true;
    for (int i = 0; i < 6; ++i) {
        cJSON *value = cJSON_GetObjectItemCaseSensitive(json, names[i]);
        values[i] = cJSON_IsNumber(value) ? value->valuedouble : NAN;
        valid &= isfinite(values[i]);
    }
    cJSON *array = cJSON_GetObjectItemCaseSensitive(json, "distortion");
    float distortion[5];
    valid &= cJSON_IsArray(array) && cJSON_GetArraySize(array) == 5;
    for (int i = 0; i < 5; ++i) {
        cJSON *value = cJSON_GetArrayItem(array, i);
        distortion[i] = cJSON_IsNumber(value) ? value->valuedouble : NAN;
        valid &= isfinite(distortion[i]);
    }
    valid &= values[0] > 0 && values[1] > 0 && values[4] > 0 && values[5] >= 0 && values[5] < 30 &&
             values[5] == floorf(values[5]);
    if (valid) {
        config::fx = values[0];
        config::fy = values[1];
        config::cx = values[2];
        config::cy = values[3];
        config::tagSizeM = values[4] / 1000;
        config::tagId = int(values[5]);
        memcpy(config::distortion, distortion, sizeof(distortion));
    }
    cJSON_Delete(json);
    return valid;
}

void loadCalibration() {
    // Lens calibration must match the upright sensor orientation.
    assert(settings.begin("camera16up", false));
    String saved = settings.isKey("calibration") ? settings.getString("calibration") : "";
    if (saved.length())
        assert(applyCalibration(saved));
}

// The detector never waits for video: one latest-frame slot, with a try-lock.
void publishPreview(camera_fb_t *frame) {
    if (!previewActive || frame->len > previewCapacity || xSemaphoreTake(previewMutex, 0) != pdTRUE)
        return;
    memcpy(previewJpeg, frame->buf, frame->len);
    previewLength = frame->len;
    ++previewSequence;
    xSemaphoreGive(previewMutex);
}

esp_err_t streamPreview(httpd_req_t *request) {
    const int noDelay = 1;
    setsockopt(httpd_req_to_sockfd(request), IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));
    httpd_resp_set_type(request, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    uint8_t *jpeg = static_cast<uint8_t *>(ps_malloc(previewCapacity));
    assert(jpeg);
    previewActive = true;
    uint32_t seen = previewSequence;
    esp_err_t result = ESP_OK;
    while (result == ESP_OK) {
        const TickType_t started = xTaskGetTickCount();
        xSemaphoreTake(previewMutex, portMAX_DELAY);
        const bool fresh = seen != previewSequence;
        const size_t length = previewLength;
        if (fresh) {
            memcpy(jpeg, previewJpeg, length);
            seen = previewSequence;
        }
        xSemaphoreGive(previewMutex);
        if (fresh) {
            char header[100];
            const int size =
                snprintf(header, sizeof(header),
                         "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                         unsigned(length));
            result = httpd_resp_send_chunk(request, header, size);
            if (result == ESP_OK)
                result = httpd_resp_send_chunk(request, reinterpret_cast<char *>(jpeg), length);
            if (result == ESP_OK)
                result = httpd_resp_send_chunk(request, "\r\n", 2);
            ++previewCount;
        }
        const TickType_t elapsed = xTaskGetTickCount() - started;
        vTaskDelay(elapsed < pdMS_TO_TICKS(42) ? pdMS_TO_TICKS(42) - elapsed : 1);
    }
    previewActive = false;
    free(jpeg);
    return result;
}

void startPreview() {
    previewJpeg = static_cast<uint8_t *>(ps_malloc(previewCapacity));
    assert(previewJpeg);
    previewMutex = xSemaphoreCreateMutex();
    assert(previewMutex);
    httpd_config_t serverConfig = HTTPD_DEFAULT_CONFIG();
    serverConfig.server_port = 81;
    serverConfig.core_id = 0;
    serverConfig.task_priority = 1; // Video networking stays off the detector's core.
    serverConfig.stack_size = 8192;
    serverConfig.send_wait_timeout = 1;
    httpd_handle_t server;
    ESP_ERROR_CHECK(httpd_start(&server, &serverConfig));
    httpd_uri_t stream = {};
    stream.uri = "/stream";
    stream.method = HTTP_GET;
    stream.handler = streamPreview;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &stream));
    web.on("/config", HTTP_POST, []() {
        web.client().setNoDelay(true);
        const String body = web.arg("plain");
        if (!applyCalibration(body)) {
            web.send(400, "application/json", "{\"error\":\"Invalid camera calibration\"}");
            return;
        }
        settings.putString("calibration", body);
        web.send(200, "application/json", "{\"saved\":true}");
    });
    web.on("/status", HTTP_GET, []() {
        web.client().setNoDelay(true);
        char body[400];
        const bool valid = lastPose.valid &&
                           uint32_t(esp_timer_get_time()) - lastPose.captureUs <= config::maxAgeUs;
        snprintf(
            body, sizeof(body),
            "{\"calibrated\":%s,\"tag_id\":%d,\"tag_size_mm\":%.3f,\"capture_count\":%u,\"pose_"
            "count\":%u,\"preview_count\":%u,\"valid\":%s,\"position_m\":[%.4f,%.4f,%.4f]}",
            calibrated() ? "true" : "false", config::tagId, config::tagSizeM * 1000,
            unsigned(captureCount), unsigned(poseCount), unsigned(previewCount),
            valid ? "true" : "false", lastPose.position[0], lastPose.position[1],
            lastPose.position[2]);
        web.send(200, "application/json", body);
    });
    web.begin();
}
