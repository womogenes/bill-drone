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
// The preview asks for a frame by setting previewWanted; the detector then hands over its buffer instead of
// returning it. The detector never waits for video, and nothing is copied or encoded on its core.
QueueHandle_t previewQueue;
volatile bool previewWanted = false;
// While streaming, the encoder's allocations would also land in internal RAM during tracking; see main.cpp.
volatile bool previewStreaming = false;
constexpr TickType_t previewPeriod = pdMS_TO_TICKS(66); // At most 15 fps.
volatile uint32_t captureCount = 0, poseCount = 0, previewCount = 0, detectUs = 0, frameUs = 0, readoutAgeUs = 0,
                  trackedCount = 0, detectedCount = 0;
PosePacket lastPose;
extern uint32_t stalls, recoveries, lastCrumbs[3];

// The last 2 KB of ESP-IDF log output (the camera driver's warnings and errors), for /log.
char logRing[2048];
size_t logHead = 0;
// Frames the driver dropped because lines went missing (its "FB-SIZE" error).
volatile uint32_t shortFrames = 0;
portMUX_TYPE logLock = portMUX_INITIALIZER_UNLOCKED;

int logToRing(const char *format, va_list args) {
    char line[160];
    const int length = vsnprintf(line, sizeof(line), format, args);
    shortFrames += strstr(line, "FB-SIZE") != nullptr;
    portENTER_CRITICAL(&logLock);
    for (int i = 0; i < length && i < int(sizeof(line)) - 1; ++i)
        logRing[logHead++ % sizeof(logRing)] = line[i];
    portEXIT_CRITICAL(&logLock);
    return length;
}
extern bool cameraReady, cameraStalled;
extern bool tracking;
extern float trackedCorners[4][2];

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
    assert(settings.begin("camera16crop", false));
    String saved = settings.isKey("calibration") ? settings.getString("calibration") : "";
    if (saved.length())
        assert(applyCalibration(saved));
}

void finishFrame(camera_fb_t *frame) {
    if (previewWanted) {
        previewWanted = false;
        if (xQueueSend(previewQueue, &frame, 0) == pdTRUE)
            return;
    }
    esp_camera_fb_return(frame);
}

void dropPreviewFrames() {
    camera_fb_t *frame;
    while (xQueueReceive(previewQueue, &frame, 0) == pdTRUE)
        esp_camera_fb_return(frame);
}

// Runs in the core-0 HTTP task: encodes the handed-over Y8 frame, returns it to the driver, then sends.
esp_err_t streamPreview(httpd_req_t *request) {
    const int noDelay = 1;
    setsockopt(httpd_req_to_sockfd(request), IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));
    httpd_resp_set_type(request, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    dropPreviewFrames();
    previewStreaming = true;
    previewWanted = true;
    TickType_t started = xTaskGetTickCount();
    esp_err_t result = ESP_OK;
    while (result == ESP_OK) {
        camera_fb_t *frame;
        if (xQueueReceive(previewQueue, &frame, pdMS_TO_TICKS(1000)) != pdTRUE)
            continue;
        uint8_t *jpeg;
        size_t length;
        const bool encoded = frame2jpg(frame, 80, &jpeg, &length);
        esp_camera_fb_return(frame);
        assert(encoded);
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
        free(jpeg);
        ++previewCount;
        vTaskDelayUntil(&started, previewPeriod);
        previewWanted = true;
    }
    previewWanted = false;
    previewStreaming = false;
    dropPreviewFrames();
    return result;
}

void startPreview() {
    previewQueue = xQueueCreate(1, sizeof(camera_fb_t *));
    assert(previewQueue);
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
    esp_log_set_vprintf(logToRing);
    esp_log_level_set("*", ESP_LOG_WARN);
    web.on("/log", HTTP_GET, []() {
        String text;
        portENTER_CRITICAL(&logLock);
        const size_t start = logHead > sizeof(logRing) ? logHead - sizeof(logRing) : 0;
        char copy[sizeof(logRing) + 1];
        size_t n = 0;
        for (size_t i = start; i < logHead; ++i)
            copy[n++] = logRing[i % sizeof(logRing)];
        portEXIT_CRITICAL(&logLock);
        copy[n] = 0;
        web.send(200, "text/plain", copy);
    });
    web.on("/status", HTTP_GET, []() {
        web.client().setNoDelay(true);
        char body[1100];
        // The sensor's AEC exposure (lines; the effective exposure is capped at the frame) and real gain.
        sensor_t *sensor = cameraReady ? esp_camera_sensor_get() : nullptr;
        const bool valid = lastPose.valid &&
                           uint32_t(esp_timer_get_time()) - lastPose.captureUs <= config::maxAgeUs;
        snprintf(
            body, sizeof(body),
            "{\"camera\":\"%s\",\"calibrated\":%s,\"tag_id\":%d,\"tag_size_mm\":%.3f,\"capture_count\":%u,\"pose_"
            "count\":%u,\"tracked_count\":%u,\"detected_count\":%u,\"preview_count\":%u,\"detect_us\":%u,\"frame_us\":%u,\"age_us\":%u,\"frame_age_us\":%u,\"valid\":%s,"
            "\"position_m\":[%.4f,%.4f,%.4f],\"margin\":%.1f,\"reprojection_px\":%.3f,\"internal_min\":%u,\"uptime_ms\":%u,\"reset\":%d,"
            "\"flow_px\":[%.3f,%.3f],\"flow_inliers\":%u,\"flow_residual_px\":%.3f,\"short_frames\":%u,\"stalls\":%u,\"recoveries\":%u,"
            "\"last_crumbs\":[%u,%u,%u],\"gain\":%.2f,\"exposure_lines\":%d,\"frame_lines\":%d}",
            cameraStalled ? "stalled" : cameraReady ? "running" : "stopped", calibrated() ? "true" : "false", config::tagId, config::tagSizeM * 1000,
            unsigned(captureCount), unsigned(poseCount), unsigned(trackedCount), unsigned(detectedCount),
            unsigned(previewCount), unsigned(detectUs),
            unsigned(frameUs), unsigned(lastPose.ageUs), unsigned(readoutAgeUs), valid ? "true" : "false", lastPose.position[0],
            lastPose.position[1], lastPose.position[2], lastPose.margin, lastPose.reprojectionPx,
            unsigned(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)), unsigned(millis()), int(esp_reset_reason()),
            lastPose.flow[0], lastPose.flow[1], unsigned(lastPose.flowInliers), lastPose.flowResidualPx, unsigned(shortFrames), unsigned(stalls),
            unsigned(recoveries), unsigned(lastCrumbs[0]), unsigned(lastCrumbs[1]), unsigned(lastCrumbs[2]),
            sensor ? sensor->get_reg(sensor, 0x350a, 0x3ff) / 16.0f : 0, sensor ? sensor->get_reg(sensor, 0x3500, 0xffffff) >> 4 : 0,
            sensor ? sensor->get_reg(sensor, 0x380e, 0xffff) : 0);
        // Tracked corners in preview pixels for the dashboard overlay; core 1 may be mid-update, which only shifts the drawing.
        const int length = strlen(body) - 1;
        if (tracking)
            snprintf(body + length, sizeof(body) - length, ",\"corners_px\":[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]]}",
                     trackedCorners[0][0], trackedCorners[0][1], trackedCorners[1][0], trackedCorners[1][1],
                     trackedCorners[2][0], trackedCorners[2][1], trackedCorners[3][0], trackedCorners[3][1]);
        else
            snprintf(body + length, sizeof(body) - length, ",\"corners_px\":null}");
        web.send(200, "application/json", body);
    });
    web.begin();
}
