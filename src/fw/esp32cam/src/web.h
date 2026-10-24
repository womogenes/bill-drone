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
// The preview asks for a frame by setting previewWanted; the detector then hands over, with the tag it found, the flow
// task's 80x60 reduction of the frame (flow.h, in internal RAM), and a core-0 task copies it for the stream to encode.
// The preview never holds a frame buffer or reads PSRAM. Copying preview pixels out of the PSRAM frame, even 160x120
// of them, competed with the driver's own PSRAM writes and lost the next frame about half the time (2026-10-22, with
// fw/esp32cam/record.py). Calibration needs real pixels: /stream?full=1 hands over the frame buffer itself for a
// full 320x240 copy, at that cost. The detector never waits for video, and nothing is encoded on its core.
struct PreviewFrame {
    camera_fb_t *buffer;    // full mode: the frame, returned by the copier
    const uint8_t *reduced; // otherwise: the flow's reduced frame
    uint8_t tag; // 0 none, 1 tracked, 2 tracked with a valid pose
    float corners[4][2];
    float position[3];
    uint32_t handoverUs;
};
QueueHandle_t handoverQueue;
SemaphoreHandle_t previewReady;
PreviewFrame previewFrame; // the copy's tag; previewFrame.buffer is unused
uint8_t *previewPixels;    // Y8 in PSRAM, owned by the copier until previewReady, then by the stream
uint8_t previewReduced[flow::width * flow::height]; // the default preview, in internal RAM
volatile bool previewFull = false;
int previewWidth = flow::width, previewHeight = flow::height;
uint8_t *previewJpeg;      // encoder output in PSRAM
constexpr size_t previewJpegBytes = 40 * 1024;
volatile bool previewWanted = false;
// Longest time since the last /status that the copier held a frame buffer, and frames the detector found too old.
volatile uint32_t previewHoldUs = 0, staleFrames = 0;
// While streaming, the encoder's small allocations would also land in internal RAM during a search; see main.cpp.
volatile bool previewStreaming = false;
constexpr TickType_t previewPeriod = pdMS_TO_TICKS(66); // At most 15 fps.
volatile uint32_t captureCount = 0, poseCount = 0, previewCount = 0, detectUs = 0, frameUs = 0, readoutAgeUs = 0,
                  trackedCount = 0, detectedCount = 0;
PosePacket lastPose;
extern uint32_t stalls, recoveries, lastCrumbs[3];
extern volatile uint32_t linkSent, linkDropped, linkReceived;
namespace blackbox {
int status(char *out, size_t size);
}

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
extern UBaseType_t driverPriority;

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

void finishFrame(camera_fb_t *frame, bool found, const float corners[4][2], const PosePacket &packet,
                 const uint8_t *reduced) {
    if (previewWanted) {
        previewWanted = false;
        const bool full = previewFull;
        PreviewFrame handover = {full ? frame : nullptr, reduced, uint8_t(found ? (packet.valid ? 2 : 1) : 0), {}, {},
                                 uint32_t(esp_timer_get_time())};
        if (found)
            memcpy(handover.corners, corners, sizeof(handover.corners));
        memcpy(handover.position, packet.position, sizeof(handover.position));
        if (xQueueSend(handoverQueue, &handover, 0) == pdTRUE && full)
            return;
    }
    esp_camera_fb_return(frame);
}

// Core 0, above the flow and HTTP tasks and below the driver's: the copy runs while the detector waits for its next
// frame, so the two cores do not compete for PSRAM while the detector works.
void copyLoop(void *) {
    for (;;) {
        PreviewFrame handover;
        xQueueReceive(handoverQueue, &handover, portMAX_DELAY);
        bool fits = true;
        if (handover.buffer) {
            fits = handover.buffer->len == 320 * 240;
            if (fits)
                memcpy(previewPixels, handover.buffer->buf, handover.buffer->len);
            esp_camera_fb_return(handover.buffer);
            const uint32_t held = uint32_t(esp_timer_get_time()) - handover.handoverUs;
            if (held > previewHoldUs)
                previewHoldUs = held;
            previewWidth = 320;
            previewHeight = 240;
        } else {
            memcpy(previewReduced, handover.reduced, sizeof(previewReduced));
            previewWidth = flow::width;
            previewHeight = flow::height;
        }
        if (!fits) {
            previewWanted = true;
            continue;
        }
        previewFrame = handover;
        xSemaphoreGive(previewReady);
    }
}

struct JpegOutput {
    size_t length;
    bool overflow;
};

constexpr size_t jpegOffset = 256; // room for the part header in front

size_t appendJpeg(void *arg, size_t index, const void *data, size_t length) {
    JpegOutput &output = *static_cast<JpegOutput *>(arg);
    if (jpegOffset + index + length + 2 > previewJpegBytes) {
        output.overflow = true;
        return 0;
    }
    memcpy(previewJpeg + jpegOffset + index, data, length);
    output.length = index + length;
    return length;
}

// Runs in the core-0 HTTP task. Each part carries its frame's tag in an X-Tag header ("none", or the state, the four
// corners in full-resolution 320x240 pixels and the camera position in metres), so the overlay matches the picture.
esp_err_t streamPreview(httpd_req_t *request) {
    // Every Wi-Fi packet competes on core 0 with the camera driver, which must empty a 12-line DMA block every 0.7 ms
    // or lose the frame: chunked sends cost about nine packets a frame and lost the next frame 40% of the time. The
    // response is written raw instead, one send (one packet for the small preview) per frame.
    const int noDelay = 1;
    setsockopt(httpd_req_to_sockfd(request), IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));
    static const char head[] = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=frame\r\n"
                               "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
    if (httpd_send(request, head, sizeof(head) - 1) < 0)
        return ESP_FAIL;
    char query[32];
    previewFull = httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK && strstr(query, "full=1");
    xSemaphoreTake(previewReady, 0);
    previewStreaming = true;
    previewWanted = true;
    TickType_t started = xTaskGetTickCount();
    esp_err_t result = ESP_OK;
    while (result == ESP_OK) {
        if (xSemaphoreTake(previewReady, pdMS_TO_TICKS(1000)) != pdTRUE) {
            previewWanted = true;
            continue;
        }
        const PreviewFrame frame = previewFrame;
        // The part header goes in front of the JPEG in the same buffer, so the whole part is one send.
        JpegOutput output = {0, false};
        uint8_t *pixels = previewWidth == 320 ? previewPixels : previewReduced;
        const bool encoded = fmt2jpg_cb(pixels, previewWidth * previewHeight, previewWidth, previewHeight, PIXFORMAT_GRAYSCALE, 75, appendJpeg, &output);
        // The copy is free again: the next one overlaps this frame's sending.
        previewWanted = true;
        if (!encoded || output.overflow)
            continue;
        char tag[160] = "none";
        if (frame.tag)
            snprintf(tag, sizeof(tag), "%s %.1f,%.1f %.1f,%.1f %.1f,%.1f %.1f,%.1f %.3f,%.3f,%.3f",
                     frame.tag == 2 ? "valid" : "tracked", frame.corners[0][0], frame.corners[0][1], frame.corners[1][0],
                     frame.corners[1][1], frame.corners[2][0], frame.corners[2][1], frame.corners[3][0], frame.corners[3][1],
                     frame.position[0], frame.position[1], frame.position[2]);
        char header[jpegOffset];
        const int size = snprintf(header, sizeof(header),
                                  "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nX-Tag: %s\r\n\r\n",
                                  unsigned(output.length), tag);
        char *part = reinterpret_cast<char *>(previewJpeg) + jpegOffset - size;
        memcpy(part, header, size);
        memcpy(part + size + output.length, "\r\n", 2);
        const int total = size + output.length + 2;
        for (int sent = 0; sent < total && result == ESP_OK;) {
            const int n = httpd_send(request, part + sent, total - sent);
            result = n > 0 ? ESP_OK : ESP_FAIL;
            sent += n;
        }
        ++previewCount;
        vTaskDelayUntil(&started, previewPeriod);
    }
    previewWanted = false;
    previewStreaming = false;
    return result;
}

void startPreview() {
    handoverQueue = xQueueCreate(1, sizeof(PreviewFrame));
    previewReady = xSemaphoreCreateBinary();
    previewPixels = static_cast<uint8_t *>(heap_caps_malloc(320 * 240, MALLOC_CAP_SPIRAM));
    previewJpeg = static_cast<uint8_t *>(heap_caps_malloc(previewJpegBytes, MALLOC_CAP_SPIRAM));
    assert(handoverQueue && previewReady && previewPixels && previewJpeg);
    xTaskCreatePinnedToCore(copyLoop, "preview copy", 2048, nullptr, 3, nullptr, 0);
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
        char body[1400];
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
            "\"last_crumbs\":[%u,%u,%u],\"gain\":%.2f,\"exposure_lines\":%d,\"frame_lines\":%d,\"preview_hold_us\":%u,\"stale_frames\":%u,\"driver_priority\":[%u,%u],"
            "\"link_sent\":%u,\"link_dropped\":%u,\"link_rx_bytes\":%u,",
            cameraStalled ? "stalled" : cameraReady ? "running" : "stopped", calibrated() ? "true" : "false", config::tagId, config::tagSizeM * 1000,
            unsigned(captureCount), unsigned(poseCount), unsigned(trackedCount), unsigned(detectedCount),
            unsigned(previewCount), unsigned(detectUs),
            unsigned(frameUs), unsigned(lastPose.ageUs), unsigned(readoutAgeUs), valid ? "true" : "false", lastPose.position[0],
            lastPose.position[1], lastPose.position[2], lastPose.margin, lastPose.reprojectionPx,
            unsigned(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)), unsigned(millis()), int(esp_reset_reason()),
            lastPose.flow[0], lastPose.flow[1], unsigned(lastPose.flowInliers), lastPose.flowResidualPx, unsigned(shortFrames), unsigned(stalls),
            unsigned(recoveries), unsigned(lastCrumbs[0]), unsigned(lastCrumbs[1]), unsigned(lastCrumbs[2]),
            sensor ? sensor->get_reg(sensor, 0x350a, 0x3ff) / 16.0f : 0, sensor ? sensor->get_reg(sensor, 0x3500, 0xffffff) >> 4 : 0,
            sensor ? sensor->get_reg(sensor, 0x380e, 0xffff) : 0, unsigned(previewHoldUs), unsigned(staleFrames),
            unsigned(driverPriority), unsigned(configMAX_PRIORITIES - 1), unsigned(linkSent), unsigned(linkDropped),
            unsigned(linkReceived));
        const size_t used = strlen(body);
        const int more = blackbox::status(body + used, sizeof(body) - used - 1);
        if (more > 0 && used + more + 2 <= sizeof(body))
            strcat(body, "}");
        else
            strcpy(body + used - 1, "}");
        previewHoldUs = 0;
        web.send(200, "application/json", body);
    });
    web.begin();
}
