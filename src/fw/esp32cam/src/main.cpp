#include <Arduino.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "tag16h5.h"
#include "pose.h"
#include "track.h"
#include "tag.h"
#include "flow.h"
#include "ota.h"

#include "web.h"
#include "record.h"

TagFinder finder;
uint32_t sequence = 0;
bool cameraReady = false;
// Set by OTA; the detection loop stops the camera between frames, so OTA never has to win a lock against it.
volatile bool cameraStopRequested = false;
bool cameraStalled = false;
uint32_t lastCaptureUs = 0;
// GET /search?frames=N: the next N frames forget the tag and search the whole frame, to measure that path.
volatile uint32_t searchTests = 0;

camera_config_t cameraConfig = {};
uint32_t stalls = 0, recoveries = 0;
int exposureLines;
// What each core was doing, kept across a panic restart: /status shows the values from before the last reset.
RTC_NOINIT_ATTR uint32_t crumbs[3];
uint32_t lastCrumbs[3];

// Wi-Fi, OTA and the control server share core 0 with the radio and the camera driver's copy task. Core 1 only captures and detects.
void serviceTask(void *) {
    while (true) {
        serviceOta();
        web.handleClient();
        delay(2);
    }
}

uint32_t frameStart(const camera_fb_t *frame) {
    return uint32_t(uint64_t(frame->timestamp.tv_sec) * 1000000 + frame->timestamp.tv_usec);
}

// The driver's queue is first in, first out, and with two buffers it stops capturing while the detector holds one and
// the other waits finished. That keeps the camera DMA copy from competing with the detector for PSRAM, but the waiting
// frame may have finished long ago. Such a frame is returned and the next one, started afterwards, is used instead.
UBaseType_t driverPriority = 0; // as the driver created its task, for /status

camera_fb_t *freshFrame() {
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame || uint32_t(esp_timer_get_time()) - frameStart(frame) <= config::maxFrameAgeUs)
        return frame;
    esp_camera_fb_return(frame);
    ++staleFrames;
    return esp_camera_fb_get();
}

struct {
    camera_fb_t *frame;
    bool excluded, found;
    float box[4];
    flow::Result result;
} flowJob;
TaskHandle_t flowTask;
SemaphoreHandle_t flowDone;

void flowLoop(void *) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        crumbs[1] = 1;
        flowJob.found = flow::update(flowJob.frame->buf, flowJob.frame->width, flowJob.excluded ? flowJob.box : nullptr,
                                     flowJob.result);
        crumbs[1] = 0;
        xSemaphoreGive(flowDone);
    }
}

// Starts the camera: the driver, then mirror and flip (a 180-degree rotation from the previous orientation), and on
// the OV3660 the PLL and window of config.h. The driver's raw-QVGA clock (20 MHz SYSCLK, 11 fps) becomes its
// small-frame one, 40 MHz SYSCLK with the same 10 MHz PCLK; a faster line rate panicked within minutes.
bool startCamera() {
    if (esp_camera_init(&cameraConfig) != ESP_OK)
        return false;
    sensor_t *sensor = esp_camera_sensor_get();
    const bool configured =
        sensor->set_hmirror(sensor, 0) == 0 && sensor->set_vflip(sensor, 1) == 0 &&
        (sensor->id.PID != OV3660_PID ||
         (sensor->set_pll(sensor, 0, 8, 1, 0, 0, 0, 1, 8) == 0 &&
          sensor->set_res_raw(sensor, (2048 - config::windowW) / 2, (1536 - config::windowH) / 2,
                              (2048 + config::windowW) / 2 + 31, (1536 + config::windowH) / 2 + 11, 8, 2, config::hts,
                              config::vts, config::outW, config::outH, true, true) == 0));
    if (!configured) {
        esp_camera_deinit();
        return false;
    }
    // The driver's task must empty a 12-line DMA block every 0.7 ms or lose the frame. It is created at the same
    // priority as the Wi-Fi task on core 0, so Wi-Fi traffic (the preview, /status, records) cost frames; above it,
    // its short copies only delay Wi-Fi.
    if (TaskHandle_t driver = xTaskGetHandle("cam_task")) {
        driverPriority = uxTaskPriorityGet(driver);
        vTaskPrioritySet(driver, configMAX_PRIORITIES - 1);
    }
    // Exposure is ours (adjustExposure), gain stays automatic up to its 64x ceiling.
    exposureLines = config::exposureLines;
    return sensor->set_exposure_ctrl(sensor, 0) == 0 && sensor->set_aec_value(sensor, exposureLines) == 0 &&
           sensor->set_reg(sensor, 0x3a18, 0xff, 0x03) == 0 && sensor->set_reg(sensor, 0x3a19, 0xff, 0xff) == 0;
}

// Short exposures keep motion blur down; the automatic gain makes up the light. Only when the gain nears its ceiling
// does the exposure grow, up to the whole frame, and it shrinks back as the gain falls.
void adjustExposure() {
    sensor_t *sensor = esp_camera_sensor_get();
    const float gain = sensor->get_reg(sensor, 0x350a, 0x3ff) / 16.0f;
    int lines = exposureLines;
    if (gain > config::raiseExposureGain)
        lines = min(config::vts - 4, lines * 5 / 4);
    else if (gain < config::lowerExposureGain)
        lines = max(config::exposureLines, lines * 4 / 5);
    if (lines != exposureLines && sensor->set_aec_value(sensor, lines) == 0)
        exposureLines = lines;
}

// The detector's working set is PSRAM-bound (union-find alone is 8 bytes per pixel). A window is small enough to keep
// in internal RAM; the limit is global, so it stays at the default while the preview encoder is running too. A
// whole-frame search's many small allocations (clusters, quads) took internal RAM down to 7.7 KB, so they all go to
// PSRAM; Wi-Fi and lwip allocate internal RAM explicitly and are unaffected.
void detectorWorkspace(Workspace where) {
    heap_caps_malloc_extmem_enable(where == workspaceInternal && !previewStreaming ? config::trackInternalBytes
                                   : where == workspaceExternal                    ? 0
                                                                                   : 4096);
}

void setup() {
    memcpy(lastCrumbs, crumbs, sizeof(crumbs));
    memset(crumbs, 0, sizeof(crumbs));
    // UART0 carries pose packets to the drone from here on; the boot ROM's own messages on GPIO1 fail the drone's magic
    // and CRC checks. A transmit buffer lets publish() return without waiting for the wire.
    Serial.setTxBufferSize(1024);
    Serial.begin(config::linkBaud, SERIAL_8N1, config::linkRxPin, config::linkTxPin);
    startWifi();
    assert(psramFound());
    heap_caps_malloc_extmem_enable(4096);
    pinMode(config::syncPin, OUTPUT);
    digitalWrite(config::syncPin, LOW);
    loadCalibration();

    // AI Thinker ESP32-CAM pinout; the camera driver identifies the sensor.
    camera_config_t &camera = cameraConfig;
    camera.pin_pwdn = 32;
    camera.pin_reset = -1;
    camera.pin_xclk = 0;
    camera.pin_sccb_sda = 26;
    camera.pin_sccb_scl = 27;
    camera.pin_d0 = 5;
    camera.pin_d1 = 18;
    camera.pin_d2 = 19;
    camera.pin_d3 = 21;
    camera.pin_d4 = 36;
    camera.pin_d5 = 39;
    camera.pin_d6 = 34;
    camera.pin_d7 = 35;
    camera.pin_vsync = 25;
    camera.pin_href = 23;
    camera.pin_pclk = 22;
    camera.xclk_freq_hz = 20000000;
    camera.ledc_timer = LEDC_TIMER_0;
    camera.ledc_channel = LEDC_CHANNEL_0;
    // Raw Y8: no JPEG decode on the detector and no compression artefacts on tag edges.
    camera.pixel_format = PIXFORMAT_GRAYSCALE;
    camera.frame_size = FRAMESIZE_QVGA;
    // One for the detector, one for the driver; only the full-resolution preview holds a frame, for its copy.
    camera.fb_count = 2;
    camera.fb_location = CAMERA_FB_IN_PSRAM;
    camera.grab_mode = CAMERA_GRAB_LATEST;
    if (!startCamera()) {
        xTaskCreatePinnedToCore(serviceTask, "service", 8192, nullptr, 1, nullptr, 0);
        return;
    }

    apriltag_detector_t *detector = finder.detector = apriltag_detector_create();
    finder.family = tag16h5_create();
    apriltag_detector_add_family_bits(detector, finder.family, 0);
    detector->nthreads = 1;
    detector->refine_edges = true;
    // A tag is high-contrast black on white: reject faint and tiny blobs at thresholding, before the expensive
    // union-find, clustering and quad fitting, which otherwise scale with every noise blob in a dim, cluttered scene.
    detector->qtp.min_white_black_diff = 20;
    detector->qtp.max_nmaxima = 6;
    finder.workspace = detectorWorkspace;
    web.on("/search", HTTP_GET, []() {
        searchTests = uint32_t(max(0L, min(1000L, web.arg("frames").toInt())));
        web.send(200, "application/json", "{\"searching\":true}");
    });
    startPreview();
    startRecording();
    flowDone = xSemaphoreCreateBinary();
    assert(flowDone);
    xTaskCreatePinnedToCore(flowLoop, "flow", 4096, nullptr, 2, &flowTask, 0);
    cameraReady = true;
    xTaskCreatePinnedToCore(serviceTask, "service", 8192, nullptr, 1, nullptr, 0);
}

// Each frame's packet goes out on UART0 as soon as it is ready; at 2 Mbaud its 164 bytes take 0.8 ms, far inside a frame.
// syncPin's rising edge marks the moment the packet's age was measured, so the drone dates the capture by that edge
// whatever its UART latency. If the transmit buffer is full (the line is stuck), the packet is dropped.
void publish(const PosePacket &packet) {
    if (Serial.availableForWrite() < int(sizeof(packet)))
        return;
    gpio_set_level(gpio_num_t(config::syncPin), 1);
    Serial.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    gpio_set_level(gpio_num_t(config::syncPin), 0);
}

void loop() {
    if (cameraStopRequested && cameraReady) {
        esp_camera_deinit();
        cameraReady = false;
    }
    if (!cameraReady) {
        delay(100);
        return;
    }
    crumbs[0] = 1;
    camera_fb_t *frame = freshFrame();
    // A driver that stops delivering frames is restarted, up to three tries; Wi-Fi and OTA stay up regardless, and
    // /status counts stalls and recoveries.
    if (!frame) {
        ++stalls;
        crumbs[2] = stalls;
        esp_camera_deinit();
        cameraReady = false;
        for (int attempt = 0; attempt < 3 && !cameraReady; ++attempt) {
            delay(200);
            cameraReady = startCamera();
        }
        cameraStalled = !cameraReady;
        recoveries += cameraReady;
        finder.tracking = false;
        flow::primed = false;
        return;
    }
    crumbs[0] = 2;
    const uint32_t started = uint32_t(esp_timer_get_time());
    ++captureCount;
    PosePacket packet;
    packet.sequence = sequence++;
    packet.tagId = config::tagId;
    // Readout of this frame's centre row on the CAM clock; once a tag is found, of the tag's centre row instead.
    const uint32_t frameCentreUs = frameStart(frame) + uint32_t(config::outH / 2 * config::rowUs);
    packet.captureUs = frameCentreUs;
    // The flow runs on core 0 alongside the tag search; it skips the predicted tag and half its size around it.
    flowJob.frame = frame;
    flowJob.excluded = finder.tracking;
    if (finder.tracking) {
        flowJob.box[0] = flowJob.box[1] = INFINITY;
        flowJob.box[2] = flowJob.box[3] = -INFINITY;
        float predicted[4][2];
        finder.predict(frameStart(frame), predicted);
        for (int i = 0; i < 4; ++i)
            for (int axis = 0; axis < 2; ++axis) {
                flowJob.box[axis] = fminf(flowJob.box[axis], predicted[i][axis]);
                flowJob.box[axis + 2] = fmaxf(flowJob.box[axis + 2], predicted[i][axis]);
            }
        for (int axis = 0; axis < 2; ++axis) {
            flowJob.box[axis] -= finder.sidePx / 2;
            flowJob.box[axis + 2] += finder.sidePx / 2;
        }
    }
    xTaskNotifyGive(flowTask);
    crumbs[0] = 3;
    TagKind kind = tagNone;
    float corners[4][2] = {}, margin = 0;
    if (calibrated()) {
        ++poseCount;
        if (searchTests) {
            --searchTests;
            finder.tracking = false;
            finder.seenUs = 0;
        }
        kind = finder.find(frame->buf, frame->width, frame->height, frameStart(frame), corners, margin);
        trackedCount += kind == tagTracked;
        detectedCount += kind == tagDetected;
    }
    crumbs[0] = 6;
    const uint32_t flowWaitStart = uint32_t(esp_timer_get_time());
    xSemaphoreTake(flowDone, portMAX_DELAY);
    const uint32_t flowWaitUs = uint32_t(esp_timer_get_time()) - flowWaitStart;
    crumbs[0] = 7;
    if (flowJob.found && frameStart(frame) - lastCaptureUs <= 2 * config::framePeriodUs) {
        memcpy(packet.flow, flowJob.result.shift, sizeof(packet.flow));
        packet.flowIntervalUs = frameStart(frame) - lastCaptureUs;
        packet.flowResidualPx = flowJob.result.residualPx;
        packet.flowInliers = flowJob.result.inliers;
    }
    // The pose waits for the flow, which removes the rolling shutter when the tag's own velocity is not fresh.
    if (calibrated()) {
        float unskewed[4][2];
        tagPose(finder, kind, corners, margin, frameStart(frame), flowJob.result.shift, packet.flowIntervalUs, packet,
                unskewed);
    }
    packet.flowOffsetUs = int16_t(frameCentreUs - packet.captureUs);
    const uint32_t now = uint32_t(esp_timer_get_time());
    packet.ageUs = now - packet.captureUs;
    if (packet.ageUs > config::maxAgeUs)
        packet.valid = packet.trusted = 0;
    if (!packet.trusted) {
        packet.valid = 0;
        memset(packet.position, 0, sizeof(packet.position));
        memset(packet.rotation, 0, sizeof(packet.rotation));
        memset(packet.tagPosition, 0, sizeof(packet.tagPosition));
        memset(packet.altPosition, 0, sizeof(packet.altPosition));
        memset(packet.altRotation, 0, sizeof(packet.altRotation));
    }
    packet.checksum =
        crc16(reinterpret_cast<const uint8_t *>(&packet), offsetof(PosePacket, checksum));
    publish(packet);
    lastPose = packet;
    detectUs = now - started;
    readoutAgeUs = now - frameStart(frame);
    frameUs = frameStart(frame) - lastCaptureUs;
    lastCaptureUs = frameStart(frame);
    crumbs[0] = 8;
    if (recording()) {
        FrameRecord record;
        record.packet = packet;
        record.reserved = 0;
        if (kind)
            memcpy(record.corners, finder.corners, sizeof(record.corners));
        else
            memset(record.corners, 0, sizeof(record.corners));
        record.frameStartUs = frameStart(frame);
        record.processUs = detectUs;
        record.flowWaitUs = uint16_t(min(flowWaitUs, 65535u));
        record.found = kind;
        record.trackFailed = uint8_t(finder.trackFailed);
        record.alignment = uint8_t(fminf(255, fmaxf(0, finder.alignment * 100)));
        record.measured = finder.measured;
        const TagFinder::Work &work = finder.work;
        record.measures = uint8_t(work.measures);
        record.alignCoarse = uint16_t(min(work.align.coarse + work.alignFrame.coarse, 65535));
        record.alignFine = uint16_t(min(work.align.fine + work.alignFrame.fine, 65535));
        record.windowPixels = uint16_t(min(work.windowPixels, 65535));
        record.framePixels = uint16_t(min(work.framePixels, 65535));
        record.preview = previewWanted;
        record.shortFrames = uint16_t(shortFrames);
        addRecord(record);
    }
    finishFrame(frame, kind != tagNone, finder.corners, packet, flow::images[flow::current]);
    crumbs[0] = 9;
    if (packet.sequence % 15 == 0)
        adjustExposure();
    crumbs[0] = 10;
}
