#include <Arduino.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "tag16h5.h"
#include "pose.h"
#include "track.h"
#include "flow.h"
#include "ota.h"

#include "web.h"

PosePacket *wirePacket;
apriltag_detector_t *detector;
apriltag_family_t *family;
uint32_t sequence = 0;
spi_slave_transaction_t transfer = {};
bool transferPending = false;
bool cameraReady = false;
// Set by OTA; the detection loop stops the camera between frames, so OTA never has to win a lock against it.
volatile bool cameraStopRequested = false;
bool cameraStalled = false;
// Shortest side of the last accepted tag, in pixels; zero after a miss.
float tagSidePx = 0;
struct Window {
    int x, y, w, h;
};
// The tracked tag's corners in this and the previous frame; tracking is false after a miss.
bool tracking = false;
float trackedCorners[4][2], previousCorners[4][2];
// The tracked tag's neighbourhood, copied out of the frame.
uint8_t window[8 * 1024];
uint32_t lastTagSequence = 0;
uint32_t lastCaptureUs = 0;

camera_config_t cameraConfig = {};
uint32_t stalls = 0, recoveries = 0;
int exposureLines;
// What each core was doing, kept across a panic restart: /status shows the values from before the last reset.
RTC_NOINIT_ATTR uint32_t crumbs[3];
uint32_t lastCrumbs[3];

// Driver callbacks run only after DMA is armed / the transfer has completed.
void IRAM_ATTR transferReady(spi_slave_transaction_t *) {
    gpio_set_level(gpio_num_t(config::readyPin), 1);
}

void IRAM_ATTR transferDone(spi_slave_transaction_t *) {
    gpio_set_level(gpio_num_t(config::readyPin), 0);
}

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
camera_fb_t *freshFrame() {
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame || uint32_t(esp_timer_get_time()) - frameStart(frame) <= config::maxFrameAgeUs)
        return frame;
    esp_camera_fb_return(frame);
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

void setup() {
    memcpy(lastCrumbs, crumbs, sizeof(crumbs));
    memset(crumbs, 0, sizeof(crumbs));
    Serial.begin(115200);
    startWifi();
    assert(psramFound());
    heap_caps_malloc_extmem_enable(4096);
    pinMode(config::readyPin, OUTPUT);
    digitalWrite(config::readyPin, LOW);
    loadCalibration();
    spi_bus_config_t bus = {};
    bus.mosi_io_num = config::mosiPin;
    bus.miso_io_num = config::misoPin;
    bus.sclk_io_num = config::clockPin;
    bus.quadwp_io_num = bus.quadhd_io_num = -1;
    bus.max_transfer_sz = sizeof(PosePacket);
    spi_slave_interface_config_t slave = {};
    slave.spics_io_num = config::csPin;
    slave.mode = 1; // ESP32 slave DMA supports modes 1 and 3.
    slave.queue_size = 1;
    slave.post_setup_cb = transferReady;
    slave.post_trans_cb = transferDone;
    wirePacket = static_cast<PosePacket *>(
        heap_caps_malloc(sizeof(PosePacket), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    assert(wirePacket);
    ESP_ERROR_CHECK(spi_slave_initialize(HSPI_HOST, &bus, &slave, SPI_DMA_CH_AUTO));

    // AI Thinker ESP32-CAM pinout; the camera driver identifies the sensor. The SPI link uses the
    // otherwise unused microSD pins.
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
    // One for the detector, one for the driver; the preview holds a frame only between captures.
    camera.fb_count = 2;
    camera.fb_location = CAMERA_FB_IN_PSRAM;
    camera.grab_mode = CAMERA_GRAB_LATEST;
    if (!startCamera()) {
        Serial.println("Camera start failed; OTA remains available.");
        xTaskCreatePinnedToCore(serviceTask, "service", 8192, nullptr, 1, nullptr, 0);
        return;
    }

    detector = apriltag_detector_create();
    family = tag16h5_create();
    apriltag_detector_add_family_bits(detector, family, 0);
    detector->nthreads = 1;
    detector->refine_edges = true;
    // A tag is high-contrast black on white: reject faint and tiny blobs at thresholding, before the expensive
    // union-find, clustering and quad fitting, which otherwise scale with every noise blob in a dim, cluttered scene.
    detector->qtp.min_white_black_diff = 20;
    detector->qtp.max_nmaxima = 6;
    startPreview();
    flowDone = xSemaphoreCreateBinary();
    assert(flowDone);
    xTaskCreatePinnedToCore(flowLoop, "flow", 4096, nullptr, 2, &flowTask, 0);
    cameraReady = true;
    xTaskCreatePinnedToCore(serviceTask, "service", 8192, nullptr, 1, nullptr, 0);
    Serial.println("AprilTag pose SPI ready: 320x240 Y8, tag16h5, mode 1.");
}

// One outstanding result. READY is raised by the driver, then the host clocks exactly 100 bytes. A result the host
// has not read yet is kept and this one dropped: the host measures how long READY was high, so it knows that age.
void publish(const PosePacket &packet) {
    if (transferPending) {
        spi_slave_transaction_t *finished;
        const esp_err_t result = spi_slave_get_trans_result(HSPI_HOST, &finished, 0);
        if (result == ESP_ERR_TIMEOUT)
            return;
        ESP_ERROR_CHECK(result);
    }
    memcpy(wirePacket, &packet, sizeof(packet));
    transfer = {};
    transfer.length = sizeof(packet) * 8;
    transfer.tx_buffer = wirePacket;
    ESP_ERROR_CHECK(spi_slave_queue_trans(HSPI_HOST, &transfer, 0));
    transferPending = true;
}

// The detector, on a window around the last tag (or the whole frame): finds quads at a resolution that keeps the tag's
// sides about decimatedSidePx and refines and decodes at full resolution. Corners come back in the calibration's pixel
// convention, centres at integers; the detector's are half a pixel further right and down.
bool detectTag(camera_fb_t *frame, const float (*predicted)[2], float corners[4][2], float &margin) {
    const int stride = frame->width;
    // The window: the predicted corners' box grown by half its size and 8 px on each side.
    Window searched = {};
    if (predicted) {
        float low[2] = {INFINITY, INFINITY}, high[2] = {-INFINITY, -INFINITY};
        for (int i = 0; i < 4; ++i)
            for (int axis = 0; axis < 2; ++axis) {
                low[axis] = fminf(low[axis], predicted[i][axis]);
                high[axis] = fmaxf(high[axis], predicted[i][axis]);
            }
        const float grow = fmaxf(high[0] - low[0], high[1] - low[1]) / 2 + 8;
        const int x0 = max(0, int(low[0] - grow)), y0 = max(0, int(low[1] - grow));
        const int x1 = min(stride, int(high[0] + grow) + 1), y1 = min(int(frame->height), int(high[1] + grow) + 1);
        if (x1 - x0 >= 16 && y1 - y0 >= 16)
            searched = {x0, y0, x1 - x0, y1 - y0};
    }
    image_u8_t image = {searched.w ? searched.w : stride, searched.w ? searched.h : int(frame->height), stride,
                        frame->buf + searched.y * stride + searched.x};
    detector->quad_decimate = searched.w ? fminf(3, fmaxf(1, floorf(tagSidePx / config::decimatedSidePx))) : 2;
    // In a window, a cluster much shorter than the tag's outline (such as a single data bit) cannot be the tag.
    detector->qtp.min_cluster_pixels =
        searched.w ? max(config::minClusterPixels, int(config::clusterPerSide * tagSidePx / detector->quad_decimate))
                   : config::minClusterPixels;
    // The detector's working set is PSRAM-bound (union-find alone is 8 bytes per pixel). A window is small enough to
    // keep in internal RAM; a whole-frame search could exhaust what Wi-Fi needs, so it stays in PSRAM. The limit is
    // global, so it stays at the default while the preview encoder is running too.
    heap_caps_malloc_extmem_enable(searched.w && !previewStreaming ? config::trackInternalBytes : 4096);
    zarray_t *tags = apriltag_detector_detect(detector, &image);
    heap_caps_malloc_extmem_enable(4096);
    apriltag_detection_t *target = nullptr;
    int matches = 0;
    for (int i = 0; i < zarray_size(tags); ++i) {
        apriltag_detection_t *tag;
        zarray_get(tags, i, &tag);
        if (tag->id == config::tagId) {
            ++matches;
            target = tag;
        }
    }
    const bool found = matches == 1 && target->hamming == 0 && target->decision_margin >= config::minMargin;
    if (found) {
        for (int i = 0; i < 4; ++i) {
            corners[i][0] = target->p[i][0] + searched.x - 0.5f;
            corners[i][1] = target->p[i][1] + searched.y - 0.5f;
        }
        margin = target->decision_margin;
        ++detectedCount;
    }
    apriltag_detections_destroy(tags);
    return found;
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
        tracking = false;
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
    flowJob.excluded = tracking;
    if (tracking) {
        flowJob.box[0] = flowJob.box[1] = INFINITY;
        flowJob.box[2] = flowJob.box[3] = -INFINITY;
        for (int i = 0; i < 4; ++i)
            for (int axis = 0; axis < 2; ++axis) {
                const float predicted = 2 * trackedCorners[i][axis] - previousCorners[i][axis];
                flowJob.box[axis] = fminf(flowJob.box[axis], predicted);
                flowJob.box[axis + 2] = fmaxf(flowJob.box[axis + 2], predicted);
            }
        for (int axis = 0; axis < 2; ++axis) {
            flowJob.box[axis] -= tagSidePx / 2;
            flowJob.box[axis + 2] += tagSidePx / 2;
        }
    }
    xTaskNotifyGive(flowTask);
    crumbs[0] = 3;
    if (calibrated()) {
        ++poseCount;
        float corners[4][2], margin;
        bool found = false;
        // Tracking: the last corners, moved by their motion over the last frame, are re-measured in this frame.
        if (tracking) {
            const bool consecutive = lastTagSequence + 1 == packet.sequence;
            for (int i = 0; i < 4; ++i)
                for (int axis = 0; axis < 2; ++axis)
                    corners[i][axis] = 2 * trackedCorners[i][axis] - (consecutive ? previousCorners[i][axis] : trackedCorners[i][axis]);
            // The tracker reads around the tag at random; a copy in internal RAM, when it fits, beats the PSRAM frame.
            float low[2] = {INFINITY, INFINITY}, high[2] = {-INFINITY, -INFINITY};
            for (auto &corner : corners)
                for (int axis = 0; axis < 2; ++axis) {
                    low[axis] = fminf(low[axis], corner[axis]);
                    high[axis] = fmaxf(high[axis], corner[axis]);
                }
            const float grow = fmaxf(high[0] - low[0], high[1] - low[1]) / 4 + 4;
            const int left = max(0, int(low[0] - grow)), top = max(0, int(low[1] - grow));
            const int right = min(int(frame->width), int(high[0] + grow) + 2);
            const int bottom = min(int(frame->height), int(high[1] + grow) + 2);
            Image8 image = {frame->buf, 0, 0, int(frame->width), int(frame->height)};
            if (right > left && bottom > top && (right - left) * (bottom - top) <= int(sizeof(window))) {
                for (int y = top; y < bottom; ++y)
                    memcpy(window + (y - top) * (right - left), frame->buf + y * frame->width + left, right - left);
                image = {window, left, top, right - left, bottom - top};
            }
            found = trackCorners(image, corners, family, family->codes[config::tagId], margin);
            trackedCount += found;
        }
        crumbs[0] = 4;
        if (!found)
            found = detectTag(frame, tracking ? corners : nullptr, corners, margin);
        crumbs[0] = 5;
        tracking = found;
        if (found) {
            memcpy(previousCorners, trackedCorners, sizeof(corners));
            memcpy(trackedCorners, corners, sizeof(corners));
            lastTagSequence = packet.sequence;
            float top = INFINITY, bottom = -INFINITY;
            for (auto &corner : corners) {
                top = fminf(top, corner[1]);
                bottom = fmaxf(bottom, corner[1]);
            }
            tagSidePx = shortestSide(corners);
            // Rolling shutter: each output row is read config::rowUs after the one above it.
            packet.captureUs = frameStart(frame) + uint32_t((top + bottom) / 2 * config::rowUs);
            estimatePose(corners, margin, packet);
        }
    }
    crumbs[0] = 6;
    xSemaphoreTake(flowDone, portMAX_DELAY);
    crumbs[0] = 7;
    if (flowJob.found && frameStart(frame) - lastCaptureUs <= 2 * config::framePeriodUs) {
        memcpy(packet.flow, flowJob.result.shift, sizeof(packet.flow));
        packet.flowIntervalUs = frameStart(frame) - lastCaptureUs;
        packet.flowResidualPx = flowJob.result.residualPx;
        packet.flowInliers = flowJob.result.inliers;
    }
    packet.flowOffsetUs = int16_t(frameCentreUs - packet.captureUs);
    const uint32_t now = uint32_t(esp_timer_get_time());
    packet.ageUs = now - packet.captureUs;
    if (packet.ageUs > config::maxAgeUs)
        packet.valid = 0;
    if (!packet.valid) {
        memset(packet.position, 0, sizeof(packet.position));
        memset(packet.rotation, 0, sizeof(packet.rotation));
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
    finishFrame(frame);
    crumbs[0] = 9;
    if (packet.sequence % 15 == 0)
        adjustExposure();
    crumbs[0] = 10;
}
