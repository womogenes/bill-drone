#include <Arduino.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "tag16h5.h"
#include "pose.h"
#include "ota.h"

#include "web.h"

PosePacket *wirePacket;
apriltag_detector_t *detector;
uint32_t sequence = 0;
spi_slave_transaction_t transfer = {};
bool transferPending = false;
bool cameraReady = false;
uint8_t *rgbFrame, *grayFrame;

// Driver callbacks run only after DMA is armed / the transfer has completed.
void IRAM_ATTR transferReady(spi_slave_transaction_t *) {
    gpio_set_level(gpio_num_t(config::readyPin), 1);
}

void IRAM_ATTR transferDone(spi_slave_transaction_t *) {
    gpio_set_level(gpio_num_t(config::readyPin), 0);
}

void setup() {
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
    camera_config_t camera = {};
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
    camera.pixel_format = PIXFORMAT_JPEG;
    camera.jpeg_quality = 10;
    camera.frame_size = FRAMESIZE_QVGA;
    camera.fb_count = 2;
    camera.fb_location = CAMERA_FB_IN_PSRAM;
    camera.grab_mode = CAMERA_GRAB_LATEST;
    const esp_err_t cameraError = esp_camera_init(&camera);
    if (cameraError != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x; OTA remains available.\n", cameraError);
        return;
    }
    sensor_t *sensor = esp_camera_sensor_get();
    Serial.printf("Camera sensor PID: 0x%04x\n", sensor->id.PID);
    // Toggle both axes from the previous orientation: a 180-degree rotation.
    ESP_ERROR_CHECK(sensor->set_hmirror(sensor, 0));
    ESP_ERROR_CHECK(sensor->set_vflip(sensor, 1));

    rgbFrame = static_cast<uint8_t *>(ps_malloc(320 * 240 * 3));
    grayFrame = static_cast<uint8_t *>(ps_malloc(320 * 240));
    assert(rgbFrame && grayFrame);
    detector = apriltag_detector_create();
    apriltag_detector_add_family_bits(detector, tag16h5_create(), 0);
    detector->nthreads = 1;
    detector->quad_decimate = 1.0f;
    detector->refine_edges = true;
    startPreview();
    cameraReady = true;
    Serial.println("AprilTag pose SPI ready: 320x240, tag16h5, mode 1.");
}

void loop() {
    serviceOta();
    if (!cameraReady) {
        delay(5);
        return;
    }
    web.handleClient();
    if (transferPending) {
        spi_slave_transaction_t *finished;
        const esp_err_t result = spi_slave_get_trans_result(HSPI_HOST, &finished, 0);
        if (result != ESP_ERR_TIMEOUT) {
            ESP_ERROR_CHECK(result);
            transferPending = false;
        }
    }
    camera_fb_t *frame = esp_camera_fb_get();
    assert(frame && frame->format == PIXFORMAT_JPEG);
    ++captureCount;
    PosePacket packet;
    packet.sequence = sequence++;
    packet.tagId = config::tagId;
    packet.captureUs =
        uint32_t(uint64_t(frame->timestamp.tv_sec) * 1000000 + frame->timestamp.tv_usec);
    image_u8_t image = {int(frame->width), int(frame->height), int(frame->width), grayFrame};
    if (calibrated()) {
        const bool decoded = fmt2rgb888(frame->buf, frame->len, PIXFORMAT_JPEG, rgbFrame);
        assert(decoded);
        for (int i = 0; i < 320 * 240; ++i)
            grayFrame[i] =
                (77 * rgbFrame[3 * i] + 150 * rgbFrame[3 * i + 1] + 29 * rgbFrame[3 * i + 2]) >> 8;
        ++poseCount;
        zarray_t *tags = apriltag_detector_detect(detector, &image);
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
        if (matches == 1 && target->hamming == 0 && target->decision_margin >= config::minMargin)
            estimatePose(target, packet);
        apriltag_detections_destroy(tags);
    }
    publishPreview(frame);
    esp_camera_fb_return(frame);
    packet.ageUs = uint32_t(esp_timer_get_time()) - packet.captureUs;
    packet.valid &= packet.ageUs <= config::maxAgeUs;
    if (!packet.valid) {
        memset(packet.position, 0, sizeof(packet.position));
        memset(packet.rotation, 0, sizeof(packet.rotation));
    }
    packet.checksum =
        crc16(reinterpret_cast<const uint8_t *>(&packet), offsetof(PosePacket, checksum));
    lastPose = packet;
    if (!transferPending) {
        memcpy(wirePacket, &packet, sizeof(packet));
        transfer = {};
        transfer.length = sizeof(packet) * 8;
        transfer.tx_buffer = wirePacket;
        // One outstanding result. READY is raised by the driver, then the host clocks
        // exactly 80 bytes. The host includes its polling delay in the measurement age.
        ESP_ERROR_CHECK(spi_slave_queue_trans(HSPI_HOST, &transfer, 0));
        transferPending = true;
    }
    delay(1);
}
