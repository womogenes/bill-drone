#include <Arduino.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/pcnt.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"
#include "tag16h5.h"
#include "pose.h"
#include "track.h"
#include "tag.h"
#include "flow.h"
#include "ota.h"
#include "clocks.h"
#include "update.h"
#include "air_update.h"

#include "web.h"
#include "record.h"
#include "sd.h"

TagFinder finder;
uint32_t sequence = 0;
bool cameraReady = false;
// Set by OTA; the detection loop stops the camera between frames, so OTA never has to win a lock against it.
volatile bool cameraStopRequested = false;
bool cameraStalled = false;
uint32_t lastCaptureUs = 0;

camera_config_t cameraConfig = {};
uint32_t lastStartMs = 0;
esp_err_t startError = ESP_OK;
uint32_t stalls = 0, recoveries = 0;
int exposureLines;
// The sensor's gain at the last exposure adjustment, for K S.
float exposureGain = 0;
// The longest exposure adjustExposure may reach (lines), set by the drone's K E and kept across restarts (Preferences "exposure"): a long exposure smears the tag when the drone rotates, and in dim light the exposure ran to the frame's 23 ms, where with the tag in view 73% of frames at 40-70 degrees/s of rotation lost it, against 8% under 20 (2026-11-02). Darker but sharp finds the tag; bright but smeared does not.
int maxExposureLines = config::vts - 4;
// What each core was doing, kept across a panic restart: /status shows the values from before the last reset.
RTC_NOINIT_ATTR uint32_t crumbs[3];
uint32_t lastCrumbs[3];

// The drone link on UART0 (config::linkTxPin and linkRxPin). The receive side carries firmware updates from the drone (update.h); its buffer holds a whole sector's request. The transmit buffer holds a few packets; publish() never queues behind an unsent one.
constexpr uart_port_t linkPort = UART_NUM_0;
constexpr int linkTxBytes = 1024, linkRxBytes = 6144;
volatile uint32_t linkSent = 0, linkDropped = 0, linkReceived = 0;

void noPrint(char) {}

// Link diagnostics (2026-11-14, the drone's requests not answered): a pulse counter shares the link's RX pin through the GPIO matrix and counts its falling edges, so a silent wire can be told from a receiver fault; each wrap at rxEdgeLimit is counted by its interrupt. linkLate counted packets written behind other bytes in the drone's radio relay (gone; it stays 0 in the reports), and startDelayUs the stamp-to-written time of the others (the start bit begins within it). They, with linkReceived, go to the drone in TAG3 packets without a trusted tag (packet.h altRotation) and answer update op Y.
constexpr pcnt_unit_t rxEdgeUnit = PCNT_UNIT_0;
constexpr int16_t rxEdgeLimit = 30000;
volatile uint32_t rxEdgeWraps = 0, linkLate = 0;
volatile uint32_t startDelaySumUs = 0, startDelayCount = 0, startDelayMaxUs = 0;

void IRAM_ATTR rxEdgeWrap(void *) {
    rxEdgeWraps = rxEdgeWraps + 1;
}

uint32_t rxEdges() {
    uint32_t wraps;
    int16_t count = 0;
    do {
        wraps = rxEdgeWraps;
        pcnt_get_counter_value(rxEdgeUnit, &count);
    } while (wraps != rxEdgeWraps);
    return wraps * uint32_t(rxEdgeLimit) + uint16_t(count);
}

void countRxEdges() {
    pcnt_config_t unit = {};
    unit.pulse_gpio_num = config::linkRxPin;
    unit.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    unit.lctrl_mode = PCNT_MODE_KEEP;
    unit.hctrl_mode = PCNT_MODE_KEEP;
    unit.pos_mode = PCNT_COUNT_DIS;
    unit.neg_mode = PCNT_COUNT_INC;
    unit.counter_h_lim = rxEdgeLimit;
    unit.counter_l_lim = -1;
    unit.unit = rxEdgeUnit;
    unit.channel = PCNT_CHANNEL_0;
    if (pcnt_unit_config(&unit) != ESP_OK)
        return;
    pcnt_filter_disable(rxEdgeUnit);
    pcnt_event_enable(rxEdgeUnit, PCNT_EVT_H_LIM);
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(rxEdgeUnit, rxEdgeWrap, nullptr);
    pcnt_counter_pause(rxEdgeUnit);
    pcnt_counter_clear(rxEdgeUnit);
    pcnt_counter_resume(rxEdgeUnit);
}

// A second receiver on the link's RX pin at a test baud (UART1 through the GPIO matrix, receive only), so the drone's bytes can be tried at rates below the link's 2 Mbaud while UART0 keeps sending at 2 Mbaud; counts bytes and those that are 0x55 (the drone's H burst).
constexpr uart_port_t testPort = UART_NUM_1;
bool testUart = false;
volatile uint32_t testBytes = 0, testPattern = 0;

void setTestBaud(uint32_t baud) {
    if (!baud)
        return;
    if (!testUart) {
        uart_config_t test = {};
        test.baud_rate = int(baud);
        test.data_bits = UART_DATA_8_BITS;
        test.parity = UART_PARITY_DISABLE;
        test.stop_bits = UART_STOP_BITS_1;
        test.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        test.source_clk = UART_SCLK_APB;
        if (uart_driver_install(testPort, 2048, 0, 0, nullptr, 0) != ESP_OK || uart_param_config(testPort, &test) != ESP_OK)
            return;
        uart_set_pin(testPort, UART_PIN_NO_CHANGE, config::linkRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        // As for the edge counter: the link's own pin setup again gives the pad back to UART0.
        uart_set_pin(linkPort, config::linkTxPin, config::linkRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        testUart = true;
    }
    uart_set_baudrate(testPort, baud);
    uart_flush_input(testPort);
}

void readTestUart() {
    if (!testUart)
        return;
    uint8_t bytes[256];
    int n;
    while ((n = uart_read_bytes(testPort, bytes, sizeof(bytes), 0)) > 0) {
        uint32_t pattern = 0;
        for (int i = 0; i < n; ++i)
            pattern += bytes[i] == 0x55;
        testBytes = testBytes + n;
        testPattern = testPattern + pattern;
    }
}

// DEBUG_CONSOLE (the debug build environment): UART0 is a 115200 console with ESP logs and a status line instead of the drone link; no packets are sent.
void startLink() {
#ifndef DEBUG_CONSOLE
    esp_rom_install_channel_putc(1, noPrint);
#endif
    uart_config_t link = {};
#ifdef DEBUG_CONSOLE
    link.baud_rate = 115200;
#else
    link.baud_rate = int(config::linkBaud);
#endif
    link.data_bits = UART_DATA_8_BITS;
    link.parity = UART_PARITY_DISABLE;
    link.stop_bits = UART_STOP_BITS_1;
    link.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    link.source_clk = UART_SCLK_APB;
    ESP_ERROR_CHECK(uart_driver_install(linkPort, linkRxBytes, linkTxBytes, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(linkPort, &link));
    ESP_ERROR_CHECK(uart_set_pin(linkPort, config::linkTxPin, config::linkRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // The old SYNC pin is never driven: on the drone its line is shorted to the drone's TX (2026-11-14), so driving it fought the drone's requests.
    if (!config::sdDat3IsSync)
        pinMode(config::syncPin, INPUT);
    // The counter's setup selects the pad's GPIO function; the UART's pin setup again afterwards gives the pad back to UART0 (its IO_MUX function where the driver uses it), while the counter keeps the pad's input through the matrix.
    countRxEdges();
    ESP_ERROR_CHECK(uart_set_pin(linkPort, config::linkTxPin, config::linkRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Wi-Fi, OTA and the control server share core 0 with the radio and the camera driver's copy task. Core 1 only captures and detects.
void serviceTask(void *) {
#if defined(DEBUG_CONSOLE) || CONFIG_IDF_TARGET_ESP32S3
    uint32_t printed = 0;
#endif
    while (true) {
#if CONFIG_IDF_TARGET_ESP32S3
        // The S3's own USB is free of the drone link: a status line every 2 s for a laptop on its USB-C.
        if (Serial && millis() - printed >= 2000) {
            printed = millis();
            Serial.printf("status: wifi %d ip %s rssi %d camera %d error 0x%x frames %u linkSent %u linkReceived %u stalls %u heap %u psram %u\n", int(WiFi.status()),
                          WiFi.localIP().toString().c_str(), int(WiFi.RSSI()), int(cameraReady), unsigned(startError), unsigned(sequence), unsigned(linkSent),
                          unsigned(linkReceived), unsigned(stalls), unsigned(ESP.getFreeHeap()), unsigned(ESP.getFreePsram()));
        }
#endif
#ifdef DEBUG_CONSOLE
        if (millis() - printed >= 2000) {
            printed = millis();
            printf("status: wifi %d ip %s rssi %d camera %d linkSent %u stalls %u heap %u psram %u\n", int(WiFi.status()), WiFi.localIP().toString().c_str(),
                   int(WiFi.RSSI()), int(cameraReady), unsigned(linkSent), unsigned(stalls), unsigned(ESP.getFreeHeap()), unsigned(ESP.getFreePsram()));
        }
#endif
        serviceOta();
        fpv::poll();
        web.handleClient();
#ifndef DEBUG_CONSOLE
        uint8_t received[512];
        int n;
        while ((n = uart_read_bytes(linkPort, received, sizeof(received), 0)) > 0) {
            linkReceived = linkReceived + n;
            update::receive(linkPort, received, n);
        }
        readTestUart();
        update::poll();
        air::poll();
#endif
        delay(2);
    }
}

uint32_t frameStart(const camera_fb_t *frame) {
    return uint32_t(uint64_t(frame->timestamp.tv_sec) * 1000000 + frame->timestamp.tv_usec);
}

// The driver's queue is first in, first out, and with two buffers it stops capturing while the detector holds one and the other waits finished. That keeps the camera DMA copy from competing with the detector for PSRAM, but the waiting frame may have finished long ago. Such a frame is returned and the next one, started afterwards, is used instead.
UBaseType_t driverPriority = 0; // as the driver created its task, for /status

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

// Starts the camera: the driver, then mirror and flip, the clocks (clocks.h), and on the OV3660 the window of config.h. The driver's raw-QVGA clock (20 MHz SYSCLK, 11 fps) becomes its small-frame one, 40 MHz SYSCLK with the same 10 MHz PCLK; a faster line rate panicked within minutes. FPV takes the driver's JPEG mode as it comes, its PLL kept whatever the XCLK.
bool startCamera(bool jpeg) {
    cameraConfig.pixel_format = jpeg ? PIXFORMAT_JPEG : PIXFORMAT_GRAYSCALE;
    fpv::large = jpeg && fpv::radioWanted() && fpv::radioBudget >= config::fpvLargeBudget;
    cameraConfig.frame_size = !jpeg ? FRAMESIZE_QVGA : fpv::large ? config::fpvLargeFrameSize : config::fpvFrameSize;
    // XCLK, the sensor's PLL and the pad drives come from clocks.h (the drone's K C may override them until restart).
    const clocks::Plan clock = clocks::plan(jpeg);
    cameraConfig.xclk_freq_hz = int(clock.integerHz);
    cameraConfig.jpeg_quality = fpvQuality;
    startError = esp_camera_init(&cameraConfig);
    if (startError != ESP_OK)
        return false;
    sensor_t *sensor = esp_camera_sensor_get();
    const bool configured =
        sensor->set_hmirror(sensor, 0) == 0 && sensor->set_vflip(sensor, 1) == 0 &&
        clocks::apply(clock, jpeg, sensor, cameraConfig.pin_xclk) &&
        (jpeg || sensor->id.PID != OV3660_PID ||
         (sensor->set_res_raw(sensor, (2048 - config::windowW) / 2, (1536 - config::windowH) / 2,
                              (2048 + config::windowW) / 2 + 31, (1536 + config::windowH) / 2 + 11, 8, 2, config::hts,
                              config::vts, config::outW, config::outH, true, true) == 0 &&
          (config::rowSkip == 1 || sensor->set_reg(sensor, 0x3815, 0xff, 0x71) == 0)));
    if (!configured) {
        esp_camera_deinit();
        return false;
    }
    clocks::readBack(jpeg, sensor, cameraConfig.pin_xclk);
    // The driver's task must empty a 12-line DMA block every 0.7 ms or lose the frame. It is created at the same priority as the Wi-Fi task on core 0, so Wi-Fi traffic (the preview, /status, records) cost frames; above it, its short copies only delay Wi-Fi.
    if (TaskHandle_t driver = xTaskGetHandle("cam_task")) {
        driverPriority = uxTaskPriorityGet(driver);
        vTaskPrioritySet(driver, configMAX_PRIORITIES - 1);
    }
    if (jpeg)
        return true;
    // Exposure is ours (adjustExposure), gain stays automatic up to its 64x ceiling.
    exposureLines = config::exposureLines;
    return sensor->set_exposure_ctrl(sensor, 0) == 0 && sensor->set_aec_value(sensor, exposureLines) == 0 &&
           sensor->set_reg(sensor, 0x3a18, 0xff, 0x03) == 0 && sensor->set_reg(sensor, 0x3a19, 0xff, 0xff) == 0;
}

// Short exposures keep motion blur down; the automatic gain makes up the light. Only when the gain nears its ceiling does the exposure grow, up to the whole frame, and it shrinks back as the gain falls.
void adjustExposure() {
    sensor_t *sensor = esp_camera_sensor_get();
    const float gain = sensor->get_reg(sensor, 0x350a, 0x3ff) / 16.0f;
    exposureGain = gain;
    int lines = exposureLines;
    if (gain > config::raiseExposureGain)
        lines = min(maxExposureLines, lines * 5 / 4);
    else if (gain < config::lowerExposureGain)
        lines = max(min(config::exposureLines, maxExposureLines), lines * 4 / 5);
    lines = min(lines, maxExposureLines);
    if (lines != exposureLines && sensor->set_aec_value(sensor, lines) == 0)
        exposureLines = lines;
}

// The detector's working set is PSRAM-bound (union-find alone is 8 bytes per pixel). A window is small enough to keep in internal RAM; the limit is global, so it stays at the default while the preview encoder is running too. A whole-frame search's many small allocations (clusters, quads) took internal RAM down to 7.7 KB, so they all go to PSRAM; Wi-Fi and lwip allocate internal RAM explicitly and are unaffected.
void detectorWorkspace(Workspace where) {
    heap_caps_malloc_extmem_enable(where == workspaceInternal && !previewStreaming ? config::trackInternalBytes
                                   : where == workspaceExternal                    ? 0
                                                                                   : 4096);
}

// Sets the exposure cap (lines; 0 = the whole frame) and keeps it.
void setExposureCap(int lines) {
    maxExposureLines = lines > 0 ? constrain(lines, 20, config::vts - 4) : config::vts - 4;
    Preferences store;
    if (store.begin("exposure", false)) {
        store.putUShort("max", uint16_t(maxExposureLines));
        store.end();
    }
}

void loadExposureCap() {
    Preferences store;
    if (store.begin("exposure", true)) {
        maxExposureLines = constrain(int(store.getUShort("max", config::vts - 4)), 20, config::vts - 4);
        store.end();
    }
}

void setup() {
    memcpy(lastCrumbs, crumbs, sizeof(crumbs));
    memset(crumbs, 0, sizeof(crumbs));
#if CONFIG_IDF_TARGET_ESP32S3
    Serial.begin(115200);
#endif
    // UART0 is the drone link from here on, so nothing else may write to it: log output goes to the /log ring and the ROM's printf to nowhere. Only the boot ROM and bootloader print, at 115200 before this, and the drone rejects that.
#ifdef DEBUG_CONSOLE
    esp_log_level_set("*", ESP_LOG_INFO);
    startLink();
    printf("debug console: booted, reset reason %d\n", int(esp_reset_reason()));
#else
    esp_log_set_vprintf(logToRing);
    esp_log_level_set("*", ESP_LOG_WARN);
    startLink();
#endif
    pinMode(config::flashLedPin, OUTPUT);
    digitalWrite(config::flashLedPin, LOW);
    health::countBoot();
    health::watchWifi();
    startWifi();
    if (wifiConfigured())
        fpv::start();
    // Updates over the FPV receiver's ESP-NOW (air_update.h), wherever the radio is up.
    if (WiFi.getMode() != WIFI_MODE_NULL)
        air::start();
    assert(psramFound());
    heap_caps_malloc_extmem_enable(4096);
    // Mounting leaves the sync line low (on the ESP32-CAM, the card's DAT3, held high until the card is up).
    blackbox::mount();
    loadCalibration();
    loadExposureCap();

    // The camera driver identifies the sensor.
    camera_config_t &camera = cameraConfig;
#if CONFIG_IDF_TARGET_ESP32S3
    // NULLLAB ESP32S3-CAM: the sensor's power-down is tied on through 1 kohm and its reset is an RC on the board, so neither is a pin.
    camera.pin_pwdn = -1;
    camera.pin_reset = -1;
    camera.pin_xclk = 15;
    camera.pin_sccb_sda = 4;
    camera.pin_sccb_scl = 5;
    camera.pin_d0 = 11;
    camera.pin_d1 = 9;
    camera.pin_d2 = 8;
    camera.pin_d3 = 10;
    camera.pin_d4 = 12;
    camera.pin_d5 = 18;
    camera.pin_d6 = 17;
    camera.pin_d7 = 16;
    camera.pin_vsync = 6;
    camera.pin_href = 7;
    camera.pin_pclk = 13;
#else
    // AI Thinker ESP32-CAM.
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
#endif
    camera.xclk_freq_hz = int(config::tagXclkHz);
    camera.ledc_timer = LEDC_TIMER_0;
    camera.ledc_channel = LEDC_CHANNEL_0;
    // Tracking takes raw Y8 (startCamera): no JPEG decode on the detector and no compression artefacts on tag edges.
    // One for the detector, one for the driver; only the full-resolution preview holds a frame, for its copy.
    camera.fb_count = 2;
    camera.fb_location = CAMERA_FB_IN_PSRAM;
    camera.grab_mode = CAMERA_GRAB_LATEST;

    apriltag_detector_t *detector = finder.detector = apriltag_detector_create();
    finder.family = tag16h5_create();
    apriltag_detector_add_family_bits(detector, finder.family, 0);
    detector->nthreads = 1;
    detector->refine_edges = true;
    // A tag is high-contrast black on white: reject faint and tiny blobs at thresholding, before the expensive union-find, clustering and quad fitting, which otherwise scale with every noise blob in a dim, cluttered scene.
    detector->qtp.min_white_black_diff = 20;
    detector->qtp.max_nmaxima = 6;
    finder.workspace = detectorWorkspace;
    startPreview();
    startRecording();
    blackbox::start();
    flowDone = xSemaphoreCreateBinary();
    assert(flowDone);
    xTaskCreatePinnedToCore(flowLoop, "flow", 4096, nullptr, 2, &flowTask, 0);
    // A camera that does not start is tried again from the loop; Wi-Fi, the web server and the drone link run regardless.
    lastStartMs = millis();
    cameraReady = startCamera(false);
    cameraStalled = !cameraReady;
    xTaskCreatePinnedToCore(serviceTask, "service", 8192, nullptr, 1, nullptr, 0);
}

// Each frame's packet goes out on UART0 at 2 Mbaud, 164 bytes in 0.8 ms. The drone dates it by the falling edge of its first start bit (fw/drone/main/camera.c), so the packet's age is stamped just before it is written to an idle UART: the driver's transmit interrupt (on this core) fills the FIFO at once and the start bit follows the stamp by the time it takes to write (startDelayUs, a few tens of microseconds). The UART is waited for up to linkIdleWaitUs (an update reply is 0.3 ms); if it is still busy the packet is dropped. Until 2026-11-14 a SYNC pulse on syncPin dated it; the drone's end of that line is now shorted to its TX, so syncPin is never driven (startLink).
constexpr uint32_t linkIdleWaitUs = 1000;

bool linkIdle() {
    size_t room = 0;
    uart_get_tx_buffer_free_size(linkPort, &room);
    return room >= size_t(linkTxBytes) && uart_wait_tx_done(linkPort, 0) == ESP_OK;
}

void publish(PosePacket &packet) {
    const uint32_t waited = uint32_t(esp_timer_get_time());
    bool idle;
    while (!(idle = linkIdle()) && uint32_t(esp_timer_get_time()) - waited < linkIdleWaitUs) {
    }
    if (!idle) {
        ++linkDropped;
        return;
    }
    if (!packet.trusted || uint32_t(esp_timer_get_time()) - packet.captureUs > config::maxAgeUs) {
        packet.valid = packet.trusted = 0;
        memset(packet.position, 0, sizeof(packet.position));
        memset(packet.rotation, 0, sizeof(packet.rotation));
        memset(packet.tagPosition, 0, sizeof(packet.tagPosition));
        memset(packet.altPosition, 0, sizeof(packet.altPosition));
        memset(packet.altRotation, 0, sizeof(packet.altRotation));
        // The link diagnostics, where a frame without a trusted tag has no alternative pose (packet.h).
        const uint32_t report[5] = {linkReceived, rxEdges(), linkLate, testBytes, testPattern};
        memcpy(packet.altRotation, report, sizeof(report));
    }
    const uint32_t now = uint32_t(esp_timer_get_time());
    packet.ageUs = now - packet.captureUs;
    packet.checksum = crc16(reinterpret_cast<const uint8_t *>(&packet), offsetof(PosePacket, checksum));
#ifndef DEBUG_CONSOLE
    uart_write_bytes(linkPort, &packet, sizeof(packet));
#endif
    const uint32_t delay = uint32_t(esp_timer_get_time()) - now;
    ++linkSent;
    startDelaySumUs = startDelaySumUs + delay;
    startDelayCount = startDelayCount + 1;
    if (delay > startDelayMaxUs)
        startDelayMaxUs = delay;
}

// Restarts the camera in the mode wanted (fpvActive).
void restartCamera() {
    esp_camera_deinit();
    cameraReady = false;
    fpv::useLargeParity(false);
    lastStartMs = millis();
    for (int attempt = 0; attempt < 3 && !cameraReady; ++attempt) {
        // After a failed start the sensor is held in power-down (where the board has the pin) for a second before the next try. A sensor that stopped answering its probe on 2026-10-30 recovered only from a power cycle; whether power-down clears that state is untested.
        if (startError != ESP_OK) {
            if (cameraConfig.pin_pwdn >= 0) {
                pinMode(cameraConfig.pin_pwdn, OUTPUT);
                digitalWrite(cameraConfig.pin_pwdn, HIGH);
            }
            delay(1000);
            // A JPEG start that the sensor refused (ESP_FAIL, its settings not taken) kept failing until a grayscale start had set it up again (2026-10-30), so one goes first.
            if (fpvActive && startCamera(false))
                esp_camera_deinit();
        } else if (attempt) {
            delay(200);
        }
        cameraReady = startCamera(fpvActive);
    }
    cameraStalled = !cameraReady;
    fpv::useLargeParity(fpvActive && cameraReady);
    finder.tracking = false;
    flow::primed = false;
}

void loop() {
    if (cameraStopRequested && cameraReady) {
        esp_camera_deinit();
        cameraReady = false;
    }
    // A camera that failed to start (in either mode) is tried again every second.
    const bool retry = !cameraReady && millis() - lastStartMs >= 1000;
    // K C (update.h) restarts the camera in its mode for new clocks.
    if (clocks::restartWanted && cameraReady && !cameraStopRequested) {
        clocks::restartWanted = false;
        restartCamera();
        fpv::nativeLines = 0;
        fpv::lastCaptureUs = 0;
        return;
    }
    if (!cameraStopRequested && (retry || (cameraReady && fpv::wanted() != fpvActive))) {
        fpvActive = fpv::wanted();
        esp_camera_deinit();
        cameraReady = false;
        if (fpv::shareRadio())
            delay(300);
        restartCamera();
        fpv::nativeLines = 0;
        fpv::lastCaptureUs = 0;
        recoveries += retry && cameraReady;
        return;
    }
    if (!cameraReady) {
        delay(100);
        return;
    }
    if (fpvActive) {
        if (!fpv::send()) {
            ++stalls;
            restartCamera();
            recoveries += cameraReady;
        }
        return;
    }
    crumbs[0] = 1;
    camera_fb_t *frame = freshFrame();
    // A driver that stops delivering frames is restarted, up to three tries; Wi-Fi and OTA stay up regardless, and /status counts stalls and recoveries.
    if (!frame) {
        ++stalls;
        crumbs[2] = stalls;
        restartCamera();
        recoveries += cameraReady;
        return;
    }
    crumbs[0] = 2;
#ifdef DEBUG_CONSOLE
    // An F on the console dumps this frame: FRAME, width and height (u16), then the grayscale pixels.
    uint8_t request = 0;
    if (uart_read_bytes(linkPort, &request, 1, 0) == 1 && request == 'F') {
        const uint16_t size[2] = {uint16_t(frame->width), uint16_t(frame->height)};
        uart_write_bytes(linkPort, "FRAME", 5);
        uart_write_bytes(linkPort, size, sizeof(size));
        uart_write_bytes(linkPort, frame->buf, frame->len);
        uart_wait_tx_done(linkPort, portMAX_DELAY);
    }
#endif
    const uint32_t started = uint32_t(esp_timer_get_time());
    ++captureCount;
    PosePacket packet;
    packet.sequence = sequence++;
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
        kind = finder.find(frame->buf, frame->width, frame->height, frameStart(frame), corners, margin);
        trackedCount += kind == tagTracked;
        detectedCount += kind == tagDetected;
        if (kind)
            packet.tagId = uint16_t(finder.id);
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
    publish(packet);
    const uint32_t now = packet.captureUs + packet.ageUs;
    lastPose = packet;
    detectUs = now - started;
    readoutAgeUs = now - frameStart(frame);
    frameUs = frameStart(frame) - lastCaptureUs;
    lastCaptureUs = frameStart(frame);
    crumbs[0] = 8;
    if (recording() || blackbox::active) {
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
        if (recording())
            addRecord(record);
        // The black box takes the tag crop mostly from the tracker's internal copy, and the reduced frame the flow left in internal RAM; only a full frame (every fullEvery-th) is a large PSRAM read.
        const Image8 &copy = finder.measuredImage;
        const bool inWindow = copy.pixels == finder.window;
        const blackbox::CropSource crop = {frame->buf, int(frame->width), int(frame->height), inWindow ? finder.window : nullptr,
                                           copy.left, copy.top, copy.width, copy.height};
        const bool full = blackbox::fullEvery && packet.sequence % blackbox::fullEvery == 0 && frame->len == config::outW * config::outH;
        blackbox::addFrame(record, flow::images[flow::current], kind ? &crop : nullptr, finder.corners, full ? frame->buf : nullptr);
    }
    finishFrame(frame, kind != tagNone, finder.corners, packet, flow::images[flow::current]);
    crumbs[0] = 9;
    if (packet.sequence % 15 == 0)
        adjustExposure();
    crumbs[0] = 10;
}
