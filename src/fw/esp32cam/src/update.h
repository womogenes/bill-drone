#pragma once
#include <Arduino.h>
#include "driver/uart.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "mbedtls/sha256.h"
#include "packet.h"
#include "clocks.h"
extern bool cameraReady, cameraStalled;
extern int exposureLines, maxExposureLines;
extern float exposureGain;
void setExposureCap(int lines);
extern volatile bool fpvActive;
extern volatile uint32_t fpvRadioMs;
namespace fpv {
extern esp_err_t rawError;
extern uint32_t rawSent;
extern float radioBudget, radioShardBudget;
extern volatile uint32_t previewRadioMs;
extern wifi_phy_rate_t rateOverride;
extern int nativeLines, frameLines, windowRegisters[7], gapOverrideUs, parityOverride, parityCapacity;
extern int qualityEvery;
extern uint32_t qualityChanges;
extern uint32_t rawFailed, rawRetries, framesSkipped, sendLongestUs, sendTotalUs, sendFrames;
extern float measuredPeriodUs;
}
extern char logRing[2048];
extern volatile uint32_t linkReceived, syncTestPulses, testBytes, testPattern;
extern volatile uint8_t syncMode;
uint32_t rxEdges();
void syncTest(uint32_t count);
void setSyncMode(uint8_t mode, uint32_t seconds);
void setTestBaud(uint32_t baud);
extern size_t logHead;
extern esp_err_t startError;
extern volatile bool cameraStopRequested;
namespace blackbox {
const char *control(uint16_t every, uint8_t mode, char *text, size_t size);
}

// Firmware updates through the drone's radio link, for when the camera cannot reach Wi-Fi (fw/tools/ota_direct.py camera): the drone forwards each request on the link's receive side and returns the reply. S reports the Wi-Fi network, address and camera state; N the camera's MAC and name; V asks for FPV over raw 802.11 (fpv.h) for another few seconds, optionally with the receiver's byte budget; P for tag mode's preview over it (web.h), tracking on; L reads the log 50 characters at a time; A starts the access point (ota.h) until the camera restarts; I describes the slots (running label, spare label and size, and the first 8 bytes of the running image's SHA-256, which the build appends to the image); W writes one 4 KB sector of the spare slot and reads it back, after checking its hash; B selects the spare slot, which checks the whole image, and restarts into it; X abandons the update and restarts; J restarts joined to the update network for one boot (ota.h), for a Wi-Fi update; R sets the drone's radio relay mode (relay.h), restarting into a changed one; D controls the black box on the microSD (sd.h); F reports FPV's sending (frames skipped, failed sends, send time) and G the sensor's readout for the rolling shutter; Q overrides FPV's PHY rate, shard gap and parity for tests; C overrides the image sensor's clocks and drives (clocks.h) until restart; Y reports the link diagnostics (main.cpp rxEdges), after emitting SYNC pulses when asked. The first W or B stops the camera, as a Wi-Fi update does, and a camera left stopped with no request for abandonUs restarts. Requests come one at a time, each after the last reply, so nothing arrives while the flash is being written.
namespace update {
// Requests answered, and ones that failed their checksum or length, for K S.
uint32_t goodRequests = 0, badRequests = 0;
constexpr size_t sector = 4096, hashBytes = 8, maxBody = 2 + hashBytes + sector;
constexpr uint32_t abandonUs = 30000000;

uint8_t frame[sizeof(UpdateRequest) + maxBody + 2];
size_t have = 0;
bool stopped = false, restarting = false;
uint32_t lastUs = 0;

void stopCamera() {
    if (stopped)
        return;
    cameraStopRequested = true;
    while (cameraReady)
        delay(10);
    stopped = true;
}

const char *handle(uint8_t op, const uint8_t *body, size_t length, char *text, size_t size) {
    if (op == 'R') {
        // The drone's radio mode (relay.h): 0 for this board's own Wi-Fi, or 1, the drone's MAC and its controller's for the relay. A change is kept and the camera restarts into it after the reply; the reply says "restart" then, so the drone keeps asking until it reads the mode back without.
        if (length < 1 || body[0] > 1 || (body[0] && length < 13))
            return "Expected 0, or 1 and two MACs";
        const bool wanted = body[0];
        const bool changed = wanted != relay::on || (wanted && (memcmp(body + 1, relay::own, 6) || memcmp(body + 7, relay::peer, 6)));
        if (changed) {
            relay::save(wanted, body + 1, body + 7);
            restarting = true;
        }
        snprintf(text, size, "relay %d%s", int(wanted), changed ? " restart" : wanted && !relay::active ? " failed" : "");
        return nullptr;
    }
    // FPV, the radio preview and the access point would leave the link's channel or take its airtime.
    if (relay::on && op && strchr("HAVPQ", op))
        return "Not while relaying the drone's radio";
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
    if (op == 'I') {
        // Not the app description's ELF hash, which these builds do not keep in step with the image.
        uint8_t s[32] = {};
        esp_partition_get_sha256(running, s);
        snprintf(text, size, "%s %s %u %02x%02x%02x%02x%02x%02x%02x%02x", running->label, target ? target->label : "none",
                 unsigned(target ? target->size : 0), s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]);
        return nullptr;
    }
    if (op == 'S') {
        wifiState(text, size);
        const size_t used = strlen(text);
        // Then the exposure (lines of config::rowUs-ish 57.5 us) and the gain the tracking runs at.
        snprintf(text + used, size - used, " %s 0x%x %u %u/%u exp %d/%d gain %.0f", cameraStalled ? "stalled" : !cameraReady ? "stopped" : fpvActive ? "fpv" : "running", unsigned(startError), unsigned(fpv::rawSent), unsigned(goodRequests), unsigned(badRequests), exposureLines, maxExposureLines, exposureGain);
        return nullptr;
    }
    if (op == 'F') {
        // FPV's sending: frames sent, those the driver captured that were never sent, raw sends failed after retries, retries for a full queue, the average and longest send of a frame since the last F (ms), the parity shards it has room for, the internal RAM free (KB) and the JPEG quality changes.
        snprintf(text, size, "f%u sk%u x%u r%u s%.1f/%.1f p%d i%u q%u", unsigned(fpv::sendFrames), unsigned(fpv::framesSkipped), unsigned(fpv::rawFailed), unsigned(fpv::rawRetries),
                 fpv::sendFrames ? fpv::sendTotalUs / 1000.0f / fpv::sendFrames : 0.0f, fpv::sendLongestUs / 1000.0f, fpv::parityCapacity,
                 unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024), unsigned(fpv::qualityChanges));
        fpv::sendTotalUs = fpv::sendLongestUs = fpv::sendFrames = 0;
        return nullptr;
    }
    if (op == 'G') {
        // The sensor's FPV readout (fpv.h windowRegisters), for the rolling shutter: window rows, row increments, binning, output rows, HTS, VTS at FPV's start and now, and the frame period measured (us); the first row to the last takes (rows read) x period / VTS.
        const int *w = fpv::windowRegisters;
        snprintf(text, size, "y%d-%d i%02x b%02x%02x o%d h%d v%d/%d p%.0f", w[0], w[1], w[2], w[3], w[4], w[5], w[6], fpv::nativeLines, fpv::frameLines, fpv::measuredPeriodUs);
        return nullptr;
    }
    if (op == 'N') {
        // The camera's MAC and name (ota.h cameraNames): the dashboard takes only its own drone's camera's FPV, by that MAC.
        snprintf(text, size, "%02x:%02x:%02x:%02x:%02x:%02x %s", cameraMac[0], cameraMac[1], cameraMac[2], cameraMac[3], cameraMac[4], cameraMac[5], cameraName);
        return nullptr;
    }
    if (op == 'L') {
        // 50 characters of the log ring (web.h /log), the page counted back from the end by the body's byte.
        const size_t end = logHead - min(logHead, size_t(length ? body[0] : 0) * 50), start = end - min(end, size_t(50));
        for (size_t i = start; i < end; ++i)
            text[i - start] = logRing[i % sizeof(logRing)] >= ' ' ? logRing[i % sizeof(logRing)] : '|';
        text[end - start] = 0;
        return nullptr;
    }
    if (op == 'E') {
        // The exposure cap in lines (u16; 0 = the whole frame), kept across restarts; the reply is the cap now in force.
        if (length < 2)
            return "Malformed exposure cap";
        setExposureCap(body[0] | body[1] << 8);
        snprintf(text, size, "%d", maxExposureLines);
        return nullptr;
    }
    if (op == 'D') {
        // The black box on the microSD (sd.h control): every Nth frame's whole image (u16, 0 none), then 0 stop, 1 record or 2 a new file; for visual-inertial datasets (sw/vio/camera_sd.py).
        if (length < 3)
            return "Expected full_every (u16) and mode";
        return blackbox::control(body[0] | body[1] << 8, body[2], text, size);
    }
    if (op == 'H') {
        // Radio channel for FPV's raw frames (1-13), kept across restarts (ota.h saveRadio); the receiver (sw/fpv_monitor.py) searches for it.
        if (length < 1 || body[0] < 1 || body[0] > 13) return "Channel 1-13";
        radioChannel = body[0];
        esp_err_t error = esp_wifi_set_channel(radioChannel, WIFI_SECOND_CHAN_NONE);
        if (!error)
            saveRadio();
        snprintf(text, size, "ch%d", radioChannel);
        return error ? esp_err_to_name(error) : nullptr;
    }
    if (op == 'T') {
        // Transmit power in quarter dBm (8-84: 2-21 dBm), kept across restarts and applied once Wi-Fi is up (ota.h).
        if (length < 1 || body[0] < 8 || body[0] > 84) return "Power 8-84 (quarter dBm)";
        esp_err_t error = esp_wifi_set_max_tx_power(body[0]);
        if (!error) {
            radioPower = body[0];
            saveRadio();
        }
        int8_t power = 0;
        esp_wifi_get_max_tx_power(&power);
        snprintf(text, size, "%.2f dBm", power / 4.0);
        return error ? esp_err_to_name(error) : nullptr;
    }
    if (op == 'Y') {
        // Link diagnostics, all optional: a u16 count of SYNC pulses to emit (syncTest, at most 20000); a SYNC mode (main.cpp syncMode) and u16 seconds to keep it (at most 600), then the mode reverts to 0; a u32 baud for the test receiver on the RX pin (setTestBaud). The mode is set before the pulses. Answers bytes received on the link, falling edges on its RX pin, SYNC test pulses so far, requests answered and refused, the RX and SYNC pins' levels now, the SYNC mode, and the test receiver's bytes and 0x55 bytes.
        const uint32_t pulses = length >= 2 ? body[0] | body[1] << 8 : 0;
        if (pulses > 20000)
            return "At most 20000 pulses";
        if (length >= 5) {
            const uint32_t seconds = body[3] | body[4] << 8;
            if (body[2] > 2 || seconds > 600)
                return "Mode 0-2 for at most 600 s";
            setSyncMode(body[2], seconds);
        }
        if (length >= 9)
            setTestBaud(body[5] | body[6] << 8 | body[7] << 16 | uint32_t(body[8]) << 24);
        syncTest(pulses);
        snprintf(text, size, "rx%u e%u s%u g%u b%u l%d%d m%d t%u/%u", unsigned(linkReceived), unsigned(rxEdges()), unsigned(syncTestPulses), unsigned(goodRequests), unsigned(badRequests),
                 gpio_get_level(gpio_num_t(config::linkRxPin)), gpio_get_level(gpio_num_t(config::syncPin)), int(syncMode), unsigned(testPattern), unsigned(testBytes));
        return nullptr;
    }
    if (op == 'A') {
        startAp();
        return nullptr;
    }
    if (op == 'Q') {
        // A PHY rate (wifi_phy_rate_t) for every raw frame until the camera restarts, or 0xff for each frame's own; replies with the rate, frames sent and the last send's error.
        // Then optionally the gap between a frame's shards (us, u16, 0xffff for the frame's own), the parity shards (0xff for the frame's own) and how many frames apart the JPEG quality may change, also until the camera restarts.
        if (length < 1 || (body[0] != 0xff && body[0] >= WIFI_PHY_RATE_MAX)) return "Rate: a wifi_phy_rate_t, or 0xff for none";
        fpv::rateOverride = body[0] == 0xff ? WIFI_PHY_RATE_MAX : wifi_phy_rate_t(body[0]);
        if (length >= 3)
            fpv::gapOverrideUs = (body[1] | body[2] << 8) == 0xffff ? -1 : body[1] | body[2] << 8;
        if (length >= 4)
            fpv::parityOverride = body[3] == 0xff ? -1 : body[3];
        if (length >= 5)
            fpv::qualityEvery = max(1, int(body[4]));
        snprintf(text, size, "rate 0x%02x sent %u error %s", unsigned(fpv::rateOverride), unsigned(fpv::rawSent), esp_err_to_name(fpv::rawError));
        return nullptr;
    }
    if (op == 'C') {
        // The sensor's clocks (clocks.h): mode (0 tag, 1 FPV), then XCLK (Hz, u32, 0 for the mode's default), flags (bit 0 allows a fractional divider), the XCLK pin's drive (0-3, 0xff the default) and the sensor pads' (0-3, 0xff the driver's), then optionally the PLL multiplier (0 keeps the mode's PLLCLK), prediv index and PCLK divider (0 the mode's own). Kept until restart; the camera restarts now to take them in its current mode, the other mode at its next start. With only the mode it describes that mode's clocks (clocks::describe), with nothing the current mode's.
        const bool jpeg = length ? body[0] != 0 : fpvActive;
        if (length > 1) {
            if (length < 8)
                return "Expected mode, xclk u32, flags, pin and pad drive";
            const uint32_t hz = body[1] | body[2] << 8 | body[3] << 16 | uint32_t(body[4]) << 24;
            if (hz && (hz < 6000000 || hz > 27000000))
                return "XCLK 6-27 MHz, or 0";
            if ((body[6] != 0xff && body[6] > 3) || (body[7] != 0xff && body[7] > 3))
                return "Drives 0-3, or 0xff";
            if (length >= 10 && body[8] && (body[8] > 31 || body[9] > 3))
                return "Multiplier 1-31, prediv 0-3";
            if (length >= 11 && body[10] > 31)
                return "PCLK divider 0-31";
            clocks::Setting &setting = clocks::settings[jpeg];
            setting.xclkHz = hz;
            setting.fractional = body[5] & 1;
            setting.pinDrive = body[6] == 0xff ? -1 : body[6];
            setting.padDrive = body[7] == 0xff ? -1 : body[7];
            setting.multiplier = length >= 10 ? body[8] : 0;
            setting.prediv = length >= 10 ? body[9] : 0;
            setting.pclkDivider = length >= 11 ? body[10] : 0;
            clocks::appliedValid[jpeg] = false;
            clocks::restartWanted = jpeg == fpvActive;
        }
        clocks::describe(jpeg, text, size);
        return nullptr;
    }
    if (op == 'P') {
        fpv::previewRadioMs = millis();
        return nullptr;
    }
    if (op == 'V') {
        fpvRadioMs = millis();
        // The receiver's byte budget (KB/s, little-endian), when the request carries one, then its shard budget (shards/s), when it carries that too.
        if (length >= 2)
            fpv::radioBudget = max(20, body[0] | body[1] << 8) * 1000.0f;
        fpv::radioShardBudget = length >= 4 ? max(20, body[2] | body[3] << 8) : config::fpvRadioShardsPerSecond;
        return nullptr;
    }
    if (op == 'X') {
        restarting = true;
        return nullptr;
    }
    if (op == 'J') {
        // Restart joined to the update network for one boot (ota.h updateNetwork), for make camera-fast-wifi.
        if (!updateNetwork.ssid[0])
            return "No update network built in";
        requestUpdateJoin();
        restarting = true;
        return nullptr;
    }
    if (!target)
        return "No spare slot";
    if (op == 'W') {
        if (length != maxBody)
            return "Malformed sector write";
        const uint32_t index = body[0] | body[1] << 8;
        const uint8_t *data = body + 2 + hashBytes;
        uint8_t digest[32];
        mbedtls_sha256_ret(data, sector, digest, 0);
        if (memcmp(digest, body + 2, hashBytes))
            return "Sector arrived corrupted";
        if ((index + 1) * sector > target->size)
            return "Sector outside the slot";
        stopCamera();
        esp_err_t error = esp_partition_erase_range(target, index * sector, sector);
        if (!error)
            error = esp_partition_write(target, index * sector, data, sector);
        uint8_t check[256];
        for (size_t offset = 0; !error && offset < sector; offset += sizeof(check)) {
            error = esp_partition_read(target, index * sector + offset, check, sizeof(check));
            if (!error && memcmp(check, data + offset, sizeof(check)))
                return "Sector readback mismatch";
        }
        return error ? esp_err_to_name(error) : nullptr;
    }
    if (op == 'B') {
        stopCamera();
        const esp_err_t error = esp_ota_set_boot_partition(target);
        if (error)
            return esp_err_to_name(error);
        restarting = true;
        return nullptr;
    }
    return "Unknown update request";
}

void answer(uart_port_t port, uint32_t id, const char *failure, const char *text) {
    UpdateReply reply;
    reply.id = id;
    reply.status = failure != nullptr;
    const char *shown = failure ? failure : text;
    reply.length = uint8_t(strnlen(shown, sizeof(reply.text)));
    memcpy(reply.text, shown, reply.length);
    reply.checksum = crc16(reinterpret_cast<const uint8_t *>(&reply), offsetof(UpdateReply, checksum));
    uart_write_bytes(port, &reply, sizeof(reply));
    if (restarting) {
        uart_wait_tx_done(port, pdMS_TO_TICKS(100));
        delay(50);
        ESP.restart();
    }
}

// Feeds the link's received bytes through: a request whose magic, length and CRC check out is handled and answered; anything else is skipped.
void receive(uart_port_t port, const uint8_t *bytes, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        const uint8_t b = bytes[i];
        if (have < 4) {
            if (b == uint8_t("CUP1"[have]))
                frame[have++] = b;
            else
                have = b == 'C' ? (frame[0] = b, 1) : 0;
            continue;
        }
        frame[have++] = b;
        if (have < sizeof(UpdateRequest))
            continue;
        UpdateRequest header;
        memcpy(&header, frame, sizeof(header));
        if (header.length > maxBody) {
            ++badRequests;
            have = 0;
            continue;
        }
        const size_t total = sizeof(UpdateRequest) + header.length + 2;
        if (have < total)
            continue;
        have = 0;
        const uint16_t sent = frame[total - 2] | frame[total - 1] << 8;
        if (crc16(frame, total - 2) != sent) {
            ++badRequests;
            continue;
        }
        ++goodRequests;
        // Only the update's own requests keep a stopped camera waiting: the dashboard's status requests kept it stopped after an abandoned update (2026-10-30).
        if (header.op == 'W' || header.op == 'B')
            lastUs = uint32_t(esp_timer_get_time());
        char text[sizeof(UpdateReply::text) + 1] = {};
        const char *failure = handle(header.op, frame + sizeof(UpdateRequest), header.length, text, sizeof(text));
        answer(port, header.id, failure, text);
    }
}

// An update abandoned part way (the laptop gone) must not leave the camera stopped.
void poll() {
    if (stopped && uint32_t(esp_timer_get_time()) - lastUs > abandonUs)
        ESP.restart();
}
} // namespace update
