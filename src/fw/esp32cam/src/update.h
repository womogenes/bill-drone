#pragma once
#include <Arduino.h>
#include "driver/uart.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "mbedtls/sha256.h"
#include "packet.h"
extern bool cameraReady, cameraStalled;
extern volatile bool fpvActive;
extern esp_err_t startError;
extern volatile bool cameraStopRequested;

// Firmware updates through the drone's radio link, for when the camera cannot reach Wi-Fi (fw/tools/ota_direct.py camera): the drone forwards each request on the link's receive side and returns the reply. S reports the Wi-Fi network, address and camera state; I describes the slots (running label, spare label and size, and the first 8 bytes of the running image's SHA-256, which the build appends to the image); W writes one 4 KB sector of the spare slot and reads it back, after checking its hash; B selects the spare slot, which checks the whole image, and restarts into it; X abandons the update and restarts. The first W or B stops the camera, as a Wi-Fi update does, and a camera left stopped with no request for abandonUs restarts. Requests come one at a time, each after the last reply, so nothing arrives while the flash is being written.
namespace update {
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
        snprintf(text + used, size - used, " %s 0x%x", cameraStalled ? "stalled" : !cameraReady ? "stopped" : fpvActive ? "fpv" : "running", unsigned(startError));
        return nullptr;
    }
    if (op == 'X') {
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
            have = 0;
            continue;
        }
        const size_t total = sizeof(UpdateRequest) + header.length + 2;
        if (have < total)
            continue;
        have = 0;
        const uint16_t sent = frame[total - 2] | frame[total - 1] << 8;
        if (crc16(frame, total - 2) != sent)
            continue;
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
