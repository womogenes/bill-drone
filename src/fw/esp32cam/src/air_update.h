#pragma once
#include <Arduino.h>
#include "esp_now.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "freertos/queue.h"
#include "mbedtls/sha256.h"
#include "update.h"

// Firmware updates over ESP-NOW from the FPV receiver (fw/fpv_rx, on the laptop's USB; fw/tools/camera_air_ota.py, make camera-fast), which needs no network and no wiring: the drone's wired link is about 4 KB a request at its radio's pace, this is the receiver's ESP-NOW on the camera's own channel. Every message starts with "CUE1" and a type. D: one piece of a sector (index u16, offset u16, up to pieceBytes), unanswered. R: a request (id u32, op, body) answered by A (id, status 0 done or 1 failed, text), the same id answered again from the last reply without repeating it. Its ops are update.h's (I, B, X; S and N too), W with the sector index and its hash, which writes the sector its pieces built (or names the pieces missing as a hex bitmap), and H (first sector u16, count u8), which replies with the first hashBytes of each spare-slot sector's SHA-256, so a sector already there is not sent again and an interrupted update resumes. W and B stop the camera and keep update.h's abandon timer running, as over the wire.
namespace air {
constexpr size_t pieceBytes = 232, maxMessage = 250, maxHashes = 28;
constexpr uint8_t magic[4] = {'C', 'U', 'E', '1'};

struct Message {
    uint8_t from[6];
    uint8_t length;
    uint8_t bytes[maxMessage];
};
QueueHandle_t inbox = nullptr;
bool started = false;

// The sector the pieces build, which pieces have come, and the last reply for a repeated request.
uint8_t staged[update::sector];
int stagedIndex = -1;
uint32_t piecesSeen = 0;
uint32_t lastId = 0;
uint8_t lastReply[maxMessage];
size_t lastReplyLength = 0;
bool answeredOnce = false;
uint32_t pieces = 0, requests = 0;

void received(const uint8_t *mac, const uint8_t *data, int length) {
    if (length < 5 || length > int(maxMessage) || memcmp(data, magic, 4))
        return;
    Message message;
    memcpy(message.from, mac, 6);
    message.length = uint8_t(length);
    memcpy(message.bytes, data, length);
    xQueueSend(inbox, &message, 0);
}

// After WiFi starts.
inline void start() {
    inbox = xQueueCreate(24, sizeof(Message));
    started = inbox && esp_now_init() == ESP_OK && esp_now_register_recv_cb(received) == ESP_OK;
}

inline void reply(const uint8_t *to, const uint8_t *bytes, size_t length) {
    if (!esp_now_is_peer_exist(to)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, to, 6);
        peer.ifidx = WIFI_IF_STA;
        esp_now_add_peer(&peer);
    }
    esp_now_send(to, bytes, length);
}

// One request's answer into text (up to size), as update::handle's, with the air's own W and H.
inline const char *handle(uint8_t op, const uint8_t *body, size_t length, char *text, size_t size, size_t &textLength) {
    textLength = 0;
    if (op == 'H') {
        if (length < 3 || body[2] > maxHashes)
            return "Expected first sector (u16) and count (up to 28)";
        const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
        if (!target)
            return "No spare slot";
        const uint32_t first = body[0] | body[1] << 8;
        for (uint32_t index = first; index < first + body[2] && (index + 1) * update::sector <= target->size; ++index) {
            if (esp_partition_read(target, index * update::sector, staged, update::sector) != ESP_OK)
                return "Read failed";
            uint8_t digest[32];
            mbedtls_sha256_ret(staged, update::sector, digest, 0);
            memcpy(text + textLength, digest, update::hashBytes);
            textLength += update::hashBytes;
        }
        // The staging buffer held slot sectors, not pieces.
        stagedIndex = -1;
        piecesSeen = 0;
        return nullptr;
    }
    if (op == 'W') {
        if (length < 2 + update::hashBytes)
            return "Expected sector index and hash";
        const int index = body[0] | body[1] << 8;
        constexpr uint32_t all = (1u << ((update::sector + pieceBytes - 1) / pieceBytes)) - 1;
        if (index != stagedIndex || piecesSeen != all) {
            snprintf(text, size, "missing %lx", (unsigned long)(index == stagedIndex ? all & ~piecesSeen : all));
            textLength = strlen(text);
            return text;
        }
        static uint8_t request[update::maxBody];
        memcpy(request, body, 2 + update::hashBytes);
        memcpy(request + 2 + update::hashBytes, staged, update::sector);
        const char *failure = update::handle('W', request, update::maxBody, text, size);
        // A sector that did not take is sent again whole.
        if (failure) {
            stagedIndex = -1;
            piecesSeen = 0;
        }
        textLength = strlen(failure ? failure : text);
        return failure;
    }
    const char *failure = update::handle(op, body, length, text, size);
    textLength = strlen(failure ? failure : text);
    return failure;
}

// From the service task: the queued pieces and requests.
inline void poll() {
    if (!started)
        return;
    Message message;
    while (xQueueReceive(inbox, &message, 0) == pdTRUE) {
        const uint8_t *data = message.bytes;
        if (data[4] == 'D' && message.length > 9) {
            const int index = data[5] | data[6] << 8, offset = data[7] | data[8] << 8;
            const size_t length = message.length - 9;
            if (offset % pieceBytes || offset + length > update::sector)
                continue;
            if (index != stagedIndex) {
                stagedIndex = index;
                piecesSeen = 0;
            }
            memcpy(staged + offset, data + 9, length);
            piecesSeen |= 1u << (offset / pieceBytes);
            ++pieces;
            continue;
        }
        if (data[4] != 'R' || message.length < 10)
            continue;
        ++requests;
        uint32_t id;
        memcpy(&id, data + 5, 4);
        const uint8_t op = data[9];
        if (op == 'W' || op == 'B')
            update::lastUs = uint32_t(esp_timer_get_time());
        if (!(answeredOnce && id == lastId)) {
            char text[maxMessage] = {};
            size_t textLength = 0;
            const char *failure = handle(op, data + 10, message.length - 10, text, maxMessage - 10, textLength);
            memcpy(lastReply, magic, 4);
            lastReply[4] = 'A';
            memcpy(lastReply + 5, &id, 4);
            lastReply[9] = failure != nullptr;
            memcpy(lastReply + 10, failure ? failure : text, textLength);
            lastReplyLength = 10 + textLength;
            lastId = id;
            answeredOnce = true;
        }
        reply(message.from, lastReply, lastReplyLength);
        if (update::restarting) {
            delay(100);
            ESP.restart();
        }
    }
}
} // namespace air
