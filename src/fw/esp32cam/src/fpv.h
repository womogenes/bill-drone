#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "config.h"

// FPV video over UDP. A viewer (the dashboard's /api/camera) sends "FPVW" to fpvPort every half second; while one has in the last fpvLingerMs, the loop on core 1 stops tracking and sends each JPEG frame to the latest viewer as datagrams of up to fpvChunkBytes, each with an FpvChunk header. A lost datagram costs its frame; over TCP a lost segment stalled the stream for the retransmission timeout (one 2.6 s gap in 2 minutes on 2026-10-30, with the camera's network on the drone's channel).
namespace fpv {
constexpr uint16_t port = 5601;
constexpr size_t chunkBytes = 1400;

struct __attribute__((packed)) Chunk {
    char magic[4]; // "FPV1"
    uint16_t frame;
    uint8_t index, count;
    uint32_t captureUs, sentUs; // the frame's capture start and the moment its first datagram left, on the camera's clock
};
} // namespace fpv

volatile bool fpvActive = false;
volatile uint32_t fpvSeenMs = 0;
// The JPEG quality FPV last settled on, the starting point of the next stream.
int fpvQuality = 20;
extern volatile uint32_t previewCount;

namespace fpv {
int socket = -1;
sockaddr_in viewer;
portMUX_TYPE viewerLock = portMUX_INITIALIZER_UNLOCKED;
uint16_t frameNumber = 0;
uint8_t datagram[sizeof(Chunk) + chunkBytes];

// Only with Wi-Fi started (startWifi), which brings up the network stack.
inline void start() {
    socket = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_port = htons(port);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    assert(socket >= 0 && lwip_bind(socket, reinterpret_cast<sockaddr *>(&local), sizeof(local)) == 0);
    lwip_fcntl(socket, F_SETFL, O_NONBLOCK);
}

// From the service task: takes the viewers' keepalives.
inline void poll() {
    if (socket < 0)
        return;
    char text[8];
    sockaddr_in from;
    socklen_t length = sizeof(from);
    while (lwip_recvfrom(socket, text, sizeof(text), 0, reinterpret_cast<sockaddr *>(&from), &length) == 4) {
        if (!memcmp(text, "FPVW", 4)) {
            portENTER_CRITICAL(&viewerLock);
            viewer = from;
            portEXIT_CRITICAL(&viewerLock);
            fpvSeenMs = millis();
        }
        length = sizeof(from);
    }
}

inline bool wanted() { return fpvSeenMs && millis() - fpvSeenMs < config::fpvLingerMs; }

// From the loop while FPV is on: one frame out, then the quality stepped toward fpvTargetBytes. False when the driver gave no frame.
inline bool send() {
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame)
        return false;
    portENTER_CRITICAL(&viewerLock);
    const sockaddr_in to = viewer;
    portEXIT_CRITICAL(&viewerLock);
    Chunk header = {{'F', 'P', 'V', '1'}, frameNumber++, 0, uint8_t((frame->len + chunkBytes - 1) / chunkBytes),
                    uint32_t(uint64_t(frame->timestamp.tv_sec) * 1000000 + frame->timestamp.tv_usec), uint32_t(esp_timer_get_time())};
    for (size_t offset = 0; offset < frame->len; offset += chunkBytes, ++header.index) {
        const size_t length = min(chunkBytes, frame->len - offset);
        memcpy(datagram, &header, sizeof(header));
        memcpy(datagram + sizeof(header), frame->buf + offset, length);
        // Out of Wi-Fi buffers the rest of the frame is dropped: the viewer cannot use it.
        if (lwip_sendto(socket, datagram, sizeof(header) + length, 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to)) < 0)
            break;
    }
    // A step of quality per frame, with a dead band so it settles.
    const int step = frame->len > config::fpvTargetBytes ? 1 : frame->len < config::fpvTargetBytes * 3 / 4 ? -1 : 0;
    esp_camera_fb_return(frame);
    const int quality = constrain(fpvQuality + step, config::fpvQualityRange[0], config::fpvQualityRange[1]);
    sensor_t *sensor = esp_camera_sensor_get();
    if (quality != fpvQuality && sensor->set_quality(sensor, quality) == 0)
        fpvQuality = quality;
    ++previewCount;
    return true;
}
} // namespace fpv
