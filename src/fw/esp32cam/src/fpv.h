#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "config.h"

// FPV video. While a viewer has asked in the last fpvLingerMs the loop on core 1 stops tracking and sends each JPEG frame as chunks of up to chunkBytes, each with a Chunk header, by whichever way it was asked: over UDP to the latest viewer that sent "FPVW" to port (the dashboard's /api/camera on the camera's network), and as raw broadcast 802.11 action frames on the camera's current channel for the FPV receiver (fw/fpv_rx) when the drone forwards request K V (update.h), which needs no network. A lost chunk costs its frame; over TCP a lost segment stalled the stream for the retransmission timeout (one 2.6 s gap in 2 minutes on 2026-10-30, with the camera's network on the drone's channel). Raw frames go unacknowledged at rate, so there are no retries either.
namespace fpv {
constexpr uint16_t port = 5601;
constexpr size_t chunkBytes = 1400;
// Raw frames: broadcast action frames with this BSSID (fpv_rx filters on it), sent at rate (MCS3, 26 Mbps).
constexpr uint8_t bssid[6] = {'F', 'P', 'V', 'R', 'A', 'W'};
constexpr wifi_phy_rate_t rate = WIFI_PHY_RATE_MCS3_LGI;
// The 802.11 header, then the action frame's category and a marker, ahead of each chunk.
constexpr size_t macHeader = 28;
constexpr uint8_t actionBody[4] = {127, 'F', 'P', 'V'};

struct __attribute__((packed)) Chunk {
    char magic[4]; // "FPV1"
    uint16_t frame;
    uint8_t index, count;
    uint32_t captureUs, sentUs; // the frame's capture start and the moment its first datagram left, on the camera's clock
};
} // namespace fpv

volatile bool fpvActive = false;
volatile uint32_t fpvSeenMs = 0, fpvRadioMs = 0;
// The JPEG quality FPV last settled on, the starting point of the next stream.
int fpvQuality = 20;
extern volatile uint32_t previewCount;

namespace fpv {
int socket = -1;
sockaddr_in viewer;
portMUX_TYPE viewerLock = portMUX_INITIALIZER_UNLOCKED;
uint16_t frameNumber = 0;
uint8_t datagram[macHeader + sizeof(Chunk) + chunkBytes];
int rateSet = -1;
// The last raw send's result, for K S.
esp_err_t rawError = ESP_OK;
uint32_t rawSent = 0;

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

inline bool recent(uint32_t ms, uint32_t linger) { return ms && millis() - ms < linger; }
inline bool udpWanted() { return recent(fpvSeenMs, config::fpvLingerMs); }
inline bool radioWanted() { return recent(fpvRadioMs, config::fpvRadioLingerMs); }
inline bool wanted() { return udpWanted() || radioWanted(); }

// From the loop, before the camera restarts for FPV: an access point asked for (ota.h) gives way to the parked station while the drone asks for FPV over the radio, since with it up no raw frame went out. It stays down: switching back and forth left JPEG capture without its DMA memory. A joined network stays joined, and the video goes out on its channel; on one sharing the drone's channel 6 that took the drone's status stream from 19.9 to 17.3 messages/s and stalled the video for 2.9 s at a time (2026-10-30), and leaving and rejoining it for the video was unreliable, so a camera meant for FPV over the radio lists no networks. True when it switched.
inline bool shareRadio() {
    if (!radioWanted() || !apStarted)
        return false;
    park();
    return true;
}

// One chunk (at datagram + macHeader) as a raw frame; a full transmit queue gets a moment to drain.
inline void broadcast(size_t length) {
    // A vendor-specific action frame (category 127) from the station, joined or parked (ota.h): the access point refused data frames or accepted and never sent them, and a station joined to nothing sent no data frames. The third address is the BSSID fpv_rx looks for.
    const wifi_interface_t ifx = WIFI_IF_STA;
    if (rateSet != ifx && esp_wifi_config_80211_tx_rate(ifx, rate) == ESP_OK)
        rateSet = ifx;
    static const uint8_t control[4] = {0xd0, 0x00, 0x00, 0x00};
    memcpy(datagram, control, 4);
    memset(datagram + 4, 0xff, 6);
    esp_wifi_get_mac(ifx, datagram + 10);
    memcpy(datagram + 16, bssid, 6);
    memcpy(datagram + 24, actionBody, sizeof(actionBody));
    for (int tries = 0; tries < 3 && (rawError = esp_wifi_80211_tx(ifx, datagram, macHeader + length, true)) == ESP_ERR_NO_MEM; ++tries)
        delay(1);
    rawSent += rawError == ESP_OK;
}

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
    // The radio's byte budget, refilled with time up to one second's worth: a frame it cannot pay for is not broadcast.
    static float budget = 0;
    static uint32_t budgetUs = 0;
    const uint32_t nowUs = uint32_t(esp_timer_get_time());
    budget = min(config::fpvRadioBytesPerSecond, budget + (nowUs - budgetUs) * 1e-6f * config::fpvRadioBytesPerSecond);
    budgetUs = nowUs;
    const bool udp = udpWanted(), radio = radioWanted() && budget >= frame->len;
    if (radio)
        budget -= frame->len;
    uint8_t *chunk = datagram + macHeader;
    for (size_t offset = 0; offset < frame->len; offset += chunkBytes, ++header.index) {
        const size_t length = min(chunkBytes, frame->len - offset);
        memcpy(chunk, &header, sizeof(header));
        memcpy(chunk + sizeof(header), frame->buf + offset, length);
        if (radio)
            broadcast(sizeof(header) + length);
        // Out of Wi-Fi buffers the rest of the frame is dropped: the viewer cannot use it.
        if (udp && lwip_sendto(socket, chunk, sizeof(header) + length, 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to)) < 0)
            break;
    }
    // A step of quality per frame, with a dead band so it settles.
    const size_t target = udp ? config::fpvTargetBytes : config::fpvRadioTargetBytes;
    const int step = frame->len > target ? 1 : frame->len < target * 3 / 4 ? -1 : 0;
    esp_camera_fb_return(frame);
    const int quality = constrain(fpvQuality + step, config::fpvQualityRange[0], config::fpvQualityRange[1]);
    sensor_t *sensor = esp_camera_sensor_get();
    if (quality != fpvQuality && sensor->set_quality(sensor, quality) == 0)
        fpvQuality = quality;
    ++previewCount;
    return true;
}
} // namespace fpv
