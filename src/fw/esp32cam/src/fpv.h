#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "config.h"
#include "fec.h"

// FPV video. While a viewer has asked in the last fpvLingerMs the loop on core 1 stops tracking and sends each JPEG frame as chunks of up to chunkBytes, each with a Chunk header, by whichever way it was asked: over UDP to the latest viewer that sent "FPVW" to port (the dashboard's /api/camera on the camera's network), and as raw broadcast 802.11 action frames on the camera's current channel for the FPV receiver (fw/fpv_rx) when the drone forwards request K V (update.h), which needs no network. A lost chunk costs its frame; over TCP a lost segment stalled the stream for the retransmission timeout (one 2.6 s gap in 2 minutes on 2026-10-30, with the camera's network on the drone's channel). Raw frames go unacknowledged at rate, so there are no retries either.
namespace fpv {
constexpr uint16_t port = 5601;
constexpr size_t chunkBytes = 1400;
// Raw frames: broadcast action frames with this BSSID (fpv_rx filters on it), FPV's sent at rate (MCS3, 26 Mbps) and tag mode's small preview at previewRate (MCS0, 6.5 Mbps), which has the airtime to spare.
constexpr uint8_t bssid[6] = {'F', 'P', 'V', 'R', 'A', 'W'};
constexpr wifi_phy_rate_t rate = WIFI_PHY_RATE_MCS3_LGI, previewRate = WIFI_PHY_RATE_MCS0_LGI;
// The 802.11 header, then the action frame's category and a marker, ahead of each chunk.
constexpr size_t macHeader = 28;
constexpr uint8_t actionBody[4] = {127, 'F', 'P', 'V'};

// Each frame goes out as shards (fec.h): its JPEG split evenly into data shards of up to chunkBytes, and over the radio as many parity shards plus one, any data of which rebuild it.
struct __attribute__((packed)) Chunk {
    char magic[4]; // "FPV2"
    uint16_t frame;
    uint8_t index, total, data, reserved; // this shard, the frame's shards, and how many of them carry the JPEG
    uint16_t length; // the JPEG's bytes
    uint32_t captureUs, sentUs; // the frame's capture start and the moment its first shard left, on the camera's clock
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
// The receiver's byte budget (bytes/s), from the drone's K V request: the ESP32 receiver's USB link or the monitor-mode adapter's air.
float radioBudget = config::fpvRadioBytesPerSecond;

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

inline bool recent(uint32_t ms) { return ms && millis() - ms < config::fpvLingerMs; }
inline bool udpWanted() { return recent(fpvSeenMs); }
inline bool radioWanted() { return recent(fpvRadioMs); }
inline bool wanted() { return udpWanted() || radioWanted(); }

// From the loop, before the camera restarts for FPV: an access point asked for (ota.h) gives way to the parked station while the drone asks for FPV over the radio, since with it up no raw frame went out. It stays down: switching back and forth left JPEG capture without its DMA memory. A joined network stays joined, and the video goes out on its channel; on one sharing the drone's channel 6 that took the drone's status stream from 19.9 to 17.3 messages/s and stalled the video for 2.9 s at a time (2026-10-30), and leaving and rejoining it for the video was unreliable, so a camera meant for FPV over the radio lists no networks. True when it switched.
inline bool shareRadio() {
    if (!radioWanted() || !apStarted)
        return false;
    park();
    return true;
}

// One chunk (at datagram + macHeader) as a raw frame; a full transmit queue gets a moment to drain.
inline void broadcast(size_t length, wifi_phy_rate_t phyRate = rate) {
    // A vendor-specific action frame (category 127) from the station, joined or parked (ota.h): the access point refused data frames or accepted and never sent them, and a station joined to nothing sent no data frames. The third address is the BSSID fpv_rx looks for.
    const wifi_interface_t ifx = WIFI_IF_STA;
    if (rateSet != phyRate && esp_wifi_config_80211_tx_rate(ifx, phyRate) == ESP_OK)
        rateSet = phyRate;
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

// The last time the drone asked for tag mode's preview over the radio (K P, web.h radioPreviewLoop).
volatile uint32_t previewRadioMs = 0;

// A whole JPEG as shards: the data shards over UDP to the viewer (to) when udp, and with the parity shards as raw frames at phyRate when radio. Parity replaced whole copies of the frame: the monitor-mode adapter misses a shard at random, 30-45% of them (2026-10-30, as it misses access points' beacons: its receiver, not the air or the camera), and a frame of three shards sent twice lost one 29% of the time, where any three of seven should lose about 2%. A one-shard preview frame goes out three times, as before. The adapter also goes deaf for a few shards at a time (runs of 2-6 missed, up to 14, 2026-10-30), so the shards are spread gapUs apart over the frame rather than sent back to back, which lost 8% of FPV frames with 77% of shards heard.
inline void sendShards(const uint8_t *data, size_t length, uint32_t captureUs, bool radio, bool udp, const sockaddr_in *to,
                       wifi_phy_rate_t phyRate, uint32_t gapUs) {
    const int shards = max(1, int((length + chunkBytes - 1) / chunkBytes)), parity = radio ? shards + 1 : 0;
    const size_t size = (length + shards - 1) / shards;
    if (length > 0xffff || shards + parity > 255)
        return;
    Chunk header = {{'F', 'P', 'V', '2'}, frameNumber++, 0, uint8_t(shards + parity), uint8_t(shards), 0, uint16_t(length), captureUs,
                    uint32_t(esp_timer_get_time())};
    uint8_t *chunk = datagram + macHeader;
    for (int index = 0; index < shards + parity; ++index) {
        header.index = uint8_t(index);
        memcpy(chunk, &header, sizeof(header));
        size_t part = size;
        if (index < shards) {
            part = min(size, length - index * size);
            memcpy(chunk + sizeof(header), data + index * size, part);
        } else {
            fec::parity(data, length, shards, size, index - shards, chunk + sizeof(header));
        }
        if (radio) {
            broadcast(sizeof(header) + part, phyRate);
            // Long gaps sleep, which frees the core; short ones wait out a tick's imprecision.
            if (index + 1 < shards + parity && gapUs >= 4000)
                delay(gapUs / 1000);
            else if (index + 1 < shards + parity)
                delayMicroseconds(gapUs);
        }
        // Out of Wi-Fi buffers the rest of the frame is dropped: the viewer cannot use it.
        if (udp && index < shards && lwip_sendto(socket, chunk, sizeof(header) + part, 0, reinterpret_cast<const sockaddr *>(to), sizeof(*to)) < 0)
            udp = false;
    }
}

// Tag mode's preview over the radio, from the preview task while FPV is off.
inline void broadcastFrame(const uint8_t *data, size_t length, uint32_t captureUs) { sendShards(data, length, captureUs, true, false, nullptr, previewRate, 8000); }

// Over the radio the sensor's frame (VTS, register 0x380e, in lines) is lengthened until the average frame fits within radioBudget at the frame rate, and shortened back as frames shrink, never below its own; nativeLines is cleared at each camera start.
int nativeLines = 0, frameLines = 0;
float averageBytes = 0;

inline void pace(sensor_t *sensor, size_t bytes) {
    averageBytes = averageBytes ? averageBytes * 0.9f + bytes * 0.1f : bytes;
    if (!nativeLines)
        nativeLines = frameLines = sensor->get_reg(sensor, 0x380e, 0xffff);
    if (nativeLines <= 0 || frameNumber % 8)
        return;
    // Aimed at 85% of the budget, which then only catches bursts: aimed at all of it, one frame in five was skipped.
    const int wanted = max(nativeLines, int(nativeLines * averageBytes / (0.85f * radioBudget * config::fpvFramePeriodS)));
    // A change of less than 5% is left alone, so the rate does not wander.
    if (abs(wanted - frameLines) * 20 > frameLines && sensor->set_reg(sensor, 0x380e, 0xffff, wanted) == 0)
        frameLines = wanted;
}

// From the loop while FPV is on: one frame out, then the quality stepped toward fpvTargetBytes. False when the driver gave no frame.
inline bool send() {
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame)
        return false;
    portENTER_CRITICAL(&viewerLock);
    const sockaddr_in to = viewer;
    portEXIT_CRITICAL(&viewerLock);
    const bool udp = udpWanted();
    // Spread over most of a frame period, leaving time to fetch the next.
    const uint32_t gapUs = uint32_t(config::fpvFramePeriodS * 0.8e6f) / (2 * max(1u, unsigned((frame->len + chunkBytes - 1) / chunkBytes)) + 1);
    sendShards(frame->buf, frame->len, uint32_t(uint64_t(frame->timestamp.tv_sec) * 1000000 + frame->timestamp.tv_usec), radioWanted(), udp,
               &to, rate, gapUs);
    // A step of quality per frame, with a dead band so it settles.
    // Over the radio no more than the budget's share of a frame at the sensor's own rate.
    const size_t target = udp ? config::fpvTargetBytes : size_t(min(float(config::fpvTargetBytes), 0.85f * radioBudget * config::fpvFramePeriodS));
    const int step = frame->len > target ? 1 : frame->len < target * 3 / 4 ? -1 : 0;
    const size_t frameBytes = frame->len;
    esp_camera_fb_return(frame);
    const int quality = constrain(fpvQuality + step, config::fpvQualityRange[0], config::fpvQualityRange[1]);
    sensor_t *sensor = esp_camera_sensor_get();
    if (quality != fpvQuality && sensor->set_quality(sensor, quality) == 0)
        fpvQuality = quality;
    if (radioWanted() && !udp)
        pace(sensor, frameBytes);
    ++previewCount;
    return true;
}
} // namespace fpv
