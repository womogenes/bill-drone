#pragma once
#include "lwip/sockets.h"
#include "packet.h"
#include "web.h"

// Per-frame records for offline analysis (fw/esp32cam/record.py), sent over UDP to one subscriber. Core 1 only copies
// a record into a ring in internal RAM, and only while someone subscribes; a low-priority core-0 task batches and sends.
struct __attribute__((packed)) FrameRecord {
    PosePacket packet;
    float corners[4][2];   // tag corners in pixels, zero when not found
    uint32_t frameStartUs; // driver timestamp of the frame start, CAM clock
    uint16_t processUs;    // frame fetched to packet published
    uint16_t flowWaitUs;   // detector blocked on the flow task
    uint8_t found;         // 0 none, 1 tracked, 2 detected
    uint8_t preview;       // 1 if this frame was handed to the preview copier
    uint16_t shortFrames;  // driver FB-SIZE drops so far (low 16 bits)
};
static_assert(sizeof(FrameRecord) == 144, "Record format changed");

constexpr int recordSlots = 32; // about a second at 28 Hz; the sender drains it every 100 ms
FrameRecord records[recordSlots];
volatile uint32_t recordsWritten = 0, recordSubscriberIp = 0, recordExpiresMs = 0;
volatile uint16_t recordSubscriberPort = 0;

inline bool recording() { return recordSubscriberPort != 0; }

inline void addRecord(const FrameRecord &record) {
    records[recordsWritten % recordSlots] = record;
    recordsWritten = recordsWritten + 1;
}

// Datagrams are "FRM1" followed by up to nine records.
void recordLoop(void *) {
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(sock >= 0);
    uint8_t datagram[4 + 9 * sizeof(FrameRecord)];
    memcpy(datagram, "FRM1", 4);
    uint32_t sent = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (recording() && int32_t(millis() - recordExpiresMs) > 0)
            recordSubscriberPort = 0;
        const uint32_t written = recordsWritten;
        if (!recording()) {
            sent = written;
            continue;
        }
        if (written - sent > recordSlots - 2)
            sent = written - (recordSlots - 2); // overrun: the receiver sees the sequence gap
        sockaddr_in to = {};
        to.sin_family = AF_INET;
        to.sin_port = htons(recordSubscriberPort);
        to.sin_addr.s_addr = recordSubscriberIp;
        while (sent != written) {
            int count = 0;
            for (; sent != written && count < 9; ++sent, ++count)
                memcpy(datagram + 4 + count * sizeof(FrameRecord), &records[sent % recordSlots], sizeof(FrameRecord));
            sendto(sock, datagram, 4 + count * sizeof(FrameRecord), 0, reinterpret_cast<sockaddr *>(&to), sizeof(to));
        }
    }
}

// GET /record?port=N streams records to the caller's address on UDP port N for 5 s; call again to keep it going.
void startRecording() {
    web.on("/record", HTTP_GET, []() {
        const long port = web.arg("port").toInt();
        if (port <= 0 || port > 65535) {
            web.send(400, "text/plain", "port?");
            return;
        }
        recordSubscriberIp = uint32_t(web.client().remoteIP());
        recordExpiresMs = millis() + 5000;
        recordSubscriberPort = uint16_t(port);
        web.send(200, "application/json", "{\"record_bytes\":144,\"magic\":\"FRM1\"}");
    });
    xTaskCreatePinnedToCore(recordLoop, "record", 4096, nullptr, 1, nullptr, 0);
}
