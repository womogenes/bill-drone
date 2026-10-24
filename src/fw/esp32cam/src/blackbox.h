#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// The black box's file format and the ring that feeds it, without anything ESP32-specific, so the host test
// (test/blackbox_test.cpp) runs this same code; sd.h mounts the card and writes. A file is a FileHeader followed by one
// entry per recorded frame; fw/esp32cam/blackbox.py reads it. All fields are little-endian.
namespace blackbox {

struct __attribute__((packed)) FileHeader {
    char magic[8] = {'B', 'I', 'L', 'L', 'B', 'B', 'X', '1'};
    uint32_t headerBytes = sizeof(FileHeader);
    uint32_t file = 0;            // the NNNN of blackbox/NNNN.bin
    uint32_t recordBytes = 0;     // FrameRecord (record.h), at the start of every entry
    uint16_t reducedWidth = 0, reducedHeight = 0; // the flow's reduced frame, after the record
    uint16_t cropSize = 0;        // the tag crop's side, full-resolution pixels
    uint16_t fullWidth = 0, fullHeight = 0;
    uint16_t fullEvery = 0;       // every Nth frame carries the full frame when this file started; 0 never
    uint32_t startUs = 0;         // CAM clock when the file was opened
    float fx = 0, fy = 0, cx = 0, cy = 0, distortion[5] = {}; // the calibration in use
    float tagSizeM = 0;
    uint32_t tagId = 0;
    char build[24] = {};          // __DATE__ " " __TIME__ of the firmware
};
static_assert(sizeof(FileHeader) == 104, "Black box header changed");

// Each entry: EntryHeader, then length bytes: the FrameRecord, the reduced frame, a CropHeader and its pixels when
// flags has hasCrop, the full frame when it has hasFull, and zero padding to a multiple of four.
enum EntryFlags : uint16_t { hasCrop = 1, hasFull = 2 };

struct __attribute__((packed)) EntryHeader {
    char magic[4] = {'B', 'B', 'E', '1'};
    uint16_t flags = 0;
    uint16_t reserved = 0;
    uint32_t length = 0;   // payload bytes after this header, a multiple of four
    uint32_t dropped = 0;  // frames this file had to drop so far because the card fell behind
};
static_assert(sizeof(EntryHeader) == 16, "Black box entry changed");

struct __attribute__((packed)) CropHeader {
    int16_t left = 0, top = 0;     // full-resolution position of the crop's top-left pixel
    uint16_t width = 0, height = 0;
};
static_assert(sizeof(CropHeader) == 8, "Black box crop changed");

// A byte ring with one producer (the detector, core 1) and one consumer (the card writer, core 0). The counters only
// grow, so their difference is the fill even when they wrap; size must be a power of two. The producer never waits:
// an entry that does not fit is refused, and the consumer only ever sees whole committed entries.
struct Ring {
    uint8_t *bytes = nullptr;
    uint32_t size = 0;
    volatile uint32_t head = 0, tail = 0;

    uint32_t used() const { return __atomic_load_n(&head, __ATOMIC_ACQUIRE) - __atomic_load_n(&tail, __ATOMIC_ACQUIRE); }
    uint32_t space() const { return size - used(); }

    void copyIn(uint32_t at, const void *data, uint32_t length) {
        const uint32_t offset = at & (size - 1), first = length < size - offset ? length : size - offset;
        memcpy(bytes + offset, data, first);
        memcpy(bytes, static_cast<const uint8_t *>(data) + first, length - first);
    }

    void copyOut(uint32_t at, void *data, uint32_t length) const {
        const uint32_t offset = at & (size - 1), first = length < size - offset ? length : size - offset;
        memcpy(data, bytes + offset, first);
        memcpy(static_cast<uint8_t *>(data) + first, bytes, length - first);
    }

    // The consumer's side: copies up to length of the oldest bytes out and frees them.
    uint32_t take(void *data, uint32_t length) {
        const uint32_t available = used(), count = length < available ? length : available;
        copyOut(tail, data, count);
        __atomic_store_n(&tail, tail + count, __ATOMIC_RELEASE);
        return count;
    }
};

// The producer's side of one entry: reserve its whole length, append its parts, then commit, which publishes it.
struct Writer {
    Ring &ring;
    uint32_t at;
    bool reserved;
    Writer(Ring &target, uint32_t length) : ring(target), at(target.head), reserved(target.space() >= length) {}
    void append(const void *data, uint32_t length) {
        ring.copyIn(at, data, length);
        at += length;
    }
    void commit() {
        static const uint8_t zeros[4] = {};
        append(zeros, (4 - (at - ring.head) % 4) % 4);
        __atomic_store_n(&ring.head, at, __ATOMIC_RELEASE);
    }
};

inline uint32_t padded(uint32_t length) { return (length + 3) & ~3u; }

// Where the tag crop comes from: the frame, and the tracker's internal-RAM copy of the tag's neighbourhood (window,
// whose top-left is frame pixel windowLeft, windowTop; null when the tracker read the frame itself).
struct CropSource {
    const uint8_t *frame;
    int width, height;
    const uint8_t *window;
    int windowLeft, windowTop, windowWidth, windowHeight;
};

// A size x size crop centred on the tag's corners and kept inside the frame.
inline CropHeader placeCrop(const float corners[4][2], int size, int width, int height) {
    float x = 0, y = 0;
    for (int i = 0; i < 4; ++i) {
        x += corners[i][0] / 4;
        y += corners[i][1] / 4;
    }
    CropHeader crop;
    const int left = int(x + 0.5f) - size / 2, top = int(y + 0.5f) - size / 2;
    crop.left = int16_t(left < 0 ? 0 : left > width - size ? width - size : left);
    crop.top = int16_t(top < 0 ? 0 : top > height - size ? height - size : top);
    crop.width = crop.height = uint16_t(size);
    return crop;
}

// The crop's rows, top to bottom, as pieces passed to append(pointer, length): from the window where it covers them, so
// the PSRAM frame is read only around it.
template <typename Append> inline void copyCrop(const CropSource &source, const CropHeader &crop, Append &&append) {
    const int left = crop.left, right = crop.left + crop.width;
    const int coveredLeft = source.window && source.windowLeft > left ? source.windowLeft : left;
    const int coveredRight =
        source.window && source.windowLeft + source.windowWidth < right ? source.windowLeft + source.windowWidth : right;
    for (int row = crop.top; row < crop.top + crop.height; ++row) {
        const uint8_t *line = source.frame + row * source.width;
        if (!source.window || row < source.windowTop || row >= source.windowTop + source.windowHeight ||
            coveredLeft >= coveredRight) {
            append(line + left, right - left);
            continue;
        }
        const uint8_t *copy = source.window + (row - source.windowTop) * source.windowWidth - source.windowLeft;
        append(line + left, coveredLeft - left);
        append(copy + coveredLeft, coveredRight - coveredLeft);
        append(line + coveredRight, right - coveredRight);
    }
}

// Queues one frame's entry: the record, the reduced frame, the tag crop when crop is not null, and the full frame when
// full is not null. False, with nothing queued, when the ring has no room for all of it.
inline bool encodeFrame(Ring &ring, uint32_t dropped, const void *record, uint32_t recordBytes, const uint8_t *reduced,
                        uint32_t reducedBytes, const CropSource *source, const CropHeader &crop, const uint8_t *full,
                        uint32_t fullBytes) {
    EntryHeader entry;
    entry.flags = (source ? hasCrop : 0) | (full ? hasFull : 0);
    entry.length = padded(recordBytes + reducedBytes + (source ? sizeof(CropHeader) + crop.width * crop.height : 0) +
                          (full ? fullBytes : 0));
    entry.dropped = dropped;
    Writer writer(ring, sizeof(entry) + entry.length);
    if (!writer.reserved)
        return false;
    writer.append(&entry, sizeof(entry));
    writer.append(record, recordBytes);
    writer.append(reduced, reducedBytes);
    if (source) {
        writer.append(&crop, sizeof(crop));
        copyCrop(*source, crop, [&](const uint8_t *data, int length) { writer.append(data, length); });
    }
    if (full)
        writer.append(full, fullBytes);
    writer.commit();
    return true;
}

} // namespace blackbox
