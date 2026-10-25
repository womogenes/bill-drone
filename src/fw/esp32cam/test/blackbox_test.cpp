// Writes a black-box file on the host with the camera's own encoder and ring (src/blackbox.h), draining the ring in staging-sized pieces as sd.h does, with a writer that stalls now and then so entries are dropped. blackbox_test.py builds and runs it and decodes the file with blackbox.py. argv: output file. stdout: one line per frame offered, "sequence queued crop_left crop_top full", then the dropped count.
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../src/blackbox.h"

struct __attribute__((packed)) Record { // a FrameRecord's layout (record.h): the TAG3 packet, then the rest
    char magic[4];
    uint32_t sequence, captureUs, ageUs;
    uint8_t rest[224 - 16];
};

int main(int, char **argv) {
    using namespace blackbox;
    constexpr int width = 320, height = 240, reducedWidth = 80, reducedHeight = 60, cropSize = 96, fullEvery = 7;
    constexpr uint32_t staging = 8192;
    FILE *file = fopen(argv[1], "wb");
    std::vector<uint8_t> ringBytes(1 << 17), chunk(staging), frame(width * height), window(24 * 1024), reduced(reducedWidth * reducedHeight);
    Ring ring;
    ring.bytes = ringBytes.data();
    ring.size = uint32_t(ringBytes.size());
    FileHeader header;
    header.file = 1;
    header.recordBytes = sizeof(Record);
    header.reducedWidth = reducedWidth;
    header.reducedHeight = reducedHeight;
    header.cropSize = cropSize;
    header.fullWidth = width;
    header.fullHeight = height;
    header.fullEvery = fullEvery;
    memcpy(chunk.data(), &header, sizeof(header));
    uint32_t fill = sizeof(header), dropped = 0;
    srand(3);
    for (uint32_t sequence = 0; sequence < 300; ++sequence) {
        // Each frame's pixels are a function of position and frame, so the decoder's images can be checked exactly.
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
                frame[y * width + x] = uint8_t(x * 3 + y * 7 + sequence);
        for (int i = 0; i < reducedWidth * reducedHeight; ++i)
            reduced[i] = uint8_t(i + sequence);
        // The tag wanders, also off the edges, and the tracker's window covers part of the crop or none of it.
        const float cx = float(rand() % 400) - 40, cy = float(rand() % 320) - 40;
        const float corners[4][2] = {{cx - 20, cy - 20}, {cx + 20, cy - 20}, {cx + 20, cy + 20}, {cx - 20, cy + 20}};
        const int wl = int(cx) - 40 + rand() % 30, wt = int(cy) - 40 + rand() % 30, ww = 70, wh = 60;
        const bool useWindow = rand() % 3 && wl >= 0 && wt >= 0 && wl + ww <= width && wt + wh <= height;
        if (useWindow)
            for (int y = 0; y < wh; ++y)
                memcpy(window.data() + y * ww, frame.data() + (wt + y) * width + wl, ww);
        const CropSource source = {frame.data(), width, height, useWindow ? window.data() : nullptr, wl, wt, ww, wh};
        const bool found = sequence % 5 != 0, full = sequence % fullEvery == 0;
        const CropHeader crop = placeCrop(corners, cropSize, width, height);
        Record record = {};
        memcpy(record.magic, "TAG3", 4);
        record.sequence = sequence;
        record.ageUs = 36000;
        const bool queued = encodeFrame(ring, dropped, &record, sizeof(record), reduced.data(), uint32_t(reduced.size()),
                                        found ? &source : nullptr, crop, full ? frame.data() : nullptr, width * height);
        dropped += !queued;
        printf("%u %d %d %d %d\n", sequence, queued, found ? crop.left : -1, found ? crop.top : -1, full);
        // The writer: some frames it keeps up, in bursts it stalls and the ring fills.
        if ((sequence / 40) % 3 != 2)
            for (;;) {
                fill += ring.take(chunk.data() + fill, staging - fill);
                if (fill < staging)
                    break;
                fwrite(chunk.data(), 1, fill, file);
                fill = 0;
            }
    }
    for (;;) {
        fill += ring.take(chunk.data() + fill, staging - fill);
        if (!fill)
            break;
        fwrite(chunk.data(), 1, fill, file);
        fill = 0;
    }
    fclose(file);
    printf("dropped %u\n", dropped);
    return 0;
}
