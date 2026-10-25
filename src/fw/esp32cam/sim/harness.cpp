// Runs the camera's own tag pipeline (tag.h, track.h, pose.h, flow.h) on rendered frames; built and driven by sim.py.
// stdin: "FRM0", int32 width, height, count, then per frame int64 frame start (us) and width x height Y8 pixels.
// stdout: one line per frame: kind valid margin reprojection_px host_us corners(8, rolling shutter removed) position(3) rotation(9) flow(2) flow_found track_failure predicted(8) (the tracker's prediction, zero when it did not run) trusted tag_position(3).
// argv: fx fy cx cy [timed]. When timed, each frame's work (TagFinder::work) gives how long the camera would be busy, and frames it would miss meanwhile are reported with kind -1: as on the camera, a frame can be processed only once its readout has ended and the previous one is done. host_us is then that camera time.
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <vector>
using std::max;
using std::min;
#include "../src/config.h"
#include "../src/packet.h"
#include "../src/pose.h"
#include "../src/track.h"
#include "../src/tag.h"
#include "../src/flow.h"
#include "tag16h5.h"

int main(int argc, char **argv) {
    // Rendered frames have no lens distortion.
    for (float &d : config::distortion)
        d = 0;
    if (argc >= 5) {
        config::fx = atof(argv[1]);
        config::fy = atof(argv[2]);
        config::cx = atof(argv[3]);
        config::cy = atof(argv[4]);
    }
    const bool timed = argc >= 6 && atoi(argv[5]);
    // Camera costs, measured on 2026-10-22 except where estimated: a frame's fixed work (flow wait, pose, packet), an edge tracker run, an alignment shift tried on the nearest pixels or interpolated (64 samples) in the internal copy (estimated) or in the PSRAM frame (18 us measured), and per decimated pixel the detector in an internal-RAM window (estimated) or on a PSRAM whole frame (80 ms at decimation 3, including the refinement).
    constexpr double fixedUs = 2500, trackUs = 4000, coarseUs = 4, fineUs = 20, frameCoarseUs = 18, frameFineUs = 40,
                     detectorRunUs = 1500, windowPixelUs = 4, framePixelUs = 8.5;
    double busyUntil = 0;
    int64_t previousStart = -int64_t(config::framePeriodUs);
    char magic[4];
    int32_t header[3];
    if (fread(magic, 1, 4, stdin) != 4 || fread(header, 4, 3, stdin) != 3)
        return 1;
    const int width = header[0], height = header[1], count = header[2];
    TagFinder finder;
    finder.detector = apriltag_detector_create();
    finder.family = tag16h5_create();
    apriltag_detector_add_family_bits(finder.detector, finder.family, 0);
    finder.detector->nthreads = 1;
    finder.detector->refine_edges = true;
    finder.detector->qtp.min_white_black_diff = 20;
    finder.detector->qtp.max_nmaxima = 6;
    if (argc >= 7)
        finder.frameDecimate = atof(argv[6]);
    if (argc >= 9) {
        finder.maxHamming = atoi(argv[7]);
        finder.minMargin = atof(argv[8]);
        apriltag_detector_add_family_bits(finder.detector, finder.family, finder.maxHamming);
    }
    std::vector<uint8_t> frame(size_t(width) * height);
    for (int n = 0; n < count; ++n) {
        int64_t start;
        if (fread(&start, 8, 1, stdin) != 1 || fread(frame.data(), 1, frame.size(), stdin) != frame.size())
            return 1;
        const double readoutEnd = double(start) + config::framePeriodUs;
        if (timed && readoutEnd < busyUntil) {
            printf("-1 0 0 0 0");
            for (int i = 0; i < 20; ++i)
                printf(" 0");
            printf(" 0 0 0 0");
            for (int i = 0; i < 12; ++i)
                printf(" 0");
            printf("\n");
            continue;
        }
        const std::clock_t began = std::clock();
        // The flow excludes the predicted tag, as on the camera (where it runs on the other core).
        float box[4];
        if (finder.tracking) {
            box[0] = box[1] = INFINITY;
            box[2] = box[3] = -INFINITY;
            float predicted[4][2];
            finder.predict(uint32_t(start), predicted);
            for (int i = 0; i < 4; ++i)
                for (int axis = 0; axis < 2; ++axis) {
                    box[axis] = fminf(box[axis], predicted[i][axis]);
                    box[axis + 2] = fmaxf(box[axis + 2], predicted[i][axis]);
                }
            for (int axis = 0; axis < 2; ++axis) {
                box[axis] -= finder.sidePx / 2;
                box[axis + 2] += finder.sidePx / 2;
            }
        }
        flow::Result flowResult = {};
        const bool flowFound = flow::update(frame.data(), width, finder.tracking ? box : nullptr, flowResult);
        PosePacket packet;
        packet.sequence = n;
        float corners[4][2] = {}, margin = 0, predicted[4][2] = {};
        if (finder.tracking)
            finder.predict(uint32_t(start) + 1, predicted);
        const TagKind kind = finder.find(frame.data(), width, height, uint32_t(start) + 1, corners, margin);
        const int failure = finder.trackFailed;
        const uint32_t intervalUs = flowFound ? uint32_t(start - previousStart) : 0;
        float unskewed[4][2];
        tagPose(finder, kind, corners, margin, uint32_t(start), flowResult.shift, intervalUs, packet, unskewed);
        if (kind)
            memcpy(corners, unskewed, sizeof(corners));
        const auto &work = finder.work;
        const double us = timed ? fixedUs + work.measures * trackUs + work.align.coarse * coarseUs + work.align.fine * fineUs +
                                      work.alignFrame.coarse * frameCoarseUs + work.alignFrame.fine * frameFineUs + (work.windowPixels > 0) * detectorRunUs +
                                      (work.framePixels > 0) * detectorRunUs + work.windowPixels * windowPixelUs +
                                      work.framePixels * framePixelUs
                                : double(std::clock() - began) * 1e6 / CLOCKS_PER_SEC;
        busyUntil = readoutEnd + us;
        previousStart = start;
        printf("%d %d %.2f %.4f %.0f", int(kind), int(packet.valid), margin, packet.reprojectionPx, us);
        for (auto &corner : corners)
            printf(" %.3f %.3f", corner[0], corner[1]);
        for (int i = 0; i < 3; ++i)
            printf(" %.5f", packet.position[i]);
        for (int i = 0; i < 9; ++i)
            printf(" %.5f", packet.rotation[i]);
        printf(" %.3f %.3f %d %d", flowResult.shift[0], flowResult.shift[1], int(flowFound), failure);
        for (auto &corner : predicted)
            printf(" %.3f %.3f", corner[0], corner[1]);
        printf(" %d %.5f %.5f %.5f\n", int(packet.trusted), packet.tagPosition[0], packet.tagPosition[1], packet.tagPosition[2]);
    }
    return 0;
}
