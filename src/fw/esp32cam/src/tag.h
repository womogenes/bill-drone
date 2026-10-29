#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "apriltag.h"
#include "config.h"
#include "track.h"

// Finding the tag in one frame, independent of the camera and the board so the host simulator (fw/esp32cam/sim) runs the same code: re-measure the tracked tag (track.h), else run the detector around where it was, else on the whole frame.

enum TagKind : uint8_t { tagNone = 0, tagTracked = 1, tagDetected = 2 };
enum Workspace { workspaceDefault, workspaceInternal, workspaceExternal };

struct TagFinder {
    apriltag_detector_t *detector = nullptr;
    apriltag_family_t *family = nullptr;
    // Where the detector's allocations go: called before a run in a window (which may use internal RAM) or on the whole frame (which must not starve Wi-Fi of it), and with workspaceDefault after; null on the host.
    void (*workspace)(Workspace where) = nullptr;
    // The tag's corners when last seen, at seenUs (frame start); tracking is false after a miss. Any tag16h5 ID is taken; id is the one last found (-1 before any).
    bool tracking = false;
    int id = -1;
    float corners[4][2] = {};
    uint32_t seenUs = 0;
    // The corners' velocity in pixels per frame period, over the interval before seenUs, which ended lag frame periods after the velocity's moment; known when measured over a recent interval (else zero), and fresh when that interval was about one frame. Their acceleration is measured between two fresh velocities, else zero.
    bool velocityKnown = false, velocityFresh = false;
    float velocity[4][2] = {}, acceleration[4][2] = {}, lag = 0;
    // A tag seen within this many frame periods (0.5 s) is first looked for where its velocity puts it.
    static constexpr float recentFrames = 15;
    // Image speed assumed, in pixels per frame period, when the tag's velocity is not known.
    static constexpr float unknownSpeedPx = 15;
    // Shortest side of the last accepted tag, in pixels; zero after a miss.
    float sidePx = 0;
    // What the last find did, for timing: the edge tracker's runs, the alignment's shifts tried in the internal copy and (when the neighbourhood did not fit) in the PSRAM frame, and the decimated pixels searched by the detector in windows and on whole frames.
    struct Work {
        int measures;
        AlignWork align, alignFrame;
        int windowPixels, framePixels;
    } work = {};
    // The tracker's alignment search reaches up to maxAlignPx from the prediction and is used when its correlation is at least minAlignment; alignment is the last one's correlation.
    float maxAlignPx = 40, minAlignment = 0.5f, alignment = 0;
    // Whether the edge tracker measured the last corners found (always when tracked, and when it confirmed a detection), and why tracking failed in the last find (trackFailure), or 0.
    bool measured = false;
    int trackFailed = 0;
    // Faster than this, in pixels per frame period, blur and changing motion corrupt the tag's shape too much for a pose.
    float maxPoseSpeedPx = 30;
    // Decimation of whole-frame searches, and what a detection must meet.
    float frameDecimate = 3;
    int maxHamming = 0;
    float minMargin = config::minMargin;
    // The tracked tag's neighbourhood, copied out of the frame, and where the last measure read: this copy, or the frame itself when the neighbourhood did not fit (the black box crops the tag from it).
    uint8_t window[24 * 1024];
    Image8 measuredImage = {};

    struct Window {
        int x, y, w, h;
    };

    // The detector, on a window around the predicted corners (or the whole frame): finds quads at a resolution that keeps the tag's sides about decimatedSidePx and refines and decodes at full resolution. Corners come back in the calibration's pixel convention, centres at integers; the detector's are half a pixel further right and down.
    bool detect(const uint8_t *frame, int width, int height, const float (*predicted)[2], float out[4][2],
                float &margin, float uncertainty = 0) {
        Window searched = {};
        if (predicted) {
            // The box of the predicted and the last corners (in case the motion stopped), grown by half the tag's size,
            // 8 px and the prediction's uncertainty on each side.
            float low[2] = {INFINITY, INFINITY}, high[2] = {-INFINITY, -INFINITY};
            for (int i = 0; i < 4; ++i)
                for (int axis = 0; axis < 2; ++axis) {
                    low[axis] = fminf(low[axis], fminf(predicted[i][axis], corners[i][axis]));
                    high[axis] = fmaxf(high[axis], fmaxf(predicted[i][axis], corners[i][axis]));
                }
            const float grow = sidePx / 2 + 8 + uncertainty;
            const int x0 = low[0] - grow > 0 ? int(low[0] - grow) : 0, y0 = low[1] - grow > 0 ? int(low[1] - grow) : 0;
            const int x1 = high[0] + grow + 1 < width ? int(high[0] + grow) + 1 : width;
            const int y1 = high[1] + grow + 1 < height ? int(high[1] + grow) + 1 : height;
            if (x1 - x0 >= 16 && y1 - y0 >= 16)
                searched = {x0, y0, x1 - x0, y1 - y0};
        }
        image_u8_t image = {searched.w ? searched.w : width, searched.w ? searched.h : height, width,
                            const_cast<uint8_t *>(frame) + searched.y * width + searched.x};
        detector->quad_decimate = searched.w ? fminf(3, fmaxf(1, floorf(sidePx / config::decimatedSidePx))) : frameDecimate;
        // In a window, a cluster much shorter than the tag's outline (such as a single data bit) cannot be the tag.
        const int minimum = int(config::clusterPerSide * sidePx / detector->quad_decimate);
        detector->qtp.min_cluster_pixels =
            searched.w && minimum > config::minClusterPixels ? minimum : config::minClusterPixels;
        const int searchedPixels = int(image.width * image.height / (detector->quad_decimate * detector->quad_decimate));
        (searched.w ? work.windowPixels : work.framePixels) += searchedPixels;
        if (workspace)
            workspace(searched.w ? workspaceInternal : workspaceExternal);
        zarray_t *tags = apriltag_detector_detect(detector, &image);
        if (workspace)
            workspace(workspaceDefault);
        // The tag last found when it is among the acceptable detections, else the clearest; refused when its ID shows up twice.
        apriltag_detection_t *target = nullptr;
        for (int i = 0; i < zarray_size(tags); ++i) {
            apriltag_detection_t *tag;
            zarray_get(tags, i, &tag);
            if (tag->hamming > maxHamming || tag->decision_margin < minMargin)
                continue;
            if (!target || (tag->id == id) > (target->id == id) ||
                ((tag->id == id) == (target->id == id) && tag->decision_margin > target->decision_margin))
                target = tag;
        }
        int matches = 0;
        for (int i = 0; target && i < zarray_size(tags); ++i) {
            apriltag_detection_t *tag;
            zarray_get(tags, i, &tag);
            matches += tag->id == target->id;
        }
        const bool found = matches == 1;
        if (found) {
            id = target->id;
            for (int i = 0; i < 4; ++i) {
                out[i][0] = target->p[i][0] + searched.x - 0.5f;
                out[i][1] = target->p[i][1] + searched.y - 0.5f;
            }
            margin = target->decision_margin;
        }
        apriltag_detections_destroy(tags);
        return found;
    }

    // Rolling shutter: each row is read config::rowUs after the one above, so a moving tag is sheared, which looks like tilt. These are the corners as they were when the tag's middle row (between its top and bottom corners, the capture time) was read, by the tag's velocity brought forward to this frame, else by flow (the image shift in pixels per frame period, or null).
    void unskew(const float in[4][2], const float *flow, float out[4][2]) const {
        float top = INFINITY, bottom = -INFINITY;
        for (int i = 0; i < 4; ++i) {
            top = fminf(top, in[i][1]);
            bottom = fmaxf(bottom, in[i][1]);
        }
        for (int i = 0; i < 4; ++i) {
            const float frames = (in[i][1] - (top + bottom) / 2) * config::rowUs / config::framePeriodUs;
            for (int axis = 0; axis < 2; ++axis)
                out[i][axis] = in[i][axis] - frames * (velocityFresh ? velocity[i][axis] + lag * acceleration[i][axis]
                                                       : flow                ? flow[axis]
                                                                             : 0);
        }
    }

    // Whether the last corners found are good enough for a pose: measured by the edge tracker, with the rolling shutter removable (velocity from consecutive frames, else flow: the image shift per frame period over at most about one, or null), and moving slower than maxPoseSpeedPx per frame period, beyond which blur and changing motion corrupt the shape.
    bool trustworthy(const float *flow) const {
        if (!measured || !(velocityFresh || flow))
            return false;
        float speed = velocityFresh ? 0 : hypotf(flow[0], flow[1]);
        if (velocityFresh)
            for (int i = 0; i < 4; ++i)
                speed = fmaxf(speed, hypotf(velocity[i][0], velocity[i][1]));
        return speed <= maxPoseSpeedPx;
    }

    // Frame periods from when the tag was last seen to a frame starting at timeUs (frames can be skipped).
    float framesSince(uint32_t timeUs) const { return float(timeUs - seenUs) / config::framePeriodUs; }

    // Where the corners should be in a frame starting at timeUs, by their last velocity.
    void predict(uint32_t timeUs, float out[4][2]) const {
        const float frames = framesSince(timeUs);
        for (int i = 0; i < 4; ++i)
            for (int axis = 0; axis < 2; ++axis)
                out[i][axis] = corners[i][axis] + frames * velocity[i][axis];
    }

    // Re-measures the tag near the corners out in this frame with the edge tracker, after aligning them within radius pixels. False if the edges or the code do not check out.
    bool measure(const uint8_t *frame, int width, int height, float out[4][2], float radius, float &margin) {
        float low[2] = {INFINITY, INFINITY}, high[2] = {-INFINITY, -INFINITY};
        for (int i = 0; i < 4; ++i)
            for (int axis = 0; axis < 2; ++axis) {
                low[axis] = fminf(low[axis], out[i][axis]);
                high[axis] = fmaxf(high[axis], out[i][axis]);
            }
        // The tracker reads around the tag at random; a copy in internal RAM, when it fits, beats the PSRAM frame.
        const float grow = fmaxf(high[0] - low[0], high[1] - low[1]) / 4 + 4 + radius;
        const int left = low[0] - grow > 0 ? int(low[0] - grow) : 0, top = low[1] - grow > 0 ? int(low[1] - grow) : 0;
        const int right = high[0] + grow + 2 < width ? int(high[0] + grow) + 2 : width;
        const int bottom = high[1] + grow + 2 < height ? int(high[1] + grow) + 2 : height;
        Image8 image = {frame, 0, 0, width, height};
        if (right > left && bottom > top && (right - left) * (bottom - top) <= int(sizeof(window))) {
            for (int y = top; y < bottom; ++y)
                memcpy(window + (y - top) * (right - left), frame + y * width + left, right - left);
            image = {window, left, top, right - left, bottom - top};
        }
        measuredImage = image;
        ++work.measures;
        const uint32_t code = uint32_t(family->codes[id]);
        float aligned[4][2];
        memcpy(aligned, out, sizeof(aligned));
        alignment = alignTag(image, aligned, family, code, radius, image.pixels == frame ? work.alignFrame : work.align);
        if (alignment >= minAlignment)
            memcpy(out, aligned, sizeof(aligned));
        return trackCorners(image, out, family, code, margin);
    }

    // Finds the tag in a width x height Y8 frame that started at timeUs. Returns the kind, with the corners and margin when found; measured says whether the edge tracker measured these corners (always when tracked).
    TagKind find(const uint8_t *frame, int width, int height, uint32_t timeUs, float out[4][2], float &margin) {
        TagKind kind = tagNone;
        work = {};
        measured = false;
        alignment = 0;
        trackFailed = 0;
        float guess[4][2];
        const float frames = framesSince(timeUs);
        const bool recent = seenUs != 0 && frames <= recentFrames;
        predict(timeUs, guess);
        // The prediction can be off by about the distance moved since, as the motion changes, or when the velocity is not known, by as far as a quick hand moves the view.
        float speed = velocityKnown ? 0 : unknownSpeedPx;
        for (int i = 0; i < 4; ++i)
            speed = fmaxf(speed, hypotf(velocity[i][0], velocity[i][1]));
        if (tracking) {
            // The last corners, moved by their velocity, are aligned with this frame and then re-measured.
            memcpy(out, guess, sizeof(guess));
            if (measure(frame, width, height, out, fminf(maxAlignPx, 6 + frames * speed), margin))
                kind = tagTracked;
            else
                trackFailed = trackFailure;
            // If the motion stopped or reversed, the tag is nearer where it was.
            float moved = 0;
            for (int i = 0; i < 4; ++i)
                moved = fmaxf(moved, hypotf(guess[i][0] - corners[i][0], guess[i][1] - corners[i][1]));
            if (!kind && moved > 3) {
                memcpy(out, corners, sizeof(corners));
                if (measure(frame, width, height, out, fminf(maxAlignPx, 6 + frames * unknownSpeedPx), margin))
                    kind = tagTracked;
            }
        }
        // Then the detector around the prediction while the tag was seen recently, and on the whole frame.
        if (!kind && recent && detect(frame, width, height, guess, out, margin, 0.5f * frames * speed))
            kind = tagDetected;
        if (!kind && detect(frame, width, height, nullptr, out, margin))
            kind = tagDetected;
        // The detector's corners are refined by the tracker, which measures blurred edges better.
        measured = kind == tagTracked;
        if (kind == tagDetected) {
            float refined[4][2], unused;
            memcpy(refined, out, sizeof(refined));
            if (measure(frame, width, height, refined, 0, unused)) {
                memcpy(out, refined, sizeof(refined));
                measured = true;
            }
        }
        tracking = kind != tagNone;
        if (tracking) {
            // The velocity is the motion since the tag was last seen, unless that was too long ago to say anything.
            const bool wasFresh = velocityFresh;
            velocityKnown = recent && frames > 0.5f;
            velocityFresh = velocityKnown && frames <= 1.5f;
            for (int i = 0; i < 4; ++i)
                for (int axis = 0; axis < 2; ++axis) {
                    const float fresh = velocityKnown ? (out[i][axis] - corners[i][axis]) / frames : 0;
                    acceleration[i][axis] = wasFresh && velocityFresh ? (fresh - velocity[i][axis]) / (lag + frames / 2) : 0;
                    velocity[i][axis] = fresh;
                }
            lag = frames / 2;
            memcpy(corners, out, sizeof(corners));
            seenUs = timeUs;
            sidePx = shortestSide(out);
        }
        return kind;
    }
};

// The tag fields of this frame's packet from a find: the capture time, the readout of the tag's middle row (of the frame's middle row without a tag), and the pose from the corners with the rolling shutter removed, which can be valid only if the finder trusts them. flow is this frame's image shift since the frame intervalUs earlier (0 for none). unskewed gets the corners the pose used. After a miss, an ambiguous pose cannot be resolved until an unambiguous one.
inline void tagPose(const TagFinder &finder, TagKind kind, const float corners[4][2], float margin,
                    uint32_t frameStartUs, const float flow[2], uint32_t intervalUs, PosePacket &packet,
                    float unskewed[4][2]) {
    packet.captureUs = frameStartUs + uint32_t(config::outH / 2 * config::rowUs);
    if (!kind) {
        anchored = false;
        return;
    }
    float top = INFINITY, bottom = -INFINITY;
    for (int i = 0; i < 4; ++i) {
        top = fminf(top, corners[i][1]);
        bottom = fmaxf(bottom, corners[i][1]);
    }
    packet.captureUs = frameStartUs + uint32_t((top + bottom) / 2 * config::rowUs);
    // The flow per frame period, if it spans about one.
    const float periods = float(intervalUs) / config::framePeriodUs;
    const float rate[2] = {intervalUs ? flow[0] / periods : 0, intervalUs ? flow[1] / periods : 0};
    const float *usable = intervalUs && periods <= 1.5f ? rate : nullptr;
    finder.unskew(corners, usable, unskewed);
    estimatePose(unskewed, margin, packet, finder.trustworthy(usable));
}
