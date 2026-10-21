#pragma once
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Global image motion between consecutive frames, for the drone to turn into velocity after removing rotation with its
// gyro. Each frame is reduced to a quarter-size image; the most textured blocks of the previous one are found in the
// current one by sum of absolute differences, refined by two Lucas-Kanade steps, and the shift is the mean of the
// blocks within one small pixel of the median. The tag's area can be excluded.

namespace flow {
constexpr int scale = 4, width = 80, height = 60;
constexpr int block = 8, reach = 4, blocks = 12;

struct Result {
    float shift[2];   // Full-resolution pixels, previous frame to this one; image right and down.
    float residualPx; // Median absolute deviation of the accepted blocks from the median shift.
    int inliers;      // Blocks within 1 small pixel of the median.
};

uint8_t images[2][width * height];
int current = 0;
bool primed = false;

// Reduce a full-resolution frame by scale: the mean of four pixels from one row in four, one 32-bit read each (frames
// are word-aligned). Reading the whole frame from PSRAM would cost about 6 ms.
inline void reduce(const uint8_t *frame, int frameWidth, uint8_t *out) {
    for (int y = 0; y < height; ++y) {
        const uint32_t *p = reinterpret_cast<const uint32_t *>(frame + (y * scale + scale / 2) * frameWidth);
        for (int x = 0; x < width; ++x) {
            const uint32_t pairs = (p[x] & 0x00ff00ff) + ((p[x] >> 8) & 0x00ff00ff);
            out[y * width + x] = ((pairs & 0xffff) + (pairs >> 16)) / scale;
        }
    }
}

// Sum of absolute differences, abandoned by rows once it exceeds limit.
inline unsigned sad(const uint8_t *a, const uint8_t *b, unsigned limit) {
    unsigned sum = 0;
    for (int y = 0; y < block && sum < limit; ++y, a += width, b += width)
        for (int x = 0; x < block; ++x)
            sum += abs(a[x] - b[x]);
    return sum;
}

inline float median(float *values, int n) {
    for (int i = 1; i < n; ++i)
        for (int j = i; j > 0 && values[j] < values[j - 1]; --j) {
            const float swap = values[j];
            values[j] = values[j - 1];
            values[j - 1] = swap;
        }
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}

// Adds a frame. Returns false for the first frame. exclude is a full-resolution box (x0, y0, x1, y1) to ignore, or
// null.
inline bool update(const uint8_t *frame, int frameWidth, const float *exclude, Result &result) {
    current ^= 1;
    reduce(frame, frameWidth, images[current]);
    if (!primed) {
        primed = true;
        return false;
    }
    const uint8_t *previous = images[current ^ 1], *now = images[current];
    // Candidate blocks on a grid, scored by gradient energy in the previous image.
    struct Candidate {
        int x, y;
        unsigned score;
    } best[blocks];
    int count = 0;
    for (int y = reach; y + block + reach <= height; y += block / 2)
        for (int x = reach; x + block + reach <= width; x += block / 2) {
            if (exclude && (x + block) * scale > exclude[0] && x * scale < exclude[2] &&
                (y + block) * scale > exclude[1] && y * scale < exclude[3])
                continue;
            unsigned gx = 0, gy = 0;
            for (int j = 0; j < block; ++j)
                for (int i = 0; i < block; ++i) {
                    const uint8_t *p = previous + (y + j) * width + x + i;
                    gx += abs(p[1] - p[-1]);
                    gy += abs(p[width] - p[-width]);
                }
            // Both directions must be textured, or the block slides along an edge.
            const unsigned score = gx < gy ? gx : gy;
            if (count < blocks) {
                best[count++] = {x, y, score};
            } else {
                int worst = 0;
                for (int k = 1; k < blocks; ++k)
                    if (best[k].score < best[worst].score)
                        worst = k;
                if (score > best[worst].score)
                    best[worst] = {x, y, score};
            }
        }
    float dx[blocks], dy[blocks];
    int found = 0;
    for (int k = 0; k < count; ++k) {
        if (best[k].score < block * block * 4)
            continue;
        const uint8_t *reference = previous + best[k].y * width + best[k].x;
        unsigned lowest = UINT32_MAX;
        int bx = 0, by = 0;
        for (int v = -reach; v <= reach; ++v)
            for (int u = -reach; u <= reach; ++u) {
                const unsigned c = sad(reference, now + (best[k].y + v) * width + best[k].x + u, lowest);
                if (c < lowest) {
                    lowest = c;
                    bx = u;
                    by = v;
                }
            }
        if (abs(bx) == reach || abs(by) == reach)
            continue;
        // Sub-pixel: two Lucas-Kanade steps from the best integer shift, with the previous block's gradients.
        float u = bx, v = by;
        for (int step = 0; step < 2; ++step) {
            float gxx = 0, gxy = 0, gyy = 0, bxs = 0, bys = 0;
            const int iu = int(floorf(u)), iv = int(floorf(v));
            const float fu = u - iu, fv = v - iv;
            for (int j = 0; j < block; ++j)
                for (int i = 0; i < block; ++i) {
                    const uint8_t *r = reference + j * width + i;
                    const float gx = (r[1] - r[-1]) * 0.5f, gy = (r[width] - r[-width]) * 0.5f;
                    const uint8_t *c = now + (best[k].y + iv + j) * width + best[k].x + iu + i;
                    const float sampled = (c[0] * (1 - fu) + c[1] * fu) * (1 - fv) + (c[width] * (1 - fu) + c[width + 1] * fu) * fv;
                    const float difference = sampled - r[0];
                    gxx += gx * gx;
                    gxy += gx * gy;
                    gyy += gy * gy;
                    bxs += gx * difference;
                    bys += gy * difference;
                }
            const float det = gxx * gyy - gxy * gxy;
            if (det <= 0)
                break;
            u -= (gyy * bxs - gxy * bys) / det;
            v -= (gxx * bys - gxy * bxs) / det;
        }
        if (fabsf(u - bx) > 1 || fabsf(v - by) > 1)
            continue;
        dx[found] = u;
        dy[found] = v;
        ++found;
    }
    if (found < 3)
        return false;
    float sx[blocks], sy[blocks];
    memcpy(sx, dx, sizeof(float) * found);
    memcpy(sy, dy, sizeof(float) * found);
    const float mx = median(sx, found), my = median(sy, found);
    float deviation[blocks];
    float sumX = 0, sumY = 0;
    int inliers = 0;
    for (int k = 0; k < found; ++k) {
        deviation[k] = hypotf(dx[k] - mx, dy[k] - my);
        if (deviation[k] <= 1) {
            sumX += dx[k];
            sumY += dy[k];
            ++inliers;
        }
    }
    result.shift[0] = sumX / inliers * scale;
    result.shift[1] = sumY / inliers * scale;
    result.residualPx = median(deviation, found) * scale;
    result.inliers = inliers;
    return true;
}
}
