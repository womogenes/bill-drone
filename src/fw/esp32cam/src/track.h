#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "pose.h"


// Frame-to-frame tag tracking without the detector: each edge of the black square is measured afresh in the new frame from intensity profiles across its predicted position, the corners are the intersections of the fitted lines, and the tag's 16 data bits are read back through the new corners and must match the code the detector accepted. Every pose is measured from the frame's own pixels; the prediction only places the search.

// A window of a frame: pixels holds width x height pixels whose top-left is frame pixel (left, top).
struct Image8 {
    const uint8_t *pixels;
    int left, top, width, height;
};

inline float bilinear(const Image8 &image, float x, float y) {
    x -= image.left;
    y -= image.top;
    const int x0 = int(floorf(x)), y0 = int(floorf(y));
    const float fx = x - x0, fy = y - y0;
    const uint8_t *p = image.pixels + y0 * image.width + x0;
    const float top = p[0] + fx * (p[1] - p[0]);
    const float bottom = p[image.width] + fx * (p[image.width + 1] - p[image.width]);
    return top + fy * (bottom - top);
}

inline bool inside(const Image8 &image, float x, float y, float border) {
    x -= image.left;
    y -= image.top;
    return x >= border && y >= border && x < image.width - 1 - border && y < image.height - 1 - border;
}

// Corners in detection order to a homography from the square's half-width coordinates (cornerX, cornerY) to pixels.
inline void cornerHomography(const float corners[4][2], float H[9]) {
    squareHomography(corners, H);
}

inline void project(const float H[9], float x, float y, float &u, float &v) {
    const float w = H[6] * x + H[7] * y + 1;
    u = (H[0] * x + H[1] * y + H[2]) / w;
    v = (H[3] * x + H[4] * y + H[5]) / w;
}

// The family's data bits in its order, read through H; the threshold is the middle of the black border and the white margin. Returns the bits and the mean distance of the bit samples from the threshold.
inline bool readCode(const Image8 &image, const float H[9], const apriltag_family_t *family, uint32_t &code,
                     float &margin) {
    // Bit cell (x, y) of the black square, width_at_border cells wide, maps to half-width coordinates.
    auto sample = [&](float cellX, float cellY, float &value) {
        float u, v;
        const float cells = family->width_at_border;
        project(H, 2 * ((cellX + 0.5f) / cells - 0.5f), 2 * ((cellY + 0.5f) / cells - 0.5f), u, v);
        if (!inside(image, u, v, 0))
            return false;
        value = bilinear(image, u, v);
        return true;
    };
    float black = 0, white = 0, value;
    int blacks = 0, whites = 0;
    for (int i = 0; i < int(family->width_at_border); ++i) {
        const float last = family->width_at_border - 1;
        const float ring[4][2] = {{float(i), 0}, {float(i), last}, {0, float(i)}, {last, float(i)}};
        const float margin4[4][2] = {{float(i), -1}, {float(i), last + 1}, {-1, float(i)}, {last + 1, float(i)}};
        for (int k = 0; k < 4; ++k) {
            if (sample(ring[k][0], ring[k][1], value)) {
                black += value;
                ++blacks;
            }
            if (sample(margin4[k][0], margin4[k][1], value)) {
                white += value;
                ++whites;
            }
        }
    }
    if (blacks < 12 || whites < 12)
        return false;
    const float threshold = (black / blacks + white / whites) / 2;
    code = 0;
    margin = 0;
    for (uint32_t i = 0; i < family->nbits; ++i) {
        if (!sample(family->bit_x[i], family->bit_y[i], value))
            return false;
        code = code << 1 | (value > threshold);
        margin += fabsf(value - threshold);
    }
    margin /= family->nbits;
    return true;
}

// Coarse alignment before the edges are measured: the tag as the predicted corners place it (its code, black border and white margin, one sample per cell) is correlated with the frame at shifts up to radius pixels. Shifts a whole number of pixels apart, about half a cell, are tried on the nearest pixels; then the best is refined with interpolated samples, halving the step down to a pixel. A blurred tag still correlates best where it is. The corners are moved by the best shift; returns its normalized correlation (0 when nothing could be compared), and counts the coarse and fine shifts tried.
struct AlignWork {
    int coarse, fine;
};

inline float alignTag(const Image8 &image, float corners[4][2], const apriltag_family_t *family, uint32_t code,
                      float radius, AlignWork &work) {
    float H[9];
    cornerHomography(corners, H);
    const int cells = int(family->width_at_border), across = cells + 2;
    constexpr int maxPoints = 100;
    float pu[maxPoints], pv[maxPoints];
    int px[maxPoints], py[maxPoints];
    bool white[maxPoints];
    int points = 0;
    for (int cy = -1; cy <= cells && points + across <= maxPoints; ++cy)
        for (int cx = -1; cx <= cells; ++cx) {
            bool value = cx < 0 || cy < 0 || cx >= cells || cy >= cells;
            for (uint32_t i = 0; i < family->nbits; ++i)
                if (int(family->bit_x[i]) == cx && int(family->bit_y[i]) == cy)
                    value = code >> (family->nbits - 1 - i) & 1;
            project(H, 2 * ((cx + 0.5f) / cells - 0.5f), 2 * ((cy + 0.5f) / cells - 0.5f), pu[points], pv[points]);
            px[points] = int(floorf(pu[points] + 0.5f)) - image.left;
            py[points] = int(floorf(pv[points] + 0.5f)) - image.top;
            white[points++] = value;
        }
    // Normalized cross-correlation of the samples with the expected pattern, over the samples in the frame; the pattern is 0 or 1, so the products are the sum over white cells.
    auto correlation = [&](float n, float sum, float squares, float whiteSum, float whites) {
        if (n < points * 3 / 4)
            return -1.0f;
        const float covariance = whiteSum - sum * whites / n;
        const float variance = (squares - sum * sum / n) * (whites - whites * whites / n);
        return variance > 0 ? covariance / sqrtf(variance) : -1.0f;
    };
    auto coarse = [&](int dx, int dy) {
        ++work.coarse;
        int n = 0, whites = 0;
        uint32_t sum = 0, squares = 0, whiteSum = 0;
        for (int i = 0; i < points; ++i) {
            const int x = px[i] + dx, y = py[i] + dy;
            if (x < 0 || y < 0 || x >= image.width || y >= image.height)
                continue;
            const uint32_t value = image.pixels[y * image.width + x];
            ++n;
            sum += value;
            squares += value * value;
            if (white[i]) {
                ++whites;
                whiteSum += value;
            }
        }
        return correlation(n, sum, squares, whiteSum, whites);
    };
    auto fine = [&](float dx, float dy) {
        ++work.fine;
        int n = 0, whites = 0;
        float sum = 0, squares = 0, whiteSum = 0;
        for (int i = 0; i < points; ++i) {
            const float u = pu[i] + dx, v = pv[i] + dy;
            if (!inside(image, u, v, 0))
                continue;
            const float value = bilinear(image, u, v);
            ++n;
            sum += value;
            squares += value * value;
            if (white[i]) {
                ++whites;
                whiteSum += value;
            }
        }
        return correlation(n, sum, squares, whiteSum, whites);
    };
    const int step = max(1, int(shortestSide(corners) / cells / 2 + 0.5f)), steps = int(radius / step);
    float best = -1;
    int bestX = 0, bestY = 0;
    for (int j = -steps; j <= steps; ++j)
        for (int i = -steps; i <= steps; ++i) {
            const float score = coarse(i * step, j * step);
            if (score > best) {
                best = score;
                bestX = i * step;
                bestY = j * step;
            }
        }
    if (best <= 0)
        return 0;
    float x = bestX, y = bestY;
    best = fine(x, y);
    for (float s = step / 2.0f; s >= 0.5f; s /= 2) {
        const float centreX = x, centreY = y;
        for (int j = -1; j <= 1; ++j)
            for (int i = -1; i <= 1; ++i) {
                if (!i && !j)
                    continue;
                const float score = fine(centreX + i * s, centreY + j * s);
                if (score > best) {
                    best = score;
                    x = centreX + i * s;
                    y = centreY + j * s;
                }
            }
    }
    if (best <= 0)
        return 0;
    for (int i = 0; i < 4; ++i) {
        corners[i][0] += x;
        corners[i][1] += y;
    }
    return best;
}

struct EdgeLine {
    // Unit normal (pointing into the tag) and offset: normal . p = offset.
    float nx, ny, offset;
};

// Fits one edge from a to b; the tag interior is on the side of centre. Returns false if too few profiles found it.
inline bool fitEdge(const Image8 &image, const float a[2], const float b[2], const float centre[2], float reach,
                    EdgeLine &line) {
    const float dx = b[0] - a[0], dy = b[1] - a[1], length = hypotf(dx, dy);
    float nx = -dy / length, ny = dx / length;
    if (nx * (centre[0] - a[0]) + ny * (centre[1] - a[1]) < 0) {
        nx = -nx;
        ny = -ny;
    }
    const int samples = max(8, int(length / 1.5f));
    constexpr int maxSamples = 64, half = 8, steps = 2 * half + 1;
    float px[maxSamples], py[maxSamples], weight[maxSamples];
    int found = 0;
    const int n = min(samples, maxSamples);
    const float step = reach / half;
    for (int s = 0; s < n; ++s) {
        const float t = 0.15f + 0.7f * (s + 0.5f) / n;
        const float cx = a[0] + t * dx, cy = a[1] + t * dy;
        if (!inside(image, cx - nx * reach, cy - ny * reach, 1) || !inside(image, cx + nx * reach, cy + ny * reach, 1))
            continue;
        float profile[steps];
        for (int k = 0; k < steps; ++k) {
            const float o = (k - half) * step;
            profile[k] = bilinear(image, cx + nx * o, cy + ny * o);
        }
        // A bright-outside to dark-inside step. Its crossing of the middle level places a window of about two pixels; the edge is then the window's area above the dark level divided by the step height, which is exact for any blur of a straight edge, unlike a gradient peak or a crossing between interpolated samples.
        const float outside = (profile[0] + profile[1] + profile[2] + profile[3]) / 4;
        const float inner = (profile[steps - 4] + profile[steps - 3] + profile[steps - 2] + profile[steps - 1]) / 4;
        const float height = outside - inner;
        float drop = 0;
        for (int k = 1; k < steps - 1; ++k)
            drop = fmaxf(drop, profile[k - 1] - profile[k + 1]);
        if (height < drop / 2)
            continue;
        const float middle = (outside + inner) / 2;
        int crossing = -1;
        for (int k = 0; k < steps - 1 && crossing < 0; ++k)
            if (profile[k] >= middle && profile[k + 1] < middle)
                crossing = k;
        if (crossing < 0)
            continue;
        const int window = max(1, int(1.0f / step + 0.5f));
        const int from = max(0, crossing - window), to = min(steps - 1, crossing + 1 + window);
        float area = 0;
        for (int k = from; k < to; ++k)
            area += fminf(1, fmaxf(0, ((profile[k] + profile[k + 1]) / 2 - inner) / height));
        const float o = (from - half + area) * step;
        px[found] = cx + nx * o;
        py[found] = cy + ny * o;
        weight[found] = height;
        ++found;
    }
    if (found < max(6, n / 2))
        return false;
    // Weighted total least squares, then once more without points over 1 px off the line.
    for (int pass = 0; pass < 2; ++pass) {
        float W = 0, mx = 0, my = 0;
        for (int i = 0; i < found; ++i) {
            W += weight[i];
            mx += weight[i] * px[i];
            my += weight[i] * py[i];
        }
        mx /= W;
        my /= W;
        float sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < found; ++i) {
            const float ex = px[i] - mx, ey = py[i] - my;
            sxx += weight[i] * ex * ex;
            sxy += weight[i] * ex * ey;
            syy += weight[i] * ey * ey;
        }
        // Normal = eigenvector of the smaller eigenvalue.
        const float angle = 0.5f * atan2f(2 * sxy, sxx - syy);
        float lx = -sinf(angle), ly = cosf(angle);
        if (lx * nx + ly * ny < 0) {
            lx = -lx;
            ly = -ly;
        }
        line = {lx, ly, lx * mx + ly * my};
        int kept = 0;
        for (int i = 0; i < found; ++i) {
            if (fabsf(lx * px[i] + ly * py[i] - line.offset) > 1)
                weight[i] = 0;
            kept += weight[i] > 0;
        }
        if (kept < 6)
            return false;
    }
    return true;
}

// Why the last trackCorners failed: 1 an edge, 2 the corner shape, 3 a corner moved past reach, 4 the code unreadable,
// 5 the code did not match.
static int trackFailure = 0;

// Re-measures the corners in this frame from predicted ones. False if any edge, the shape or the code check fails.
inline bool trackCorners(const Image8 &image, float corners[4][2], const apriltag_family_t *family,
                         uint32_t expectedCode, float &margin) {
    float centre[2] = {0, 0};
    for (int i = 0; i < 4; ++i) {
        centre[0] += corners[i][0] / 4;
        centre[1] += corners[i][1] / 4;
    }
    EdgeLine lines[4];
    float side = INFINITY;
    for (int e = 0; e < 4; ++e)
        side = fminf(side, hypotf(corners[(e + 1) % 4][0] - corners[e][0], corners[(e + 1) % 4][1] - corners[e][1]));
    // Search across the edge for two thirds of the black border (a sixth of the tag), so the profile ends stay in the border and the white margin.
    const float reach = fmaxf(1.5f, side / 9);
    for (int e = 0; e < 4; ++e) {
        if (!fitEdge(image, corners[e], corners[(e + 1) % 4], centre, reach, lines[e]))
            return trackFailure = 1, false;
    }
    float fresh[4][2];
    for (int i = 0; i < 4; ++i) {
        const EdgeLine &p = lines[(i + 3) % 4], &q = lines[i];
        const float det = p.nx * q.ny - p.ny * q.nx;
        if (fabsf(det) < 0.2f)
            return trackFailure = 2, false;
        fresh[i][0] = (p.offset * q.ny - p.ny * q.offset) / det;
        fresh[i][1] = (p.nx * q.offset - p.offset * q.nx) / det;
        // Written so that a NaN corner fails too.
        if (!(hypotf(fresh[i][0] - corners[i][0], fresh[i][1] - corners[i][1]) <= reach))
            return trackFailure = 3, false;
    }
    float H[9];
    cornerHomography(fresh, H);
    uint32_t code;
    if (!readCode(image, H, family, code, margin))
        return trackFailure = 4, false;
    if (__builtin_popcount(code ^ expectedCode) > 1)
        return trackFailure = 5, false;
    memcpy(corners, fresh, sizeof(fresh));
    return true;
}
