#pragma once
#include <math.h>
#include <string.h>
#include "apriltag.h"
#include "config.h"
#include "packet.h"

// Tag-frame corners in detection order (the library's flipped-Y convention: x right, y down, z into the tag), in
// half tag widths.
constexpr float cornerX[4] = {-1, 1, 1, -1}, cornerY[4] = {1, 1, -1, -1};

// Distorted pixel to normalized pinhole coordinates, inverting OpenCV's Brown-Conrady model by fixed-point iteration.
inline void undistort(const float pixel[2], float normal[2]) {
    const float xd = (pixel[0] - config::cx) / config::fx;
    const float yd = (pixel[1] - config::cy) / config::fy;
    float x = xd, y = yd;
    const float *d = config::distortion;
    for (int step = 0; step < 8; ++step) {
        const float r2 = x*x + y*y;
        const float radial = 1 + r2*(d[0] + r2*(d[1] + r2*d[4]));
        const float dx = 2*d[2]*x*y + d[3]*(r2 + 2*x*x);
        const float dy = d[2]*(r2 + 2*y*y) + 2*d[3]*x*y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }
    normal[0] = x;
    normal[1] = y;
}

inline float shortestSide(const float corners[4][2]) {
    float side = INFINITY;
    for (int i = 0; i < 4; ++i)
        side = fminf(side, hypotf(corners[(i + 1) % 4][0] - corners[i][0], corners[(i + 1) % 4][1] - corners[i][1]));
    return side;
}

// camera = R * tag + t, metres; error is the summed squared reprojection error in pixels.
struct PlanarPose {
    float R[9], t[3], error;
};

// Homography from the corners (half widths) to normalized image points, with H[8] = 1: 8x8 Gaussian elimination.
inline void squareHomography(const float v[4][2], float H[9]) {
    float A[8][9];
    for (int i = 0; i < 4; ++i) {
        const float X = cornerX[i], Y = cornerY[i], u = v[i][0], w = v[i][1];
        const float rowU[9] = {X, Y, 1, 0, 0, 0, -u*X, -u*Y, u};
        const float rowV[9] = {0, 0, 0, X, Y, 1, -w*X, -w*Y, w};
        memcpy(A[2*i], rowU, sizeof(rowU));
        memcpy(A[2*i + 1], rowV, sizeof(rowV));
    }
    for (int col = 0; col < 8; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 8; ++row)
            if (fabsf(A[row][col]) > fabsf(A[pivot][col])) pivot = row;
        for (int k = 0; k < 9; ++k) {
            const float swap = A[col][k];
            A[col][k] = A[pivot][k];
            A[pivot][k] = swap;
        }
        for (int row = 0; row < 8; ++row) {
            if (row == col) continue;
            const float f = A[row][col] / A[col][col];
            for (int k = col; k < 9; ++k) A[row][k] -= f * A[col][k];
        }
    }
    for (int i = 0; i < 8; ++i) H[i] = A[i][8] / A[i][i];
    H[8] = 1;
}

// Least-squares translation for a known rotation: x - u z = 0 and y - v z = 0 at every corner.
inline void planarTranslation(const float v[4][2], float half, PlanarPose &pose) {
    float a02 = 0, a12 = 0, a22 = 0, b0 = 0, b1 = 0, b2 = 0;
    const float *R = pose.R;
    for (int i = 0; i < 4; ++i) {
        const float X = half * cornerX[i], Y = half * cornerY[i], u = v[i][0], w = v[i][1];
        const float rx = R[0]*X + R[1]*Y, ry = R[3]*X + R[4]*Y, rz = R[6]*X + R[7]*Y;
        const float bx = u*rz - rx, by = w*rz - ry;
        a02 -= u;
        a12 -= w;
        a22 += u*u + w*w;
        b0 += bx;
        b1 += by;
        b2 -= u*bx + w*by;
    }
    // Solve [[4,0,a02],[0,4,a12],[a02,a12,a22]] t = b.
    const float tz = (b2 - a02*b0/4 - a12*b1/4) / (a22 - (a02*a02 + a12*a12)/4);
    pose.t[0] = (b0 - a02*tz) / 4;
    pose.t[1] = (b1 - a12*tz) / 4;
    pose.t[2] = tz;
}

inline float planarError(const float v[4][2], float half, const PlanarPose &pose) {
    float error = 0;
    const float *R = pose.R, *t = pose.t;
    for (int i = 0; i < 4; ++i) {
        const float X = half * cornerX[i], Y = half * cornerY[i];
        const float x = R[0]*X + R[1]*Y + t[0], y = R[3]*X + R[4]*Y + t[1], z = R[6]*X + R[7]*Y + t[2];
        if (!(z > 0)) return INFINITY;
        const float dx = config::fx * (x/z - v[i][0]), dy = config::fy * (y/z - v[i][1]);
        error += dx*dx + dy*dy;
    }
    return error;
}

// Both planar solutions of a square by IPPE (Collins and Bartoli 2014, as in OpenCV's SOLVEPNP_IPPE_SQUARE): closed
// form, no allocation. Returns false for a degenerate image.
inline bool solveSquare(const float v[4][2], float half, PlanarPose poses[2]) {
    float H[9];
    squareHomography(v, H);
    // Corners are in half widths; the Jacobian at the tag centre is per metre.
    const float j00 = (H[0] - H[6]*H[2]) / half, j01 = (H[1] - H[7]*H[2]) / half;
    const float j10 = (H[3] - H[6]*H[5]) / half, j11 = (H[4] - H[7]*H[5]) / half;
    const float p = H[2], q = H[5];
    // rv rotates +z onto the tag centre's viewing ray.
    const float n = sqrtf(p*p + q*q + 1), ax = p/n, ay = q/n, az = 1/n, d = 1 / (1 + az);
    const float rv[9] = {1 - ax*ax*d, -ax*ay*d, ax, -ax*ay*d, 1 - ay*ay*d, ay, -ax, -ay, 1 - (ax*ax + ay*ay)*d};
    const float b00 = rv[0] - p*rv[6], b01 = rv[1] - p*rv[7], b10 = rv[3] - q*rv[6], b11 = rv[4] - q*rv[7];
    const float inverse = 1 / (b00*b11 - b01*b10);
    const float a00 = inverse * (b11*j00 - b01*j10), a01 = inverse * (b11*j01 - b01*j11);
    const float a10 = inverse * (b00*j10 - b10*j00), a11 = inverse * (b00*j11 - b10*j01);
    const float s00 = a00*a00 + a01*a01, s01 = a00*a10 + a01*a11, s11 = a10*a10 + a11*a11;
    const float gamma = sqrtf(0.5f * (s00 + s11 + sqrtf((s00 - s11)*(s00 - s11) + 4*s01*s01)));
    if (!(gamma > 1e-7f)) return false;
    const float r00 = a00/gamma, r01 = a01/gamma, r10 = a10/gamma, r11 = a11/gamma;
    const float c0 = sqrtf(fmaxf(0, 1 - r00*r00 - r10*r10));
    const float c1 = copysignf(sqrtf(fmaxf(0, 1 - r01*r01 - r11*r11)), -r00*r01 - r10*r11);
    for (int k = 0; k < 2; ++k) {
        const float e0 = k ? -c0 : c0, e1 = k ? -c1 : c1;
        // Columns of the rotation before Rv: (r00, r10, e0), (r01, r11, e1) and their cross product.
        const float col[3][3] = {{r00, r10, e0}, {r01, r11, e1},
                                 {r10*e1 - e0*r11, e0*r01 - r00*e1, r00*r11 - r01*r10}};
        float *R = poses[k].R;
        for (int row = 0; row < 3; ++row)
            for (int c = 0; c < 3; ++c)
                R[3*row + c] = rv[3*row]*col[c][0] + rv[3*row + 1]*col[c][1] + rv[3*row + 2]*col[c][2];
        planarTranslation(v, half, poses[k]);
        poses[k].error = planarError(v, half, poses[k]);
    }
    return true;
}

// The last valid solution (camera = R * tag + t) and its capture time. While recent, it decides between ambiguous
// solutions: the true tilt changes little between frames, the mirrored one sits tens of degrees away.
float anchorR[9];
uint32_t anchorUs;
bool anchored = false;

inline float angleBetween(const float a[9], const float b[9]) {
    float trace = 0;
    for (int i = 0; i < 9; ++i) trace += a[i] * b[i];
    return acosf(fminf(1, fmaxf(-1, (trace - 1) / 2)));
}

// valid is 1 for an unambiguous pose, 2 for an ambiguous one resolved by continuity with the previous valid pose, and 0
// otherwise. trusted (the corners are good enough for a pose) gives the packet both solutions, the chosen one (else
// the better fitting) first, and the tag's position in the camera frame, also when the tilt stays ambiguous.
inline void estimatePose(const float corners[4][2], float margin, PosePacket &packet, bool trusted = true) {
    packet.margin = margin;
    float v[4][2];
    for (int i = 0; i < 4; ++i) undistort(corners[i], v[i]);
    PlanarPose poses[2];
    if (!solveSquare(v, config::tagSizeM / 2, poses)) return;
    int best = poses[0].error <= poses[1].error ? 0 : 1;
    // A second similarly good solution means the tilt is ambiguous. Below the floor both fit within corner noise.
    const float floor = 4 * config::ambiguityFloorPx * config::ambiguityFloorPx;
    const bool unambiguous = poses[1 - best].error > config::minAmbiguityRatio * fmaxf(poses[best].error, floor);
    if (unambiguous) {
        packet.valid = 1;
    } else if (anchored && packet.captureUs - anchorUs <= config::anchorUs) {
        const float angles[2] = {angleBetween(anchorR, poses[0].R), angleBetween(anchorR, poses[1].R)};
        best = angles[0] <= angles[1] ? 0 : 1;
        const bool decided = angles[best] <= config::anchorMaxRad && angles[best] * config::anchorDecisive <= angles[1 - best];
        packet.valid = decided ? 2 : 0;
    }
    packet.reprojectionPx = sqrtf(poses[best].error / 4);
    packet.altReprojectionPx = sqrtf(poses[1 - best].error / 4);
    bool finite = true;
    // Invert tag -> camera: R_tc = R_ct^T, p_tc = -R_ct^T * t_ct.
    for (int k = 0; k < 2; ++k) {
        const PlanarPose &pose = poses[k ? 1 - best : best];
        float rotation[9], position[3] = {};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                const float r = pose.R[3*col + row];
                rotation[3*row + col] = r;
                position[row] -= r * pose.t[col];
                finite = finite && isfinite(r) && isfinite(position[row]);
            }
        }
        // The packet is packed, so its fields are written whole.
        memcpy(k ? packet.altRotation : packet.rotation, rotation, sizeof(rotation));
        memcpy(k ? packet.altPosition : packet.position, position, sizeof(position));
    }
    memcpy(packet.tagPosition, poses[best].t, sizeof(packet.tagPosition));
    packet.trusted = trusted && finite && packet.reprojectionPx <= config::maxReprojectionPx;
    if (!packet.trusted)
        packet.valid = 0;
    if (packet.valid) {
        memcpy(anchorR, poses[best].R, sizeof(anchorR));
        anchorUs = packet.captureUs;
        anchored = true;
    }
}
