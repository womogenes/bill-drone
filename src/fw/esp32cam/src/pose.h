#pragma once
#include <math.h>
#include "apriltag_pose.h"
#include "common/homography.h"
#include "config.h"
#include "packet.h"

// Undistort four corners, then rebuild the pose solver's initial homography.
inline void undistort(apriltag_detection_t *tag) {
    zarray_t *pairs = zarray_create(4 * sizeof(float));
    for (int i = 0; i < 4; ++i) {
        const float xd = (tag->p[i][0] - config::cx) / config::fx;
        const float yd = (tag->p[i][1] - config::cy) / config::fy;
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
        tag->p[i][0] = config::fx*x + config::cx;
        tag->p[i][1] = config::fy*y + config::cy;
        float pair[4] = {float(i == 1 || i == 2 ? 1 : -1),
                         float(i < 2 ? 1 : -1), tag->p[i][0], tag->p[i][1]};
        zarray_add(pairs, pair);
    }
    matd_destroy(tag->H);
    tag->H = homography_compute(pairs, HOMOGRAPHY_COMPUTE_FLAG_SVD);
    zarray_destroy(pairs);
}

inline float reprojectionError(const apriltag_detection_t *tag, const apriltag_pose_t &pose) {
    float squaredError = 0;
    for (int i = 0; i < 4; ++i) {
        const float x = (i == 1 || i == 2 ? 1 : -1) * config::tagSizeM / 2;
        const float y = (i < 2 ? 1 : -1) * config::tagSizeM / 2;
        float camera[3];
        for (int row = 0; row < 3; ++row)
            camera[row] = MATD_EL(pose.R, row, 0)*x + MATD_EL(pose.R, row, 1)*y + pose.t->data[row];
        if (!(camera[2] > 0)) return INFINITY;
        const float dx = config::fx*camera[0]/camera[2] + config::cx - tag->p[i][0];
        const float dy = config::fy*camera[1]/camera[2] + config::cy - tag->p[i][1];
        squaredError += dx*dx + dy*dy;
    }
    return sqrtf(squaredError / 4);
}

inline void estimatePose(apriltag_detection_t *tag, PosePacket &packet) {
    packet.margin = tag->decision_margin;
    undistort(tag);
    apriltag_detection_info_t info = {tag, config::tagSizeM, config::fx, config::fy, config::cx, config::cy};
    apriltag_pose_t poses[2] = {};
    float errors[2];
    estimate_tag_pose_orthogonal_iteration(&info, &errors[0], &poses[0], &errors[1], &poses[1], 50);
    const int best = errors[0] <= errors[1] ? 0 : 1;
    const apriltag_pose_t &pose = poses[best];
    if (pose.R && pose.t && isfinite(errors[best])) {
        packet.reprojectionPx = reprojectionError(tag, pose);
        // A second similarly good solution means the planar pose is ambiguous.
        const bool unambiguous = !poses[1-best].R ||
            errors[1-best] > config::minAmbiguityRatio * fmaxf(errors[best], 1e-12f);
        packet.valid = unambiguous && isfinite(packet.reprojectionPx) &&
            packet.reprojectionPx <= config::maxReprojectionPx;
        // Invert tag -> camera: R_tc = R_ct^T, p_tc = -R_ct^T * t_ct.
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                const float r = MATD_EL(pose.R, col, row);
                packet.rotation[3*row + col] = r;
                packet.position[row] -= r * pose.t->data[col];
                packet.valid &= isfinite(r) && isfinite(packet.position[row]);
            }
        }
    }
    for (auto &pose : poses) {
        matd_destroy(pose.R);
        matd_destroy(pose.t);
    }
}

