#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stdint.h>

// One AprilTag pose from the ESP32-CAM (fw/esp32cam/README.md): point_tag = rotation * point_camera + position_m.
typedef struct {
    // The camera's frame number; use each pose once.
    uint32_t sequence;
    // Readout of the tag's centre row on this drone's esp_timer clock: READY's rising edge minus the camera's age.
    int64_t capture_us;
    uint16_t tag_id;
    // 1: unambiguous; 2: the tilt was ambiguous and the camera chose the solution nearest its previous pose; 0: the
    // tilt is ambiguous and undecided (only from camera_candidates).
    uint16_t valid;
    // Camera origin in the tag frame, metres; camera -> tag rotation, row-major.
    float position_m[3], rotation[9];
    // Decode margin and reprojection RMS in pixels: higher margin and lower RMS are better.
    float margin, reprojection_px;
    // The tag centre in the camera frame, metres (camera axes: right, down, forward): well determined even when the
    // tilt is not.
    float tag_position_m[3];
    // The other planar solution (the mirrored tilt) and its reprojection RMS. When valid is 0, the attitude estimate
    // (gravity, with the tag's known mounting) should choose between this and the first solution.
    float alt_position_m[3], alt_rotation[9], alt_reprojection_px;
} CameraPose;

// Image motion between two consecutive camera frames, for velocity after the drone removes rotation (f * gyro rate)
// and scales by depth. Pixels are the camera's (fx, fy in fw/esp32cam/src/config.h); x right, y down.
typedef struct {
    uint32_t sequence;
    // Readout of the newer frame's centre row on this drone's clock, and the time since the older frame's.
    int64_t capture_us;
    uint32_t interval_us;
    float shift_px[2];
    // Median deviation of the matched blocks from the median shift, and how many blocks agreed.
    float residual_px;
    uint16_t inliers;
} CameraFlow;

typedef struct {
    // packets: transfers clocked; frames: well-formed new frames, with or without a tag; poses: accepted poses.
    // corrupt: bad magic or CRC; repeats: a sequence seen last time; stale: older than the limit on arrival;
    // unsynced: READY found high without an edge (at start or after a missed edge), so its wait was unknown.
    uint32_t packets, frames, poses, flows, corrupt, repeats, stale, unsynced;
    // Arrival of the last well-formed frame on the esp_timer clock, -1 before the first.
    int64_t frame_us;
} CameraStats;

// SPI2 master on GPIO27 SCLK, 26 MOSI, 34 MISO, 23 CS; READY on GPIO4 with a pull-down; a receiver task on core 0.
// Not started by app_main: nothing uses the camera yet.
void camera_start(void);
// Copies the newest frame's pose if that frame saw the tag, the camera decided its tilt (valid 1 or 2), and the pose
// is at most max_age_us old now.
bool camera_pose(CameraPose *pose, uint32_t max_age_us);
// As camera_pose, but also when the tilt is undecided (valid 0): both solutions of every frame whose corners the camera
// trusted.
bool camera_candidates(CameraPose *pose, uint32_t max_age_us);
// Copies the newest frame's flow if it has one and it is at most max_age_us old now.
bool camera_flow(CameraFlow *flow, uint32_t max_age_us);
void camera_stats(CameraStats *stats);

#endif
