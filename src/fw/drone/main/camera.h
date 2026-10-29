#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// One AprilTag pose from the ESP32-CAM (fw/esp32cam/README.md): point_tag = rotation * point_camera + position_m.
typedef struct {
    // The camera's frame number; use each pose once.
    uint32_t sequence;
    // Readout of the tag's centre row on this drone's esp_timer clock: the SYNC edge minus the camera's age.
    int64_t capture_us;
    uint16_t tag_id;
    // 1: unambiguous; 2: the tilt was ambiguous and the camera chose the solution nearest its previous pose; 0: the tilt is ambiguous and undecided (only from camera_candidates).
    uint16_t valid;
    // Camera origin in the tag frame, metres; camera -> tag rotation, row-major.
    float position_m[3], rotation[9];
    // Decode margin and reprojection RMS in pixels: higher margin and lower RMS are better.
    float margin, reprojection_px;
    // The tag centre in the camera frame, metres (camera axes: right, down, forward): well determined even when the tilt is not.
    float tag_position_m[3];
    // The other planar solution (the mirrored tilt) and its reprojection RMS. When valid is 0, the attitude estimate (gravity, with the tag's known mounting) should choose between this and the first solution.
    float alt_position_m[3], alt_rotation[9], alt_reprojection_px;
} CameraPose;

// Image motion between two consecutive camera frames, for velocity after the drone removes rotation (f * gyro rate) and scales by depth. Pixels are the camera's (fx, fy in fw/esp32cam/src/config.h); x right, y down.
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
    // packets: packets with the magic and a matching CRC; frames: those that were new and dated, with or without a tag; poses: accepted poses. corrupt: full-length candidates whose CRC failed; repeats: a sequence seen last time; stale: older than the limit on arrival; unsynced: no unused SYNC edge in the 10 ms before the packet completed, so its capture time is unknown and it is discarded.
    uint32_t packets, frames, poses, flows, corrupt, repeats, stale, unsynced;
    // skipped_bytes: bytes that could not start a packet (the camera's boot messages, noise); overflows: the receive buffer overflowed and was flushed; line_errors: framing, parity or break errors (115200-baud boot messages).
    uint32_t skipped_bytes, overflows, line_errors;
    // SYNC rising edges, one per packet the camera queues; rx_edges: falling edges on the RX pin, counted in hardware, so a silent wire can be told from a receiver fault.
    uint32_t sync_edges, rx_edges;
    // Arrival of the last well-formed frame on the esp_timer clock, -1 before the first.
    int64_t frame_us;
    // That frame's sequence, capture time, and its pose's valid (as CameraPose) and trusted flags, whether or not the pose was accepted.
    uint32_t sequence;
    int64_t capture_us;
    uint8_t valid, trusted;
} CameraStats;

// UART2 at 2 Mbaud, 8N1: RX on GPIO34 from the camera's TX (GPIO1), TX on GPIO27 to its RX (GPIO3, camera firmware updates); SYNC from the camera's GPIO13 on GPIO13 with a pull-down; a receiver task on core 0 at priority 4, below the control task on core 1. Call from a core-0 task (app_main) so the UART and GPIO interrupts are on core 0 too.
void camera_start(void);
// Copies the newest frame's pose if that frame saw the tag, the camera decided its tilt (valid 1 or 2), and the pose is at most max_age_us old now.
bool camera_pose(CameraPose *pose, uint32_t max_age_us);
// As camera_pose, but also when the tilt is undecided (valid 0): both solutions of every frame whose corners the camera trusted.
bool camera_candidates(CameraPose *pose, uint32_t max_age_us);
// Copies the newest frame's flow if it has one and it is at most max_age_us old now.
bool camera_flow(CameraFlow *flow, uint32_t max_age_us);
void camera_stats(CameraStats *stats);
// The largest body of a camera update request: a sector write (index, hash prefix, 4 KB).
#define CAMERA_UPDATE_BODY (2 + 8 + 4096)
// Sends one camera firmware update request (fw/esp32cam/src/update.h: op I, W, B or X and its body) and waits up to timeout_ms for the answer. Returns an error, or NULL with the answer's text in text (NUL-terminated). Call from one task at a time.
const char *camera_update(uint8_t op, const uint8_t *body, size_t length, uint32_t timeout_ms, char *text, size_t size);

#endif
