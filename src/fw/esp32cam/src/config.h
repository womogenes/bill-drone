#pragma once
#include "sensor.h"

namespace config {
// Measure the black square's outer edge, excluding the white margin.
float tagSizeM = 0.151350000f;

// Calibrate at 320x240 with hmirror=0 and vflip=1, in the sensor mode below. Zero means unset.
// The whole-sensor mode's: the central-window calibration (fx 492.660, fy 494.564, cx 160.561, cy 107.775) scaled by 1536/2048 about the image centre, which reproduces the original whole-sensor calibration to every digit. The distortion coefficients act on normalised coordinates, so they carry over.
float fx = 369.494757463f, fy = 370.922837184f, cx = 160.420727397f, cy = 110.831046213f;
// OpenCV Brown-Conrady coefficients: k1, k2, p1, p2, k3.
float distortion[5] = {-0.031215286f, 0.896256747f, 0.000000000f, 0.000000000f, 0.000000000f};

// Link to the drone: UART0 (the USB-serial pins, GPIO1 TX and GPIO3 RX) at linkBaud, 8N1, and syncPin pulsed high for syncPulseUs as each packet is queued, so the drone can date it. GPIO14, 15 and 2 are the microSD slot in 1-bit mode; GPIO12 (a boot strap that must stay low) is unconnected and GPIO4 (the flash LED) is held low.
constexpr int linkTxPin = 1, linkRxPin = 3, syncPin = 13, flashLedPin = 4;
constexpr unsigned long linkBaud = 2000000;
constexpr unsigned syncPulseUs = 20;
// Black box on the microSD card (sd.h): a ring in PSRAM fed by the detector and written in staging-sized pieces from DMA-capable internal RAM, the tag crop's side, how often a frame also carries the whole 320x240 image (0 never), and the size at which a file is closed and the next one started (FAT32 files stop at 4 GiB).
constexpr uint32_t blackBoxRingBytes = 1 << 20, blackBoxStagingBytes = 8192;
constexpr int blackBoxCropSize = 96;
constexpr unsigned blackBoxFullEvery = 0;
constexpr uint64_t blackBoxFileBytes = 3500000000ull;
constexpr float minMargin = 30.0f;
constexpr float maxReprojectionPx = 2.0f;
constexpr float minAmbiguityRatio = 2.0f, ambiguityFloorPx = 0.3f;
// An ambiguous pose takes the solution nearest the last valid one if that is this recent and this close, and the other solution is at least anchorDecisive times further away.
constexpr uint32_t anchorUs = 500000;
constexpr float anchorMaxRad = 0.35f, anchorDecisive = 2;
// Capture-to-ready limit; a tracked pose is typically 35-45 ms old (up to 33 ms of readout, then tracking).
constexpr unsigned maxAgeUs = 150000;
// OV3660 mode: the whole 2048x1536 array (about 47 x 36 degrees through this lens), binned 2x2 and scaled to 320x240; the central 1536x1152 window (36 x 27 degrees) ran at 29.4 fps but lost the tag on modest attitude swings in hover. A line takes hts SYSCLK cycles; a frame is the window's binned lines plus 15. The ESP32's 10 MHz pixel clock needs 32 us to move a 320-pixel line, and a faster line rate than this panicked, so 320x240 stops at 29.4 fps from the central window and 22.2 fps from the whole array. For 100 fps (measured 101.5 Hz, detection off) the output must be 160x120: outW 160, outH 120, hts 1300, sysclkMhz 80 (PLL multiplier and PCLK divider 16), frame buffers in DRAM, intrinsics halved. Its 9.6 ms exposure needs several times this room's light, and whole-frame searches with DRAM frame buffers ran internal RAM down to 5 KB and panicked.
constexpr int windowW = 2048, windowH = 1536, outW = 320, outH = 240;
constexpr int hts = 2300, sysclkMhz = 40, vts = windowH / 2 + 15;
constexpr unsigned framePeriodUs = unsigned(vts) * hts / sysclkMhz;
// Readout time per output row.
constexpr float rowUs = float(windowH / 2) / outH * hts / sysclkMhz;
// A frame older than this when fetched is replaced by the next capture; a just-finished one is one period old.
constexpr unsigned maxFrameAgeUs = 2 * framePeriodUs;
// Exposure in lines of 57.5 us: 174 is 10 ms, a third of the frame. It grows by a quarter while the gain is above raiseExposureGain and shrinks back while the gain is below lowerExposureGain. Measured on 2026-10-21 with the tag at 1.7 m, 10 ms needed 26.6x gain and raised the position jitter from 4.4/3.2/1.8 mm (33 ms) to 8.6/5.9/3.3 mm.
constexpr int exposureLines = 174;
constexpr float raiseExposureGain = 56, lowerExposureGain = 24;
// FPV (fpv.h): the sensor's own JPEG at automatic exposure, no tag tracking. Measured on 2026-10-30 (OV3660, 20 MHz XCLK): 480x320 comes off the sensor every 31.2 ms, VGA and QVGA every 36 ms; a 24 MHz XCLK reads out faster but the driver then loses frames with Wi-Fi busy on the same core. The JPEG quality (fpvQualityRange, lower is finer) holds each frame near fpvTargetBytes, four datagrams, whatever the light: that bounds the video's airtime on a channel the drone's radio may share (it took the drone's status stream from 20.0 to 19.3 messages/s on its channel), and a frame loses one datagram less often. Over a 2-minute soak the dashboard received 32.0 fps with no frame lost, gaps p99 55 ms, max 211 ms. It switches back to tracking fpvLingerMs after the last keepalive, so a Wi-Fi dropout resumes without restarting the camera twice.
constexpr framesize_t fpvFrameSize = FRAMESIZE_HVGA;
constexpr size_t fpvTargetBytes = 5000;
// Through the FPV receiver (fw/fpv_rx) the video must fit its USB bridge, about 98 KB/s at 1 Mbaud: the sensor's frame is lengthened until the average frame fits fpvRadioBytesPerSecond, so every frame goes out evenly spaced (the receiver drops whole frames if a burst outruns it), and their size aims lower. fpvFramePeriodS is the sensor's own 480x320 JPEG frame. Each goes out fpvRadioCopies times: with one, a fifth of the frames lost a chunk (2026-10-30); with two, 1%; three did no better.
constexpr size_t fpvRadioTargetBytes = 2600;
constexpr float fpvRadioBytesPerSecond = 90000, fpvFramePeriodS = 0.0312f;
constexpr int fpvRadioCopies = 2;
constexpr int fpvQualityRange[2] = {10, 63};
constexpr uint32_t fpvLingerMs = 3000;
// Over the radio the keepalive (the drone's K V) goes unanswered for about 2 s at times while the video runs, so FPV outlasts two missed ones.
constexpr uint32_t fpvRadioLingerMs = 6000;
// While tracking, detector allocations below this size go to internal RAM.
constexpr size_t trackInternalBytes = 32768;
// While tracking, quads are found at the decimation (1 to 3) that keeps the tag's shortest side near this.
constexpr float decimatedSidePx = 12;
// Smallest edge cluster considered, and while tracking at least this many points per decimated tag side.
constexpr int minClusterPixels = 24;
constexpr float clusterPerSide = 2;
}

