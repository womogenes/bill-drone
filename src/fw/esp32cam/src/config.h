#pragma once

namespace config {
// Measure the black square's outer edge, excluding the white margin.
int tagId = 0; // This library includes tag16h5 IDs 0..29.
float tagSizeM = 0.151350000f;

// Calibrate at 320x240 with hmirror=0 and vflip=1, in the sensor mode below. Zero means unset.
float fx = 492.659676617f, fy = 494.563782912f, cx = 160.561003196f, cy = 107.774728284f;
// OpenCV Brown-Conrady coefficients: k1, k2, p1, p2, k3.
float distortion[5] = {-0.031215286f, 0.896256747f, 0.000000000f, 0.000000000f, 0.000000000f};

// Link to the drone: UART0 (the USB-serial pins, GPIO1 TX and GPIO3 RX) at linkBaud, 8N1, and syncPin pulsed high for
// syncPulseUs as each packet is queued, so the drone can date it. GPIO14, 15 and 2 are the microSD slot in 1-bit mode;
// GPIO12 (a boot strap that must stay low) is unconnected and GPIO4 (the flash LED) is held low.
constexpr int linkTxPin = 1, linkRxPin = 3, syncPin = 13, flashLedPin = 4;
constexpr unsigned long linkBaud = 2000000;
constexpr unsigned syncPulseUs = 20;
// Black box on the microSD card (sd.h): a ring in PSRAM fed by the detector and written in staging-sized pieces from
// DMA-capable internal RAM, the tag crop's side, how often a frame also carries the whole 320x240 image (0 never), and
// the size at which a file is closed and the next one started (FAT32 files stop at 4 GiB).
constexpr uint32_t blackBoxRingBytes = 1 << 20, blackBoxStagingBytes = 8192;
constexpr int blackBoxCropSize = 96;
constexpr unsigned blackBoxFullEvery = 0;
constexpr uint64_t blackBoxFileBytes = 3500000000ull;
constexpr float minMargin = 30.0f;
constexpr float maxReprojectionPx = 2.0f;
constexpr float minAmbiguityRatio = 2.0f, ambiguityFloorPx = 0.3f;
// An ambiguous pose takes the solution nearest the last valid one if that is this recent and this close, and the other
// solution is at least anchorDecisive times further away.
constexpr uint32_t anchorUs = 500000;
constexpr float anchorMaxRad = 0.35f, anchorDecisive = 2;
// Capture-to-ready limit; a tracked pose is typically 35-45 ms old (up to 33 ms of readout, then tracking).
constexpr unsigned maxAgeUs = 150000;
// OV3660 mode: the central window of the 2048x1536 array, binned 2x2 and scaled to 320x240. A line takes hts SYSCLK
// cycles; a frame is the window's binned lines plus 15. The ESP32's 10 MHz pixel clock needs 32 us to move a 320-pixel
// line, and a faster line rate than this panicked, so 320x240 stops at 29.4 fps. For 100 fps (measured 101.5 Hz,
// detection off) the output must be 160x120: outW 160, outH 120, hts 1300, sysclkMhz 80 (PLL multiplier and PCLK
// divider 16), frame buffers in DRAM, intrinsics halved. Its 9.6 ms exposure needs several times this room's light, and
// whole-frame searches with DRAM frame buffers ran internal RAM down to 5 KB and panicked.
constexpr int windowW = 1536, windowH = 1152, outW = 320, outH = 240;
constexpr int hts = 2300, sysclkMhz = 40, vts = windowH / 2 + 15;
constexpr unsigned framePeriodUs = unsigned(vts) * hts / sysclkMhz;
// Readout time per output row.
constexpr float rowUs = float(windowH / 2) / outH * hts / sysclkMhz;
// A frame older than this when fetched is replaced by the next capture; a just-finished one is one period old.
constexpr unsigned maxFrameAgeUs = 2 * framePeriodUs;
// Exposure in lines of 57.5 us: 174 is 10 ms, a third of the frame. It grows by a quarter while the gain is above
// raiseExposureGain and shrinks back while the gain is below lowerExposureGain. Measured on 2026-10-21 with the tag
// at 1.7 m, 10 ms needed 26.6x gain and raised the position jitter from 4.4/3.2/1.8 mm (33 ms) to 8.6/5.9/3.3 mm.
constexpr int exposureLines = 174;
constexpr float raiseExposureGain = 56, lowerExposureGain = 24;
// While tracking, detector allocations below this size go to internal RAM.
constexpr size_t trackInternalBytes = 32768;
// While tracking, quads are found at the decimation (1 to 3) that keeps the tag's shortest side near this.
constexpr float decimatedSidePx = 12;
// Smallest edge cluster considered, and while tracking at least this many points per decimated tag side.
constexpr int minClusterPixels = 24;
constexpr float clusterPerSide = 2;
}

