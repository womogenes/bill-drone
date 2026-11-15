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

// Link to the drone: UART0 at linkBaud, 8N1; the drone dates each packet by its first start bit (main.cpp publish). syncPin, which pulsed as each packet was queued until 2026-11-14, is never driven on the S3 board: the drone's end of its line is shorted to the drone's TX. The flash LED is held low.
#if CONFIG_IDF_TARGET_ESP32S3
// NULLLAB ESP32S3-CAM (github.com/nulllaborg/esp32s3-cam, from 2026-11-02): UART0's own pins GPIO43 TX and 44 RX (header J5), sync on GPIO14 (J4), the flash LEDs on GPIO3, the microSD in 1-bit mode on GPIO39 CLK, 38 CMD and 40 D0 (DAT3 unconnected). The USB-C port is the chip's own USB, so UART0 has no USB-serial chip on it.
constexpr int linkTxPin = 43, linkRxPin = 44, syncPin = 14, flashLedPin = 3;
constexpr int sdClkPin = 39, sdCmdPin = 38, sdD0Pin = 40;
constexpr bool sdDat3IsSync = false;
#else
// AI Thinker ESP32-CAM: the USB-serial pins GPIO1 TX and GPIO3 RX, GPIO4 the flash LED. GPIO14, 15 and 2 are the microSD slot in 1-bit mode (fixed pins) and its DAT3 is the sync line; GPIO12 (a boot strap that must stay low) is unconnected.
constexpr int linkTxPin = 1, linkRxPin = 3, syncPin = 13, flashLedPin = 4;
constexpr bool sdDat3IsSync = true;
#endif
constexpr unsigned long linkBaud = 2000000;
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
// Binned rows read per row kept: 2 reads every other binned row (vertical increment 0x71 in place of binning's 0x31), 384 of the 768 for the 240 output rows, which halves the frame at the same line time and so the same DMA pace; the field of view and calibration stay.
constexpr int rowSkip = 2;
// The weakest edges on the camera's clocks and pixel bus, both modes: the XCLK pin's drive (gpio_drive_cap_t, 0 = 5 mA; ESP-IDF's default 2) and the OV3660's output pads (0x302c bits 7:6, 0 = 1x; its driver's 3 = 4x). Measured 2026-11-14 indoors through the GPS's MON-SPAN (camera_clock_sweep): with the defaults the camera put tones 14-17 dB above the floor at 1580 MHz and 8-13 dB at 1570/1590 in tag mode (CW jamming indicator 24-32), 14-15 dB at 1580 in FPV (29-35); both at their weakest left 1580 at +4 dB and 1570/1590 at +1 (indicator 6-14, about the camera-off 6), with FPV's JPEGs all decoding at the same 26-27 fps and tag mode's frame rate unchanged. Either alone was not enough in tag mode: the pads' drive took out 1570/1590 (PCLK's 157th and 159th) but 1580 also carries XCLK's 79th.
constexpr int xclkPinDrive = 0, sensorPadDrive = 0;
// Tag mode's XCLK; the sensor's PLL (clocks.h) keeps SYSCLK at sysclkMhz and PCLK at 10 MHz for any XCLK.
constexpr uint32_t tagXclkHz = 20000000;
constexpr int hts = 2300, sysclkMhz = 40, vts = windowH / 2 / rowSkip + 15;
constexpr unsigned framePeriodUs = unsigned(vts) * hts / sysclkMhz;
// Readout time per output row.
constexpr float rowUs = float(windowH / 2 / rowSkip) / outH * hts / sysclkMhz;
// A frame older than this when fetched is replaced by the next capture; a just-finished one is one period old.
constexpr unsigned maxFrameAgeUs = 2 * framePeriodUs;
// Exposure in lines of 57.5 us: 174 is 10 ms, a third of the frame. It grows by a quarter while the gain is above raiseExposureGain and shrinks back while the gain is below lowerExposureGain. Measured on 2026-10-21 with the tag at 1.7 m, 10 ms needed 26.6x gain and raised the position jitter from 4.4/3.2/1.8 mm (33 ms) to 8.6/5.9/3.3 mm.
constexpr int exposureLines = 174;
constexpr float raiseExposureGain = 56, lowerExposureGain = 24;
// FPV (fpv.h): the sensor's own JPEG at automatic exposure, no tag tracking. Measured on 2026-10-30 (OV3660, 20 MHz XCLK): at 20 MHz XCLK 480x320 comes off the sensor every 31.2 ms, VGA and QVGA every 36 ms. The JPEG quality (fpvQualityRange, lower is finer) holds each frame near fpvTargetBytes, four datagrams, whatever the light: that bounds the video's airtime on a channel the drone's radio may share (it took the drone's status stream from 20.0 to 19.3 messages/s on its channel), and a frame loses one datagram less often. Over a 2-minute soak the dashboard received 32.0 fps with no frame lost, gaps p99 55 ms, max 211 ms. It switches back to tracking fpvLingerMs after the last keepalive, so a Wi-Fi dropout resumes without restarting the camera twice.
constexpr framesize_t fpvFrameSize = FRAMESIZE_HVGA;
// 20 MHz XCLK, 480x320 every 31.2 ms. 24 MHz gave a frame every 26.0 ms (38.4 fps over UDP, 2026-10-30) but corrupted most JPEGs, with bands shifted and brightness jumping part way down the frame, unnoticed until the pictures were looked at; at 20 MHz none were in 60 consecutive frames.
constexpr uint32_t fpvXclkHz = 20000000;
// 12 KB from 2026-10-31, for footage clean enough for vision work; the 5 KB before suited Wi-Fi on the drone's channel and the ESP32 receiver's USB link, neither in use now. Over the radio the receiver's budget caps it too.
constexpr size_t fpvTargetBytes = 12000;
// Over the radio the video must fit the receiver, whose budget comes with the drone's K V request: the ESP32 receiver's (fw/fpv_rx) USB bridge, about 98 KB/s at 1 Mbaud, or the monitor-mode adapter's air (sw/fpv_monitor.py); fpvRadioBytesPerSecond until one says. The sensor's frame is lengthened until the average frame fits, so every frame goes out evenly spaced (the ESP32 receiver drops whole frames if a burst outruns it), and their size aims at the budget's share. fpvFramePeriodS is the sensor's own 480x320 JPEG frame at fpvXclkHz. Each goes out as erasure-coded shards (fpv.h sendShards): with one copy a fifth of the frames lost a chunk at the ESP32 receiver (2026-10-30).
constexpr float fpvRadioBytesPerSecond = 90000, fpvFramePeriodS = 0.0312f;
// The monitor-mode adapter took in about 210 shards a second however many were sent (2026-10-31); paced a little under it, frames arrive whole and evenly.
constexpr float fpvRadioShardsPerSecond = 190;
// The largest frame over the radio when the receiver's budget allows it (the S3 receiver, fw/fpv_rx make s3: its own USB has no 1 Mbaud limit), for finer JPEGs than fpvTargetBytes: under the driver's JPEG buffer for 480x320 (width x height / 5, about 30 KB), which a larger frame would overrun.
constexpr size_t fpvRadioMaxBytes = 24000;
// With a receiver whose budget is at least fpvLargeBudget (the S3 receiver's) FPV starts at 640x480 instead: twice the pixels, a frame every 36 ms (VGA's at 20 MHz XCLK, measured 2026-10-30) instead of 31.2, and frames up to fpvLargeMaxBytes under VGA's JPEG buffer (about 61 KB). The budget at FPV's start decides; a receiver changed mid-stream keeps the size until the next start.
constexpr float fpvLargeBudget = 300000;
constexpr framesize_t fpvLargeFrameSize = FRAMESIZE_VGA;
constexpr float fpvLargeFramePeriodS = 0.036f;
constexpr size_t fpvLargeMaxBytes = 40000;
constexpr int fpvQualityRange[2] = {10, 63};
constexpr uint32_t fpvLingerMs = 3000;
// While tracking, detector allocations below this size go to internal RAM.
constexpr size_t trackInternalBytes = 32768;
// While tracking, quads are found at the decimation (1 to 3) that keeps the tag's shortest side near this.
constexpr float decimatedSidePx = 12;
// Smallest edge cluster considered, and while tracking at least this many points per decimated tag side.
constexpr int minClusterPixels = 24;
constexpr float clusterPerSide = 2;
}

