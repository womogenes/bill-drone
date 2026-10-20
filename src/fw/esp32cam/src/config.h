#pragma once

namespace config {
// Measure the black square's outer edge, excluding the white margin.
int tagId = 0; // This library includes tag16h5 IDs 0..29.
float tagSizeM = 0.151350000f;

// Calibrate at 320x240 with hmirror=0 and vflip=1. Zero means unset.
float fx = 369.494757463f, fy = 370.922837184f, cx = 160.420727397f, cy = 110.831046213f;
// OpenCV Brown-Conrady coefficients: k1, k2, p1, p2, k3.
float distortion[5] = {-0.031215286f, 0.896256747f, 0.000000000f, 0.000000000f, 0.000000000f};

// Native HSPI pins on the camera; microSD must remain unused.
constexpr int clockPin = 14, mosiPin = 13, misoPin = 12, csPin = 15;
constexpr int readyPin = 2;
constexpr float minMargin = 30.0f;
constexpr float maxReprojectionPx = 2.0f;
constexpr float minAmbiguityRatio = 2.0f;
constexpr unsigned maxAgeUs = 250000;
}

