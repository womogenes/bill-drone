#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "driver/gpio.h"
#include "config.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "soc/lcd_cam_struct.h"
#endif

// The image sensor's clocks, per mode, and the drone's K C override of them (update.h) for finding a clock whose harmonics stay off the GPS: with the camera running the drone's u-blox M10 reported jamming (CW indicator 38-46 against 6, best-four C/N0 26.5 against 40 dB-Hz, 2026-11-14) and cutting the camera's radio to 2 dBm changed nothing, so the camera's own clocks were suspected: 20 MHz XCLK's 79th harmonic and 10 MHz PCLK's 158th both land on 1580 MHz, 4.6 MHz from L1 (1575.42).
// On the ESP32-S3 the driver makes XCLK with the LCD_CAM peripheral's clock divider (its ll_cam.c xclk_timer_conf, not LEDC): 160 MHz (PLL_F160M) over an integer 160e6 / xclk_freq_hz, truncated, so the exact frequencies are 160 / N MHz (20 at 8, 17.78 at 9, 16 at 10) and anything between is the next one up. The divider also has a fraction (N + b/a, a up to 63), which K C uses only when asked: it dithers the period by a whole 6.25 ns source cycle, so its energy spreads into sidebands around every harmonic. On the ESP32 the driver uses LEDC from the 80 MHz APB with a 1-bit duty, exact at 40 / N MHz.
// The OV3660 multiplies XCLK up (its driver's set_pll, registers 0x303a-0x303d, 0x3824): PLLCLK = XCLK x multiplier / prediv (prediv index 0-3: 1, 1.5, 2, 3) / sys divider, SYSCLK = PLLCLK / 4 (the line and frame timing: config.h hts, vts, rowUs, fpvFramePeriodS) and PCLK = PLLCLK / 2 / pclk divider (the pixel bus to the ESP32). For any XCLK the multiplier and prediv are chosen to keep each mode's own PLLCLK (tag mode 160 MHz, set here; FPV's JPEG 200 MHz, the driver's for 20 MHz), so the timing constants and the 10 MHz PCLK stay as they were and only XCLK moves. Its driver picks a slower PLL for exactly 16 MHz XCLK in JPEG mode (32 MHz SYSCLK, 8 MHz PCLK, whose 197th harmonic is 1576 MHz), which this overrides back to 200 MHz.
// Also the edges, which set how much of a clock reaches the 79th or 158th harmonic: the XCLK pin's drive (gpio_drive_cap_t 0-3, 5-40 mA; default 2) and the sensor's output pads (register 0x302c bits 7:6, 1x-4x; its driver sets 4x), PCLK, HREF, VSYNC and the eight data lines.
namespace clocks {
#if CONFIG_IDF_TARGET_ESP32S3
constexpr uint32_t sourceHz = 160000000;
#else
constexpr uint32_t sourceHz = 40000000;
#endif
// Each mode's PLLCLK at 20 MHz XCLK: tag mode's set_pll(8, prediv 1, sys 1, pclk 8) and the driver's JPEG set_pll(30, prediv 3, sys 1, pclk 10).
constexpr uint32_t tagPllHz = 160000000, fpvPllHz = 200000000;
constexpr int tagPclkDivider = 8, fpvPclkDivider = 10;
constexpr int prediv2x[4] = {2, 3, 4, 6};

struct Setting {
    uint32_t xclkHz;     // asked for
    bool fractional;     // allow the divider's fraction (S3 only)
    uint8_t multiplier;  // 0: chosen to keep the mode's PLLCLK
    uint8_t prediv;      // index 0-3, with multiplier
    uint8_t pclkDivider; // 0: the mode's own
    int8_t pinDrive;     // XCLK pin, gpio_drive_cap_t; -1 leaves the default
    int8_t padDrive;     // sensor pads 0-3 (1x-4x); -1 leaves the driver's
};

// What a start will use or used: the divider (N + b/a), the frequency it makes, and the sensor's PLL.
struct Plan {
    uint32_t integerHz; // given to the driver, which makes N from it
    int divider, fractionB, fractionA;
    double xclkHz;
    int multiplier, prediv, sysDivider, pclkDivider;
    int pinDrive, padDrive;
    double pllHz() const { return xclkHz * multiplier * 2 / prediv2x[prediv] / sysDivider; }
};

// Until restart only: a value that stops the camera is gone after a reset. Index 0 tag mode, 1 FPV.
Setting settings[2] = {{config::tagXclkHz, false, 0, 0, 0, -1, -1}, {config::fpvXclkHz, false, 0, 0, 0, -1, -1}};
// The last start's plan as read back from the hardware, per mode, and whether that start has happened.
Plan applied[2];
bool appliedValid[2] = {false, false};
volatile bool restartWanted = false;

inline Plan plan(bool jpeg) {
    const Setting &s = settings[jpeg];
    Plan p = {};
    const uint32_t asked = s.xclkHz ? s.xclkHz : (jpeg ? config::fpvXclkHz : config::tagXclkHz);
    const double ratio = double(sourceHz) / asked;
    p.divider = constrain(int(ratio + 0.5), 2, 255);
#if CONFIG_IDF_TARGET_ESP32S3
    if (s.fractional && fabs(ratio - p.divider) > 1e-6) {
        p.divider = constrain(int(ratio), 2, 254);
        const double rest = ratio - p.divider;
        double best = 1;
        for (int a = 2; a <= 63; ++a) {
            const int b = int(rest * a + 0.5);
            if (b > 0 && b < a && fabs(rest - double(b) / a) < best) {
                best = fabs(rest - double(b) / a);
                p.fractionA = a;
                p.fractionB = b;
            }
        }
    }
#endif
    p.integerHz = sourceHz / p.divider;
    p.xclkHz = double(sourceHz) / (p.divider + (p.fractionA ? double(p.fractionB) / p.fractionA : 0));
    p.sysDivider = 1;
    p.pclkDivider = s.pclkDivider ? s.pclkDivider : jpeg ? fpvPclkDivider : tagPclkDivider;
    if (s.multiplier) {
        p.multiplier = s.multiplier;
        p.prediv = s.prediv & 3;
    } else {
        // The mode's own prediv first, so 20 MHz gives the settings it always had.
        const uint32_t target = jpeg ? fpvPllHz : tagPllHz;
        const int own = jpeg ? 3 : 0;
        double bestError = INFINITY;
        for (int k = 0; k < 4; ++k) {
            const int prediv = (own + k) % 4;
            const int multiplier = int(double(target) * prediv2x[prediv] / 2 / p.xclkHz + 0.5);
            if (multiplier < 1 || multiplier > 31)
                continue;
            const double error = fabs(p.xclkHz * multiplier * 2 / prediv2x[prediv] - target);
            if (error < bestError - 1) {
                bestError = error;
                p.multiplier = multiplier;
                p.prediv = prediv;
            }
        }
    }
    p.pinDrive = s.pinDrive;
    p.padDrive = s.padDrive;
    return p;
}

// After esp_camera_init with plan.integerHz: the fraction, the drives and the sensor's PLL. JPEG mode keeps the driver's own PLL when it is what the plan wants anyway (20 MHz).
inline bool apply(const Plan &p, bool jpeg, sensor_t *sensor, int xclkPin) {
#if CONFIG_IDF_TARGET_ESP32S3
    if (p.fractionA) {
        LCD_CAM.cam_ctrl.cam_clkm_div_a = p.fractionA;
        LCD_CAM.cam_ctrl.cam_clkm_div_b = p.fractionB;
        LCD_CAM.cam_ctrl.cam_update = 1;
    }
#endif
    if (p.pinDrive >= 0 && gpio_set_drive_capability(gpio_num_t(xclkPin), gpio_drive_cap_t(p.pinDrive)) != ESP_OK)
        return false;
    if (sensor->id.PID != OV3660_PID)
        return true;
    const bool driverOwn = jpeg && p.integerHz == 20000000 && !p.fractionA && p.multiplier == 30 && p.prediv == 3 && p.pclkDivider == fpvPclkDivider;
    if (!driverOwn && sensor->set_pll(sensor, 0, p.multiplier, p.sysDivider, 0, p.prediv, 0, 1, p.pclkDivider) != 0)
        return false;
    return p.padDrive < 0 || sensor->set_reg(sensor, 0x302c, 0xc0, p.padDrive << 6) == 0;
}

// The started camera's clocks as the hardware has them, kept for K C (read here, on the camera's own core, not from the request's).
inline void readBack(bool jpeg, sensor_t *sensor, int xclkPin) {
    Plan p = plan(jpeg);
#if CONFIG_IDF_TARGET_ESP32S3
    p.divider = LCD_CAM.cam_ctrl.cam_clkm_div_num;
    p.fractionA = LCD_CAM.cam_ctrl.cam_clkm_div_a;
    p.fractionB = LCD_CAM.cam_ctrl.cam_clkm_div_b;
    if (!p.fractionA || !p.fractionB)
        p.fractionA = p.fractionB = 0;
    p.xclkHz = double(sourceHz) / ((p.divider ? p.divider : 256) + (p.fractionA ? double(p.fractionB) / p.fractionA : 0));
#endif
    gpio_drive_cap_t drive = GPIO_DRIVE_CAP_DEFAULT;
    gpio_get_drive_capability(gpio_num_t(xclkPin), &drive);
    p.pinDrive = int(drive);
    if (sensor->id.PID == OV3660_PID) {
        p.multiplier = sensor->get_reg(sensor, 0x303b, 0x1f);
        p.sysDivider = max(1, sensor->get_reg(sensor, 0x303c, 0x0f));
        p.prediv = (sensor->get_reg(sensor, 0x303d, 0xff) >> 4) & 3;
        p.pclkDivider = max(1, sensor->get_reg(sensor, 0x3824, 0x1f));
        p.padDrive = sensor->get_reg(sensor, 0x302c, 0xff) >> 6;
    }
    applied[jpeg] = p;
    appliedValid[jpeg] = true;
}

// K C's answer (52 bytes at most): mode, XCLK (MHz), its divider (+b/a when fractional, which dithers), PLLCLK, SYSCLK and PCLK (MHz), XCLK pin and sensor pad drives, and whether this is the hardware's (hw, from the mode's last start) or the next start's plan.
inline void describe(bool jpeg, char *text, size_t size) {
    const bool hw = appliedValid[jpeg];
    const Plan p = hw ? applied[jpeg] : plan(jpeg);
    char divider[16];
    if (p.fractionA)
        snprintf(divider, sizeof(divider), "%d+%d/%d", p.divider, p.fractionB, p.fractionA);
    else
        snprintf(divider, sizeof(divider), "%d", p.divider);
    const double pll = p.pllHz() / 1e6;
    snprintf(text, size, "%c %.6f /%s v%.1f s%.1f p%.2f d%d/%d %s", jpeg ? 'f' : 't', p.xclkHz / 1e6, divider, pll, pll / 4,
             pll / 2 / p.pclkDivider, p.pinDrive, p.padDrive, hw ? "hw" : "plan");
}
} // namespace clocks
