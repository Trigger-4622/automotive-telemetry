/**
 * @file DisplayManager.h
 * @ingroup hal
 * @brief NV3041A panel bring-up (Arduino_GFX), LVGL initialization, backlight
 *        control and the LVGL↔LittleFS filesystem bridge ("L:" drive).
 *
 * ## Why Arduino_GFX here and LovyanGFX on the round board
 *
 * The round board's GC9A01 is an ordinary single-line SPI panel and LovyanGFX
 * drives it perfectly. This panel is Quad-SPI, and that is a different story.
 *
 * LovyanGFX's QSPI support is recent: it only reached the develop branch in
 * May 2025, having previously existed as "a proof of concept on a heavily
 * modified fork". Driving this panel through it produced a persistent scatter
 * of wrong pixels through freshly drawn areas that survived every fix aimed
 * at it — DMA buffer alignment, even-width flush windows, one transaction per
 * flush, bus re-init per transaction, SPI mode, and clock rates from 80 MHz
 * down to 26.7 MHz. The clock work only ever *reduced* the artifacts.
 *
 * The decisive evidence was that the vendor's own demo firmware for this exact
 * board is clean, and it uses Arduino_GFX. The two libraries frame a QSPI
 * transfer differently: Arduino_GFX declares `command_bits = 8` and
 * `address_bits = 24` in the device config and lets the SPI hardware emit the
 * command and address phases as part of one atomic transaction, while
 * LovyanGFX hand-sends four command bytes on a single line and then switches
 * the peripheral into quad mode for the payload. That software-managed
 * single→quad transition, repeated for every partial flush LVGL performs, is
 * where the corruption came from.
 *
 * So this file uses the library that the hardware is known to work with.
 * Nothing above the HAL changed: @ref DisplayManager keeps the same three
 * entry points, so main.cpp and UIBuilder are untouched.
 */
#pragma once

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>

#include "HardwareConfig.h"

/**
 * @ingroup hal
 * @brief Owns the panel and the LVGL display driver.
 *
 * Singleton — see @ref Display. Call @ref begin once from setup(), then
 * @ref update every loop iteration.
 */
class DisplayManager {
public:
    /**
     * @brief Bring up the panel and LVGL.
     *
     * Initializes the QSPI bus and NV3041A, allocates the two partial draw
     * buffers in DMA-capable internal RAM, registers the flush and rounder
     * callbacks, starts the backlight PWM and mounts the LVGL "L:" drive.
     */
    void begin();

    /**
     * @brief Pump LVGL — call every loop() iteration.
     * @return Milliseconds until LVGL next needs servicing, so the caller can
     *         size its sleep instead of guessing.
     */
    uint32_t update() { return lv_timer_handler(); }

    /**
     * @brief Set the backlight level.
     * @param level 0 (off) to 255 (full). Driven by day/night switching —
     *              see @ref BRIGHTNESS_DAY and @ref BRIGHTNESS_NIGHT.
     */
    void setBrightness(uint8_t level);

    /**
     * @brief Set per-channel colour gain, applied live to every flush.
     *
     * Exists because a panel's colour cast cannot be argued about usefully -
     * it has to be dialled out while looking at it. The web portal drives this
     * straight from sliders so the screen responds as they move, with no
     * reboot and nothing written to flash until you choose to save.
     *
     * Implemented as three small lookup tables (32/64/32 entries for the
     * 5/6/5 bits of RGB565) rather than per-pixel arithmetic, so the cost is
     * one table read per channel. Gains of exactly 1.0 disable the pass
     * entirely, and then it costs nothing at all.
     *
     * @param r Red gain,   0.0-2.0. 1.0 leaves the channel untouched.
     * @param g Green gain, 0.0-2.0.
     * @param b Blue gain,  0.0-2.0.
     */
    void setCalibration(float r, float g, float b);

    /** @return true when a non-unity gain is currently being applied. */
    bool calibrationActive() const { return _calibActive; }

    /**
     * @brief The registered LVGL display object.
     * @return Handle for LVGL calls that need an explicit display, or
     *         nullptr before @ref begin has run.
     */
    lv_disp_t *disp() { return _disp; }

private:
    /**
     * @brief LVGL flush callback — pushes one rendered area to the panel.
     *
     * `draw16bitBeRGBBitmap` takes big-endian RGB565, which is exactly what
     * LVGL produces with `LV_COLOR_16_SWAP = 1`, so the buffer goes to the
     * panel with no per-pixel conversion.
     *
     * @param drv    LVGL driver whose `user_data` is the DisplayManager.
     * @param area   Rectangle being flushed, in display coordinates.
     * @param pixels Rendered pixel buffer for @p area.
     */
    static void flushCb(lv_disp_drv_t *drv, const lv_area_t *area,
                        lv_color_t *pixels);

    /**
     * @brief Snap every flush to an even column span.
     *
     * LVGL invalidates the exact bounding box of whatever changed, so a label
     * or an arc routinely lands on odd x1/x2. This panel takes pixel data as a
     * 16-bit stream, and an odd-width window leaves its column counter half a
     * word out of step with the data. Widening to even boundaries costs at
     * most two columns per flush.
     *
     * @param[in,out] area Rectangle LVGL is about to render.
     */
    static void rounderCb(lv_disp_drv_t *drv, lv_area_t *area);

    /**
     * @brief Register the LVGL "L:" drive backed by LittleFS.
     *
     * Lets layout.json reference uploaded images as `L:/assets/bg.bin`.
     */
    void registerLvglFilesystem();

    Arduino_DataBus   *_bus  = nullptr;    /**< Quad-SPI bus.               */
    Arduino_GFX       *_gfx  = nullptr;    /**< NV3041A panel.              */
    lv_disp_draw_buf_t _drawBuf;           /**< Double partial-buffer desc. */
    lv_disp_drv_t      _dispDrv;           /**< LVGL display driver.        */
    lv_disp_t         *_disp = nullptr;    /**< Registered LVGL display.    */
    bool               _calibActive = false; /**< Any gain differs from 1.0.*/
};

/** @brief Global display singleton — the board has exactly one panel. */
extern DisplayManager Display;
