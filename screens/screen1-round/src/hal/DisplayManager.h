/**
 * @file DisplayManager.h
 * @ingroup hal
 * @brief GC9A01 panel bring-up (LovyanGFX), LVGL initialization, backlight
 *        control and the LVGL↔LittleFS filesystem bridge ("L:" drive) used
 *        for custom background assets.
 *
 * The panel/bus are configured entirely in C++ so that every GPIO comes from
 * @ref HardwareConfig.h — no pin ever hides in a build flag.
 */

/**
 * @defgroup hal Hardware Abstraction
 * @brief Drivers for the all-in-one board's panel, touch controller and
 *        side button.
 *
 * Everything in this group is the only code allowed to touch a GPIO, and
 * every pin it uses is declared in @ref HardwareConfig.h. Porting to a
 * differently-routed clone board should mean editing that one header.
 * @{
 */
#pragma once

#include <Arduino.h>
#include <lvgl.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "HardwareConfig.h"

/**
 * @brief LovyanGFX device describing the all-in-one board's pre-routed
 *        GC9A01 SPI panel and PWM backlight.
 */
class LGFX_RoundC3 : public lgfx::LGFX_Device {
    lgfx::Panel_GC9A01 _panel;   /**< GC9A01 240×240 round IPS controller. */
    lgfx::Bus_SPI      _bus;     /**< Write-only 4-wire SPI bus.           */
    lgfx::Light_PWM    _light;   /**< PWM-dimmable backlight.              */

public:
    /**
     * @brief Describe the board's fixed wiring to LovyanGFX.
     *
     * The board is pre-routed, so none of this is user-configurable at
     * runtime; it exists in code purely so the pin numbers can live in
     * @ref HardwareConfig.h instead of build flags.
     */
    LGFX_RoundC3() {
        {   // SPI bus — write-only (the board does not route MISO)
            auto cfg        = _bus.config();
            cfg.spi_host    = SPI2_HOST;
            cfg.spi_mode    = 0;
            cfg.freq_write  = LCD_SPI_HZ;
            cfg.spi_3wire   = true;
            cfg.use_lock    = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk    = PIN_LCD_SCLK;
            cfg.pin_mosi    = PIN_LCD_MOSI;
            cfg.pin_miso    = -1;
            cfg.pin_dc      = PIN_LCD_DC;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {   // GC9A01 panel
            auto cfg          = _panel.config();
            cfg.pin_cs        = PIN_LCD_CS;
            cfg.pin_rst       = PIN_LCD_RST;
            cfg.pin_busy      = -1;
            cfg.panel_width   = LCD_WIDTH;
            cfg.panel_height  = LCD_HEIGHT;
            cfg.offset_x      = 0;
            cfg.offset_y      = 0;
            cfg.readable      = false;
            cfg.invert        = true;    // GC9A01 IPS: colors need inversion
            cfg.rgb_order     = false;
            cfg.dlen_16bit    = false;
            cfg.bus_shared    = false;   // display owns the SPI bus exclusively
            _panel.config(cfg);
        }
        {   // PWM-dimmable backlight
            auto cfg        = _light.config();
            cfg.pin_bl      = PIN_LCD_BL;
            cfg.invert      = false;
            cfg.freq        = LCD_BL_PWM_FREQ;
            cfg.pwm_channel = 1;
            _light.config(cfg);
            _panel.setLight(&_light);
        }
        setPanel(&_panel);
    }
};

/**
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
     * Initializes the GC9A01, allocates the two partial draw buffers,
     * registers the flush callback and mounts the LVGL "L:" filesystem
     * drive that maps onto LittleFS.
     *
     * @pre LittleFS must already be mounted (ConfigManager::begin) if any
     *      layout references a file-backed image.
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
     * @brief The registered LVGL display object.
     * @return Handle for LVGL calls that need an explicit display, or
     *         nullptr before @ref begin has run.
     */
    lv_disp_t *disp() { return _disp; }

private:
    /**
     * @brief LVGL flush callback — streams one rendered area to the panel.
     *
     * Performs a zero-copy DMA push: with `LV_COLOR_16_SWAP = 1` the buffer is
     * already in panel wire order, so no per-pixel conversion happens.
     * `lv_disp_flush_ready` is signalled immediately, letting LVGL render the
     * next chunk into its second buffer while this one is still in flight;
     * LovyanGFX blocks internally if a new transfer starts before the previous
     * DMA completes, which is what keeps that safe.
     *
     * @param drv    LVGL driver whose `user_data` is the DisplayManager.
     * @param area   Rectangle being flushed, in display coordinates.
     * @param pixels Rendered pixel buffer for @p area.
     */
    static void flushCb(lv_disp_drv_t *drv, const lv_area_t *area,
                        lv_color_t *pixels);

    /**
     * @brief Register the LVGL "L:" drive backed by LittleFS.
     *
     * Lets layout.json reference uploaded images as `L:/assets/bg.bin` and
     * have lv_img stream them from flash.
     */
    void registerLvglFilesystem();

    LGFX_RoundC3       _gfx;               /**< Panel device.               */
    lv_disp_draw_buf_t _drawBuf;           /**< Double partial-buffer desc. */
    lv_disp_drv_t      _dispDrv;           /**< LVGL display driver.        */
    lv_disp_t         *_disp = nullptr;    /**< Registered LVGL display.    */
};

/** @brief Global display singleton — the board has exactly one panel. */
extern DisplayManager Display;

/** @} */  // end of hal group
