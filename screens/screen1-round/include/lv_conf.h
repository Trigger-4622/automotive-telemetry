/**
 * @file lv_conf.h
 * @brief LVGL v8.3 configuration for the ESP32-C3 round telemetry display.
 *
 * Only the options that DIFFER from LVGL defaults are defined here.
 * Every option left undefined falls back to the default value in
 * lvgl/src/lv_conf_internal.h (each option there is #ifndef-guarded),
 * which keeps this file small and upgrade-friendly.
 */
#ifndef LV_CONF_H
#define LV_CONF_H

/*====================
 * COLOR SETTINGS
 *====================*/
#define LV_COLOR_DEPTH     16
/* 1 = LVGL renders in panel wire byte order (big-endian RGB565). This lets
 * DisplayManager stream buffers with pushImageDMA() as pure zero-copy DMA —
 * no per-pixel CPU conversion — which is the backbone of smooth swipe
 * transitions. Uploaded .bin assets must be exported as "RGB565 Swap". */
#define LV_COLOR_16_SWAP   1

/*====================
 * MEMORY SETTINGS
 *====================*/
/* Internal TLSF pool for all LVGL objects/styles/anims.
 * The dynamic gauge engine builds every screen at boot. The shipped layout
 * needs about 32 KB of these 56 (test_host/run.py works the device's figure
 * out from its 32-bit struct sizes); running out is a null pointer and a
 * reboot, not an error message. */
#define LV_MEM_CUSTOM      0
#define LV_MEM_SIZE        (56U * 1024U)

/*====================
 * HAL SETTINGS
 *====================*/
#define LV_DISP_DEF_REFR_PERIOD   20    /* [ms] 50 fps target — the DMA+80 MHz
                                           pipeline keeps up; anims look fluid */
#define LV_INDEV_DEF_READ_PERIOD  20    /* [ms] CST816S poll cadence */

/* Drive lv_tick from Arduino millis() — no dedicated tick timer needed. */
#define LV_TICK_CUSTOM               1
#define LV_TICK_CUSTOM_INCLUDE       "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

/*====================
 * WIDGETS
 *====================*/
#define LV_USE_METER   1   /* Radial tachometer (needle) screens            */
#define LV_USE_ARC     1   /* Primary arc gauges                            */
#define LV_USE_IMG     1   /* Custom background assets from LittleFS       */
#define LV_USE_SPINNER 1   /* Config-mode screen                            */
#define LV_USE_CHART   1   /* Trend/history screens                         */

/*====================
 * FONTS
 *====================*/
/* The dynamic layout engine maps requested sizes to the nearest enabled
 * font (see UIBuilder::fontForSize). */
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_14  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_20  1
#define LV_FONT_MONTSERRAT_24  1
#define LV_FONT_MONTSERRAT_28  1
#define LV_FONT_MONTSERRAT_32  1
#define LV_FONT_MONTSERRAT_40  1
#define LV_FONT_MONTSERRAT_48  1
#define LV_FONT_DEFAULT        &lv_font_montserrat_14

/*====================
 * DEBUG (disable in production)
 *====================*/
#define LV_USE_PERF_MONITOR 0   /* Set 1 to overlay FPS/CPU% while tuning */
#define LV_USE_LOG          0

#endif /* LV_CONF_H */
