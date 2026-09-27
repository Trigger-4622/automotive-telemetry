/**
 * @file lv_conf.h
 * @brief LVGL v8.3 configuration for the 480x272 ESP32-S3 dash cluster.
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

/*====================
 * RENDER QUALITY
 *====================*/
/* Both default to 1, but they are stated here so a future trim of this file
 * cannot quietly turn them off. ANTIALIAS smooths glyph and line edges;
 * DRAW_COMPLEX is what makes arcs, rounded corners and gradients smooth
 * rather than stair-stepped, and it is the one that matters most on a panel
 * full of dials. Neither can raise the panel's 480x272 pixel count - at 4.3"
 * that is about 128 PPI, so type below ~14 px will look coarse no matter
 * what the renderer does. */
#define LV_ANTIALIAS        1
#define LV_DRAW_COMPLEX     1
/* Cache the mask of the most common circle radii instead of recomputing them
 * every frame - pure speed, and it lets DRAW_COMPLEX stay on cheaply. */
#define LV_CIRCLE_CACHE_SIZE 8
/* 1 = LVGL renders in panel wire order (big-endian RGB565), which lets
 * DisplayManager stream buffers with pushImageDMA as zero-copy DMA — the same
 * arrangement as the round board, and the reason both hit their frame budget.
 * Uploaded .bin assets must be exported "RGB565 Swap".
 *
 * If colours come out with red and blue transposed on this panel, this single
 * setting is the thing to flip (and then regenerate the assets to match). */
#define LV_COLOR_16_SWAP   1

/*====================
 * MEMORY SETTINGS
 *====================*/
/* 480x272 is 2.3x the pixels of the round board, and the cluster screen alone
 * builds far more objects than any round screen did, so the pool is larger.
 *
 * This stays in INTERNAL RAM rather than PSRAM on purpose: it holds LVGL's
 * object tree and style data, which is walked constantly during layout and
 * redraw. Latency matters more than capacity here, and the S3 has 512 KB to
 * draw on. (PSRAM's size would pay off for an image cache, read in big
 * sequential runs - none is configured.) */
#define LV_MEM_CUSTOM      0
/* 80 KB. The shipped layout - eight screens including the dash - needs about
 * 61 KB here (measured by test_host/run.py, which works out the device's
 * figure from its 32-bit struct sizes). 64 KB left too little for what comes
 * and goes at run time; running out is not an error LVGL reports, it is a
 * null pointer and a reboot. The live figure is logged at boot and shown on
 * the studio's System tab. */
#define LV_MEM_SIZE        (80U * 1024U)

/*====================
 * HAL SETTINGS
 *====================*/
#define LV_DISP_DEF_REFR_PERIOD   20    /* [ms] 50 fps target                */
#define LV_INDEV_DEF_READ_PERIOD  20    /* [ms] GT911 poll cadence           */

/* Drive lv_tick from Arduino millis() — no dedicated tick timer needed. */
#define LV_TICK_CUSTOM               1
#define LV_TICK_CUSTOM_INCLUDE       "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

/*====================
 * WIDGETS
 *====================*/
#define LV_USE_METER   1   /* Radial tachometer (needle) screens            */
#define LV_USE_ARC     1   /* Arc gauges                                    */
#define LV_USE_IMG     1   /* Backgrounds and icons                         */
#define LV_USE_SPINNER 1   /* Config-mode screen                            */
#define LV_USE_CHART   1   /* Trend/history screens                         */
#define LV_USE_BAR     1   /* Horizontal bar gauges (landscape-only screen) */

/*====================
 * FONTS
 *====================*/
/* A 480x272 panel viewed at arm's length in a car wants genuinely large
 * numerals — the cluster screen's speed readout is 48 px. The layout engine
 * maps a requested size to the nearest enabled face (UIBuilder::fontForSize). */
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_14  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_20  1
#define LV_FONT_MONTSERRAT_24  1
#define LV_FONT_MONTSERRAT_28  1
#define LV_FONT_MONTSERRAT_32  1
#define LV_FONT_MONTSERRAT_40  1
#define LV_FONT_MONTSERRAT_48  1
#define LV_FONT_DEFAULT        &lv_font_montserrat_16

/*====================
 * DEBUG (disable in production)
 *====================*/
#define LV_USE_PERF_MONITOR 0   /* Set 1 to overlay FPS/CPU% while tuning */
#define LV_USE_LOG          0

#endif /* LV_CONF_H */
