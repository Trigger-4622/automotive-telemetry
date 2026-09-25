/**
 * @file DisplayManager.cpp
 * @ingroup hal
 * @brief Panel + LVGL bring-up and the LVGL "L:" LittleFS driver.
 */
#include "DisplayManager.h"

#include <LittleFS.h>
#include <esp_heap_caps.h>

#include "ConfigManager.h"

DisplayManager Display;

/** LEDC channel driving the backlight. Channel 0 is left free for anything
 *  the Arduino core wants to grab first. */
static constexpr uint8_t BL_LEDC_CHANNEL = 1;
/** 8-bit duty so setBrightness maps 1:1 onto 0-255. */
static constexpr uint8_t BL_LEDC_BITS    = 8;

/* Two 34-line partial buffers (480 × 34 × 2 B = 31.9 KB each). 34 divides 272
 * exactly, so a full-screen repaint lands in 8 clean flushes.
 *
 * These live in INTERNAL SRAM deliberately, not PSRAM: every pixel LVGL
 * renders would otherwise cross the octal bus twice (written during render,
 * read during DMA), and rendering is the bottleneck.
 *
 * 68-line buffers were tried first — fewer flushes, in principle faster — but
 * at 128 KB the pair pushed static RAM to 85%, leaving too little heap for the
 * Wi-Fi stack to bring ESP-NOW up reliably. */
static constexpr uint32_t BUF_LINES = 34;

/*
 * Allocated from the DMA-capable internal heap rather than declared as plain
 * arrays. A `static lv_color_t buf[]` is only guaranteed 2-byte aligned,
 * because that is the natural alignment of its element type, and the SPI DMA
 * engine wants its source word-aligned. heap_caps_malloc with MALLOC_CAP_DMA
 * guarantees both DMA-reachable memory and the alignment the engine expects.
 */
static lv_color_t *s_buf1 = nullptr;
static lv_color_t *s_buf2 = nullptr;


/* ───────────────────────── colour calibration ─────────────────────────────
 * Three lookup tables, one per RGB565 channel (5, 6 and 5 bits). Building
 * them once per slider move keeps the per-pixel cost to three table reads and
 * a repack, which is what makes live adjustment affordable at 50 fps.
 *
 * File scope rather than members so the flush path reaches them without going
 * through the instance pointer - this runs on every pixel of every flush. */
static uint8_t s_lutR[32], s_lutG[64], s_lutB[32];

/**
 * @brief Apply the calibration tables to one big-endian RGB565 pixel.
 *
 * The buffer is big-endian because LV_COLOR_16_SWAP is 1, so each pixel is
 * byte-swapped on the way in and back on the way out.
 *
 * @param be Pixel as stored in the LVGL draw buffer.
 * @return The corrected pixel, in the same byte order.
 */
static inline uint16_t calibPixel(uint16_t be) {
    const uint16_t v = (uint16_t)((be >> 8) | (be << 8));
    const uint16_t out = (uint16_t)((s_lutR[(v >> 11) & 0x1F] << 11) |
                                    (s_lutG[(v >>  5) & 0x3F] <<  5) |
                                    (s_lutB[ v        & 0x1F]));
    return (uint16_t)((out >> 8) | (out << 8));
}

void DisplayManager::setCalibration(float r, float g, float b) {
    auto build = [](uint8_t *lut, int steps, float gain) {
        const int maxv = steps - 1;
        for (int i = 0; i < steps; i++) {
            int v = (int)lroundf(i * gain);
            lut[i] = (uint8_t)(v < 0 ? 0 : (v > maxv ? maxv : v));
        }
    };
    build(s_lutR, 32, r);
    build(s_lutG, 64, g);
    build(s_lutB, 32, b);
    // Exactly 1.0 on every channel means the pass can be skipped outright.
    _calibActive = (fabsf(r - 1.0f) > 0.001f) || (fabsf(g - 1.0f) > 0.001f) ||
                   (fabsf(b - 1.0f) > 0.001f);
    log_i("colour gain R=%.2f G=%.2f B=%.2f (%s)", r, g, b,
          _calibActive ? "active" : "bypassed");
}

void DisplayManager::rounderCb(lv_disp_drv_t * /*drv*/, lv_area_t *area) {
    area->x1 &= ~1;              // round the left edge down to even
    area->x2 |= 1;               // round the right edge up to odd => even width
    if (area->x2 >= LCD_WIDTH) area->x2 = LCD_WIDTH - 1;
}

void DisplayManager::flushCb(lv_disp_drv_t *drv, const lv_area_t *area,
                             lv_color_t *pixels) {
    auto *self = static_cast<DisplayManager *>(drv->user_data);
    const int16_t w = area->x2 - area->x1 + 1;
    const int16_t h = area->y2 - area->y1 + 1;

    // Correct the buffer in place. LVGL re-renders it before the next use, so
    // there is nothing to restore afterwards.
    if (self->_calibActive) {
        uint16_t *p = reinterpret_cast<uint16_t *>(pixels);
        for (int32_t i = (int32_t)w * h; i > 0; i--, p++) *p = calibPixel(*p);
    }

    self->_gfx->draw16bitBeRGBBitmap(area->x1, area->y1,
                                     reinterpret_cast<uint16_t *>(pixels),
                                     w, h);
    lv_disp_flush_ready(drv);
}

void DisplayManager::begin() {
    /*
     * Pin order is the SPI peripheral's, not the silkscreen's: in quad mode
     * IO0 is MOSI, IO1 is MISO, IO2 is WP and IO3 is HD. So the board's
     * D0..D3 map onto mosi/miso/quadwp/quadhd in that order.
     */
    _bus = new Arduino_ESP32QSPI(PIN_LCD_CS,   PIN_LCD_SCLK,
                                 PIN_LCD_D0,   PIN_LCD_D1,
                                 PIN_LCD_D2,   PIN_LCD_D3);

    /*
     * ips = true is what applies inversion on this panel; there is no separate
     * invert call to make afterwards. Rotation 0 gives the native 480x272
     * landscape, which is the only orientation this panel supports properly.
     */
    _gfx = new Arduino_NV3041A(_bus, GFX_NOT_DEFINED /* RST not routed */,
                               0 /* rotation */, LCD_INVERT /* ips */,
                               LCD_WIDTH, LCD_HEIGHT);

    if (!_gfx->begin(LCD_SPI_HZ)) {
        log_e("NV3041A begin() failed — display dead");
    }
    /*
     * Arduino_GFX always writes MADCTL with the RGB bit set, which is right
     * for the reference board. Some panels of this family are wired BGR, and
     * the symptom is subtle enough to argue about - so it is a runtime switch
     * rather than a rebuild, and the DIAGNOSTICS screen carries a colour strip
     * that answers the question outright. MX|MY keeps the landscape
     * orientation the driver just set; only bit 3 changes.
     */
    if (Config.bgrOrder()) {
        _bus->beginWrite();
        _bus->writeC8D8(0x36, 0x40 | 0x80 | 0x08);   // MX | MY | BGR
        _bus->endWrite();
        log_i("panel forced to BGR order");
    }

    _gfx->fillScreen(BLACK);

    // Backlight. Arduino_GFX has no light driver, so LEDC is driven directly.
    ledcSetup(BL_LEDC_CHANNEL, LCD_BL_PWM_FREQ, BL_LEDC_BITS);
    ledcAttachPin(PIN_LCD_BL, BL_LEDC_CHANNEL);
    setBrightness(BRIGHTNESS_DAY);
    setCalibration(Config.colorGainR(), Config.colorGainG(), Config.colorGainB());

    lv_init();

    const size_t bufPixels = LCD_WIDTH * BUF_LINES;
    const size_t bufBytes  = bufPixels * sizeof(lv_color_t);
    s_buf1 = (lv_color_t *)heap_caps_malloc(bufBytes,
                                            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_buf2 = (lv_color_t *)heap_caps_malloc(bufBytes,
                                            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_buf1 || !s_buf2) {
        // Falling back to a single buffer is far better than running with a
        // null one; LVGL simply loses the render/flush overlap.
        log_e("draw buffer alloc failed (%u B each) — degrading", (unsigned)bufBytes);
        if (!s_buf1) s_buf1 = s_buf2;
        s_buf2 = nullptr;
    }
    lv_disp_draw_buf_init(&_drawBuf, s_buf1, s_buf2, bufPixels);

    lv_disp_drv_init(&_dispDrv);
    _dispDrv.hor_res   = LCD_WIDTH;
    _dispDrv.ver_res   = LCD_HEIGHT;
    _dispDrv.flush_cb  = flushCb;
    _dispDrv.rounder_cb = rounderCb;
    _dispDrv.draw_buf  = &_drawBuf;
    _dispDrv.user_data = this;
    _disp = lv_disp_drv_register(&_dispDrv);

    registerLvglFilesystem();
    log_i("Display up: %dx%d @ %d MHz QSPI (Arduino_GFX)", LCD_WIDTH, LCD_HEIGHT,
          LCD_SPI_HZ / 1000000);
}

void DisplayManager::setBrightness(uint8_t level) {
    ledcWrite(BL_LEDC_CHANNEL, level);
}

/* ─────────────────── LVGL "L:" drive backed by LittleFS ────────────────────
 * Lets layout.json reference uploaded images as  "L:/assets/bg.bin"  and have
 * lv_img stream them straight from flash.
 *
 * NOTE ON COST: this is a streaming bridge, not a cache. LVGL's built-in
 * decoder re-reads a file-backed image line by line on EVERY repaint, so a
 * full-screen background costs a great deal of flash reads per repaint.
 * Prefer a gradient or a compiled-in tile (see Textures.h) on any screen that
 * animates. */

/**
 * @brief LVGL fs_drv open callback. Read-only.
 * @param path Path with the "L:" prefix already stripped by LVGL.
 * @param mode Requested access; anything but LV_FS_MODE_RD is refused.
 * @return Heap-allocated File handle, or nullptr on failure.
 */
static void *fsOpen(lv_fs_drv_t *, const char *path, lv_fs_mode_t mode) {
    if (mode != LV_FS_MODE_RD) return nullptr;
    String full = String("/") + path;          // LVGL strips "L:" and the '/'
    full.replace("//", "/");
    File f = LittleFS.open(full, "r");
    if (!f) return nullptr;
    return new File(f);
}

/**
 * @brief LVGL fs_drv close callback — closes and frees the File handle.
 * @param fp Handle returned by fsOpen.
 * @return Always LV_FS_RES_OK.
 */
static lv_fs_res_t fsClose(lv_fs_drv_t *, void *fp) {
    File *f = static_cast<File *>(fp);
    f->close();
    delete f;
    return LV_FS_RES_OK;
}

/**
 * @brief LVGL fs_drv read callback.
 * @param fp  Handle returned by fsOpen.
 * @param buf Destination buffer.
 * @param btr Bytes to read.
 * @param[out] br Bytes actually read (short reads signal end of file).
 * @return Always LV_FS_RES_OK.
 */
static lv_fs_res_t fsRead(lv_fs_drv_t *, void *fp, void *buf, uint32_t btr,
                          uint32_t *br) {
    *br = static_cast<File *>(fp)->read(static_cast<uint8_t *>(buf), btr);
    return LV_FS_RES_OK;
}

/**
 * @brief LVGL fs_drv seek callback.
 * @param fp     Handle returned by fsOpen.
 * @param pos    Target offset, interpreted per @p whence.
 * @param whence LV_FS_SEEK_SET / _CUR / _END.
 * @return Always LV_FS_RES_OK.
 */
static lv_fs_res_t fsSeek(lv_fs_drv_t *, void *fp, uint32_t pos,
                          lv_fs_whence_t whence) {
    File *f = static_cast<File *>(fp);
    switch (whence) {
        case LV_FS_SEEK_SET: f->seek(pos, SeekSet); break;
        case LV_FS_SEEK_CUR: f->seek(pos, SeekCur); break;
        case LV_FS_SEEK_END: f->seek(pos, SeekEnd); break;
    }
    return LV_FS_RES_OK;
}

/**
 * @brief LVGL fs_drv tell callback.
 * @param fp Handle returned by fsOpen.
 * @param[out] pos Current read offset.
 * @return Always LV_FS_RES_OK.
 */
static lv_fs_res_t fsTell(lv_fs_drv_t *, void *fp, uint32_t *pos) {
    *pos = static_cast<File *>(fp)->position();
    return LV_FS_RES_OK;
}

void DisplayManager::registerLvglFilesystem() {
    static lv_fs_drv_t drv;
    lv_fs_drv_init(&drv);
    drv.letter   = 'L';
    drv.open_cb  = fsOpen;
    drv.close_cb = fsClose;
    drv.read_cb  = fsRead;
    drv.seek_cb  = fsSeek;
    drv.tell_cb  = fsTell;
    lv_fs_drv_register(&drv);
}
