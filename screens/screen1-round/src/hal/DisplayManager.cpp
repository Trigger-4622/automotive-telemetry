/**
 * @file DisplayManager.cpp
 * @ingroup hal
 * @brief Panel + LVGL bring-up and the LVGL "L:" LittleFS driver.
 */
#include "DisplayManager.h"

#include <LittleFS.h>

DisplayManager Display;

/* Two 60-line partial buffers (240 × 60 × 2 B = 28.1 KB each) let LVGL render
 * into one while the other streams out over SPI. 60 divides 240 exactly, so a
 * full-screen repaint — what every screen transition costs — is 4 flushes
 * instead of 6, cutting DMA setup overhead on the frames that need it most. */
static constexpr uint32_t BUF_LINES = 60;
static lv_color_t s_buf1[LCD_WIDTH * BUF_LINES];
static lv_color_t s_buf2[LCD_WIDTH * BUF_LINES];

void DisplayManager::flushCb(lv_disp_drv_t *drv, const lv_area_t *area,
                             lv_color_t *pixels) {
    auto *self = static_cast<DisplayManager *>(drv->user_data);
    const uint32_t w = area->x2 - area->x1 + 1;
    const uint32_t h = area->y2 - area->y1 + 1;

    // Zero-copy DMA: LV_COLOR_16_SWAP=1 means the buffer is already in panel
    // wire order (swap565_t), so no CPU conversion happens. flush_ready fires
    // immediately — LVGL renders the next chunk into its second buffer while
    // this one streams out; LovyanGFX blocks internally if a new transfer is
    // issued before the previous DMA completes, so buffer reuse stays safe.
    if (self->_gfx.getStartCount() == 0) self->_gfx.startWrite();
    self->_gfx.pushImageDMA(area->x1, area->y1, w, h,
                            reinterpret_cast<lgfx::swap565_t *>(&pixels->full));
    lv_disp_flush_ready(drv);
}

void DisplayManager::begin() {
    _gfx.init();
    _gfx.setBrightness(BRIGHTNESS_DAY);
    _gfx.startWrite();   // display owns the SPI bus — hold CS for DMA overlap

    lv_init();
    lv_disp_draw_buf_init(&_drawBuf, s_buf1, s_buf2, LCD_WIDTH * BUF_LINES);

    lv_disp_drv_init(&_dispDrv);
    _dispDrv.hor_res   = LCD_WIDTH;
    _dispDrv.ver_res   = LCD_HEIGHT;
    _dispDrv.flush_cb  = flushCb;
    _dispDrv.draw_buf  = &_drawBuf;
    _dispDrv.user_data = this;
    _disp = lv_disp_drv_register(&_dispDrv);

    registerLvglFilesystem();
    log_i("Display up: %dx%d @ %d MHz SPI", LCD_WIDTH, LCD_HEIGHT,
          LCD_SPI_HZ / 1000000);
}

void DisplayManager::setBrightness(uint8_t level) {
    _gfx.setBrightness(level);
}

/* ─────────────────── LVGL "L:" drive backed by LittleFS ────────────────────
 * Lets layout.json reference uploaded images as  "L:/assets/bg.bin"  and have
 * lv_img stream them straight from flash.
 *
 * NOTE ON COST: this is a streaming bridge, not a cache. LVGL's built-in
 * decoder re-reads a file-backed image line by line on EVERY repaint, so a
 * full-screen 240x240 background costs ~115 KB of flash reads per full
 * repaint. Prefer a gradient or a compiled-in tile (see Textures.h) on any
 * screen that animates. */

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
