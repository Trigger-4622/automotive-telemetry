/*
 * LVGL on the PC: a framebuffer display the size of the board's panel, PNG
 * screenshots of it, and the loop that runs virtual time while the simulated
 * master broadcasts.
 */
#include <Arduino.h>
#include <lvgl.h>

#include <string>
#include <vector>

#include "HardwareConfig.h"
#include "MasterPacket.h"
#include "TelemetryStore.h"
#include "host.h"

static lv_color_t          s_fb[LCD_WIDTH * LCD_HEIGHT];
static lv_color_t          s_draw[LCD_WIDTH * 40];
static lv_disp_draw_buf_t  s_dbuf;
static lv_disp_drv_t       s_drv;

static void flushCb(lv_disp_drv_t *d, const lv_area_t *a, lv_color_t *px) {
    const int w = a->x2 - a->x1 + 1;
    for (int y = a->y1; y <= a->y2; y++)
        memcpy(&s_fb[y * LCD_WIDTH + a->x1], px + (y - a->y1) * w, w * sizeof(lv_color_t));
    lv_disp_flush_ready(d);
}

void host::lvglInit() {
    lv_init();
    lv_disp_draw_buf_init(&s_dbuf, s_draw, nullptr, LCD_WIDTH * 40);
    lv_disp_drv_init(&s_drv);
    s_drv.hor_res  = LCD_WIDTH;
    s_drv.ver_res  = LCD_HEIGHT;
    s_drv.flush_cb = flushCb;
    s_drv.draw_buf = &s_dbuf;
    lv_disp_drv_register(&s_drv);
    // The theme the UI will apply, so the baseline includes its styles.
    lv_theme_default_init(lv_disp_get_default(), lv_palette_main(LV_PALETTE_CYAN),
                          lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    heapBaseline();
}

host::Mem host::lvglMem() {
    lv_mem_monitor_t m;
    lv_mem_monitor(&m);
    return {(uint32_t)(m.total_size - m.free_size), (uint32_t)m.free_size,
            (uint32_t)m.free_biggest_size, m.frag_pct};
}

/* ──────────────────────────────── PNG ──────────────────────────────────── */

static uint32_t crc32(const uint8_t *p, size_t n, uint32_t c = 0) {
    static uint32_t t[256];
    if (!t[1])
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t v = i;
            for (int k = 0; k < 8; k++) v = (v & 1) ? 0xEDB88320u ^ (v >> 1) : v >> 1;
            t[i] = v;
        }
    c = ~c;
    while (n--) c = t[(c ^ *p++) & 0xFF] ^ (c >> 8);
    return ~c;
}

static void be32(std::vector<uint8_t> &o, uint32_t v) {
    for (int s = 24; s >= 0; s -= 8) o.push_back((uint8_t)(v >> s));
}

static void chunk(std::vector<uint8_t> &png, const char *type, const std::vector<uint8_t> &d) {
    be32(png, (uint32_t)d.size());
    std::vector<uint8_t> td(type, type + 4);
    td.insert(td.end(), d.begin(), d.end());
    png.insert(png.end(), td.begin(), td.end());
    be32(png, crc32(td.data(), td.size()));
}

/* Stored (uncompressed) deflate: no zlib needed, and the files are small. */
static void writePng(const std::string &path, int w, int h, const std::vector<uint8_t> &rgb) {
    std::vector<uint8_t> raw;
    for (int y = 0; y < h; y++) {
        raw.push_back(0);
        raw.insert(raw.end(), rgb.begin() + y * w * 3, rgb.begin() + (y + 1) * w * 3);
    }
    std::vector<uint8_t> z = {0x78, 0x01};
    uint32_t a = 1, b = 0;
    for (uint8_t v : raw) { a = (a + v) % 65521; b = (b + a) % 65521; }
    for (size_t off = 0; off < raw.size(); off += 65535) {
        const size_t n = std::min<size_t>(65535, raw.size() - off);
        z.push_back(off + n >= raw.size() ? 1 : 0);
        z.push_back(n & 0xFF); z.push_back(n >> 8);
        z.push_back(~n & 0xFF); z.push_back((~n >> 8) & 0xFF);
        z.insert(z.end(), raw.begin() + off, raw.begin() + off + n);
    }
    be32(z, (b << 16) | a);

    std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    std::vector<uint8_t> ihdr;
    be32(ihdr, w); be32(ihdr, h);
    ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});
    chunk(png, "IHDR", ihdr);
    chunk(png, "IDAT", z);
    chunk(png, "IEND", {});
    FILE *f = fopen(path.c_str(), "wb");
    if (f) { fwrite(png.data(), 1, png.size(), f); fclose(f); }
}

void host::screenshot(const std::string &path) {
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(nullptr);
    std::vector<uint8_t> rgb(LCD_WIDTH * LCD_HEIGHT * 3);
    const bool round = LCD_WIDTH == LCD_HEIGHT;
    const float r = LCD_WIDTH / 2.0f;
    for (int y = 0; y < LCD_HEIGHT; y++)
        for (int x = 0; x < LCD_WIDTH; x++) {
            const uint32_t c = lv_color_to32(s_fb[y * LCD_WIDTH + x]);
            uint8_t *p = &rgb[(y * LCD_WIDTH + x) * 3];
            p[0] = (c >> 16) & 0xFF; p[1] = (c >> 8) & 0xFF; p[2] = c & 0xFF;
            // The round board's glass: what lies outside the circle is never seen.
            const float dx = x + 0.5f - r, dy = y + 0.5f - r;
            if (round && dx * dx + dy * dy > r * r) { p[0] = p[1] = p[2] = 40; }
        }
    writePng(path, LCD_WIDTH, LCD_HEIGHT, rgb);
}

/* ─────────────────────────────── the car ───────────────────────────────── */

host::Car host::car;

void host::Car::send() {
    if (!radio) return;
    std::vector<std::pair<uint16_t, float>> all(values.begin(), values.end());
    static const uint8_t mac[6] = {0x24, 0x6F, 0x28, 1, 2, 3};
    for (size_t off = 0; off < all.size() || off == 0; off += TELEMETRY_MAX_METRICS) {
        MasterTelemetryPacket p{};
        p.sequence_id  = ++seq;
        p.timestamp_ms = millis();
        const size_t n = std::min(all.size() - off, (size_t)TELEMETRY_MAX_METRICS);
        p.metric_count = (uint8_t)n;
        for (size_t i = 0; i < n; i++) {
            const uint16_t id = all[off + i].first;
            p.metrics[i].metric_id = id;
            p.metrics[i].value     = all[off + i].second;
            p.metrics[i].flags     = METRIC_FLAG_VALID | (night ? METRIC_FLAG_NIGHT : 0) |
                                     (flags.count(id) ? flags[id] : 0);
        }
        Telemetry.ingestRaw(mac, (const uint8_t *)&p, (int)TELEMETRY_PACKET_SIZE(n));
        if (all.empty()) break;
    }
    lastSend = millis();
}

void host::run(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 5) {
        setMillis(millis() + 5);
        if (millis() - car.lastSend >= 100) car.send();
        lv_timer_handler();
    }
}
