/**
 * @file DriveViews.cpp
 * @brief The G-meter and the performance timers - see DriveViews.h.
 */
#include "DriveViews.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Palette.h"

/** @brief A label, aligned in @p parent. */
static lv_obj_t *label(lv_obj_t *parent, const char *txt, const lv_font_t *font,
                       uint32_t color, lv_align_t align, int x, int y) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_obj_align(l, align, x, y);
    return l;
}

/** @brief A plain shape: no scrolling, no clicks, no padding. */
static lv_obj_t *shape(lv_obj_t *parent, int w, int h) {
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, w, h);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return o;
}

static void setText(lv_obj_t *l, char *last, size_t cap, const char *txt) {
    if (!strcmp(last, txt)) return;
    strncpy(last, txt, cap - 1);
    last[cap - 1] = '\0';
    lv_label_set_text(l, txt);
}

/* ═══════════════════════════════ G-meter ═════════════════════════════════ */

void gmeterBuild(GMeterView &v, lv_obj_t *parent, int w, int h, float maxG) {
    v.maxG = maxG > 0.1f ? maxG : 1.0f;
    const bool wide = w > h * 1.4f;
    // Round: the circle in the middle, the numbers inside its lower half.
    // Landscape: the circle on the left, the numbers to its right.
    v.r  = wide ? (h - 56) / 2 : (int)(w * 0.36f);
    v.cx = wide ? v.r + 36 : w / 2;
    v.cy = wide ? h / 2 + 6 : h / 2 + 4;

    auto ring = [&](int rad, int width, uint32_t color) {
        lv_obj_t *o = shape(parent, 2 * rad, 2 * rad);
        lv_obj_set_pos(o, v.cx - rad, v.cy - rad);
        lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(o, width, 0);
        lv_obj_set_style_border_color(o, lv_color_hex(color), 0);
        lv_obj_set_style_border_opa(o, LV_OPA_COVER, 0);
    };
    ring(v.r, 2, UI_LINE);
    ring(v.r / 2, 1, UI_LINE_SOFT);
    lv_obj_t *hx = shape(parent, 2 * v.r, 1), *vx = shape(parent, 1, 2 * v.r);
    lv_obj_t *cross[2] = { hx, vx };
    for (lv_obj_t *o : cross) {
        lv_obj_set_style_bg_color(o, lv_color_hex(UI_LINE_SOFT), 0);
        lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    }
    lv_obj_set_pos(hx, v.cx - v.r, v.cy);
    lv_obj_set_pos(vx, v.cx, v.cy - v.r);

    char t[16];
    snprintf(t, sizeof t, "%.1f", v.maxG);
    label(parent, t, &lv_font_montserrat_12, UI_DIM, LV_ALIGN_TOP_LEFT, v.cx + 4, v.cy - v.r + 2);
    snprintf(t, sizeof t, "%.2g", v.maxG / 2);
    label(parent, t, &lv_font_montserrat_12, UI_DIM, LV_ALIGN_TOP_LEFT, v.cx + 4, v.cy - v.r / 2 + 2);
    label(parent, "G", &lv_font_montserrat_14, UI_ACCENT, LV_ALIGN_TOP_MID,
          wide ? v.cx - w / 2 : 0, wide ? 8 : 18);

    if (wide) {
        const int x = v.cx + v.r + 36;
        label(parent, "LATERAL", &lv_font_montserrat_12, UI_MUTED, LV_ALIGN_TOP_LEFT, x, h / 2 - 82);
        v.lat = label(parent, "--", &lv_font_montserrat_32, UI_TEXT, LV_ALIGN_TOP_LEFT, x, h / 2 - 66);
        label(parent, "LONGITUDINAL", &lv_font_montserrat_12, UI_MUTED, LV_ALIGN_TOP_LEFT, x, h / 2 - 20);
        v.lon = label(parent, "--", &lv_font_montserrat_32, UI_TEXT, LV_ALIGN_TOP_LEFT, x, h / 2 - 4);
        v.peak = label(parent, "", &lv_font_montserrat_14, UI_MUTED, LV_ALIGN_TOP_LEFT, x, h / 2 + 48);
    } else {
        v.lat  = label(parent, "--", &lv_font_montserrat_14, UI_TEXT_DIM, LV_ALIGN_TOP_RIGHT,
                       -(w / 2 + 10), v.cy + v.r / 2 + 6);
        v.lon  = label(parent, "--", &lv_font_montserrat_14, UI_TEXT_DIM, LV_ALIGN_TOP_LEFT,
                       w / 2 + 10, v.cy + v.r / 2 + 6);
        v.peak = label(parent, "", &lv_font_montserrat_12, UI_MUTED, LV_ALIGN_TOP_MID, 0,
                       v.cy + v.r / 2 + 26);
    }

    v.dot = shape(parent, 16, 16);
    lv_obj_set_style_radius(v.dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(v.dot, lv_color_hex(UI_ACCENT), 0);
    lv_obj_set_style_bg_opa(v.dot, LV_OPA_COVER, 0);
    lv_obj_set_pos(v.dot, v.cx - 8, v.cy - 8);
    lv_obj_add_flag(v.dot, LV_OBJ_FLAG_HIDDEN);
    v.lastShown = false;
}

void gmeterUpdate(GMeterView &v, float latG, bool latOk, float lonG, bool lonOk) {
    if (!v.dot) return;
    const bool show = latOk || lonOk;
    if (show != v.lastShown) {
        v.lastShown = show;
        if (show) lv_obj_clear_flag(v.dot, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_add_flag(v.dot, LV_OBJ_FLAG_HIDDEN);
    }
    const float x = latOk ? latG : 0, y = lonOk ? lonG : 0;
    // Left turn (lateral +) to the left, speeding up (longitudinal +) upward;
    // held at the rim beyond it.
    float dx = -x / v.maxG * v.r, dy = -y / v.maxG * v.r;
    const float d = sqrtf(dx * dx + dy * dy);
    if (d > v.r) { dx *= v.r / d; dy *= v.r / d; }
    const int16_t px = (int16_t)lroundf(v.cx + dx - 8), py = (int16_t)lroundf(v.cy + dy - 8);
    if (show && (px != v.lastX || py != v.lastY)) {
        v.lastX = px; v.lastY = py;
        lv_obj_set_pos(v.dot, px, py);
    }

    char t[20];
    if (latOk) snprintf(t, sizeof t, "%s%.2f", x > 0.005f ? "L " : x < -0.005f ? "R " : "", fabsf(x));
    else       strcpy(t, "--");
    setText(v.lat, v.lastLat, sizeof v.lastLat, t);
    if (lonOk) snprintf(t, sizeof t, "%s%.2f", y > 0.005f ? "ACC " : y < -0.005f ? "BRK " : "", fabsf(y));
    else       strcpy(t, "--");
    setText(v.lon, v.lastLon, sizeof v.lastLon, t);
    const float g = sqrtf(x * x + y * y);
    if (show && g > v.peakG) v.peakG = g;
    if (v.peakG > 0.05f) snprintf(t, sizeof t, "MAX %.2f g", v.peakG);
    else                 t[0] = '\0';
    setText(v.peak, v.lastPeak, sizeof v.lastPeak, t);
}

/* ═══════════════════════════ performance timers ══════════════════════════ */

static const char *const RUN_NAME[PerfTimer::RUNS] = { "0-100", "80-120", "1/4 MI" };

void perfBuild(PerfView &v, lv_obj_t *parent, int w, int h) {
    const bool wide = w > h * 1.4f;
    // Round: three rows inside the circle. Landscape: the same, wider apart.
    const int x = wide ? 56 : 34, top = wide ? 44 : 50, pitch = wide ? 62 : 46;
    label(parent, "PERFORMANCE", &lv_font_montserrat_12, UI_ACCENT, LV_ALIGN_TOP_MID, 0, wide ? 12 : 24);
    for (int r = 0; r < PerfTimer::RUNS; r++) {
        const int y = top + r * pitch;
        label(parent, RUN_NAME[r], &lv_font_montserrat_14, UI_MUTED, LV_ALIGN_TOP_LEFT, x, y + 2);
        v.best[r] = label(parent, "", &lv_font_montserrat_12, UI_DIM, LV_ALIGN_TOP_LEFT, x, y + 21);
        v.val[r]  = label(parent, "--", wide ? &lv_font_montserrat_40 : &lv_font_montserrat_28,
                          UI_TEXT, LV_ALIGN_TOP_RIGHT, -x, y);
        v.lastColor[r] = UI_TEXT;
    }
    // Under the quarter mile's time, right-aligned with it.
    v.trap = label(parent, "", &lv_font_montserrat_12, UI_DIM, LV_ALIGN_TOP_RIGHT, -x,
                   top + 2 * pitch + (wide ? 44 : 31));
    v.status = label(parent, "", &lv_font_montserrat_14, UI_MUTED, LV_ALIGN_TOP_MID, 0,
                     top + 3 * pitch + (wide ? 4 : 2));
}

void perfUpdate(PerfView &v, const PerfTimer &p, uint32_t masterNow, bool speedKnown) {
    if (!v.status) return;
    bool any = false;
    for (int r = 0; r < PerfTimer::RUNS; r++) {
        const PerfTimer::Run run = (PerfTimer::Run)r;
        char t[16], b[32];
        uint32_t color = UI_TEXT;
        if (p.running(run)) {
            any = true;
            snprintf(t, sizeof t, "%.1f", p.elapsed(run, masterNow));
            color = UI_ACCENT;
        } else if (!isnan(p.last(run).secs)) {
            snprintf(t, sizeof t, "%.2fs", p.last(run).secs);
        } else {
            strcpy(t, "--");
        }
        setText(v.val[r], v.lastVal[r], sizeof v.lastVal[r], t);
        if (color != v.lastColor[r]) {
            v.lastColor[r] = color;
            lv_obj_set_style_text_color(v.val[r], lv_color_hex(color), 0);
        }
        const PerfTimer::Result &best = p.best(run), &last = p.last(run);
        b[0] = '\0';
        if (!isnan(best.secs)) snprintf(b, sizeof b, "BEST %.2f", best.secs);
        setText(v.best[r], v.lastBest[r], sizeof v.lastBest[r], b);
        if (run == PerfTimer::QUARTER) {
            char tr[20] = "";
            if (!isnan(last.trapKmh) && !p.running(run)) snprintf(tr, sizeof tr, "AT %.0f KM/H", last.trapKmh);
            setText(v.trap, v.lastTrap, sizeof v.lastTrap, tr);
        }
    }
    const char *s = !speedKnown ? "NO SPEED"
                  : any         ? "TIMING"
                  : p.ready()   ? "READY - GO"
                                : "STOP TO START A RUN";
    setText(v.status, v.lastStatus, sizeof v.lastStatus, s);
    const uint32_t c = !speedKnown ? UI_DIM : any ? UI_ACCENT : p.ready() ? UI_OK : UI_MUTED;
    if (c != v.lastStatusColor) {
        v.lastStatusColor = c;
        lv_obj_set_style_text_color(v.status, lv_color_hex(c), 0);
    }
}
