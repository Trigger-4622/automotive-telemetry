/**
 * @file DriveViews.cpp
 * @brief The G-meter, the performance timers and the parking screen - see
 *        DriveViews.h.
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

/* ═══════════════════════════ the car from above ══════════════════════════ */

/* The Legacy BM's measures [m]: the wheelbase is the layout's. */
static constexpr float CAR_W = 1.78f, CAR_LEN = 4.735f, CAR_RO = 1.035f, CAR_TRACK = 1.53f;
static constexpr int   CAR_PTS = 2 * 6 + 2 * 2 + 2 * CAR_PATH_PTS;   /**< Points of every line. */
static constexpr float DEG = 0.017453293f;                         /**< To radians.          */

static constexpr float CAR_FRONT = CAR_LEN - CAR_RO;               /**< Front bumper [m].    */
static int   carX(const CarView &v, float x) { return (int)lroundf(v.cx + x * v.ppm); }
static int   carY(const CarView &v, float y) { return (int)lroundf(v.axleY - y * v.ppm); }

/** Put line @p l through @p n points (parent coordinates): the object spans
 *  only them, so a change redraws only around it. */
static void carLine(lv_obj_t *l, lv_point_t *p, const int (*xy)[2], int n) {
    int ox = xy[0][0], oy = xy[0][1];
    for (int i = 1; i < n; i++) { if (xy[i][0] < ox) ox = xy[i][0]; if (xy[i][1] < oy) oy = xy[i][1]; }
    ox -= 6; oy -= 6;
    for (int i = 0; i < n; i++) { p[i].x = (lv_coord_t)(xy[i][0] - ox); p[i].y = (lv_coord_t)(xy[i][1] - oy); }
    lv_obj_set_pos(l, ox, oy);
    lv_line_set_points(l, p, (uint16_t)n);
}

static lv_obj_t *carLineNew(lv_obj_t *parent, int width, uint32_t color) {
    lv_obj_t *l = lv_line_create(parent);
    lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_line_width(l, width, 0);
    lv_obj_set_style_line_rounded(l, true, 0);
    lv_obj_set_style_line_color(l, lv_color_hex(color), 0);
    return l;
}

static void showIf(lv_obj_t *o, bool on) {
    if (on) lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
    else    lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}

/** Side @p i (doors FL FR RL RR, boot, bonnet) shut, or swung open. */
static void carSide(CarView &v, int i, bool open) {
    int xy[2][2];
    const float wb = v.g.wheelbase;
    if (i < 4) {
        const float sx = (i & 1) ? 1.0f : -1.0f;               // FL RL left, FR RR right
        const float y1 = (i < 2 ? 0.80f : 0.42f) * wb, y2 = (i < 2 ? 0.42f : 0.08f) * wb;
        const float len = y1 - y2, x = sx * CAR_W / 2;
        xy[0][0] = carX(v, x); xy[0][1] = carY(v, y1);           // the hinge, at the front
        if (open) { xy[1][0] = carX(v, x + sx * len * 0.82f); xy[1][1] = carY(v, y1 - len * 0.57f); }
        else      { xy[1][0] = carX(v, x);                     xy[1][1] = carY(v, y2); }
    } else {
        const bool boot = i == 4;
        const float y = boot ? (open ? -CAR_RO - 0.40f : -CAR_RO + 0.10f)
                             : (open ? CAR_FRONT + 0.40f : CAR_FRONT - 0.10f);
        xy[0][0] = carX(v, -CAR_W / 2 + 0.28f); xy[0][1] = carY(v, y);
        xy[1][0] = carX(v,  CAR_W / 2 - 0.28f); xy[1][1] = carY(v, y);
    }
    carLine(v.side[i], v.pts + 2 * i, xy, 2);
}

/** The front wheels, turned @p road degrees (+ left). */
static void carWheels(CarView &v, float road) {
    const float s = sinf(road * DEG) * 0.31f, c = cosf(road * DEG) * 0.31f;
    for (int k = 0; k < 2; k++) {
        const float x = (k ? 1.0f : -1.0f) * CAR_TRACK / 2, y = v.g.wheelbase;
        const int xy[2][2] = { { carX(v, x + s), carY(v, y - c) }, { carX(v, x - s), carY(v, y + c) } };
        carLine(v.wheel[k], v.pts + 12 + 2 * k, xy, 2);
    }
}

/** The corners' path: @p dir -1 back from the rear bumper, 1 ahead from the
 *  front one, the road wheels turned @p road degrees (+ left). */
static void carPath(CarView &v, float road, int dir) {
    const float wb = v.g.wheelbase, y0 = dir < 0 ? -CAR_RO : CAR_FRONT;
    const float room = (dir < 0 ? v.bottom - carY(v, y0) : carY(v, y0) - v.top) / v.ppm;
    const float len = room < v.g.pathM ? (room > 0 ? room : 0) : v.g.pathM;
    const float t = tanf(road * DEG);
    for (int k = 0; k < 2; k++) {
        const float x0 = (k ? 1.0f : -1.0f) * CAR_W / 2;
        int xy[CAR_PATH_PTS][2];
        for (int n = 0; n < CAR_PATH_PTS; n++) {
            const float sArc = dir * len * n / (CAR_PATH_PTS - 1);
            float x = x0, y = y0 + sArc;
            if (fabsf(t) > 1e-4f) {
                // About a centre on the rear axle's line, wb / tan(road) to the
                // side the wheels are turned; the rear axle moves sArc along it.
                const float R = wb / t, cxm = -R, phi = sArc / R;
                const float dx = x0 - cxm, dy = y0;
                x = cxm + dx * cosf(phi) - dy * sinf(phi);
                y = dx * sinf(phi) + dy * cosf(phi);
            }
            xy[n][0] = carX(v, x);
            xy[n][1] = carY(v, y);
        }
        carLine(v.path[k], v.pts + 16 + CAR_PATH_PTS * k, xy, CAR_PATH_PTS);
    }
}

void carBuild(CarView &v, lv_obj_t *parent, int cx, int axleY, float ppm, int top, int bottom,
              const CarGeom &g) {
    v.g = g;
    if (!(v.g.ratio >= 5 && v.g.ratio <= 40)) v.g.ratio = 15.0f;
    if (!(v.g.wheelbase >= 1.5f && v.g.wheelbase <= 4.0f)) v.g.wheelbase = 2.75f;
    v.ppm = ppm; v.cx = cx; v.axleY = axleY; v.top = top; v.bottom = bottom;
    const float wb = v.g.wheelbase;

    v.body = shape(parent, (int)lroundf(CAR_W * ppm), (int)lroundf(CAR_LEN * ppm));
    lv_obj_set_pos(v.body, carX(v, -CAR_W / 2), carY(v, CAR_FRONT));
    lv_obj_set_style_radius(v.body, (int)lroundf(0.35f * ppm), 0);
    lv_obj_set_style_bg_color(v.body, lv_color_hex(UI_PANEL), 0);
    lv_obj_set_style_bg_opa(v.body, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(v.body, 2, 0);
    lv_obj_set_style_border_color(v.body, lv_color_hex(UI_MUTED), 0);
    lv_obj_set_style_border_opa(v.body, LV_OPA_COVER, 0);
    const float cy1 = 0.86f * wb, cy0 = 0.02f * wb;                // the glasshouse
    v.cabin = shape(parent, (int)lroundf((CAR_W - 0.36f) * ppm), (int)lroundf((cy1 - cy0) * ppm));
    lv_obj_set_pos(v.cabin, carX(v, -CAR_W / 2 + 0.18f), carY(v, cy1));
    lv_obj_set_style_radius(v.cabin, (int)lroundf(0.25f * ppm), 0);
    lv_obj_set_style_border_width(v.cabin, 1, 0);
    lv_obj_set_style_border_color(v.cabin, lv_color_hex(UI_LINE), 0);
    lv_obj_set_style_border_opa(v.cabin, LV_OPA_COVER, 0);

    const int ww = (int)lroundf(0.2f * ppm) < 4 ? 4 : (int)lroundf(0.2f * ppm);
    for (int k = 0; k < 2; k++) {                                  // rear wheels
        lv_obj_t *r = shape(parent, ww, (int)lroundf(0.62f * ppm));
        lv_obj_set_pos(r, carX(v, (k ? 1.0f : -1.0f) * CAR_TRACK / 2) - ww / 2, carY(v, 0.31f));
        lv_obj_set_style_radius(r, 2, 0);
        lv_obj_set_style_bg_color(r, lv_color_hex(UI_DIM), 0);
        lv_obj_set_style_bg_opa(r, LV_OPA_COVER, 0);
        v.wheel[2 + k] = r;
    }
    v.pts = (lv_point_t *)lv_mem_alloc(sizeof(lv_point_t) * CAR_PTS);   // for good, as the screen
    if (!v.pts) return;
    memset(v.pts, 0, sizeof(lv_point_t) * CAR_PTS);
    for (int k = 0; k < 2; k++) v.wheel[k] = carLineNew(parent, ww, UI_DIM);
    carWheels(v, 0);
    for (int i = 0; i < 6; i++) {
        v.side[i] = carLineNew(parent, i < 4 ? 4 : 3, UI_LINE);
        carSide(v, i, false);
    }
    for (int i = 0; i < 4; i++) {                                  // indicators at the corners
        lv_obj_t *d = shape(parent, 8, 8);
        lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(d, lv_color_hex(UI_LAMP_GREEN), 0);
        lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
        const float x = (i & 1 ? 1.0f : -1.0f) * (CAR_W / 2 + 0.05f);
        const float y = i < 2 ? CAR_FRONT - 0.25f : -CAR_RO + 0.25f;
        lv_obj_set_pos(d, carX(v, x) - 4, carY(v, y) - 4);
        lv_obj_add_flag(d, LV_OBJ_FLAG_HIDDEN);
        v.corner[i] = d;
    }
    for (int k = 0; k < 2; k++) {                                  // headlights
        lv_obj_t *h = shape(parent, (int)lroundf(0.38f * ppm), 3);
        lv_obj_set_pos(h, carX(v, (k ? 1.0f : -1.0f) * (CAR_W / 2 - 0.30f)) - (int)lroundf(0.19f * ppm),
                       carY(v, CAR_FRONT - 0.12f));
        lv_obj_set_style_radius(h, 1, 0);
        lv_obj_set_style_bg_opa(h, LV_OPA_COVER, 0);
        lv_obj_add_flag(h, LV_OBJ_FLAG_HIDDEN);
        v.head[k] = h;
    }
    for (int k = 0; k < 2; k++) {
        v.path[k] = carLineNew(parent, 3, UI_ACCENT);
        lv_obj_add_flag(v.path[k], LV_OBJ_FLAG_HIDDEN);
    }
    v.lastDeg = INT16_MIN;
    v.lastDir = 0;
}

void carUpdate(CarView &v, const CarState &s) {
    if (!v.body || !v.pts) return;
    // Doors: where none is known on its own, "a door is open" lights them all.
    const bool singles = s.door[0] >= 0 || s.door[1] >= 0 || s.door[2] >= 0 || s.door[3] >= 0;
    for (int i = 0; i < 6; i++) {
        int8_t st = i < 4 ? s.door[i] : i == 4 ? s.boot : s.bonnet;
        bool amberShut = false;
        if (i < 4 && !singles && s.anyDoor >= 0) { amberShut = s.anyDoor == 1; st = 0; }
        const int8_t code = amberShut ? 2 : st;                  // 2: one of them, not known which
        if (code == v.lastSide[i]) continue;
        v.lastSide[i] = code;
        carSide(v, i, code == 1);
        const uint32_t c = code == 1 || code == 2 ? UI_WARN : code == 0 ? UI_MUTED : UI_LINE;
        lv_obj_set_style_line_color(v.side[i], lv_color_hex(c), 0);
    }
    const uint8_t lamps = (uint8_t)((s.turnL ? 1 : 0) | (s.turnR ? 2 : 0) | (s.lights == 1 ? 4 : 0) |
                                    (s.highBeam == 1 ? 8 : 0));
    if (lamps != v.lastLamps) {
        v.lastLamps = lamps;
        showIf(v.corner[0], s.turnL); showIf(v.corner[2], s.turnL);
        showIf(v.corner[1], s.turnR); showIf(v.corner[3], s.turnR);
        for (lv_obj_t *h : v.head) {
            showIf(h, s.lights == 1 || s.highBeam == 1);
            lv_obj_set_style_bg_color(h, lv_color_hex(s.highBeam == 1 ? UI_LAMP_BLUE : UI_LAMP_GREEN), 0);
        }
    }
    const int8_t dir = s.angleOk && v.g.path ? (s.reverse ? -1 : 1) : 0;
    const int16_t deg = s.angleOk ? (int16_t)lroundf(s.angle > 3600 ? 3600 : s.angle < -3600 ? -3600 : s.angle) : 0;
    if (deg != v.lastDeg || dir != v.lastDir) {
        v.lastDeg = deg;
        v.lastDir = dir;
        const float road = deg / v.g.ratio;
        carWheels(v, road);
        for (lv_obj_t *w : v.wheel)
            if (w) lv_obj_set_style_line_color(w, lv_color_hex(s.angleOk ? UI_TEXT_DIM : UI_DIM), 0);
        for (lv_obj_t *p : v.path) showIf(p, dir != 0);
        if (dir) carPath(v, road, dir);
    }
}

/* The "car" screen: round, the car in the middle and one line under it;
   landscape, the car on the left and a row for each thing to its right. */

void carScreenBuild(CarScreenView &v, lv_obj_t *parent, int w, int h, const CarGeom &g) {
    const bool wide = w > h * 1.4f;
    CarGeom gg = g;
    gg.pathM = 1.5f;
    const float ppm = wide ? (h - 16) / (CAR_LEN + 3.0f) : 22.0f;
    const int cx = wide ? 150 : w / 2;
    const int mid = wide ? h / 2 : h / 2 - 4;
    const int axleY = (int)lroundf(mid + (CAR_LEN / 2 - CAR_RO) * ppm);
    carBuild(v.car, parent, cx, axleY, ppm, wide ? 4 : 42, h - 4, gg);
    if (wide) {
        // Two labels, a line each, not ten: the cluster's LVGL heap is the
        // tight one. Captions above the values, both 46 px apart.
        lv_obj_t *cap = label(parent, "DOORS\nBOOT\nBONNET\nLIGHTS\nHANDBRAKE", &lv_font_montserrat_12,
                              UI_MUTED, LV_ALIGN_TOP_LEFT, 300, 28);
        lv_obj_set_style_text_line_space(cap, 46 - 14, 0);
        v.vals = label(parent, "", &lv_font_montserrat_20, UI_TEXT_DIM, LV_ALIGN_TOP_LEFT, 300, 44);
        lv_obj_set_style_text_line_space(v.vals, 46 - 22, 0);
        lv_label_set_recolor(v.vals, true);
    } else {
        // At the top: the lamp strip has the bottom of the glass.
        v.status = label(parent, "", &lv_font_montserrat_14, UI_MUTED, LV_ALIGN_TOP_MID, 0, 18);
    }
}

static void setTextColor(lv_obj_t *l, uint32_t &last, uint32_t rgb) {
    if (last == rgb) return;
    last = rgb;
    lv_obj_set_style_text_color(l, lv_color_hex(rgb), 0);
}

void carScreenUpdate(CarScreenView &v, const CarState &s) {
    carUpdate(v.car, s);
    static const char *const DOOR[4] = {"FL", "FR", "RL", "RR"};
    // Which doors are open, in words.
    char doors[24] = "";
    int open = 0, known = 0;
    for (int i = 0; i < 4; i++) {
        if (s.door[i] < 0) continue;
        known++;
        if (s.door[i] == 1) {
            if (open++) strlcat(doors, " ", sizeof doors);
            strlcat(doors, DOOR[i], sizeof doors);
        }
    }
    int8_t anyDoor = known ? (open ? 1 : 0) : s.anyDoor;
    if (!known && s.anyDoor == 1) strlcpy(doors, "OPEN", sizeof doors);
    if (v.status) {
        char t[40];
        uint32_t c = UI_MUTED;
        if (anyDoor == 1 || s.boot == 1 || s.bonnet == 1) {
            t[0] = '\0';
            if (anyDoor == 1) { strlcat(t, open > 1 ? "DOORS " : "DOOR ", sizeof t); strlcat(t, doors, sizeof t); }
            if (s.boot == 1)   strlcat(t, t[0] ? " + BOOT" : "BOOT OPEN", sizeof t);
            if (s.bonnet == 1) strlcat(t, t[0] ? " + BONNET" : "BONNET OPEN", sizeof t);
            c = UI_WARN;
        } else if (anyDoor == 0 || s.boot == 0 || s.bonnet == 0) {
            strlcpy(t, s.handbrake == 1 ? "ALL SHUT \xE2\x80\xA2 HANDBRAKE" : "ALL SHUT", sizeof t);
            c = UI_OK;
        } else {
            strlcpy(t, "NO DOOR DATA", sizeof t);
            c = UI_DIM;
        }
        setText(v.status, v.lastStatus, sizeof v.lastStatus, t);
        setTextColor(v.status, v.lastStatusRgb, c);
    }
    if (v.vals) {
        const int8_t st[5] = {anyDoor, s.boot, s.bonnet, s.lights, s.handbrake};
        char all[160] = "";
        for (int i = 0; i < 5; i++) {
            const char *t = st[i] < 0 ? "--" : (i == 0 && st[i] == 1) ? doors
                          : i == 3 ? (st[i] ? (s.highBeam == 1 ? "HIGH BEAM" : "ON") : "OFF")
                          : i == 4 ? (st[i] ? "ON" : "OFF") : (st[i] ? "OPEN" : "SHUT");
            const uint32_t c = st[i] < 0 ? UI_DIM
                             : i < 3 ? (st[i] ? UI_WARN : UI_OK)
                             : i == 3 ? (st[i] ? (s.highBeam == 1 ? UI_LAMP_BLUE : UI_LAMP_GREEN) : UI_TEXT_DIM)
                             : (st[i] ? UI_WARN : UI_TEXT_DIM);
            char line[40];
            snprintf(line, sizeof line, "%s#%06X %s#", i ? "\n" : "", (unsigned)c, t);
            strlcat(all, line, sizeof all);
        }
        setText(v.vals, v.lastVals, sizeof v.lastVals, all);
    }
}

/* ═════════════════════════════ parking screen ════════════════════════════ */

/** The spokes' places on the wheel, clockwise from twelve o'clock [deg]; the
 *  last is the stripe at the top of the rim. */
static const float SPOKE_AT[4] = { -90, 90, 180, 0 };

/** Put line @p i of the wheel at its place with the wheel turned @p deg. */
static void parkLine(ParkView &v, int i, float deg) {
    const float a = (SPOKE_AT[i] - deg) * 0.017453293f;          // to radians
    const bool stripe = i == 3;
    const float r0 = stripe ? v.r - 16 : v.hub, r1 = stripe ? v.r + 2 : v.r - 4;
    const float s = sinf(a), c = cosf(a);
    const int x0 = (int)lroundf(v.cx + r0 * s), y0 = (int)lroundf(v.cy - r0 * c);
    const int x1 = (int)lroundf(v.cx + r1 * s), y1 = (int)lroundf(v.cy - r1 * c);
    // The object spans only the line, so a turn redraws only around it.
    const int pad = 6, ox = (x0 < x1 ? x0 : x1) - pad, oy = (y0 < y1 ? y0 : y1) - pad;
    lv_point_t *p = v.pts + 2 * i;
    p[0].x = (lv_coord_t)(x0 - ox); p[0].y = (lv_coord_t)(y0 - oy);
    p[1].x = (lv_coord_t)(x1 - ox); p[1].y = (lv_coord_t)(y1 - oy);
    lv_obj_set_pos(v.line[i], ox, oy);
    lv_line_set_points(v.line[i], p, 2);
}

void parkBuild(ParkView &v, lv_obj_t *parent, int w, int h, float steerMax, float straightDeg,
               bool pathView, const CarGeom &g) {
    v.steerMax = steerMax >= 30 ? steerMax : 540;
    v.straight = straightDeg >= 0 ? straightDeg : 10;
    const bool wide = w > h * 1.4f;
    if (pathView) {
        // The car from above and its path where the wheel would be; the angle
        // to its left (round) or right (landscape), as in the wheel view.
        v.pathView = true;
        const float ppm = wide ? (h - 16) / (CAR_LEN + 4.0f) : 20.0f;
        const int cx = wide ? (h - 48) / 2 + 40 : w / 2 + 12;
        const int axleY = (int)lroundf(h / 2 + (CAR_LEN / 2 - CAR_RO) * ppm);
        carBuild(v.car, parent, cx, axleY, ppm, 8, h - 8, g);
        v.rim = v.car.body;                       // built: parkUpdate's guard
        if (wide) {
            const int x = cx + (h - 48) / 2 + 36;
            label(parent, "STEERING", &lv_font_montserrat_12, UI_MUTED, LV_ALIGN_TOP_LEFT, x, h / 2 - 78);
            v.value = label(parent, "--", &lv_font_montserrat_48, UI_TEXT, LV_ALIGN_TOP_LEFT, x, h / 2 - 60);
            v.word  = label(parent, "", &lv_font_montserrat_20, UI_MUTED, LV_ALIGN_TOP_LEFT, x, h / 2 + 2);
        } else {
            v.value = label(parent, "--", &lv_font_montserrat_28, UI_TEXT, LV_ALIGN_TOP_LEFT, 14, h / 2 - 26);
            v.word  = label(parent, "", &lv_font_montserrat_12, UI_MUTED, LV_ALIGN_TOP_LEFT, 14, h / 2 + 8);
        }
        v.lastOk = true;
        parkUpdate(v, 0, false);
        return;
    }
    // Round: the wheel in the middle, the angle in it, the arc round it with
    // its gap at the bottom for the word. Landscape: wheel and arc on the
    // left, the angle and the word to their right.
    const int R = wide ? (h - 48) / 2 : (int)(w * 0.45f);     // the arc
    v.cx = wide ? R + 40 : w / 2;
    v.cy = wide ? h / 2 + 4 : h / 2 - 6;
    v.r  = wide ? (int)(R * 0.70f) : (int)(R * 0.69f);
    v.hub = wide ? 16 : 44;

    // From straight ahead (the top) to full lock either way: the arc's ends.
    v.lock = lv_arc_create(parent);
    lv_obj_remove_style(v.lock, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(v.lock, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(v.lock, 2 * R, 2 * R);
    lv_obj_set_pos(v.lock, v.cx - R, v.cy - R);
    lv_arc_set_bg_angles(v.lock, 135, 45);
    lv_arc_set_mode(v.lock, LV_ARC_MODE_SYMMETRICAL);
    const int16_t lim = (int16_t)lroundf(v.steerMax);
    lv_arc_set_range(v.lock, -lim, lim);
    lv_arc_set_value(v.lock, 0);
    lv_obj_set_style_arc_width(v.lock, 8, LV_PART_MAIN);
    lv_obj_set_style_arc_width(v.lock, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(v.lock, lv_color_hex(UI_LINE_SOFT), LV_PART_MAIN);
    lv_obj_set_style_arc_color(v.lock, lv_color_hex(UI_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(v.lock, false, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(v.lock, false, LV_PART_INDICATOR);
    lv_obj_set_style_pad_all(v.lock, 0, 0);
    lv_obj_set_style_bg_opa(v.lock, LV_OPA_TRANSP, 0);

    // The wheel: its rim, three spokes and the stripe at its top.
    v.rim = shape(parent, 2 * v.r, 2 * v.r);
    lv_obj_set_pos(v.rim, v.cx - v.r, v.cy - v.r);
    lv_obj_set_style_radius(v.rim, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(v.rim, wide ? 12 : 10, 0);
    lv_obj_set_style_border_color(v.rim, lv_color_hex(UI_LINE), 0);
    lv_obj_set_style_border_opa(v.rim, LV_OPA_COVER, 0);
    if (wide) {
        lv_obj_t *hub = shape(parent, 2 * v.hub, 2 * v.hub);
        lv_obj_set_pos(hub, v.cx - v.hub, v.cy - v.hub);
        lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(hub, lv_color_hex(UI_LINE), 0);
        lv_obj_set_style_bg_opa(hub, LV_OPA_COVER, 0);
    }
    v.pts = (lv_point_t *)lv_mem_alloc(sizeof(lv_point_t) * 8);   // for good, as the screen
    if (v.pts) memset(v.pts, 0, sizeof(lv_point_t) * 8);
    for (int i = 0; i < 4; i++) {
        lv_obj_t *l = lv_line_create(parent);
        lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_line_width(l, i == 3 ? 8 : (wide ? 10 : 8), 0);
        lv_obj_set_style_line_rounded(l, true, 0);
        lv_obj_set_style_line_color(l, lv_color_hex(i == 3 ? UI_ACCENT : UI_LINE), 0);
        v.line[i] = l;
        if (v.pts) parkLine(v, i, 0);
    }

    if (wide) {
        const int x = v.cx + R + 36;
        label(parent, "STEERING", &lv_font_montserrat_12, UI_MUTED, LV_ALIGN_TOP_LEFT, x, h / 2 - 78);
        v.value = label(parent, "--", &lv_font_montserrat_48, UI_TEXT, LV_ALIGN_TOP_LEFT, x, h / 2 - 60);
        v.word  = label(parent, "", &lv_font_montserrat_20, UI_MUTED, LV_ALIGN_TOP_LEFT, x, h / 2 + 2);
        char t[24];
        snprintf(t, sizeof t, "LOCK %d\xC2\xB0 EACH WAY", (int)lim);
        label(parent, t, &lv_font_montserrat_12, UI_DIM, LV_ALIGN_TOP_LEFT, x, h / 2 + 40);
    } else {
        v.value = label(parent, "--", &lv_font_montserrat_32, UI_TEXT, LV_ALIGN_CENTER, 0, v.cy - h / 2 - 2);
        v.word  = label(parent, "", &lv_font_montserrat_14, UI_MUTED, LV_ALIGN_TOP_MID, 0, v.cy + R - 12);
    }
    v.lastOk = true;
    parkUpdate(v, 0, false);
}

void parkUpdate(ParkView &v, float deg, bool ok, const CarState *car) {
    if (!v.rim) return;
    if (v.pathView) {
        CarState c = car ? *car : CarState();
        c.angleOk = ok;
        c.angle = deg;
        carUpdate(v.car, c);
    }
    if (ok != v.lastOk) {
        v.lastOk = ok;
        for (lv_obj_t *l : v.line) {
            if (!l) continue;
            if (ok) lv_obj_clear_flag(l, LV_OBJ_FLAG_HIDDEN);
            else    lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
        }
        if (!ok) {
            if (v.lock) lv_arc_set_value(v.lock, 0);
            setText(v.value, v.lastValue, sizeof v.lastValue, "--");
            setText(v.word, v.lastWord, sizeof v.lastWord, "NO STEERING ANGLE");
            lv_obj_set_style_text_color(v.word, lv_color_hex(UI_DIM), 0);
            lv_obj_set_style_text_color(v.value, lv_color_hex(UI_TEXT), 0);
            v.lastDeg = INT16_MIN;
            v.lastSide = 2;
        }
    }
    if (!ok) return;
    if (!(deg > -3600 && deg < 3600)) deg = deg > 0 ? 3600 : -3600;   // and NaN
    const int16_t d = (int16_t)lroundf(deg);
    if (d != v.lastDeg) {
        v.lastDeg = d;
        if (v.pts) for (int i = 0; i < 4; i++) parkLine(v, i, (float)d);
        const float lim = v.steerMax;
        if (v.lock) lv_arc_set_value(v.lock, (int16_t)lroundf(-d > lim ? lim : -d < -lim ? -lim : -d));
        char t[12];
        snprintf(t, sizeof t, "%d\xC2\xB0", d < 0 ? -d : d);
        setText(v.value, v.lastValue, sizeof v.lastValue, t);
    }
    const int8_t side = fabsf(deg) <= v.straight ? 0 : deg > 0 ? 1 : -1;
    if (side != v.lastSide) {
        v.lastSide = side;
        const uint32_t c = side ? UI_ACCENT : UI_OK;
        setText(v.word, v.lastWord, sizeof v.lastWord,
                side > 0 ? LV_SYMBOL_LEFT " LEFT" : side < 0 ? "RIGHT " LV_SYMBOL_RIGHT : "STRAIGHT");
        lv_obj_set_style_text_color(v.word, lv_color_hex(side ? UI_TEXT_DIM : UI_OK), 0);
        lv_obj_set_style_text_color(v.value, lv_color_hex(side ? UI_TEXT : UI_OK), 0);
        if (v.line[3]) lv_obj_set_style_line_color(v.line[3], lv_color_hex(c), 0);
        if (v.lock) lv_obj_set_style_arc_color(v.lock, lv_color_hex(c), LV_PART_INDICATOR);
        for (lv_obj_t *p : v.car.path)
            if (p) lv_obj_set_style_line_color(p, lv_color_hex(c), 0);
    }
}
