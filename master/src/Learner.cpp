/**
 * @file Learner.cpp
 * @brief Automatic discovery of broadcast CAN signals — see Learner.h.
 *
 * ## Breadth
 *
 * Every value OBD-II or SSM2 returns is a reference, not a fixed list: RPM
 * and speed, but also MAF, timing, every temperature, fuel rate, torque,
 * pedal and throttle channels, the ECU's switch inputs, the check-engine
 * lamp. Whatever the car also happens to broadcast gets found. Up to
 * NA_MAX values and NB_MAX switches are worked on at once; a slot is
 * recycled as soon as its value is learned or goes quiet - or, when another
 * value is moving and waiting for one, once its own value has sat still for
 * half a minute. Every value arrives continuously, so without that the first
 * twenty to turn up (barometric pressure, fuel level…) would hold the slots
 * for good and nothing that only moves later would ever be looked for.
 *
 * ## Memory
 *
 * The candidate tables are sized at boot from the heap that is actually
 * free, keeping a reserve for Wi-Fi and the portal. Candidates that have had
 * a fair chance and match nothing are evicted, so bytes that only start
 * moving later (speed, gear) always find room.
 *
 * ## Event-driven frames
 *
 * Many body frames are only sent when something changes. The last value of
 * every frame is held, so such a frame is compared with every reference
 * sample, not only the ones that happen to land right after it was sent.
 */
#include "Learner.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <math.h>

#include "MasterConfig.h"
#include "MasterPacket.h"
#include "MasterTelemetry.h"

namespace {

constexpr size_t NA_MAX      = 20;    /**< Analog values learned at once.  */
constexpr size_t NB_MAX      = 12;    /**< Switches learned at once.       */
constexpr size_t MAX_EX      = 48;    /**< Rejected (field, value) pairs.  */
constexpr size_t MAX_DEAD    = 512;   /**< Evicted candidates remembered.  */
constexpr size_t MAX_LEARNED = MAX_RT_SIGNALS;  /**< Metrics on the bus.   */
constexpr size_t MAX_WATCH   = 160;   /**< Values waiting for a slot.      */
constexpr uint32_t STILL_MS  = 30000; /**< Unmoving this long: slot freed. */
constexpr size_t HEAP_RESERVE = 95 * 1024;   /**< Left for Wi-Fi + portal. */

/* ─────────────────────────── what to require ─────────────────────────────
 *
 * "need" is how far a value must swing - up AND down, twice - before a fit
 * against it means anything; a field only correlates by coincidence over a
 * small or one-way range. Known quantities get a sensible figure; anything
 * else needs 10 % of its own magnitude.
 */
struct NeedDef { uint16_t metric; float need; };
const NeedDef NEEDS[] = {
    { METRIC_ID_RPM, 500 },          { METRIC_ID_SPEED, 20 },
    { METRIC_ID_THROTTLE, 15 },      { METRIC_ID_ACCEL_PEDAL_D, 15 },
    { METRIC_ID_ACCEL_PEDAL_E, 15 }, { METRIC_ID_ACCEL_PEDAL_F, 15 },
    { METRIC_ID_REL_PEDAL, 15 },     { METRIC_ID_REL_THROTTLE, 15 },
    { METRIC_ID_ABS_THROTTLE_B, 15 },{ METRIC_ID_ABS_THROTTLE_C, 15 },
    { METRIC_ID_CMD_THROTTLE, 15 },  { METRIC_ID_SSM_ACCEL_ANGLE, 15 },
    { METRIC_ID_ENGINE_LOAD, 15 },   { METRIC_ID_ABS_LOAD, 15 },
    { METRIC_ID_MAP, 15 },           { METRIC_ID_MAF, 10 },
    { METRIC_ID_TIMING_ADV, 6 },     { METRIC_ID_COOLANT_TEMP, 5 },
    { METRIC_ID_IAT, 4 },            { METRIC_ID_OIL_TEMP, 5 },
    { METRIC_ID_AMBIENT_TEMP, 3 },   { METRIC_ID_SSM_FUEL_TEMP, 3 },
    { METRIC_ID_FUEL_LEVEL, 3 },     { METRIC_ID_BATT_VOLTAGE, 0.6f },
    { METRIC_ID_GEAR, 1.5f },        { METRIC_ID_TORQUE_DEMAND, 15 },
    { METRIC_ID_TORQUE_ACTUAL, 15 }, { METRIC_ID_FUEL_RATE, 2 },
    { METRIC_ID_SSM_BRAKE_BOOST, 20 },{ METRIC_ID_BARO, 3 },
    { METRIC_ID_CAT_TEMP_B1S1, 60 }, { METRIC_ID_CAT_TEMP_B2S1, 60 },
};

float needFor(uint16_t m, float ymin, float ymax) {
    for (const auto &n : NEEDS) if (n.metric == m) return n.need;
    return max(1.0f, 0.1f * max(fabsf(ymin), fabsf(ymax)));
}

/** Values that make no sense to look for on the bus: counters that only
 *  climb, enumerations, our own housekeeping. */
bool learnable(uint16_t m) {
    if ((m & 0xFF00) == 0x1F00) return false;
    switch (m) {
        case METRIC_ID_RUN_TIME:   case METRIC_ID_DIST_MIL:    case METRIC_ID_WARMUPS:
        case METRIC_ID_DIST_CLEARED: case METRIC_ID_TIME_MIL:  case METRIC_ID_TIME_CLEARED:
        case METRIC_ID_FUEL_TYPE:  case METRIC_ID_DTC_COUNT:   case METRIC_ID_FUEL_SYS:
        case METRIC_ID_ODOMETER:   case METRIC_ID_SSM_ODOMETER: case METRIC_ID_TORQUE_REF:
        case METRIC_ID_NIGHT_SENSE: case METRIC_ID_SSM_ALT_MODE: case METRIC_ID_SSM_SI_DRIVE:
            return false;
        default: return true;
    }
}

bool isBitMetric(uint16_t m) { return (m & 0xFF00) == 0x2100 || m == METRIC_ID_MIL; }

/* ─────────────────────────────── state ─────────────────────────────────── */

/** @brief A field of a frame being tried, and its value when first seen
 *         (sums are centred on it for float precision). */
struct Cand    { uint32_t id; uint8_t start, len; bool be; float x0; uint8_t ci; };
struct BitCand { uint32_t id; uint8_t bit; uint8_t ci; };

/**
 * @brief Running regression of a value on a field, kept as means and
 *        co-moments (Welford) rather than raw sums.
 *
 * Raw sums lose the variance whenever the samples sit far from the point the
 * sums are centred on: a field first seen mid-ramp, then sampled 3800 counts
 * away, left float32 with 0.002 % of its sum to hold the whole variance - r2
 * came out at 1.0075, and a coarse field "fitted" better than the exact one
 * for ever. Means and co-moments are exact to float precision wherever the
 * samples sit. The weights halve as n grows, so old samples fade.
 *
 * @c levels has a bit per band of half the needed movement the value was
 * sampled in: a fit through two levels is always perfect, whatever the
 * field, so a match needs three.
 */
struct Acc     { float n, mx, my, m2x, m2y, cxy; uint32_t levels; };
/** @brief Bit counts: samples, agreements, reference on, bit on - enough for
 *         the phi coefficient - and the switch's flip count when this pair
 *         started, so a match must have seen the switch flip itself. */
struct BitAcc  { uint16_t n, agree, ones, xones, tog0; };

/** @brief One value being learned. metric 0 = free slot. */
struct Ref {
    uint16_t metric;
    uint32_t lastMs, samples;
    uint32_t allocMs, lastMoveMs;   /**< Slot given; value last moved.       */
    bool     init;
    float    y0, ymin, ymax, ext, prevY;
    int8_t   dir;
    uint8_t  swings;
    bool     lastBit;
    uint16_t toggles, ones;
    uint8_t  state;
    float    bestFit;
    int16_t  best;
    uint16_t bestN;
};

struct Excl { uint32_t id; uint8_t start, len; bool be; uint16_t metric; };
struct Dead { uint32_t id; uint8_t start, len; bool be; };
struct Sample { uint16_t metric; float value; };
/** @brief A value without a slot: just enough to tell whether it moves. */
struct Watch { uint16_t metric; bool init, lastBit; float lo, hi; uint32_t seenMs, movedMs; };
/** @brief A portal request, run on the learner task. */
struct Cmd { uint8_t op; uint16_t metric; };
enum : uint8_t { CMD_ACCEPT = 1, CMD_UNLEARN = 2 };

QueueHandle_t s_q = nullptr, s_cmdQ = nullptr;
size_t   s_maxC = 0, s_maxB = 0;
Cand    *s_c = nullptr;  Acc    *s_acc  = nullptr;  size_t s_nc = 0;
BitCand *s_b = nullptr;  BitAcc *s_bacc = nullptr;  size_t s_nb = 0;
Ref      s_ar[NA_MAX] = {}, s_br[NB_MAX] = {};
Excl     s_ex[MAX_EX];       size_t s_nex = 0, s_exHead = 0;
Dead     s_dead[MAX_DEAD];   size_t s_ndead = 0, s_deadHead = 0;
Watch    s_watch[MAX_WATCH]; size_t s_nWatch = 0;
uint16_t s_learned[MAX_LEARNED]; volatile size_t s_nLearned = 0;
volatile bool s_resetReq = false;
bool s_fullLogged = false, s_tableFullLogged = false;

/*
 * One-sample look-ahead.
 *
 * A requested value is always some tens of milliseconds older than the frame
 * beside it. For a value that moves smoothly the steadiness test takes care of
 * that, but a value that changes in a step - a gear, every switch - has one
 * sample at each change where the frame already shows the new state and the
 * reference still shows the old one, and nothing about the reference gives it
 * away. Those samples alone held the right field below the match threshold.
 *
 * So a sample is only counted once the NEXT reading shows the value did not
 * change around it; the one right before a change is dropped. That covers the
 * lag whenever the polling interval is longer than the reply takes, which it
 * is for both engines. Holding a sample means holding each field's value as
 * it was then. A switch drops the one right after a flip as well (settled):
 * there the frame can still show the old state, and whether a flip costs the
 * sample before or after it depends on how frame and reading line up.
 */
struct Pend { bool valid; bool yb; float yc; uint32_t band; bool settled; };
Pend     s_apend[NA_MAX] = {}, s_bpend[NB_MAX] = {};
float   *s_px  = nullptr;   /**< [cand * NA_MAX + slot]: field, NAN = frame missing. */
uint8_t *s_pbx = nullptr;   /**< [slot * s_bBytes + bit/8]: bit value held.       */
uint8_t *s_pbv = nullptr;   /**< ...and whether it was there to hold.             */
size_t   s_bBytes = 0;

inline bool getBit(const uint8_t *m, size_t i) { return (m[i >> 3] >> (i & 7)) & 1; }
inline void putBit(uint8_t *m, size_t i, bool v) {
    if (v) m[i >> 3] |= (uint8_t)(1 << (i & 7)); else m[i >> 3] &= (uint8_t)~(1 << (i & 7));
}

bool alreadyLearned(uint16_t m) {
    for (size_t i = 0; i < s_nLearned; i++) if (s_learned[i] == m) return true;
    return false;
}

int findSlot(Ref *arr, size_t n, uint16_t m) {
    for (size_t i = 0; i < n; i++) if (arr[i].metric == m) return (int)i;
    return -1;
}

/**
 * @brief Give a value a slot, recycling one that is learned, hopeless or
 *        quiet - or, for a value that is moving, one whose own value has
 *        not moved for STILL_MS. Its column of sums is cleared for the
 *        newcomer.
 */
int allocSlot(bool bit, uint16_t m, bool moving) {
    Ref *arr = bit ? s_br : s_ar;
    const size_t n = bit ? NB_MAX : NA_MAX;
    int victim = -1;
    uint32_t worst = 0;
    const uint32_t now = millis();
    for (size_t i = 0; i < n; i++) {
        if (!arr[i].metric) { victim = (int)i; break; }
        const Ref &r = arr[i];
        const uint32_t idle = now - r.lastMs, still = now - r.lastMoveMs;
        bool recyclable = r.state == LS_LEARNED || idle > 60000 ||
                          (r.state == LS_NOT_FOUND && r.samples > 4000);
        if (!recyclable && moving && (r.state == LS_NEED_MOVE || r.state == LS_WAIT_REF) &&
            now - r.allocMs > STILL_MS && still > STILL_MS)
            recyclable = true;
        const uint32_t score = max(idle, still);
        if (recyclable && score >= worst) { worst = score; victim = (int)i; }
    }
    if (victim < 0) return -1;
    memset(&arr[victim], 0, sizeof(Ref));
    arr[victim].metric = m;
    arr[victim].best = -1;
    arr[victim].allocMs = arr[victim].lastMoveMs = now;
    (bit ? s_bpend : s_apend)[victim].valid = false;      // the old value's sample
    if (bit) for (size_t i = 0; i < s_nb; i++) memset(&s_bacc[i * NB_MAX + victim], 0, sizeof(BitAcc));
    else     for (size_t i = 0; i < s_nc; i++) memset(&s_acc[i * NA_MAX + victim], 0, sizeof(Acc));
    return victim;
}

/**
 * @brief Follow a value that has no slot, cheaply, to see whether it moves.
 * @return true when it moved (half its need, or a switch flipped) in the
 *         last minute - the claim it needs to take a slot from a still value.
 */
bool watchMoving(uint16_t m, bool bit, float y) {
    const uint32_t now = millis();
    Watch *w = nullptr;
    for (size_t i = 0; i < s_nWatch; i++)
        if (s_watch[i].metric == m) { w = &s_watch[i]; break; }
    if (!w) {
        if (s_nWatch < MAX_WATCH) w = &s_watch[s_nWatch++];
        else {                               // replace the longest unseen
            size_t v = 0;
            for (size_t i = 1; i < MAX_WATCH; i++)
                if (now - s_watch[i].seenMs > now - s_watch[v].seenMs) v = i;
            w = &s_watch[v];
        }
        memset(w, 0, sizeof(*w));
        w->metric = m;
    }
    w->seenMs = now;
    if (!w->init) { w->init = true; w->lo = w->hi = y; w->lastBit = y > 0.5f; return false; }
    if (bit) {
        const bool yb = y > 0.5f;
        if (yb != w->lastBit) { w->lastBit = yb; w->movedMs = now; }
    } else {
        w->lo = min(w->lo, y); w->hi = max(w->hi, y);
        if (w->hi - w->lo >= needFor(m, w->lo, w->hi) * 0.5f) { w->movedMs = now; w->lo = w->hi = y; }
    }
    return w->movedMs && now - w->movedMs < 60000;
}

/** @brief Never propose this field for this value again (a ring: the oldest
 *         exclusion goes when it is full). */
void exclude(uint32_t id, uint8_t start, uint8_t len, bool be, uint16_t metric) {
    s_ex[s_exHead] = { id, start, len, be, metric };
    s_exHead = (s_exHead + 1) % MAX_EX;
    if (s_nex < MAX_EX) s_nex++;
}

const CensusView *lookup(const CensusView *cen, size_t n, uint32_t id, uint8_t &ci) {
    if (ci < n && cen[ci].id == id && !cen[ci].extd) return &cen[ci];
    for (size_t i = 0; i < n; i++)
        if (cen[i].id == id && !cen[i].extd) { ci = (uint8_t)i; return &cen[i]; }
    return nullptr;
}

bool excluded(uint32_t id, uint8_t start, uint8_t len, bool be, uint16_t metric) {
    for (size_t i = 0; i < s_nex; i++)
        if (s_ex[i].id == id && s_ex[i].start == start && s_ex[i].len == len &&
            s_ex[i].be == be && s_ex[i].metric == metric) return true;
    return false;
}

bool isDead(uint32_t id, uint8_t start, uint8_t len, bool be) {
    for (size_t i = 0; i < s_ndead; i++)
        if (s_dead[i].id == id && s_dead[i].start == start && s_dead[i].len == len &&
            s_dead[i].be == be) return true;
    return false;
}

void markDead(uint32_t id, uint8_t start, uint8_t len, bool be) {
    s_dead[s_deadHead] = { id, start, len, be };
    s_deadHead = (s_deadHead + 1) % MAX_DEAD;
    if (s_ndead < MAX_DEAD) s_ndead++;
}

void resetAll() {
    s_nc = s_nb = 0;
    memset(s_ar, 0, sizeof(s_ar));
    memset(s_br, 0, sizeof(s_br));
    memset(s_apend, 0, sizeof(s_apend));
    memset(s_bpend, 0, sizeof(s_bpend));
    s_nex = s_exHead = s_ndead = s_deadHead = s_nWatch = 0;
    s_fullLogged = s_tableFullLogged = false;
}

/* ─────────────────────────── candidate discovery ───────────────────────── */

void addCand(const CensusView &e, uint8_t ci, uint8_t start, uint8_t len, bool be) {
    if (isDead(e.id, start, len, be)) return;
    for (size_t i = 0; i < s_nc; i++)
        if (s_c[i].id == e.id && s_c[i].start == start && s_c[i].len == len && s_c[i].be == be)
            return;
    if (s_nc >= s_maxC) {
        if (!s_fullLogged) { s_fullLogged = true; log_i("learner: field table full, evicting as it goes"); }
        return;
    }
    s_c[s_nc] = { e.id, start, len, be, (float)masterExtract(e.data, e.dlc, start, len, be), ci };
    memset(&s_acc[s_nc * NA_MAX], 0, sizeof(Acc) * NA_MAX);
    for (size_t ai = 0; ai < NA_MAX; ai++) s_px[s_nc * NA_MAX + ai] = NAN;  // nothing held yet
    s_nc++;
}

void addBit(const CensusView &e, uint8_t ci, uint8_t bit) {
    if (isDead(e.id, bit, 1, false)) return;
    for (size_t i = 0; i < s_nb; i++)
        if (s_b[i].id == e.id && s_b[i].bit == bit) return;
    if (s_nb >= s_maxB) return;
    s_b[s_nb] = { e.id, bit, ci };
    memset(&s_bacc[s_nb * NB_MAX], 0, sizeof(BitAcc) * NB_MAX);
    for (size_t bi = 0; bi < NB_MAX; bi++) putBit(&s_pbv[bi * s_bBytes], s_nb, false);
    s_nb++;
}

/**
 * @brief Turn every byte that has ever changed into candidate fields.
 *
 * Runs every two seconds, so a byte that only starts moving once the car
 * does joins the moment it first changes. 8-bit fields at every byte, 16-bit
 * Intel and Motorola pairs, and 12/14-bit variants where status flags share
 * the top of a byte.
 */
void discover(const CensusView *cen, size_t n) {
    for (size_t i = 0; i < n; i++) {
        const CensusView &e = cen[i];
        if (e.extd || e.dlc == 0 || e.count < 5 || e.ageMs > 10000) continue;
        if (e.id >= 0x7DF && e.id <= 0x7EF) continue;        // diagnostic replies
        const uint8_t ci = (uint8_t)i;
        for (uint8_t k = 0; k < e.dlc; k++) {
            const uint8_t ch = e.changed[k];
            if (!ch) continue;
            for (uint8_t b = 0; b < 8; b++)
                if (ch & (1 << b)) addBit(e, ci, 8 * k + b);
            // A single changing bit is a flag, not a value.
            if ((ch & (ch - 1)) == 0 && !(k + 1 < e.dlc && e.changed[k + 1])) continue;
            addCand(e, ci, 8 * k, 8, false);
            if (k + 1 < e.dlc && e.changed[k + 1]) {
                const uint8_t hi = e.changed[k + 1];
                addCand(e, ci, 8 * k, 16, false);
                if ((hi & 0xC0) && (hi & 0x3F)) addCand(e, ci, 8 * k, 14, false);
                if ((hi & 0xF0) && (hi & 0x0F)) addCand(e, ci, 8 * k, 12, false);
                addCand(e, ci, 8 * k + 7, 16, true);
                if ((ch & 0xC0) && (ch & 0x3F)) addCand(e, ci, 8 * k + 5, 14, true);
                if ((ch & 0xF0) && (ch & 0x0F)) addCand(e, ci, 8 * k + 3, 12, true);
            }
        }
    }
}

/* ─────────────────────────────── sampling ──────────────────────────────── */

void trackSwing(Ref &r, float y) {
    if (!r.init) {
        r.init = true; r.y0 = r.ymin = r.ymax = r.ext = r.prevY = y; r.dir = 0;
        return;
    }
    if (y < r.ymin || y > r.ymax) r.lastMoveMs = millis();
    r.ymin = min(r.ymin, y); r.ymax = max(r.ymax, y);
    const float h = needFor(r.metric, r.ymin, r.ymax) * 0.5f;
    const uint8_t swings = r.swings;
    if (r.dir == 0) {
        if (y >= r.ext + h)      { r.dir =  1; r.ext = y; }
        else if (y <= r.ext - h) { r.dir = -1; r.ext = y; }
    } else if (r.dir > 0) {
        if (y > r.ext) r.ext = y;
        else if (y <= r.ext - h) { r.dir = -1; r.ext = y; if (r.swings < 255) r.swings++; }
    } else {
        if (y < r.ext) r.ext = y;
        else if (y >= r.ext + h) { r.dir =  1; r.ext = y; if (r.swings < 255) r.swings++; }
    }
    if (r.swings != swings) r.lastMoveMs = millis();
}

/** @brief Count the held analog sample of slot @p ai into its sums. */
void commitAnalog(size_t ai) {
    const Pend &p = s_apend[ai];
    for (size_t i = 0; i < s_nc; i++) {
        const float x = s_px[i * NA_MAX + ai];
        if (isnan(x)) continue;                       // frame was not there
        Acc &a = s_acc[i * NA_MAX + ai];
        a.n += 1;
        const float dx = x - a.mx, dy = p.yc - a.my;
        a.mx += dx / a.n;
        a.my += dy / a.n;
        a.m2x += dx * (x - a.mx);
        a.m2y += dy * (p.yc - a.my);
        a.cxy += dx * (p.yc - a.my);
        a.levels |= p.band;
        if (a.n >= 2048) { a.n *= 0.5f; a.m2x *= 0.5f; a.m2y *= 0.5f; a.cxy *= 0.5f; }
    }
}

void sampleAnalog(size_t ai, float y, const CensusView *cen, size_t n) {
    Ref &r = s_ar[ai];
    const bool first = !r.init;
    trackSwing(r, y);
    r.samples++;
    r.lastMs = millis();
    /*
     * Fit on steady samples only. A requested value is tens of milliseconds
     * older than the broadcast frame beside it; while it moves fast that lag
     * alone scatters the fit. Holding still at a few levels gives clean
     * samples, and the movement still counts toward the swing test. Steady
     * on both sides: the held sample counts only if this one did not move
     * away from it (see Pend).
     */
    const float need = needFor(r.metric, r.ymin, r.ymax);
    const bool steady = first || fabsf(y - r.prevY) <= need * Cfg.learnSteady;
    r.prevY = y;
    Pend &p = s_apend[ai];
    if (p.valid && steady) commitAnalog(ai);
    p.valid = false;
    if (!steady) return;

    // Hold this one until the next reading vouches for it.
    p.valid = true;
    p.yc    = y - r.y0;
    p.band  = 1u << ((uint32_t)(int32_t)floorf(p.yc / (need * 0.5f)) & 31);
    for (size_t i = 0; i < s_nc; i++) {
        Cand &c = s_c[i];
        const CensusView *e = lookup(cen, n, c.id, c.ci);
        s_px[i * NA_MAX + ai] = (!e || e->ageMs > 5000)      // frame gone; held otherwise
            ? NAN : (float)masterExtract(e->data, e->dlc, c.start, c.len, c.be) - c.x0;
    }
}

/** @brief Count the held switch sample of slot @p bi into its counts. */
void commitBits(size_t bi, bool yb) {
    const Ref &r = s_br[bi];
    const uint8_t *vx = &s_pbx[bi * s_bBytes], *vv = &s_pbv[bi * s_bBytes];
    for (size_t i = 0; i < s_nb; i++) {
        if (!getBit(vv, i)) continue;
        const bool xb = getBit(vx, i);
        BitAcc &a = s_bacc[i * NB_MAX + bi];
        if (a.n == 0) a.tog0 = r.toggles;
        a.n++; if (xb == yb) a.agree++; if (yb) a.ones++; if (xb) a.xones++;
        if (a.n >= 60000) { a.n /= 2; a.agree /= 2; a.ones /= 2; a.xones /= 2; }
    }
}

void sampleBit(size_t bi, float y, const CensusView *cen, size_t n) {
    Ref &r = s_br[bi];
    const bool yb = y > 0.5f;
    const bool same = r.init && yb == r.lastBit;
    if (r.init && !same) {
        if (r.toggles < 65535) r.toggles++;
        r.lastMoveMs = millis();
    }
    r.init = true; r.lastBit = yb;
    r.samples++;
    if (yb && r.ones < 65535) r.ones++;
    r.lastMs = millis();

    // The held sample counts only if the switch held its state on both sides
    // of it: the one right before a flip may pair the old state with a frame
    // that already shows the new one, and the one right after it the new
    // state with a frame that still shows the old - the frame and the ECU's
    // reading are never taken at the same moment (see Pend). Which of the two
    // a flip costs depends on how they happen to line up, so both are left
    // out rather than one.
    Pend &p = s_bpend[bi];
    if (p.valid && p.settled && same) commitBits(bi, p.yb);
    p.valid   = true;
    p.yb      = yb;
    p.settled = same;
    uint8_t *vx = &s_pbx[bi * s_bBytes], *vv = &s_pbv[bi * s_bBytes];
    for (size_t i = 0; i < s_nb; i++) {
        BitCand &b = s_b[i];
        const CensusView *e = lookup(cen, n, b.id, b.ci);
        const bool ok = e && e->ageMs <= 5000 && b.bit / 8 < e->dlc;
        putBit(vv, i, ok);
        putBit(vx, i, ok && ((e->data[b.bit / 8] >> (b.bit % 8)) & 1));
    }
}

/* ─────────────────────────────── decisions ─────────────────────────────── */

/** @brief Regression of the value on the field: r2, slope and intercept (in
 *         the centred units the sums use). @p minN samples at least. */
bool fit(const Acc &a, double &r2, double &k, double &cc, float minN) {
    if (a.n < minN) return false;
    const double vx = a.m2x, vy = a.m2y, cv = a.cxy;
    if (vx <= 1e-9 || vy <= 1e-9) return false;
    r2 = min(1.0, cv * cv / (vx * vy));      // rounding can never make it better than perfect
    k  = cv / vx;
    cc = a.my - k * a.mx;
    return true;
}
bool fit(const Acc &a, double &r2, double &k, double &cc) { return fit(a, r2, k, cc, Cfg.learnMinN); }

/** @brief Phi coefficient: +1 the bit follows the switch, -1 is its inverse,
 *         0 unrelated. Not fooled by a mostly-off switch and a stuck bit. */
float phi(const BitAcc &a) {
    const float n = a.n, n11 = (a.agree + a.ones + a.xones - n) * 0.5f;
    const float n1x = a.xones, n0x = n - a.xones, ny1 = a.ones, ny0 = n - a.ones;
    const float den = sqrtf(n1x * n0x * ny1 * ny0);
    return den > 0 ? (n11 * n - n1x * ny1) / den : 0.0f;
}

bool moved(const Ref &r) {
    return r.metric && r.ymax - r.ymin >= needFor(r.metric, r.ymin, r.ymax) && r.swings >= 2;
}

/** @brief Free the slots of candidates that had a fair chance and match
 *         nothing, so later-moving bytes always find room. */
void evict() {
    for (size_t i = 0; i < s_nc;) {
        bool tested = false, promising = false;
        for (size_t ai = 0; ai < NA_MAX; ai++) {
            if (!moved(s_ar[ai])) continue;
            const Acc &a = s_acc[i * NA_MAX + ai];
            if (a.n < 150) { promising = true; continue; }
            tested = true;
            double r2, k, cc;
            if (fit(a, r2, k, cc) && r2 >= 0.25) promising = true;
        }
        if (tested && !promising) {
            markDead(s_c[i].id, s_c[i].start, s_c[i].len, s_c[i].be);
            const size_t last = --s_nc;
            if (i != last) {
                s_c[i] = s_c[last];
                memcpy(&s_acc[i * NA_MAX], &s_acc[last * NA_MAX], sizeof(Acc) * NA_MAX);
                memcpy(&s_px[i * NA_MAX], &s_px[last * NA_MAX], sizeof(float) * NA_MAX);
            }
            continue;
        }
        i++;
    }
    for (size_t i = 0; i < s_nb;) {
        bool tested = false, promising = false;
        for (size_t bi = 0; bi < NB_MAX; bi++) {
            if (!s_br[bi].metric || s_br[bi].toggles < 3) continue;
            const BitAcc &a = s_bacc[i * NB_MAX + bi];
            if (a.n < 150) { promising = true; continue; }
            tested = true;
            if (fabsf(phi(a)) >= 0.3f) promising = true;
        }
        if (tested && !promising) {
            markDead(s_b[i].id, s_b[i].bit, 1, false);
            const size_t last = --s_nb;
            if (i != last) {
                s_b[i] = s_b[last];
                memcpy(&s_bacc[i * NB_MAX], &s_bacc[last * NB_MAX], sizeof(BitAcc) * NB_MAX);
                for (size_t bi = 0; bi < NB_MAX; bi++) {
                    uint8_t *vx = &s_pbx[bi * s_bBytes], *vv = &s_pbv[bi * s_bBytes];
                    putBit(vx, i, getBit(vx, last));
                    putBit(vv, i, getBit(vv, last));
                }
            }
            continue;
        }
        i++;
    }
}

/** @return false when the signal table is full (said once in the log). */
bool propose(uint16_t metric, uint32_t id, uint8_t start, uint8_t len, bool be,
             float scale, float offset, float vtol, float vspread,
             uint8_t mode = SIG_AUTO) {
    bool added = false;
    Cfg.lock();
    if (Cfg.signals.size() < MAX_RT_SIGNALS) {
        RtSignal s;
        s.canId = id; s.extended = false; s.startBit = start; s.bitLength = len;
        s.bigEndian = be; s.isSigned = false; s.scale = scale; s.offset = offset;
        s.metricId = metric; s.mode = mode; s.refMetric = metric; s.learned = true;
        s.vtol = vtol; s.vspread = vspread;
        snprintf(s.name, sizeof(s.name), "learned %04X", metric);
        Cfg.signals.push_back(s);
        Cfg.generation++;
        Cfg.requestSave();
        added = true;
    }
    Cfg.unlock();
    if (!added && !s_tableFullLogged) {
        s_tableFullLogged = true;
        log_w("learner: the signal table is full (%u) - remove signals to learn more",
              (unsigned)MAX_RT_SIGNALS);
    }
    return added;
}

/** @brief Read back what is on the bus already, what is being confirmed,
 *         and what the verifier rejected (remembered, and removed). */
void syncWithSignals(uint16_t *pending, size_t &nPending) {
    nPending = 0;
    size_t nl = 0;
    Cfg.lock();
    bool changed = false;
    for (size_t i = 0; i < Cfg.signals.size();) {
        const RtSignal &s = Cfg.signals[i];
        if (s.publishes()) { if (nl < MAX_LEARNED) s_learned[nl++] = s.metricId; }
        else if (s.mode == SIG_AUTO && s.learned) { if (nPending < MAX_RT_SIGNALS) pending[nPending++] = s.metricId; }
        else if (s.mode == SIG_REJECTED && s.learned) {
            exclude(s.canId, s.startBit, s.bitLength, s.bigEndian, s.metricId);
            log_w("learner: 0x%04X at 0x%03X was rejected by the verifier - searching again",
                  s.metricId, s.canId);
            Cfg.signals.erase(Cfg.signals.begin() + i);
            changed = true;
            continue;
        }
        i++;
    }
    s_nLearned = nl;
    if (changed) { Cfg.generation++; Cfg.requestSave(); }
    Cfg.unlock();
}

bool contains(const uint16_t *a, size_t n, uint16_t m) {
    for (size_t i = 0; i < n; i++) if (a[i] == m) return true;
    return false;
}

void evaluate() {
    static uint16_t pending[MAX_RT_SIGNALS]; size_t nPending;
    syncWithSignals(pending, nPending);
    evict();
    const uint32_t now = millis();

    for (size_t ai = 0; ai < NA_MAX; ai++) {
        Ref &r = s_ar[ai];
        if (!r.metric) continue;
        uint8_t st;
        r.best = -1; r.bestFit = 0; r.bestN = 0;
        if (alreadyLearned(r.metric))                   st = LS_LEARNED;
        else if (contains(pending, nPending, r.metric)) st = LS_CONFIRM;
        else if (!r.samples || now - r.lastMs > 15000)  st = LS_WAIT_REF;
        else if (!moved(r))                             st = LS_NEED_MOVE;
        else                                            st = LS_SEARCHING;

        double bk = 0, bcc = 0, br2 = 0; int best = -1;
        const float need = needFor(r.metric, r.ymin, r.ymax);
        for (size_t i = 0; i < s_nc; i++) {
            const Cand &c = s_c[i];
            if (excluded(c.id, c.start, c.len, c.be, r.metric)) continue;
            const Acc &a = s_acc[i * NA_MAX + ai];
            double r2, k, cc;
            if (!fit(a, r2, k, cc)) continue;
            /*
             * Two guards against a coincidence. The pair must have seen the
             * value at three levels at least - through two, any field that
             * happened to change at the same moment fits perfectly. And one
             * step of the field must be worth less than the movement asked of
             * the value: a byte whose one step is 1600 rpm is not the RPM.
             */
            if (__builtin_popcount(a.levels) < 3 || fabs(k) > need) continue;
            // A wider field wins only if it halves the residual: the high byte
            // of a 16-bit value also fits well, but coarsely.
            if (best < 0 || (1.0 - r2) < (1.0 - br2) * 0.5) { best = (int)i; br2 = r2; bk = k; bcc = cc; }
        }
        if (best >= 0) { r.best = best; r.bestFit = (float)br2; r.bestN = (uint16_t)s_acc[best * NA_MAX + ai].n; }

        if (st == LS_SEARCHING && best >= 0 && br2 >= Cfg.learnR2) {
            const Cand &c = s_c[best];
            const float range = r.ymax - r.ymin;
            if (propose(r.metric, c.id, c.start, c.len, c.be, (float)bk,
                        (float)(r.y0 + bcc - bk * c.x0),
                        max(need * 0.2f, 0.03f * range), need)) {
                log_i("learner: 0x%04X found in 0x%03X start %u len %u %s (r2 %.4f, n %u) - confirming",
                      r.metric, c.id, c.start, c.len, c.be ? "BE" : "LE", br2, r.bestN);
                st = LS_CONFIRM;
            }
        } else if (st == LS_SEARCHING && r.samples > 1500 && r.swings >= 6) {
            st = LS_NOT_FOUND;
        }
        r.state = st;
    }

    for (size_t bi = 0; bi < NB_MAX; bi++) {
        Ref &r = s_br[bi];
        if (!r.metric) continue;
        uint8_t st;
        r.best = -1; r.bestFit = 0; r.bestN = 0;
        const bool mv = r.toggles >= 3 && r.ones >= 8 && r.samples - r.ones >= 8;
        if (alreadyLearned(r.metric))                   st = LS_LEARNED;
        else if (contains(pending, nPending, r.metric)) st = LS_CONFIRM;
        else if (!r.samples || now - r.lastMs > 15000)  st = LS_WAIT_REF;
        else if (!mv)                                   st = LS_NEED_MOVE;
        else                                            st = LS_SEARCHING;

        int best = -1; float bq = 0; bool binv = false;
        for (size_t i = 0; i < s_nb; i++) {
            const BitAcc &a = s_bacc[i * NB_MAX + bi];
            if (a.n < Cfg.learnMinN || a.ones < 8 || a.n - a.ones < 8) continue;
            // The switch must have flipped three times while this bit was
            // watched: one flip that a status bit happened to share is not a
            // match.
            if ((uint16_t)(r.toggles - a.tog0) < 3) continue;
            if (excluded(s_b[i].id, s_b[i].bit, 1, false, r.metric)) continue;
            const float p = phi(a);
            if (fabsf(p) > bq) { bq = fabsf(p); best = (int)i; binv = p < 0; }
        }
        if (best >= 0) { r.best = best; r.bestFit = bq; r.bestN = s_bacc[best * NB_MAX + bi].n; }
        if (st == LS_SEARCHING && best >= 0 && bq >= Cfg.learnPhi) {
            const BitCand &b = s_b[best];
            if (propose(r.metric, b.id, b.bit, 1, false, binv ? -1.0f : 1.0f, binv ? 1.0f : 0.0f,
                        0.5f, 1.0f)) {
                log_i("learner: 0x%04X found at 0x%03X bit %u%s (phi %.3f) - confirming",
                      r.metric, b.id, b.bit, binv ? " inverted" : "", bq);
                st = LS_CONFIRM;
            }
        } else if (st == LS_SEARCHING && r.toggles >= 12) {
            st = LS_NOT_FOUND;
        }
        r.state = st;
    }
}

/**
 * @brief "Listen now": publish the best match so far, confirmed or not.
 *
 * If the verifier is already checking a learned signal for this value, that
 * one is simply switched on. Otherwise the best-fitting field is added as a
 * live signal straight away - the user has decided it is good enough.
 */
void doAccept(uint16_t m) {
    Cfg.lock();
    for (const auto &s : Cfg.signals)
        if (s.metricId == m && s.publishes()) {        // a second one would only fight it
            Cfg.unlock();
            log_i("learner: 0x%04X is already read from the bus", m);
            return;
        }
    for (auto &s : Cfg.signals)
        if (s.metricId == m && s.learned && s.mode == SIG_AUTO) {
            s.mode = SIG_ON;
            Cfg.generation++;
            Cfg.requestSave();
            Cfg.unlock();
            log_i("learner: 0x%04X accepted by the user (was being confirmed)", m);
            return;
        }
    Cfg.unlock();

    const bool bit = isBitMetric(m);
    const int slot = findSlot(bit ? s_br : s_ar, bit ? NB_MAX : NA_MAX, m);
    if (slot < 0) { log_w("learner: 0x%04X has no learning data to accept", m); return; }
    if (bit) {
        int best = -1; float bq = 0; bool inv = false;
        for (size_t i = 0; i < s_nb; i++) {
            const BitAcc &a = s_bacc[i * NB_MAX + slot];
            if (a.n < 10 || !a.ones || a.ones == a.n) continue;
            const float p = phi(a);
            if (fabsf(p) > bq) { bq = fabsf(p); best = (int)i; inv = p < 0; }
        }
        if (best < 0) { log_w("learner: 0x%04X - no candidate bit yet", m); return; }
        if (propose(m, s_b[best].id, s_b[best].bit, 1, false, inv ? -1.0f : 1.0f,
                    inv ? 1.0f : 0.0f, 0.5f, 1.0f, SIG_ON))
            log_i("learner: 0x%04X accepted by the user: 0x%03X bit %u (phi %.2f)",
                  m, s_b[best].id, s_b[best].bit, bq);
        return;
    }
    const Ref &r = s_ar[slot];
    double bk = 0, bcc = 0, br2 = 0; int best = -1;
    for (size_t i = 0; i < s_nc; i++) {
        const Cand &c = s_c[i];
        if (excluded(c.id, c.start, c.len, c.be, m)) continue;
        double r2, k, cc;
        if (!fit(s_acc[i * NA_MAX + slot], r2, k, cc, 10)) continue;   // the user decides: 10 is enough
        if (best < 0 || (1.0 - r2) < (1.0 - br2) * 0.5) {
            best = (int)i; br2 = r2; bk = k; bcc = cc;
        }
    }
    if (best < 0) { log_w("learner: 0x%04X - no candidate field yet", m); return; }
    const Cand &c = s_c[best];
    const float range = r.ymax - r.ymin, need = needFor(m, r.ymin, r.ymax);
    if (propose(m, c.id, c.start, c.len, c.be, (float)bk, (float)(r.y0 + bcc - bk * c.x0),
                max(need * 0.2f, 0.03f * range), need, SIG_ON))
        log_i("learner: 0x%04X accepted by the user: 0x%03X start %u len %u (r2 %.3f)",
              m, c.id, c.start, c.len, br2);
}

/**
 * @brief "Request instead": drop the learned signal and never propose its
 *        field for this value again. A signal mapped by hand is switched off
 *        rather than deleted, so the mapping is not lost.
 */
void doUnlearn(uint16_t m) {
    Cfg.lock();
    for (size_t i = 0; i < Cfg.signals.size();) {
        RtSignal &s = Cfg.signals[i];
        if (s.metricId == m && s.learned) {
            exclude(s.canId, s.startBit, s.bitLength, s.bigEndian, m);
            Cfg.signals.erase(Cfg.signals.begin() + i);
            continue;
        }
        if (s.metricId == m && s.publishes()) s.mode = SIG_OFF;
        i++;
    }
    Cfg.generation++;
    Cfg.requestSave();
    Cfg.unlock();
    for (size_t i = 0; i < s_nLearned; i++)
        if (s_learned[i] == m) s_learned[i] = 0;
    log_i("learner: 0x%04X returned to requests by the user", m);
}

/* ─────────────────────────────────── task ──────────────────────────────── */

void task(void *) {
    static CensusView cen[128];
    Sample buf[64];
    uint32_t lastDisc = 0, lastEval = 0;

    for (;;) {
        size_t n = 0;
        if (xQueueReceive(s_q, &buf[0], pdMS_TO_TICKS(250)) == pdTRUE) {
            n = 1;
            while (n < 64 && xQueueReceive(s_q, &buf[n], 0) == pdTRUE) n++;
        }
        if (s_resetReq) { s_resetReq = false; resetAll(); }
        Cmd cmd;
        while (xQueueReceive(s_cmdQ, &cmd, 0) == pdTRUE) {
            if (cmd.op == CMD_ACCEPT) doAccept(cmd.metric);
            else                      doUnlearn(cmd.metric);
        }
        if (!Cfg.learnEnabled) continue;

        const uint32_t now = millis();
        const size_t nc = masterCensusRaw(cen, 128);
        if (now - lastDisc >= 2000) { lastDisc = now; discover(cen, nc); }

        for (size_t i = 0; i < n; i++) {
            const uint16_t m = buf[i].metric;
            const bool bit = isBitMetric(m);
            int slot = findSlot(bit ? s_br : s_ar, bit ? NB_MAX : NA_MAX, m);
            if (slot < 0) slot = allocSlot(bit, m, watchMoving(m, bit, buf[i].value));
            if (slot < 0) continue;                 // all slots busy; next time
            if (bit) sampleBit(slot, buf[i].value, cen, nc);
            else     sampleAnalog(slot, buf[i].value, cen, nc);
        }
        if (now - lastEval >= 2000) { lastEval = now; evaluate(); }
    }
}

}  // namespace

/* ─────────────────────────────── public API ────────────────────────────── */

void learnerBegin() {
    /*
     * Size the tables to the heap that is really free, keeping a reserve
     * for Wi-Fi and the portal: about 80 % to value fields, 20 % to bits.
     */
    const size_t freeH  = ESP.getFreeHeap();
    const size_t budget = freeH > HEAP_RESERVE + 30 * 1024 ? freeH - HEAP_RESERVE : 30 * 1024;
    // Per field: its sums for every value, and its value in each held sample;
    // per bit the same, the held values as two bits per switch.
    const size_t perC   = sizeof(Cand) + NA_MAX * (sizeof(Acc) + sizeof(float));
    const size_t perB   = sizeof(BitCand) + NB_MAX * sizeof(BitAcc) + (NB_MAX * 2 + 7) / 8;
    s_maxC = constrain((budget * 8 / 10) / perC, (size_t)48, (size_t)400);
    s_maxB = constrain((budget * 2 / 10) / perB, (size_t)48, (size_t)400);
    s_bBytes = (s_maxB + 7) / 8;

    s_c    = (Cand *)   calloc(s_maxC, sizeof(Cand));
    s_acc  = (Acc *)    calloc(s_maxC * NA_MAX, sizeof(Acc));
    s_px   = (float *)  calloc(s_maxC * NA_MAX, sizeof(float));
    s_b    = (BitCand *)calloc(s_maxB, sizeof(BitCand));
    s_bacc = (BitAcc *) calloc(s_maxB * NB_MAX, sizeof(BitAcc));
    s_pbx  = (uint8_t *)calloc(s_bBytes * NB_MAX, 1);
    s_pbv  = (uint8_t *)calloc(s_bBytes * NB_MAX, 1);
    s_q    = xQueueCreate(96, sizeof(Sample));
    s_cmdQ = xQueueCreate(8, sizeof(Cmd));
    if (!s_c || !s_acc || !s_px || !s_b || !s_bacc || !s_pbx || !s_pbv || !s_q || !s_cmdQ) {
        log_e("learner: out of memory - automatic learning disabled");
        free(s_c); free(s_acc); free(s_px); free(s_b); free(s_bacc); free(s_pbx); free(s_pbv);
        s_c = nullptr; s_acc = nullptr; s_px = nullptr; s_b = nullptr; s_bacc = nullptr;
        s_pbx = s_pbv = nullptr;
        if (s_q)    vQueueDelete(s_q);
        if (s_cmdQ) vQueueDelete(s_cmdQ);
        s_q = s_cmdQ = nullptr;
        return;
    }
    resetAll();
    xTaskCreatePinnedToCore(task, "learner", 6144, nullptr, 2, nullptr, 0);
    log_i("learner started: %u fields x %u values, %u bits x %u switches (%u KB, %u KB left)",
          (unsigned)s_maxC, (unsigned)NA_MAX, (unsigned)s_maxB, (unsigned)NB_MAX,
          (unsigned)((s_maxC * perC + s_maxB * perB) / 1024), (unsigned)(ESP.getFreeHeap() / 1024));
}

void learnerOnRef(uint16_t metric, float value) {
    if (!s_q || !learnable(metric) || alreadyLearned(metric)) return;
    const Sample s = { metric, value };
    xQueueSend(s_q, &s, 0);     // never block the publisher
}

size_t learnerStatus(LearnView *out, size_t max) {
    size_t w = 0;
    for (size_t i = 0; i < NA_MAX + NB_MAX && w < max; i++) {
        const bool isBit = i >= NA_MAX;
        const Ref &r = isBit ? s_br[i - NA_MAX] : s_ar[i];
        if (!r.metric) continue;
        LearnView &v = out[w++];
        v.metric = r.metric; v.state = r.state; v.isBit = isBit;
        v.fit = r.bestFit; v.samples = r.bestN;
        if (isBit) v.progress = min(1.0f, r.toggles / 3.0f);
        else {
            const float need = needFor(r.metric, r.ymin, r.ymax);
            v.progress = 0.5f * min(1.0f, (r.ymax - r.ymin) / need) + 0.5f * min(1.0f, r.swings / 2.0f);
        }
        v.canId = 0; v.start = v.len = 0; v.be = false;
        if (r.best >= 0) {
            if (isBit && (size_t)r.best < s_nb) {
                v.canId = s_b[r.best].id; v.start = s_b[r.best].bit; v.len = 1;
            } else if (!isBit && (size_t)r.best < s_nc) {
                v.canId = s_c[r.best].id; v.start = s_c[r.best].start;
                v.len = s_c[r.best].len; v.be = s_c[r.best].be;
            }
        }
    }
    /*
     * Values already read from the bus whose slot has since gone to another
     * value. A learned value frees its slot, so listing slots alone made the
     * portal's count of listening values fall back towards zero as learning
     * went on - the better it worked, the less it seemed to have done.
     */
    for (size_t i = 0; i < s_nLearned && w < max; i++) {
        const uint16_t m = s_learned[i];
        bool listed = !m;
        for (size_t k = 0; k < w && !listed; k++) listed = out[k].metric == m;
        if (listed) continue;
        LearnView &v = out[w++];
        memset(&v, 0, sizeof(v));
        v.metric = m; v.state = LS_LEARNED; v.isBit = isBitMetric(m);
        v.progress = 1.0f; v.fit = 1.0f;
    }
    return w;
}

void learnerCounts(uint16_t &fields, uint16_t &bits) {
    fields = (uint16_t)s_nc; bits = (uint16_t)s_nb;
}

void learnerReset() { s_resetReq = true; }

/* Queued rather than a single flag, so two quick clicks on two values both
 * happen. */
void learnerAccept(uint16_t metric) {
    const Cmd c = { CMD_ACCEPT, metric };
    if (s_cmdQ) xQueueSend(s_cmdQ, &c, 0);
}
void learnerUnlearn(uint16_t metric) {
    const Cmd c = { CMD_UNLEARN, metric };
    if (s_cmdQ) xQueueSend(s_cmdQ, &c, 0);
}

void learnerForget() {
    Cfg.lock();
    for (size_t i = 0; i < Cfg.signals.size();)
        if (Cfg.signals[i].learned) Cfg.signals.erase(Cfg.signals.begin() + i);
        else i++;
    Cfg.generation++;
    Cfg.requestSave();
    Cfg.unlock();
    s_nLearned = 0;
    learnerReset();
}
