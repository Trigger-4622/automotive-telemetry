/**
 * @file BodyUnit.cpp
 * @brief The scan for the car's computers and the BIU reader - see BodyUnit.h.
 */
#include "BodyUnit.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <string.h>

#include "MasterConfig.h"
#include "MasterPacket.h"
#include "MasterTelemetry.h"
#include "Ssm2.h"

static constexpr uint16_t SCAN_FIRST = 0x700, SCAN_LAST = 0x7F7;
static constexpr uint32_t SCAN_REPLY_MS = 60;    /**< A computer answers well within it. */
static constexpr uint32_t READ_REPLY_MS = 150;
static constexpr uint8_t  STEP_IDS      = 5;     /**< IDs a call of bodyStep() asks.     */
static constexpr uint8_t  FAILS_SLOW    = 5;     /**< Unanswered reads before slowing... */
static constexpr uint32_t SLOW_MS       = 10000; /**< ...to one every this.              */
/** The BIU's data blocks the reader asks for (local identifiers). */
static const uint8_t BLOCKS[2] = { 0x50, 0x51 };

enum ScanPhase : uint8_t { SCAN_IDLE, SCAN_PRESENT, SCAN_BIU };

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_q = nullptr;
static volatile bool s_scanWanted = false;
static ScanPhase s_phase = SCAN_IDLE;
static uint16_t  s_next = SCAN_FIRST;
static uint8_t   s_biuAt = 0;                    /**< The found one asked "21 50" next. */
static BodyFound s_found[BODY_MAX_FOUND];
static uint8_t   s_foundN = 0;
static uint8_t   s_before[32];                   /**< 0x7xx IDs heard before the scan.  */
static BodyStatus s_st;                          /**< Under s_mux for the portal.       */
static uint32_t  s_lastReadMs = 0, s_lastOkMs = 0;
static uint8_t   s_failRun = 0;

void bodyBegin() {
    if (!s_q) s_q = xQueueCreate(8, sizeof(twai_message_t));
}

bool bodyTakeFrame(twai_message_t &out) {
    return s_q && xQueueReceive(s_q, &out, 0) == pdTRUE;
}

/** The 0x700-0x7FF identifiers the frame table has, as a bitmap. */
static void heard7xx(uint8_t map[32]) {
    memset(map, 0, 32);
    uint8_t order[128];
    const size_t n = masterCensusOrder(order, sizeof order);
    for (size_t i = 0; i < n; i++) {
        CensusView c;
        if (!masterCensusRow(order[i], c, nullptr) || c.extd || c.id < 0x700 || c.id > 0x7FF) continue;
        map[(c.id - 0x700) >> 3] |= (uint8_t)(1u << (c.id & 7));
    }
}

/** Why a scan may not start, or must stop: "" when it may go on. */
static const char *scanBlocked() {
    float v;
    uint32_t age;
    uint8_t src;
    if (metricLookup(METRIC_ID_SPEED, v, age, src) && age < 3000 && v > 2.0f)
        return "not while moving";
    if (!diagBusAlive())    return "the bus is quiet - switch the ignition on";
    if (masterListenOnly()) return "the master is listening only";
    if (diagSettleLeftMs()) return "the bus is still settling";
    return "";
}

bool bodyRequestScan(char *why, size_t cap) {
    const char *b = scanBlocked();
    if (*b) { snprintf(why, cap, "%s", b); return false; }
    if (s_scanWanted || s_phase != SCAN_IDLE) { snprintf(why, cap, "a scan is running"); return false; }
    s_scanWanted = true;
    return true;
}

static void scanStop(const char *note) {
    portENTER_CRITICAL(&s_mux);
    s_phase = SCAN_IDLE;
    s_st.scanning = false;
    strlcpy(s_st.scanNote, note, sizeof s_st.scanNote);
    portEXIT_CRITICAL(&s_mux);
    log_i("body: scan stopped - %s", note);
}

static void scanFinish() {
    // Answers on an ID other than the request's + 8: whatever 0x7xx ID the
    // frame table gained meanwhile that is no found computer's reply.
    uint8_t after[32];
    heard7xx(after);
    uint16_t others[8];
    uint8_t on = 0;
    for (uint16_t id = 0x700; id <= 0x7FF && on < 8; id++) {
        const uint8_t bit = (uint8_t)(1u << (id & 7));
        if (!(after[(id - 0x700) >> 3] & bit) || (s_before[(id - 0x700) >> 3] & bit)) continue;
        bool known = id == 0x7DF;
        for (uint8_t i = 0; i < s_foundN && !known; i++) known = s_found[i].rx == id || s_found[i].tx == id;
        if (!known) others[on++] = id;
    }
    portENTER_CRITICAL(&s_mux);
    s_phase = SCAN_IDLE;
    s_st.scanning = false;
    s_st.scanned = true;
    s_st.pct = 100;
    s_st.found = s_foundN;
    memcpy(s_st.others, others, sizeof others);
    s_st.otherN = on;
    s_st.scanNote[0] = '\0';
    portEXIT_CRITICAL(&s_mux);
    uint8_t biu = 0;
    for (uint8_t i = 0; i < s_foundN; i++) biu += s_found[i].biu;
    log_i("body: scan done - %u computer(s) answered, %u like a body computer", (unsigned)s_foundN,
          (unsigned)biu);
}

/** One request of the scan. @return false: the bus would not take it - stop. */
static bool scanAsk(uint16_t tx, const uint8_t *req, size_t len, uint8_t *rsp, size_t max, size_t &got,
                    bool &answered) {
    char why[48];
    answered = false;
    for (int tries = 0; tries < 5; tries++) {
        const IsoResult r = isoExchange(tx, tx + 8u, tx, req, len, rsp, max, got, SCAN_REPLY_MS, why, sizeof why);
        if (r == ISO_BUSY) {
            // The lock was taken (the OBD-II poller goes first) - or the bus
            // will not have it, which the next check says.
            if (*scanBlocked()) return false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        answered = r == ISO_OK || r == ISO_NRC;
        return true;
    }
    return true;                               // the lock never came: count as no answer
}

static void scanStep() {
    const char *b = scanBlocked();
    if (*b) { scanStop(b); return; }
    uint8_t rsp[48];
    size_t got;
    bool answered;
    if (s_phase == SCAN_PRESENT) {
        static const uint8_t TP[2] = { 0x3E, 0x00 };
        for (uint8_t k = 0; k < STEP_IDS && s_next <= SCAN_LAST; k++, s_next++) {
            if (s_next == 0x7DF) continue;     // every computer at once: not a scan
            portENTER_CRITICAL(&s_mux);
            s_st.at = s_next;
            s_st.pct = (uint8_t)((s_next - SCAN_FIRST) * 90 / (SCAN_LAST - SCAN_FIRST + 1));
            portEXIT_CRITICAL(&s_mux);
            if (!scanAsk(s_next, TP, sizeof TP, rsp, sizeof rsp, got, answered)) {
                scanStop(*scanBlocked() ? scanBlocked() : "the bus would not take the requests");
                return;
            }
            if (!answered || s_foundN >= BODY_MAX_FOUND) continue;
            BodyFound &f = s_found[s_foundN++];
            f = BodyFound();
            f.tx = s_next;
            f.rx = (uint16_t)(s_next + 8);
            f.replyLen = (uint8_t)(got < sizeof f.reply ? got : sizeof f.reply);
            memcpy(f.reply, rsp, f.replyLen);
            log_i("body: 0x%03X answers on 0x%03X", (unsigned)f.tx, (unsigned)f.rx);
        }
        if (s_next > SCAN_LAST) { s_phase = SCAN_BIU; s_biuAt = 0; }
        return;
    }
    if (s_phase == SCAN_BIU) {
        static const uint8_t RD[2] = { 0x21, 0x50 };
        for (uint8_t k = 0; k < STEP_IDS && s_biuAt < s_foundN; k++, s_biuAt++) {
            BodyFound &f = s_found[s_biuAt];
            if (!scanAsk(f.tx, RD, sizeof RD, rsp, sizeof rsp, got, answered)) {
                scanStop(*scanBlocked() ? scanBlocked() : "the bus would not take the requests");
                return;
            }
            if (answered && got >= 2 && rsp[0] == 0x61 && rsp[1] == 0x50) {
                f.biu = true;
                f.biuLen = (uint8_t)(got - 2);
                log_i("body: 0x%03X answers 21 50 with %u bytes - a body computer", (unsigned)f.tx,
                      (unsigned)f.biuLen);
            }
        }
        if (s_biuAt >= s_foundN) scanFinish();
    }
}

/** The BIU's answer to "21 <lid>": into the frame table, eight bytes a frame. */
static void blockFrames(uint8_t lid, const uint8_t *p, size_t n) {
    for (uint8_t part = 0; part * 8u < n && part < 4; part++) {
        twai_message_t m = {};
        m.extd = 1;
        m.identifier = BODY_PSEUDO_BASE | (uint32_t)lid << 8 | part;
        const size_t k = n - part * 8u;
        m.data_length_code = (uint8_t)(k > 8 ? 8 : k);
        memcpy(m.data, p + part * 8, m.data_length_code);
        if (s_q) xQueueSend(s_q, &m, 0);   // a full queue: the next read brings it again
    }
}

static void readStep() {
    const uint16_t tx = Cfg.biuTx;
    if (!tx) return;
    const uint16_t rx = Cfg.biuRx ? Cfg.biuRx : (uint16_t)(tx + 8);
    const uint32_t now = millis();
    const uint32_t every = s_failRun >= FAILS_SLOW ? SLOW_MS : Cfg.biuMs;
    if (s_lastReadMs && now - s_lastReadMs < every) return;
    s_lastReadMs = now;
    for (int i = 0; i < 2; i++) {
        const uint8_t req[2] = { 0x21, BLOCKS[i] };
        uint8_t rsp[40];
        size_t got;
        char why[48];
        const IsoResult r = isoExchange(tx, rx, tx, req, sizeof req, rsp, sizeof rsp, got, READ_REPLY_MS,
                                        why, sizeof why);
        if (r == ISO_BUSY) return;             // the bus is not to be asked now: no failure
        const bool ok = r == ISO_OK && got >= 2 && rsp[0] == 0x61 && rsp[1] == BLOCKS[i];
        portENTER_CRITICAL(&s_mux);
        s_st.tx = tx;
        s_st.rx = rx;
        if (ok) {
            s_st.reads++;
            const size_t n = got - 2 < sizeof s_st.block[i] ? got - 2 : sizeof s_st.block[i];
            memcpy(s_st.block[i], rsp + 2, n);
            s_st.blockLen[i] = (uint8_t)n;
            s_st.note[0] = '\0';
        } else {
            s_st.fails++;
            if (r == ISO_NRC) snprintf(s_st.note, sizeof s_st.note, "21 %02X refused (code 0x%02X)", BLOCKS[i],
                                       got >= 3 ? rsp[2] : 0);
            else snprintf(s_st.note, sizeof s_st.note, "21 %02X: %s", BLOCKS[i], *why ? why : "no answer");
        }
        portEXIT_CRITICAL(&s_mux);
        if (ok) {
            s_failRun = 0;
            s_lastOkMs = now ? now : 1;
            blockFrames(BLOCKS[i], rsp + 2, got - 2);
        } else if (s_failRun < 255) {
            s_failRun++;
        }
    }
}

void bodyStep() {
    if (s_scanWanted) {
        s_scanWanted = false;
        heard7xx(s_before);
        portENTER_CRITICAL(&s_mux);
        s_foundN = 0;
        s_next = SCAN_FIRST;
        s_phase = SCAN_PRESENT;
        s_st.scanning = true;
        s_st.scanned = false;
        s_st.pct = 0;
        s_st.found = 0;
        s_st.otherN = 0;
        s_st.scanNote[0] = '\0';
        portEXIT_CRITICAL(&s_mux);
        log_i("body: scanning 0x%03X-0x%03X for the car's computers", (unsigned)SCAN_FIRST,
              (unsigned)SCAN_LAST);
    }
    if (s_phase != SCAN_IDLE) { scanStep(); return; }   // the reader waits for the scan
    readStep();
}

void bodyGetStatus(BodyStatus &out) {
    portENTER_CRITICAL(&s_mux);
    out = s_st;
    portEXIT_CRITICAL(&s_mux);
    out.found = s_st.scanning ? s_foundN : out.found;
    out.ageMs = s_lastOkMs ? millis() - s_lastOkMs : UINT32_MAX;
    if (!Cfg.biuTx) { out.tx = out.rx = 0; }
    else { out.tx = Cfg.biuTx; out.rx = Cfg.biuRx ? Cfg.biuRx : (uint16_t)(Cfg.biuTx + 8); }
}

bool bodyFoundAt(size_t i, BodyFound &out) {
    if (i >= s_foundN) return false;
    out = s_found[i];
    return true;
}
