/**
 * @file Computers.cpp
 * @brief Finding the car's computers, asking who they are, searching and
 *        reading their data blocks - see Computers.h.
 */
#include "Computers.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <new>
#include <string.h>

#include "MasterConfig.h"
#include "MasterPacket.h"
#include "MasterTelemetry.h"
#include "Ssm2.h"

static constexpr uint32_t FIRST11     = 0x700, LAST11 = 0x7F7;
static constexpr uint32_t FUNC29      = 0x18DB33F1;  /**< Every 29-bit computer (ISO 15765-4). */
static constexpr uint32_t PROBE_MS    = 50;          /**< A TesterPresent answers well within it. */
static constexpr uint32_t PROBE29_MS  = 200;         /**< ...from every 29-bit computer at once.  */
static constexpr uint32_t ASK_MS      = 150;
static constexpr uint32_t READ_MS     = 150;
static constexpr uint8_t  STEP_ASKS   = 5;           /**< Requests a call of compStep() sends.    */
static constexpr uint8_t  FAILS_SLOW  = 5;           /**< Unanswered reads in a row before...     */
static constexpr uint32_t SLOW_MS     = 10000;       /**< ...that block is asked only this often. */
static constexpr uint8_t  MAX_PARTS   = 8;           /**< Eight bytes a frame-table row: 64 bytes. */

enum Phase : uint8_t { PH_IDLE, PH_FIND11, PH_FIND29, PH_ASK, PH_SEARCH, PH_VERIFY };

/** The questions of "who are you", in CompAsk order. */
static const uint8_t ASK_REQ[ASK_N][3] = {
    { 0x01, 0x00, 0 },          // OBD-II: the PIDs it supports
    { 0xAA, 0, 0 },             // Subaru SSM: system and ROM IDs
    { 0x1A, 0x80, 0 },          // KWP2000: its identification
    { 0x22, 0xF1, 0x97 },       // UDS: system name
    { 0x22, 0xF1, 0x87 },       // UDS: part number
};
static const uint8_t ASK_LEN[ASK_N] = { 2, 1, 2, 3, 3 };

static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;   /**< s_found, s_st, s_rs. */
static QueueHandle_t s_q = nullptr;
static volatile bool s_scanWanted = false;
static volatile int  s_searchWanted = -1;
static Phase     s_phase = PH_IDLE;
static uint32_t  s_next = FIRST11;
static uint8_t   s_askAt = 0, s_askWhat = 0;
static uint8_t   s_verifyAt = 0;
static uint16_t  s_lid = 1;
static uint8_t   s_lidSilent = 0;
static CompFound *s_found = nullptr;     /**< COMP_MAX, from the heap at the first scan. */
static bool      s_foundMulti[COMP_MAX] = {}; /**< True if discovered via multi-responder broadcast. */
static uint8_t   s_foundSvc[COMP_MAX]   = {}; /**< Probe service that elicited the hit (0x3E / 0x01). */
static uint8_t   s_foundN = 0;
static CompStatus s_st;

/** One row of the reads table as it goes (from the heap, when there is one). */
struct ReadState {
    uint32_t tx; uint8_t lid;
    uint32_t lastMs, okMs, reads, fails;
    uint8_t  failRun, len;
    char     note[40];
};
static ReadState *s_rs = nullptr;

void compBegin() {
    if (!s_q) s_q = xQueueCreate(12, sizeof(twai_message_t));
}

bool compTakeFrame(twai_message_t &out) {
    return s_q && xQueueReceive(s_q, &out, 0) == pdTRUE;
}

/** The 11-bit 0x700-0x7FF IDs other modules send on (the frame table's). */
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

/** Why a scan or search may not start, or must stop: "" when it may go on. */
static const char *blocked() {
    float v;
    uint32_t age;
    uint8_t src;
    if (metricLookup(METRIC_ID_SPEED, v, age, src) && age < 3000 && v > 2.0f) return "not while moving";
    if (!diagBusAlive())    return "the bus is quiet - switch the ignition on";
    if (masterListenOnly()) return "the master is listening only";
    if (diagSettleLeftMs()) return "the bus is still settling";
    if (!diagGuardOk())     return "the bus guard has stopped requests";
    return "";
}

static bool busyPhase() { return s_scanWanted || s_searchWanted >= 0 || s_phase != PH_IDLE; }

bool compRequestScan(char *why, size_t cap) {
    const char *b = blocked();
    if (*b) { snprintf(why, cap, "%s", b); return false; }
    if (busyPhase()) { snprintf(why, cap, "a scan or search is running"); return false; }
    s_scanWanted = true;
    return true;
}

bool compRequestSearch(size_t i, char *why, size_t cap) {
    const char *b = blocked();
    if (*b) { snprintf(why, cap, "%s", b); return false; }
    if (busyPhase()) { snprintf(why, cap, "a scan or search is running"); return false; }
    if (i >= s_foundN) { snprintf(why, cap, "no such computer - find them first"); return false; }
    s_searchWanted = (int)i;
    return true;
}

static void stop(const char *note) {
    portENTER_CRITICAL(&s_mux);
    if (s_phase == PH_SEARCH && s_st.searching >= 0 && s_found[s_st.searching].search == SEARCH_RUNNING)
        s_found[s_st.searching].search = SEARCH_NONE;
    s_phase = PH_IDLE;
    s_st.phase = PH_IDLE;
    s_st.searching = -1;
    strlcpy(s_st.note, note, sizeof s_st.note);
    portEXIT_CRITICAL(&s_mux);
    log_i("computers: stopped - %s", note);
}

static void setPhase(Phase p) {
    portENTER_CRITICAL(&s_mux);
    s_phase = p;
    s_st.phase = p;
    portEXIT_CRITICAL(&s_mux);
}

/** Add a computer that answered (once per request and reply ID); PH_VERIFY
 *  re-queries each directly to drop strays before deduplicating by reply ID. */
static void addFound(uint32_t tx, const IsoHit &h, uint8_t svc, bool multi) {
    for (uint8_t i = 0; i < s_foundN; i++)
        if (s_found[i].tx == tx && s_found[i].rx == h.id) return;
    if (s_foundN >= COMP_MAX) return;
    CompFound f;
    f.tx = tx;
    f.rx = h.id;
    memcpy(f.hello, h.data, 8);
    portENTER_CRITICAL(&s_mux);
    s_found[s_foundN]      = f;
    s_foundMulti[s_foundN] = multi;
    s_foundSvc[s_foundN]   = svc;
    s_foundN++;
    s_st.found = s_foundN;
    portEXIT_CRITICAL(&s_mux);
    log_i("computers: 0x%X answers on 0x%X", (unsigned)tx, (unsigned)h.id);
}

/** Deduplicate verified computers that share the same reply CAN ID (e.g. a
 *  module answering both functional 0x700 and physical 0x750 or 0x7DE),
 *  preferring single-responder physical addressing and canonical tx == rx - 8. */
static void dedupByRx() {
    uint8_t w = 0;
    for (uint8_t i = 0; i < s_foundN; i++) {
        int dup = -1;
        for (uint8_t j = 0; j < w; j++)
            if (s_found[j].rx == s_found[i].rx) { dup = (int)j; break; }
        if (dup < 0) {
            if (w != i) {
                s_found[w]      = s_found[i];
                s_foundMulti[w] = s_foundMulti[i];
                s_foundSvc[w]   = s_foundSvc[i];
            }
            w++;
            continue;
        }
        const bool oldMulti = s_foundMulti[dup], newMulti = s_foundMulti[i];
        const bool oldCanon = s_found[dup].tx <= 0x7FF && s_found[dup].tx + 8 == s_found[dup].rx;
        const bool newCanon = s_found[i].tx <= 0x7FF && s_found[i].tx + 8 == s_found[i].rx;
        const bool better = (!newMulti && oldMulti) || (newMulti == oldMulti && newCanon && !oldCanon);
        if (better) {
            log_i("computers: 0x%X preferred over 0x%X for reply 0x%X",
                  (unsigned)s_found[i].tx, (unsigned)s_found[dup].tx, (unsigned)s_found[i].rx);
            s_found[dup]      = s_found[i];
            s_foundMulti[dup] = s_foundMulti[i];
            s_foundSvc[dup]   = s_foundSvc[i];
        } else {
            log_i("computers: dropped duplicate 0x%X for reply 0x%X (kept 0x%X)",
                  (unsigned)s_found[i].tx, (unsigned)s_found[i].rx, (unsigned)s_found[dup].tx);
        }
    }
    portENTER_CRITICAL(&s_mux);
    s_foundN = w;
    s_st.found = s_foundN;
    portEXIT_CRITICAL(&s_mux);
}

/** A request the bus would not take now. @return true: stop here, try later. */
static bool busy(IsoResult r) {
    if (r != ISO_BUSY) return false;
    const char *b = blocked();
    if (*b) stop(b);
    else vTaskDelay(pdMS_TO_TICKS(20));      // the lock was taken: the OBD-II poller goes first
    return true;
}

static void findStep() {
    uint8_t heard[32];
    heard7xx(heard);
    static const uint8_t TP[2]  = { 0x3E, 0x00 };
    static const uint8_t OBD[2] = { 0x01, 0x00 };
    IsoHit hits[4];
    size_t n;
    char why[48];
    for (uint8_t k = 0; k < STEP_ASKS && s_next <= LAST11; k++) {
        const uint32_t id = s_next;
        bool skip = id == 0x7DF || (id >= 0x7E8 && id <= 0x7EF);
        // Another module sends on it: a frame of ours there could meet one of
        // its own in arbitration with the same ID and different data.
        if (!skip && (heard[(id - 0x700) >> 3] & (1u << (id & 7)))) {
            skip = true;
            portENTER_CRITICAL(&s_mux);
            s_st.skipped++;
            portEXIT_CRITICAL(&s_mux);
        }
        for (uint8_t i = 0; i < s_foundN && !skip; i++) skip = s_found[i].rx == id;
        if (!skip) {
            portENTER_CRITICAL(&s_mux);
            s_st.at  = id;
            s_st.pct = (uint8_t)((id - FIRST11) * 65 / (LAST11 - FIRST11 + 1));
            portEXIT_CRITICAL(&s_mux);
            const IsoResult r = isoProbe(id, TP, sizeof TP, PROBE_MS, hits, 4, n, why, sizeof why);
            if (busy(r)) return;                 // the same ID again next time
            uint8_t svc = 0x3E;
            // Emissions ECUs (ECM 0x7E0, TCM 0x7E1...) often ignore 3E 00
            // TesterPresent while answering OBD-II 01 00 cleanly.
            if (n == 0 && id >= 0x7E0 && id <= 0x7E7) {
                const IsoResult r2 = isoProbe(id, OBD, sizeof OBD, PROBE_MS, hits, 4, n, why, sizeof why);
                if (busy(r2)) return;
                svc = 0x01;
            }
            const bool multi = n > 1;
            for (size_t h = 0; h < n; h++) addFound(id, hits[h], svc, multi);
        }
        s_next = id + 1;
    }
    if (s_next > LAST11) setPhase(PH_FIND29);
}

static void find29Step() {
    static const uint8_t TP[2] = { 0x3E, 0x00 };
    IsoHit hits[8];
    size_t n;
    char why[48];
    portENTER_CRITICAL(&s_mux);
    s_st.at = FUNC29;
    s_st.pct = 68;
    portEXIT_CRITICAL(&s_mux);
    const IsoResult r = isoProbe(FUNC29, TP, sizeof TP, PROBE29_MS, hits, 8, n, why, sizeof why);
    if (busy(r)) return;
    for (size_t h = 0; h < n; h++)
        if (hits[h].ext && (hits[h].id & 0xFFFFFF00u) == 0x18DAF100u)
            addFound(0x18DA00F1u | (hits[h].id & 0xFF) << 8, hits[h], 0x3E, false);
    s_verifyAt = 0;
    s_askAt = 0;
    s_askWhat = 0;
    setPhase(s_foundN ? PH_VERIFY : PH_ASK);
}

/** Directly re-query each discovered (tx, rx) pair to drop stray/late replies
 *  captured during isoProbe() (in-place on heap-backed s_found). */
static void verifyStep() {
    uint8_t rsp[32];
    size_t got;
    char why[48];
    for (uint8_t k = 0; k < STEP_ASKS && s_verifyAt < s_foundN; k++) {
        const uint32_t tx = s_found[s_verifyAt].tx, rx = s_found[s_verifyAt].rx;
        const uint8_t req[2] = { s_foundSvc[s_verifyAt] ? s_foundSvc[s_verifyAt] : (uint8_t)0x3E, 0x00 };
        portENTER_CRITICAL(&s_mux);
        s_st.at  = tx;
        s_st.pct = (uint8_t)(68 + s_verifyAt * 7 / (s_foundN ? s_foundN : 1));
        portEXIT_CRITICAL(&s_mux);
        const IsoResult r = isoExchange(tx, rx, tx, req, 2, rsp, sizeof rsp, got, PROBE_MS, why, sizeof why);
        if (busy(r)) return;
        if (r == ISO_OK || r == ISO_NRC) {
            s_verifyAt++;
        } else {
            log_i("computers: dropped stray 0x%X -> 0x%X (no direct reply)", (unsigned)tx, (unsigned)rx);
            portENTER_CRITICAL(&s_mux);
            for (uint8_t j = s_verifyAt; j + 1 < s_foundN; j++) {
                s_found[j]      = s_found[j + 1];
                s_foundMulti[j] = s_foundMulti[j + 1];
                s_foundSvc[j]   = s_foundSvc[j + 1];
            }
            s_foundN--;
            s_st.found = s_foundN;
            portEXIT_CRITICAL(&s_mux);
        }
    }
    if (s_verifyAt >= s_foundN) {
        dedupByRx();
        s_askAt = 0;
        s_askWhat = 0;
        setPhase(PH_ASK);
    }
}

static void askStep() {
    uint8_t rsp[128];
    size_t got;
    char why[48];
    for (uint8_t k = 0; k < STEP_ASKS && s_askAt < s_foundN; k++) {
        const uint32_t tx = s_found[s_askAt].tx, rx = s_found[s_askAt].rx;
        portENTER_CRITICAL(&s_mux);
        s_st.at  = tx;
        s_st.pct = (uint8_t)(75 + (s_askAt * ASK_N + s_askWhat) * 25 / (s_foundN * ASK_N));
        portEXIT_CRITICAL(&s_mux);
        const IsoResult r = isoExchange(tx, rx, tx, ASK_REQ[s_askWhat], ASK_LEN[s_askWhat], rsp, sizeof rsp,
                                        got, ASK_MS, why, sizeof why);
        if (busy(r)) return;
        CompAnswer a;
        a.how = r == ISO_OK ? HOW_ANSWERED : r == ISO_NRC ? HOW_REFUSED : HOW_SILENT;
        if (r == ISO_NRC) a.code = got >= 3 ? rsp[2] : 0;
        if (r == ISO_OK) {
            a.len = (uint8_t)(got < sizeof a.d ? got : sizeof a.d);
            memcpy(a.d, rsp, a.len);
        }
        portENTER_CRITICAL(&s_mux);
        s_found[s_askAt].ans[s_askWhat] = a;
        portEXIT_CRITICAL(&s_mux);
        if (++s_askWhat >= ASK_N) { s_askWhat = 0; s_askAt++; }
    }
    if (s_askAt >= s_foundN) {
        portENTER_CRITICAL(&s_mux);
        s_phase = PH_IDLE;
        s_st.phase = PH_IDLE;
        s_st.scanned = true;
        s_st.pct = 100;
        s_st.note[0] = '\0';
        portEXIT_CRITICAL(&s_mux);
        log_i("computers: %u found, %u IDs left alone (other modules send on them)", (unsigned)s_foundN,
              (unsigned)s_st.skipped);
    }
}

static void searchStep() {
    const int i = s_st.searching;
    const uint32_t tx = s_found[i].tx, rx = s_found[i].rx;
    uint8_t rsp[128];
    size_t got;
    char why[48];
    for (uint8_t k = 0; k < STEP_ASKS && s_lid <= 0xFE; k++) {
        portENTER_CRITICAL(&s_mux);
        s_st.at  = s_lid;
        s_st.pct = (uint8_t)(s_lid * 100 / 0xFF);
        portEXIT_CRITICAL(&s_mux);
        const uint8_t req[2] = { 0x21, (uint8_t)s_lid };
        const IsoResult r = isoExchange(tx, rx, tx, req, 2, rsp, sizeof rsp, got, READ_MS, why, sizeof why);
        if (busy(r)) return;
        uint8_t state = SEARCH_RUNNING;
        if (r == ISO_OK && got >= 2 && rsp[0] == 0x61 && rsp[1] == s_lid) {
            portENTER_CRITICAL(&s_mux);
            CompFound &f = s_found[i];
            if (f.nLids < COMP_MAX_LIDS) {
                f.lid[f.nLids] = (uint8_t)s_lid;
                f.lidLen[f.nLids] = (uint8_t)(got - 2 > 255 ? 255 : got - 2);
                f.nLids++;
            }
            portEXIT_CRITICAL(&s_mux);
        } else if (r == ISO_NRC && got >= 3 && rsp[2] == 0x11) {
            state = SEARCH_NO_SERVICE;                // no service 21 at all
        } else if (r == ISO_NO_REPLY && ++s_lidSilent >= 3 && s_found[i].nLids == 0 && s_lid <= 3) {
            state = SEARCH_NO_SERVICE;                // silent to the first three: not a 21 computer
        }
        s_lid++;
        if (state == SEARCH_NO_SERVICE || s_lid > 0xFE) {
            portENTER_CRITICAL(&s_mux);
            s_found[i].search = state == SEARCH_NO_SERVICE ? SEARCH_NO_SERVICE : SEARCH_DONE;
            s_phase = PH_IDLE;
            s_st.phase = PH_IDLE;
            s_st.searching = -1;
            s_st.pct = 100;
            portEXIT_CRITICAL(&s_mux);
            log_i("computers: 0x%X has %u data block(s)%s", (unsigned)tx, (unsigned)s_found[i].nLids,
                  state == SEARCH_NO_SERVICE ? " - it does not have service 21" : "");
            return;
        }
    }
}

/* ─────────────────────────────── the reader ─────────────────────────────── */

/** An answer to "21 <lid>": into the frame table, eight bytes a row. */
static void blockFrames(uint32_t tx, uint8_t lid, const uint8_t *p, size_t n) {
    for (uint8_t part = 0; part * 8u < n && part < MAX_PARTS; part++) {
        twai_message_t m = {};
        m.extd = 1;
        m.identifier = compPseudo(tx, lid, part);
        const size_t k = n - part * 8u;
        m.data_length_code = (uint8_t)(k > 8 ? 8 : k);
        memcpy(m.data, p + part * 8, m.data_length_code);
        if (s_q) xQueueSend(s_q, &m, 0);   // a full queue: the next read brings it again
    }
}

static void readStep() {
    MasterConfig::RtRead rows[MasterConfig::MAX_READS];
    size_t nr = 0;
    Cfg.lock();
    for (const auto &r : Cfg.reads) if (nr < MasterConfig::MAX_READS) rows[nr++] = r;
    Cfg.unlock();
    if (!nr) return;
    if (!s_rs) {
        ReadState *rs = new (std::nothrow) ReadState[MasterConfig::MAX_READS];
        if (!rs) return;
        memset(rs, 0, sizeof(ReadState) * MasterConfig::MAX_READS);
        portENTER_CRITICAL(&s_mux);
        s_rs = rs;
        portEXIT_CRITICAL(&s_mux);
    }
    const uint32_t now = millis();
    int pick = -1;
    uint32_t most = 0;
    for (size_t i = 0; i < nr; i++) {
        ReadState &s = s_rs[i];
        if (s.tx != rows[i].tx || s.lid != rows[i].lid) {   // the table changed under it
            portENTER_CRITICAL(&s_mux);
            memset(&s, 0, sizeof s);
            s.tx = rows[i].tx;
            s.lid = rows[i].lid;
            portEXIT_CRITICAL(&s_mux);
        }
        const uint32_t every = s.failRun >= FAILS_SLOW ? SLOW_MS : rows[i].ms;
        const uint32_t late = s.lastMs ? now - s.lastMs : UINT32_MAX;
        if (late >= every && (pick < 0 || late - every > most)) { pick = (int)i; most = late - every; }
    }
    if (pick < 0) return;
    const MasterConfig::RtRead &r = rows[pick];
    const uint8_t req[2] = { 0x21, r.lid };
    uint8_t rsp[2 + MAX_PARTS * 8];
    size_t got;
    char why[40];
    const IsoResult res = isoExchange(r.tx, r.rx, r.tx, req, 2, rsp, sizeof rsp, got, READ_MS, why, sizeof why);
    if (res == ISO_BUSY) return;                  // not asked: no failure
    const bool ok = res == ISO_OK && got >= 2 && rsp[0] == 0x61 && rsp[1] == r.lid;
    portENTER_CRITICAL(&s_mux);
    ReadState &s = s_rs[pick];
    s.lastMs = now ? now : 1;
    if (ok) {
        s.reads++;
        s.failRun = 0;
        s.okMs = s.lastMs;
        s.len = (uint8_t)(got - 2);
        s.note[0] = '\0';
    } else {
        s.fails++;
        if (s.failRun < 255) s.failRun++;
        if (res == ISO_NRC) snprintf(s.note, sizeof s.note, "refused (code 0x%02X)", got >= 3 ? rsp[2] : 0);
        else snprintf(s.note, sizeof s.note, "%s", *why ? why : "no answer");
    }
    portEXIT_CRITICAL(&s_mux);
    if (ok) blockFrames(r.tx, r.lid, rsp + 2, got - 2);
}

void compStep() {
    if (s_scanWanted || s_searchWanted >= 0) {
        if (!s_found) {
            CompFound *f = new (std::nothrow) CompFound[COMP_MAX];
            if (!f) { s_scanWanted = false; s_searchWanted = -1; stop("out of memory"); return; }
            portENTER_CRITICAL(&s_mux);
            s_found = f;
            portEXIT_CRITICAL(&s_mux);
        }
        if (s_scanWanted) {
            s_scanWanted = false;
            portENTER_CRITICAL(&s_mux);
            s_foundN = 0;
            memset(s_foundMulti, 0, sizeof s_foundMulti);
            memset(s_foundSvc, 0, sizeof s_foundSvc);
            s_next = FIRST11;
            s_phase = PH_FIND11;
            s_st = CompStatus();
            s_st.phase = PH_FIND11;
            portEXIT_CRITICAL(&s_mux);
            log_i("computers: asking 0x%03X-0x%03X and every 29-bit one", (unsigned)FIRST11, (unsigned)LAST11);
        } else {
            const int i = s_searchWanted;
            s_searchWanted = -1;
            portENTER_CRITICAL(&s_mux);
            CompFound &f = s_found[i];
            f.nLids = 0;
            f.search = SEARCH_RUNNING;
            s_lid = 1;
            s_lidSilent = 0;
            s_phase = PH_SEARCH;
            s_st.phase = PH_SEARCH;
            s_st.searching = (int8_t)i;
            s_st.pct = 0;
            s_st.note[0] = '\0';
            portEXIT_CRITICAL(&s_mux);
            log_i("computers: searching the data blocks of 0x%X", (unsigned)f.tx);
        }
    }
    if (s_phase != PH_IDLE) {
        const char *b = blocked();
        if (*b) { stop(b); return; }
        switch (s_phase) {
            case PH_FIND11: findStep(); break;
            case PH_FIND29: find29Step(); break;
            case PH_VERIFY: verifyStep(); break;
            case PH_ASK:    askStep(); break;
            case PH_SEARCH: searchStep(); break;
            default: break;
        }
        return;                                    // the reader waits for it
    }
    readStep();
}

void compGetStatus(CompStatus &out) {
    portENTER_CRITICAL(&s_mux);
    out = s_st;
    out.found = s_foundN;
    // Asked for, not yet begun (the codes task takes it within 100 ms).
    if (s_scanWanted) out.phase = PH_FIND11;
    else if (s_searchWanted >= 0) { out.phase = PH_SEARCH; out.searching = (int8_t)s_searchWanted; }
    portEXIT_CRITICAL(&s_mux);
}

bool compFoundAt(size_t i, CompFound &out) {
    bool ok = false;
    portENTER_CRITICAL(&s_mux);
    if (s_found && i < s_foundN) { out = s_found[i]; ok = true; }
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

bool compReadAt(size_t i, CompReadView &out) {
    MasterConfig::RtRead r;
    Cfg.lock();
    const bool have = i < Cfg.reads.size();
    if (have) r = Cfg.reads[i];
    Cfg.unlock();
    if (!have) return false;
    out = CompReadView();
    out.tx = r.tx;
    out.rx = r.rx;
    out.lid = r.lid;
    out.ms = r.ms;
    portENTER_CRITICAL(&s_mux);
    if (s_rs && i < MasterConfig::MAX_READS && s_rs[i].tx == r.tx && s_rs[i].lid == r.lid) {
        const ReadState &s = s_rs[i];
        out.reads = s.reads;
        out.fails = s.fails;
        out.len = s.len;
        if (s.okMs) out.ageMs = millis() - s.okMs;
        memcpy(out.note, s.note, sizeof out.note);
    }
    portEXIT_CRITICAL(&s_mux);
    return true;
}
