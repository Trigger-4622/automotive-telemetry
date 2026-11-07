/**
 * @file DiagCodes.cpp
 * @brief Trouble codes and identity over OBD-II - see DiagCodes.h.
 */
#include "DiagCodes.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "DtcText.h"
#include "MasterConfig.h"
#include "MasterPacket.h"
#include "MasterTelemetry.h"
#include "Ssm2.h"

/** The two computers asked: request ID, reply ID. */
static const struct { uint32_t tx, rx; } ECU[2] = { { 0x7E0, 0x7E8 }, { 0x7E1, 0x7E9 } };
static const uint8_t SERVICE[3] = { 0x03, 0x07, 0x0A };   /**< By DtcKind. */

/** First read this long after the bus comes up: the OBD-II probe and SSM2
 *  init go first, and the settle wait (if any) is over. */
static constexpr uint32_t FIRST_READ_MS = 8000;
/** A read that could not be sent (guard, settle, lock) is tried again after. */
static constexpr uint32_t RETRY_MS      = 5000;
/** Codes are re-published this often, so the displays keep them. */
static constexpr uint32_t PUBLISH_MS    = 500;
/** How long a computer has to answer (P2*CAN is 5 s, with 0x78 first). */
static constexpr uint32_t REPLY_MS      = 500;

static SemaphoreHandle_t s_mx = nullptr;
static CodesStatus       s_st = {};
static volatile bool     s_readWanted = false, s_clearWanted = false;
static uint32_t          s_readAtMs = 0;
static bool              s_identDone = false;
static uint8_t           s_identTries = 0;
static constexpr uint8_t IDENT_TRIES = 3;     /**< Reads with service-09 replies missing. */
/** Reads in a row each list went unanswered (lost reply, bus busy). */
static uint8_t           s_missed[2][3] = {};
/** Reads a list's codes are kept for while it goes unanswered. */
static constexpr uint8_t KEEP_READS = 2;

/* ───────────────────────────── one request ─────────────────────────────── */

/**
 * @brief Ask one computer. The TCM is asked on its own address; if it never
 *        answers there it is asked through 0x7DF, its reply picked out by ID.
 */
static IsoResult ask(uint8_t module, const uint8_t *req, size_t len, uint8_t *rsp, size_t max,
                     size_t &got, char *why, size_t whyCap) {
    const bool func = module == DTC_TRANSMISSION && s_st.tcmFunctional;
    const IsoResult r = isoExchange(func ? 0x7DF : ECU[module].tx, ECU[module].rx, ECU[module].tx,
                                    req, len, rsp, max, got, REPLY_MS, why, whyCap);
    // Leave room between requests: other traffic, and the ECU's own work.
    vTaskDelay(pdMS_TO_TICKS(20));
    return r;
}

/** @brief The codes in a positive reply to 03/07/0A: `4x NN (hi lo)*`, or on
 *         some computers the pairs without the count. */
static void parseCodes(const uint8_t *r, size_t got, uint8_t kind, uint8_t module,
                       DtcEntry *out, uint8_t &n) {
    if (got < 2) return;
    size_t first = 1, pairs = (got - 1) / 2;
    if ((got - 2) % 2 == 0 && (size_t)r[1] * 2 + 2 <= got) { first = 2; pairs = r[1]; }
    for (size_t i = 0; i < pairs && n < CODES_MAX; i++) {
        const uint16_t raw = (uint16_t)(r[first + 2 * i] << 8 | r[first + 2 * i + 1]);
        if (!raw) continue;                      // padding
        bool dup = false;
        for (uint8_t k = 0; k < n; k++)
            dup |= out[k].raw == raw && out[k].kind == kind && out[k].module == module;
        if (!dup) out[n++] = { raw, kind, module };
    }
}

/** @brief Text from a service-09 reply: printable characters only. */
static void ascii(const uint8_t *p, size_t n, char *out, size_t cap) {
    size_t w = 0;
    for (size_t i = 0; i < n && w + 1 < cap; i++)
        if (p[i] > 0x20 && p[i] < 0x7F) out[w++] = (char)p[i];
    out[w] = '\0';
}

/* ──────────────────────────────── a read ───────────────────────────────── */

static void readIdentity() {
    uint8_t rsp[80];
    size_t got;
    char why[40];
    const uint8_t vin[2] = { 0x09, 0x02 }, cal[2] = { 0x09, 0x04 }, cvn[2] = { 0x09, 0x06 };
    char v[18] = "", c[2][17] = { "", "" }, h[2][9] = { "", "" };
    bool gotV = false, gotC = false, gotH = false;
    int  settled = 0;                        // answered, or refused: no use asking again
    IsoResult r;
    // 49 02 01 <17 characters>
    r = ask(DTC_ENGINE, vin, 2, rsp, sizeof rsp, got, why, sizeof why);
    settled += r == ISO_OK || r == ISO_NRC;
    if (r == ISO_OK && got >= 20 && rsp[0] == 0x49 && rsp[1] == 0x02) {
        ascii(&rsp[got - 17], 17, v, sizeof v);
        gotV = true;
    }
    // 49 04 <count> <16 characters each>
    r = ask(DTC_ENGINE, cal, 2, rsp, sizeof rsp, got, why, sizeof why);
    settled += r == ISO_OK || r == ISO_NRC;
    if (r == ISO_OK && got >= 19 && rsp[0] == 0x49 && rsp[1] == 0x04) {
        for (size_t i = 0; i < 2 && 3 + 16 * (i + 1) <= got; i++) ascii(&rsp[3 + 16 * i], 16, c[i], 17);
        gotC = true;
    }
    // 49 06 <count> <4 bytes each>
    r = ask(DTC_ENGINE, cvn, 2, rsp, sizeof rsp, got, why, sizeof why);
    settled += r == ISO_OK || r == ISO_NRC;
    if (r == ISO_OK && got >= 7 && rsp[0] == 0x49 && rsp[1] == 0x06) {
        for (size_t i = 0; i < 2 && 3 + 4 * (i + 1) <= got; i++)
            snprintf(h[i], 9, "%02X%02X%02X%02X", rsp[3 + 4 * i], rsp[4 + 4 * i], rsp[5 + 4 * i], rsp[6 + 4 * i]);
        gotH = true;
    }
    // Done once each has an answer - an ECU that refuses service 09 is not
    // asked three more times a minute for ever - or after a few reads with
    // replies missing.
    if (settled == 3 || ++s_identTries >= IDENT_TRIES) s_identDone = true;
    if (!gotV && !gotC && !gotH) return;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    // Only what was read: one that went unanswered keeps what an earlier read found.
    if (gotV) strlcpy(s_st.vin, v, sizeof s_st.vin);
    for (int i = 0; i < 2; i++) {
        if (gotC) strlcpy(s_st.calId[i], c[i], sizeof s_st.calId[i]);
        if (gotH) strlcpy(s_st.cvn[i], h[i], sizeof s_st.cvn[i]);
    }
    xSemaphoreGive(s_mx);
    log_i("OBD-II: VIN %s, calibration %s %s, CVN %s", v[0] ? v : "-", c[0][0] ? c[0] : "-", c[1], h[0]);
    // Remembered, so the portal shows them with the car off too.
    Cfg.lock();
    if ((v[0] && strcmp(Cfg.vin, v)) || (c[0][0] && strcmp(Cfg.ecuCalId, c[0]))) {
        if (v[0])    strlcpy(Cfg.vin, v, sizeof Cfg.vin);
        if (c[0][0]) strlcpy(Cfg.ecuCalId, c[0], sizeof Cfg.ecuCalId);
        Cfg.requestSave();
    }
    Cfg.unlock();
}

enum ReadResult : uint8_t {
    READ_NOTHING_SENT,      /**< Not one request left: try again soon.          */
    READ_DONE,
    READ_SOME_BUSY,         /**< Done, but the bus was busy for a list: soon.   */
};

static ReadResult readCodes() {
    DtcEntry codes[CODES_MAX];
    uint8_t  n = 0, list[2][3] = {};
    uint8_t  rsp[128];
    size_t   got;
    char     why[40] = "";
    bool     sent = false, tcmFunc = s_st.tcmFunctional;

    for (uint8_t m = 0; m < 2; m++) {
        for (uint8_t k = 0; k < 3; k++) {
            const uint8_t req[1] = { SERVICE[k] };
            IsoResult r = ask(m, req, 1, rsp, sizeof rsp, got, why, sizeof why);
            // The TCM silent on its own address: once through 0x7DF.
            if (r == ISO_NO_REPLY && m == DTC_TRANSMISSION && k == DTC_STORED && !tcmFunc) {
                s_st.tcmFunctional = true;
                r = ask(m, req, 1, rsp, sizeof rsp, got, why, sizeof why);
                if (r == ISO_OK) { tcmFunc = true; log_i("codes: the transmission answers only on 0x7DF"); }
                else s_st.tcmFunctional = false;
            }
            if (r == ISO_BUSY) { list[m][k] = LIST_NOT_ASKED; continue; }
            sent = true;
            if (r == ISO_OK && rsp[0] == SERVICE[k] + 0x40) {
                list[m][k] = LIST_ANSWERED;
                parseCodes(rsp, got, k, m, codes, n);
            } else if (r == ISO_NRC) {
                list[m][k] = LIST_REFUSED;
            } else {
                list[m][k] = LIST_NO_REPLY;
                // A computer that does not answer its stored codes will not
                // answer the other two lists either.
                if (k == DTC_STORED) { list[m][1] = list[m][2] = LIST_NO_REPLY; break; }
            }
        }
    }
    if (!sent) return READ_NOTHING_SENT;

    bool answered = false, busy = false;
    for (uint8_t m = 0; m < 2; m++)
        for (uint8_t k = 0; k < 3; k++) {
            const uint8_t l = list[m][k];
            answered |= l == LIST_ANSWERED;
            busy     |= l == LIST_NOT_ASKED;
            if (l == LIST_ANSWERED || l == LIST_REFUSED) s_missed[m][k] = 0;
            else if (s_missed[m][k] < 255) s_missed[m][k]++;
        }
    // A list that went unanswered this time - its reply lost, the bus busy -
    // keeps the codes it had for a read or two: a code does not drop off the
    // displays for a minute because one request of six went unanswered.
    bool kept = false;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    for (uint8_t i = 0; i < s_st.n && n < CODES_MAX; i++) {
        const DtcEntry &e = s_st.codes[i];
        if (e.module < 2 && e.kind < 3 && s_missed[e.module][e.kind] &&
            s_missed[e.module][e.kind] <= KEEP_READS) {
            codes[n++] = e;
            kept = true;
        }
    }
    xSemaphoreGive(s_mx);
    char note[96];
    if (!answered) snprintf(note, sizeof note, "Neither computer answered (%s)", why[0] ? why : "no reply");
    else if (!n)   snprintf(note, sizeof note, "No codes");
    else           snprintf(note, sizeof note, "%u code%s%s", n, n == 1 ? "" : "s",
                            kept ? " (an unanswered list kept from the last read)" : "");

    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (answered) {
        memcpy(s_st.codes, codes, sizeof codes);
        s_st.n = n;
        s_st.haveRead = true;
        s_readAtMs = millis();
        s_st.reads++;
    }
    memcpy(s_st.list, list, sizeof list);
    strlcpy(s_st.note, note, sizeof s_st.note);
    xSemaphoreGive(s_mx);
    if (answered) {
        char line[160] = "";
        size_t w = 0;
        for (uint8_t i = 0; i < n && w < sizeof line - 16; i++) {
            char c[6];
            dtcCode(codes[i].raw, c);
            w += snprintf(line + w, sizeof line - w, " %s(%c%c)", c, "EP"[codes[i].module],
                          "SPX"[codes[i].kind]);
        }
        log_i("codes: %s%s", note, line);
    } else {
        log_w("codes: %s", note);
    }
    return busy ? READ_SOME_BUSY : READ_DONE;
}

static void clearCodes() {
    uint8_t rsp[16];
    size_t  got;
    char    why[40], note[96];
    size_t  w = snprintf(note, sizeof note, "Clear:");
    for (uint8_t m = 0; m < 2; m++) {
        const uint8_t req[1] = { 0x04 };
        const IsoResult r = ask(m, req, 1, rsp, sizeof rsp, got, why, sizeof why);
        const char *what = r == ISO_OK && rsp[0] == 0x44 ? "cleared"
                         : r == ISO_NRC && got >= 3 && rsp[2] == 0x22 ? "refused - switch the engine off, ignition on"
                         : r == ISO_NRC ? "refused"
                         : r == ISO_BUSY ? why : "no reply";
        w += snprintf(note + w, w < sizeof note ? sizeof note - w : 0, " %s %s%s",
                      DTC_MODULE_NAME[m], what, m ? "" : ";");
        if (w >= sizeof note) w = sizeof note - 1;
    }
    log_i("codes: %s", note);
    xSemaphoreTake(s_mx, portMAX_DELAY);
    strlcpy(s_st.clearNote, note, sizeof s_st.clearNote);
    s_st.clears++;
    xSemaphoreGive(s_mx);
}

/* ──────────────────────────────── the task ─────────────────────────────── */

/**
 * @brief The codes for the displays: each code once (a confirmed code is
 *        often pending as well), stored before pending before permanent, the
 *        engine's before the transmission's - the card shows the first.
 */
static void publish() {
    xSemaphoreTake(s_mx, portMAX_DELAY);
    const bool have = s_st.haveRead;
    const uint8_t n = s_st.n;
    DtcEntry c[CODES_MAX];
    memcpy(c, s_st.codes, sizeof c);
    xSemaphoreGive(s_mx);
    if (!have) return;
    DtcEntry out[CODES_MAX];
    uint8_t total = 0;
    for (uint8_t kind = 0; kind < 3; kind++)
        for (uint8_t i = 0; i < n; i++) {
            if (c[i].kind != kind) continue;
            bool seen = false;
            for (uint8_t k = 0; k < total; k++)
                seen |= out[k].raw == c[i].raw && out[k].module == c[i].module;
            if (!seen) out[total++] = c[i];
        }
    publishMetric(METRIC_ID_DTC_TOTAL, (float)total, SRC_DERIVED);
    for (uint8_t i = 0; i < total && i < DTC_SLOTS; i++)
        publishMetric(METRIC_ID_DTC_1 + i, dtcPack(out[i].raw, out[i].kind, out[i].module), SRC_DERIVED);
}

static void codesTask(void *) {
    bool     wasAlive = false;
    uint32_t nextRead = 0, lastPub = 0;
    float    lastMil = -1, lastCount = -1;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        const uint32_t now = millis();
        // Car off: nothing asked, nothing published (the displays let the
        // codes go with everything else).
        if (!diagBusAlive()) { wasAlive = false; continue; }
        if (!wasAlive) {
            wasAlive = true;
            nextRead = now + FIRST_READ_MS;
        }
        // The lamp or the ECM's own count changed: read again soon.
        float v; uint32_t age; uint8_t src;
        if (metricLookup(METRIC_ID_MIL, v, age, src) && age < 5000) {
            if (lastMil >= 0 && v != lastMil) nextRead = now + 1000;
            lastMil = v;
        }
        if (metricLookup(METRIC_ID_DTC_COUNT, v, age, src) && age < 5000) {
            if (lastCount >= 0 && v != lastCount) nextRead = now + 1000;
            lastCount = v;
        }

        // dtc_poll_s 0: only when the portal asks.
        const bool due = Cfg.dtcPollS && nextRead && (int32_t)(now - nextRead) >= 0;
        if (s_clearWanted || s_readWanted || due) {
            xSemaphoreTake(s_mx, portMAX_DELAY);
            s_st.busy = true;
            xSemaphoreGive(s_mx);
            if (s_clearWanted) {
                s_clearWanted = false;
                clearCodes();
                nextRead = millis() + 1500;      // and read what is left
            } else {
                s_readWanted = false;
                const ReadResult r = readCodes();
                if (r != READ_NOTHING_SENT && !s_identDone) readIdentity();
                // With dtc_poll_s 0 the next read still has a time, so that
                // switching polling on in the portal starts it (it is only
                // due while dtc_poll_s is not 0).
                const uint32_t every = (Cfg.dtcPollS ? Cfg.dtcPollS : 60) * 1000UL;
                nextRead = millis() + (r == READ_DONE ? every : RETRY_MS);
            }
            xSemaphoreTake(s_mx, portMAX_DELAY);
            s_st.busy = false;
            xSemaphoreGive(s_mx);
        }
        if (now - lastPub >= PUBLISH_MS) { lastPub = now; publish(); }
    }
}

/* ──────────────────────────────── the API ──────────────────────────────── */

void codesBegin() {
    if (!s_mx) s_mx = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(codesTask, "codes", 6144, nullptr, 3, nullptr, 1);
}

void codesRequestRead() { s_readWanted = true; }

bool codesRequestClear(char *why, size_t cap) {
    float v; uint32_t age; uint8_t src;
    if (!diagBusAlive()) { snprintf(why, cap, "the car is off - switch the ignition on"); return false; }
    if (Cfg.diagMode == DIAG_MODE_SILENT) { snprintf(why, cap, "requests are off (listen only)"); return false; }
    if (metricLookup(METRIC_ID_SPEED, v, age, src) && age < 3000 && v > 2.0f) {
        snprintf(why, cap, "stop the car first");
        return false;
    }
    s_clearWanted = true;
    return true;
}

void codesGetStatus(CodesStatus &out) {
    if (!s_mx) { out = {}; return; }
    xSemaphoreTake(s_mx, portMAX_DELAY);
    out = s_st;
    out.readAgeMs = s_st.haveRead ? millis() - s_readAtMs : 0;
    xSemaphoreGive(s_mx);
}

void codesName(uint16_t raw, char out[6]) { dtcCode(raw, out); }
const char *codesText(uint16_t raw)       { return dtcText(raw); }
