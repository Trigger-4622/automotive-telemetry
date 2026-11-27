/**
 * @file RawRec.cpp
 * @brief The raw bus recorder - see RawRec.h for what it records and how.
 *
 * Producers (the receive task, every task that transmits, the evidence log)
 * encode a record into a RAM ring under a spinlock - a few microseconds, no
 * I/O, never blocking. The recorder's own task copies whole 256-byte pages
 * from the ring to the pre-erased partition, one page a call, so the flash
 * never holds the CAN interrupt off for longer than a page program. Records
 * are placed so none crosses a page and each 4 KB sector starts with a sync
 * record: the stream is laid out in the ring exactly as it lands on flash.
 */
#include "RawRec.h"

#include <esp_partition.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include "DriveLog.h"
#include "MasterConfig.h"
#include "MasterPacket.h"
#include "MasterTelemetry.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

static constexpr const char *LABEL    = "logs";
static constexpr char        MAGIC[8] = {'C', 'A', 'N', 'R', 'E', 'C', '1', '\0'};
static constexpr uint16_t    VERSION  = 1;
static constexpr uint32_t    PAGE     = 256;
static constexpr uint32_t    SECTOR   = 4096;
static constexpr uint32_t    DATA_OFF = SECTOR;          /**< The header has sector 0.      */
static constexpr uint32_t    STOP_OFF = 64;              /**< Header byte: 0x00 once stopped. */
static constexpr uint32_t    ERASE_STEP = 64 * 1024;     /**< One block erase at a time.    */
/** The RAM between the encoder and the flash: about two seconds of this car's
 *  bus (6 KB/s), where the recorder's task writes a page every ~40 ms. */
static constexpr uint32_t    RING     = 12 * 1024;
static constexpr uint8_t     MAX_IDX  = 0xE0;            /**< Indices 0..0xDF.              */
static constexpr size_t      SYNC_LEN = 16;
static constexpr uint32_t    WRITE_MS = 20;              /**< The task's pace.              */

enum : uint8_t { T_FULL = 0xE0, T_SYNC, T_SESSION, T_LOST, T_EVENT, T_STOP, T_PAD = 0xFF };
enum : uint8_t { LOST_BUFFER = 1, LOST_CONTROLLER = 2 };

/** An ID's last frame as recorded: what its next one is coded against. */
struct Ref { uint32_t id; uint8_t ext, tx, dlc; uint8_t d[8]; };   ///< tx: an index of the master's own frames

static const esp_partition_t *s_part = nullptr;
static volatile RawRecState s_state = RR_NONE;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;   /**< The encoder and the ring. */
static SemaphoreHandle_t s_wmx = nullptr;                    /**< Flash writes.             */
static uint8_t *s_ring = nullptr;
static Ref     *s_ids  = nullptr;
static uint8_t  s_nIds = 0;
static uint32_t s_pos  = 0;    /**< Data offset of the next byte the encoder places. */
static uint32_t s_wpos = 0;    /**< Data offset of the next byte to go to flash.     */
static uint32_t s_cap  = 0;    /**< Data offsets the partition has.                  */
static uint64_t s_lastUs = 0;  /**< The previous record's time, as a reader adds it up. */
static uint16_t s_session = 0;
static uint32_t s_frames = 0, s_txFrames = 0, s_lost = 0, s_sinceSync = 0;
static uint32_t s_lostBuf = 0, s_lostCtrl = 0;   /**< Lost, not yet in the recording. */
static volatile bool    s_armReq = false;
static volatile uint8_t s_erasePct = 0;
static char     s_note[64] = "";
static uint32_t s_armMs = 0;
static uint32_t s_rateMs = 0, s_rate = 0, s_ratePos = 0;

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static size_t uleb(uint8_t *p, uint64_t v) {
    size_t n = 0;
    do {
        const uint8_t b = v & 0x7F;
        v >>= 7;
        p[n++] = (uint8_t)(b | (v ? 0x80 : 0));
    } while (v);
    return n;
}
static uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }

/* ─────────────────────────── the encoder (under s_mux) ─────────────────── */

static uint32_t ringFree() { return RING - (s_pos - s_wpos); }
static void ringPut(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) s_ring[(s_pos + i) % RING] = p[i];
    s_pos += (uint32_t)n;
}
static void ringPad(uint32_t n) {
    for (uint32_t i = 0; i < n; i++) s_ring[(s_pos + i) % RING] = T_PAD;
    s_pos += n;
}

/** Out of room on the partition: the page in hand is padded for the writer,
 *  and nothing more is placed. */
static void fullLocked() {
    if (s_state != RR_RECORDING) return;
    const uint32_t in = s_pos % PAGE;
    if (in && ringFree() >= PAGE - in) ringPad(PAGE - in);
    s_state = RR_FULL;
    strlcpy(s_note, "the recording filled the partition", sizeof s_note);
}

/**
 * @brief Place one record: on the next page if it does not fit in this one,
 *        behind a sync record if that page starts a sector.
 * @param tag   The record's first byte.
 * @param tUs   When it happened.
 * @param timed The record carries dt (all but sync and session records).
 * @return false when it was not placed: no room in the ring, or the partition
 *         full (the recording then ends).
 */
static bool placeLocked(uint8_t tag, uint64_t tUs, const uint8_t *body, size_t blen, bool timed) {
    if (s_state != RR_RECORDING || !s_ring) return false;
    uint8_t head[12];
    size_t hn = 1;
    head[0] = tag;
    uint64_t base = s_lastUs;
    auto build = [&]() {
        hn = 1;
        if (timed) hn += uleb(head + 1, tUs > base ? (tUs - base) / 10 : 0);
    };
    build();
    uint32_t pad = 0;
    const uint32_t in = s_pos % PAGE;
    if (in && in + hn + blen > PAGE) pad = PAGE - in;
    const uint32_t at = s_pos + pad;
    const bool sync = at % SECTOR == 0 && tag != T_SYNC && tag != T_SESSION;
    if (sync) { base = tUs; build(); }
    const uint32_t need = pad + (sync ? SYNC_LEN : 0) + (uint32_t)(hn + blen);
    if (s_pos + need > s_cap) { fullLocked(); return false; }
    if (need > ringFree()) return false;
    ringPad(pad);
    if (sync) {
        uint8_t sb[SYNC_LEN];
        sb[0] = T_SYNC;
        put64(sb + 1, tUs);
        put32(sb + 9, s_sinceSync);
        put16(sb + 13, s_session);
        sb[15] = masterListenOnly() ? 1 : 0;
        ringPut(sb, SYNC_LEN);
        s_lastUs = tUs;
    }
    ringPut(head, hn);
    ringPut(body, blen);
    if (timed) s_lastUs = base + (tUs > base ? (tUs - base) / 10 * 10 : 0);
    return true;
}

/** Frames lost and not yet said so: said before the next record. */
static bool lostLocked(uint64_t tUs) {
    for (uint8_t why = LOST_BUFFER; why <= LOST_CONTROLLER; why++) {
        uint32_t &n = why == LOST_BUFFER ? s_lostBuf : s_lostCtrl;
        while (n) {
            const uint16_t part = n > 0xFFFF ? 0xFFFF : (uint16_t)n;
            uint8_t b[3];
            put16(b, part);
            b[2] = why;
            if (!placeLocked(T_LOST, tUs, b, 3, true)) return false;
            n -= part;
        }
    }
    return true;
}

static int findLocked(uint32_t id, bool ext, bool tx) {
    for (uint8_t i = 0; i < s_nIds; i++)
        if (s_ids[i].id == id && s_ids[i].ext == (uint8_t)ext && s_ids[i].tx == (uint8_t)tx) return i;
    return -1;
}

/** One frame, the bus's or ours. */
static void frameLocked(const twai_message_t &m, uint64_t tUs, bool ours) {
    if (s_state != RR_RECORDING) return;
    if (!lostLocked(tUs)) {
        if (s_state == RR_RECORDING) { s_lostBuf++; s_lost++; }
        return;
    }
    const uint8_t dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
    const bool data = !m.rtr;
    int idx = data ? findLocked(m.identifier, m.extd, ours) : -1;
    uint8_t b[20];
    bool ok;
    if (idx >= 0 && s_ids[idx].dlc == dlc) {
        // As the bytes that changed since its last frame.
        uint8_t mask = 0;
        size_t n = 1;
        for (uint8_t i = 0; i < dlc; i++)
            if (m.data[i] != s_ids[idx].d[i]) { mask |= (uint8_t)(1u << i); b[n++] = m.data[i]; }
        b[0] = mask;
        ok = placeLocked((uint8_t)idx, tUs, b, n, true);
        if (ok) memcpy(s_ids[idx].d, m.data, dlc);
    } else {
        // In full: a new ID or length (it becomes the reference), a remote
        // frame, or an ID past the table. The master's own frames get indices
        // of their own: their delta records are its frames too.
        const bool define = data && (idx >= 0 || s_nIds < MAX_IDX);
        if (define && idx < 0) idx = s_nIds;
        size_t n = 0;
        b[n++] = (uint8_t)((m.extd ? 1 : 0) | (m.rtr ? 2 : 0) | (ours ? 4 : 0) | (define ? 8 : 0));
        if (define) b[n++] = (uint8_t)idx;
        if (m.extd) { put32(b + n, m.identifier); n += 4; }
        else        { put16(b + n, (uint16_t)m.identifier); n += 2; }
        b[n++] = dlc;
        if (data) { memcpy(b + n, m.data, dlc); n += dlc; }
        ok = placeLocked(T_FULL, tUs, b, n, true);
        if (ok && define) {
            Ref &r = s_ids[idx];
            r.id = m.identifier; r.ext = m.extd; r.tx = ours; r.dlc = dlc;
            memcpy(r.d, m.data, 8);
            if (idx == s_nIds) s_nIds++;
        }
    }
    if (ok) {
        s_frames++;
        s_sinceSync++;
        if (ours) s_txFrames++;
    } else if (s_state == RR_RECORDING) {
        s_lostBuf++;
        s_lost++;
    }
}

/** A session begins: arming, a boot or a wake. Every index is forgotten. */
static void sessionLocked(uint64_t tUs) {
    s_nIds = 0;
    s_session++;
    uint8_t b[11];
    put64(b, tUs);
    put16(b + 8, s_session);
    b[10] = (uint8_t)esp_reset_reason();
    if (placeLocked(T_SESSION, tUs, b, sizeof b, false)) s_lastUs = tUs;
}

/* ─────────────────────────── the flash side ────────────────────────────── */

/** Write the pages that are complete, or with @p all the one in hand too
 *  (padded). One page a flash call. Under s_wmx. */
static bool writePages(bool all) {
    for (;;) {
        uint8_t page[PAGE];
        uint32_t from;
        portENTER_CRITICAL(&s_mux);
        if (all && s_ring) {
            const uint32_t in = s_pos % PAGE;
            if (in && ringFree() >= PAGE - in) ringPad(PAGE - in);
        }
        const bool ready = s_ring && s_pos - s_wpos >= PAGE;
        from = s_wpos;
        if (ready) for (uint32_t i = 0; i < PAGE; i++) page[i] = s_ring[(from + i) % RING];
        portEXIT_CRITICAL(&s_mux);
        if (!ready) return true;
        if (esp_partition_write(s_part, DATA_OFF + from, page, PAGE) != ESP_OK) {
            portENTER_CRITICAL(&s_mux);
            if (s_state == RR_RECORDING || s_state == RR_FULL) s_state = RR_STOPPED;
            portEXIT_CRITICAL(&s_mux);
            strlcpy(s_note, "stopped: a flash write failed", sizeof s_note);
            log_e("rawrec: %s at %u", s_note, (unsigned)from);
            return false;
        }
        portENTER_CRITICAL(&s_mux);
        s_wpos = from + PAGE;
        portEXIT_CRITICAL(&s_mux);
    }
}

static void freeBuffers() {
    portENTER_CRITICAL(&s_mux);
    uint8_t *r = s_ring;
    Ref *ids = s_ids;
    s_ring = nullptr;
    s_ids = nullptr;
    portEXIT_CRITICAL(&s_mux);
    free(r);
    free(ids);
}

static bool allocBuffers() {
    if (s_ring) return true;
    uint8_t *r = (uint8_t *)malloc(RING);
    Ref *ids = (Ref *)malloc(sizeof(Ref) * MAX_IDX);
    if (!r || !ids) { free(r); free(ids); return false; }
    portENTER_CRITICAL(&s_mux);
    s_ring = r;
    s_ids = ids;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

/** Erase the partition, write the header and start recording. The task's. */
static void doArm() {
    const uint32_t size = s_part->size;
    for (uint32_t off = 0; off < size; off += ERASE_STEP) {
        masterBusStall();                    // an erase holds the CAN interrupt off
        const uint32_t n = size - off < ERASE_STEP ? size - off : ERASE_STEP;
        if (esp_partition_erase_range(s_part, off, n) != ESP_OK) {
            strlcpy(s_note, "the erase failed - try again", sizeof s_note);
            log_e("rawrec: erase failed at %u", (unsigned)off);
            s_state = RR_STOPPED;
            s_armReq = false;
            return;
        }
        s_erasePct = (uint8_t)((uint64_t)(off + n) * 100 / size);
        vTaskDelay(1);                       // the receive task drains what came meanwhile
    }
    uint8_t h[STOP_OFF];
    memset(h, 0xFF, sizeof h);
    memcpy(h, MAGIC, 8);
    put16(h + 8, VERSION);
    put16(h + 10, PAGE);
    put32(h + 12, DATA_OFF);
    put32(h + 16, size);
    put32(h + 20, Cfg.bitrateKbps);
    put32(h + 24, millis());
    memset(h + 28, 0, 32);
    strlcpy((char *)h + 28, FIRMWARE_VERSION, 32);
    if (esp_partition_write(s_part, 0, h, sizeof h) != ESP_OK) {
        strlcpy(s_note, "the header would not write - try again", sizeof s_note);
        s_state = RR_STOPPED;
        s_armReq = false;
        return;
    }
    portENTER_CRITICAL(&s_mux);
    s_pos = s_wpos = 0;
    s_cap = (size - DATA_OFF) / PAGE * PAGE;
    s_session = 0;
    s_lostBuf = s_lostCtrl = 0;
    s_note[0] = '\0';
    s_state = RR_RECORDING;
    sessionLocked(nowUs());
    portEXIT_CRITICAL(&s_mux);
    s_armMs = millis();
    s_armReq = false;
    log_i("rawrec: armed - %u KB for the recording", (unsigned)(s_cap / 1024));
}

static void recTask(void *) {
    masterWatchStack("rawrec");
    for (;;) {
        if (s_armReq) doArm();
        const RawRecState st = s_state;
        if ((st == RR_RECORDING || st == RR_FULL || st == RR_STOPPED) && s_ring) {
            xSemaphoreTake(s_wmx, portMAX_DELAY);
            writePages(st != RR_RECORDING);
            xSemaphoreGive(s_wmx);
            // Finished and written out: the buffers go back to the heap.
            if (st != RR_RECORDING && s_pos == s_wpos) freeBuffers();
        }
        vTaskDelay(pdMS_TO_TICKS(WRITE_MS));
    }
}

/** First page that starts with 0xFF (written pages always start with a record). */
static uint32_t findEnd() {
    uint32_t lo = 0, hi = s_cap / PAGE;      // pages [0, lo) written, [hi, ..) not
    while (lo < hi) {
        const uint32_t mid = lo + (hi - lo) / 2;
        uint8_t b = 0xFF;
        esp_partition_read(s_part, DATA_OFF + mid * PAGE, &b, 1);
        if (b != T_PAD) lo = mid + 1; else hi = mid;
    }
    return lo * PAGE;
}

/**
 * The last session number in the recording. The sector of its last page
 * starts with a sync or a session record, which has the session then; one
 * begun later in that sector starts a page of its own (a boot or a wake goes
 * on at the next page), so the pages' first records have the rest.
 */
static uint16_t lastSession(uint32_t end) {
    if (!end) return 0;
    const uint32_t sec = (end - 1) / SECTOR * SECTOR;
    uint16_t s = 0;
    for (uint32_t pg = sec; pg < end; pg += PAGE) {
        uint8_t b[SYNC_LEN];
        if (esp_partition_read(s_part, DATA_OFF + pg, b, sizeof b) != ESP_OK) break;
        if (b[0] == T_SYNC && pg == sec) s = (uint16_t)(b[13] | b[14] << 8);
        else if (b[0] == T_SESSION)      s = (uint16_t)(b[9] | b[10] << 8);
    }
    return s;
}

/* ──────────────────────────────── the API ──────────────────────────────── */

void rawRecBegin() {
    static TaskHandle_t task = nullptr;
    if (!s_wmx) s_wmx = xSemaphoreCreateMutex();
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, LABEL);
    if (!s_part) {
        s_state = RR_NONE;
        return;
    }
    s_cap = (s_part->size - DATA_OFF) / PAGE * PAGE;
    uint8_t h[STOP_OFF + 1];
    const bool ours = esp_partition_read(s_part, 0, h, sizeof h) == ESP_OK && !memcmp(h, MAGIC, 8);
    if (!ours) {
        s_state = RR_OFF;
    } else {
        const uint32_t end = findEnd();
        s_pos = s_wpos = end;
        s_session = lastSession(end);
        if (h[STOP_OFF] == 0x00) {
            s_state = RR_STOPPED;
        } else if (end + PAGE > s_cap) {
            s_state = RR_FULL;
            strlcpy(s_note, "the recording filled the partition", sizeof s_note);
        } else if (!allocBuffers()) {
            s_state = RR_STOPPED;
            strlcpy(s_note, "stopped: no memory for the recorder", sizeof s_note);
        } else {
            portENTER_CRITICAL(&s_mux);
            s_state = RR_RECORDING;
            sessionLocked(nowUs());
            portEXIT_CRITICAL(&s_mux);
            log_i("rawrec: recording on, session %u, %u of %u KB used",
                  (unsigned)s_session, (unsigned)(end / 1024), (unsigned)(s_cap / 1024));
        }
    }
    // Before the learner sizes its tables (setup() calls this first), so the
    // stack comes out of the heap, not out of the reserve the learner keeps.
    if (!task) xTaskCreatePinnedToCore(recTask, "rawrec", 4096, nullptr, 5, &task, 0);
}

bool rawRecOwnsPartition() {
    const RawRecState s = s_state;
    return s == RR_ERASING || s == RR_RECORDING || s == RR_FULL || s == RR_STOPPED;
}

bool rawRecActive() {
    const RawRecState s = s_state;
    return s == RR_ERASING || s == RR_RECORDING;
}

void rawRecRx(const twai_message_t &m) {
    if (s_state != RR_RECORDING) return;
    const uint64_t t = nowUs();
    portENTER_CRITICAL(&s_mux);
    frameLocked(m, t, false);
    portEXIT_CRITICAL(&s_mux);
}

void rawRecTx(const twai_message_t &m) {
    if (s_state != RR_RECORDING) return;
    const uint64_t t = nowUs();
    portENTER_CRITICAL(&s_mux);
    frameLocked(m, t, true);
    portEXIT_CRITICAL(&s_mux);
}

void rawRecLost(uint32_t n) {
    if (s_state != RR_RECORDING || !n) return;
    portENTER_CRITICAL(&s_mux);
    s_lostCtrl += n;
    s_lost += n;
    portEXIT_CRITICAL(&s_mux);
}

void rawRecEvent(uint8_t type, uint16_t a, uint16_t b) {
    if (s_state != RR_RECORDING) return;
    const uint64_t t = nowUs();
    uint8_t body[5] = {type, (uint8_t)a, (uint8_t)(a >> 8), (uint8_t)b, (uint8_t)(b >> 8)};
    portENTER_CRITICAL(&s_mux);
    placeLocked(T_EVENT, t, body, sizeof body, true);
    portEXIT_CRITICAL(&s_mux);
}

bool rawRecArm(char *why, size_t cap) {
    if (!s_part) { snprintf(why, cap, "this master's partition table has no log space (a USB upload of this firmware adds it)"); return false; }
    if (s_state == RR_ERASING || s_armReq) { snprintf(why, cap, "the erase is already running"); return false; }
    float v; uint32_t age; uint8_t src;
    if (metricLookup(METRIC_ID_SPEED, v, age, src) && age < 3000 && v > 2.0f) {
        snprintf(why, cap, "not while moving: the erase leaves the master deaf to the bus for up to a minute");
        return false;
    }
    if (s_state == RR_RECORDING) rawRecStop();
    if (!allocBuffers()) { snprintf(why, cap, "not enough memory for the recorder"); return false; }
    // The partition is the recorder's from here: the drive logs let go of it
    // before the first erase.
    s_erasePct = 0;
    s_state = RR_ERASING;
    driveLogRelease();
    s_note[0] = '\0';
    s_armReq = true;
    log_i("rawrec: erasing the log partition for a recording");
    return true;
}

void rawRecStop() {
    if (s_state != RR_RECORDING) return;
    portENTER_CRITICAL(&s_mux);
    placeLocked(T_STOP, nowUs(), nullptr, 0, true);
    s_state = RR_STOPPED;
    portEXIT_CRITICAL(&s_mux);
    xSemaphoreTake(s_wmx, portMAX_DELAY);
    writePages(true);
    const uint8_t zero = 0;
    esp_partition_write(s_part, STOP_OFF, &zero, 1);   // 0xFF -> 0x00: no erase needed
    xSemaphoreGive(s_wmx);
    strlcpy(s_note, "stopped", sizeof s_note);
    log_i("rawrec: stopped at %u KB", (unsigned)(s_wpos / 1024));
}

bool rawRecRelease(char *why, size_t cap) {
    if (!s_part) { snprintf(why, cap, "no log space on this partition table"); return false; }
    if (s_state == RR_ERASING || s_armReq) { snprintf(why, cap, "wait for the erase to finish"); return false; }
    if (!rawRecOwnsPartition()) { snprintf(why, cap, "the drive logs have it already"); return false; }
    rawRecStop();
    xSemaphoreTake(s_wmx, portMAX_DELAY);
    freeBuffers();
    // The header and LittleFS's two superblocks go: the drive logs format it.
    masterBusStall();
    esp_partition_erase_range(s_part, 0, 2 * SECTOR);
    s_state = RR_OFF;
    xSemaphoreGive(s_wmx);
    s_note[0] = '\0';
    driveLogReclaim();
    log_i("rawrec: the log partition is the drive logs' again");
    return true;
}

void rawRecFlush() {
    if (!s_wmx || !s_ring) return;
    xSemaphoreTake(s_wmx, portMAX_DELAY);
    writePages(true);
    xSemaphoreGive(s_wmx);
}

void rawRecStatus(RawRecStatus &out) {
    out = {};
    portENTER_CRITICAL(&s_mux);
    out.state    = s_state;
    out.used     = s_pos;
    out.cap      = s_cap;
    out.frames   = s_frames;
    out.txFrames = s_txFrames;
    out.lost     = s_lost;
    out.session  = s_session;
    portEXIT_CRITICAL(&s_mux);
    out.erasePct = s_erasePct;
    // The rate over the last stretch of ten seconds or more with the bus up.
    const uint32_t now = millis();
    if (out.state == RR_RECORDING && diagBusAlive()) {
        if (!s_rateMs) { s_rateMs = now; s_ratePos = out.used; }
        else if (now - s_rateMs >= 10000) {
            s_rate = (uint32_t)((uint64_t)(out.used - s_ratePos) * 1000 / (now - s_rateMs));
            s_rateMs = now;
            s_ratePos = out.used;
        }
    } else {
        s_rateMs = 0;
    }
    out.rate = s_rate;
    strlcpy(out.note, s_note, sizeof out.note);
}

uint32_t rawRecSize() {
    if (!rawRecOwnsPartition() || s_state == RR_ERASING) return 0;
    return DATA_OFF + s_wpos;
}

bool rawRecRead(uint32_t offset, uint8_t *buf, size_t n) {
    if (!s_part) return false;
    return esp_partition_read(s_part, offset, buf, n) == ESP_OK;
}
