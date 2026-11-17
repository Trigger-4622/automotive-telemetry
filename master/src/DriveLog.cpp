/**
 * @file DriveLog.cpp
 * @brief The drive logger - see DriveLog.h.
 */
#include "DriveLog.h"

#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <algorithm>
#include <string>
#include <vector>

#include "MasterConfig.h"
#include "MasterTelemetry.h"

static constexpr const char *DIR      = "/log";
static constexpr size_t   BUF_BYTES   = 4096;     /**< Written when this full…  */
static constexpr uint32_t FLUSH_MS    = 15000;    /**< …or this old.            */
static constexpr size_t   RESERVE     = 128 * 1024;  /**< Never used: settings saves. */
static constexpr uint32_t FRESH_MS    = 1500;     /**< Older values log as NaN. */
static constexpr size_t   MAX_COLS    = 24;

/*
 * Where the logs live. The partition table gives them their own 8 MB ("logs",
 * the upper half of the 16 MB flash, which the stock 8 MB layout left blank);
 * the settings partition before it is where it always was, so nothing saved
 * moves. A master still on the older table has no "logs": the logs stay on
 * the settings partition as before - at most SHARED_MAX_KB of it, with RESERVE
 * always left free for settings saves.
 */
static constexpr const char *LOG_LABEL     = "logs";
static constexpr const char *LOG_MOUNT     = "/logfs";
static constexpr uint32_t    SHARED_MAX_KB = 1200;       /**< Of the settings partition.   */
static constexpr size_t      OWN_RESERVE   = 32 * 1024;  /**< LittleFS's working room.     */
static fs::LittleFSFS  s_logFs;
static fs::LittleFSFS *s_fs  = &LittleFS;  /**< The logs' file system (one of the two). */
static bool            s_own = false;      /**< On their own partition.                 */

static size_t   reserve()  { return s_own ? OWN_RESERVE : RESERVE; }
/** @brief The space the logs may take: log_max_kb, and on the settings
 *         partition never more than they could before. */
static uint32_t capBytes() {
    uint32_t kb = Cfg.logMaxKb;
    if (!s_own && kb > SHARED_MAX_KB) kb = SHARED_MAX_KB;
    return kb * 1024UL;
}

static SemaphoreHandle_t s_mx = nullptr;
static uint8_t  s_buf[BUF_BYTES];
static size_t   s_len = 0;
static bool     s_rec = false;              /**< A drive is open.               */
static char     s_path[24] = "", s_name[16] = "";
static std::vector<uint16_t> s_cols;
static uint8_t  s_hz = 10;
static uint32_t s_t0 = 0, s_rows = 0, s_lastFlush = 0, s_flushes = 0, s_next = 1;
static char     s_note[64] = "";
static bool     s_full = false;             /**< Out of room: wait for the next drive. */

static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/** @brief A log file's name: five digits and ".tlg", nothing else. */
static bool plainName(const char *n) {
    if (!n || strlen(n) != 9 || strcmp(n + 5, ".tlg")) return false;
    for (int i = 0; i < 5; i++) if (n[i] < '0' || n[i] > '9') return false;
    return true;
}

/** @brief Log files with their sizes, oldest (lowest number) first. */
struct Entry { char name[16]; uint32_t bytes; };
static std::vector<Entry> scanOn(fs::LittleFSFS &fsys) {
    std::vector<Entry> v;
    File d = fsys.open(DIR);
    if (!d || !d.isDirectory()) return v;
    for (File f = d.openNextFile(); f; f = d.openNextFile()) {
        const char *n = f.name();
        const char *slash = strrchr(n, '/');
        if (slash) n = slash + 1;
        if (!plainName(n)) continue;
        Entry e;
        strlcpy(e.name, n, sizeof e.name);
        e.bytes = (uint32_t)f.size();
        v.push_back(e);
    }
    std::sort(v.begin(), v.end(), [](const Entry &a, const Entry &b) { return strcmp(a.name, b.name) < 0; });
    return v;
}
static std::vector<Entry> scan() { return scanOn(*s_fs); }

static uint32_t logBytes(const std::vector<Entry> &v) {
    uint32_t n = 0;
    for (const auto &e : v) n += e.bytes;
    return n;
}

/**
 * @brief Make room for @p more bytes: delete the oldest drives until the logs
 *        fit log_max_kb and the partition keeps its reserve.
 * @return false when even with every other file gone it would not fit.
 */
static bool makeRoom(size_t more) {
    const uint32_t cap = capBytes();
    // Drives that would not delete: esp_littlefs will not unlink a file that
    // is open - one being downloaded from the portal. Passed over, or the
    // loop would try the same one for ever.
    std::vector<std::string> stuck;
    for (;;) {
        std::vector<Entry> v = scan();
        const bool capOk  = logBytes(v) + more <= cap;
        const size_t used = s_fs->usedBytes(), total = s_fs->totalBytes();
        const bool diskOk = used + more + reserve() <= total;
        if (capOk && diskOk) return true;
        // The oldest that is not the drive being written.
        const Entry *victim = nullptr;
        for (const auto &e : v) {
            if (s_rec && !strcmp(e.name, s_name)) continue;
            if (std::find(stuck.begin(), stuck.end(), e.name) != stuck.end()) continue;
            victim = &e;
            break;
        }
        if (!victim) return false;
        char p[24];
        snprintf(p, sizeof p, "%s/%s", DIR, victim->name);
        masterBusStall();
        if (!s_fs->remove(p)) {
            log_w("log: could not delete %s (open?) - trying the next", victim->name);
            stuck.push_back(victim->name);
            continue;
        }
        log_i("log: deleted %s to make room", victim->name);
    }
}

/** @brief Write the RAM rows to the file. Under s_mx. */
static void flushLocked() {
    if (!s_rec || !s_len || s_full) return;
    if (!makeRoom(s_len)) {
        s_full = true;
        snprintf(s_note, sizeof s_note, "stopped: log space full (%u KB)", (unsigned)(capBytes() / 1024));
        log_w("log: %s", s_note);
        s_len = 0;
        return;
    }
    masterBusStall();                       // the write holds the CAN interrupt off
    File f = s_fs->open(s_path, "a");
    const size_t w = f ? f.write(s_buf, s_len) : 0;
    if (f) f.close();
    if (w != s_len) {
        s_full = true;
        snprintf(s_note, sizeof s_note, "stopped: flash write failed");
        log_e("log: %s (%u of %u bytes)", s_note, (unsigned)w, (unsigned)s_len);
    }
    s_len = 0;
    s_lastFlush = millis();
    s_flushes++;
}

static void closeLocked(const char *why) {
    if (!s_rec) return;
    flushLocked();
    log_i("log: %s closed, %u rows (%s)", s_name, (unsigned)s_rows, why);
    s_rec = false;
    s_full = false;
    s_name[0] = s_path[0] = '\0';
}

/** @brief A new file, its header in the RAM buffer. Under s_mx. */
static void startLocked(uint32_t session) {
    Cfg.lock();
    s_cols.assign(Cfg.logIds.begin(), Cfg.logIds.end());
    s_hz = Cfg.logHz;
    Cfg.unlock();
    if (s_cols.size() > MAX_COLS) s_cols.resize(MAX_COLS);
    if (s_cols.empty()) { snprintf(s_note, sizeof s_note, "nothing chosen to record"); return; }
    s_fs->mkdir(DIR);
    std::vector<Entry> v = scan();
    for (const auto &e : v) s_next = max(s_next, (uint32_t)atoi(e.name) + 1);
    if (s_next > 99999) s_next = 1;
    snprintf(s_name, sizeof s_name, "%05u.tlg", (unsigned)s_next++);
    snprintf(s_path, sizeof s_path, "%s/%s", DIR, s_name);
    s_len = 0;
    memcpy(s_buf, "TLG1", 4);
    s_buf[4] = 1; s_buf[5] = s_hz; s_buf[6] = (uint8_t)s_cols.size(); s_buf[7] = 0;
    put32(s_buf + 8, session);
    put32(s_buf + 12, millis());
    s_len = 16;
    for (uint16_t id : s_cols) { put16(s_buf + s_len, id); s_len += 2; }
    s_t0 = millis();
    s_rows = 0;
    s_rec = true;
    s_full = false;
    s_note[0] = '\0';
    s_lastFlush = millis();
    log_i("log: recording %u values at %u Hz into %s", (unsigned)s_cols.size(), s_hz, s_name);
}

static bool colsChanged() {
    Cfg.lock();
    const bool ch = Cfg.logHz != s_hz || Cfg.logIds.size() != s_cols.size() ||
                    !std::equal(s_cols.begin(), s_cols.end(), Cfg.logIds.begin());
    Cfg.unlock();
    return ch;
}

/**
 * @brief Move the drives the older firmware left on the settings partition to
 *        the logs' own: each copied, its size checked, then deleted there.
 *
 * Once, at the start, before anything is recorded (s_buf is the copy buffer).
 * Every write is announced like the logger's own. A copy that falls short is
 * removed and the original kept; one already there whole (a move cut short by
 * a power cut) only has its original deleted.
 */
static void moveOldLogsLocked() {
    if (!s_own) return;
    const std::vector<Entry> old = scanOn(LittleFS);
    if (old.empty()) return;
    s_logFs.mkdir(DIR);
    unsigned moved = 0;
    for (const auto &e : old) {
        char p[24];
        snprintf(p, sizeof p, "%s/%s", DIR, e.name);
        File have = s_logFs.open(p, "r");
        const bool already = have && have.size() == e.bytes;
        if (have) have.close();
        if (!already) {
            File src = LittleFS.open(p, "r");
            File dst = s_logFs.open(p, "w");
            size_t copied = 0;
            bool ok = src && dst;
            while (ok && copied < e.bytes) {
                const size_t n = src.read(s_buf, BUF_BYTES);
                if (!n) break;
                masterBusStall();               // a flash write holds the CAN interrupt off
                ok = dst.write(s_buf, n) == n;
                copied += n;
            }
            if (src) src.close();
            if (dst) dst.close();
            if (!ok || copied != e.bytes) {
                masterBusStall();
                s_logFs.remove(p);
                log_e("log: could not move %s to the logs partition (%u of %u bytes) - left where it was",
                      e.name, (unsigned)copied, (unsigned)e.bytes);
                return;
            }
        }
        masterBusStall();
        LittleFS.remove(p);
        moved++;
    }
    log_i("log: %u drive(s) moved off the settings partition to the logs partition", moved);
}

static void logTask(void *) {
    masterWatchStack("drivelog");
    xSemaphoreTake(s_mx, portMAX_DELAY);
    moveOldLogsLocked();
    xSemaphoreGive(s_mx);
    uint32_t session = 0;
    bool wasAlive = false;
    for (;;) {
        const uint8_t hz = s_rec ? s_hz : Cfg.logHz;
        vTaskDelay(pdMS_TO_TICKS(1000 / (hz ? hz : 10)));
        const bool alive = diagBusAlive();
        if (alive && !wasAlive) session++;
        wasAlive = alive;

        xSemaphoreTake(s_mx, portMAX_DELAY);
        if (!Cfg.logOn || !alive) {
            closeLocked(!Cfg.logOn ? "switched off" : "the car went off");
        } else {
            if (s_rec && colsChanged()) closeLocked("new choice of values");
            if (!s_rec) startLocked(session);
            const size_t row = 4 + 4 * s_cols.size();
            // The car going off: the last values are still "fresh" for a
            // moment, but nothing new is on the bus - no rows of stale data.
            if (s_rec && !s_full && masterHeardWithin(500)) {
                if (s_len + row > BUF_BYTES) flushLocked();
                uint8_t *p = s_buf + s_len;
                const uint32_t now = millis();
                put32(p, now - s_t0);
                for (size_t i = 0; i < s_cols.size(); i++) {
                    float v = NAN; uint32_t age; uint8_t src;
                    if (!metricLookup(s_cols[i], v, age, src) || age > FRESH_MS) v = NAN;
                    memcpy(p + 4 + 4 * i, &v, 4);
                }
                s_len += row;
                s_rows++;
                if (millis() - s_lastFlush >= FLUSH_MS) flushLocked();
            }
        }
        xSemaphoreGive(s_mx);
    }
}

/* ──────────────────────────────── the API ──────────────────────────────── */

/**
 * @brief Mount the logs' own partition, if the partition table has one.
 *
 * In setup(), before the learner sizes its tables: the mount's buffers come
 * out of the heap here, not out of the reserve it keeps. Three tries before a
 * format, as for the settings - a mount can fail once while the supply sags.
 * A partition never formatted (the first boot after the new table went on) is
 * formatted then; one that is not in the table is never formatted - the core
 * formats only a partition it found and could not mount.
 */
static void mountOwn() {
    bool ok = false;
    for (int i = 0; i < 3 && !ok; i++) ok = s_logFs.begin(false, LOG_MOUNT, 4, LOG_LABEL);
    if (!ok) {
        masterBusStall();
        ok = s_logFs.begin(true, LOG_MOUNT, 4, LOG_LABEL);
    }
    if (ok) {
        s_fs = &s_logFs;
        s_own = true;
        log_i("log: drive logs on their own partition (%u KB)", (unsigned)(s_logFs.totalBytes() / 1024));
    } else {
        log_w("log: no \"%s\" partition (the older partition table) - drive logs stay on the "
              "settings partition, at most %u KB", LOG_LABEL, (unsigned)SHARED_MAX_KB);
    }
}

void driveLogBegin() {
    s_mx = xSemaphoreCreateMutex();
    mountOwn();
    xTaskCreatePinnedToCore(logTask, "drivelog", 6144, nullptr, 2, nullptr, 0);
}

File driveLogOpen(const char *path) { return s_fs->open(path, "r"); }

size_t driveLogList(LogFileInfo *out, size_t max) {
    if (!s_mx) return 0;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    std::vector<Entry> v = scan();
    size_t n = 0;
    for (const auto &e : v) {
        if (n >= max) break;
        LogFileInfo &o = out[n++];
        o = {};
        strlcpy(o.name, e.name, sizeof o.name);
        o.bytes = e.bytes;
        o.recording = s_rec && !strcmp(e.name, s_name);
        uint32_t bytes = e.bytes + (o.recording ? s_len : 0);
        // The header says how long a row is; the last row says how long it ran.
        char p[24];
        snprintf(p, sizeof p, "%s/%s", DIR, e.name);
        File f = s_fs->open(p, "r");
        uint8_t h[16];
        if (f && f.read(h, 16) == 16 && !memcmp(h, "TLG1", 4)) {
            o.hz = h[5]; o.cols = h[6];
            const uint32_t head = 16 + 2u * o.cols, row = 4 + 4u * o.cols;
            o.rows = bytes > head ? (bytes - head) / row : 0;
            o.seconds = o.hz ? (float)o.rows / o.hz : 0;
            if (o.recording) o.seconds = (millis() - s_t0) / 1000.0f;
        }
        if (f) f.close();
    }
    xSemaphoreGive(s_mx);
    return n;
}

bool driveLogPath(const char *name, char *path, size_t cap) {
    if (!plainName(name)) return false;
    snprintf(path, cap, "%s/%s", DIR, name);
    return s_fs->exists(path);
}

void driveLogFlush() {
    if (!s_mx) return;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    flushLocked();
    xSemaphoreGive(s_mx);
}

bool driveLogDelete(const char *name) {
    char p[24];
    if (!s_mx || !driveLogPath(name, p, sizeof p)) return false;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    if (s_rec && !strcmp(name, s_name)) { closeLocked("deleted"); }
    masterBusStall();
    const bool ok = s_fs->remove(p);
    xSemaphoreGive(s_mx);
    return ok;
}

void driveLogDeleteAll() {
    if (!s_mx) return;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    closeLocked("deleted");
    for (const auto &e : scan()) {
        char p[24];
        snprintf(p, sizeof p, "%s/%s", DIR, e.name);
        masterBusStall();
        s_fs->remove(p);
    }
    xSemaphoreGive(s_mx);
}

void driveLogStatus(DriveLogStatus &out) {
    out = {};
    out.on = Cfg.logOn;
    out.capKb = capBytes() / 1024;
    out.own = s_own;
    out.partKb = (uint32_t)(s_fs->totalBytes() / 1024);
    if (!s_mx) return;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    out.recording = s_rec && !s_full;
    strlcpy(out.file, s_name, sizeof out.file);
    out.rows = s_rows;
    out.usedKb = (logBytes(scan()) + 1023) / 1024;
    const size_t used = s_fs->usedBytes(), total = s_fs->totalBytes();
    out.freeKb = total > used ? (uint32_t)((total - used) / 1024) : 0;
    out.flushes = s_flushes;
    strlcpy(out.note, s_note, sizeof out.note);
    xSemaphoreGive(s_mx);
}
