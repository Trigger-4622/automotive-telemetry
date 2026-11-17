/**
 * @file FwUpdate.cpp
 * @brief A new master firmware over Wi-Fi - see FwUpdate.h.
 */
#include "FwUpdate.h"

#include <Update.h>
#include <esp_ota_ops.h>
#include <string.h>

#include "MasterPacket.h"
#include "MasterTelemetry.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

/** In every master image, and in no other firmware: an upload must carry it.
 *  Shown by the portal (fwStatus()), so the linker keeps it. */
extern "C" const char FW_MARK[] = "CANTELEMETRY-MASTER-FIRMWARE/1";
static constexpr size_t MARK_LEN = sizeof(FW_MARK) - 1;

static bool     s_busy = false;
static uint32_t s_got = 0, s_t0 = 0, s_lastWrite = 0;
static uint32_t s_bootMs = 0;      /**< millis() at fwBootReport(): the trial is timed from it. */
static bool     s_markSeen = false;
static uint8_t  s_tail[MARK_LEN];   /**< The end of the last piece: the mark can straddle two. */
static size_t   s_tailLen = 0;

/* ─────────────────────────────── the mark ─────────────────────────────── */

static bool contains(const uint8_t *p, size_t n) {
    if (n < MARK_LEN) return false;
    const uint8_t first = (uint8_t)FW_MARK[0];
    for (size_t i = 0; i + MARK_LEN <= n;) {
        const void *hit = memchr(p + i, first, n - MARK_LEN + 1 - i);
        if (!hit) return false;
        i = (size_t)((const uint8_t *)hit - p);
        if (!memcmp(p + i, FW_MARK, MARK_LEN)) return true;
        i++;
    }
    return false;
}

static void scanMark(const uint8_t *d, size_t n) {
    if (s_markSeen || !n) return;
    // The seam: the end of the previous piece and the start of this one.
    uint8_t seam[2 * MARK_LEN];
    const size_t head = n < MARK_LEN - 1 ? n : MARK_LEN - 1;
    memcpy(seam, s_tail, s_tailLen);
    memcpy(seam + s_tailLen, d, head);
    if (contains(seam, s_tailLen + head) || contains(d, n)) { s_markSeen = true; return; }
    // Keep the last MARK_LEN - 1 bytes seen.
    if (n >= MARK_LEN - 1) {
        memcpy(s_tail, d + n - (MARK_LEN - 1), MARK_LEN - 1);
        s_tailLen = MARK_LEN - 1;
    } else {
        uint8_t joined[2 * MARK_LEN];
        memcpy(joined, s_tail, s_tailLen);
        memcpy(joined + s_tailLen, d, n);
        const size_t all = s_tailLen + n, keep = all < MARK_LEN - 1 ? all : MARK_LEN - 1;
        memcpy(s_tail, joined + all - keep, keep);
        s_tailLen = keep;
    }
}

/* ─────────────────────────────── the upload ────────────────────────────── */

bool fwUpdateAllowed(char *why, size_t cap) {
    float v; uint32_t age; uint8_t src;
    if (metricLookup(METRIC_ID_SPEED, v, age, src) && age < 3000 && v > 2.0f) {
        snprintf(why, cap, "stop the car first");
        return false;
    }
    return true;
}

static void stop() {
    Update.abort();
    s_busy = false;
}

bool fwUpdateBegin(char *why, size_t cap) {
    if (s_busy) stop();                      // a new upload replaces one cut off
    if (!fwUpdateAllowed(why, cap)) return false;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        snprintf(why, cap, "cannot start: %s", Update.errorString());
        return false;
    }
    s_busy = true;
    s_got = 0;
    s_markSeen = false;
    s_tailLen = 0;
    s_t0 = s_lastWrite = millis();
    log_i("update: receiving a new firmware into the spare slot");
    return true;
}

bool fwUpdateWrite(const uint8_t *data, size_t len, char *why, size_t cap) {
    if (!s_busy) { snprintf(why, cap, "no update in progress"); return false; }
    if (!len) return true;
    if (s_got == 0 && data[0] != 0xE9) {
        stop();
        snprintf(why, cap, "not a firmware image (the .bin a build makes)");
        return false;
    }
    scanMark(data, len);
    masterBusStall();                        // a flash write holds the CAN interrupt off
    if (Update.write(const_cast<uint8_t *>(data), len) != len) {
        snprintf(why, cap, "flash write failed: %s", Update.errorString());
        stop();
        return false;
    }
    s_got += len;
    s_lastWrite = millis();
    return true;
}

bool fwUpdateEnd(char *why, size_t cap) {
    if (!s_busy) { snprintf(why, cap, "no update in progress"); return false; }
    if (!s_markSeen) {
        stop();
        snprintf(why, cap, "not the master's firmware (a display's?)");
        log_w("update: refused - %u bytes without the master's mark", (unsigned)s_got);
        return false;
    }
    masterBusStall();
    // Checks the image (checksum, hash, chip) and makes its slot the one to boot.
    if (!Update.end(true)) {
        snprintf(why, cap, "the image did not check out: %s", Update.errorString());
        s_busy = false;
        log_w("update: %s", why);
        return false;
    }
    s_busy = false;
    log_i("update: %u bytes in %u s, checked - restarting into it (on trial for %u s)",
          (unsigned)s_got, (unsigned)((millis() - s_t0) / 1000), (unsigned)(FW_CONFIRM_MS / 1000));
    return true;
}

void fwUpdateAbort() {
    if (!s_busy) return;
    stop();
    log_w("update: cut off after %u bytes - nothing changed", (unsigned)s_got);
}

bool fwUpdateBusy() { return s_busy; }

/* ───────────────────────────── trial and status ─────────────────────────── */

/**
 * The bootloader's rollback, not Arduino's default (which confirms every new
 * firmware the moment it starts): the master confirms itself in fwUpdateTick().
 */
extern "C" bool verifyRollbackLater() { return true; }

static bool onTrial() {
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    return run && esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;
}

void fwUpdateTick(bool portalUp, bool beforeSleep) {
    if (s_busy && millis() - s_lastWrite > FW_STALL_MS) {
        stop();
        log_w("update: nothing received for %u s - given up, nothing changed", (unsigned)(FW_STALL_MS / 1000));
    }
    static bool checked = false, pending = false;
    if (!checked) { checked = true; pending = onTrial(); }
    if (!pending || !portalUp) return;       // a firmware whose portal is down is never kept
    if (!beforeSleep && millis() - s_bootMs < FW_CONFIRM_MS) return;
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        pending = false;
        log_i("update: the new firmware has run properly - kept");
    }
}

void fwBootReport() {
    s_bootMs = millis();                     // the trial runs from here, whatever the clock read
    FwStatus s;
    fwStatus(s);
    log_i("firmware %s, build %s, slot %s%s", s.version, s.build, s.slot,
          s.trial ? " - new, on trial: kept once it has run properly" : "");
    if (s.rolledBack)
        log_w("update: the last update did not run properly - back on this firmware");
}

void fwStatus(FwStatus &out) {
    out = {};
    out.version = FIRMWARE_VERSION;
    char sha[65] = "";
    esp_ota_get_app_elf_sha256(sha, sizeof sha);
    strlcpy(out.build, sha, sizeof out.build);
    const esp_partition_t *run = esp_ota_get_running_partition();
    out.slot = run ? run->label : "?";
    out.trial = onTrial();
    out.rolledBack = esp_ota_get_last_invalid_partition() != nullptr;
    out.busy = s_busy;
    out.received = s_got;
    out.mark = FW_MARK;                      // used at run time: the linker keeps it in the image
}
