/**
 * @file main.cpp
 * @ingroup master
 * @brief CAN Telemetry Bridge — MASTER NODE reference implementation.
 *
 * Architecture (FreeRTOS, ESP32-S3):
 *
 *   ┌─────────────┐   twai_receive   ┌──────────────┐  publishMetric  ┌──────────────┐
 *   │ Vehicle CAN │ ───────────────► │ twaiRxTask   │ ──────────────► │ Metric store │
 *   │  (TWAI +    │                  │ · OBD resp   │                 │ (192 slots,  │
 *   │ SN65HVD230) │ ◄─────────────── │ · RAW table  │                 │  spinlock,   │
 *   └─────────────┘  twai_transmit   │ · verifier   │                 │  per-source  │
 *          ▲                         └──────────────┘                 │  priority)   │
 *          │                                                          └──────┬───────┘
 *   ┌──────┴───────┐  ┌──────────────┐                     ┌────────────────▼──────────┐
 *   │ obdPollTask  │  │ ssm2Task     │  one dialogue       │ broadcastTask (25 Hz)     │
 *   │ 0x7DF PIDs   │  │ 0x7E0 ISO-TP │  at a time (mutex)  │ changed metrics + keep-   │
 *   └──────────────┘  └──────────────┘                     │ alives → ≤28/frame BURST  │
 *                                                          │ → ESP-NOW FF:…FF          │
 *                                                          └───────────────────────────┘
 *
 * The node has ZERO UI knowledge: it never knows what a slave renders.
 * Everything vehicle-specific is in include/CanDecoderConfig.h and the SSM2
 * seed table in MasterConfig.cpp. Full design rationale:
 * MASTER_NODE_BLUEPRINT.md (next to platformio.ini)
 */
#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/semphr.h>
#include <vector>
#include <LittleFS.h>
#include "driver/twai.h"
#include "driver/gpio.h"

#include "MasterPacket.h"
#include "CanDecoderConfig.h"
#include "MasterConfig.h"
#include "MasterTelemetry.h"
#include "Learner.h"
#include "Ssm2.h"
#include "WebPortal.h"

/**
 * @defgroup master Master Node
 * @brief CAN acquisition and the ESP-NOW broadcaster.
 *
 * Vehicle-specific configuration lives entirely in
 * @ref CanDecoderConfig.h; nothing in this file needs editing to port the
 * bridge to another car.
 * @{
 */

/* ═══════════════════════════ metric store ════════════════════════════════ */

/** @brief Capacity of the master's metric store.
 *
 *  More than @ref TELEMETRY_MAX_METRICS on purpose: the broadcaster splits
 *  the store into multi-frame bursts, so the total channel count is not
 *  limited by the 250-byte ESP-NOW frame.
 */
static constexpr size_t MAX_MASTER_METRICS = 192;

/** A value from a higher-ranked source blocks lower-ranked writers for this
 *  long. Long enough to cover one poll period of the faster source, short
 *  enough that a source going away hands over within half a second. */
/* The hold time is Cfg.sourceHoldMs - editable in the portal. */

/** @brief One parsed channel awaiting broadcast. */
struct MasterMetric {
    uint16_t id;            /**< Metric ID — see @ref MetricIDs.        */
    float    value;         /**< Latest engineering-unit value.         */
    uint32_t lastUpdateMs;  /**< millis() of the last update, for TTL.  */
    uint8_t  source;        /**< @ref MetricSource of the last write.   */
    bool     used;          /**< Slot has been claimed.                 */
    float    sentValue;     /**< Value in the last broadcast.           */
    uint32_t sentMs;        /**< millis() of the last broadcast.        */
    bool     everSent;      /**< At least one broadcast has carried it. */
};

/** @brief The metric store, shared between the CAN and broadcast tasks. */
static MasterMetric s_metrics[MAX_MASTER_METRICS] = {};
/** @brief Guards @ref s_metrics across the task boundary. */
static portMUX_TYPE s_metricMux = portMUX_INITIALIZER_UNLOCKED;
/** @brief Latest night-mode decision, stamped onto every outgoing entry. */
static volatile bool s_night = false;

/** @name Counters for the 5 s serial heartbeat
 *  @{ */
static volatile uint32_t s_canFramesRx  = 0;  /**< CAN frames received.     */
static volatile uint32_t s_obdResponses = 0;  /**< OBD replies decoded.     */
static volatile uint32_t s_espnowFrames = 0;  /**< Broadcasts accepted.     */
static volatile uint32_t s_espnowFailed = 0;  /**< Broadcasts refused.      */
/**
 * Metric entries actually carried, which is the number that answers "is data
 * flowing". Frames alone are misleading once the broadcaster is change-driven:
 * a quiet bench with five unchanging housekeeping channels produces a handful
 * of keep-alive frames per second, while a running engine produces far more
 * than the old send-everything-every-burst scheme ever did. Counting payload
 * makes that visible instead of alarming.
 */
static volatile uint32_t s_espnowMetrics = 0;
/** millis() of the last CAN frame. Zero means none since boot. */
static volatile uint32_t s_lastCanMs = 0;
/** millis() when the bus last came up: its first frame since boot, or after
 *  BUS_QUIET_MS without one. The settle wait counts from here. */
static volatile uint32_t s_busUpMs = 0;
static volatile uint32_t s_busSession = 0;      /**< Times it came up (0 = never). */
static volatile uint32_t s_settledSession = 0;  /**< The one whose wait is over, */
static volatile uint32_t s_settledAtMs = 0;     /**< ...and since when.          */
/** @} */

/* ═══════════════════════════ evidence log ════════════════════════════════
 *
 * A small persistent record of what the master was doing when the bus misbehaved,
 * so one drive can say which fault it is instead of a shrug. Boot and reset
 * reasons (a brownout at cranking looks nothing like a clean power-on), the bus
 * coming up and settling, every guard trip with the error counters and what we
 * were transmitting at the time, bus-off, and sleep. Kept in a ring on LittleFS
 * (its own file, so it survives a firmware update alongside the config) and shown
 * in the portal's Diagnostics tab.
 *
 * Writing is split in two: evLog() only appends to the RAM ring under a spinlock
 * (safe from any task, no I/O), and loop() flushes the ring to flash when it is
 * dirty - so the CAN receive path never touches the filesystem.
 */
enum EvType : uint8_t {
    EV_BOOT = 0,     /**< a = esp_reset_reason(), b = cfg version.            */
    EV_BUS_UP,       /**< the bus produced its first frame (a = session).     */
    EV_SETTLE_END,   /**< the settle wait ended; requests allowed.            */
    EV_TX_TRIP,      /**< bus guard: errors during our own frames (a = count).*/
    EV_RX_TRIP,      /**< receive-error guard: we were corrupting frames (a=REC).*/
    EV_BUS_OFF,      /**< the controller went bus-off.                        */
    EV_SLEEP,        /**< entering deep sleep.                                */
    EV_RESUME,       /**< the guard was cleared from the portal.              */
    EV_TYPE_COUNT
};

/** What the master was transmitting when an event happened, as a bitmask, so a
 *  burst can be pinned to what provoked it. */
static uint8_t diagActivityMask();

struct EvRecord {
    uint32_t ms;    /**< millis() when it happened.                 */
    uint8_t  type;  /**< @ref EvType.                               */
    uint8_t  act;   /**< diagActivityMask() at the time.            */
    uint16_t a;     /**< Type-specific (reset reason, error count). */
    uint16_t b;     /**< Type-specific.                             */
};

static constexpr size_t EV_MAX = 48;
static EvRecord s_ev[EV_MAX] = {};
static volatile uint8_t s_evHead = 0;    /**< Next write slot.        */
static volatile uint8_t s_evCount = 0;   /**< Records held (<= EV_MAX).*/
static portMUX_TYPE s_evMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_evDirty = false;
/** One writer of the file at a time: loop() flushes it, and the sleep path
 *  flushes it from the housekeeping task. */
static SemaphoreHandle_t s_evSaveMutex = nullptr;
static constexpr const char *EV_PATH = "/evlog.json";
static constexpr const char *EV_TMP  = "/evlog.tmp";

/** Append one event to the RAM ring. No I/O: loop() writes it to flash. */
static void evLog(uint8_t type, uint16_t a = 0, uint16_t b = 0) {
    const uint32_t now = millis();
    const uint8_t act = diagActivityMask();
    portENTER_CRITICAL(&s_evMux);
    EvRecord &r = s_ev[s_evHead];
    r.ms = now; r.type = type; r.act = act; r.a = a; r.b = b;
    s_evHead = (uint8_t)((s_evHead + 1) % EV_MAX);
    if (s_evCount < EV_MAX) s_evCount++;
    s_evDirty = true;
    portEXIT_CRITICAL(&s_evMux);
}

/** Load the ring from flash at boot, oldest first, so history survives a reset. */
static void evLoad() {
    File f = LittleFS.open(EV_PATH, "r");
    if (!f && LittleFS.exists(EV_TMP)) f = LittleFS.open(EV_TMP, "r");
    if (!f) return;
    JsonDocument d;
    const DeserializationError err = deserializeJson(d, f);
    f.close();
    if (err || !d.is<JsonArrayConst>()) return;
    for (JsonVariantConst v : d.as<JsonArrayConst>()) {
        if (!v.is<JsonArrayConst>()) continue;
        JsonArrayConst rec = v.as<JsonArrayConst>();
        if (rec.size() < 5) continue;
        EvRecord &r = s_ev[s_evHead];
        r.ms   = rec[0] | 0u;
        r.type = (uint8_t)(rec[1] | 0);
        r.act  = (uint8_t)(rec[2] | 0);
        r.a    = (uint16_t)(rec[3] | 0);
        r.b    = (uint16_t)(rec[4] | 0);
        s_evHead = (uint8_t)((s_evHead + 1) % EV_MAX);
        if (s_evCount < EV_MAX) s_evCount++;
    }
}

/** Copy the ring oldest-first into @p out. */
static size_t evSnapshot(EvRecord *out, size_t max) {
    portENTER_CRITICAL(&s_evMux);
    const uint8_t n = s_evCount;
    const uint8_t start = (uint8_t)((s_evHead + EV_MAX - n) % EV_MAX);
    size_t w = 0;
    for (uint8_t i = 0; i < n && w < max; i++)
        out[w++] = s_ev[(start + i) % EV_MAX];
    portEXIT_CRITICAL(&s_evMux);
    return w;
}

/** Flush the ring to flash (atomic tmp+rename, like the config). */
static void evSaveNow() {
    if (!s_evSaveMutex) return;
    xSemaphoreTake(s_evSaveMutex, portMAX_DELAY);
    static EvRecord snap[EV_MAX];
    const size_t n = evSnapshot(snap, EV_MAX);
    JsonDocument d;
    JsonArray arr = d.to<JsonArray>();
    for (size_t i = 0; i < n; i++) {
        JsonArray r = arr.add<JsonArray>();
        r.add(snap[i].ms); r.add(snap[i].type); r.add(snap[i].act);
        r.add(snap[i].a);  r.add(snap[i].b);
    }
    File f = LittleFS.open(EV_TMP, "w");
    if (f) {
        const size_t expected = measureJson(d);
        const size_t written = serializeJson(d, f);
        f.close();
        if (!written || written != expected) {
            LittleFS.remove(EV_TMP);              // a short write never replaces the log
        } else if (!LittleFS.rename(EV_TMP, EV_PATH)) {
            LittleFS.remove(EV_PATH);
            LittleFS.rename(EV_TMP, EV_PATH);
        }
    }
    xSemaphoreGive(s_evSaveMutex);
}

/** Called from loop(): write the ring to flash when it changed, at most every
 *  few seconds so a run of events is not a run of flash writes. */
static void evService() {
    static uint32_t lastSave = 0;
    if (!s_evDirty) return;
    if (lastSave && millis() - lastSave < 3000) return;
    s_evDirty = false;
    lastSave = millis();
    evSaveNow();
}

/** Human-readable event names, shared by the serial log and the portal. */
static const char *evName(uint8_t type) {
    static const char *N[] = { "boot", "bus-up", "settled", "tx-guard-trip",
                               "rx-guard-trip", "bus-off", "sleep", "resume" };
    return type < EV_TYPE_COUNT ? N[type] : "?";
}

static MasterMetric *findSlot(uint16_t id) {
    for (auto &m : s_metrics)
        if (m.used && m.id == id) return &m;
    return nullptr;
}

void publishMetric(uint16_t id, float value, uint8_t source) {
    const uint32_t now = millis();
    portENTER_CRITICAL(&s_metricMux);
    MasterMetric *slot = findSlot(id);
    if (!slot)
        for (auto &m : s_metrics)
            if (!m.used) { slot = &m; slot->used = true; slot->id = id;
                           slot->everSent = false; slot->source = SRC_NONE; break; }
    if (slot) {
        /*
         * Source priority. A raw broadcast frame at 50 Hz must not be
         * overwritten by the same quantity from a 5 Hz OBD poll a moment
         * later - the needle would jitter between two sample clocks. So a
         * lower-ranked writer yields while a higher-ranked one is fresh.
         */
        const bool blocked = slot->source > source &&
                             now - slot->lastUpdateMs < Cfg.sourceHoldMs;
        if (!blocked) {
            slot->value        = value;
            slot->source       = source;
            slot->lastUpdateMs = now;
        }
    }
    portEXIT_CRITICAL(&s_metricMux);
    // Every requested value is a reference the learner can match the bus
    // against - that is how requests teach the master to stop requesting.
    if (source == SRC_OBD || source == SRC_SSM) learnerOnRef(id, value);
}

bool metricLookup(uint16_t id, float &value, uint32_t &ageMs, uint8_t &source) {
    bool ok = false;
    portENTER_CRITICAL(&s_metricMux);
    const MasterMetric *m = findSlot(id);
    if (m) {
        value  = m->value;
        ageMs  = millis() - m->lastUpdateMs;
        source = m->source;
        ok = true;
    }
    portEXIT_CRITICAL(&s_metricMux);
    return ok;
}

bool metricCoveredAbove(uint16_t id, uint8_t below, uint32_t maxAgeMs) {
    bool covered = false;
    portENTER_CRITICAL(&s_metricMux);
    const MasterMetric *m = findSlot(id);
    if (m) covered = m->source > below && millis() - m->lastUpdateMs < maxAgeMs;
    portEXIT_CRITICAL(&s_metricMux);
    return covered;
}

/**
 * @brief Map a value onto its wire flags.
 * @param id Metric ID, used to find the row in @ref THRESHOLD_TABLE.
 * @param v  Value to classify.
 * @return @ref METRIC_FLAG_VALID plus any warning/critical/night bits. A NAN
 *         bound in the table disables that comparison.
 */
static uint8_t computeFlags(uint16_t id, float v) {
    uint8_t flags = METRIC_FLAG_VALID;
    for (auto &t : THRESHOLD_TABLE) {
        if (t.metric_id != id) continue;
        if ((!isnan(t.crit_low)  && v <= t.crit_low) ||
            (!isnan(t.crit_high) && v >= t.crit_high))
            flags |= METRIC_FLAG_CRITICAL;
        else if ((!isnan(t.warn_low)  && v <= t.warn_low) ||
                 (!isnan(t.warn_high) && v >= t.warn_high))
            flags |= METRIC_FLAG_WARNING;
        break;
    }
    if (s_night) flags |= METRIC_FLAG_NIGHT;
    return flags;
}

/* ═══════════════════════ diagnostics arbitration ═════════════════════════
 *
 * Both engines transmit to the ECU and both listen on 0x7E8. Left to
 * themselves they would interleave on the bus, and an OBD single-frame reply
 * landing in the middle of an SSM2 multi-frame exchange corrupts both. So a
 * mutex serialises dialogues, and the arbiter decides which engine runs at
 * all: in AUTO the order is listen, then OBD-II, then SSM2 (see
 * diagSsmWanted below). Each engine skips whatever a better-ranked source
 * already delivers, so no value is asked for twice.
 */
static SemaphoreHandle_t s_busMutex   = nullptr;
static volatile uint8_t  s_ssmAnswers = 0;   /**< 0 unknown, 1 yes, 2 no. */
static volatile uint8_t  s_obdAnswers = 0;
static volatile bool     s_obdActive  = false;

/** No frame for this long and the bus counts as down (car off, or wiring). */
static constexpr uint32_t BUS_QUIET_MS = 2000;

bool diagBusAlive() {
    const uint32_t last = s_lastCanMs;
    return last && millis() - last < BUS_QUIET_MS;
}

static volatile bool s_obdSettled = false;   /**< One full OBD pass done. */
bool diagObdSettled() { return s_obdSettled; }

/*
 * AUTO is "listen, then OBD-II, then SSM2". OBD-II always runs (it skips any
 * PID a broadcast frame already delivers). SSM2 starts only once OBD-II has
 * finished a full pass - so it can see what OBD-II covers and ask for the
 * rest - or once OBD-II has proved the ECU will not answer it at all.
 */
bool diagSsmWanted() {
    switch (Cfg.diagMode) {
        case DIAG_MODE_SSM: case DIAG_MODE_BOTH: return true;
        case DIAG_MODE_AUTO: return s_obdAnswers == 2 || s_obdSettled;
        default: return false;
    }
}

bool diagObdWanted() {
    switch (Cfg.diagMode) {
        case DIAG_MODE_OBD: case DIAG_MODE_BOTH: return true;
        case DIAG_MODE_AUTO: return true;
        default: return false;
    }
}

void diagReportSsm(bool answered) { s_ssmAnswers = answered ? 1 : 2; }
void diagReportObd(bool answered) { s_obdAnswers = answered ? 1 : 2; }

/* ═══════════════════════════════ bus guard ═══════════════════════════════
 *
 * In normal mode the controller does more than transmit: it also ACKs every
 * frame and, when it sees a bit or stuff error, signals it with an error
 * frame - which destroys the frame in flight for EVERY module. If our link
 * is marginal (a missing ground, the module's own 120 ohm terminator left
 * fitted, a sample point the bus dislikes) that turns us into a source of
 * errors for the whole car, and other modules log CAN faults.
 *
 * So errors are watched and attributed, the moment they happen (the driver
 * raises an alert): an error counts against us only when our transmit error
 * counter rose with it, which the controller does for errors in frames it was
 * sending and nothing else. (An earlier version also blamed us for any error
 * within 5 ms of one of our frames. With SSM2 and OBD-II running we transmit
 * about every 5 ms, so that blamed us for nearly every error on the bus: in
 * simulation, two unrelated errors a second tripped the guard three times and
 * silenced the master for errors it did not cause.) Three of ours in ten
 * seconds pause every request for 30 s, and so does going bus-off. A third
 * pause in one session puts the controller into listen-only mode, where it
 * cannot drive the bus at all, until the user resumes from the portal, turns
 * the guard off, or the master sleeps and wakes.
 *
 * The controller's mode always follows the settings: listen-only in SILENT,
 * after the guard gave up, or while the bus settles after coming up (see
 * settleLeft), normal otherwise - switched live, no reboot.
 */
static volatile uint32_t s_errWhileTx   = 0;
static volatile uint32_t s_errIdle      = 0;
/** A pause is a start and a length, not an end time: compared as elapsed
 *  time it stays right across the millis() wrap (an end time of 0 compared
 *  as signed would read as "paused" once the uptime passed 24.8 days). */
static volatile uint32_t s_guardPauseAt = 0;
static volatile uint32_t s_guardPauseMs = 0;
static volatile uint8_t  s_guardTrips   = 0;
static volatile bool     s_guardSilent  = false;
/** The receive-error guard's own listen-only latch. Separate from s_guardSilent
 *  so it protects even with the (transmit-side) bus guard turned off - which is
 *  how the car is configured - and clears on its own setting, not the guard's. */
static volatile bool     s_rxGuardSilent = false;
static volatile bool     s_twaiReinstall = false;   /**< RX task reinstalls TWAI. */
static volatile bool     s_twaiWantSilent = false;  /**< ...in this mode.        */
static volatile bool     s_twaiSilentNow  = false;  /**< Mode actually installed. */
static volatile bool     s_twaiShutdown   = false;  /**< Sleep: RX task removes it. */
static volatile bool     s_twaiDown       = false;  /**< ...and it has.            */
/** Held around anything that stops, starts or reinstalls the driver, so the
 *  bus-off recovery can never restart a controller that is being replaced. */
static SemaphoreHandle_t s_twaiCtl = nullptr;

/** @brief Milliseconds of guard pause left (0 = not paused). */
static uint32_t guardPauseLeft() {
    const uint32_t len = s_guardPauseMs, elapsed = millis() - s_guardPauseAt;
    return elapsed < len ? len - elapsed : 0;
}

/**
 * @brief The settle wait: listen only for Cfg.startDelayS after the bus comes
 *        up (BUS_SETTLE_S explains why). Milliseconds left, 0 = over or off.
 *
 * Counted from the bus, not from boot. On the always-live OBD port the master
 * is often already awake when the car is switched on - it was restarted
 * within sleep_idle_s of being switched off - and the bus needs the time just
 * as much then. While the bus is quiet the whole wait is still ahead, so the
 * controller stays listen-only until the car wakes.
 *
 * The setting is read live, so a wait in progress can be lengthened,
 * shortened, or ended with 0. Once a wait is over it stays over until the bus
 * next goes quiet and comes back: raising the setting mid-drive never
 * silences a running bus.
 *
 * @param graceMs Count this much longer: the controller takes a moment to
 *        switch back to normal after the wait (see masterGetStats).
 */
static uint32_t settleLeft(uint32_t graceMs = 0) {
    // The RX task writes s_busUpMs, s_busSession and then s_lastCanMs.
    // Reading them in the reverse order, and the clock last, never pairs a
    // new frame time with the previous bus-up - which would end the wait at
    // once.
    const uint32_t last    = s_lastCanMs;
    const uint32_t session = s_busSession;
    const uint32_t up      = s_busUpMs;
    const uint32_t now     = millis();
    const uint32_t len     = Cfg.startDelayS * 1000UL;
    if (!session || now - last >= BUS_QUIET_MS) return len;   // all still ahead
    if (s_settledSession != session) {
        const uint32_t since = now - up;
        if (since < len) return len - since;
        // Over. Any task may note it: each writes the session it saw, so a
        // late write can never mark a newer session as settled.
        s_settledAtMs    = now;
        s_settledSession = session;
    }
    const uint32_t after = now - s_settledAtMs;
    return after < graceMs ? graceMs - after : 0;
}

uint32_t diagSettleLeftMs() { return settleLeft(); }

bool diagGuardOk() {
    // A listen-only controller cannot transmit whatever the guard setting,
    // and nothing is sent while the bus settles.
    if (s_guardSilent || s_rxGuardSilent || s_twaiSilentNow || settleLeft()) return false;
    if (!Cfg.guardEnabled) return true;
    return guardPauseLeft() == 0;
}

/** What the master was transmitting, as a bitmask, for the evidence log. */
static uint8_t diagActivityMask() {
    uint8_t m = 0;
    if (s_obdActive)                        m |= 0x01;   // OBD-II poller running
    if (settleLeft())                       m |= 0x02;   // in the settle wait
    if (s_guardSilent || s_rxGuardSilent || s_twaiSilentNow) m |= 0x04;  // listen-only
    if (!(m & 0x04) && guardPauseLeft() == 0 && !settleLeft()) m |= 0x08; // free to transmit
    m |= (uint8_t)((Cfg.diagMode & 0x0F) << 4);
    return m;
}

void diagGuardReset() {
    s_guardPauseMs = 0;
    s_guardTrips = 0;
    s_guardSilent = false;           // the guard task switches the mode back
    s_rxGuardSilent = false;         // and leaves the receive-error listen-only
    evLog(EV_RESUME);
    log_i("bus guard: reset by user - requests resumed");
}

/** @brief One guard trip: pause requests, or give up and go listen-only. */
static void guardTrip(const char *why) {
    s_guardTrips++;
    if (s_guardTrips >= Cfg.guardTrips) {
        s_guardSilent = true;
        log_e("bus guard: %s - switching to listen-only for this session", why);
    } else {
        s_guardPauseMs = 0;              // never a half-updated pause
        s_guardPauseAt = millis();
        s_guardPauseMs = Cfg.guardPauseS * 1000UL;
        log_w("bus guard: %s - requests paused for %u s (trip %u of %u)",
              why, Cfg.guardPauseS, s_guardTrips, Cfg.guardTrips);
    }
    evLog(EV_TX_TRIP, s_guardTrips, s_guardSilent ? 1 : 0);
}

bool diagBusLock(uint32_t timeoutMs) {
    if (!diagGuardOk()) return false;
    return s_busMutex && xSemaphoreTake(s_busMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}
void diagBusUnlock() { if (s_busMutex) xSemaphoreGive(s_busMutex); }

/* ═══════════════════════════ CAN ID census ═══════════════════════════════
 *
 * A running tally of every distinct identifier seen on the bus, printed with
 * the heartbeat. This is the single most useful thing to have while bringing
 * a new vehicle up: it turns "is anything happening?" into a list of exactly
 * which modules are talking and how often, which is where RAW_SIGNAL_TABLE
 * entries come from. It also proves the wiring and bitrate are right long
 * before any decoding is attempted - a wrong bitrate yields zero IDs, not
 * wrong ones.
 *
 * Written only by twaiRxTask - a reset from the portal is a request that task
 * carries out - and read by loop(), the portal and the learner. The values
 * are word sized and a torn read costs at most one misprinted count, so this
 * stays lock-free on purpose: taking the metric spinlock on every received
 * frame would be a real cost for a diagnostic.
 */
static constexpr size_t MAX_CENSUS = 128;

/**
 * @brief One distinct CAN identifier: how often, and what it last said.
 *
 * The payload and the "bits that have ever changed" mask are what make the
 * census usable for mapping. An ID count alone says a module is talking; the
 * bytes say where in the frame the moving value sits, and the change mask
 * separates live data from constants and counters at a glance.
 */
struct CanIdCount {
    uint32_t id;         /**< Identifier, as received.             */
    uint32_t count;      /**< Frames seen since reset.             */
    uint8_t  dlc;        /**< Payload length of the last one.      */
    bool     extd;       /**< 29-bit identifier.                   */
    uint8_t  data[8];    /**< Last payload.                        */
    uint8_t  changed[8]; /**< Bits that have toggled since reset.  */
    uint32_t lastMs;     /**< millis() of the last frame.          */
};
static CanIdCount s_census[MAX_CENSUS] = {};
static volatile uint8_t s_censusUsed = 0;

/** @name Bus-load and transmit accounting
 *  Bits on the wire, estimated per frame, so the portal can show load as a
 *  percentage of the bitrate rather than a frame count whose meaning depends
 *  on payload sizes.
 *  @{ */
static volatile uint32_t s_busBitsRx  = 0;
static volatile uint32_t s_canTxCount = 0;
void masterCountTx() { s_canTxCount++; }
/** @} */
static volatile bool    s_censusFull = false;
static volatile bool    s_censusResetReq = false;

/** @brief Carry out a census reset asked for by the portal (RX task only). */
static void censusServiceReset() {
    if (!s_censusResetReq) return;
    s_censusResetReq = false;
    s_censusUsed = 0;
    s_censusFull = false;
    memset(s_census, 0, sizeof(s_census));
}

/**
 * @brief Record one frame in the census.
 * @param msg Frame just received.
 */
static void censusAdd(const twai_message_t &msg) {
    const uint8_t dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
    for (uint8_t i = 0; i < s_censusUsed; i++) {
        CanIdCount &c = s_census[i];
        if (c.id == msg.identifier && c.extd == (bool)msg.extd) {
            c.count++;
            c.dlc = dlc;
            for (uint8_t b = 0; b < dlc; b++) {
                c.changed[b] |= c.data[b] ^ msg.data[b];
                c.data[b] = msg.data[b];
            }
            c.lastMs = millis();
            return;
        }
    }
    if (s_censusUsed >= MAX_CENSUS) { s_censusFull = true; return; }
    CanIdCount &c = s_census[s_censusUsed];
    c.id     = msg.identifier;
    c.count  = 1;
    c.dlc    = dlc;
    c.extd   = msg.extd;
    memset(c.data, 0, 8);
    memcpy(c.data, msg.data, dlc);
    memset(c.changed, 0, 8);
    c.lastMs = millis();
    s_censusUsed++;
}

/** @brief Print the census, busiest identifiers first. */
static void censusPrint() {
    const uint8_t n = s_censusUsed;
    if (n == 0) {
        Serial.println("[bus ] no CAN frames yet - check CANH/CANL, bitrate, "
                       "and that the ignition is on");
        return;
    }
    Serial.printf("[bus ] %u distinct IDs%s\n", n,
                  s_censusFull ? " (census full)" : "");
    // Simple selection print: n is small and this runs once every 5 s.
    bool shown[MAX_CENSUS] = {false};
    const uint8_t lines = n < 12 ? n : 12;
    for (uint8_t k = 0; k < lines; k++) {
        uint8_t best = 0xFF;
        for (uint8_t i = 0; i < n; i++)
            if (!shown[i] && (best == 0xFF || s_census[i].count > s_census[best].count))
                best = i;
        if (best == 0xFF) break;
        shown[best] = true;
        Serial.printf("        %s0x%03X  dlc %u  x%u\n",
                      s_census[best].extd ? "ext " : "", s_census[best].id,
                      s_census[best].dlc, s_census[best].count);
    }
}

/* ═══════════════════════ DBC-style signal extraction ═════════════════════ */

/**
 * @brief Extract a raw bit field from a CAN payload.
 *
 * Intel (little-endian): the 64-bit word is assembled with byte 0 as the
 *   least-significant byte; start_bit is the LSB position of the signal.
 * Motorola (big-endian): the word is assembled with byte 0 as the most-
 *   significant byte; start_bit uses DBC MSB-first numbering (bit 7 of
 *   byte 0 is "7", bit 7 of byte 1 is "15", …).
 *
 * @param d   Frame payload.
 * @param dlc Payload length; bytes beyond it read as zero.
 * @param s   Signal definition giving position, length and byte order.
 * @return The unscaled, unsigned bit field.
 */
static uint64_t extractRaw(const uint8_t *d, uint8_t dlc, const RtSignal &s) {
    uint64_t word = 0;
    if (!s.bigEndian) {
        for (int i = 7; i >= 0; i--)
            word = (word << 8) | (i < dlc ? d[i] : 0);
        word >>= s.startBit;
    } else {
        for (int i = 0; i < 8; i++)
            word = (word << 8) | (i < dlc ? d[i] : 0);
        // DBC Motorola start bit → offset from the MSB of the 64-bit word
        const uint8_t msbOffset = (s.startBit / 8) * 8 + (7 - s.startBit % 8);
        const int shift = 64 - msbOffset - s.bitLength;
        word = shift >= 0 ? (word >> shift) : 0;
    }
    if (s.bitLength >= 64) return word;
    return word & ((1ULL << s.bitLength) - 1);
}

/**
 * @brief Decode one CAN signal to its engineering value.
 * @return `raw * scale + offset`, sign-extended first when `s.isSigned`.
 */
static float decodeSignal(const uint8_t *d, uint8_t dlc, const RtSignal &s) {
    uint64_t raw = extractRaw(d, dlc, s);
    int64_t  val = (int64_t)raw;
    if (s.isSigned && s.bitLength < 64 &&
        (raw & (1ULL << (s.bitLength - 1))))
        val = (int64_t)(raw | (~0ULL << s.bitLength));   // sign extension
    return (float)val * s.scale + s.offset;
}

/* ═══════════════════════ raw signal verification ═════════════════════════
 *
 * A signal in AUTO is decoded on every matching frame and compared with the
 * ECU's own figure for the same quantity. Agreement over enough samples,
 * spread over a wide enough range that coincidence is ruled out, promotes it
 * - and the other AUTO signals in the same frame that have no reference of
 * their own are promoted with it, because the frame layout is now known to
 * be the one the ECU agrees with. Sustained disagreement rejects it.
 *
 * The thresholds are per quantity: 75 rpm is nothing, 75 km/h is everything.
 */
static constexpr size_t MAX_SIGNALS = MAX_RT_SIGNALS;

struct VerifyState {
    uint16_t agree, disagree;
    float    lastVal, lastRef, minVal, maxVal;
    bool     seen, haveRef;
    bool     held;           /**< A comparison waits for the next reference. */
    float    heldRef, heldVal;
    uint32_t refStamp;       /**< When the reference in hand was written.   */
};
static VerifyState s_verify[MAX_SIGNALS] = {};
static std::vector<RtSignal> s_sigs;        /**< RX task's copy of the table. */
static uint32_t s_sigGen = 0xFFFFFFFF;
static volatile bool s_verifyClearReq = false;  /**< Portal: start over.      */

static void verifyLimits(uint16_t metric, float &absTol, float &spread) {
    switch (metric) {
        case METRIC_ID_RPM:           absTol = 75;  spread = 300; return;
        case METRIC_ID_SPEED:         absTol = 4;   spread = 15;  return;
        case METRIC_ID_COOLANT_TEMP:
        case METRIC_ID_OIL_TEMP:
        case METRIC_ID_IAT:           absTol = 4;   spread = 3;   return;
        case METRIC_ID_GEAR:          absTol = 0.5f; spread = 1;  return;
        case METRIC_ID_BATT_VOLTAGE:  absTol = 0.3f; spread = 0.5f; return;
        case METRIC_ID_FUEL_LEVEL:    absTol = 4;   spread = 2;   return;
        case METRIC_ID_MAP:           absTol = 5;   spread = 10;  return;
        case METRIC_ID_NIGHT_SENSE:   absTol = 0.5f; spread = 1;  return;
        default:
            if ((metric & 0xFF00) == 0x2100) { absTol = 0.5f; spread = 1; return; }
            absTol = 6; spread = 15;            // percentages and the rest
    }
}

/** @brief Same decode and same reference: verification progress carries over. */
static bool sameSignal(const RtSignal &a, const RtSignal &b) {
    return a.canId == b.canId && a.extended == b.extended && a.startBit == b.startBit &&
           a.bitLength == b.bitLength && a.bigEndian == b.bigEndian &&
           a.isSigned == b.isSigned && a.scale == b.scale && a.offset == b.offset &&
           a.metricId == b.metricId && a.refMetric == b.refMetric;
}

/**
 * @brief Re-copy the signal table when the portal or the learner changed it.
 *
 * The table changes often while the learner works - every proposal, every
 * rejection - so the counters of a signal still being checked are carried
 * across by identity. Clearing them all on every change would mean a signal
 * could only be verified in a quiet spell long enough to collect all its
 * samples at once, which on a moving car may never come.
 */
static void refreshSignals() {
    if (s_sigGen == Cfg.generation) return;
    static std::vector<RtSignal> prev;
    static VerifyState prevV[MAX_SIGNALS];
    prev.swap(s_sigs);
    memcpy(prevV, s_verify, sizeof(s_verify));
    const bool clear = s_verifyClearReq;
    s_verifyClearReq = false;

    Cfg.lock();
    s_sigGen = Cfg.generation;
    s_sigs   = Cfg.signals;
    Cfg.unlock();
    if (s_sigs.size() > MAX_SIGNALS) s_sigs.resize(MAX_SIGNALS);
    memset(s_verify, 0, sizeof(s_verify));
    if (clear) return;
    for (size_t i = 0; i < s_sigs.size(); i++) {
        if (s_sigs[i].mode != SIG_AUTO) continue;
        for (size_t j = 0; j < prev.size() && j < MAX_SIGNALS; j++)
            if (prev[j].mode == SIG_AUTO && sameSignal(prev[j], s_sigs[i])) {
                s_verify[i] = prevV[j];
                break;
            }
    }
}

/**
 * @brief Write a mode change back to the configuration.
 *
 * Promoting a signal promotes the reference-less AUTO signals of the same
 * frame with it. Skipped if the portal rewrote the table under us; the next
 * frame simply starts over from the new table.
 */
static void setSignalMode(size_t idx, uint8_t mode, bool promoteSiblings) {
    Cfg.lock();
    if (Cfg.generation == s_sigGen && idx < Cfg.signals.size()) {
        Cfg.signals[idx].mode = mode;
        s_sigs[idx].mode = mode;
        if (promoteSiblings) {
            for (size_t j = 0; j < Cfg.signals.size() && j < s_sigs.size(); j++) {
                RtSignal &o = Cfg.signals[j];
                if (j != idx && o.canId == s_sigs[idx].canId &&
                    o.extended == s_sigs[idx].extended &&
                    o.mode == SIG_AUTO && o.refMetric == 0) {
                    o.mode = SIG_VERIFIED;
                    s_sigs[j].mode = SIG_VERIFIED;
                    log_i("signal '%s' verified along with its frame 0x%03X",
                          o.name, o.canId);
                }
            }
        }
        Cfg.generation++;
        s_sigGen = Cfg.generation;      // our own change; no re-copy needed
        Cfg.requestSave();
    }
    Cfg.unlock();
}

/** @brief Read a reference with the moment it was written (not its age, which
 *         moves between two calls to millis()). */
static bool metricRef(uint16_t id, float &value, uint32_t &stampMs, uint8_t &source) {
    bool ok = false;
    portENTER_CRITICAL(&s_metricMux);
    const MasterMetric *m = findSlot(id);
    if (m) { value = m->value; stampMs = m->lastUpdateMs; source = m->source; ok = true; }
    portEXIT_CRITICAL(&s_metricMux);
    return ok;
}

/** @brief One decoded AUTO sample against its reference. */
static void verifySample(size_t i, float val) {
    VerifyState &st = s_verify[i];
    const RtSignal &s = s_sigs[i];
    if (!s.refMetric) return;

    float ref; uint32_t stamp; uint8_t src;
    if (!metricRef(s.refMetric, ref, stamp, src) || millis() - stamp > 600 || src == SRC_RAW) {
        st.haveRef = false;
        st.held = false;                 // a gap breaks the chain of readings
        return;
    }
    st.haveRef = true;
    st.lastRef = ref;
    if (st.held && stamp == st.refStamp) return;   // same reading: already held

    float absTol, spread;
    verifyLimits(s.refMetric, absTol, spread);
    // A learned signal carries tolerances sized to what was actually seen.
    if (s.vtol > 0)    absTol = s.vtol;
    if (s.vspread > 0) spread = s.vspread;

    /*
     * One comparison per reading of the reference, settled by the next one.
     *
     * A requested value is some tens to hundreds of milliseconds old by the
     * time it arrives (a slow PID is asked only every 250-1000 ms); the frame
     * is not. Compared frame by frame, every step in the value leaves a run
     * of frames that already show the new state against a reference that
     * still shows the old one - and nothing in the reference gives it away,
     * because it has not changed yet. For a slowly polled value that was
     * about 10 % disagreement: never enough to reject, always too much to
     * verify, so a correct signal stayed "confirming" for ever and its
     * requests never stopped.
     *
     * So the frame's value is held when a new reading arrives, and counted
     * only if the NEXT reading shows the value did not move in between: then
     * the frame and the reading describe the same state.
     */
    const bool held = st.held;
    const float heldRef = st.heldRef, heldVal = st.heldVal;
    st.held = true;
    st.heldRef = ref;
    st.heldVal = val;
    st.refStamp = stamp;
    if (!held || fabsf(ref - heldRef) > absTol * 0.5f) return;   // it moved: not comparable

    const float tol = max(absTol, 0.03f * fabsf(heldRef));
    const bool agree = fabsf(heldVal - heldRef) <= tol;
    val = heldVal;                       // the value that was compared
    ref = heldRef;

    if (st.agree == 0 && st.disagree == 0) { st.minVal = st.maxVal = val; }
    else { st.minVal = min(st.minVal, val); st.maxVal = max(st.maxVal, val); }
    if (agree) { if (st.agree < 0xFFFF) st.agree++; }
    else       { if (st.disagree < 0xFFFF) st.disagree++; }

    if (st.agree >= Cfg.verifyN && st.disagree * 10 <= st.agree &&
        (st.maxVal - st.minVal) >= spread) {
        log_i("signal '%s' (0x%03X) VERIFIED: %u agree, %u disagree, range %.1f..%.1f",
              s.name, s.canId, st.agree, st.disagree, st.minVal, st.maxVal);
        setSignalMode(i, SIG_VERIFIED, true);
    } else if (st.disagree >= Cfg.verifyN && st.disagree > st.agree * 2) {
        log_w("signal '%s' (0x%03X) REJECTED: %u agree, %u disagree (last %.1f vs ECU %.1f)",
              s.name, s.canId, st.agree, st.disagree, val, ref);
        setSignalMode(i, SIG_REJECTED, false);
    }
}

/* ═══════════════════════════ OBD-II engine ═══════════════════════════════ */

static constexpr uint32_t OBD_FUNCTIONAL_ID = 0x7DF; /**< to every ECU        */
static constexpr uint32_t OBD_PHYSICAL_ID   = 0x7E0; /**< to the engine ECU   */
static constexpr uint32_t OBD_RESPONSE_MIN  = 0x7E8; /**< ECU replies …       */
static constexpr uint32_t OBD_RESPONSE_MAX  = 0x7EF; /**< … 0x7E8-0x7EF       */

static SemaphoreHandle_t s_obdSem      = nullptr;  /**< Reply arrived.      */
static volatile uint8_t  s_obdWaitPid  = 0xFF;     /**< PID in flight.      */
static volatile bool     s_obdWaitPhys = true;     /**< ...sent to 0x7E0.   */
static volatile uint8_t  s_obdNrc      = 0;        /**< ...refused, code.   */
static uint8_t           s_pidSupport[32] = {0};   /**< Bitmap, PID 0-255.  */
static volatile bool     s_pidSupportKnown = false;
static volatile uint8_t  s_pidSupportCount = 0;
/** Supported-PID ranges whose bitmap has arrived: bit n = PID 0x20·n. */
static volatile uint8_t  s_rangeGot = 0;
/**
 * Physical addressing: requests go to the engine ECU alone on 0x7E0 instead
 * of to every ECU on 0x7DF. Mode-01 data lives in the engine ECU, so nothing
 * is lost - but the transmission ECU is no longer asked every question and
 * no longer answers the ones it happens to share, which halves the reply
 * traffic and keeps the diagnostics away from the module whose CAN code
 * started this. Falls back to functional if the ECU ignores physical.
 */
static volatile bool     s_obdPhysical = true;

static bool obdPidSupported(uint8_t pid) {
    if (!s_pidSupportKnown) return pid == 0x00;
    return (s_pidSupport[pid >> 3] >> (pid & 7)) & 1;
}

bool masterPidSupported(uint8_t pid) {
    return s_pidSupportKnown && obdPidSupported(pid);
}

/**
 * @brief Fire one Service-01 single-frame request.
 * @param pid        The PID to request.
 * @param functional Send to 0x7DF (all ECUs) instead of 0x7E0.
 * @return false if the controller would not take the frame.
 * @note Transmits, so it requires TWAI_MODE_NORMAL.
 */
static bool obdRequest(uint8_t pid, bool functional) {
    twai_message_t msg = {};
    msg.identifier       = functional ? OBD_FUNCTIONAL_ID : OBD_PHYSICAL_ID;
    msg.data_length_code = 8;
    msg.data[0] = 0x02;      // ISO-TP single frame, 2 payload bytes
    msg.data[1] = 0x01;      // Service 01: show current data
    msg.data[2] = pid;
    // Pad with 0x00, as J2534 interfaces do and as this ECU pads its replies.
    for (int i = 3; i < 8; i++) msg.data[i] = 0x00;
    if (twai_transmit(&msg, pdMS_TO_TICKS(20)) != ESP_OK) return false;
    masterCountTx();
    return true;
}

/**
 * @brief Parse a 0x7E8-0x7EF reply against @ref OBD_POLL_TABLE.
 *
 * Every row of the table with this PID is decoded, so a PID that returns two
 * values publishes both. Replies to someone else's scanner are decoded too:
 * in silent mode that is free data.
 *
 * @param msg Received frame, already known to be in the response ID range.
 * @note Single-frame only; every Mode-01 PID fits in one.
 */
static void handleObdResponse(const twai_message_t &msg) {
    if (msg.data_length_code < 4)   return;
    if ((msg.data[0] & 0xF0) != 0)  return;    // single frame only
    const uint8_t len = msg.data[0] & 0x0F;    // service + PID + data bytes

    /*
     * Negative response to Service 01 (7F 01 code). Only the engine ECU's, to
     * a physical request, ends the wait: to a functional one another module
     * may refuse a PID the engine ECU is about to answer. 0x78 means the
     * answer is still coming, so the wait goes on.
     */
    if (len >= 3 && msg.data[1] == 0x7F && msg.data[2] == 0x01) {
        if (msg.data[3] != 0x78 && s_obdWaitPhys && msg.identifier == OBD_RESPONSE_MIN &&
            s_obdWaitPid != 0xFF && s_obdSem) {
            s_obdNrc = msg.data[3] ? msg.data[3] : 0xFF;
            xSemaphoreGive(s_obdSem);
        }
        return;
    }
    if (len < 3 || msg.data[1] != 0x41) return;
    const uint8_t pid   = msg.data[2];
    const uint8_t avail = (uint8_t)min((int)len - 2, (int)msg.data_length_code - 3);
    const uint8_t *d    = &msg.data[3];

    if ((pid & 0x1F) == 0 && pid <= 0xC0) {
        if (avail < 4) return;
        /*
         * Supported-PID bitmap: bit 7 of A is PID base+1 … bit 0 of D is
         * base+32. Bits are only ever SET: with functional addressing the
         * transmission ECU answers too, and its much shorter list must not
         * erase what the engine ECU reported.
         */
        for (uint8_t i = 0; i < 32; i++) {
            if (!((d[i / 8] >> (7 - i % 8)) & 1)) continue;
            const uint16_t p = pid + 1 + i;
            if (p <= 0xFF) s_pidSupport[p >> 3] |= (1 << (p & 7));
        }
        // Only the engine ECU's list completes a range: the transmission
        // ECU's shorter one answering first must not stand in for it.
        if (msg.identifier == OBD_RESPONSE_MIN) s_rangeGot |= (uint8_t)(1 << (pid >> 5));
    } else {
        bool any = false;
        for (const auto &def : OBD_POLL_TABLE) {
            if (def.pid != pid || def.off + def.bytes > avail) continue;
            uint32_t raw = 0;
            for (uint8_t b = 0; b < def.bytes; b++) raw = (raw << 8) | d[def.off + b];
            raw >>= def.shift;
            if (def.mask) raw &= def.mask;
            int64_t v = raw;
            if (def.is_signed && !def.mask) {
                const uint8_t nb = def.bytes * 8 - def.shift;
                if (raw & (1UL << (nb - 1))) v -= (int64_t)1 << nb;
            }
            publishMetric(def.metric_id, (float)v * def.scale + def.offset, SRC_OBD);
            any = true;
        }
        if (any) s_obdResponses++;
    }
    if (pid == s_obdWaitPid && s_obdSem) xSemaphoreGive(s_obdSem);
}

/** @brief How one request went. */
enum AskResult : uint8_t {
    ASK_OK,        /**< The ECU answered.                                  */
    ASK_NO_REPLY,  /**< Nothing within the timeout.                        */
    ASK_NRC,       /**< The ECU refused it (negative response).            */
    ASK_BUSY,      /**< Never sent: bus held, guard up, or TX refused.     */
};

/**
 * @brief Send one request and wait for its reply, holding the bus.
 *
 * "Busy" is kept apart from "no reply": a request that never left must not
 * count against the ECU, or SSM2 holding the bus for a moment would look
 * like an ECU that does not speak OBD-II.
 */
static AskResult obdAsk(uint8_t pid, uint32_t waitMs, bool functional) {
    if (!diagBusLock(250)) return ASK_BUSY;
    xSemaphoreTake(s_obdSem, 0);            // clear a stale give
    s_obdNrc = 0;
    s_obdWaitPhys = !functional;
    s_obdWaitPid = pid;
    const bool sent = obdRequest(pid, functional);
    const bool got = sent && xSemaphoreTake(s_obdSem, pdMS_TO_TICKS(waitMs)) == pdTRUE;
    s_obdWaitPid = 0xFF;
    diagBusUnlock();
    if (!sent) return ASK_BUSY;
    if (!got)  return ASK_NO_REPLY;
    return s_obdNrc ? ASK_NRC : ASK_OK;
}
static AskResult obdAsk(uint8_t pid, uint32_t waitMs) {
    return obdAsk(pid, waitMs, !s_obdPhysical);
}

/**
 * @brief Fetch every supported-PID bitmap after 0x00 that is known to exist
 *        and has not arrived yet, in order (each one says whether the next
 *        exists). A transient miss is retried, and whatever is still missing
 *        is fetched on a later pass instead of being lost for the session.
 * @return true once every existing range is in.
 */
static bool obdProbeRanges() {
    for (uint16_t base = 0x20; base <= 0xC0; base += 0x20) {
        // Bit 0 of the previous range's D says whether this one exists.
        if (!((s_pidSupport[base >> 3] >> (base & 7)) & 1)) break;
        if (s_rangeGot & (1 << (base >> 5))) continue;
        bool got = false;
        for (int attempt = 0; attempt < 3 && !got; attempt++) {
            const AskResult r = obdAsk((uint8_t)base, 150);
            got = r == ASK_OK && (s_rangeGot & (1 << (base >> 5)));
            if (r == ASK_NRC) break;           // refused: it will not change
        }
        if (!got) return false;
    }
    return true;
}

/** @brief Supported PIDs, and the count the portal shows. */
static void obdCountSupported() {
    uint8_t cnt = 0;
    for (int p = 1; p < 256; p++)
        if ((s_pidSupport[p >> 3] >> (p & 7)) & 1) cnt++;
    s_pidSupportCount = cnt;
}

/**
 * @brief Read the supported-PID bitmaps 0x00, 0x20 … 0xC0.
 *
 * Asks the engine ECU directly first. If it will not answer a physical
 * request, asks everyone and stays functional from then on.
 *
 * @return ASK_OK when the ECU answered PID 0x00, ASK_BUSY when the question
 *         could not be put, ASK_NO_REPLY when nobody answered it.
 */
/** A physical (0x7E0) request has answered this session: once true, the probe
 *  never reverts to functional 0x7DF, which would reach the transmission ECU. */
static bool    s_physicalConfirmed = false;
static uint8_t s_physMisses = 0;   /**< Consecutive physical probe misses (auto). */

static AskResult obdProbe(bool &rangesComplete) {
    // Addressing: 1 = engine ECU only, 2 = all ECUs, 0 = try the first.
    if (Cfg.obdAddressing == 2) s_obdPhysical = false;
    else if (Cfg.obdAddressing == 1) s_obdPhysical = true;
    AskResult r;
    if (Cfg.obdAddressing == 2) {
        r = obdAsk(0x00, 150, true);
        if (r != ASK_OK) return r;
    } else {
        r = obdAsk(0x00, 150, false);           // the engine ECU directly, on 0x7E0
        if (r == ASK_OK) { s_physicalConfirmed = true; s_physMisses = 0; }
        else if (r == ASK_BUSY || Cfg.obdAddressing == 1) return r;
        // Never revert to functional once physical has worked, and give physical
        // several tries before ever addressing every ECU - a transient miss at
        // startup must not switch us to 0x7DF, which the transmission ECU answers.
        else if (s_physicalConfirmed || ++s_physMisses < 4) return r;
        else {
            r = obdAsk(0x00, 150, true);        // last resort: functional 0x7DF
            if (r != ASK_OK) return r;
            s_obdPhysical = false;
            log_w("OBD-II: engine ECU ignored %u physical probes - falling back to 0x7DF",
                  s_physMisses);
        }
    }
    rangesComplete = obdProbeRanges();
    obdCountSupported();
    s_pidSupportKnown = true;
    log_i("OBD-II: ECU supports %u PIDs (%s addressing)%s", s_pidSupportCount,
          s_obdPhysical ? "physical 0x7E0" : "functional 0x7DF",
          rangesComplete ? "" : " - some ranges still to read");
    return ASK_OK;
}

/** @brief Is every value this PID returns already arriving from a better
 *         source?  Then asking for it again is pure bus time. */
static bool obdPidCovered(uint8_t pid) {
    bool anyRow = false;
    for (const auto &def : OBD_POLL_TABLE) {
        if (def.pid != pid) continue;
        anyRow = true;
        // Cfg.coverMs (2.5 s by default): longer than the slowest refresh of
        // a broadcast value, so a slow-but-live one is never taken for missing.
        if (!metricCoveredAbove(def.metric_id, SRC_OBD, Cfg.coverMs)) return false;
    }
    return anyRow;
}

/**
 * @brief PID scheduler with per-PID periods, one request in flight at a time.
 *
 * Waits for each reply rather than a fixed gap, so a quick ECU is polled
 * quickly and a slow one is never flooded. Learns the supported-PID bitmap
 * first and only ever asks for what the ECU claims, then skips anything a
 * better source already delivers.
 *
 * A PID the ECU claims but does not answer (or refuses) is asked less and
 * less often - down to once every 10 s - instead of costing a full timeout
 * every period; one answer puts it straight back on its normal period. With
 * the ECU asleep and the bus still awake that turns a steady stream of
 * unanswered requests into a trickle.
 */
/** @name OBD-II request pacing
 *  P2CAN cool-down after a timeout, and the overall request budget. Kept as a
 *  start and a length, not an end time, so they read right across the millis()
 *  wrap (see the guard pause for the same reasoning).
 *  @{ */
static uint32_t s_obdCooldownAt = 0;   /**< When the unanswered request was sent... */
static uint32_t s_obdCooldownMs = 0;   /**< ...and how long to hold off after it.    */
static uint32_t s_reqLastSendMs = 0;   /**< Last OBD-II request, for req_max_hz.     */
/** @} */

static void obdPollTask(void *) {
    static uint32_t nextDue[256] = {};
    static uint8_t  misses[256]  = {};
    std::vector<RtPid> pids;
    uint32_t pidGen = 0xFFFFFFFF;
    uint32_t lastProbe = 0, lastRanges = 0;
    uint8_t  probeFails = 0, rangeTries = 0;
    bool     rangesComplete = true;

    for (;;) {
        if (!diagObdWanted() || !diagBusAlive() || !diagGuardOk()) {
            s_obdActive = false;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (pidGen != Cfg.generation) {
            Cfg.lock(); pidGen = Cfg.generation; pids = Cfg.pids; Cfg.unlock();
        }
        // A fixed addressing choice from the portal applies at once; "auto"
        // keeps whatever the probe found.
        if (Cfg.obdAddressing == 1) s_obdPhysical = true;
        else if (Cfg.obdAddressing == 2) s_obdPhysical = false;

        if (!s_pidSupportKnown) {
            s_obdActive = false;
            const uint32_t now = millis();
            if (now - lastProbe >= (probeFails < 3 ? 1000u : 10000u)) {
                lastProbe = lastRanges = now;
                const AskResult r = obdProbe(rangesComplete);
                if (r == ASK_OK) { probeFails = 0; diagReportObd(true); }
                else if (r != ASK_BUSY) {
                    probeFails = (uint8_t)min(probeFails + 1, 200);
                    if (probeFails == 3) {
                        diagReportObd(false);
                        log_w("OBD-II: no answer to PID 0x00 three times - retrying every 10 s");
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        s_obdActive = true;

        // Bitmap ranges that did not arrive at the probe: try again now and
        // then, so PIDs above 0x20 are not lost to one missed reply.
        if (!rangesComplete && millis() - lastRanges >= 15000) {
            lastRanges = millis();
            rangesComplete = obdProbeRanges();
            obdCountSupported();
            if (rangesComplete)
                log_i("OBD-II: all PID ranges read - ECU supports %u PIDs", s_pidSupportCount);
            else if (++rangeTries >= 20) {           // 5 minutes: it is not coming
                rangesComplete = true;
                log_w("OBD-II: some PID ranges never arrived - using the %u PIDs known",
                      s_pidSupportCount);
            }
        }

        bool sentAny = false;
        for (const auto &p : pids) {
            if (!p.enabled || !obdPidSupported(p.pid)) continue;
            const uint32_t now = millis();
            if ((int32_t)(now - nextDue[p.pid]) < 0) continue;
            if (obdPidCovered(p.pid)) { nextDue[p.pid] = now + 1000; continue; }
            // Pace before sending: the overall request budget (req_max_hz, 0 =
            // off) spreads requests over time, and the post-timeout P2CAN
            // cool-down (obd_p2can) keeps the next request from landing in the
            // tail of a reply a slow ECU is still sending. Both delay, never drop.
            {
                const uint32_t nowP = millis();
                uint32_t wait = 0;
                if (Cfg.reqMaxHz) {
                    const uint32_t minGap = max((uint32_t)1, (uint32_t)(1000u / Cfg.reqMaxHz));
                    const uint32_t since  = nowP - s_reqLastSendMs;
                    if (since < minGap) wait = minGap - since;
                }
                const uint32_t sinceTimeout = nowP - s_obdCooldownAt;
                if (sinceTimeout < s_obdCooldownMs)
                    wait = max(wait, s_obdCooldownMs - sinceTimeout);
                if (wait) vTaskDelay(pdMS_TO_TICKS(wait));
            }
            const uint32_t sendMs = millis();
            const AskResult r = obdAsk(p.pid, Cfg.obdTimeoutMs);
            s_reqLastSendMs = sendMs;
            if (r == ASK_NO_REPLY || r == ASK_NRC) {
                s_obdCooldownMs = 0;             // never a half-updated hold-off
                s_obdCooldownAt = sendMs;
                s_obdCooldownMs = Cfg.obdP2CanMs;
            } else if (r == ASK_OK) {
                s_obdCooldownMs = 0;             // answered: nothing to wait for
            }
            uint32_t period = p.periodMs ? p.periodMs : 100;
            if (r == ASK_OK) {
                misses[p.pid] = 0;
                masterUpdateDerived();
            } else if (r == ASK_BUSY) {
                period = 20;                 // not the ECU's fault: soon again
            } else {
                if (misses[p.pid] < 255) misses[p.pid]++;
                if (misses[p.pid] >= 3) {
                    const uint32_t back = min((uint32_t)10000,
                                              (uint32_t)500 << min(misses[p.pid] - 3, 5));
                    period = max(period, back);
                }
            }
            nextDue[p.pid] = millis() + period;
            sentAny = true;
            vTaskDelay(pdMS_TO_TICKS(Cfg.obdGapMs ? Cfg.obdGapMs : 1));
        }
        s_obdSettled = true;                 // a full pass: coverage is known
        if (!sentAny) vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ═══════════════════════════ derived channels ════════════════════════════ */

/**
 * @brief Publish the channels that are computed rather than read.
 *
 * Boost in bar for the gauges, AFR from lambda, injector duty from pulse
 * width and RPM, and the electrical channels the displays already have gauges
 * for: the ECU's supply voltage is the 12 V rail, and once the engine turns
 * it is the alternator's output. Each has an SSM2 route and an OBD-II route,
 * so the gauges work whichever engine is talking.
 */
void masterUpdateDerived() {
    float v, v2; uint32_t age, age2; uint8_t src, src2;
    const uint32_t FRESH = 600;

    if (metricLookup(METRIC_ID_SSM_MAP_REL, v, age, src) && age < FRESH)
        publishMetric(METRIC_ID_BOOST, v / 100.0f, SRC_DERIVED);
    else if (metricLookup(METRIC_ID_MAP, v, age, src) && age < FRESH &&
             ((metricLookup(METRIC_ID_SSM_ATMOS, v2, age2, src2) && age2 < 5000) ||
              (metricLookup(METRIC_ID_BARO,      v2, age2, src2) && age2 < 5000)))
        publishMetric(METRIC_ID_BOOST, (v - v2) / 100.0f, SRC_DERIVED);

    static const uint16_t LAMBDAS[] = { METRIC_ID_SSM_LAMBDA, METRIC_ID_WB_B1S1_LAMBDA,
                                        METRIC_ID_WBC_B1S1_LAMBDA };
    for (uint16_t id : LAMBDAS)
        if (metricLookup(id, v, age, src) && age < FRESH && v > 0.3f && v < 3.0f) {
            publishMetric(METRIC_ID_AFR, v * 14.7f, SRC_DERIVED);
            break;
        }

    float rpm = 0; bool haveRpm = false;
    if (metricLookup(METRIC_ID_RPM, rpm, age, src) && age < FRESH) haveRpm = true;

    if (haveRpm && metricLookup(METRIC_ID_SSM_INJ_PW1, v, age, src) && age < FRESH)
        publishMetric(METRIC_ID_SSM_INJ_DUTY, rpm * v / 1200.0f, SRC_DERIVED);

    if (metricLookup(METRIC_ID_BATT_VOLTAGE, v, age, src) && age < 1500) {
        publishMetric(METRIC_ID_RAIL_12V, v, SRC_DERIVED);
        if (haveRpm && rpm > 400.0f)
            publishMetric(METRIC_ID_ALT_VOLTAGE, v, SRC_DERIVED);
    }
}

static void setupTwai(bool silent);

/** @brief Sleep the master when the bus has been quiet long enough.
 *  Defined further down, next to the rest of the sleep handling. */
static void sleepCheck();

/* ═══════════════════════ CAN TX line held recessive ══════════════════════
 *
 * A CAN bus idles recessive (high) and is pulled dominant (low) only while a
 * node transmits. Nothing here holds GPIO CAN_TX_GPIO high across a reset or a
 * brown-out, so if the transceiver's TX input floats or is driven low while the
 * ESP32 restarts - which is exactly what a cranking voltage dip can cause - the
 * transceiver drives the bus dominant and jams every module, the transmission
 * ECU included. The window before this firmware runs can only be covered by an
 * external pull-up on the TX line (see the flash checklist); what firmware can
 * do is drive the pin recessive the instant it starts, and latch it recessive
 * across deep sleep. Opt out of the sleep latch with tx_hold.
 */
static void txRecessiveBoot() {
    gpio_hold_dis((gpio_num_t)CAN_TX_GPIO);   // release any latch from before sleep
    pinMode(CAN_TX_GPIO, OUTPUT);
    digitalWrite(CAN_TX_GPIO, HIGH);          // recessive = bus idle
    gpio_pullup_en((gpio_num_t)CAN_TX_GPIO);  // and pulled up should the pin float
}
static void txRecessiveForSleep() {
    if (!Cfg.txRecessiveHold) return;
    pinMode(CAN_TX_GPIO, OUTPUT);
    digitalWrite(CAN_TX_GPIO, HIGH);
    gpio_hold_en((gpio_num_t)CAN_TX_GPIO);    // latch it high for the whole sleep
    gpio_deep_sleep_hold_en();
}

/* ═══════════════════════════ FreeRTOS tasks ══════════════════════════════ */

/**
 * @brief Blocking CAN receiver: diagnostic replies plus the raw signal table.
 *
 * Pinned to core 1 so CAN timing never contends with the Wi-Fi stack.
 * Never returns.
 */
static void twaiRxTask(void *) {
    twai_message_t msg;
    for (;;) {
        if (s_twaiShutdown) {
            // Going to sleep: the driver goes, and so does this task's use of it.
            xSemaphoreTake(s_twaiCtl, portMAX_DELAY);
            twai_stop();
            twai_driver_uninstall();
            s_twaiDown = true;
            xSemaphoreGive(s_twaiCtl);
            for (;;) vTaskDelay(portMAX_DELAY);
        }
        if (s_twaiReinstall) {
            // Only this task consumes the driver's queue, so only it may
            // tear the driver down and bring it back in another mode.
            xSemaphoreTake(s_twaiCtl, portMAX_DELAY);
            s_twaiReinstall = false;
            twai_stop();
            twai_driver_uninstall();
            setupTwai(s_twaiWantSilent);
            xSemaphoreGive(s_twaiCtl);
        }
        censusServiceReset();
        const esp_err_t rr = twai_receive(&msg, pdMS_TO_TICKS(200));
        if (rr == ESP_ERR_INVALID_STATE) {
            // No driver (install failed): returning at once, this loop would
            // hold core 1 at top priority and starve everything else on it.
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (rr != ESP_OK) continue;
        s_canFramesRx++;
        const uint32_t rxMs = millis();
        // The first frame since boot, or after the bus was quiet, is the bus
        // coming up: the settle wait counts from it. Written before
        // s_lastCanMs, since settleLeft() reads them in the other order.
        if (!s_busSession || rxMs - s_lastCanMs >= BUS_QUIET_MS) {
            s_busUpMs    = rxMs;
            s_busSession = s_busSession + 1;
            evLog(EV_BUS_UP, (uint16_t)s_busSession);
        }
        s_lastCanMs = rxMs;          // the sleep timer's only input
        // SOF..EOF + IFS is 47 bits standard, 67 extended, plus ~10% stuffing.
        s_busBitsRx += ((msg.extd ? 67 : 47) + 8 * msg.data_length_code) * 11 / 10;
        censusAdd(msg);
        if (ssm2FeedFrame(msg)) continue;   // mid-exchange SSM2 reply

        if (!msg.extd && msg.identifier >= OBD_RESPONSE_MIN &&
            msg.identifier <= OBD_RESPONSE_MAX) {
            handleObdResponse(msg);
            continue;
        }

        /*
         * The runtime table, not the compiled one. Editing a signal in the
         * portal has to take effect on the next frame - the whole point is
         * finding a value by trial while sitting in the car, and a rebuild
         * between attempts would make that unusable.
         */
        refreshSignals();
        for (size_t i = 0; i < s_sigs.size(); i++) {
            const RtSignal &rs = s_sigs[i];
            if (rs.mode == SIG_OFF || rs.mode == SIG_REJECTED) continue;
            if (rs.canId != msg.identifier || rs.extended != (bool)msg.extd)
                continue;
            const float val = decodeSignal(msg.data, msg.data_length_code, rs);
            s_verify[i].seen = true;
            s_verify[i].lastVal = val;
            if (rs.publishes()) publishMetric(rs.metricId, val, SRC_RAW);
            else                verifySample(i, val);
        }
    }
}

/**
 * @brief 25 Hz firehose. Broadcasts every metric whose value changed since
 *        its last frame, plus a keep-alive for the ones that did not, as a
 *        MULTI-FRAME BURST: as many ≤250-byte frames as needed, each one a
 *        complete valid packet with its own sequence_id. Slaves merge frames
 *        by metric_id, so the total channel count is unbounded.
 *
 * Change-driven rather than everything-every-time: with a hundred channels
 * in the store and most of them temperatures, sending only what moved cuts
 * the radio traffic to a fraction and leaves the fast channels their full
 * cadence. A night-flag change forces a full burst, since it rides on every
 * entry's flags.
 */
static void broadcastTask(void *) {
    static const uint8_t BCAST[6] = TELEMETRY_BROADCAST_ADDR;
    static uint32_t seq = 0;
    MasterTelemetryPacket pkt;
    MetricEntry fresh[MAX_MASTER_METRICS];
    uint8_t     slotOf[MAX_MASTER_METRICS];
    bool lastNight = false;

    for (;;) {
        // Derived channels from whatever is live now. The engines also do
        // this after each reply, but values read from the bus arrive with no
        // reply at all - listening only, boost and AFR would otherwise freeze.
        masterUpdateDerived();

        const uint32_t now = millis();
        const bool night = s_night;
        const bool forceAll = night != lastNight;
        lastNight = night;

        // Snapshot under lock (fast copy, no radio inside lock)
        size_t total = 0;
        portENTER_CRITICAL(&s_metricMux);
        for (size_t i = 0; i < MAX_MASTER_METRICS; i++) {
            MasterMetric &m = s_metrics[i];
            if (!m.used || now - m.lastUpdateMs > Cfg.metricTtlMs) continue;
            const bool due = forceAll || !m.everSent || m.value != m.sentValue ||
                             now - m.sentMs >= Cfg.keepaliveMs;
            if (!due) continue;
            fresh[total].metric_id = m.id;
            fresh[total].value     = m.value;
            fresh[total].flags     = 0;      // flags computed outside the lock
            slotOf[total] = (uint8_t)i;
            total++;
        }
        portEXIT_CRITICAL(&s_metricMux);

        for (size_t i = 0; i < total; i++)
            fresh[i].flags = computeFlags(fresh[i].metric_id, fresh[i].value);

        // Burst: chunk into frames of ≤ TELEMETRY_MAX_METRICS entries
        for (size_t off = 0; off < total; off += TELEMETRY_MAX_METRICS) {
            const uint8_t n =
                (uint8_t)min(total - off, (size_t)TELEMETRY_MAX_METRICS);
            pkt.sequence_id  = seq++;
            pkt.timestamp_ms = now;
            pkt.metric_count = n;
            memcpy(pkt.metrics, &fresh[off], n * sizeof(MetricEntry));

            const bool sent = esp_now_send(BCAST, (const uint8_t *)&pkt,
                                           TELEMETRY_PACKET_SIZE(n)) == ESP_OK;
            if (sent) { s_espnowFrames++; s_espnowMetrics += n; }
            else      { s_espnowFailed++; }

            if (sent) {
                portENTER_CRITICAL(&s_metricMux);
                for (size_t k = 0; k < n; k++) {
                    MasterMetric &m = s_metrics[slotOf[off + k]];
                    m.sentValue = fresh[off + k].value;
                    m.sentMs    = now;
                    m.everSent  = true;
                }
                portEXIT_CRITICAL(&s_metricMux);
            }
            if (total > TELEMETRY_MAX_METRICS)
                vTaskDelay(1);               // let the radio drain the burst
        }
        vTaskDelay(pdMS_TO_TICKS(Cfg.broadcastMs));
    }
}

/**
 * @brief Night detection (LDR or vehicle data), bus-health metrics, sleep,
 *        and the TWAI health watchdog.
 *
 * The bus-off recovery here is what stops a transient short or a derailed bus
 * from permanently bricking the bridge. Never returns.
 */
static void housekeepingTask(void *) {
    for (;;) {
        if (Cfg.nightSource == NIGHT_SOURCE_LDR) {
            const int adc = analogRead(NIGHT_LDR_GPIO);
            if      (adc < Cfg.ldrDark)  s_night = true;
            else if (adc > Cfg.ldrLight) s_night = false;
            publishMetric(METRIC_ID_NIGHT_SENSE, s_night ? 1.0f : 0.0f, SRC_DERIVED);
        } else if (Cfg.nightSource == NIGHT_SOURCE_VEHICLE) {
            // The light switch itself if it is known (learned from the bus or
            // read over SSM2), else whatever else publishes the night flag.
            float v; uint32_t age; uint8_t src;
            if (metricLookup(METRIC_ID_SW_LIGHTS, v, age, src) && age < 3000)
                s_night = v > 0.5f;
            else if (metricLookup(METRIC_ID_NIGHT_SENSE, v, age, src) && age < 3000)
                s_night = v > 0.5f;
        } else {
            s_night = false;
        }
        publishMetric(METRIC_ID_MASTER_UPTIME, millis() / 1000.0f, SRC_DERIVED);

        /*
         * Bus health as telemetry, not just serial.
         *
         * Rates rather than totals: a total that stops rising is easy to
         * misread as a live value on a gauge, whereas a rate that falls to
         * zero is unmistakable. Computed over the interval between passes so
         * it stays correct if the task period ever changes.
         */
        static uint32_t lastRxCount = 0, lastObdCount = 0, lastSsmCount = 0, lastRateMs = 0;
        const uint32_t nowMs = millis();
        if (lastRateMs && nowMs > lastRateMs) {
            const float dt = (nowMs - lastRateMs) / 1000.0f;
            publishMetric(METRIC_ID_MASTER_CAN_RX,
                          (s_canFramesRx  - lastRxCount)  / dt, SRC_DERIVED);
            publishMetric(METRIC_ID_MASTER_OBD_RX,
                          (s_obdResponses - lastObdCount) / dt, SRC_DERIVED);
            publishMetric(METRIC_ID_MASTER_SSM_RX,
                          (ssm2Responses() - lastSsmCount) / dt, SRC_DERIVED);
        }
        lastRxCount  = s_canFramesRx;
        lastObdCount = s_obdResponses;
        lastSsmCount = ssm2Responses();
        lastRateMs   = nowMs;
        publishMetric(METRIC_ID_MASTER_CAN_IDS, (float)s_censusUsed, SRC_DERIVED);

        sleepCheck();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/**
 * @brief The bus guard, the controller's mode, and bus-off recovery.
 *
 * Woken by the driver's alerts the moment a bus error happens, so an error
 * is judged against what we were doing at that instant rather than
 * sampled a quarter-second later - by which time the OBD-II poller has
 * nearly always sent something, and every error on the bus would look like
 * ours. Never returns.
 */
static void busGuardTask(void *) {
    uint32_t lastErrCount = 0, lastTec = 0, winStart = 0, winErrs = 0, lastSwitch = 0;
    bool     wasPassive = false, wasSettling = false;
    for (;;) {
        // Mode follows the settings. Turning the guard off also ends a
        // listen-only it imposed: the user has taken responsibility.
        if (!Cfg.guardEnabled && s_guardSilent) {
            s_guardSilent = false;
            s_guardTrips  = 0;
            log_i("bus guard: turned off - leaving listen-only");
        }
        // The receive-error guard has its own switch, so it releases on its own.
        if (!Cfg.rxGuardEnabled && s_rxGuardSilent) {
            s_rxGuardSilent = false;
            log_i("receive-error guard: turned off - leaving listen-only");
        }
        // The settle wait, logged once each way: it explains a quiet start.
        const bool settling = diagBusAlive() && settleLeft();
        if (settling && !wasSettling)
            log_i("bus up - listening only for %u s while it settles", Cfg.startDelayS);
        else if (!settling && wasSettling && diagBusAlive()) {
            log_i("bus settled - requests allowed");
            evLog(EV_SETTLE_END, (uint16_t)Cfg.startDelayS);
        }
        wasSettling = settling;
        const bool wantSilent = Cfg.diagMode == DIAG_MODE_SILENT || s_guardSilent ||
                                s_rxGuardSilent || settleLeft();
        // (At most every 2 s, so a controller that will not install is not
        // retried in a tight loop.)
        if (wantSilent != s_twaiSilentNow && !s_twaiReinstall && millis() - lastSwitch > 2000) {
            lastSwitch = millis();
            s_twaiWantSilent = wantSilent;
            s_twaiReinstall  = true;         // carried out by the RX task
        }

        /*
         * Wait for an alert under the driver lock: uninstalling the driver
         * deletes the semaphore this call blocks on, so a reinstall must
         * never happen while we are inside it. The RX task outranks this
         * one and gets the lock the moment it is released.
         *
         * Bus-off self-healing is done here too: a shorted or derailed bus
         * must not brick the node.
         */
        uint32_t alerts = 0;
        twai_status_info_t st = {};
        bool haveStatus = false, wentBusOff = false;
        xSemaphoreTake(s_twaiCtl, portMAX_DELAY);
        if (twai_read_alerts(&alerts, pdMS_TO_TICKS(100)) != ESP_ERR_INVALID_STATE &&
            twai_get_status_info(&st) == ESP_OK) {
            haveStatus = true;
            if (!s_twaiReinstall) {
                if (st.state == TWAI_STATE_BUS_OFF) {
                    wentBusOff = true;
                    twai_initiate_recovery();
                } else if (st.state == TWAI_STATE_STOPPED) {
                    twai_start();             // recovery finished
                }
            }
        }
        xSemaphoreGive(s_twaiCtl);
        if (!haveStatus) {                   // driver being reinstalled
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        const uint32_t delta = st.bus_error_count - lastErrCount;
        lastErrCount = st.bus_error_count;
        // The transmit error counter only rises for errors in frames this
        // controller was sending: that is ours beyond doubt.
        const bool tecRose = st.tx_error_counter > lastTec;
        lastTec = st.tx_error_counter;
        if (delta && delta < 10000) {        // (a reinstall resets the count)
            if (tecRose) {
                s_errWhileTx += delta;
                const uint32_t now = millis();
                if (now - winStart > Cfg.guardWindowS * 1000UL) { winStart = now; winErrs = 0; }
                winErrs += delta;
                if (Cfg.guardEnabled && winErrs >= Cfg.guardErrs && diagGuardOk()) {
                    char why[64];
                    snprintf(why, sizeof(why), "%u bus errors during our frames in %u s",
                             (unsigned)winErrs, (unsigned)Cfg.guardWindowS);
                    winErrs = 0;
                    guardTrip(why);
                }
            } else {
                s_errIdle += delta;
            }
        }

        /*
         * Receive-error guard. Our controller's receive-error counter climbing
         * into the error-warning region means it is detecting - and, as a
         * normal-mode node, error-flagging - a sustained stream of faults in
         * frames it only receives. Every one of those error flags destroys the
         * frame in flight for EVERY module, which is exactly how the transmission
         * ECU loses the engine ECU's broadcasts (P1718). So drop to listen-only,
         * where the controller cannot signal an error at all. Independent of the
         * transmit-side bus guard and of whether it is on, because a marginal
         * link corrupts the car's own traffic whether or not we are polling. REC
         * winds back down on every good frame, so this only fires on a real
         * storm, not the odd stray error. Opt out with rx_guard.
         */
        if (Cfg.rxGuardEnabled && !s_rxGuardSilent && !s_twaiSilentNow &&
            st.state == TWAI_STATE_RUNNING && st.rx_error_counter >= Cfg.rxGuardRec) {
            s_rxGuardSilent = true;
            evLog(EV_RX_TRIP, (uint16_t)st.rx_error_counter, (uint16_t)st.tx_error_counter);
            log_e("receive-error guard: REC %u - our controller is corrupting other "
                  "nodes' frames; switching to listen-only (resume from the portal)",
                  (unsigned)st.rx_error_counter);
        }

        const bool passive = st.state == TWAI_STATE_RUNNING && st.tx_error_counter >= 128;
        if (passive && !wasPassive)
            log_w("TWAI error-passive (TEC %u, REC %u)", (unsigned)st.tx_error_counter,
                  (unsigned)st.rx_error_counter);
        wasPassive = passive;

        // Bus-off means our own frames failed over and over, so it is a
        // guard trip as well - recovering and carrying on would just put the
        // same errors back on the car's bus.
        if (wentBusOff) {
            log_w("TWAI bus-off - recovering");
            evLog(EV_BUS_OFF, (uint16_t)st.tx_error_counter);
            if (Cfg.guardEnabled && diagGuardOk()) guardTrip("controller went bus-off");
        }
    }
}

/* ═════════════════════════════ deep sleep ════════════════════════════════
 *
 * The OBD port is permanently live, so an idle master is a slow drain on the
 * battery — the kind that is only noticed after a fortnight parked.
 *
 * Bus silence is a better "ignition off" signal than any voltage threshold:
 * when the car sleeps, its modules stop transmitting, and a quiet bus is
 * unambiguous in a way that 12.4 V versus 12.6 V is not.
 *
 * Waking is the elegant half. A CAN line idles recessive (high) and is pulled
 * low by the first dominant bit of any transmission, so an ext0 wake on the
 * receive pin going low means the master comes back the instant *anything*
 * speaks — no polling, no timer, no missed start.
 */

/**
 * @brief Enter deep sleep, arming a wake on the first CAN activity.
 *
 * Does not return: the chip resets on wake, which is why configuration lives
 * on the filesystem rather than in RAM.
 */
static void enterDeepSleep() {
    log_i("bus quiet for %u s - sleeping, will wake on CAN activity",
          Cfg.sleepIdleS);
    Cfg.serviceSave();               // never lose a pending save to sleep
    evLog(EV_SLEEP);
    evSaveNow();                     // and the evidence log
    Serial.flush();

    // Leave the bus and the radio cleanly rather than mid-transfer. The RX
    // task removes the driver: uninstalling it here would free the queue that
    // task is blocked on (within 200 ms it notices the request).
    s_twaiShutdown = true;
    for (int i = 0; i < 50 && !s_twaiDown; i++) vTaskDelay(pdMS_TO_TICKS(10));
    if (!s_twaiDown) {                   // RX task stuck: do it anyway
        xSemaphoreTake(s_twaiCtl, pdMS_TO_TICKS(500));
        twai_stop();
        twai_driver_uninstall();
    }
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);

    // Hold the CAN TX line recessive for the whole sleep, so nothing on our side
    // drives the bus dominant while the chip is down.
    txRecessiveForSleep();

    // Wake when the receive pin is pulled low, i.e. the moment a dominant bit
    // appears. The pin must be RTC-capable for ext0 to watch it while asleep.
    esp_sleep_enable_ext0_wakeup((gpio_num_t)CAN_RX_GPIO, 0);
    esp_deep_sleep_start();
}

/**
 * @brief Decide whether it is time to sleep, and say so before doing it.
 *
 * Held awake while someone is connected to the portal — configuring a device
 * that disappears mid-edit would be its own kind of bug — but only for so
 * long: a phone that remembers the AP and re-joins it every time the car is
 * in the garage must not be able to keep the master awake for a fortnight.
 */
static void sleepCheck() {
    if (!Cfg.sleepEnabled) return;

    const uint32_t idleMs = (uint32_t)Cfg.sleepIdleS * 1000UL;
    const uint32_t quietFor = millis() - (s_lastCanMs ? s_lastCanMs : 0);
    if (s_lastCanMs == 0 && millis() < idleMs) return;   // still early after boot
    if (quietFor < idleMs) return;

    const uint32_t holdMax = max((uint32_t)(idleMs * 3), (uint32_t)300000);
    if (Portal.running() && WiFi.softAPgetStationNum() > 0 && quietFor < holdMax) {
        static uint32_t nagged = 0;
        if (millis() - nagged > 30000) {
            nagged = millis();
            log_i("bus quiet but a portal client is connected - staying awake "
                  "(at most %u s more)", (unsigned)((holdMax - quietFor) / 1000));
        }
        return;
    }
    enterDeepSleep();
}

/** @brief Report why we woke, so a sleep cycle is visible in the log. */
static void logWakeCause() {
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT0:
            log_i("woke on CAN activity");
            break;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            break;                       // ordinary power-on, nothing to say
        default:
            log_i("woke from sleep (cause %d)", (int)esp_sleep_get_wakeup_cause());
            break;
    }
}

/* ═════════════════ read-only window for the portal ══════════════════════
 *
 * The counters, census and metric store stay file-static so nothing outside
 * can write to them. The portal only needs to display them, which these
 * snapshots provide without handing out the underlying storage.
 */

void masterGetStats(MasterStats &out) {
    Ssm2Status ss;
    ssm2GetStatus(ss);
    out.canRx   = s_canFramesRx;
    out.obdRx   = s_obdResponses;
    out.espTx      = s_espnowFrames;
    out.espFail    = s_espnowFailed;
    out.espMetrics = s_espnowMetrics;
    out.lastCanAgeMs = s_lastCanMs ? millis() - s_lastCanMs : 0xFFFFFFFF;
    out.night   = s_night;
    twai_status_info_t st = {};
    out.busOff = (twai_get_status_info(&st) == ESP_OK) &&
                 (st.state == TWAI_STATE_BUS_OFF);
    out.busAlive   = diagBusAlive();
    out.ssmActive  = ss.active;
    out.obdActive  = s_obdActive;
    out.ssmAnswers = s_ssmAnswers;
    out.obdAnswers = s_obdAnswers;
    out.obdSupported = s_pidSupportKnown ? s_pidSupportCount : 0;
    out.obdPhysical  = s_obdPhysical;
    // Listen-only the user chose or the guard imposed - not the settle wait,
    // nor the moment after it while the controller switches back to normal.
    out.silent     = Cfg.diagMode == DIAG_MODE_SILENT || s_guardSilent || s_rxGuardSilent ||
                     (s_twaiSilentNow && !settleLeft(3000));
    out.settleMs   = settleLeft();
    out.errWhileTx = s_errWhileTx;
    out.errIdle    = s_errIdle;
    out.guardTrips = s_guardTrips;
    out.guardSilent = s_guardSilent || s_rxGuardSilent;
    out.rxGuardSilent = s_rxGuardSilent;
    out.guardPauseMs = guardPauseLeft();
    out.busBitsRx  = s_busBitsRx;
    out.canTx      = s_canTxCount;
    out.bitrate    = Cfg.bitrateKbps;
    out.tec = out.rec = 0;
    out.busErrors = out.arbLost = out.txFailed = out.rxMissed = out.rxOverrun = 0;
    if (twai_get_status_info(&st) == ESP_OK) {
        out.tec       = st.tx_error_counter;
        out.rec       = st.rx_error_counter;
        out.busErrors = st.bus_error_count;
        out.arbLost   = st.arb_lost_count;
        out.txFailed  = st.tx_failed_count;
        out.rxMissed  = st.rx_missed_count;
        out.rxOverrun = st.rx_overrun_count;
    }
}

size_t masterCensusRaw(CensusView *out, size_t max) {
    const uint8_t n = s_censusUsed;
    const uint32_t now = millis();
    size_t w = 0;
    for (uint8_t i = 0; i < n && w < max; i++, w++) {
        const CanIdCount &c = s_census[i];
        out[w].id = c.id; out[w].count = c.count; out[w].dlc = c.dlc; out[w].extd = c.extd;
        memcpy(out[w].data, c.data, 8);
        memcpy(out[w].changed, c.changed, 8);
        out[w].ageMs = now - c.lastMs;
    }
    return w;
}

uint64_t masterExtract(const uint8_t *d, uint8_t dlc, uint8_t start, uint8_t len, bool be) {
    RtSignal s;
    s.startBit = start; s.bitLength = len; s.bigEndian = be;
    return extractRaw(d, dlc, s);
}

size_t masterGetCensus(CensusView *out, size_t max) {
    const uint8_t n = s_censusUsed;
    size_t w = 0;
    const uint32_t now = millis();
    for (uint8_t i = 0; i < n && w < max; i++) {
        const CanIdCount &c = s_census[i];
        out[w].id    = c.id;
        out[w].count = c.count;
        out[w].dlc   = c.dlc;
        out[w].extd  = c.extd;
        memcpy(out[w].data, c.data, 8);
        memcpy(out[w].changed, c.changed, 8);
        out[w].ageMs = now - c.lastMs;
        w++;
    }
    // By identifier: rows that stay put are what you can map against, and on
    // CAN the lower ID is also the higher priority, the order a DBC lists them.
    for (size_t a = 0; a + 1 < w; a++)
        for (size_t b = a + 1; b < w; b++)
            if (out[b].id < out[a].id) {
                CensusView t = out[a]; out[a] = out[b]; out[b] = t;
            }
    return w;
}

size_t masterGetMetrics(MetricViewM *out, size_t max) {
    const uint32_t now = millis();
    size_t w = 0;
    portENTER_CRITICAL(&s_metricMux);
    for (auto &m : s_metrics) {
        if (!m.used || w >= max) continue;
        out[w].id     = m.id;
        out[w].value  = m.value;
        out[w].ageMs  = now - m.lastUpdateMs;
        out[w].source = m.source;
        w++;
    }
    portEXIT_CRITICAL(&s_metricMux);
    return w;
}

size_t masterEventLog(EvView *out, size_t max) {
    static EvRecord snap[EV_MAX];
    const size_t n = evSnapshot(snap, EV_MAX);
    size_t w = 0;
    for (size_t i = 0; i < n && w < max; i++) {
        out[w].ms = snap[i].ms; out[w].type = snap[i].type;
        out[w].act = snap[i].act; out[w].a = snap[i].a; out[w].b = snap[i].b;
        w++;
    }
    return w;
}

const char *masterEventName(uint8_t type) { return evName(type); }

void masterEventLogClear() {
    portENTER_CRITICAL(&s_evMux);
    s_evHead = 0;
    s_evCount = 0;
    s_evDirty = true;                // loop() writes the empty log
    portEXIT_CRITICAL(&s_evMux);
}

void masterResetCensus() {
    // Done by the RX task: clearing the table under it while it appends a new
    // identifier would leave a half-written row behind the new count.
    s_censusResetReq = true;
}

void masterResetSignalVerify() {
    Cfg.lock();
    for (auto &s : Cfg.signals)
        if (s.mode == SIG_VERIFIED || s.mode == SIG_REJECTED) s.mode = SIG_AUTO;
    s_verifyClearReq = true;           // counters too, not just the modes
    Cfg.generation++;                  // the RX task re-copies the table
    Cfg.unlock();
    Cfg.requestSave();
}

/* ═══════════════════════════════ bring-up ════════════════════════════════ */

/**
 * @brief Install and start the TWAI driver.
 *
 * Runs in listen-only mode when the diagnostic mode is SILENT. Listen-only is
 * electrically incapable of disturbing the vehicle bus and is the correct
 * mode for first contact with an unfamiliar car.
 */
static void setupTwai(bool silent) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO, (gpio_num_t)CAN_RX_GPIO,
        /*
         * SILENT MODE is simply neither transmitter enabled.
         *
         * Listen-only cannot ACK, so the controller is electrically incapable
         * of disturbing the bus - and because it never spends bus time asking
         * questions, values arrive as fast as the car broadcasts them rather
         * than as fast as we can poll.
         */
        silent ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL
    );
    g.rx_queue_len = 64;          // survive 100%-load 500 kbit/s bursts
    g.tx_queue_len = 16;          // an ISO-TP request is up to 15 frames
    // What the bus guard wakes on (busGuardTask).
    g.alerts_enabled = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF |
                       TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS |
                       TWAI_ALERT_ABOVE_ERR_WARN;

    /* Bitrate is a boot-time decision because it configures the peripheral's
     * timing registers; the portal marks it as needing a reboot for exactly
     * this reason. */
    twai_timing_config_t t;
    switch (Cfg.bitrateKbps) {
        case 125:  t = TWAI_TIMING_CONFIG_125KBITS();  break;
        case 250:  t = TWAI_TIMING_CONFIG_250KBITS();  break;
        case 1000: t = TWAI_TIMING_CONFIG_1MBITS();    break;
        default:   t = TWAI_TIMING_CONFIG_500KBITS();  break;
    }
    /*
     * Sample point. The driver's presets sample at 80 % of the bit; many
     * vehicle buses are specified nearer 87.5 %. A node that samples early
     * on a bus with slow edges sees errors nobody else sees - and in normal
     * mode it signals them, corrupting everyone's frames. Selectable from the
     * portal for exactly that case: 16 time quanta, 14 before the sample.
     */
    if (Cfg.canSample875 && Cfg.bitrateKbps != 1000) {
        t.brp   = 80000 / (16 * Cfg.bitrateKbps);   // APB 80 MHz: 500k -> 10
        t.tseg_1 = 13;
        t.tseg_2 = 2;
        t.sjw    = 2;
        t.triple_sampling = false;
    }
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
        log_e("TWAI init failed — check CAN_TX/RX_GPIO wiring");
        return;
    }
    s_twaiSilentNow = silent;
    log_i("TWAI up (%s mode) at %u kbit/s, sample point %s",
          silent ? "SILENT/listen-only" : "NORMAL", Cfg.bitrateKbps,
          Cfg.canSample875 && Cfg.bitrateKbps != 1000 ? "87.5%" : "80%");
}

/**
 * @brief Bring up Wi-Fi and register the broadcast peer.
 * @note Broadcast frames cannot be encrypted in ESP-NOW, so the peer is
 *       added unencrypted by necessity, not by oversight.
 */
static void setupEspNow() {
    /*
     * The peer must be registered on the interface that actually exists.
     *
     * When the portal is up the radio is already in AP mode on the telemetry
     * channel, and registering the broadcast peer against WIFI_IF_STA then
     * refers to an interface that has been torn down - every send fails with
     * no hint as to why, and both displays go dark. So the interface is chosen
     * from whichever mode is running, and the AP's channel is left alone
     * because the portal already pinned it.
     */
    const bool viaAp = Portal.running();
    if (!viaAp) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(false, true);
        esp_wifi_set_channel(Cfg.wifiChannel, WIFI_SECOND_CHAN_NONE);
    }

    if (esp_now_init() != ESP_OK) {
        log_e("esp_now_init failed");
        return;
    }
    esp_now_peer_info_t peer = {};
    const uint8_t bcast[6] = TELEMETRY_BROADCAST_ADDR;
    memcpy(peer.peer_addr, bcast, 6);
    peer.channel = Cfg.wifiChannel;
    peer.ifidx   = viaAp ? WIFI_IF_AP : WIFI_IF_STA;
    peer.encrypt = false;        // broadcast frames cannot be encrypted
    esp_now_add_peer(&peer);
    /*
     * Print our own MAC. The slaves can filter on it (network.master_mac in
     * their layout) so a second CAN node, or someone else's project on the
     * same channel, cannot inject frames into your gauges. Copying it by hand
     * is the only way to set that filter, so it has to be printed somewhere.
     */
    log_i("ESP-NOW broadcasting on channel %u via %s, this master is %s",
          Cfg.wifiChannel, viaAp ? "AP" : "STA",
          viaAp ? WiFi.softAPmacAddress().c_str()
                : WiFi.macAddress().c_str());
}

/**
 * @brief Bring up the CAN interface, the radio, and the worker tasks.
 *
 * CAN work is pinned to core 1 and radio work to core 0, so a busy bus cannot
 * delay a broadcast and vice versa.
 */
void setup() {
    Serial.begin(115200);
    log_i("CAN Telemetry Master v%s (proto v%d)", FIRMWARE_VERSION,
          TELEMETRY_PROTO_VERSION);

    // Put the CAN TX line into its recessive idle the instant we run, before the
    // driver or anything else can leave it low.
    txRecessiveBoot();

    logWakeCause();

    s_busMutex = xSemaphoreCreateMutex();
    s_obdSem   = xSemaphoreCreateBinary();
    s_twaiCtl  = xSemaphoreCreateMutex();
    s_evSaveMutex = xSemaphoreCreateMutex();

    // Configuration first: TWAI mode, bitrate and the ESP-NOW channel are all
    // decided from it during the bring-up that follows.
    Cfg.begin();
    analogReadResolution(12);

    // The evidence log lives beside the config on the filesystem, so it survives
    // a firmware update. Load the history, then stamp this boot with why we reset
    // - a brown-out at cranking reads very differently from a clean power-on.
    evLoad();
    const esp_reset_reason_t rr = esp_reset_reason();
    evLog(EV_BOOT, (uint16_t)rr, MasterConfig::CFG_VERSION);
    log_i("boot: reset reason %d", (int)rr);

    // Listen-only to begin with while there is a settle wait: it runs from
    // the bus's first frame, and busGuardTask switches to normal after it.
    setupTwai(Cfg.diagMode == DIAG_MODE_SILENT || settleLeft());
    // Portal first: it decides the radio mode and channel, and setupEspNow
    // registers its peer against whichever interface that leaves running.
    Portal.begin();
    setupEspNow();

    // CAN work pinned to core 1; Wi-Fi stack lives on core 0.
    xTaskCreatePinnedToCore(twaiRxTask,      "twai_rx",   6144, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(obdPollTask,     "obd_poll",  4096, nullptr,  5, nullptr, 1);
    xTaskCreatePinnedToCore(broadcastTask,   "broadcast", 6144, nullptr,  8, nullptr, 0);
    xTaskCreatePinnedToCore(housekeepingTask,"housekeep", 4096, nullptr,  3, nullptr, 0);
    xTaskCreatePinnedToCore(busGuardTask,    "busguard",  3072, nullptr,  9, nullptr, 1);
    ssm2Begin();
    learnerBegin();
}

/**
 * @brief 5 s serial heartbeat, portal service, deferred saves.
 *
 * Worth watching during bring-up: `CAN rx` climbing proves the wiring and
 * bitrate are right, and `SSM`/`OBD` climbing proves the ECU is answering.
 */
void loop() {
    static uint32_t last = 0;
    if (millis() - last >= 5000) {
        const uint32_t nowMs = millis();
        const float dt = last ? (nowMs - last) / 1000.0f : 0.0f;
        static uint32_t lastFrames = 0, lastMetrics = 0;
        const float fps = dt ? (s_espnowFrames  - lastFrames)  / dt : 0.0f;
        const float mps = dt ? (s_espnowMetrics - lastMetrics) / dt : 0.0f;
        lastFrames = s_espnowFrames; lastMetrics = s_espnowMetrics;
        last = nowMs;
        Ssm2Status ss;
        ssm2GetStatus(ss);
        /*
         * Frames AND metrics per second. The broadcaster only sends channels
         * whose value moved, so the frame rate is a function of how much the
         * car is doing, not of how healthy the link is - metrics/s is the one
         * to watch.
         */
        Serial.printf("[stats] CAN rx: %u | OBD resp: %u | SSM2: %u ok/%u err "
                      "(%.0f/s, %u addr) | ESP-NOW: %.0f frame/s, %.0f metric/s "
                      "(%u sent, %u fail) | night: %d | heap: %u\n",
                      s_canFramesRx, s_obdResponses, ss.responses, ss.errors,
                      ss.exchPerSec, ss.batch, fps, mps,
                      s_espnowFrames, s_espnowFailed,
                      (int)s_night, ESP.getFreeHeap());
        static const char *MODES[] = {"AUTO", "SSM2", "OBD-II", "BOTH", "SILENT"};
        char settleTxt[48] = "";
        const uint32_t settle = settleLeft();
        if (settle && diagBusAlive())
            snprintf(settleTxt, sizeof(settleTxt), " | bus settling, requests in %u s",
                     (unsigned)((settle + 999) / 1000));
        Serial.printf("[diag ] mode %s%s | SSM2 %s%s | OBD-II %s\n",
                      MODES[Cfg.diagMode > 4 ? 0 : Cfg.diagMode], settleTxt,
                      ss.active ? "active" : (s_ssmAnswers == 2 ? "no answer" : "idle"),
                      ss.initOk ? (String(", ECU ") + ss.ecuId + " (" + ss.supported +
                                   "/" + ss.total + " params)").c_str() : "",
                      s_obdActive ? "active" : (s_obdAnswers == 2 ? "no answer" : "idle"));
        censusPrint();
    }
    Portal.loop();
    Cfg.serviceSave();
    evService();
    delay(10);
}

/** @} */  // end of master group
