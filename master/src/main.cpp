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
#include <algorithm>
#include <vector>
#include <LittleFS.h>
#include "driver/twai.h"
#include "driver/gpio.h"
#include "hal/twai_ll.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

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
    masterBusStall();                            // the flash write holds the CAN interrupt off
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
    // A NaN would also defeat the broadcaster's change test (NaN != NaN) and
    // go out every burst.
    if (!isfinite(value)) return;
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
/** Held around every change to the controller's registers or driver - the
 *  reinstall, the bus-off restart and the passive-mode TEC top-up. Unlike
 *  s_twaiCtl it is never held while waiting for an alert, so a lower-priority
 *  request task gets it at once. Order: s_twaiCtl before s_twaiReg. */
static SemaphoreHandle_t s_twaiReg = nullptr;

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

static void txPassiveTopUp();

bool diagBusLock(uint32_t timeoutMs) {
    if (!diagGuardOk()) return false;
    if (!(s_busMutex && xSemaphoreTake(s_busMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)) return false;
    // Asked again: the guard may have tripped, or listen-only begun, while we
    // waited for the bus.
    if (!diagGuardOk()) {
        xSemaphoreGive(s_busMutex);
        return false;
    }
    txPassiveTopUp();                // nothing of ours in flight now
    return true;
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
    /** Flips of each bit (bit n = byte n/8, bit n%8), counting up and
     *  wrapping at 256. Teach-by-doing reads them twice or more a second and
     *  takes differences: a turn signal flashing inside the ON step counts its
     *  flashes exactly, however slowly the phone polls. */
    uint8_t  edges[64];
    uint32_t lastMs;     /**< millis() of the last frame.          */
    /** @name Missed frames (missAccount)
     *  @{ */
    uint32_t lastUs;      /**< micros() of the last frame.                  */
    uint32_t periodUs;    /**< Its period as learned; 0 = not yet.          */
    uint16_t regular;     /**< Gaps within 30 % of the period.              */
    uint16_t irregular;   /**< Gaps that fit no whole number of periods.    */
    uint32_t blindSeen;   /**< s_blindEpoch at the last frame.              */
    uint32_t stallSeen;   /**< s_stallEpoch at the last frame.              */
    bool     skipNext;    /**< The last frame was timed late: its gap too.  */
    uint32_t missed[2];   /**< Its frames the master did not receive, by
                               mode: [0] normal, [1] listen-only.           */
    uint32_t expected[2]; /**< Its frames due over the same gaps.           */
    /** @} */
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

/* ═════════════════════════ frames the master missed ══════════════════════
 *
 * Most of the car's broadcasts are periodic, so a frame the master misread -
 * rejected by its controller with a bus error while the rest of the car took
 * it - shows as a gap of two or more periods in that ID. Counted per ID, that
 * says whose frames the master cannot read and how often, in listen-only and
 * normal mode alike, without trusting an error counter: with errors let pass
 * (tx_passive) a misread frame is simply dropped, and the gap is all that is
 * left of it.
 *
 * A gap only counts if the master was listening throughout. Anything that
 * makes it deaf for a moment calls masterBusBlind() - a reinstall, a TEC write
 * (reset mode), a flash write (the cache is off, and the CAN interrupt with
 * it), frames dropped for a full queue - and a gap across one is left out,
 * its missing and its expected frames both.
 */
static volatile uint32_t s_blindEpoch = 0;
/** ...and a stall: the frames around it were timed late (a flash write held
 *  the CAN interrupt off, or the queue overflowed), so the gaps next to it say
 *  nothing about the rhythm either. */
static volatile uint32_t s_stallEpoch = 0;
void masterBusBlind() { s_blindEpoch = s_blindEpoch + 1; }
void masterBusStall() { s_stallEpoch = s_stallEpoch + 1; masterBusBlind(); }

/** Frames received and bus errors counted, by mode: [0] normal, [1] listen-only. */
static volatile uint32_t s_rxByMode[2]  = {0, 0};
static volatile uint32_t s_errByMode[2] = {0, 0};

/** Periodic enough to judge: 20 regular gaps, no more than one in eight not. */
static bool idPeriodic(const CanIdCount &c) {
    return c.regular >= 20 && c.irregular * 8u <= c.regular;
}
/** Diagnostic requests and replies come when asked, not on a clock. */
static bool idOnRequest(const CanIdCount &c) {
    return !c.extd && c.id >= 0x7DF && c.id <= 0x7EF;
}

/** No car broadcast repeats faster than this. Frames of one ID closer than
 *  that are a burst out of the receive queue, not their rhythm (see below). */
static constexpr uint32_t MIN_PERIOD_US = 2000;

/**
 * @brief Book the gap before this frame of @p c (see above).
 *
 * Frames are timed as the RX task takes them from the queue, not as they
 * crossed the wire. After a stall - a flash write holding the CAN interrupt
 * off - the queued ones come out microseconds apart, the first of them late,
 * and the gap after it short. None of those gaps is the ID's rhythm: learning
 * from them taught periods of microseconds, after which every real gap looked
 * like a pause and the ID was never judged again. So a gap across a stall, a
 * gap inside a burst, and the gap after either are all skipped.
 *
 * @param silent Listen-only now: the mode the gap is booked under.
 */
static void missAccount(CanIdCount &c, uint32_t nowUs, bool silent) {
    const uint32_t gap = nowUs - c.lastUs;
    const uint32_t epoch = s_blindEpoch, stall = s_stallEpoch;
    const bool blind = epoch != c.blindSeen, stalled = stall != c.stallSeen;
    c.blindSeen = epoch;
    c.stallSeen = stall;
    if (stalled || gap < MIN_PERIOD_US) { c.skipNext = true; return; }   // timed late, or a burst
    if (c.skipNext) { c.skipNext = false; return; }      // it began at a frame timed late
    const uint64_t g = gap, p = c.periodUs;             // 64-bit: slow IDs cannot overflow
    const auto relearn = [&]() { c.periodUs = gap; c.regular = c.irregular = 0; };
    // Learning: a shorter gap is the shorter period, and a far longer one says
    // the first guess was wrong (or the bus paused) - start over from it.
    if (!p || (c.regular < 20 && (g * 10 < p * 7 || g > p * 11))) { relearn(); return; }
    if (g > p * 11) return;                             // a pause, not ours to judge
    if (c.regular == 0xFFFF || c.irregular == 0xFFFF) { // halved together: the ratio counts,
        c.regular /= 2;                                 // and a long drive must not tip it
        c.irregular /= 2;
    }
    const uint8_t m = silent ? 1 : 0;
    if (g * 10 >= p * 7 && g * 10 <= p * 13) {          // on time
        c.regular++;
        c.periodUs = (uint32_t)((int64_t)p + ((int64_t)g - (int64_t)p) / 16);  // its real rate
        if (!blind && idPeriodic(c)) c.expected[m]++;
        return;
    }
    if (g * 10 > p * 13) {
        const uint64_t k = (g + p / 2) / p;             // periods in the gap
        const uint64_t off = g > k * p ? g - k * p : k * p - g;
        if (k >= 2 && k <= 10 && off * 10 <= p * 3) {   // whole periods: k - 1 missing
            if (!blind && idPeriodic(c)) {
                c.missed[m]   += (uint32_t)(k - 1);
                c.expected[m] += (uint32_t)k;
            }
            return;
        }
    }
    // Sooner than its period, or no whole number of them. An ID that keeps
    // doing it has changed its rate, or never had one: learn it again.
    if (++c.irregular * 4 > c.regular) relearn();
}

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
    const uint32_t nowUs = micros();
    for (uint8_t i = 0; i < s_censusUsed; i++) {
        CanIdCount &c = s_census[i];
        if (c.id == msg.identifier && c.extd == (bool)msg.extd) {
            c.count++;
            c.dlc = dlc;
            for (uint8_t b = 0; b < dlc; b++) {
                uint8_t x = c.data[b] ^ msg.data[b];
                c.changed[b] |= x;
                c.data[b] = msg.data[b];
                for (uint8_t *e = &c.edges[8 * b]; x; x >>= 1, e++)
                    if (x & 1) (*e)++;
            }
            c.lastMs = millis();
            if (!idOnRequest(c)) missAccount(c, nowUs, s_twaiSilentNow);
            c.lastUs = nowUs;
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
    memset(c.edges, 0, sizeof(c.edges));
    c.lastMs = millis();
    c.lastUs = nowUs;
    c.periodUs = 0;
    c.regular = c.irregular = 0;
    c.blindSeen = s_blindEpoch;
    c.stallSeen = s_stallEpoch;
    c.skipNext  = false;
    memset(c.missed, 0, sizeof(c.missed));
    memset(c.expected, 0, sizeof(c.expected));
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
    if (s.nBits) {                       // a bit combination: gather its bits
        uint64_t v = 0;
        for (uint8_t i = 0; i < s.nBits; i++) {
            const uint8_t b = s.bitList[i];
            if ((b >> 3) < dlc && ((d[b >> 3] >> (b & 7)) & 1)) v |= 1ULL << i;
        }
        return v;
    }
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
 * @return `raw * scale + offset`, sign-extended first when `s.isSigned`; for a
 *         signal with a value table, the value its raw reading maps to, or NaN
 *         (publishes nothing) for a raw reading that is not in the table.
 */
static float decodeSignal(const uint8_t *d, uint8_t dlc, const RtSignal &s) {
    uint64_t raw = extractRaw(d, dlc, s);
    if (s.nMap) {
        for (uint8_t k = 0; k < s.nMap; k++)
            if (raw == s.mapRaw[k]) return s.mapVal[k];
        return NAN;
    }
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
/** millis() when obdAsk() last put a request on the bus. Pacing counts from
 *  here, not from when the poller asked for the bus: in AUTO an SSM2 exchange
 *  can hold the bus for tens of ms first. */
static uint32_t s_obdSentAtMs = 0;

static AskResult obdAsk(uint8_t pid, uint32_t waitMs, bool functional) {
    if (!diagBusLock(250)) return ASK_BUSY;
    xSemaphoreTake(s_obdSem, 0);            // clear a stale give
    s_obdNrc = 0;
    s_obdWaitPhys = !functional;
    s_obdWaitPid = pid;
    const bool sent = obdRequest(pid, functional);
    if (sent) s_obdSentAtMs = millis();
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
            const AskResult r = obdAsk(p.pid, Cfg.obdTimeoutMs);
            // Both paces count from when the request really went out (see
            // s_obdSentAtMs); a request that never went out counts for neither.
            const uint32_t sendMs = s_obdSentAtMs;
            if (r != ASK_BUSY) s_reqLastSendMs = sendMs;
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

    /*
     * The gear lever and "a door is open", from switches taught one at a
     * time: a lever whose positions are separate bits (P in one byte, R in
     * another), and doors the car reports one by one. A field taught onto the
     * lever or onto "door open" itself wins; these only stand in for it.
     * Body frames can be slow, hence the longer freshness.
     */
    const uint32_t BODY_FRESH = 2500;
    auto rawFresh = [&](uint16_t id) {
        return metricLookup(id, v, age, src) && src == SRC_RAW && age < BODY_FRESH;
    };
    if (!rawFresh(METRIC_ID_GEAR_LEVER)) {
        static const struct { uint16_t id; char c; } POS[] = {
            { METRIC_ID_REVERSE, 'R' }, { METRIC_ID_PARK, 'P' },
            { METRIC_ID_NEUTRAL, 'N' }, { METRIC_ID_DRIVE, 'D' } };
        for (const auto &p : POS)
            if (metricLookup(p.id, v, age, src) && age < BODY_FRESH && v >= 0.5f) {
                publishMetric(METRIC_ID_GEAR_LEVER, (float)p.c, SRC_DERIVED);
                break;
            }
    }
    if (!rawFresh(METRIC_ID_DOOR_OPEN)) {
        static const uint16_t DOORS[] = { METRIC_ID_DOOR_FL, METRIC_ID_DOOR_FR, METRIC_ID_DOOR_RL,
                                          METRIC_ID_DOOR_RR, METRIC_ID_TRUNK, METRIC_ID_HOOD };
        bool known = false, open = false;
        for (uint16_t id : DOORS)
            if (metricLookup(id, v, age, src) && age < BODY_FRESH) {
                known = true;
                open |= v >= 0.5f;
            }
        if (known) publishMetric(METRIC_ID_DOOR_OPEN, open ? 1.0f : 0.0f, SRC_DERIVED);
    }
}

static void setupTwai(bool silent);
static void errKindAdd(uint8_t code, bool silent, uint32_t errors);
static void applyRadioPower();

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
/**
 * @brief Take the TX pad away from the CAN controller and drive it recessive.
 *
 * The level goes into the GPIO latch first and the pad becomes a GPIO output
 * second (gpio_set_direction routes it to the latch, away from the controller's
 * TX signal). The other way round - pinMode, then digitalWrite - the pad shows
 * the latch's reset value, 0 = dominant, for the moment in between.
 * txPadAttach() - or the next twai_driver_install() - routes the pad back to
 * the controller.
 */
static void txPadRecessive() {
    gpio_set_level((gpio_num_t)CAN_TX_GPIO, 1);
    gpio_set_direction((gpio_num_t)CAN_TX_GPIO, GPIO_MODE_OUTPUT);
}
/** Give the TX pad back to the controller: the very call twai_driver_install()
 *  routes it with (after gpio_config, which txPadRecessive() has done). */
static void txPadAttach() {
    esp_rom_gpio_connect_out_signal(CAN_TX_GPIO, TWAI_TX_IDX, false, false);
}
static void txRecessiveBoot() {
    gpio_hold_dis((gpio_num_t)CAN_TX_GPIO);   // release any latch from before sleep
    txPadRecessive();                         // recessive = bus idle
    gpio_pullup_en((gpio_num_t)CAN_TX_GPIO);  // and pulled up should the pin float
}
static void txRecessiveForSleep() {
    if (!Cfg.txRecessiveHold) return;
    txPadRecessive();
    gpio_hold_en((gpio_num_t)CAN_TX_GPIO);    // latch it high for the whole sleep
    gpio_deep_sleep_hold_en();
}

/** @name Passive transmit mode (Cfg.txPassive)
 *  TEC is set this high right after the controller starts in normal mode,
 *  and topped up whenever the node's own successful frames (-1 each) have
 *  brought it down to the low mark. 220 leaves room for four transmit errors
 *  (+8 each) before bus-off at 256.
 *  @{ */
static constexpr uint32_t TEC_PASSIVE     = 220;
static constexpr uint32_t TEC_PASSIVE_LOW = 170;
/** TEC writes of ours - the passive-mode top-ups, the start in normal mode,
 *  the listen-only fix - so busGuardTask never takes the jump for an error in
 *  our own frame. */
static volatile uint32_t s_tecWrites = 0;
/** Set TEC (reset mode for a moment: a frame being received is lost to us).
 *  Only with nothing of ours queued or on the wire - the caller makes sure. */
static void tecSet(uint32_t tec) {
    masterBusBlind();                // a frame arriving now is lost to us
    s_tecWrites = s_tecWrites + 1;
    twai_ll_enter_reset_mode(&TWAI);
    twai_ll_set_tec(&TWAI, tec);
    twai_ll_exit_reset_mode(&TWAI);
}

/**
 * @brief Passive mode: bring TEC back up before a request goes out.
 *
 * Called by diagBusLock() once the caller holds the bus mutex - every request
 * and SSM2 exchange takes it first, so nothing of ours is queued or on the
 * wire. The register lock keeps it clear of a reinstall or a bus-off restart.
 * Our own successful frames wind TEC down by one each; from TEC_PASSIVE to the
 * low mark is fifty frames, so a top-up every few requests keeps it passive.
 */
static void txPassiveTopUp() {
    if (!Cfg.txPassive || s_twaiSilentNow || !s_twaiReg) return;
    if (xSemaphoreTake(s_twaiReg, pdMS_TO_TICKS(40)) != pdTRUE) return;
    twai_status_info_t st;
    if (!s_twaiReinstall && !s_twaiSilentNow && twai_get_status_info(&st) == ESP_OK &&
        st.state == TWAI_STATE_RUNNING && st.msgs_to_tx == 0 && st.tx_error_counter < TEC_PASSIVE_LOW) {
        tecSet(TEC_PASSIVE);
    }
    xSemaphoreGive(s_twaiReg);
}
/** @} */

/**
 * @brief Espressif's listen-only erratum - fixed here, because the Arduino core
 *        this builds on is compiled without ESP-IDF's own fix.
 *
 * On the ESP32, S2, S3 and C3 a TWAI controller in listen-only mode still sends
 * an ACTIVE error flag - six dominant bits, which destroy the frame for every
 * module on the bus - whenever it detects an error in a frame. Listen-only also
 * freezes the error counters, and twai_start() leaves REC at 0, so the
 * controller stays error-active for as long as it listens. On a link where it
 * misreads a frame it destroys every retransmission of it and never backs off:
 * a normal-mode node's REC climbs past 127 within ~15 tries and makes it
 * error-passive (harmless) before the sender's TEC reaches bus-off, but a
 * frozen REC never does, so the SENDER is driven bus-off instead - on this car
 * the engine ECU, whose broadcasts the transmission ECU then loses: P1718. This
 * is why the master showed 100 000+ errors in listen-only and set the MIL faster
 * than the firmware before it, which stayed in normal mode throughout: the
 * settle wait and the receive-error guard had made listen-only the mode it
 * falls back to exactly when the link is bad.
 *
 * ESP-IDF's fix (CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM, "not set" in the
 * Arduino core's sdkconfig) sets REC to 128 before leaving reset mode: the
 * controller is error-passive, its error flags are recessive, and the frozen
 * counter keeps it there. The same is done here straight after twai_start() -
 * REC can only be written in reset mode. And setupTwai() takes the TX pad away
 * from the controller for as long as it listens, so nothing it does can reach
 * the transceiver whatever the silicon gets up to.
 */
static void listenOnlyErratumFix() {
    /*
     * TEC as well as REC. REC alone did not hold on the car: with errors
     * streaming, the portal showed REC back at 0 in listen-only - a good frame
     * winds REC down (from above 127 straight to 119..127, then by one) - and
     * an error-active controller whose error flags the detached TX pad keeps
     * off the bus reads its own flag back recessive, calls that a bit error,
     * flags again, and counts every round: the million errors a drive. TEC
     * moves only when the controller transmits, which it never does in
     * listen-only, so TEC 128 keeps it error-passive for good.
     */
    s_tecWrites = s_tecWrites + 1;
    twai_ll_enter_reset_mode(&TWAI);
    twai_ll_set_tec(&TWAI, 128);
    twai_ll_set_rec(&TWAI, 128);
    twai_ll_exit_reset_mode(&TWAI);
    const uint32_t tec = twai_ll_get_tec(&TWAI);
    if (tec < 128) log_e("listen-only erratum fix did not take (TEC %u)", (unsigned)tec);
}

/**
 * @brief Start the controller with its TX pad kept off the bus until the
 *        controller is in the state it is meant to be in there.
 *
 * twai_start() clears both error counters, and a controller with both below
 * 128 is error-active: an error it detects, it answers with six dominant bits
 * that destroy the frame for every module. So the pad is a recessive GPIO
 * while the controller starts and goes back to it only after that - once it
 * is error-passive with errors let pass (TEC_PASSIVE), or at once when that is
 * switched off. In listen-only it never goes back (listenOnlyErratumFix).
 * Every start goes through here: boot, a mode switch, the restart after
 * bus-off.
 */
static bool twaiGoLive(bool silent) {
    txPadRecessive();
    if (twai_start() != ESP_OK) return false;
    if (silent) {
        listenOnlyErratumFix();
        return true;
    }
    if (Cfg.txPassive) tecSet(TEC_PASSIVE);
    txPadAttach();
    return true;
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
    uint8_t noDriver = 0;                    /**< 200 ms passes without a driver. */
    for (;;) {
        if (s_twaiShutdown) {
            // Going to sleep: the driver goes, and so does this task's use of it.
            // The TX pad leaves the controller first, recessive, so no state the
            // stopping controller passes through can reach the bus.
            xSemaphoreTake(s_twaiCtl, portMAX_DELAY);
            xSemaphoreTake(s_twaiReg, portMAX_DELAY);
            txPadRecessive();
            twai_stop();
            twai_driver_uninstall();
            s_twaiDown = true;
            xSemaphoreGive(s_twaiReg);
            xSemaphoreGive(s_twaiCtl);
            for (;;) vTaskDelay(portMAX_DELAY);
        }
        if (s_twaiReinstall) {
            // Only this task consumes the driver's queue, so only it may
            // tear the driver down and bring it back in another mode.
            xSemaphoreTake(s_twaiCtl, portMAX_DELAY);
            xSemaphoreTake(s_twaiReg, portMAX_DELAY);
            s_twaiReinstall = false;
            masterBusBlind();
            txPadRecessive();                // off the bus while it changes
            twai_stop();
            twai_driver_uninstall();
            setupTwai(s_twaiWantSilent);
            xSemaphoreGive(s_twaiReg);
            xSemaphoreGive(s_twaiCtl);
        }
        censusServiceReset();
        const esp_err_t rr = twai_receive(&msg, pdMS_TO_TICKS(200));
        if (rr == ESP_ERR_INVALID_STATE) {
            // No driver (install failed: memory, an interrupt, the timing).
            // Returning at once, this loop would hold core 1 at top priority
            // and starve everything else on it. Nothing else would ever install
            // it again - busGuardTask sees no driver and waits - so every 2 s it
            // is tried here, listen-only; the guard switches it on from there.
            vTaskDelay(pdMS_TO_TICKS(200));
            if (++noDriver >= 10 && !s_twaiReinstall) {
                noDriver = 0;
                s_twaiWantSilent = true;
                s_twaiReinstall  = true;
            }
            continue;
        }
        noDriver = 0;
        if (rr != ESP_OK) continue;
        s_canFramesRx++;
        s_rxByMode[s_twaiSilentNow ? 1 : 0]++;
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
            if (isnan(val)) continue;          // a table signal between positions
            s_verify[i].seen = true;
            s_verify[i].lastVal = val;
            if (rs.publishes()) publishMetric(rs.metricId, val, SRC_RAW);
            else                verifySample(i, val);
        }
    }
}

/* ══════════════════════ bus errors against the radio ═════════════════════
 *
 * One 3.3 V rail feeds the ESP32 and the CAN transceiver, and the radio draws
 * its biggest current spikes while it transmits - the kind that browned the
 * master out on the car. If the transceiver misreads while the rail sags, the
 * bus errors bunch up around the display broadcasts. So each error is checked
 * against the radio: transmitting, or within RADIO_TAIL_US of it (the rail
 * recovering, and the guard task waking)? Errors that have nothing to do with
 * the radio land that close to it about as often as the radio is that busy,
 * which is measured alongside.
 */
static constexpr uint32_t RADIO_TAIL_US = 3000;
static portMUX_TYPE s_radioMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_radioInFlight   = 0;  /**< Packets handed over, not yet sent.   */
static uint32_t s_radioBurstUs    = 0;  /**< When the current burst began.        */
static uint32_t s_radioHotUntilUs = 0;  /**< End of the last one + RADIO_TAIL_US. */
static uint64_t s_radioHotUs      = 0;  /**< Hot time while the bus was up.       */
static uint64_t s_radioSeenUs     = 0;  /**< Bus-up time, on the same clock.      */
static volatile uint32_t s_errNearRadio = 0, s_errRadioChecked = 0;

/** @brief A packet goes to the radio (broadcastTask, before esp_now_send). */
static void radioTxBegin() {
    portENTER_CRITICAL(&s_radioMux);
    if (s_radioInFlight++ == 0) s_radioBurstUs = micros();
    portEXIT_CRITICAL(&s_radioMux);
}
/** @brief esp_now_send refused it: it never reached the air. */
static void radioTxAbort() {
    portENTER_CRITICAL(&s_radioMux);
    if (s_radioInFlight) s_radioInFlight--;
    portEXIT_CRITICAL(&s_radioMux);
}
/** @brief The radio has sent a packet (ESP-NOW send callback, Wi-Fi task). */
static void onEspNowSent(const uint8_t *, esp_now_send_status_t) {
    const uint32_t now = micros();
    const bool alive = diagBusAlive();
    portENTER_CRITICAL(&s_radioMux);
    if (s_radioInFlight && --s_radioInFlight == 0) {
        const uint32_t until = now + RADIO_TAIL_US;
        // Only what the last burst's tail has not already counted.
        const uint32_t from = (int32_t)(s_radioHotUntilUs - s_radioBurstUs) > 0 ? s_radioHotUntilUs
                                                                                : s_radioBurstUs;
        if (alive && (int32_t)(until - from) > 0) s_radioHotUs += until - from;
        s_radioHotUntilUs = until;
    }
    portEXIT_CRITICAL(&s_radioMux);
}
/** @brief Book @p n bus errors against the radio (busGuardTask, as they arrive). */
static void radioNoteErrors(uint32_t n) {
    const uint32_t now = micros();
    portENTER_CRITICAL(&s_radioMux);
    const bool hot = s_radioInFlight || (int32_t)(s_radioHotUntilUs - now) > 0;
    portEXIT_CRITICAL(&s_radioMux);
    s_errRadioChecked = s_errRadioChecked + n;
    if (hot) s_errNearRadio = s_errNearRadio + n;
}
/** @brief Bus-up time for the hot share (busGuardTask, every pass). */
static void radioObserve(bool alive) {
    static uint32_t last = 0;
    const uint32_t now = micros();
    portENTER_CRITICAL(&s_radioMux);
    if (alive && last) s_radioSeenUs += now - last;
    // A send whose callback never came must not leave the radio "hot" for good.
    if (s_radioInFlight && now - s_radioBurstUs > 200000) s_radioInFlight = 0;
    portEXIT_CRITICAL(&s_radioMux);
    last = now;
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

            radioTxBegin();
            const bool sent = esp_now_send(BCAST, (const uint8_t *)&pkt,
                                           TELEMETRY_PACKET_SIZE(n)) == ESP_OK;
            if (!sent) radioTxAbort();
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
 * @brief Night detection (LDR or vehicle data), bus-health metrics and sleep.
 *        (Bus-off recovery lives in busGuardTask.) Never returns.
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
    uint32_t lastErrCount = 0, lastTec = 0, winStart = 0, winErrs = 0, lastSwitch = 0, lastTecWrites = 0;
    uint32_t lastDropped = 0;                 /**< Queue-full + FIFO-overrun drops.   */
    uint32_t rxWinStart = 0, rxWinErrs = 0;   /**< Receive-side errors, per window. */
    bool     wasPassive = false, wasSettling = false;
    bool     ctrlRunning = false;             /**< Last status read said RUNNING. */
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
        // Towards listen-only the switch is immediate: every moment in normal
        // mode after a guard trip is another moment of error flags on the
        // car's traffic, and the likeliest trip is right after the settle
        // wait, in the first normal-mode contact with a marginal link. Back
        // towards normal, and while a controller will not install, at most
        // every 2 s so nothing is retried in a tight loop. Never while the
        // controller is recovering from bus-off: the driver cannot be replaced
        // mid-recovery.
        if (wantSilent != s_twaiSilentNow && !s_twaiReinstall &&
            ((wantSilent && ctrlRunning) || millis() - lastSwitch > 2000)) {
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
        // The mode the status below belongs to, read under the same lock the
        // RX task holds to reinstall: s_twaiSilentNow itself can change the
        // moment the lock is released, and a listen-only REC (frozen at 128 by
        // listenOnlyErratumFix) judged as a normal-mode one looks like a storm.
        bool stSilent = false;
        uint8_t ecc = 0;                     // the latest error's kind (see errKindAdd)
        uint32_t tecWrites = 0;              // our TEC writes, as of this status
        xSemaphoreTake(s_twaiCtl, portMAX_DELAY);
        if (twai_read_alerts(&alerts, pdMS_TO_TICKS(20)) != ESP_ERR_INVALID_STATE) {
            // Under the register lock, which every TEC write of ours holds: the
            // status and the count of those writes then belong together, so a
            // write can never slip between them and pass for an error of ours.
            xSemaphoreTake(s_twaiReg, portMAX_DELAY);
            if (twai_get_status_info(&st) == ESP_OK) {
                haveStatus = true;
                stSilent = s_twaiSilentNow;
                tecWrites = s_tecWrites;
                ecc = (uint8_t)(TWAI.error_code_capture_reg.val & 0xFF);
                if (!s_twaiReinstall &&
                    (st.state == TWAI_STATE_BUS_OFF || st.state == TWAI_STATE_STOPPED)) {
                    if (st.state == TWAI_STATE_BUS_OFF) {
                        wentBusOff = true;
                        twai_initiate_recovery();
                    } else {
                        twaiGoLive(s_twaiSilentNow);   // recovery finished: counters at 0
                    }
                }
            }
            xSemaphoreGive(s_twaiReg);
        }
        xSemaphoreGive(s_twaiCtl);
        if (!haveStatus) {                   // driver being reinstalled
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        ctrlRunning = st.state == TWAI_STATE_RUNNING;
        radioObserve(diagBusAlive());
        // Frames dropped for a full queue or FIFO: the master was deaf to them,
        // and the ones it kept were late.
        const uint32_t dropped = st.rx_missed_count + st.rx_overrun_count;
        if (dropped != lastDropped) { lastDropped = dropped; masterBusStall(); }

        const uint32_t delta = st.bus_error_count - lastErrCount;
        lastErrCount = st.bus_error_count;
        // The transmit error counter only rises for errors in frames this
        // controller was sending: that is ours beyond doubt. Our own TEC
        // writes raise it too (a top-up, the start after listen-only); a pass
        // that saw one of those cannot tell, so it blames nothing.
        const bool tecRose = tecWrites == lastTecWrites && st.tx_error_counter > lastTec;
        lastTecWrites = tecWrites;
        lastTec = st.tx_error_counter;
        if (delta && delta < 10000) {        // (a reinstall resets the count)
            errKindAdd(ecc, stSilent, delta);
            radioNoteErrors(delta);
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
                /*
                 * Receive-side errors while we are a normal-mode node: every one
                 * we detected, we also flagged, and the flag destroyed the frame
                 * for the module it was meant for. REC alone misses a moderate
                 * marginal link - a good frame winds it down faster than a
                 * flagged one winds it up - so the rate is watched as well: this
                 * many in the guard window and we are corrupting the car's
                 * traffic, whatever REC says. (Not counted in listen-only, where
                 * an error we see is one we cannot signal.)
                 */
                // (Not while error-passive: our flags are recessive then and
                // cannot destroy anyone's frame - passive mode's whole point.)
                if (ctrlRunning && !stSilent && st.tx_error_counter < 128) {
                    const uint32_t now = millis();
                    if (now - rxWinStart > Cfg.guardWindowS * 1000UL) { rxWinStart = now; rxWinErrs = 0; }
                    rxWinErrs += delta;
                    if (Cfg.rxGuardEnabled && !s_rxGuardSilent && rxWinErrs >= Cfg.rxGuardErrs) {
                        s_rxGuardSilent = true;
                        evLog(EV_RX_TRIP, (uint16_t)st.rx_error_counter,
                              (uint16_t)min(rxWinErrs, (uint32_t)0xFFFF));
                        log_e("receive-error guard: %u receive errors in %u s - our controller "
                              "is corrupting other nodes' frames; switching to listen-only "
                              "(resume from the portal)", (unsigned)rxWinErrs,
                              (unsigned)Cfg.guardWindowS);
                        rxWinErrs = 0;
                    }
                }
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
        if (Cfg.rxGuardEnabled && !s_rxGuardSilent && !stSilent && st.tx_error_counter < 128 &&
            ctrlRunning && st.rx_error_counter >= Cfg.rxGuardRec) {
            s_rxGuardSilent = true;
            evLog(EV_RX_TRIP, (uint16_t)st.rx_error_counter,
                  (uint16_t)min(rxWinErrs, (uint32_t)0xFFFF));
            log_e("receive-error guard: REC %u - our controller is corrupting other "
                  "nodes' frames; switching to listen-only (resume from the portal)",
                  (unsigned)st.rx_error_counter);
            rxWinErrs = 0;
        }

        const bool passive = st.state == TWAI_STATE_RUNNING && !stSilent && !Cfg.txPassive &&
                             st.tx_error_counter >= 128;
        if (passive && !wasPassive)
            log_w("TWAI error-passive (TEC %u, REC %u)", (unsigned)st.tx_error_counter,
                  (unsigned)st.rx_error_counter);
        wasPassive = passive;

        // Bus-off means our own frames failed over and over, so it is a
        // guard trip as well - recovering and carrying on would just put the
        // same errors back on the car's bus.
        if (wentBusOff) {
            log_w("TWAI bus-off - recovering");
            if (Cfg.guardEnabled) {
                evLog(EV_BUS_OFF, (uint16_t)st.tx_error_counter, 0);
                if (diagGuardOk()) guardTrip("controller went bus-off");
            } else if (Cfg.rxGuardEnabled && !s_rxGuardSilent) {
                /*
                 * Bus-off is the protocol itself throwing us off: our frames
                 * failed 32 times over, each failure an active error flag on
                 * the bus. With the transmit guard off, recovering and carrying
                 * on as before would just do it all again, so the receive-error
                 * guard's latch holds the controller listen-only until Resume.
                 */
                s_rxGuardSilent = true;
                evLog(EV_BUS_OFF, (uint16_t)st.tx_error_counter, 1);
                log_e("bus-off with the bus guard off - switching to listen-only for this "
                      "session (resume from the portal)");
            } else {
                evLog(EV_BUS_OFF, (uint16_t)st.tx_error_counter, 0);
            }
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
        xSemaphoreTake(s_twaiReg, pdMS_TO_TICKS(500));
        txPadRecessive();
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

/* ═══════════════════ what the controller says went wrong ════════════════
 *
 * The TWAI error-code-capture register holds the type, direction and frame
 * segment of the latest bus error. Sampled whenever the error count has moved
 * (busGuardTask, under the driver lock) and tallied per mode, it tells what the
 * controller objects to, which a bare count cannot: a bit error while sending
 * in the ACK slot points at the transceiver or its supply, stuff or form errors
 * while receiving at the link or the bit timing. Portal Bus tab, serial log.
 */
struct ErrKindSlot { uint8_t code; bool silent; uint32_t samples, errors; };
static constexpr size_t ERRKIND_MAX = 12;
static ErrKindSlot s_errKinds[ERRKIND_MAX] = {};
static portMUX_TYPE s_errKindMux = portMUX_INITIALIZER_UNLOCKED;

static void errKindAdd(uint8_t code, bool silent, uint32_t errors) {
    portENTER_CRITICAL(&s_errKindMux);
    ErrKindSlot *hit = nullptr, *least = &s_errKinds[0];
    for (auto &k : s_errKinds) {
        if (k.samples && k.code == code && k.silent == silent) { hit = &k; break; }
        if (k.samples < least->samples) least = &k;
    }
    if (!hit) { hit = least; *hit = {code, silent, 0, 0}; }   // the rarest makes room
    hit->samples++;
    hit->errors += errors;
    s_errByMode[silent ? 1 : 0] += errors;
    portEXIT_CRITICAL(&s_errKindMux);
}

size_t masterErrorKinds(ErrKindView *out, size_t max) {
    ErrKindSlot copy[ERRKIND_MAX];
    portENTER_CRITICAL(&s_errKindMux);
    memcpy(copy, s_errKinds, sizeof(copy));
    portEXIT_CRITICAL(&s_errKindMux);
    size_t n = 0;
    for (const auto &k : copy) if (k.samples) n++;
    std::sort(copy, copy + ERRKIND_MAX, [](const ErrKindSlot &a, const ErrKindSlot &b) {
        return a.errors > b.errors;
    });
    n = min(n, max);
    for (size_t i = 0; i < n; i++) out[i] = {copy[i].code, copy[i].silent, copy[i].samples, copy[i].errors};
    return n;
}

void errKindText(uint8_t code, char *buf, size_t len) {
    static const char *TYPE[] = {"bit error", "form error", "stuff error", "error"};
    const char *seg;
    switch (code & 0x1F) {
        case 0x03: seg = "start of frame"; break;
        case 0x02: case 0x06: case 0x07: case 0x0F: case 0x0E: seg = "identifier"; break;
        case 0x04: case 0x05: case 0x0C: case 0x0D: case 0x09: seg = "control bits"; break;
        case 0x0B: seg = "length code"; break;
        case 0x0A: seg = "data field"; break;
        case 0x08: seg = "CRC sequence"; break;
        case 0x18: seg = "CRC delimiter"; break;
        case 0x19: seg = "ACK slot"; break;
        case 0x1B: seg = "ACK delimiter"; break;
        case 0x1A: seg = "end of frame"; break;
        case 0x12: seg = "intermission"; break;
        case 0x11: seg = "active error flag"; break;
        case 0x16: seg = "passive error flag"; break;
        case 0x13: seg = "dominant bits after an error flag"; break;
        case 0x17: seg = "error delimiter"; break;
        case 0x1C: seg = "overload flag"; break;
        default:   seg = "segment ?"; break;
    }
    snprintf(buf, len, "%s %s in %s", TYPE[code >> 6], (code & 0x20) ? "receiving" : "sending", seg);
}

/** Radio transmit power, from the setting (esp_wifi: quarter-dBm units). */
static uint8_t s_radioDbmApplied = 0;
static void applyRadioPower() {
    s_radioDbmApplied = Cfg.radioDbm;
    esp_wifi_set_max_tx_power((int8_t)(Cfg.radioDbm * 4));
}

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
    for (int m = 0; m < 2; m++) {
        out.rxByMode[m]  = s_rxByMode[m];
        out.errByMode[m] = s_errByMode[m];
        out.missed[m] = out.expected[m] = 0;
    }
    const uint8_t n = s_censusUsed;
    for (uint8_t i = 0; i < n; i++) {
        const CanIdCount &c = s_census[i];
        if (!idPeriodic(c) || idOnRequest(c)) continue;
        for (int m = 0; m < 2; m++) { out.missed[m] += c.missed[m]; out.expected[m] += c.expected[m]; }
    }
    out.errNearRadio    = s_errNearRadio;
    out.errRadioChecked = s_errRadioChecked;
    portENTER_CRITICAL(&s_radioMux);
    const uint64_t hot = s_radioHotUs, seen = s_radioSeenUs;
    portEXIT_CRITICAL(&s_radioMux);
    out.radioHotPermille = seen ? (uint16_t)std::min<uint64_t>(1000, hot * 1000 / seen) : 0;
}

size_t masterMissedIds(MissView *out, size_t max) {
    size_t w = 0;
    const uint8_t n = s_censusUsed;
    for (uint8_t i = 0; i < n; i++) {
        const CanIdCount &c = s_census[i];
        if (!idPeriodic(c) || idOnRequest(c) || !(c.expected[0] + c.expected[1])) continue;
        MissView v = {c.id, c.extd, (c.periodUs + 500) / 1000, {c.missed[0], c.missed[1]},
                      {c.expected[0], c.expected[1]}};
        // Insertion into the top max, most missed first (then the busiest).
        const auto more = [](const MissView &a, const MissView &b) {
            const uint32_t ma = a.missed[0] + a.missed[1], mb = b.missed[0] + b.missed[1];
            return ma != mb ? ma > mb : a.expected[0] + a.expected[1] > b.expected[0] + b.expected[1];
        };
        size_t at = w;
        while (at > 0 && more(v, out[at - 1])) at--;
        if (at >= max) continue;
        for (size_t k = (w < max ? w : max - 1); k > at; k--) out[k] = out[k - 1];
        out[at] = v;
        if (w < max) w++;
    }
    return w;
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

bool masterCensusEdges(uint32_t id, bool extd, uint8_t out[64]) {
    const uint8_t n = s_censusUsed;
    for (uint8_t i = 0; i < n; i++)
        if (s_census[i].id == id && s_census[i].extd == extd) {
            memcpy(out, s_census[i].edges, 64);   // bytes: a torn copy is off by one flip at most
            return true;
        }
    return false;
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

void masterBeforeRestart() {
    Cfg.serviceSave();               // a save another task asked for
    evSaveNow();                     // and the evidence log's last few seconds
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
 * Runs in listen-only mode when the diagnostic mode is SILENT, while the bus
 * settles, and after a guard trip. Listen-only is incapable of disturbing the
 * vehicle bus only because of listenOnlyErratumFix() and the TX pad being taken
 * from the controller: on this silicon the mode alone still sends error flags.
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
     * Sample point and sampling. The ESP-IDF presets sample once, at 80 % of
     * the bit (SJW 3). Vehicle buses are specified at 87.5 % (SAE J1939 /
     * J2284), and a node that samples early or once on a stub with ringing
     * reads errors nobody else sees - and in normal mode it flags every one,
     * destroying the frame for the whole car. The default (canTiming 2) is the
     * timing the Arduino-CAN library programs on the ESP32, which works on
     * cars where the ESP-IDF preset produced bus errors (arduino-esp32 #9191):
     * 16 time quanta, 14 before the sample (87.5 %), SJW 2, and triple
     * sampling - three samples per bit, decided by majority, so a spike or a
     * ringing edge is outvoted instead of read as a bit. canTiming 0 keeps
     * the ESP-IDF preset (with the older 87.5 % switch, canSample875).
     */
    const bool late = Cfg.bitrateKbps != 1000 && (Cfg.canTiming >= 1 || Cfg.canSample875);
    if (late) {
        t.brp   = 80000 / (16 * Cfg.bitrateKbps);   // APB 80 MHz: 500k -> 10
        t.tseg_1 = 13;
        t.tseg_2 = 2;
        t.sjw    = 2;
    }
    t.triple_sampling = Cfg.canTiming == 2 && Cfg.bitrateKbps != 1000;
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        log_e("TWAI init failed — check CAN_TX/RX_GPIO wiring");
        return;
    }
    // The TX pad is off the controller while it starts (twaiGoLive).
    if (!twaiGoLive(silent)) {
        twai_driver_uninstall();
        log_e("TWAI init failed — check CAN_TX/RX_GPIO wiring");
        return;
    }
    s_twaiSilentNow = silent;
    log_i("TWAI up (%s mode) at %u kbit/s, sample point %s%s%s",
          silent ? "SILENT/listen-only" : "NORMAL", Cfg.bitrateKbps,
          late ? "87.5%" : "80%", t.triple_sampling ? ", triple sampling" : "",
          !silent && Cfg.txPassive ? ", errors let pass (no error frames)" : "");
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
    esp_now_register_send_cb(onEspNowSent);   // when each packet has gone (radio timing)
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
    s_twaiReg  = xSemaphoreCreateMutex();
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
    applyRadioPower();

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
 * @brief The way back into a portal that was switched off (portal_on false):
 *        the BOOT button held for 3 s switches it on again and restarts.
 *
 * Nothing else reaches the settings then - there is no serial console, and
 * the configuration survives a reflash by design. Acts only while the portal
 * is off, so a master that has it on is not touched.
 */
static void portalRescueCheck() {
    static uint32_t downSince = 0;
    static bool pinReady = false;
    if (Cfg.portalOn) { downSince = 0; return; }
    if (!pinReady) {
        gpio_set_direction((gpio_num_t)BOOT_BUTTON_GPIO, GPIO_MODE_INPUT);
        gpio_pullup_en((gpio_num_t)BOOT_BUTTON_GPIO);
        pinReady = true;
    }
    if (gpio_get_level((gpio_num_t)BOOT_BUTTON_GPIO) != 0) { downSince = 0; return; }
    const uint32_t now = millis();
    if (!downSince) { downSince = now | 1; return; }
    if (now - downSince < 3000) return;
    log_w("BOOT held 3 s: the portal is switched back on - restarting");
    Cfg.lock();
    Cfg.portalOn = true;
    Cfg.unlock();
    Cfg.save();
    masterBeforeRestart();
    ESP.restart();
    downSince = 0;                   // (only the simulator comes back here)
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
        {
            // What the controller says the errors are (see errKindAdd).
            ErrKindView ek[2];
            const size_t nk = masterErrorKinds(ek, 2);
            for (size_t i = 0; i < nk; i++) {
                char txt[64];
                errKindText(ek[i].code, txt, sizeof(txt));
                Serial.printf("[errs ] %s, %s: %u errors\n", txt,
                              ek[i].silent ? "listen-only" : "normal mode", (unsigned)ek[i].errors);
            }
            // Where they come from: frames missed per mode, and the radio.
            MasterStats ls;
            masterGetStats(ls);
            if (ls.expected[0] || ls.expected[1] || ls.errRadioChecked)
                Serial.printf("[link ] missed %u of %u normal, %u of %u listen-only | "
                              "errors near the radio %u of %u (radio busy %u.%u%%)\n",
                              (unsigned)ls.missed[0], (unsigned)ls.expected[0],
                              (unsigned)ls.missed[1], (unsigned)ls.expected[1],
                              (unsigned)ls.errNearRadio, (unsigned)ls.errRadioChecked,
                              ls.radioHotPermille / 10, ls.radioHotPermille % 10);
        }
    }
    Portal.loop();
    if (Cfg.radioDbm != s_radioDbmApplied) applyRadioPower();
    Cfg.serviceSave();
    evService();
    portalRescueCheck();
    delay(10);
}

/** @} */  // end of master group
