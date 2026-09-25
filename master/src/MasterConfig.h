/**
 * @file MasterConfig.h
 * @brief Runtime configuration for the master, persisted on LittleFS.
 *
 * ## Where it is stored
 *
 * A JSON file on LittleFS, written by this firmware and never uploaded.
 *
 * It lived in NVS first, to avoid the second upload step that the slaves need
 * for their layouts. That worked until the parameter tables grew to 58 SSM
 * entries and 36 PIDs and the document passed NVS's ~4000-byte limit for a
 * string entry — whereupon Preferences crashed inside its own error logging
 * and boot-looped the master, which is a considerably worse outcome than a
 * refused write.
 *
 * LittleFS has no such ceiling, and costs nothing here: the page is still
 * compiled into flash and the config file is created by the firmware itself,
 * so `pio run -t upload` is still the whole story. There is no `uploadfs`
 * step to forget.
 *
 * ## What is and is not live
 *
 * Anything the running tasks read every pass — the diagnostic mode (listen-
 * only included: the controller is switched live), PID periods, broadcast
 * rate, TTL, the signal and SSM tables — takes effect the moment it is saved.
 * Anything that configures a peripheral at start-up — CAN bitrate, sample
 * point, ESP-NOW channel, AP credentials — needs a reboot, and the UI says so
 * rather than silently doing nothing.
 *
 * ## Concurrency
 *
 * Three tasks read the tables continuously and the portal rewrites them from
 * a fourth. A reader holding an iterator across a rewrite would walk freed
 * memory, so writers take @ref lock() around the swap and readers either take
 * it for a short copy or watch @ref generation and re-copy when it moves.
 * Saving is done by loop() only — other tasks call requestSave().
 */
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <vector>

#include "CanDecoderConfig.h"

/** @name Diagnostic mode — see DIAG_MODE_DEFAULT in CanDecoderConfig.h
 *  @{ */
#define DIAG_MODE_AUTO    0
#define DIAG_MODE_SSM     1
#define DIAG_MODE_OBD     2
#define DIAG_MODE_BOTH    3
#define DIAG_MODE_SILENT  4
/** @} */

/** @brief Most raw signals the master decodes. Hand-mapped and learned ones
 *  share it; the learner stops proposing when it is reached. */
static constexpr size_t MAX_RT_SIGNALS = 96;

/**
 * @brief One runtime-editable CAN signal definition.
 *
 * The mutable twin of @ref CanSignalDef. The compiled table is the factory
 * default; this is what the web UI edits and what the RX task actually uses,
 * so a signal can be found and mapped in the car without a rebuild.
 */
struct RtSignal {
    uint32_t canId     = 0;      /**< 11- or 29-bit identifier.             */
    bool     extended  = false;  /**< true for 29-bit.                      */
    uint8_t  startBit  = 0;      /**< See CanSignalDef for bit numbering.   */
    uint8_t  bitLength = 8;      /**< 1-32.                                 */
    bool     bigEndian = false;  /**< Motorola byte order.                  */
    bool     isSigned  = false;  /**< Two's-complement.                     */
    float    scale     = 1.0f;   /**< value = raw * scale + offset.         */
    float    offset    = 0.0f;
    uint16_t metricId  = 0;      /**< Channel to publish on.                */
    uint8_t  mode      = SIG_OFF;/**< @ref SignalMode.                      */
    uint16_t refMetric = 0;      /**< Verified against this (0 = none).     */
    bool     learned   = false;  /**< Created by the learner, not by hand.  */
    float    vtol      = 0;      /**< Verifier tolerance (0 = built-in).    */
    float    vspread   = 0;      /**< Verifier range needed (0 = built-in). */
    char     name[16]  = {0};    /**< Label, for the UI only.               */

    /** @return true when the signal publishes. */
    bool publishes() const { return mode == SIG_ON || mode == SIG_VERIFIED; }
};

/**
 * @brief One SSM2 parameter: an ECU memory address and how to scale it.
 *
 * Subaru's own protocol reads arbitrary ECU memory by 3-byte address, which is
 * where everything OBD-II never exposes lives — knock correction, A/F
 * learning, injector pulse width. The seeded table is the standard SSM2
 * parameter set documented by RomRaider and FreeSSM; which of them THIS ECU
 * actually implements is read from the ECU itself at start-up (the init
 * response carries one support bit per parameter, @ref capByte/@ref capBit),
 * so nothing unsupported is ever asked for.
 */
struct RtSsm {
    uint32_t address  = 0;       /**< 3-byte ECU address (high byte first
                                      for two-byte values).                 */
    uint8_t  bytes    = 1;       /**< 1 or 2 consecutive bytes.             */
    bool     isSigned = false;   /**< Two's-complement before scaling.      */
    float    scale    = 1.0f;    /**< value = raw * scale + offset.         */
    float    offset   = 0.0f;
    uint16_t metricId = 0;       /**< Channel to publish on.                */
    bool     enabled  = true;    /**< User intent; support is separate.     */
    uint16_t periodMs = 0;       /**< Refresh interval; 0 = every exchange. */
    uint8_t  capByte  = 0;       /**< 1-based init-response byte (0 = no
                                      support bit known: always asked).     */
    uint8_t  capBit   = 0;       /**< 1-based bit within that byte.         */
    char     name[20] = {0};     /**< Label, for the UI only.               */
};

/** @brief Per-PID enable/period override for the compiled poll table. */
struct RtPid {
    uint8_t  pid     = 0;        /**< Service-01 PID this overrides.        */
    bool     enabled = true;     /**< Skipped entirely when false.          */
    uint16_t periodMs = 100;     /**< Poll interval.                        */
};

/**
 * @brief The master's persisted settings. Singleton — see @ref Cfg.
 */
class MasterConfig {
public:
    /** Bumped whenever a compiled default CHANGES meaning (not just grows).
     *  Each bump needs its own `_storedVersion < N` migration in begin() that
     *  touches only what changed - never a blanket re-seed, which would
     *  delete learned signals. */
    static constexpr uint8_t CFG_VERSION = 5;

    /** @brief Load from LittleFS, falling back to the compiled defaults. */
    void begin();

    /**
     * @brief Persist the current values. From loop() (and the sleep path);
     *        other tasks call requestSave(). Serialised internally.
     * @return false if the filesystem refused the write.
     */
    bool save();

    /** @brief Ask loop() to save soon. Safe from any task. */
    void requestSave() { _saveWanted = true; }

    /** @brief Perform a pending requestSave(). Called from loop(). */
    void serviceSave();

    /**
     * @brief Add parameters this firmware knows about that the stored config
     *        does not, preserving every user choice on the ones it already had.
     * @return true if anything was added.
     */
    bool mergeNewDefaults();

    /** @brief Reset every field to the compiled defaults (does not save). */
    void loadDefaults();

    /** @brief Re-seed only the SSM and raw-signal tables from the compiled
     *         tables, keeping everything else. */
    void reseedTables();

    /** @brief Re-seed only the OBD-II PID list (enable flags and periods). */
    void reseedPids();

    /** @brief Serialise into @p doc for the REST API. */
    void toJson(JsonDocument &doc) const;

    /**
     * @brief Merge a settings document (the stored file, or the REST API).
     * @param v        Object with any subset of the known keys.
     * @param fromUser An edit from the portal: seeded SSM2 parameters missing
     *                 from its table were deleted on purpose and are
     *                 remembered as such.
     * @return false if @p v is not an object.
     */
    bool fromJson(JsonVariantConst v, bool fromUser = false);

    /**
     * @brief Look up the override for a PID.
     * @param pid Service-01 PID.
     * @return Pointer into @ref pids, or nullptr when the PID has no entry.
     */
    const RtPid *pidFor(uint8_t pid) const;

    /** @name Table guard
     *  Recursive, so save() may be called with the lock already held.
     *  @{ */
    void lock()   { xSemaphoreTakeRecursive(_mutex, portMAX_DELAY); }
    void unlock() { xSemaphoreGiveRecursive(_mutex); }
    /** Incremented on every table rewrite; readers re-copy when it moves. */
    volatile uint32_t generation = 0;
    /** @} */

    /** @name Live — read every pass, so saving is enough
     *  @{ */
    uint8_t  diagMode    = DIAG_MODE_DEFAULT;   /**< DIAG_MODE_*.          */
    uint16_t broadcastMs = BROADCAST_PERIOD_MS; /**< ESP-NOW burst period. */
    uint16_t metricTtlMs = METRIC_TTL_MS;       /**< Staleness cutoff.     */
    uint8_t  nightSource = NIGHT_SOURCE;        /**< 0 none, 1 LDR, 2 car. */
    std::vector<RtSignal> signals;              /**< Raw decode table.     */
    std::vector<RtPid>    pids;                 /**< PID overrides.        */
    std::vector<RtSsm>    ssm;                  /**< Subaru SSM2 table.    */
    std::vector<uint32_t> ssmRemoved;           /**< Seeded SSM2 addresses
                                                     the user deleted.     */
    uint16_t ssmGapMs    = 100;    /**< Minimum gap between SSM2 requests. */
    uint8_t  ssmBatchMax = 33;     /**< Addresses per request (FreeSSM: 33).*/
    bool     ssmSwitches = true;   /**< Also read the switch bytes.        */
    bool     learnEnabled = true;  /**< Learn broadcast signals by itself. */
    bool     canSample875 = false; /**< 87.5 % sample point (reboot).      */

    /** @name Tunables - every one editable from the portal's Advanced tab
     *  @{ */
    bool     guardEnabled = true;  /**< Bus guard on.                          */
    uint8_t  guardErrs    = 3;     /**< Errors after our frames that trip it.  */
    uint8_t  guardWindowS = 10;    /**< ...within this many seconds.           */
    uint16_t guardPauseS  = 30;    /**< Requests paused this long per trip.    */
    uint8_t  guardTrips   = 3;     /**< Trips before forced listen-only.       */
    uint16_t startDelayS  = BUS_SETTLE_S; /**< Listen only this long after the
                                               bus comes up (0 = off).     */
    float    learnR2      = 0.985f;/**< Fit that counts as a match.            */
    uint16_t learnMinN    = 40;    /**< Samples behind a match.                */
    float    learnSteady  = 0.10f; /**< Steady = moved < this x the need.      */
    float    learnPhi     = 0.95f; /**< Bit correlation that counts as a match.*/
    uint8_t  verifyN      = 30;    /**< Agreeing samples before trusting one.  */
    uint8_t  obdGapMs     = 8;     /**< Pause between OBD-II requests.         */
    uint16_t obdTimeoutMs = 80;    /**< OBD-II reply wait.                     */
    uint8_t  obdAddressing = 0;    /**< 0 auto, 1 engine ECU, 2 all ECUs.      */
    uint16_t ssmTimeoutMs = 1000;  /**< SSM2 reply wait.                       */
    uint16_t coverMs      = 2500;  /**< A value this fresh from a better
                                        source is not requested again.         */
    uint16_t keepaliveMs  = 300;   /**< Unchanged values re-sent this often.   */
    uint16_t sourceHoldMs = 500;   /**< A better source holds a value this long.*/
    uint16_t ldrDark      = NIGHT_LDR_DARK_ADC;   /**< LDR: below = night.    */
    uint16_t ldrLight     = NIGHT_LDR_LIGHT_ADC;  /**< LDR: above = day.      */
    /** @} */

    /** @name Sleep — the battery protection
     *  A permanently-live OBD port means an idle master will flatten the car
     *  over a few weeks. Sleeping on bus silence and waking on bus activity
     *  costs nothing and removes the whole problem.
     *  @{ */
    bool     sleepEnabled = true;   /**< Deep sleep when the bus goes quiet.*/
    uint16_t sleepIdleS   = 90;     /**< Seconds of CAN silence before it.  */
    /** @} */
    /** @} */

    /** @name Boot-time — a reboot is required, and the UI says so
     *  @{ */
    uint16_t bitrateKbps = CAN_BITRATE_KBPS;     /**< 125, 250, 500, 1000.  */
    uint8_t  wifiChannel = TELEMETRY_WIFI_CHANNEL; /**< Must match slaves.  */
    char     apSsid[32]  = "Telemetry-Master-Config";
    char     apPass[32]  = "";                   /**< <8 chars = open AP.   */
    bool     portalOn    = true;                 /**< Raise the AP at boot. */
    /** @} */

    /** @name Remembered from the last ECU contact, for the portal
     *  @{ */
    char     ecuId[12]     = "";   /**< ROM ID as hex, e.g. "4B12785206". */
    char     ecuSysId[8]   = "";   /**< SYS ID as hex.                    */
    char     ecuFlags[200] = "";   /**< Capability bytes as hex.          */
    /** @} */

    MasterConfig() {
        _mutex     = xSemaphoreCreateRecursiveMutex();
        _saveMutex = xSemaphoreCreateMutex();
    }

private:
    SemaphoreHandle_t _mutex;
    /** One writer of the file at a time (loop(), and the sleep path). Not
     *  the table lock: that would stall the CAN receive task for as long as
     *  the flash write takes. */
    SemaphoreHandle_t _saveMutex;
    bool saveLocked();
    volatile bool     _saveWanted = false;
    uint8_t           _storedVersion = 0;
};

/** @brief Global configuration singleton. */
extern MasterConfig Cfg;
