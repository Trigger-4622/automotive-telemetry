/**
 * @file TelemetryStore.h
 * @ingroup telemetry
 * @brief Thread-safe cache of the latest value for every metric heard on the
 *        ESP-NOW firehose, plus kinetic smoothing state.
 *
 * Two execution contexts touch this store:
 *   - The Wi-Fi task (ESP-NOW receive callback) calls ingestRaw().
 *   - The LVGL/UI context (Arduino loop) calls sample()/stats()/resetPeaks().
 * A spinlock critical section guards the slot array; every hold is a few µs.
 *
 * Smoothing pipeline (per metric):
 *   raw  ──EMA(alpha, at packet rate)──► ema ──Lerp(factor, at UI rate)──► display
 * The EMA kills sensor/bus noise; the lerp interpolates between packets so
 * needles glide at 33 fps even when telemetry arrives at 10–20 Hz.
 */

/**
 * @defgroup telemetry Telemetry Link
 * @brief The ESP-NOW receiver and the value cache it feeds.
 *
 * This is the only place in the slave where two execution contexts meet: the
 * Wi-Fi task delivers packets while the Arduino loop renders from them.
 * @{
 */
#pragma once

#include <Arduino.h>
#include "MasterPacket.h"

/** @brief Snapshot handed to the UI for one metric. */
struct MetricSample {
    float   value;   /**< Smoothed display value (EMA + lerp).               */
    float   raw;     /**< Last raw value from the wire.                      */
    float   peak;    /**< Highest raw value seen since boot / reset.         */
    uint8_t flags;   /**< METRIC_FLAG_* bits from the last packet.           */
    bool    stale;   /**< No update within the stale timeout.                */
};

/** @brief Link-quality counters for the diagnostics screen / web portal. */
struct LinkStats {
    uint32_t packetsReceived;  /**< Valid frames accepted since boot.        */
    uint32_t packetsDropped;   /**< Inferred from sequence_id gaps.          */
    uint32_t lastSequenceId;   /**< sequence_id of the most recent frame.    */
    uint32_t lastRxMs;         /**< millis() of the last valid frame.        */
    float    packetsPerSecond; /**< Rolling 1 s window.                      */
};

/** @brief Read-only slot view used by the web portal's `/api/live`. */
struct MetricView {
    uint16_t id;      /**< Metric ID — see @ref MetricIDs.                   */
    float    raw;     /**< Last raw value from the wire.                     */
    float    peak;    /**< Highest raw value since boot / reset.             */
    uint8_t  flags;   /**< METRIC_FLAG_* bits.                               */
    uint32_t ageMs;   /**< Milliseconds since this metric last updated.      */
};

class TelemetryStore {
public:
    /* Distinct metric IDs trackable at once. Sized for multi-frame bursts:
     * a Master streaming more than one full 28-metric frame per cycle (e.g.
     * powertrain + full electrical system) still fits comfortably. */
    static constexpr size_t MAX_SLOTS = 256;

    /**
     * @brief Configure smoothing and staleness before the first packet.
     * @param emaAlpha        EMA coefficient 0..1 (higher = snappier);
     *                        clamped to a sane range internally.
     * @param staleTimeoutMs  Metric age after which it renders as "--".
     */
    void begin(float emaAlpha, uint32_t staleTimeoutMs);

    /**
     * @brief Feed a raw ESP-NOW payload into the store.
     *
     * Validates the frame, updates link statistics, then merges each metric
     * by ID — which is what makes the master's multi-frame bursts work.
     *
     * @param mac  Sender's MAC, or nullptr when unavailable. Recorded so the
     *             portal can show which master is actually transmitting —
     *             the value you need in order to set a peer filter.
     * @param data Received bytes.
     * @param len  Byte count from the ESP-NOW callback.
     * @note Called from the Wi-Fi task. Takes the spinlock.
     */
    void ingestRaw(const uint8_t *mac, const uint8_t *data, int len);

    /**
     * @brief MAC of the master whose frames are being received.
     * @return "AA:BB:CC:DD:EE:FF", or an empty string before the first frame.
     */
    String peerMac();

    /**
     * Fetch a metric for rendering and advance its lerp state one UI tick.
     * @param lerpFactor 0..1 fraction moved toward the EMA target per call.
     * @return false if the metric has never been received.
     */
    /**
     * @param id         Metric to fetch — see @ref MetricIDs.
     * @param lerpFactor 0..1 fraction moved toward the EMA target this call.
     * @param[out] out   Filled only when the metric exists.
     */
    bool sample(uint16_t id, float lerpFactor, MetricSample &out);

    /**
     * Read a metric WITHOUT advancing its interpolation state.
     *
     * Use this for anything that only inspects values — the watchdog, the web
     * API. Calling sample() from two places in one tick would advance the lerp
     * twice and make needles move at double speed.
     *
     * @param id       Metric to read — see @ref MetricIDs.
     * @param[out] out Filled only when the metric exists.
     * @return false if the metric has never been received.
     */
    bool peek(uint16_t id, MetricSample &out);

    /**
     * @brief Copy every active slot, for the web portal's `/api/live`.
     * @param[out] out Destination array.
     * @param max      Capacity of @p out.
     * @return Number of entries written, never more than @p max.
     */
    size_t snapshot(MetricView *out, size_t max);

    /** @brief Reset every peak to the metric's current raw value. */
    void resetPeaks();

    /**
     * @brief Night-mode flag as reported by the master.
     * @return true when the last frame carried @ref METRIC_FLAG_NIGHT.
     */
    bool nightMode() const { return _night; }

    /**
     * @brief Whether the telemetry link is alive.
     * @return true while packets have arrived within the stale window.
     */
    bool linkUp() const;

    /**
     * @brief Copy the link-quality counters.
     * @return A consistent snapshot taken under the lock.
     */
    LinkStats stats();

private:
    /** @brief One cached metric plus its smoothing state. */
    struct Slot {
        uint16_t id;            /**< Metric ID this slot holds.             */
        bool     used;          /**< Slot has been claimed.                 */
        float    raw;           /**< Last value from the wire.              */
        float    ema;           /**< Exponential moving average.            */
        float    display;       /**< Interpolated value actually rendered.  */
        float    peak;          /**< Highest raw value since reset.         */
        uint8_t  flags;         /**< METRIC_FLAG_* from the last packet.    */
        uint32_t lastUpdateMs;  /**< millis() of the last update.           */
    };

    /**
     * @brief Find an existing slot by metric ID.
     * @param id Metric to look up.
     * @return Slot pointer, or nullptr when not present.
     * @warning Call with the spinlock held.
     */
    Slot *find(uint16_t id);

    /**
     * @brief Find a slot, claiming a free one if the metric is new.
     * @param id Metric to look up or claim.
     * @return Slot pointer, or nullptr when all slots are in use.
     * @warning Call with the spinlock held.
     */
    Slot *findOrAlloc(uint16_t id);

    Slot          _slots[MAX_SLOTS] = {};    /**< The cache itself.         */
    float         _alpha            = 0.35f; /**< EMA coefficient.          */
    uint32_t      _staleMs          = 1500;  /**< Staleness threshold [ms]. */
    volatile bool _night            = false; /**< Latest night-mode flag.   */

    uint8_t       _peerMac[6]       = {};    /**< Last sender's MAC.        */
    bool          _peerKnown        = false; /**< A sender MAC was recorded.*/

    LinkStats     _stats            = {};    /**< Link-quality counters.    */
    uint32_t      _windowStartMs    = 0;     /**< Rate window start.        */
    uint32_t      _windowCount      = 0;     /**< Frames in current window. */

    /** @brief Guards @ref _slots and @ref _stats across the task boundary. */
    portMUX_TYPE  _mux = portMUX_INITIALIZER_UNLOCKED;
};

/** @brief Global singleton — the firehose has exactly one sink. */
extern TelemetryStore Telemetry;

/** @} */  // end of telemetry group
