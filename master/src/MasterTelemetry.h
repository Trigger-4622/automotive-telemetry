/**
 * @file MasterTelemetry.h
 * @brief The master's shared state: metric store, counters, diagnostics
 *        arbitration, and the read-only window the portal looks through.
 *
 * main.cpp owns the counters, the ID census and the metric store, and keeps
 * them file-static so nothing can write to them from outside. The portal
 * needs to *show* them, which is a different thing from being allowed to
 * touch them — hence snapshot functions rather than exported globals.
 */
#pragma once

#include <Arduino.h>

/**
 * @brief Where a value came from, in ascending order of preference.
 *
 * Listening beats asking: a value the car broadcasts costs no bus time and
 * cannot disturb anything. When the master must ask, the standard OBD-II
 * protocol comes before Subaru's own SSM2, which is only used for what
 * OBD-II has no PID for. A fresher, higher-ranked source wins, and each
 * engine skips anything a higher-ranked one already delivers - so a value
 * is only ever requested when nothing better has it.
 */
enum MetricSource : uint8_t {
    SRC_NONE    = 0,
    SRC_SSM     = 1,  /**< Subaru SSM2 read - last resort.                  */
    SRC_OBD     = 2,  /**< OBD-II Service 01 poll.                          */
    SRC_DERIVED = 3,  /**< Computed from other metrics (boost, AFR, duty).  */
    SRC_RAW     = 4,  /**< Decoded from a broadcast CAN frame - preferred.  */
};

/**
 * @brief Store or refresh one parsed engineering value.
 * @param id     Metric ID — see MasterPacket.h.
 * @param value  Value already scaled to the unit that ID mandates.
 * @param source One of @ref MetricSource. A lower-ranked source does not
 *               overwrite a value a higher-ranked one refreshed recently.
 */
void publishMetric(uint16_t id, float value, uint8_t source);

/**
 * @brief Read one metric back.
 * @param[out] value  Latest value.
 * @param[out] ageMs  Milliseconds since it was last written.
 * @param[out] source Who wrote it.
 * @return false when the metric has never been published.
 */
bool metricLookup(uint16_t id, float &value, uint32_t &ageMs, uint8_t &source);

/**
 * @brief Is a metric being delivered right now by a source ranked above
 *        @p below?  Used by the engines to skip redundant polls.
 */
bool metricCoveredAbove(uint16_t id, uint8_t below, uint32_t maxAgeMs);

/** @name Diagnostics arbitration
 *  Which engine may talk to the ECU is decided here, from the configured
 *  mode plus what the ECU has actually answered.
 *  @{ */
bool diagSsmWanted();                 /**< The SSM2 task should run.       */
bool diagObdWanted();                 /**< The OBD-II poller should run.   */
void diagReportSsm(bool answered);    /**< SSM2 init result, from its task.*/
void diagReportObd(bool answered);    /**< OBD-II probe result.            */
bool diagBusAlive();                  /**< Frames seen within the last 2 s.*/
/** One diagnostic dialogue on the bus at a time. Both engines transmit to
 *  0x7E0 and listen on 0x7E8; interleaving them corrupts both. */
bool diagBusLock(uint32_t timeoutMs);
void diagBusUnlock();
/**
 * @brief May the master transmit right now? False while the bus guard has
 *        requests paused, or has put the controller into listen-only mode,
 *        after seeing bus errors during our own transmissions.
 */
bool diagGuardOk();
/** @brief Clear the guard: resume requests (and leave forced listen-only). */
void diagGuardReset();
/**
 * @brief Milliseconds the master still only listens while the bus settles
 *        (MasterConfig::startDelayS after it came up; 0 = requests allowed).
 *        The full wait while the bus is quiet: it starts when frames do.
 */
uint32_t diagSettleLeftMs();
/** @} */

/** @brief Counters behind the serial heartbeat and the portal's Live tab. */
struct MasterStats {
    uint32_t canRx;      /**< CAN frames received since boot.          */
    uint32_t obdRx;      /**< OBD replies decoded since boot.          */
    uint32_t espTx;      /**< ESP-NOW frames accepted by the radio.    */
    uint32_t espFail;    /**< ESP-NOW frames the radio refused.        */
    uint32_t espMetrics; /**< Metric entries carried by those frames.  */
    uint32_t lastCanAgeMs; /**< Since the last CAN frame (0xFFFFFFFF = never). */
    bool     night;      /**< Current night-mode decision.             */
    bool     busOff;     /**< TWAI is in bus-off recovery.             */
    bool     busAlive;   /**< Frames within the last 2 s.              */
    bool     ssmActive;  /**< SSM2 engine currently polling.           */
    bool     obdActive;  /**< OBD-II engine currently polling.         */
    uint8_t  ssmAnswers; /**< 0 unknown, 1 yes, 2 no.                  */
    uint8_t  obdAnswers; /**< 0 unknown, 1 yes, 2 no.                  */
    uint8_t  obdSupported; /**< PIDs the ECU reports supporting.       */
    bool     obdPhysical;  /**< OBD requests go to 0x7E0, not 0x7DF.  */
    bool     silent;     /**< Listen-only by choice (SILENT) or by the
                              guard - not the settle wait below.       */
    uint32_t settleMs;   /**< Settle wait left, ms (0 = over or off).  */
    /** @name Bus health — the answer to "am I overloading the bus?"
     *  @{ */
    uint32_t busBitsRx;  /**< Estimated bits received since boot.      */
    uint32_t canTx;      /**< Frames this node transmitted.            */
    uint16_t bitrate;    /**< kbit/s, for turning bits into a load %.  */
    uint32_t tec;        /**< Transmit error counter (ours).           */
    uint32_t rec;        /**< Receive error counter.                   */
    uint32_t busErrors;  /**< Bus errors seen by the controller.       */
    uint32_t arbLost;    /**< Arbitrations we lost (we yielded).       */
    uint32_t txFailed;   /**< Our frames that never made it out.       */
    uint32_t rxMissed;   /**< Frames dropped: receive queue full.      */
    uint32_t rxOverrun;  /**< Frames dropped: controller FIFO overrun. */
    /** @} */
    /** @name Bus guard
     *  @{ */
    uint32_t errWhileTx;   /**< Bus errors in frames we were sending.      */
    uint32_t errIdle;      /**< Bus errors while we were not transmitting.  */
    uint8_t  guardTrips;   /**< Times requests were paused this session.    */
    uint32_t guardPauseMs; /**< Pause left, ms (0 = requests allowed).      */
    bool     guardSilent;  /**< Guard forced the controller to listen-only. */
    /** @} */
};

/** @brief One row of the CAN identifier census. */
struct CensusView {
    uint32_t id;     /**< Identifier as received.        */
    uint32_t count;  /**< Frames seen since boot.        */
    uint8_t  dlc;    /**< Payload length of the last.    */
    bool     extd;   /**< 29-bit identifier.             */
    uint8_t  data[8];    /**< Last payload.              */
    uint8_t  changed[8]; /**< Bits that ever toggled.    */
    uint32_t ageMs;      /**< Since the last frame.      */
};

/** @brief One decoded channel currently in the metric store. */
struct MetricViewM {
    uint16_t id;      /**< Metric ID — see MasterPacket.h.  */
    float    value;   /**< Latest engineering value.        */
    uint32_t ageMs;   /**< Milliseconds since last update.  */
    uint8_t  source;  /**< @ref MetricSource.               */
};

/**
 * @brief Copy the current counters.
 * @param[out] out Destination.
 */
void masterGetStats(MasterStats &out);

/**
 * @brief Copy the identifier census.
 * @param[out] out Destination array.
 * @param max      Capacity of @p out.
 * @return Rows written.
 */
size_t masterGetCensus(CensusView *out, size_t max);

/**
 * @brief Copy every metric in the store.
 * @param[out] out Destination array.
 * @param max      Capacity of @p out.
 * @return Rows written.
 */
size_t masterGetMetrics(MetricViewM *out, size_t max);

/** @brief Forget every identifier seen so far, for a clean capture. */
void masterResetCensus();

/** @brief Restart verification: counters cleared, VERIFIED/REJECTED entries
 *         return to AUTO. */
void masterResetSignalVerify();

/**
 * @brief Does the ECU report supporting this OBD-II PID?
 * @return false until the bitmap has been read, and for unsupported PIDs.
 */
bool masterPidSupported(uint8_t pid);

/** @brief Count one frame this node put on the bus. */
void masterCountTx();

/** @brief Census snapshot in arrival order (no sorting), for the learner. */
size_t masterCensusRaw(CensusView *out, size_t max);

/**
 * @brief Extract a bit field from a payload with the signal decoder's rules.
 * @param start DBC start bit (LSB for Intel, MSB-first numbering for Motorola).
 */
uint64_t masterExtract(const uint8_t *d, uint8_t dlc, uint8_t start, uint8_t len, bool be);

/** @brief OBD-II has finished one full pass, so what it covers is known. */
bool diagObdSettled();

/**
 * @brief Recompute the channels that are derived rather than read — boost,
 *        AFR, injector duty, 12 V rail — from whatever sources are live.
 *        Called by both diagnostic engines after each successful exchange,
 *        and by the broadcaster before every burst (bus-read inputs).
 */
void masterUpdateDerived();
