/**
 * @file PerfTimer.h
 * @brief 0-100 km/h, 80-120 km/h and the quarter mile, timed on the display
 *        from the speed the master broadcasts (and RPM, for each run's peak).
 *
 * Pure logic: speed samples in, times out - no LVGL, no store - so the host
 * tests drive it with made-up runs. Shared by both displays (identical,
 * scripts/check_sync.py).
 *
 * Times come from the master's own clock (the packet's timestamp_ms), so the
 * radio's jitter does not count; each crossing is interpolated between the
 * two samples either side of it.
 *
 * - **0-100** and the **quarter mile** start from a standstill: the car still
 *   (under 1 km/h) for half a second arms them. The start is taken back from
 *   the first two moving samples (a car's speed reads 0 until it is already
 *   rolling, and an unchanged 0 is only re-sent every 300 ms).
 *   0-100 is abandoned when the speed falls 10 km/h below its best so far;
 *   the quarter mile when the car stops. Distance is the speed integrated.
 * - **80-120** is rolling: from crossing 80 upward to 120, abandoned below
 *   75 km/h.
 * - A run longer than its limit (30 s, quarter mile 60 s), or a gap of more
 *   than 2 s in the samples, is abandoned.
 */
#pragma once
#include <math.h>
#include <stdint.h>

class PerfTimer {
public:
    enum Run : uint8_t { ZERO_100, MID_80_120, QUARTER, RUNS };

    /** @brief A finished run. secs NaN = none yet. */
    struct Result {
        float secs    = NAN;   /**< Time [s].                                   */
        float trapKmh = NAN;   /**< Speed at the finish [km/h] (quarter mile).  */
        float peakRpm = NAN;   /**< Highest RPM on the way, NaN if unknown.     */
    };

    /** @brief One speed sample. @param tMs master clock @param rpm NaN = unknown. */
    void feed(uint32_t tMs, float kmh, float rpm);

    /** @return Standing still long enough: a standing start will be timed. */
    bool ready() const { return _armed && !_launch; }
    /** @return A run is being timed. */
    bool running(Run r) const { return _run[r].on; }
    /** @return Seconds into the run at master time @p tMs (0 if not running). */
    float elapsed(Run r, uint32_t tMs) const;
    /** @return The last finished run. */
    const Result &last(Run r) const { return _last[r]; }
    /** @return The best this power-on. */
    const Result &best(Run r) const { return _best[r]; }
    /** @return Master time of the last sample (0 before the first). */
    uint32_t now() const { return _t; }
    /** @return Distance into the quarter mile [m] (0 if not running). */
    float distance() const { return _run[QUARTER].on ? _dist : 0; }
    /** @brief Forget the bests. */
    void resetBest();

    static constexpr float QUARTER_M = 402.336f;

private:
    /** Times in double: a master up for hours has more milliseconds than a
     *  float holds to the millisecond. */
    struct Live { bool on = false; double t0 = 0; float vMax = 0; float rpmMax = NAN; };
    void start(Run r, double t0, float v);
    void finish(Run r, double tEnd, float trap);
    void abandon(Run r) { _run[r].on = false; }

    bool     _have = false;      /**< A previous sample.                    */
    uint32_t _t = 0;             /**< Its master time.                      */
    float    _v = 0;             /**< Its speed.                            */
    bool     _armed = false;     /**< Stood still long enough.              */
    uint32_t _stopSince = 0;     /**< First sample of the current stop.     */
    uint32_t _lastStillMs = 0;   /**< Last sample still standing.           */
    bool     _launch = false;    /**< First moving sample seen, start pending. */
    uint32_t _t1 = 0;            /**< ...its time and speed.                */
    float    _v1 = 0;
    bool     _midArmed = false;  /**< Below 80: the next crossing starts 80-120. */
    float    _dist = 0;          /**< Quarter mile: metres so far.          */
    Live     _run[RUNS];
    Result   _last[RUNS], _best[RUNS];
};
