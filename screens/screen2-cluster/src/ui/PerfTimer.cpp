/**
 * @file PerfTimer.cpp
 * @brief Performance timers - see PerfTimer.h.
 */
#include "PerfTimer.h"

static constexpr float    STILL_KMH = 1.0f;      /**< Below: standing still.    */
static constexpr uint32_t ARM_MS    = 500;       /**< Still this long: armed.   */
static constexpr uint32_t GAP_MS    = 2000;      /**< Longer: the run is lost.  */
static constexpr float    LIFT_KMH  = 10.0f;     /**< 0-100: this far below its best. */
static const float LIMIT_S[PerfTimer::RUNS] = { 30.0f, 30.0f, 60.0f };

void PerfTimer::start(Run r, double t0, float v) {
    _run[r].on = true;
    _run[r].t0 = t0;
    _run[r].vMax = v;
    _run[r].rpmMax = NAN;
}

void PerfTimer::finish(Run r, double tEnd, float trap) {
    Result res;
    res.secs = (float)((tEnd - _run[r].t0) / 1000.0);
    res.trapKmh = trap;
    res.peakRpm = _run[r].rpmMax;
    _run[r].on = false;
    if (!(res.secs > 0)) return;
    _last[r] = res;
    if (isnan(_best[r].secs) || res.secs < _best[r].secs) _best[r] = res;
}

float PerfTimer::elapsed(Run r, uint32_t tMs) const {
    if (!_run[r].on) return 0;
    const double e = ((double)tMs - _run[r].t0) / 1000.0;
    return e > 0 ? (float)e : 0;
}

void PerfTimer::resetBest() {
    for (auto &b : _best) b = Result{};
}

void PerfTimer::feed(uint32_t tMs, float v, float rpm) {
    if (!isfinite(v)) return;
    if (v < 0) v = 0;
    if (!_have) {
        _have = true; _t = tMs; _v = v;
        if (v < STILL_KMH) { _stopSince = _lastStillMs = tMs; }
        return;
    }
    // Same sample again, or the master restarted (its clock went back).
    if (tMs == _t) return;
    if ((int32_t)(tMs - _t) < 0 || tMs - _t > GAP_MS) {
        for (auto &r : _run) r.on = false;
        _launch = _armed = _midArmed = false;
        _t = tMs; _v = v;
        if (v < STILL_KMH) _stopSince = _lastStillMs = tMs;
        return;
    }
    const double t0 = _t, t1 = tMs, dt = t1 - t0;
    const float v0 = _v;
    auto cross = [&](float level) { return t0 + (double)((level - v0) / (v - v0)) * dt; };

    for (auto &r : _run)
        if (r.on) {
            if (v > r.vMax) r.vMax = v;
            if (isfinite(rpm) && (isnan(r.rpmMax) || rpm > r.rpmMax)) r.rpmMax = rpm;
        }

    // ── standing start ────────────────────────────────────────────────────
    if (v < STILL_KMH) {
        if (v0 >= STILL_KMH) _stopSince = tMs;
        _lastStillMs = tMs;
        if (tMs - _stopSince >= ARM_MS) _armed = true;
        _launch = false;
        abandon(ZERO_100);
        abandon(QUARTER);
    } else if (_armed && !_launch && v0 < STILL_KMH) {
        _launch = true;                      // rolling: the start is known next sample
        _t1 = tMs; _v1 = v;
    } else if (_launch) {
        _launch = _armed = false;
        // Back from the first two moving samples to where the speed was 0,
        // but not before the car was last seen still.
        const double slope = (v - _v1) / (t1 - (double)_t1);       // km/h a ms
        double ts = slope > 0 ? (double)_t1 - _v1 / slope : (double)_lastStillMs;
        if (ts < (double)_lastStillMs) ts = _lastStillMs;
        start(ZERO_100, ts, v);
        start(QUARTER, ts, v);
        // The distance to the first moving sample: a straight ramp from the
        // start. From there on the quarter mile integrates sample by sample.
        _dist = (float)(_v1 / 2.0 * ((double)_t1 - ts) / 3600.0);
        _run[ZERO_100].rpmMax = _run[QUARTER].rpmMax = isfinite(rpm) ? rpm : NAN;
    } else {
        _armed = false;
    }

    // 0-100 on its way.
    if (_run[ZERO_100].on && !_launch) {
        if (v0 < 100.0f && v >= 100.0f)                     finish(ZERO_100, cross(100.0f), 100.0f);
        else if (v < _run[ZERO_100].vMax - LIFT_KMH)        abandon(ZERO_100);
    }
    // The quarter mile: integrate the distance (the launch sample has it already).
    if (_run[QUARTER].on && !_launch) {
        const float before = _dist;
        _dist += (float)((v0 + v) / 2.0 * dt / 3600.0);
        if (before < QUARTER_M && _dist >= QUARTER_M) {
            const float f = (QUARTER_M - before) / (_dist - before);
            finish(QUARTER, t0 + f * dt, v0 + f * (v - v0));
        }
    }

    // ── 80-120, rolling ───────────────────────────────────────────────────
    if (v < 80.0f) _midArmed = true;
    if (_run[MID_80_120].on) {
        if (v0 < 120.0f && v >= 120.0f)  finish(MID_80_120, cross(120.0f), 120.0f);
        else if (v < 75.0f)              abandon(MID_80_120);
    } else if (_midArmed && v0 < 80.0f && v >= 80.0f) {
        _midArmed = false;
        start(MID_80_120, cross(80.0f), v);
        if (isfinite(rpm)) _run[MID_80_120].rpmMax = rpm;
        if (v >= 120.0f) finish(MID_80_120, cross(120.0f), 120.0f);
    }

    for (uint8_t r = 0; r < RUNS; r++)
        if (_run[r].on && (t1 - _run[r].t0) / 1000.0 > LIMIT_S[r]) abandon((Run)r);

    _t = tMs; _v = v;
}
