/**
 * @file DriveViews.h
 * @brief Two driving screens: the G-meter (a friction-circle dot) and the
 *        performance timers (0-100, 80-120, quarter mile).
 *
 * Shared by both displays (identical, scripts/check_sync.py): each lays out
 * in whatever area its UIBuilder gives it - the round board's circle, the
 * cluster's landscape panel. Built once, then updated every UI tick; text and
 * positions are only written when they change, as everywhere else.
 */
#pragma once
#include <lvgl.h>
#include <stdint.h>

#include "PerfTimer.h"

/** @brief The G-meter's widgets and what they show now. */
struct GMeterView {
    lv_obj_t *dot = nullptr;           /**< The car's acceleration.            */
    lv_obj_t *lat = nullptr, *lon = nullptr, *peak = nullptr;
    int       cx = 0, cy = 0, r = 0;   /**< The circle, in the parent.         */
    float     maxG = 1.0f;             /**< Its rim.                           */
    float     peakG = 0;               /**< Most combined G this power-on.     */
    int16_t   lastX = INT16_MIN, lastY = INT16_MIN;
    bool      lastShown = true;
    char      lastLat[16] = "", lastLon[16] = "", lastPeak[20] = "";
};

/**
 * @brief Build the G-meter in @p parent (w x h px).
 * @param maxG The rim, in g (the inner ring is half of it).
 */
void gmeterBuild(GMeterView &v, lv_obj_t *parent, int w, int h, float maxG);
/**
 * @brief Move the dot. ISO 8855 signs, as the master sends them: lateral G
 *        positive turning left, longitudinal positive speeding up. The dot
 *        shows where the car accelerates: up speeding up, down braking, left
 *        in a left-hand bend.
 * @param latOk,lonOk Whether each is known; neither = the dot hides.
 */
void gmeterUpdate(GMeterView &v, float latG, bool latOk, float lonG, bool lonOk);

/** @brief The timers' widgets and what they show now. */
struct PerfView {
    lv_obj_t *status = nullptr;
    lv_obj_t *val[PerfTimer::RUNS] = {}, *best[PerfTimer::RUNS] = {};
    lv_obj_t *trap = nullptr;          /**< The quarter mile's speed at the line. */
    char      lastTrap[20] = "";
    char      lastStatus[28] = "";
    uint32_t  lastStatusColor = 0;
    char      lastVal[PerfTimer::RUNS][16] = {}, lastBest[PerfTimer::RUNS][32] = {};
    uint32_t  lastColor[PerfTimer::RUNS] = {};
};

/** @brief Build the timers in @p parent (w x h px). */
void perfBuild(PerfView &v, lv_obj_t *parent, int w, int h);
/**
 * @param p          The timers.
 * @param masterNow  The master's clock now (for a run in progress).
 * @param speedKnown A speed is arriving at all.
 */
void perfUpdate(PerfView &v, const PerfTimer &p, uint32_t masterNow, bool speedKnown);
