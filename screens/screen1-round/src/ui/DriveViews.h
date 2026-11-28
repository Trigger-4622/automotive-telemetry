/**
 * @file DriveViews.h
 * @brief The driving screens: the G-meter (a friction-circle dot), the
 *        performance timers (0-100, 80-120, quarter mile), the parking
 *        screen (the steering wheel as the driver has it turned, or the car
 *        from above with its path) and the car seen from above (what is open,
 *        the lights, and the path it is about to take).
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

/* ══════════════════════════ the car from above ══════════════════════════ */

/** @brief What the car from above shows: each -1 while it is not known. */
struct CarState {
    int8_t door[4] = {-1, -1, -1, -1};  /**< FL FR RL RR: 1 open, 0 shut.       */
    int8_t anyDoor = -1;                /**< "A door open", where the single
                                             doors are not known.               */
    int8_t boot = -1, bonnet = -1;
    bool   turnL = false, turnR = false; /**< Each indicator lit now.          */
    int8_t lights = -1, highBeam = -1, handbrake = -1;
    bool   angleOk = false;
    float  angle = 0;                   /**< Steering wheel [deg], + left.      */
    bool   reverse = false;             /**< The path goes back, not ahead.     */
};

/** @brief The car's measures for the path: from the layout. */
struct CarGeom {
    float ratio = 15.0f;       /**< Steering wheel turns per road wheel turn.  */
    float wheelbase = 2.75f;   /**< [m] - the Legacy BM's.                     */
    bool  path = true;         /**< Draw the path the car is about to take.    */
    float pathM = 3.0f;        /**< Its length at most [m] (and the room).     */
};

/** @brief Number of points along each path line. */
static constexpr int CAR_PATH_PTS = 12;

/**
 * @brief The car seen from above, front up: the body, the doors (swung out in
 *        amber when open), boot and bonnet, the indicators at the corners, the
 *        headlights, the front wheels turned by the steering, and the path the
 *        car's corners will take - back from the rear bumper in reverse, ahead
 *        from the front one otherwise.
 *
 * The path is the single-track model: the road wheels turned by the steering
 * angle over the steering ratio, about a centre on the rear axle's line.
 */
struct CarView {
    lv_obj_t  *body = nullptr, *cabin = nullptr;
    lv_obj_t  *side[6] = {};        /**< Doors FL FR RL RR, boot, bonnet (lines). */
    lv_obj_t  *wheel[4] = {};       /**< Front two lines (they turn), rear two.   */
    lv_obj_t  *corner[4] = {};      /**< Indicator lamps FL FR RL RR.             */
    lv_obj_t  *head[2] = {};        /**< Headlights.                              */
    lv_obj_t  *path[2] = {};
    lv_point_t *pts = nullptr;      /**< Ends of every line (LVGL keeps them).    */
    CarGeom    g;
    float      ppm = 20;            /**< Pixels a metre.                          */
    int        cx = 0, axleY = 0;   /**< Centre line and rear axle, in the parent. */
    int        top = 0, bottom = 0; /**< The room the path may use.               */
    int16_t    lastDeg = INT16_MIN;
    int8_t     lastDir = 0;         /**< -1 back, 1 ahead, 0 no path shown.       */
    int8_t     lastSide[6] = {9, 9, 9, 9, 9, 9};
    uint8_t    lastLamps = 0xFF;
};

/** @brief Build the car in @p parent, centred on @p cx, its rear axle at
 *         @p axleY, @p ppm pixels a metre; the path keeps within top..bottom. */
void carBuild(CarView &v, lv_obj_t *parent, int cx, int axleY, float ppm, int top, int bottom,
              const CarGeom &g);
/** @brief Show @p s: only what changed is redrawn. */
void carUpdate(CarView &v, const CarState &s);

/** @brief The "car" screen: the car from above and what is open, in words. */
struct CarScreenView {
    CarView   car;
    lv_obj_t *status = nullptr;     /**< One line over the car (round).           */
    lv_obj_t *vals = nullptr;       /**< Doors, boot, bonnet, lights, handbrake:
                                         one label, a line each (landscape).     */
    char      lastStatus[40] = "";
    char      lastVals[160] = "";
    uint32_t  lastStatusRgb = 0;
};
void carScreenBuild(CarScreenView &v, lv_obj_t *parent, int w, int h, const CarGeom &g);
void carScreenUpdate(CarScreenView &v, const CarState &s);

/** @brief The parking screen's widgets and what they show now. */
struct ParkView {
    uint16_t  metric = 0;              /**< The steering angle's metric.        */
    lv_obj_t *rim = nullptr, *lock = nullptr;
    lv_obj_t *line[4] = {};            /**< Three spokes, then the top stripe.  */
    lv_point_t *pts = nullptr;         /**< Their ends (LVGL keeps the address). */
    lv_obj_t *value = nullptr, *word = nullptr;
    CarView   car;                     /**< The "path" view: the car from above. */
    bool      pathView = false;
    int       cx = 0, cy = 0, r = 0;   /**< The wheel, in the parent.           */
    int       hub = 0;                 /**< Where the spokes start.             */
    float     steerMax = 540;          /**< Full lock, either way [deg].        */
    float     straight = 10;           /**< "Straight" within this [deg].       */
    int16_t   lastDeg = INT16_MIN;
    int8_t    lastSide = 2;            /**< 1 left, -1 right, 0 straight.       */
    bool      lastOk = true;
    char      lastValue[12] = "", lastWord[24] = "";
};

/**
 * @brief Build the parking screen in @p parent (w x h px): the steering wheel,
 *        turned as the driver has it, an arc from straight ahead to full lock,
 *        the angle, and which way - or STRAIGHT, in green, within
 *        @p straightDeg of the centre. With @p pathView the car from above
 *        and its path take the wheel's place.
 * @param steerMax Full lock either way [deg] (the arc's ends).
 */
void parkBuild(ParkView &v, lv_obj_t *parent, int w, int h, float steerMax, float straightDeg,
               bool pathView = false, const CarGeom &g = CarGeom());
/**
 * @brief Turn the wheel. ISO 8855 signs, as the master sends the angle:
 *        positive is to the left (the wheel turns anticlockwise).
 * @param ok Whether the angle is known; if not, "--" and no wheel position.
 * @param car The rest of the car, for the path view (may be null).
 */
void parkUpdate(ParkView &v, float deg, bool ok, const CarState *car = nullptr);
