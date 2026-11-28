/**
 * @file AutoScreen.h
 * @brief Screens that come up by themselves: a screen's own conditions -
 *        on the gear, the speed or any metric - in layout.json.
 *
 * Shared by both displays (identical, scripts/check_sync.py). Keys on a
 * screen (Gauge Studio: "Show this screen by itself"):
 *
 *   "show_when":     [ condition, ... ]  all hold: the display changes to it
 *   "back_after_s":  10                  they stop holding: back this much later
 *   "back_now_when": [ condition, ... ]  one holds while waiting: back at once
 *
 * A condition is {"gear": "R"} (or "op": "!=" for "is not"; P R N D M or a
 * gear number - the reverse switch, the lever's letter and the gear number all
 * count, as the gear readout shows them) or {"metric_id": "0x010D", "op": ">",
 * "value": 15}, op one of > < >= <= == != ("==" and "!=" compare whole
 * numbers). A value the display has not got, or has not had for the stale
 * time, holds no condition - so nothing comes up on missing data. A rule with
 * a condition that will not read is left out whole (a part of it would be a
 * different rule).
 *
 * Only a change brings a screen up: conditions coming to hold. Already on it
 * by the driver's choice, the screen is not taken away when they stop. A
 * swipe while an automatic screen is up keeps the driver's screen, and a
 * screen another rule brings up meanwhile goes back to the screen before the
 * first. When a rule's wait is over and another one's conditions hold, that
 * one's screen comes up instead of the one before.
 */
#pragma once
#include <ArduinoJson.h>
#include <stddef.h>
#include <stdint.h>

#include <functional>
#include <vector>

/** @brief One condition of a screen's rule. */
struct ScreenCond {
    enum Op : uint8_t { GT, LT, GE, LE, EQ, NE };
    uint16_t metric = 0;   /**< The metric compared; 0 = the gear.             */
    char     gear   = 0;   /**< The gear: 'P' 'R' 'N' 'D' 'M' or '1'-'9'.      */
    Op       op     = GT;
    float    value  = 0;
};

/** @brief Compare as a condition does: "==" and "!=" on whole numbers. */
bool screenCondCompare(ScreenCond::Op op, float a, float b);

/** @brief Whether a condition holds now (false when what it reads is unknown). */
using ScreenCondTest = std::function<bool(const ScreenCond &)>;

/** @brief Every screen's rule, and which one has its screen up. */
class AutoScreens {
public:
    /** @brief Read screen @p index's rule, if it has one. At boot. */
    void add(JsonObjectConst screen, size_t index);
    /** @brief Once a tick. @return The screen to change to, or -1. */
    int  tick(size_t active, uint32_t now, const ScreenCondTest &holds);
    /** @brief The driver changed screens: theirs stays. */
    void swiped() { _cur = -1; _off = false; }
    size_t rules() const { return _rules.size(); }

private:
    struct Rule {
        size_t   screen = 0;
        std::vector<ScreenCond> when, backNow;
        uint32_t backMs = 10000;
        bool     on = false, was = false;
    };
    static bool parseCond(JsonVariantConst v, ScreenCond &c);
    static bool anyHolds(const std::vector<ScreenCond> &cs, const ScreenCondTest &holds);

    std::vector<Rule> _rules;
    int      _cur  = -1;      /**< The rule whose screen is up, or -1.          */
    size_t   _back = 0;       /**< The screen before the first rule's.          */
    bool     _off  = false;   /**< _cur's conditions no longer hold...          */
    uint32_t _offMs = 0;      /**< ...since this millis().                      */
};
