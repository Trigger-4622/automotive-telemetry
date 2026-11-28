/**
 * @file AutoScreen.cpp
 * @brief Screens that come up by themselves - see AutoScreen.h.
 */
#include "AutoScreen.h"

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/** The operators as the layout writes them, in ScreenCond::Op order. */
static const char *const OPS[] = {">", "<", ">=", "<=", "==", "!="};

bool screenCondCompare(ScreenCond::Op op, float a, float b) {
    switch (op) {
    case ScreenCond::GT: return a > b;
    case ScreenCond::LT: return a < b;
    case ScreenCond::GE: return a >= b;
    case ScreenCond::LE: return a <= b;
    case ScreenCond::EQ: return lroundf(a) == lroundf(b);
    case ScreenCond::NE: return lroundf(a) != lroundf(b);
    }
    return false;
}

static bool opFrom(const char *s, ScreenCond::Op &op) {
    for (int i = 0; i < 6; i++)
        if (s && !strcmp(s, OPS[i])) { op = (ScreenCond::Op)i; return true; }
    return false;
}

bool AutoScreens::parseCond(JsonVariantConst v, ScreenCond &c) {
    c = ScreenCond{};
    if (!v.is<JsonObjectConst>()) return false;
    const char *gear = v["gear"] | (const char *)nullptr;
    if (gear) {
        if (!gear[0] || gear[1]) return false;                 // one letter or digit
        c.gear = (char)toupper((unsigned char)gear[0]);
        if (!strchr("PRNDM123456789", c.gear)) return false;
        if (!opFrom(v["op"] | "==", c.op) || (c.op != ScreenCond::EQ && c.op != ScreenCond::NE))
            return false;
        return true;
    }
    JsonVariantConst id = v["metric_id"];
    const unsigned long m = id.is<const char *>() ? strtoul(id.as<const char *>(), nullptr, 0)
                                                  : id.as<unsigned long>();
    if (!m || m > 0xFFFF) return false;
    c.metric = (uint16_t)m;
    if (!opFrom(v["op"] | ">", c.op)) return false;
    const float val = v["value"] | NAN;
    if (!isfinite(val)) return false;
    c.value = val;
    return true;
}

void AutoScreens::add(JsonObjectConst screen, size_t index) {
    Rule r;
    r.screen = index;
    for (JsonVariantConst v : screen["show_when"].as<JsonArrayConst>()) {
        ScreenCond c;
        if (!parseCond(v, c)) {
            log_w("screen %u: a show_when condition will not read - it never comes up by itself",
                  (unsigned)index);
            return;
        }
        r.when.push_back(c);
    }
    if (r.when.empty()) return;
    for (JsonVariantConst v : screen["back_now_when"].as<JsonArrayConst>()) {
        ScreenCond c;
        if (parseCond(v, c)) r.backNow.push_back(c);
        else log_w("screen %u: a back_now_when condition will not read - left out", (unsigned)index);
    }
    float s = screen["back_after_s"] | 10.0f;
    if (!(s >= 0)) s = 10.0f;                                  // and NaN
    if (s > 3600) s = 3600;
    r.backMs = (uint32_t)lroundf(s * 1000);
    _rules.push_back(r);
}

bool AutoScreens::anyHolds(const std::vector<ScreenCond> &cs, const ScreenCondTest &holds) {
    for (const ScreenCond &c : cs)
        if (holds(c)) return true;
    return false;
}

int AutoScreens::tick(size_t active, uint32_t now, const ScreenCondTest &holds) {
    if (_rules.empty()) return -1;
    for (Rule &r : _rules) {
        r.on = true;
        for (const ScreenCond &c : r.when)
            if (!holds(c)) { r.on = false; break; }
    }
    int to = -1;
    // The screen a rule brought up stays while its conditions hold, then for
    // the wait - unless one of its "back at once" conditions holds.
    if (_cur >= 0) {
        Rule &c = _rules[(size_t)_cur];
        if (active != c.screen) {
            _cur = -1;                                         // changed meanwhile
        } else if (c.on) {
            _off = false;
        } else {
            if (!_off) { _off = true; _offMs = now; }
            // Elapsed time, unsigned: right across the millis() wrap.
            if (anyHolds(c.backNow, holds) || now - _offMs >= c.backMs) {
                int next = -1;
                for (size_t i = 0; i < _rules.size(); i++)
                    if ((int)i != _cur && _rules[i].on) { next = (int)i; break; }
                _off = false;
                if (next >= 0) {
                    _cur = next;
                    if (_rules[(size_t)next].screen != active) to = (int)_rules[(size_t)next].screen;
                } else {
                    _cur = -1;
                    to = (int)_back;
                }
            }
        }
    }
    // Conditions that have just come to hold bring their screen up: the first
    // in screen order, if several do at once.
    bool taken = false;
    for (size_t i = 0; i < _rules.size(); i++) {
        Rule &r = _rules[i];
        const bool rose = r.on && !r.was;
        r.was = r.on;
        if (!rose || taken || (int)i == _cur) continue;
        const size_t shown = to >= 0 ? (size_t)to : active;
        if (_cur < 0 && r.screen == shown) continue;           // there already, by choice
        if (_cur < 0) _back = shown;
        _cur = (int)i;
        _off = false;
        to = (int)r.screen;
        taken = true;
    }
    return to == (int)active ? -1 : to;
}
