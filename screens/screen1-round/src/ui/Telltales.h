/**
 * @file Telltales.h
 * @ingroup ui
 * @brief Warning lamps ("telltales"): check engine, seat belt, doors,
 *        parking brake, indicators, oil, charging, temperature and the rest.
 *
 * THE THIRD ALARM PATH. UIBuilder.h describes the other two: per-gauge
 * thresholds colour one widget, and the watchdog polices a number against
 * limits. A lamp is neither. It is a yes/no condition that behaves the way the
 * matching lamp in the car's own cluster does:
 *
 *   - it lights while its condition holds, after a debounce, and can be held
 *     on for a moment after it clears so a flicker is still seen;
 *   - some only mean something with the engine running (oil pressure,
 *     charging) and are gated on it;
 *   - some only matter on the move (belt, door, parking brake): the lamp shows
 *     whenever the condition holds, but it only pops up and flashes above a
 *     speed;
 *   - indicators follow the bus when it reports the flasher, and flash by
 *     themselves when it only reports the lever.
 *
 * Every lamp, and every behaviour of every lamp, can be switched off from the
 * layout's `"warnings"` block — see @ref Telltales::configure. A lamp whose
 * metric has never been heard, or has gone stale, is simply dark: the absence
 * of data is not a fault the driver can do anything about.
 *
 * This class holds no LVGL objects. UIBuilder draws the lamps (the strip
 * overlay, the dash screen's lamp row and the alert card) from the state
 * computed here, which is what lets the logic be tested on a PC.
 *
 * Identical in both display projects.
 */
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <lvgl.h>

class TelemetryStore;

/** @brief Lamp colour, by what it asks of the driver (ISO 2575). */
enum class LampTone : uint8_t {
    Red,     /**< Act now.                 */
    Amber,   /**< Check soon.              */
    Green,   /**< Information: on.         */
    Blue     /**< Information: high beam, engine cold. */
};

/** @brief How a lamp's metric turns into on/off. */
enum class LampRule : uint8_t {
    On,      /**< Lit while value >= 0.5 (a bit that is set).     */
    Off,     /**< Lit while value <  0.5 (a bit that is clear).   */
    Above,   /**< Lit while value >= threshold (with hysteresis). */
    Below    /**< Lit while value <= threshold (with hysteresis). */
};

/** @brief Whether a lit lamp raises the alert card. */
enum class LampPopup : uint8_t {
    Off,     /**< Never; the lamp alone says it.                             */
    Once,    /**< Once per lighting; a tap or popup_secs dismisses it.        */
    Repeat   /**< Until tapped, then again every alert_ack_ms while lit.      */
};

/** @brief Symbol drawn for a lamp. Order matches the table in Telltales.cpp. */
enum LampIcon : uint8_t {
    ICON_MIL, ICON_SEATBELT, ICON_DOOR, ICON_PARK, ICON_TURN_L, ICON_TURN_R,
    ICON_HIGH_BEAM, ICON_LIGHTS, ICON_CRUISE, ICON_FUEL, ICON_BATTERY,
    ICON_OIL, ICON_COOLANT, ICON_KNOCK, ICON_WARN,
    ICON_COUNT
};

/** @brief One lamp: its configuration and its live state. */
struct Lamp {
    /** @name Configuration — defaults, then the layout's overrides
     *  @{ */
    char      key[14];        /**< Layout key, e.g. "seatbelt".               */
    char      title[24];      /**< Alert headline, e.g. "FASTEN SEAT BELT".   */
    char      detail[36];     /**< Alert sub-line.                            */
    char      units[8];       /**< Units of the value shown on the alert.     */
    uint8_t   icon;           /**< @ref LampIcon.                             */
    LampTone  tone;           /**< Lit colour.                                */
    bool      enabled;        /**< false: always dark, never pops up.         */
    bool      strip;          /**< Shown on the strip over normal screens.    */
    uint16_t  metric;         /**< Source metric; 0 = none (lamp stays dark). */
    LampRule  rule;           /**< How the metric lights the lamp.            */
    float     threshold;      /**< Above/Below limit.                         */
    float     hyst;           /**< Band the value must travel back to clear.  */
    bool      engine;         /**< Only judged once the engine has run a while.*/
    bool      turn;           /**< Indicator: follows or makes its own flash. */
    bool      critical;       /**< Alert at critical severity (pulsing).      */
    LampPopup popup;          /**< Alert card behaviour.                      */
    float     popupSpeed;     /**< Pop up only at/above this [km/h]; 0 = any. */
    float     blinkSpeed;     /**< Flash at/above this [km/h]; < 0 = never.   */
    uint16_t  popupSecs;      /**< Auto-dismiss the alert; 0 = until tapped.  */
    uint16_t  debounceMs;     /**< Condition must hold this long first.       */
    uint16_t  holdMs;         /**< Stays lit this long after it clears.       */
    uint16_t  extraMetric;    /**< Shown as the alert's value, e.g. DTC count.*/
    uint8_t   decimals;       /**< Decimals of the alert's value.             */
    /** @} */

    /** @name State — what the UI draws
     *  @{ */
    bool      on;             /**< The lamp is showing.                       */
    bool      lit;            /**< Bright in this flash phase; false = dark.  */
    float     value;          /**< Latest value of @ref metric, NaN if none.  */
    float     extra;          /**< Latest @ref extraMetric value, NaN if none.*/
    /** @} */

    /** @name Bookkeeping — private to Telltales
     *  @{ */
    bool      cond;           /**< Condition latched through hysteresis.      */
    bool      pending;        /**< Debounce is running.                       */
    uint32_t  condSince;      /**< When the debounce started.                 */
    uint32_t  lastTrueMs;     /**< Last time the condition held (for hold).   */
    uint32_t  onSince;        /**< When the lamp came on (flash phase origin).*/
    bool      rawPrev;        /**< Indicator: previous bus state.             */
    uint32_t  edgeMs;         /**< Indicator: last change of the bus state.   */
    bool      speedGate;      /**< Moving fast enough to pop up (hysteresis). */
    bool      spent;          /**< Once: already shown for this lighting.     */
    bool      snoozed;        /**< Repeat: tapped, quiet until snoozeUntil.   */
    uint32_t  snoozeUntil;
    bool      showing;        /**< The alert card showed it on the last pass. */
    uint32_t  shownSince;     /**< Start of the current display (popup_secs). */
    uint32_t  lastShownMs;
    /** @} */
};

/**
 * @brief The lamp set, evaluated against the telemetry store.
 *
 * Owned by UIBuilder. Call @ref configure once, then @ref update every UI
 * tick; everything else reads state.
 */
class Telltales {
public:
    /** Room for every built-in lamp plus a margin. At most 32: the strip
     *  keeps one bit per lamp. */
    static constexpr size_t MAX_LAMPS = 20;

    /** Engine speed above which the engine counts as running [rpm]. */
    static constexpr float    ENGINE_RPM       = 400.0f;
    /** How long it must have run before engine-gated lamps are judged. Oil
     *  pressure and charge voltage take a moment to come up after a start,
     *  and a lamp that flashes on every start teaches the driver to ignore it. */
    static constexpr uint32_t ENGINE_SETTLE_MS = 3000;
    /** Half-period of a warning lamp's flash [ms]. */
    static constexpr uint32_t BLINK_HALF_MS    = 400;
    /** An indicator reported steady for longer than this is the lever, not
     *  the flasher, and the lamp makes its own flash [ms]. */
    static constexpr uint32_t TURN_STEADY_MS   = 700;
    /** Half-period of a self-made indicator flash: 85 flashes a minute,
     *  inside the 60-120 the regulations allow [ms]. */
    static constexpr uint32_t TURN_HALF_MS     = 350;

    /**
     * @brief Load the built-in lamps, then apply the layout's overrides.
     *
     * @param warnings The layout's `"warnings"` value. Absent means every
     *                 default applies; `false` switches the whole feature off.
     *                 Otherwise an object:
     * @code
     * "warnings": {
     *   "enabled": true,           // everything below, at once
     *   "strip": true,             // lamp strip over the normal screens
     *   "strip_mode": "peek",      // or "always" - see stripFull(); the
     *                              // default follows the panel's shape
     *   "peek_secs": 6,
     *   "strip_position": "bottom",// or "top"
     *   "popups": true,            // lamps may raise the alert card
     *   "lamps": {
     *     "knock":    false,       // a lamp can be switched off outright
     *     "seatbelt": { "popup": "repeat", "popup_speed": 10, "blink_speed": 10 },
     *     "oil":      { "metric_id": "0x210D", "rule": "on" },
     *     "custom_1": { "enabled": true, "metric_id": "0x3001", "rule": "on",
     *                   "title": "ABS", "tone": "amber", "icon": "warn" }
     *   }
     * }
     * @endcode
     * Per-lamp keys: enabled, strip, metric_id, rule (on/off/above/below), threshold,
     * hysteresis, engine, debounce_ms, hold_ms, popup (off/once/repeat or a
     * bool), popup_speed, blink_speed, popup_secs, critical, tone
     * (red/amber/green/blue), icon, title, detail, units, decimals,
     * extra_metric_id.
     * @param snoozeMs How long a tapped Repeat lamp stays quiet
     *                 (global.alert_ack_ms).
     */
    void configure(JsonVariantConst warnings, uint32_t snoozeMs);

    /**
     * @brief Re-evaluate every lamp.
     * @param store Telemetry to read, with peek(): lamps never disturb the
     *              needles' interpolation.
     * @param now   millis().
     */
    void update(TelemetryStore &store, uint32_t now);

    /** @return Number of lamps, enabled or not. */
    size_t size() const { return _n; }
    /** @param i Index below size(). @return That lamp. */
    Lamp &operator[](size_t i) { return _lamps[i]; }
    /** @param i Index below size(). @return That lamp. */
    const Lamp &operator[](size_t i) const { return _lamps[i]; }

    /** @param key Layout key. @return Its index, or -1. */
    int find(const char *key) const;

    /** @return The feature is on at all. */
    bool enabled() const { return _enabled; }
    /** @return The lamp strip should be drawn over the normal screens. */
    bool strip() const { return _enabled && _strip; }
    /** @return The strip goes along the top edge instead of the bottom. */
    bool stripTop() const { return _stripTop; }

    /**
     * @brief Whether the strip shows every lit lamp right now, or has folded
     *        down to a badge of the most urgent fault.
     *
     * Any fixed place on a screen covers something on some screen - a chart
     * legend, the last bar, a unit. So by default ("peek") the full strip
     * shows for peek_secs whenever a lamp comes on, then folds to the one
     * most urgent red or amber lamp plus "+N": a fault is never out of sight,
     * and the data under the strip is only covered briefly. Indicators keep
     * the full strip up while they flash - that is the moment they matter.
     * Information lamps (high beam, cruise) simply go after the peek; the
     * dash screen shows them all the time. "strip_mode": "always" keeps the
     * whole strip up for as long as anything is lit.
     * @return true for the full strip.
     */
    bool stripFull() const { return _stripAlways || _peeking || _turnOn; }

    /**
     * @brief Does lamp @p i belong on the strip right now?
     * @param i Lamp index.
     * @return true when it is lit, allowed on the strip, and either the strip
     *         is full or it is a red or amber (fault) lamp.
     */
    bool onStrip(size_t i) const;
    /** @return Lamps may raise the alert card. */
    bool popups() const { return _enabled && _popups; }

    /**
     * @brief Does this lamp want the alert card right now?
     * @param i   Lamp index.
     * @param now millis().
     * @return true when lit, poppable, fast enough, and not dismissed.
     */
    bool wantsPopup(size_t i, uint32_t now) const;

    /**
     * @brief Tell the lamp the alert card is showing it. Call every pass
     *        while it is; this is what times popup_secs.
     * @param i   Lamp index.
     * @param now millis().
     */
    void shown(size_t i, uint32_t now);

    /**
     * @brief The driver tapped the card while it showed this lamp.
     *
     * A Once lamp stays quiet until it goes out and lights again; a Repeat
     * lamp for alert_ack_ms.
     * @param i   Lamp index.
     * @param now millis().
     */
    void acknowledge(size_t i, uint32_t now);

    /**
     * @brief Popup priority: lower is more urgent.
     * @param i Lamp index.
     * @return Critical lamps first, then by colour: red, amber, blue, green.
     */
    uint8_t rank(size_t i) const;

    /**
     * @brief The value line for the alert card: "2 CODES", "11.4 V", or "".
     * @param i        Lamp index.
     * @param[out] out Destination.
     * @param n        Size of @p out.
     */
    void valueText(size_t i, char *out, size_t n) const;

    /**
     * @brief Colour of the most urgent lit lamp drawn with @p icon.
     *
     * Two lamps can share a symbol (engine hot is red, engine cold blue) and a
     * dash has one place for it.
     * @param icon      @ref LampIcon.
     * @param[out] rgb  0xRRGGBB when a lamp is on (dark phase included).
     * @param[out] lit  Whether that lamp is in its bright phase.
     * @return true when some enabled lamp with this icon is on.
     */
    bool iconState(uint8_t icon, uint32_t &rgb, bool &lit) const;

    /** @return Engine running long enough for engine-gated lamps. */
    bool engineSettled() const { return _engineSettled; }
    /** @return Vehicle speed [km/h]; 0 when unknown. */
    float speed() const { return _speed; }

    /** @param t Tone. @return Its colour, 0xRRGGBB. */
    static uint32_t toneRgb(LampTone t);
    /**
     * @param icon  @ref LampIcon.
     * @param large false for the 24x24 bitmap (lamp rows), true for 36x36
     *              (the alert card - alpha images cannot be zoomed).
     * @return The alpha bitmap.
     */
    static const lv_img_dsc_t *iconImage(uint8_t icon, bool large = false);
    /** @param name "mil", "seatbelt"… @return The icon, or -1. */
    static int iconFromName(const char *name);

private:
    /** @brief Fill the table with the built-in lamps. */
    void loadDefaults();
    /**
     * @brief Append one built-in lamp with its common fields set.
     * @return The new lamp, for the caller to adjust.
     */
    Lamp &add(const char *key, const char *title, const char *detail,
              uint8_t icon, LampTone tone, uint16_t metric, LampRule rule,
              float threshold = 0, float hyst = 0);
    /** @brief Apply one lamp's layout overrides. */
    void override(Lamp &l, JsonVariantConst o);
    /** @brief Clear a lamp's live state (off, no timers, no dismissal). */
    static void reset(Lamp &l);

    Lamp     _lamps[MAX_LAMPS] = {};
    size_t   _n        = 0;
    bool     _enabled  = true;
    bool     _strip    = true;
    bool     _stripTop = false;
    bool     _popups   = true;
    bool     _stripAlways = false;
    uint32_t _peekMs   = 6000;
    uint32_t _snoozeMs = 30000;

    uint32_t _stripLit  = 0;      /**< Lamps lit on the strip, bit per lamp. */
    bool     _peeking   = false;  /**< Full strip after a lamp came on.     */
    uint32_t _peekSince = 0;
    bool     _turnOn    = false;  /**< An indicator is on.                  */

    float    _speed          = 0;
    bool     _engineRunning  = false;
    uint32_t _engineSince    = 0;
    bool     _engineSettled  = false;
};
