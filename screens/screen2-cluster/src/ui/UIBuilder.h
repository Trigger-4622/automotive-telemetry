/**
 * @file UIBuilder.h
 * @ingroup ui
 * @brief Dynamic gauge engine: turns layout.json into swipeable LVGL screens
 *        on the circular canvas and animates them from the TelemetryStore.
 *
 * Screen types understood from layout.json:
 *   "arc_gauge"  — one large arc + big numeric readout (single metric)
 *   "meter_gauge"— radial tachometer with needle, ticks & colored zones
 *   "quad"       — 2 or 4 metric cells split across the circle
 *   "text_list"  — up to 5 label/value rows (compact data screen)
 *   "chart"      — rolling trend graph of one metric
 *   "dash"       — mini instrument cluster: tach, speed, gear, fuel,
 *                  temperature, shift light and a row of warning lamps
 *   "diag"       — link diagnostics (rate, drops, heap, uptime, GPIO scan)
 *
 * Circular-canvas discipline: every screen owns a 240×240 root container with
 * LV_RADIUS_CIRCLE + clip_corner=true, so child widgets are masked to the
 * physical round glass and never clip awkwardly at the edges.
 *
 * THREE INDEPENDENT ALARM PATHS — do not confuse them:
 *
 *  1. Per-gauge thresholds ("thresholds" on a screen) are cosmetic. They
 *     recolor and pulse that one widget on that one screen. Nothing pops up.
 *
 *  2. The WATCHDOG ("watchdog.items" in layout.json) is the guard. It is a
 *     standalone list of metrics to police, evaluated against the telemetry
 *     store directly — the metric does not need to appear on any screen. When
 *     one crosses its limit, a warning-triangle card is raised on
 *     lv_layer_top, so it is visible no matter which screen is showing. A tap
 *     acknowledges and mutes that one fault for alert_ack_ms.
 *
 *  3. WARNING LAMPS ("warnings" in layout.json; Telltales.h) are the car's
 *     own telltales: check engine, seat belt, doors, parking brake, oil,
 *     charging, indicators... They light on a strip over the screens and on
 *     the dash screen, and the serious ones can raise the same alert card.
 *
 * Master-side METRIC_FLAG_WARNING/CRITICAL bits escalate a watched metric.
 */

/**
 * @defgroup ui User Interface
 * @brief The dynamic gauge engine — screens, alarms and the watchdog card.
 * @{
 */
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <lvgl.h>
#include <vector>

#include "TelemetryStore.h"
#include "Telltales.h"

/** @brief Three-tier alarm state, shared by gauges and the watchdog. */
enum class AlarmState : uint8_t {
    Normal,    /**< Within limits.                    */
    Warning,   /**< Warning threshold crossed.        */
    Critical   /**< Critical threshold crossed.       */
};

/** @brief Which LVGL widget family renders a binding. */
enum class WidgetKind : uint8_t {
    Arc,       /**< Large arc plus numeric readout.   */
    Meter,     /**< Needle, ticks and colored zones.  */
    Cell,      /**< One cell of a grid.               */
    TextRow,   /**< One label/value row of a list.    */
    Chart,     /**< Rolling strip chart.              */
    Bar        /**< Horizontal bar (landscape only).  */
};

/** @name Landscape capacity limits
 *  Higher than the round board's, which is the point of the bigger panel.
 *  @{ */
static constexpr size_t GRID_MAX_CELLS    = 8;  /**< 4 columns x 2 rows.     */
static constexpr size_t TEXTLIST_MAX_ROWS = 10; /**< 2 columns of 5.         */
static constexpr size_t BARS_MAX_ROWS     = 6;  /**< Stacked bar gauges.     */
/** @brief Traces overlaid on one chart. Beyond four the legend stops fitting
 *  and the traces stop being distinguishable anyway. */
static constexpr size_t CHART_MAX_SERIES  = 4;
/** @} */

/**
 * @brief How a screen change is animated.
 * @note Configurable because the cheapest option is also the smoothest on
 *       this hardware — see @ref UIBuilder::loadScreen for the cost of each.
 */
enum class SwipeStyle : uint8_t {
    Over,   /**< Only the incoming screen moves. Cheapest. */
    Move,   /**< Both screens slide. Costs the most.       */
    Fade,   /**< Cross-fade; blends every pixel.           */
    None    /**< Instant. Always perfectly smooth.         */
};

/** @brief How a binding's number is turned into text. */
enum class ValueFmt : uint8_t {
    Number,   /**< The value with its decimals.                     */
    Gear      /**< -1 = "R", 0 = "N", 1-9 as a digit.               */
};

/** @brief One alarm tier parsed from layout.json. */
struct ThresholdCfg {
    bool       enabled = false;  /**< Tier present in the layout.            */
    bool       above   = true;   /**< true: alarm when v >= value            */
    float      value   = 0;      /**< The limit itself.                      */
    float      hyst    = 0;      /**< hysteresis band for clearing           */
    lv_color_t color   = {};     /**< Accent color for this tier.            */
    String     overlayText;      /**< critical only: headline on the card    */
};

/** @brief A metric bound to a set of LVGL widgets on one screen. */
struct GaugeBinding {
    uint16_t   metricId = 0;        /**< Metric to display — @ref MetricIDs. */
    String     label;               /**< Caption shown above the value.      */
    String     units;               /**< Unit suffix, e.g. "°C".             */
    uint8_t    decimals = 0;        /**< Decimal places in the readout.      */
    float      minV = 0;            /**< Scale minimum.                      */
    float      maxV = 100;          /**< Scale maximum (always > minV).      */
    WidgetKind kind = WidgetKind::Arc;  /**< Which widget family renders it. */
    float      tickLabelDiv = 1;    /**< meter: divide tick labels (x1000)   */
    /** Meter and chart: they run in units of 1/unit. LVGL's meter and chart
     *  are integer-valued, so a 0-1.6 bar dial would have two needle positions
     *  and an 11-15 V trend five levels (see intUnitFor). A power of ten. */
    float      unit = 1;
    ValueFmt   fmt = ValueFmt::Number;  /**< How the readout is written.     */
    bool       inlineUnits = false; /**< Readout carries its units: "63 %". */

    /** @name Widget handles
     *  Any of these may be nullptr depending on @ref kind.
     *  @{ */
    lv_obj_t  *arc        = nullptr;  /**< Arc indicator.                    */
    lv_obj_t  *meter      = nullptr;  /**< Meter widget owning the needle.   */
    lv_meter_indicator_t *needle = nullptr;  /**< Needle indicator.          */
    lv_obj_t  *valueLabel = nullptr;  /**< Numeric readout.                  */
    lv_obj_t  *unitLabel  = nullptr;  /**< Unit caption.                     */
    lv_obj_t  *nameLabel  = nullptr;  /**< Metric caption.                   */
    lv_obj_t  *peakLabel  = nullptr;  /**< Peak readout; hidden by default.  */
    lv_obj_t  *bar        = nullptr;  /**< Horizontal bar indicator.         */
    /** @} */

    /* chart screens */
    lv_obj_t          *chart       = nullptr;
    lv_chart_series_t *series      = nullptr;
    uint32_t           chartPeriod = 1000;  /**< ms between samples          */
    uint32_t           lastChartMs = 0;

    lv_color_t   normalColor = {}; /**< Color when not in alarm.             */
    ThresholdCfg warn;             /**< Cosmetic warning tier.               */
    ThresholdCfg crit;             /**< Cosmetic critical tier.              */
    AlarmState   state    = AlarmState::Normal;  /**< Current tier.          */
    bool         pulsing  = false; /**< A pulse animation is running.        */
    float        lastValue = 0;    /**< Last rendered value.                 */

    /* Render guards: skip widget writes (and their invalidation/redraw cost)
     * when the quantized value did not change since last tick. */
    int32_t lastArcVal    = INT32_MIN;  /**< Last arc position written.     */
    int32_t lastNeedleVal = INT32_MIN;  /**< Last needle value written.     */
    int32_t lastBarVal    = INT32_MIN;  /**< Last bar position written.     */
    char    lastText[24]  = {0};        /**< Last readout string written.   */
    char    lastPeakText[24] = {0};     /**< Last peak string written.      */
};

/**
 * @brief One entry of the watchdog list — a metric to police regardless of
 *        whether any screen displays it.
 */
struct WatchItem {
    uint16_t     metricId = 0;   /**< Metric to police — @ref MetricIDs.     */
    String       label;          /**< Name shown on the alert card.          */
    String       units;          /**< Unit suffix on the alert card.         */
    uint8_t      decimals = 0;   /**< Decimal places on the alert card.      */
    ThresholdCfg warn;           /**< Amber tier; steady border.             */
    ThresholdCfg crit;           /**< Red tier; pulsing border.              */

    /**
     * @brief Whether tripping this item raises the alert card.
     *
     * Defaults to true. Set `"popup": false` on the item to police it
     * silently — useful when the metric already has a gauge whose own red
     * zone makes the fault obvious, so a card over the top would just be
     * covering the reading the driver is looking at.
     *
     * A silent item is still evaluated and still counts toward the "+N more"
     * badge; it simply never becomes the card's subject.
     */
    bool         popup     = true;

    AlarmState   state     = AlarmState::Normal;  /**< Current tier.         */
    float        lastValue = 0;  /**< Latest raw value, shown on the card.   */
    bool         usable    = false;  /**< Data is valid and fresh.           */
};

/** @brief One swipeable screen built from layout.json. */
struct ScreenDef {
    lv_obj_t *scr      = nullptr;  /**< LVGL screen object.                  */
    lv_obj_t *root     = nullptr;  /**< circular clipping container          */
    lv_obj_t *linkIcon = nullptr;  /**< "no link" indicator                  */
    bool      isDiag   = false;    /**< This is the diagnostics screen.      */
    /**
     * @brief Whether the alert card may be drawn while this screen is up.
     *
     * Set `"show_popup": false` on a screen to keep it clean. Useful for a
     * dial you want unobstructed: the metric is still policed and still
     * colours itself, but nothing is drawn on top while you are looking at
     * that screen. Swiping to another screen shows the alert again.
     */
    bool      allowPopup = true;
    /**
     * @brief Whether the warning-lamp strip may be drawn over this screen.
     *
     * `"show_telltales": false` on a screen keeps it clear. The dash screen
     * never shows the strip: it has its own lamp row.
     */
    bool      showTelltales = true;
    int       dash     = -1;       /**< Index into the dash list, or -1.     */
    std::vector<GaugeBinding> bindings;  /**< Metrics rendered here.         */
};

/**
 * @brief What the alert card is showing: a watchdog item or a warning lamp.
 *
 * Both paths feed the one card, so the driver sees one thing at a time, the
 * most urgent first, with "+N more" for the rest.
 */
struct AlertView {
    uint32_t    key    = 0;   /**< Metric ID, or ALERT_LAMP_KEY | lamp.       */
    AlarmState  level  = AlarmState::Normal;  /**< Severity.                  */
    lv_color_t  accent = {};  /**< Border and symbol colour.                  */
    const lv_img_dsc_t *img = nullptr;  /**< Lamp symbol; null = triangle.    */
    const char *name   = "";  /**< Metric name or lamp title.                 */
    const char *value  = "";  /**< Value and units; may be empty.             */
    const char *detail = "";  /**< Why, or what to do.                        */
};

/** Keys at or above this are lamps (low byte = lamp index), not metric IDs. */
static constexpr uint32_t ALERT_LAMP_KEY = 0x10000;

/** @brief Slots in the dash screen's lamp row. */
static constexpr size_t DASH_LAMP_SLOTS = 5;    // lit lamps, in the header
/** @brief Slots on the lamp strip overlay. */
static constexpr size_t STRIP_SLOTS     = 10;

/**
 * @brief The dash screen's own widgets beyond its gauge bindings: the lamp
 *        row, the shift light and the symbols beside the fuel and temperature
 *        readouts. Each also carries a change guard, so an unchanged lamp
 *        costs nothing per frame.
 */
struct DashView {
    size_t    screen = 0;                          /**< Screen it lives on.   */
    bool      ghosts = true;                       /**< Dim unlit lamps shown.*/
    lv_obj_t *slot[DASH_LAMP_SLOTS] = {};          /**< Lamp row images.      */
    uint8_t   slotIcon[DASH_LAMP_SLOTS] = {};      /**< Icon in each slot.    */
    uint32_t  slotRgb[DASH_LAMP_SLOTS]  = {};      /**< Last colour written.  */
    size_t    slots = 0;                           /**< Slots in use.         */
    lv_obj_t *turn[2] = {};                        /**< Left, right arrows.   */
    uint32_t  turnRgb[2] = {};

    bool      shift = false;                       /**< Shift light enabled.  */
    uint16_t  rpmMetric = 0;                       /**< Its source.           */
    float     shiftRpm = 6500, shiftStart = 5000;  /**< Full, first LED [rpm].*/
    lv_obj_t *led[5] = {};                         /**< The five LEDs.        */
    uint32_t  ledRgb[5] = {};
    lv_obj_t *gearBox = nullptr;                   /**< Flashes at the shift. */
    uint32_t  gearRgb = 0;

    lv_obj_t *fuelIcon = nullptr, *tempIcon = nullptr;  /**< Fuel, coolant.   */
    uint32_t  fuelRgb = 0, tempRgb = 0;
};

/**
 * @brief Builds the screens and drives them from the telemetry store.
 *
 * Singleton — see @ref UI.
 */
class UIBuilder {
public:
    /**
     * @brief Build every screen from the layout and start the UI timer.
     * @pre ConfigManager::begin has run, so the layout is parsed.
     */
    void begin();

    /** @brief Boot "needle sweep": min → max → min on the visible screen. */
    void startupSweep();

    /** @name Gesture entry points
     *  Wired to TouchManager and ButtonManager in main.cpp.
     *  @{ */
    /** @brief Advance one screen (swipe left / button tap). */
    void nextScreen();
    /** @brief Go back one screen (swipe right). */
    void prevScreen();
    /** @brief Show or hide the peak readouts (tap, when no alert is up). */
    void togglePeaks();
    /** @brief Swipe up: day/night override — AUTO → DAY → NIGHT → AUTO. */
    void cycleNightOverride();
    /** @brief Swipe down: zero all peak values, with toast feedback. */
    void resetPeaksWithToast();
    /** @} */

    /**
     * @brief Dismiss the alert card for a configurable cool-off period.
     * @return true if an alert was showing and the tap was consumed by it.
     *
     * Lets a single tap mean "I have seen it" while an alarm is up, and
     * "toggle peaks" the rest of the time.
     */
    bool acknowledgeAlert();

    /**
     * @brief Transient pill notification at the top of the screen (~1.3 s).
     * @param text Message to show; copied into the label immediately.
     */
    void showToast(const char *text);

    /**
     * @brief Run the on-device touch calibration wizard.
     *
     * Asks for one swipe in each direction and measures what the controller
     * actually reports, rather than assuming how the panel is bonded. See the
     * file comment in TouchManager.h for why measuring is the only reliable
     * option here.
     */
    void startTouchCalibration();

    /**
     * @brief Feed a captured raw drag into the wizard.
     * @param dxRaw Raw x travel of the completed drag.
     * @param dyRaw Raw y travel of the completed drag.
     */
    void onCalibrationDrag(int16_t dxRaw, int16_t dyRaw);

    /**
     * @brief Show a colour test pattern so calibration can be seen.
     *
     * While the configuration portal is open the panel shows the AP details —
     * precisely what you cannot see past when adjusting colour from the web
     * UI. This replaces it with swatches and a grey ramp for a few seconds
     * each time a calibration arrives, then restores the AP screen.
     */
    void showColorPreview();

    /**
     * @brief Replace the UI with the config-mode info screen.
     * @param ssid AP name to display.
     * @param ip   Portal address to display.
     */
    void showConfigScreen(const char *ssid, const char *ip);

private:
    /** @brief Day/night source selection. */
    enum class NightOverride : uint8_t {
        Auto,       /**< Follow the master's night flag. */
        ForceDay,   /**< Always day.                     */
        ForceNight  /**< Always night.                   */
    };

    /** @name Construction
     *  Everything here runs once at boot; nothing allocates afterwards.
     *  @{ */
    /**
     * @brief Build one screen and append it to the screen list.
     * @param cfg One element of the layout's `screens` array.
     */
    void buildScreen(JsonObject cfg);

    /**
     * @brief Draw the static instrument bezel — two rings and twelve ticks.
     * @param root Screen's circular container.
     * @note Created once and never touched again, so it costs nothing per
     *       frame.
     */
    void addDialFace(lv_obj_t *root);

    /** @brief Add page-position dots along the bottom rim of every screen. */
    void addPageDots();

    /**
     * @brief Create a screen's full-panel container.
     * @param sd  Screen being built; its `root` is set on return.
     * @param cfg Screen configuration.
     * @return The new container.
     */
    lv_obj_t *makeRoot(ScreenDef &sd, JsonObject cfg);

    /**
     * @brief Paint a screen's background, cheapest mechanism first.
     * @param root Screen's circular container.
     * @param cfg  Screen configuration; reads `bg_grad`, `texture` and
     *             `background_asset`.
     */
    void applyBackground(lv_obj_t *root, JsonObject cfg);

    /** @brief Build a large arc gauge with a numeric readout.
     *  @param sd Screen being built. @param cfg Screen configuration. */
    void buildArcGauge(ScreenDef &sd, JsonObject cfg);
    /** @brief Build a needle tachometer with ticks and colored zones.
     *  @param sd Screen being built. @param cfg Screen configuration. */
    void buildMeterGauge(ScreenDef &sd, JsonObject cfg);
    /** @brief Build a 2- or 4-cell split screen.
     *  @param sd Screen being built. @param cfg Screen configuration. */
    void buildQuad(ScreenDef &sd, JsonObject cfg);
    /** @brief Build a compact label/value list, one or two columns.
     *  @param sd Screen being built. @param cfg Screen configuration. */
    void buildTextList(ScreenDef &sd, JsonObject cfg);
    /** @brief Build stacked horizontal bar gauges.
     *  @param sd Screen being built. @param cfg Screen configuration. */
    void buildBars(ScreenDef &sd, JsonObject cfg);
    /**
     * @brief Build the landscape hero screen: dial, main readout, side stack.
     * @param sd  Screen being built.
     * @param cfg Screen configuration, reading `dial`, `main` and `side[]`.
     */
    void buildCluster(ScreenDef &sd, JsonObject cfg);
    /** @brief Build a rolling strip chart of one metric.
     *  @param sd Screen being built. @param cfg Screen configuration. */
    void buildChart(ScreenDef &sd, JsonObject cfg);
    /** @brief Build the link-diagnostics screen, including the GPIO scan.
     *  @param sd Screen being built. */
    void buildDiag(ScreenDef &sd);
    /**
     * @brief Build the mini instrument cluster.
     *
     * Every part is optional and can be switched off with `false`:
     * `tach`, `speed`, `gear`, `fuel`, `temp`, `info[]`, `shift_light`,
     * `lamps`. Parts given as objects override the defaults (metric_id,
     * label, units, decimals, min, max, thresholds).
     * @param sd Screen being built. @param cfg Screen configuration.
     */
    void buildDash(ScreenDef &sd, JsonObject cfg);
    /**
     * @brief Fill one dash binding from its part of the configuration.
     * @param b     Binding to fill.
     * @param v     The part: absent or true = defaults, object = overrides.
     * @param id    Default metric.
     * @param label Default caption.
     * @param units Default units.
     * @param minV  Default scale minimum.
     * @param maxV  Default scale maximum.
     * @param dec   Default decimals.
     */
    void dashPart(GaugeBinding &b, JsonVariant v, uint16_t id,
                  const char *label, const char *units, float minV,
                  float maxV, uint8_t dec);
    /** @brief Create the (hidden) lamp strip on lv_layer_top. */
    void buildTelltaleStrip();

    /** @brief Parse the layout's `watchdog` block into the guard list. */
    void parseWatchdog();

    /**
     * @brief Parse the fields every metric-bound widget shares.
     * @param[out] b Binding to populate.
     * @param cfg    Object carrying metric_id, label, units, min, max…
     */
    void parseCommon(GaugeBinding &b, JsonObject cfg);

    /**
     * @brief Parse one alarm tier.
     * @param[out] t  Threshold to populate; left disabled if @p cfg is null.
     * @param cfg     The `warning` or `critical` object.
     * @param critical true for the critical tier, which also reads
     *                 `overlay_text` and defaults to red.
     */
    void parseThreshold(ThresholdCfg &t, JsonObject cfg, bool critical);
    /** @} */

    /** @name Runtime
     *  Driven by an LVGL timer at @ref UI_UPDATE_PERIOD_MS.
     *  @{ */
    /**
     * @brief LVGL timer trampoline; forwards to @ref tick.
     * @param t Timer whose `user_data` is the UIBuilder.
     */
    static void uiTimerCb(lv_timer_t *t);

    /** @brief One UI update: sample, render, evaluate alarms. */
    void tick();

    /**
     * @brief Push one metric's value into its widgets.
     * @param b      Binding to render.
     * @param value  Smoothed value to show.
     * @param usable false when data is missing or stale — renders "--".
     * @param peak   Peak value, shown when peaks are enabled.
     */
    void renderBinding(GaugeBinding &b, float value, bool usable, float peak);

    /**
     * @brief Move a binding's arc and/or needle, skipping no-op writes.
     * @param b Binding to move.
     * @param v Value, clamped to the binding's scale.
     */
    void setGaugeValue(GaugeBinding &b, float v);

    /**
     * @brief Update one binding's cosmetic alarm tier, with hysteresis.
     * @param b      Binding to evaluate.
     * @param raw    Raw metric value.
     * @param flags  METRIC_FLAG_* bits; these escalate but never suppress.
     * @param usable false when data is missing or stale.
     */
    void evalAlarm(GaugeBinding &b, float raw, uint8_t flags, bool usable);

    /** @brief Apply colors and pulsing after a tier change. @param b Binding. */
    void onAlarmChanged(GaugeBinding &b);
    /** @brief Start the warning opacity pulse. @param b Binding. */
    void startPulse(GaugeBinding &b);
    /** @brief Stop the pulse and restore full opacity. @param b Binding. */
    void stopPulse(GaugeBinding &b);
    /** @brief Recolor a binding's widgets. @param b Binding. @param c Color. */
    void setBindingColor(GaugeBinding &b, lv_color_t c);

    /** @brief Refresh the diagnostics readouts and the live GPIO scan. */
    void updateDiag();

    /**
     * @brief Sample a metric at most once per tick.
     *
     * TelemetryStore::sample advances the metric's glide every call, so a
     * metric bound twice (a tach on the dash and again on a quad) used to
     * glide at double speed. Every binding now shares one sample per tick.
     * @param id        Metric.
     * @param lerp      Glide factor.
     * @param[out] s    The sample.
     * @return false when the metric has never been received.
     */
    bool sampleOnce(uint16_t id, float lerp, MetricSample &s);

    /** @return The reverse-gear switch is on the bus and set. */
    bool reverseEngaged();

    /**
     * @brief Write a binding's value as text (numbers, or a gear letter).
     * @param b        Binding.
     * @param v        Value.
     * @param[out] out Destination.
     * @param n        Size of @p out.
     */
    static void formatBinding(const GaugeBinding &b, float v, char *out,
                              size_t n);

    /** @brief Redraw the lamp strip from the lamps' state. */
    void updateTelltaleStrip();

    /**
     * @brief Lamps, shift light and fuel/temperature symbols of a visible dash.
     * @param dv   The dash.
     * @param lerp Glide factor, for the shared per-tick sample.
     * @param now  millis().
     */
    void updateDash(DashView &dv, float lerp, uint32_t now);

    /**
     * @brief Hide everything drawn on lv_layer_top by the gauges: the alert
     *        card and the lamp strip. For screens that are not gauges (config
     *        portal, touch calibration), which the overlays must not cover.
     */
    void hideOverlays();

    /**
     * @brief Switch the theme, backgrounds and backlight.
     * @param night true for the night palette and dimmed backlight.
     */
    void applyNight(bool night);

    /**
     * @brief Animate to another screen.
     * @param idx     Index into the screen list.
     * @param forward true to slide as if moving forward through the list.
     */
    void loadScreen(size_t idx, bool forward);
    /** @} */

    /** @name Watchdog and alert card
     *  The safety guard — see the class-level note on the two alarm paths.
     *  @{ */
    /** @brief Create the (hidden) alert card on lv_layer_top. */
    void buildAlertCard();

    /**
     * @brief Evaluate every watched metric against its own thresholds.
     * @note Uses the RAW value via TelemetryStore::peek, never the smoothed
     *       one: a guard must react to what the sensor reported, not to a
     *       display filter that lags it.
     */
    void evalWatchdog();

    /** @brief Run the watchdog and show, update or hide the alert card. */
    void manageAlert();

    /**
     * @brief Populate and reveal the alert card.
     * @param v             What to show: a watchdog item or a lamp.
     * @param othersActive  Count of *other* active faults, for the "+N more"
     *                      badge; 0 hides it.
     */
    void showAlert(const AlertView &v, uint8_t othersActive);

    /**
     * @brief Arrange the card for a watchdog item or for a lamp.
     *
     * A watchdog item is a number against a limit; a lamp is a symbol and a
     * sentence. They read best laid out differently.
     * @param lamp true for the lamp arrangement.
     */
    void applyAlertLayout(bool lamp);

    /** @brief Hide the alert card and stop its pulse. */
    void hideAlert();
    /** @} */

    /** @name Boot sweep
     *  @{ */
    /**
     * @brief Animation callback driving every gauge through its full scale.
     * @param var Cast to UIBuilder*.
     * @param v   Animation position, 0…1000.
     */
    static void sweepExecCb(void *var, int32_t v);

    /**
     * @brief Sweep completion callback; hands control back to @ref tick.
     * @param a Animation whose `var` is the UIBuilder.
     */
    static void sweepReadyCb(lv_anim_t *a);
    /** @} */

    std::vector<ScreenDef> _screens;    /**< Every built screen, in order.   */
    std::vector<WatchItem> _watch;      /**< the guard list — see class doc  */
    Telltales              _lamps;      /**< Warning lamps — see Telltales.h */
    std::vector<DashView>  _dash;       /**< Every dash screen's extras.     */

    /** @brief One metric's sample, shared by every binding this tick. */
    struct TickSample {
        uint16_t     id;   /**< Metric.                         */
        bool         ok;   /**< The store had it.               */
        MetricSample s;    /**< The sample.                     */
    };
    std::vector<TickSample> _tickCache;  /**< Cleared at the start of a tick. */
    bool       _watchEnabled = true;    /**< Watchdog armed.                 */
    /** Master switch for the on-screen alert card. Independent of the gauge
     *  colours: this is the thing drawn OVER the screen, and some drivers
     *  would rather have the fault noted without anything covering the dial. */
    bool       _watchPopups  = true;
    /**
     * Alert presentation. On a landscape panel a full card in the middle of
     * the screen covers the very gauges the driver is trying to read; a strip
     * across the top says the same thing without occluding anything. The card
     * remains available for anyone who prefers it.
     */
    bool       _alertAsBar   = true;
    /** Per-gauge threshold recoloring/pulsing enabled. When false every gauge
     *  stays its normal color; the watchdog still works independently. */
    bool       _gaugeAlarms  = true;
    size_t     _active       = 0;       /**< Index of the visible screen.    */
    int        _diagIndex    = -1;      /**< Diagnostics screen, -1 if none. */
    bool       _sweepActive  = false;   /**< Boot sweep is running.          */
    bool       _configActive = false;   /**< Config screen has taken over.   */
    bool       _peaksShown   = false;   /**< Peak readouts are visible.      */
    bool       _night        = false;   /**< Night palette is applied.       */
    NightOverride _nightOverride = NightOverride::Auto;  /**< Day/night src. */
    lv_timer_t *_timer       = nullptr; /**< Drives @ref tick.               */
    /** Gauge writes are suspended until this time so screen-transition
     *  animations get the whole frame budget (see loadScreen). */
    uint32_t   _busySinceMs  = 0;
    /** Length of the suspension, 0 when none. Kept as start + length, not
     *  an end time: comparing against an end time breaks when millis()
     *  wraps (49.7 days), and a signed difference breaks after 24.8 days. */
    uint32_t   _busyMs       = 0;

    /** @name Toast
     *  @{ */
    lv_obj_t   *_toast      = nullptr;  /**< Pill container, or nullptr.     */
    lv_timer_t *_toastTimer = nullptr;  /**< One-shot dismissal timer.       */
    /**
     * @brief Toast expiry callback — deletes the pill.
     * @param t Timer whose `user_data` is the UIBuilder.
     */
    static void toastTimerCb(lv_timer_t *t);
    /** @} */

    /** @name Alert card
     *  Lives on lv_layer_top, built once at boot, then shown and hidden.
     *  @{ */
    lv_obj_t  *_alertCard   = nullptr;  /**< Card container.                 */
    lv_obj_t  *_alertIcon   = nullptr;  /**< Warning triangle glyph.         */
    lv_obj_t  *_alertImg    = nullptr;  /**< Lamp symbol, in lamp layout.    */
    bool       _alertLampLayout = false; /**< Card is arranged for a lamp.   */
    /** Bar layout: the advice has moved into the empty value column. */
    bool       _alertDetailWide = false;
    lv_obj_t  *_alertName   = nullptr;  /**< Metric name.                    */
    lv_obj_t  *_alertValue  = nullptr;  /**< Value and units.                */
    lv_obj_t  *_alertDetail = nullptr;  /**< "HIGH · LIMIT …" or headline.   */
    lv_obj_t  *_alertMore   = nullptr;  /**< "+N more" badge.                */
    bool       _alertShown  = false;    /**< Card is visible.                */
    bool       _alertPulsing = false;   /**< Border pulse is running.        */
    AlarmState _alertLevel  = AlarmState::Normal;  /**< Tier on the card.    */
    uint32_t   _alertKey    = 0;        /**< AlertView::key on the card.     */
    /* Change guards. Sized past the longest text that can reach them: a guard
     * shorter than its text compares unequal every frame and rewrites it. */
    char       _alertLastName[40]   = {0};  /**< Change guard for the name.  */
    char       _alertLastValue[40]  = {0};  /**< Change guard for the value. */
    char       _alertLastDetail[56] = {0};  /**< Change guard for the detail.*/
    /** @} */

    /** @name Acknowledgement
     *  Silences exactly one metric/level pair until @ref _ackUntilMs, so an
     *  escalation or a second fault still gets through. Lamps keep their own
     *  acknowledgement (Telltales::acknowledge).
     *  @{ */
    bool       _ackActive   = false;    /**< A watchdog item is muted.       */
    uint32_t   _ackUntilMs  = 0;        /**< millis() when muting expires.   */
    uint16_t   _ackMetric   = 0;        /**< Which metric was acknowledged.  */
    AlarmState _ackLevel    = AlarmState::Normal;  /**< At which severity.   */
    /** @} */

    /** @name Lamp strip
     *  Lives on lv_layer_top under the alert card; shows the lit lamps over
     *  every screen that allows it.
     *  @{ */
    lv_obj_t  *_strip = nullptr;                  /**< The pill.             */
    lv_obj_t  *_stripImg[STRIP_SLOTS] = {};       /**< Its lamp images.      */
    uint8_t    _stripIcon[STRIP_SLOTS] = {};      /**< Change guards.        */
    uint32_t   _stripRgb[STRIP_SLOTS]  = {};
    lv_obj_t  *_stripMore = nullptr;              /**< "+N" when too many.   */
    size_t     _stripCount = 0;                   /**< Slots shown.          */
    int        _stripMoreN = -1;                  /**< "+N" shown, -1 none.  */
    bool       _stripShown = false;               /**< Pill visible.         */
    /** @} */

    /** @name Diagnostics screen
     *  @{ */
    /** @name Touch calibration wizard
     *  @{ */
    lv_obj_t   *_calScreen  = nullptr;  /**< Wizard screen.                 */
    lv_obj_t   *_calTitle   = nullptr;  /**< Instruction line.              */
    lv_obj_t   *_calHint    = nullptr;  /**< Sub-instruction.               */
    lv_obj_t   *_calArrow   = nullptr;  /**< Direction glyph.               */
    uint8_t     _calStep    = 0;        /**< 0 = horizontal, 1 = vertical.  */
    bool        _calActive  = false;    /**< Wizard has the display.        */
    uint32_t    _calSinceMs = 0;        /**< When it took it.               */
    /** Unanswered this long, the wizard gives the gauges back. */
    static constexpr uint32_t CAL_TIMEOUT_MS = 60000;
    /** @brief Leave the wizard without a new mapping (BOOT button, timeout). */
    void cancelTouchCalibration();
    /** @brief The screen after the wizard: the portal's while it runs (the
     *  wizard can be started from the studio), else the gauges. */
    void leaveCalibration();
    int16_t     _calHdx = 0, _calHdy = 0;  /**< Measured left->right drag.  */
    /** @} */

    /** @name Colour preview
     *  @{ */
    lv_obj_t   *_previewScreen = nullptr;  /**< Colour test pattern.        */
    lv_obj_t   *_configScreen  = nullptr;  /**< The portal's, built once.   */
    lv_timer_t *_previewTimer  = nullptr;  /**< Returns to the AP screen.   */
    String      _cfgSsid, _cfgIp;          /**< Remembered for the restore. */
    /** @brief Preview expiry — restores the AP screen. */
    static void previewTimerCb(lv_timer_t *t);
    /** @} */

    /** @brief Rows: Link, Rate, Drops, Seq, Heap, PSRAM, Uptime, GPIO.
     *  Two more than the round board's six — landscape has the room, and
     *  PSRAM presence is worth seeing on a board that depends on it. */
    static constexpr int DIAG_ROWS = 8;
    lv_obj_t *_diagVals[DIAG_ROWS]     = {};  /**< Value labels.             */
    char      _diagLast[DIAG_ROWS][24] = {};  /**< Change guards.            */
    /** Live touch readout: mapped coordinates and the last gesture seen. */
    lv_obj_t *_diagTouch      = nullptr;
    char      _diagTouchLast[32] = {};
    /** @} */
};

/** @brief Global UI singleton. */
extern UIBuilder UI;

/** @} */  // end of ui group
