/**
 * @file ButtonManager.h
 * @ingroup hal
 * @brief Debounced handler for the board's side button, plus a polarity-
 *        agnostic discovery scanner for finding which GPIO it actually is.
 *
 * The BOOT button is GPIO9 on most ESP32-C3 boards (it is the chip's download
 * strap, which only matters during reset, so it is free as a runtime input).
 * Clones vary, and some round boards have no second button on a GPIO at all —
 * their other button is wired to EN/reset, which no firmware can read.
 *
 * Rather than keep guessing, @ref poll runs a discovery scan: it records each
 * candidate pin's resting level shortly after boot and then reports any pin
 * that DEVIATES from it. Watching for change rather than for a low level is
 * what makes this work on either polarity — an active-high button on a pin
 * that already rests high can never be seen "going low", which is precisely
 * how such a button stays invisible to a naive scan.
 *
 * Actions (wired in main.cpp):
 *   short press — next screen (glove-friendly alternative to swiping)
 *   long press  — enter config AP / reboot (same as the 5 s touch hold)
 */
#pragma once

#include <Arduino.h>

#include "HardwareConfig.h"

/** @brief Button event callback (no arguments — actions are global). */
typedef void (*ButtonCallback)();

/** @brief Discovery callback, invoked once per pin that shows activity. */
typedef void (*ButtonDiscoveryCallback)(int gpio);

/**
 * @brief Debounces one active-low input and classifies presses.
 *
 * Singleton — see @ref Button.
 */
class ButtonManager {
public:
    /** @brief Handlers invoked when a press is classified. */
    struct Callbacks {
        ButtonCallback onShortPress = nullptr;  /**< Fired on release.     */
        ButtonCallback onLongPress  = nullptr;  /**< Fired once per press. */
        /** Fired the first time a candidate pin deviates from its resting
         *  level. Use it to surface the pin to the user. */
        ButtonDiscoveryCallback onDiscovery = nullptr;
    };

    /**
     * @brief Configure the input and every discovery candidate.
     * @param pin       Active-low input to watch (layout `global.button_gpio`,
     *                  defaulting to @ref PIN_BUTTON). Negative disables it.
     * @param discovery true to run the discovery scan (layout
     *                  `global.button_discovery`). Leave on until the pin is
     *                  known; it costs a handful of digitalReads per loop.
     * @param cbs       Event handlers; any member may be nullptr.
     */
    void begin(int pin, bool discovery, const Callbacks &cbs);

    /**
     * @brief Debounce, classify, and run discovery. Call every loop().
     */
    void poll();

    /**
     * @brief Render scan state for the diagnostics screen.
     *
     * Before any activity: the live levels, e.g. `9H 8H 0H 20H 21H`.
     * After activity:      the answer, e.g. `PRESS = GPIO 0`.
     *
     * @param[out] out Destination buffer.
     * @param n        Buffer size.
     */
    void scanState(char *out, size_t n);

    /**
     * @brief The pin discovery has seen change, if any.
     * @return GPIO number, or -1 when nothing has deviated yet.
     */
    int discovered() const { return _discovered; }

    /**
     * @brief The GPIO currently being watched for presses.
     * @return Pin number, or -1 when the button is disabled.
     */
    int pin() const { return _pin; }

private:
    /** @brief Per-candidate discovery state. */
    struct Candidate {
        int     gpio      = -1;   /**< Pin number.                          */
        bool    resting   = true; /**< Level sampled once it settled.       */
        bool    baselined = false;/**< Resting level has been captured.     */
        bool    reported  = false;/**< Deviation already announced.         */
        uint8_t streak    = 0;    /**< Consecutive deviating samples.       */
    };

public:
    /** @brief Upper bound on @ref BUTTON_SCAN_PINS entries. */
    static constexpr size_t SCAN_MAX = 12;

private:
    Candidate _cand[SCAN_MAX];         /**< Discovery state per candidate.  */
    size_t    _candCount     = 0;      /**< Populated entries in _cand.     */

    Callbacks _cbs;                    /**< Registered handlers.            */
    int       _pin           = -1;     /**< Watched GPIO, -1 = disabled.    */
    bool      _discovery     = true;   /**< Discovery scan enabled.         */
    int       _discovered    = -1;     /**< Pin seen deviating, or -1.      */

    bool      _stablePressed = false;  /**< Debounced press state.          */
    bool      _lastRaw       = false;  /**< Last raw level, for edge detect.*/
    uint32_t  _lastEdgeMs    = 0;      /**< millis() of the last raw edge.  */
    uint32_t  _pressT0       = 0;      /**< millis() the press stabilized.  */
    bool      _longFired     = false;  /**< Long press already reported.    */
    uint32_t  _startedMs     = 0;      /**< millis() at begin(), for settle.*/
    uint32_t  _lastTraceMs   = 0;      /**< Last serial pin-state trace.    */
};

/** @brief Global button singleton. */
extern ButtonManager Button;

/** @} */  // end of hal group
