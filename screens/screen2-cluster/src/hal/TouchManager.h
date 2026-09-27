/**
 * @file TouchManager.h
 * @ingroup hal
 * @brief GT911 driver and gesture engine.
 *
 * ## Why this was rebuilt
 *
 * The previous engine decided direction by comparing horizontal travel
 * against vertical travel in mapped screen pixels, on the assumption that the
 * mapping was right. Three separate attempts to fix swipes by adjusting that
 * mapping — swapping axes, rescaling them, lowering thresholds — each fixed
 * one direction and left the other dead. The assumption itself was the bug.
 *
 * The trouble is that the panel's bonding orientation, the controller's
 * coordinate range and the sign of each axis are all unknown, and getting any
 * one of them wrong silently biases the comparison so that one axis wins
 * every time. No amount of threshold tuning can recover from that, because
 * the two numbers being compared are not in the same units.
 *
 * ## What replaces it
 *
 * Nothing is assumed. The engine works in the controller's own raw
 * coordinates and holds two **measured** direction vectors:
 *
 *   - @ref TouchCal::hx, @ref TouchCal::hy — which way raw coordinates move
 *     when a finger travels left→right across the glass
 *   - @ref TouchCal::vx, @ref TouchCal::vy — the same for top→bottom
 *
 * A drag is projected onto both, and each projection is divided by the length
 * of a full-screen swipe along that axis. That yields two dimensionless
 * fractions — "how far across the screen" and "how far down the screen" — which
 * are finally directly comparable no matter how the panel is mounted, how the
 * controller numbers its axes, or which direction counts as positive. Mirrored,
 * rotated and even slightly skewed panels all fall out of the same maths.
 *
 * Those vectors come from @ref beginCalibration: the user is asked to swipe
 * once each way and the engine measures what actually happened. It is the only
 * way to know, and it takes five seconds.
 *
 * Until calibrated, sensible defaults are used, and thresholds are fractions
 * of a screen traversal rather than pixel counts — so they stay meaningful
 * even when the scale is wrong.
 *
 * @note Pointer coordinates handed to LVGL still go through the simple
 *       swap/scale path, which has always worked for taps. Gesture direction
 *       is the part that needed measuring, so that is the only part changed.
 */
#pragma once

#include <Arduino.h>
#include <lvgl.h>

#include "HardwareConfig.h"

/**
 * @brief Measured description of how raw touch coordinates relate to the
 *        screen. See the file comment for why this is measured, not assumed.
 */
struct TouchCal {
    bool  valid = false;  /**< False until a calibration has been captured.  */
    float hx = 1.0f;      /**< Raw-space direction of screen-RIGHT, x part.  */
    float hy = 0.0f;      /**< Raw-space direction of screen-RIGHT, y part.  */
    float vx = 0.0f;      /**< Raw-space direction of screen-DOWN, x part.   */
    float vy = 1.0f;      /**< Raw-space direction of screen-DOWN, y part.   */
    float hLen = 400.0f;  /**< Raw units spanned by a full-width swipe.      */
    float vLen = 240.0f;  /**< Raw units spanned by a full-height swipe.     */
};

/** @brief Gesture callback (no arguments — gestures are global UI events). */
typedef void (*GestureCallback)();

/**
 * @brief Reports a completed drag in RAW controller units, for calibration.
 * @param dx Raw x travel.
 * @param dy Raw y travel.
 */
typedef void (*RawDragCallback)(int16_t dx, int16_t dy);

/**
 * @brief Polls the GT911 and turns raw frames into direction-correct gestures.
 *
 * Singleton — see @ref Touch.
 */
class TouchManager {
public:
    /** @brief Handlers invoked when a gesture is recognized. */
    struct Callbacks {
        GestureCallback onSwipeLeft  = nullptr; /**< Toward screen-left.    */
        GestureCallback onSwipeRight = nullptr; /**< Toward screen-right.   */
        GestureCallback onSwipeUp    = nullptr; /**< Toward screen-top.     */
        GestureCallback onSwipeDown  = nullptr; /**< Toward screen-bottom.  */
        GestureCallback onTap        = nullptr; /**< Short press, no travel.*/
        GestureCallback onHold       = nullptr; /**< Fired once per press.  */
        RawDragCallback onRawDrag    = nullptr; /**< Calibration mode only. */
    };

    /**
     * @brief Bring up I2C, reset the controller, register with LVGL.
     * @param cbs     Gesture handlers; any member may be nullptr.
     * @param cal     Measured mapping; defaults are used when not valid.
     * @param swapXY  Pointer-mapping only — see the note in the file comment.
     * @param invertX Pointer-mapping only.
     * @param invertY Pointer-mapping only.
     * @param nativeW Controller X range, for pointer mapping.
     * @param nativeH Controller Y range, for pointer mapping.
     */
    void begin(const Callbacks &cbs, const TouchCal &cal,
               bool swapXY = false, bool invertX = false, bool invertY = false,
               int nativeW = TOUCH_NATIVE_W, int nativeH = TOUCH_NATIVE_H);

    /** @brief Replace the measured mapping and use it immediately. */
    void setCalibration(const TouchCal &cal) { _cal = cal; }
    /** @return The mapping currently in use. */
    const TouchCal &calibration() const { return _cal; }

    /**
     * @brief Enter calibration capture.
     *
     * While active, gestures are NOT classified or dispatched. Every completed
     * drag is reported raw through Callbacks::onRawDrag so the caller can
     * measure what a deliberate swipe looks like in controller coordinates.
     */
    void beginCalibration() { _calMode = true; }

    /** @brief Leave calibration capture and resume normal gestures. */
    void endCalibration() { _calMode = false; }

    /** @return true while calibration capture is active. */
    bool calibrating() const { return _calMode; }

    /** @name Diagnostics
     *  @{ */
    int16_t rawX() const { return _mapX; }      /**< Mapped X (screen px).   */
    int16_t rawY() const { return _mapY; }      /**< Mapped Y (screen px).   */
    int16_t ctrlX() const { return _rawX; }     /**< Controller-frame X.     */
    int16_t ctrlY() const { return _rawY; }     /**< Controller-frame Y.     */
    int16_t seenMaxX() const { return _seenMaxX; } /**< Largest raw X seen.  */
    int16_t seenMaxY() const { return _seenMaxY; } /**< Largest raw Y seen.  */
    /** @return Name of the last gesture recognised, or "-" if none yet. */
    const char *lastGesture() const { return _lastGesture; }
    /** @} */

private:
    /** @brief Where a contact is in its lifecycle. */
    enum class Phase : uint8_t {
        Idle,       /**< Nothing on the glass.                            */
        Candidate,  /**< Contact seen, not yet held long enough to trust. */
        Pressed     /**< Confirmed press; gestures are being tracked.     */
    };

    static void readCb(lv_indev_drv_t *drv, lv_indev_data_t *data);
    void handleRead(lv_indev_data_t *data);

    /**
     * @brief Read one raw frame.
     * @param[out] x       Controller-frame X.
     * @param[out] y       Controller-frame Y.
     * @param[out] touched True when this frame reports a contact.
     * @param[out] fresh   False means "no news"; hold previous state.
     * @return false on an I2C error.
     */
    bool readPanel(int16_t &x, int16_t &y, bool &touched, bool &fresh);

    /**
     * @brief Classify a raw drag and fire the matching callback.
     *
     * Projects onto the measured axes and compares normalised fractions of a
     * screen traversal, which is what makes the decision independent of
     * mounting orientation and coordinate scale.
     *
     * @param dxRaw     Raw x travel from the contact origin.
     * @param dyRaw     Raw y travel from the contact origin.
     * @param elapsedMs Time since first contact, for flick allowance.
     * @return true when a gesture fired; the press is then spent.
     */
    bool classifyDrag(int16_t dxRaw, int16_t dyRaw, uint32_t elapsedMs);

    /** @brief Map controller coordinates to screen pixels for LVGL. */
    void mapPointer(int16_t rx, int16_t ry);

    Callbacks      _cbs;
    lv_indev_drv_t _indevDrv;
    TouchCal       _cal;

    /** @name Gesture state machine
     *  @{ */
    Phase    _phase       = Phase::Idle;
    uint32_t _contactT0   = 0;
    uint32_t _pressT0     = 0;
    uint8_t  _emptyFrames = 0;
    uint32_t _lastFreshMs = 0;                /**< Last frame the panel sent.*/
    bool     _spent       = false;
    int16_t  _startRawX = 0, _startRawY = 0;  /**< Origin, raw units.       */
    int16_t  _rawX = 0, _rawY = 0;            /**< Latest raw coordinates.  */
    int16_t  _mapX = 0, _mapY = 0;            /**< Latest mapped, for LVGL. */
    int16_t  _reportX = 0, _reportY = 0;      /**< Deadbanded, sent to LVGL.*/
    float    _travelFrac = 0;                 /**< Peak screen fraction.    */
    /** @} */

    bool _calMode = false;   /**< Calibration capture active.               */

    /** @name Pointer mapping (unchanged path — taps already worked)
     *  @{ */
    bool    _swapXY = false, _invertX = false, _invertY = false;
    int32_t _nativeW = TOUCH_NATIVE_W, _nativeH = TOUCH_NATIVE_H;
    /** @} */

    int16_t _seenMaxX = 0, _seenMaxY = 0;
    const char *_lastGesture = "-";
};

/** @brief Global touch singleton — the board has one touch panel. */
extern TouchManager Touch;
