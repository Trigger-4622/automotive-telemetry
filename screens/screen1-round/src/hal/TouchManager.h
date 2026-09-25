/**
 * @file TouchManager.h
 * @ingroup hal
 * @brief CST816S capacitive touch driver + gesture recognition.
 *
 * The CST816S is polled over I2C from LVGL's input-device callback (every
 * LV_INDEV_DEF_READ_PERIOD ms). Gesture classification is done in firmware
 * from raw coordinates — the chip's built-in gesture register is orientation
 * dependent and unreliable across panel clones, raw deltas are not.
 *
 * Recognized gestures (thresholds in HardwareConfig.h):
 *   - Swipe left / right : screen navigation
 *   - Swipe up           : cycle day/night override (auto → day → night)
 *   - Swipe down         : reset peak values
 *   - Tap                : toggle peak-value display
 *   - 5 s hold           : launch the configuration access point
 */
#pragma once

#include <Arduino.h>
#include <lvgl.h>

#include "HardwareConfig.h"

/** @brief Gesture callback (no arguments — gestures are global UI events). */
typedef void (*GestureCallback)();

/**
 * @brief Polls the CST816S and turns raw coordinates into gestures.
 *
 * Singleton — see @ref Touch.
 */
class TouchManager {
public:
    /** @brief Handlers invoked when a gesture is recognized. */
    struct Callbacks {
        GestureCallback onSwipeLeft  = nullptr; /**< Horizontal, leftward.  */
        GestureCallback onSwipeRight = nullptr; /**< Horizontal, rightward. */
        GestureCallback onSwipeUp    = nullptr; /**< Vertical, upward.      */
        GestureCallback onSwipeDown  = nullptr; /**< Vertical, downward.    */
        GestureCallback onTap        = nullptr; /**< Short press, no travel.*/
        GestureCallback onHold       = nullptr; /**< Fired once per press.  */
    };

    /**
     * @brief Bring up I2C, reset the controller and register with LVGL.
     * @param cbs Gesture handlers; any member may be nullptr.
     */
    void begin(const Callbacks &cbs);

private:
    /**
     * @brief LVGL input-device read callback; forwards to @ref handleRead.
     * @param drv LVGL driver whose `user_data` is the TouchManager.
     * @param[out] data Pointer state LVGL will consume.
     */
    static void readCb(lv_indev_drv_t *drv, lv_indev_data_t *data);

    /**
     * @brief Poll the panel, drive the gesture state machine, report to LVGL.
     * @param[out] data Pointer state LVGL will consume.
     */
    void handleRead(lv_indev_data_t *data);

    /**
     * @brief Read one touch sample over I2C.
     * @param[out] x       X coordinate, valid only when @p touched.
     * @param[out] y       Y coordinate, valid only when @p touched.
     * @param[out] touched True while a finger is down.
     * @return false on an I2C error, in which case the outputs are untouched
     *         and the caller should hold its previous state.
     */
    bool readPanel(int16_t &x, int16_t &y, bool &touched);

    Callbacks      _cbs;        /**< Registered gesture handlers.           */
    lv_indev_drv_t _indevDrv;   /**< LVGL input-device driver.              */

    /** @name Gesture state machine
     *  Tracked across polls to classify a press on release.
     *  @{ */
    bool     _pressed   = false;  /**< A finger is currently down.          */
    bool     _holdFired = false;  /**< Hold already reported for this press.*/
    uint32_t _pressT0   = 0;      /**< millis() when the press began.       */
    int16_t  _startX = 0, _startY = 0;  /**< Where the press began.         */
    int16_t  _lastX  = 0, _lastY  = 0;  /**< Most recent coordinates.       */
    /** @} */
};

/** @brief Global touch singleton — the board has one touch panel. */
extern TouchManager Touch;
