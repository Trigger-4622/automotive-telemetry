/**
 * @file TouchManager.cpp
 * @ingroup hal
 * @brief CST816S polling driver and gesture state machine.
 */
#include "TouchManager.h"

#include <Wire.h>

TouchManager Touch;

/** @name CST816S register map (subset)
 *  @{ */
static constexpr uint8_t REG_GESTURE       = 0x01; /**< Chip gesture code (unused). */
static constexpr uint8_t REG_FINGER_NUM    = 0x02; /**< Contacts currently down.    */
static constexpr uint8_t REG_XPOS_H        = 0x03; /**< First of 4 coordinate bytes.*/
static constexpr uint8_t REG_DIS_AUTOSLEEP = 0xFE; /**< 1 = never auto-sleep.       */
/** @} */

void TouchManager::begin(const Callbacks &cbs) {
    _cbs = cbs;

    Wire.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, TOUCH_I2C_HZ);

    // Hardware reset pulse — the CST816S needs it after cold boot.
    pinMode(PIN_TOUCH_RST, OUTPUT);
    digitalWrite(PIN_TOUCH_RST, LOW);
    delay(10);
    digitalWrite(PIN_TOUCH_RST, HIGH);
    delay(80);

    pinMode(PIN_TOUCH_INT, INPUT_PULLUP);

    // Keep the controller awake — its auto-sleep stops coordinate streaming
    // which would break long-press detection.
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(REG_DIS_AUTOSLEEP);
    Wire.write(0x01);
    Wire.endTransmission();

    lv_indev_drv_init(&_indevDrv);
    _indevDrv.type      = LV_INDEV_TYPE_POINTER;
    _indevDrv.read_cb   = readCb;
    _indevDrv.user_data = this;
    lv_indev_drv_register(&_indevDrv);

    log_i("CST816S touch ready (SDA=%d SCL=%d)", PIN_TOUCH_SDA, PIN_TOUCH_SCL);
}

bool TouchManager::readPanel(int16_t &x, int16_t &y, bool &touched) {
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(REG_GESTURE);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)TOUCH_I2C_ADDR, 6) != 6) return false;

    /*uint8_t gesture =*/ Wire.read();      // ignored — see header note
    uint8_t fingers  = Wire.read();
    uint8_t xh       = Wire.read();
    uint8_t xl       = Wire.read();
    uint8_t yh       = Wire.read();
    uint8_t yl       = Wire.read();

    touched = fingers > 0;
    x = ((xh & 0x0F) << 8) | xl;
    y = ((yh & 0x0F) << 8) | yl;
    return true;
}

void TouchManager::readCb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    static_cast<TouchManager *>(drv->user_data)->handleRead(data);
}

void TouchManager::handleRead(lv_indev_data_t *data) {
    int16_t x = _lastX, y = _lastY;
    bool touched = false;
    if (!readPanel(x, y, touched)) {          // I2C hiccup — keep last state
        data->state   = _pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
        data->point.x = _lastX;
        data->point.y = _lastY;
        return;
    }

    const uint32_t now = millis();

    if (touched && !_pressed) {               // ---- press started ----
        _pressed   = true;
        _holdFired = false;
        _pressT0   = now;
        _startX = x; _startY = y;
    }

    if (touched) {
        _lastX = x; _lastY = y;
        // ---- 5-second hold (finger must stay put) ----
        if (!_holdFired &&
            now - _pressT0 >= GESTURE_HOLD_CONFIG_MS &&
            abs(x - _startX) < GESTURE_TAP_MAX_PX * 2 &&
            abs(y - _startY) < GESTURE_TAP_MAX_PX * 2) {
            _holdFired = true;
            if (_cbs.onHold) _cbs.onHold();
        }
    } else if (_pressed) {                    // ---- released: classify ----
        _pressed = false;
        const int16_t  dx = _lastX - _startX;
        const int16_t  dy = _lastY - _startY;
        const uint32_t dt = now - _pressT0;

        if (!_holdFired) {
            if (abs(dx) >= GESTURE_SWIPE_MIN_PX && abs(dx) > abs(dy)) {
                if (dx < 0) { if (_cbs.onSwipeLeft)  _cbs.onSwipeLeft();  }
                else        { if (_cbs.onSwipeRight) _cbs.onSwipeRight(); }
            } else if (abs(dy) >= GESTURE_SWIPE_MIN_PX && abs(dy) > abs(dx)) {
                if (dy < 0) { if (_cbs.onSwipeUp)   _cbs.onSwipeUp();   }
                else        { if (_cbs.onSwipeDown) _cbs.onSwipeDown(); }
            } else if (dt <= GESTURE_TAP_MAX_MS &&
                       abs(dx) < GESTURE_TAP_MAX_PX &&
                       abs(dy) < GESTURE_TAP_MAX_PX) {
                if (_cbs.onTap) _cbs.onTap();
            }
        }
    }

    data->state   = _pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->point.x = _lastX;
    data->point.y = _lastY;
}
