/**
 * @file TouchManager.cpp
 * @ingroup hal
 * @brief GT911 driver and the measured-axis gesture engine.
 */
#include "TouchManager.h"

#include <Wire.h>
#include <math.h>

TouchManager Touch;

/** @name GT911 register map (16-bit addresses, subset)
 *  @{ */
static constexpr uint16_t REG_PRODUCT_ID = 0x8140; /**< 4 ASCII bytes: "911". */
static constexpr uint16_t REG_STATUS     = 0x814E; /**< bit7 ready, bits0-3 n.*/
static constexpr uint16_t REG_POINT1     = 0x8150; /**< First contact block.  */
/** @} */

/** Address actually answering, resolved during begin(). */
static uint8_t s_addr = TOUCH_I2C_ADDR;

/**
 * @brief Write a 16-bit register address, then read @p len bytes.
 * @param reg      Register address.
 * @param[out] buf Destination.
 * @param len      Byte count.
 * @return false on any I2C error.
 */
static bool gtRead(uint16_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(s_addr);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)s_addr, (int)len) != (int)len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

/**
 * @brief Write one byte to a 16-bit register.
 * @param reg Register address.
 * @param val Value.
 * @return false on any I2C error.
 */
static bool gtWrite(uint16_t reg, uint8_t val) {
    Wire.beginTransmission(s_addr);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

/**
 * @brief Is a GT911 answering at @p addr?
 * @param addr 7-bit I2C address to probe.
 * @return true when the product-ID register reads back "911".
 */
static bool gtProbe(uint8_t addr) {
    s_addr = addr;
    uint8_t id[4] = {0};
    if (!gtRead(REG_PRODUCT_ID, id, 4)) return false;
    return id[0] == '9' && id[1] == '1' && id[2] == '1';
}

void TouchManager::begin(const Callbacks &cbs, const TouchCal &cal,
                         bool swapXY, bool invertX, bool invertY,
                         int nativeW, int nativeH) {
    _cbs     = cbs;
    _cal     = cal;
    _swapXY  = swapXY;
    _invertX = invertX;
    _invertY = invertY;
    _nativeW = (nativeW > 1) ? nativeW : TOUCH_NATIVE_W;
    _nativeH = (nativeH > 1) ? nativeH : TOUCH_NATIVE_H;

    Wire.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, TOUCH_I2C_HZ);

    /*
     * Address selection. The GT911 latches its I2C address from the INT pin
     * on the rising edge of RST:
     *     INT low  at release -> 0x5D
     *     INT high at release -> 0x14
     */
    pinMode(PIN_TOUCH_RST, OUTPUT);
    pinMode(PIN_TOUCH_INT, OUTPUT);
    digitalWrite(PIN_TOUCH_RST, LOW);
    digitalWrite(PIN_TOUCH_INT, LOW);      // select 0x5D
    delay(12);
    digitalWrite(PIN_TOUCH_RST, HIGH);     // latch the address
    delay(6);
    pinMode(PIN_TOUCH_INT, INPUT);
    delay(60);                             // controller boots

    if (!gtProbe(TOUCH_I2C_ADDR) && !gtProbe(TOUCH_I2C_ADDR_ALT)) {
        log_e("GT911 not found at 0x%02X or 0x%02X - touch dead",
              TOUCH_I2C_ADDR, TOUCH_I2C_ADDR_ALT);
    } else {
        log_i("GT911 at 0x%02X, calibration %s", s_addr,
              _cal.valid ? "loaded" : "DEFAULT (run the wizard)");
    }

    lv_indev_drv_init(&_indevDrv);
    _indevDrv.type      = LV_INDEV_TYPE_POINTER;
    _indevDrv.read_cb   = readCb;
    _indevDrv.user_data = this;
    lv_indev_drv_register(&_indevDrv);
}

bool TouchManager::readPanel(int16_t &x, int16_t &y, bool &touched, bool &fresh) {
    uint8_t status = 0;
    if (!gtRead(REG_STATUS, &status, 1)) return false;

    /*
     * Bit 7 means "a new coordinate frame is ready", NOT "a finger is down".
     * The controller refreshes around 100 Hz while LVGL polls every 20 ms,
     * unsynchronised, so many polls land between refreshes with the flag clear
     * even while a finger is still on the glass. Treating that as a release is
     * what fragments a swipe into a burst of taps.
     */
    if (!(status & 0x80)) {
        fresh   = false;
        touched = false;
        return true;
    }

    fresh = true;
    const uint8_t points = status & 0x0F;
    if (points == 0 || points > TOUCH_MAX_POINTS) {
        touched = false;
        gtWrite(REG_STATUS, 0);            // must clear, or it never re-arms
        return true;
    }

    uint8_t p[6];
    const bool ok = gtRead(REG_POINT1, p, sizeof(p));
    gtWrite(REG_STATUS, 0);
    if (!ok) return false;

    // Bytes: [0] track id, [1..2] x LE, [3..4] y LE, [5] size
    x = (int16_t)(p[1] | (p[2] << 8));
    y = (int16_t)(p[3] | (p[4] << 8));
    touched = true;
    if (x > _seenMaxX) _seenMaxX = x;
    if (y > _seenMaxY) _seenMaxY = y;
    return true;
}

void TouchManager::mapPointer(int16_t rx, int16_t ry) {
    // Deliberately the old, simple path: taps have always landed correctly,
    // and only gesture DIRECTION needed rebuilding. Changing both at once
    // would make it impossible to tell which change fixed what.
    int32_t x = rx, y = ry;
    if (_swapXY) { const int32_t t = x; x = y; y = t; }

    const int32_t nw = _swapXY ? _nativeH : _nativeW;
    const int32_t nh = _swapXY ? _nativeW : _nativeH;
    x = x * (LCD_WIDTH  - 1) / (nw > 1 ? nw - 1 : 1);
    y = y * (LCD_HEIGHT - 1) / (nh > 1 ? nh - 1 : 1);

    if (_invertX) x = (LCD_WIDTH  - 1) - x;
    if (_invertY) y = (LCD_HEIGHT - 1) - y;

    if (x < 0) x = 0; else if (x >= LCD_WIDTH)  x = LCD_WIDTH  - 1;
    if (y < 0) y = 0; else if (y >= LCD_HEIGHT) y = LCD_HEIGHT - 1;
    _mapX = (int16_t)x;
    _mapY = (int16_t)y;
}

bool TouchManager::classifyDrag(int16_t dxRaw, int16_t dyRaw,
                                uint32_t elapsedMs) {
    /*
     * Project the raw drag onto the two MEASURED axes, then divide each
     * projection by the length of a full-screen swipe along that axis.
     *
     * The division is the whole point. It turns two numbers in arbitrary,
     * unequal controller units into two dimensionless fractions - "this far
     * across the screen" and "this far down it" - which can finally be
     * compared with each other. The old engine compared raw projections
     * directly, so whichever axis happened to count more units per millimetre
     * won every contest, and one swipe direction stayed dead no matter how the
     * thresholds were set.
     */
    const float projH = dxRaw * _cal.hx + dyRaw * _cal.hy;
    const float projV = dxRaw * _cal.vx + dyRaw * _cal.vy;
    const float fracH = projH / (_cal.hLen > 1.0f ? _cal.hLen : 1.0f);
    const float fracV = projV / (_cal.vLen > 1.0f ? _cal.vLen : 1.0f);

    const float aH = fabsf(fracH), aV = fabsf(fracV);
    if (aH > _travelFrac) _travelFrac = aH;
    if (aV > _travelFrac) _travelFrac = aV;

    // Thresholds are fractions of a screen traversal, so they stay meaningful
    // even if the scale estimate is imperfect. A flick is allowed a shorter
    // distance because a fast movement is unambiguous on its own.
    const float need = (elapsedMs <= GESTURE_FLICK_MS)
                           ? GESTURE_SWIPE_FRAC_FLICK : GESTURE_SWIPE_FRAC;
    const float dominance = 1.3f;

    if (aH >= need && aH >= aV * dominance) {
        _lastGesture = (fracH < 0) ? "swipe L" : "swipe R";
        if (fracH < 0) { if (_cbs.onSwipeLeft)  _cbs.onSwipeLeft();  }
        else           { if (_cbs.onSwipeRight) _cbs.onSwipeRight(); }
        return true;
    }
    if (aV >= need && aV >= aH * dominance) {
        _lastGesture = (fracV < 0) ? "swipe U" : "swipe D";
        if (fracV < 0) { if (_cbs.onSwipeUp)   _cbs.onSwipeUp();   }
        else           { if (_cbs.onSwipeDown) _cbs.onSwipeDown(); }
        return true;
    }
    return false;
}

void TouchManager::readCb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    static_cast<TouchManager *>(drv->user_data)->handleRead(data);
}

void TouchManager::handleRead(lv_indev_data_t *data) {
    int16_t rx = _rawX, ry = _rawY;
    bool touched = false, fresh = false;
    const uint32_t now = millis();

    if (!readPanel(rx, ry, touched, fresh)) {
        // A dropped I2C read is not a lift. Hold state.
        data->state   = (_phase == Phase::Pressed) ? LV_INDEV_STATE_PRESSED
                                                   : LV_INDEV_STATE_RELEASED;
        data->point.x = _reportX;
        data->point.y = _reportY;
        return;
    }

    if (fresh) {
        if (touched) {
            _emptyFrames = 0;
            _rawX = rx;
            _rawY = ry;
            mapPointer(rx, ry);

            switch (_phase) {
                case Phase::Idle:
                    // Not reported yet - this may be a phantom frame.
                    _phase      = Phase::Candidate;
                    _contactT0  = now;
                    _startRawX  = rx;
                    _startRawY  = ry;
                    _travelFrac = 0;
                    _reportX    = _mapX;
                    _reportY    = _mapY;
                    break;

                case Phase::Candidate:
                    /*
                     * Travel is itself proof the contact is real, so a swipe
                     * may fire before the press is confirmed. That is what
                     * lets the click threshold stay long for phantom
                     * rejection without making swipes feel sluggish.
                     */
                    if (!_calMode &&
                        classifyDrag(rx - _startRawX, ry - _startRawY,
                                     now - _contactT0)) {
                        _phase   = Phase::Pressed;
                        _pressT0 = _contactT0;
                        _spent   = true;
                    } else if (now - _contactT0 >= GESTURE_PRESS_MIN_MS) {
                        _phase   = Phase::Pressed;
                        _pressT0 = _contactT0;
                        _spent   = false;
                        _reportX = _mapX;
                        _reportY = _mapY;
                    }
                    break;

                case Phase::Pressed: {
                    if (!_spent && !_calMode) {
                        if (classifyDrag(rx - _startRawX, ry - _startRawY,
                                         now - _contactT0)) {
                            _spent = true;
                        } else if (now - _pressT0 >= GESTURE_HOLD_CONFIG_MS &&
                                   _travelFrac <= GESTURE_HOLD_SLOP_FRAC) {
                            _spent = true;
                            _lastGesture = "hold";
                            if (_cbs.onHold) _cbs.onHold();
                        }
                    }
                    // Deadband what LVGL sees, so a resting finger is not a
                    // drag that cancels presses on widgets.
                    if (abs(_mapX - _reportX) >= TOUCH_JITTER_PX) _reportX = _mapX;
                    if (abs(_mapY - _reportY) >= TOUCH_JITTER_PX) _reportY = _mapY;
                    break;
                }
            }
        } else {
            // Several empty frames in a row before believing a release: the
            // controller drops the odd frame mid-drag.
            if (_emptyFrames < 255) _emptyFrames++;

            if (_emptyFrames >= GESTURE_RELEASE_FRAMES) {
                if (_phase == Phase::Pressed || _phase == Phase::Candidate) {
                    const int16_t dxRaw = _rawX - _startRawX;
                    const int16_t dyRaw = _rawY - _startRawY;

                    if (_calMode) {
                        // Measuring, not classifying: hand the caller exactly
                        // what the controller reported for this drag.
                        if (_cbs.onRawDrag) _cbs.onRawDrag(dxRaw, dyRaw);
                        _lastGesture = "captured";
                    } else if (_phase == Phase::Pressed && !_spent) {
                        const uint32_t dt = now - _pressT0;
                        if (dt <= GESTURE_TAP_MAX_MS &&
                            _travelFrac <= GESTURE_TAP_MAX_FRAC) {
                            _lastGesture = "tap";
                            if (_cbs.onTap) _cbs.onTap();
                        }
                    }
                }
                _phase      = Phase::Idle;
                _spent      = false;
                _travelFrac = 0;
            }
        }
    }
    // Not fresh: no news. Hold every bit of state exactly as it was.

    data->state   = (_phase == Phase::Pressed) ? LV_INDEV_STATE_PRESSED
                                               : LV_INDEV_STATE_RELEASED;
    data->point.x = _reportX;
    data->point.y = _reportY;
}
