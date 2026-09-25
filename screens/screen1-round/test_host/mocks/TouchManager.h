#pragma once
#include <Arduino.h>

/* Same layout as src/hal/TouchManager.h: ConfigManager stores it. */
struct TouchCal {
    bool  valid = false;
    float hx = 1.0f;
    float hy = 0.0f;
    float vx = 0.0f;
    float vy = 1.0f;
    float hLen = 400.0f;
    float vLen = 240.0f;
};

class TouchManager {
public:
    int16_t ctrlX() { return 0; }
    int16_t ctrlY() { return 0; }
    int16_t rawX() { return 0; }
    int16_t rawY() { return 0; }
    int16_t seenMaxX() { return 0; }
    int16_t seenMaxY() { return 0; }
    const char *lastGesture() { return "-"; }
    void beginCalibration() {}
    void setCalibration(const TouchCal &) {}
    void endCalibration() {}
};
extern TouchManager Touch;
