/* Host stand-ins for the hardware singletons the UI touches. The harness owns
 * the LVGL display itself (see host/lvgl_host.cpp). */
#pragma once
#include <Arduino.h>

class DisplayManager {
public:
    void setBrightness(uint8_t b) { brightness = b; }
    void setCalibration(float r, float g, float b) { gain[0] = r; gain[1] = g; gain[2] = b; }
    uint8_t brightness = 0;
    float gain[3] = {1, 1, 1};
};
extern DisplayManager Display;
