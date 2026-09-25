#pragma once
#include <Arduino.h>
class ButtonManager {
public:
    void scanState(char *out, size_t n) { strlcpy(out, "0H", n); }
};
extern ButtonManager Button;
