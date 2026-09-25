/**
 * @file Arduino.h
 * @brief Host mock of the Arduino-ESP32 core: just what the master uses, with
 *        time taken from the virtual clock (sim_rtos).
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "sim_rtos.h"

using std::max;
using std::min;
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define IRAM_ATTR

inline uint32_t millis() { return (uint32_t)(simrtos::nowUs() / 1000u); }
inline uint32_t micros() { return (uint32_t)simrtos::nowUs(); }
inline void delay(uint32_t ms) { vTaskDelay(ms); }
inline void delayMicroseconds(uint32_t us) { simrtos::sleepUs(us); }
inline void yield() { simrtos::yield(); }

size_t strlcpy(char *dst, const char *src, size_t size);

/* Logging: every line goes to the simulation log with its virtual time, and
 * the format string is checked against its arguments at compile time. */
void simLog(char level, const char *fmt, ...) __attribute__((format(gnu_printf, 2, 3)));
#define log_e(fmt, ...) simLog('E', fmt, ##__VA_ARGS__)
#define log_w(fmt, ...) simLog('W', fmt, ##__VA_ARGS__)
#define log_i(fmt, ...) simLog('I', fmt, ##__VA_ARGS__)
#define log_d(fmt, ...) simLog('D', fmt, ##__VA_ARGS__)

/** Just enough of Arduino's String for the master. */
class String {
public:
    String() {}
    String(const char *s) : _s(s ? s : "") {}
    String(const std::string &s) : _s(s) {}
    String(char c) : _s(1, c) {}
    String(int v) : _s(std::to_string(v)) {}
    String(unsigned v) : _s(std::to_string(v)) {}
    String(long v) : _s(std::to_string(v)) {}
    String(unsigned long v) : _s(std::to_string(v)) {}
    String(float v) : _s(std::to_string(v)) {}
    const char *c_str() const { return _s.c_str(); }
    size_t length() const { return _s.size(); }
    long toInt() const { return std::strtol(_s.c_str(), nullptr, 10); }
    String &operator+=(const String &o) { _s += o._s; return *this; }
    friend String operator+(String a, const String &b) { a._s += b._s; return a; }
    friend String operator+(String a, const char *b) { a._s += b; return a; }
    friend String operator+(String a, int v) { a._s += std::to_string(v); return a; }
    friend String operator+(String a, unsigned v) { a._s += std::to_string(v); return a; }
    friend String operator+(String a, unsigned long v) { a._s += std::to_string(v); return a; }
    bool operator==(const char *o) const { return _s == o; }
private:
    std::string _s;
};

class HardwareSerialMock {
public:
    void begin(unsigned long) {}
    void flush() {}
    size_t printf(const char *fmt, ...) __attribute__((format(gnu_printf, 2, 3)));
    size_t println(const char *s);
    size_t print(const char *s);
};
extern HardwareSerialMock Serial;

class EspClassMock {
public:
    uint32_t getFreeHeap();
    void restart();
};
extern EspClassMock ESP;

int analogRead(uint8_t pin);
inline void analogReadResolution(uint8_t) {}
