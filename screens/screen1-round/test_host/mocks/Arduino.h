/*
 * Host mock of the parts of Arduino-ESP32 the display firmware uses.
 *
 * The top half is plain C: LVGL's lv_conf.h names "Arduino.h" as its tick
 * source, and LVGL is C. Everything C++ sits below the __cplusplus guard.
 */
#pragma once

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Virtual milliseconds, advanced by the harness only. */
uint32_t millis(void);
uint32_t micros(void);
void     delay(uint32_t ms);

/** Not in the Windows C runtime. */
size_t strlcpy(char *dst, const char *src, size_t size);

void host_log(char level, const char *fmt, ...)
    __attribute__((format(gnu_printf, 2, 3)));

#ifdef __cplusplus
}
#endif

#define log_e(...) host_log('E', __VA_ARGS__)
#define log_w(...) host_log('W', __VA_ARGS__)
#define log_i(...) host_log('I', __VA_ARGS__)
#define log_d(...) host_log('D', __VA_ARGS__)

#define PROGMEM
#define PGM_P            const char *
#define FPSTR(p)         (p)
#define IRAM_ATTR
#define HEX 16
#define DEC 10

#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m)  ((void)(m))

#ifdef __cplusplus

#include <cmath>
#include <functional>
#include <string>

using std::isfinite;
using std::isinf;
using std::isnan;

/* Arduino's min/max accept mixed argument types (uint32_t against 1UL);
 * std::min/max do not. */
template <class A, class B>
inline auto min(const A &a, const B &b) -> decltype(a < b ? a : b) { return b < a ? b : a; }
template <class A, class B>
inline auto max(const A &a, const B &b) -> decltype(a < b ? a : b) { return a < b ? b : a; }

class String {
public:
    String(const char *s = "") : s_(s ? s : "") {}
    String(const std::string &s) : s_(s) {}
    explicit String(char c) : s_(1, c) {}
    String(int v, unsigned char base = 10)           { fmt((long long)v, base); }
    String(unsigned v, unsigned char base = 10)      { fmtu(v, base); }
    String(long v, unsigned char base = 10)          { fmt(v, base); }
    String(unsigned long v, unsigned char base = 10) { fmtu(v, base); }
    String(float v, unsigned char dec = 2)  { char b[48]; snprintf(b, sizeof b, "%.*f", dec, (double)v); s_ = b; }
    String(double v, unsigned char dec = 2) { char b[48]; snprintf(b, sizeof b, "%.*f", dec, v); s_ = b; }

    String &operator=(const char *s) { s_ = s ? s : ""; return *this; }
    const char *c_str() const { return s_.c_str(); }
    unsigned length() const { return (unsigned)s_.size(); }
    bool isEmpty() const { return s_.empty(); }
    bool concat(const char *s) { if (!s) return false; s_ += s; return true; }
    bool concat(const String &s) { s_ += s.s_; return true; }
    bool concat(char c) { s_ += c; return true; }
    void reserve(unsigned) {}
    String &operator+=(const String &o) { s_ += o.s_; return *this; }
    String &operator+=(const char *o) { if (o) s_ += o; return *this; }
    String &operator+=(char c) { s_ += c; return *this; }
    friend String operator+(const String &a, const String &b) { return String(a.s_ + b.s_); }
    friend String operator+(const String &a, const char *b) { return String(a.s_ + (b ? b : "")); }
    friend String operator+(const char *a, const String &b) { return String(std::string(a ? a : "") + b.s_); }
    bool operator==(const String &o) const { return s_ == o.s_; }
    bool operator==(const char *o) const { return s_ == (o ? o : ""); }
    bool operator!=(const String &o) const { return s_ != o.s_; }
    char operator[](unsigned i) const { return i < s_.size() ? s_[i] : 0; }
    bool endsWith(const String &x) const {
        return s_.size() >= x.s_.size() && s_.compare(s_.size() - x.s_.size(), x.s_.size(), x.s_) == 0;
    }
    bool startsWith(const String &x) const { return s_.compare(0, x.s_.size(), x.s_) == 0; }
    int indexOf(char c) const { auto p = s_.find(c); return p == std::string::npos ? -1 : (int)p; }
    int lastIndexOf(char c) const { auto p = s_.rfind(c); return p == std::string::npos ? -1 : (int)p; }
    String substring(unsigned from, unsigned to = 0xFFFFFFFFu) const {
        if (from > s_.size()) return String();
        return String(s_.substr(from, to == 0xFFFFFFFFu ? std::string::npos : to - from));
    }
    void replace(const String &from, const String &to) {
        if (from.s_.empty()) return;
        size_t p = 0;
        while ((p = s_.find(from.s_, p)) != std::string::npos) {
            s_.replace(p, from.s_.size(), to.s_);
            p += to.s_.size();
        }
    }
    void toUpperCase() { for (auto &c : s_) c = (char)toupper((unsigned char)c); }
    void toLowerCase() { for (auto &c : s_) c = (char)tolower((unsigned char)c); }
    int toInt() const { return atoi(s_.c_str()); }
    float toFloat() const { return (float)atof(s_.c_str()); }
    const std::string &str() const { return s_; }

private:
    void fmt(long long v, unsigned base) {
        if (v < 0) { fmtu((unsigned long long)(-v), base); s_ = "-" + s_; }
        else fmtu((unsigned long long)v, base);
    }
    void fmtu(unsigned long long v, unsigned base) {
        char b[70]; int i = 69; b[i] = 0;
        if (base < 2 || base > 16) base = 10;
        do { b[--i] = "0123456789abcdef"[v % base]; v /= base; } while (v);
        s_ = b + i;
    }
    std::string s_;
};

struct EspClass {
    uint32_t getFreeHeap()  { return 180000; }
    uint32_t getFreePsram() { return 8000000; }
    void     restart();
};
extern EspClass ESP;

#endif /* __cplusplus */
