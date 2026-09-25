/** @file WiFi.h — host mock. */
#pragma once
#include "Arduino.h"

typedef enum { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3 } wifi_mode_t;

class WiFiClassMock {
public:
    bool mode(wifi_mode_t m) { _mode = m; return true; }
    bool disconnect(bool = false, bool = false) { return true; }
    uint8_t softAPgetStationNum() { return stations; }
    String softAPmacAddress() { return "16:C1:9F:22:74:6C"; }
    String macAddress() { return "14:C1:9F:22:74:6C"; }
    uint8_t stations = 0;
    wifi_mode_t _mode = WIFI_OFF;
};
extern WiFiClassMock WiFi;
