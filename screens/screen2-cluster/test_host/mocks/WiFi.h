#pragma once
#include <Arduino.h>

#define WIFI_AP  2
#define WIFI_STA 1

struct IPAddress {
    String toString() const { return String("192.168.4.1"); }
};

struct WiFiClass {
    void mode(int) {}
    bool softAP(const char *, const char *, int = 1) { return true; }
    IPAddress softAPIP() { return IPAddress(); }
    String macAddress() { return String("AA:BB:CC:00:11:22"); }
};
extern WiFiClass WiFi;
