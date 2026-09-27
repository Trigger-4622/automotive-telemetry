#pragma once
#include <Arduino.h>

#include <cstring>
#include <string>

#define WIFI_AP  2
#define WIFI_STA 1

struct IPAddress {
    String toString() const { return String("192.168.4.1"); }
};

struct WiFiClass {
    void mode(int) {}
    /** Refuses what the real one refuses: no name, a WPA2 key of 1-7 or over
     *  63 characters, a channel that does not exist. */
    bool softAP(const char *ssid, const char *pass, int channel = 1) {
        apSsid = ssid ? ssid : "";
        apPass = pass ? pass : "";
        apChannel = channel;
        const size_t k = apPass.size();
        apUp = !apSsid.empty() && apSsid.size() <= 32 && (k == 0 || (k >= 8 && k <= 63)) &&
               channel >= 1 && channel <= 13;
        return apUp;
    }
    std::string apSsid, apPass;   ///< Harness: the last softAP() call...
    int  apChannel = 0;
    bool apUp = false;            ///< ...and whether the AP came up.
    IPAddress softAPIP() { return IPAddress(); }
    String macAddress() { return String("AA:BB:CC:00:11:22"); }
};
extern WiFiClass WiFi;
