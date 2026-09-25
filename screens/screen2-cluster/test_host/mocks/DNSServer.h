#pragma once
#include "WiFi.h"
class DNSServer {
public:
    bool start(uint16_t, const char *, const IPAddress &) { return true; }
    void processNextRequest() {}
};
