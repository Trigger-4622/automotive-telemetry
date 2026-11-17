/**
 * @file Update.h
 * @brief The Arduino core's firmware updater for the host tests: what it was
 *        given is kept in sim::otaImage, and end() "switches" to it
 *        (sim::otaSwitched) when the image starts like an ESP32 image.
 */
#pragma once
#include <cstddef>
#include <cstdint>

#define UPDATE_SIZE_UNKNOWN 0xFFFFFFFF
#define U_FLASH 0

class UpdateClass {
public:
    bool begin(size_t size = UPDATE_SIZE_UNKNOWN, int command = U_FLASH, int ledPin = -1,
               uint8_t ledOn = 0, const char *label = nullptr);
    size_t write(uint8_t *data, size_t len);
    bool end(bool evenIfRemaining = false);
    void abort();
    const char *errorString();
    bool hasError() { return _err != 0; }
private:
    int  _err = 0;
    bool _running = false;
};
extern UpdateClass Update;
