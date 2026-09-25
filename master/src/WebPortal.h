/**
 * @file WebPortal.h
 * @brief Configuration and diagnostics portal for the master.
 *
 * ## It runs alongside ESP-NOW, not instead of it
 *
 * The displays have to stop their ESP-NOW listener to raise their AP, because
 * a station cannot serve an access point on one interface. The master does not
 * have that problem: it only ever *transmits*, and ESP-NOW transmission works
 * happily from an AP interface as long as the AP sits on the same channel the
 * peers listen on.
 *
 * So the portal is up at the same time as the telemetry broadcast, and the
 * gauges keep updating while you configure. That is the whole reason
 * @ref MasterConfig::wifiChannel and the AP channel are pinned together —
 * letting the AP pick its own channel would silently kill every display.
 *
 * ## What it is for
 *
 * Two jobs. Settings, which are the obvious part. And a live view of the CAN
 * bus — the ID census, frame rates and decoded metrics — which is the part
 * that actually matters, because the alternative is a laptop on a serial port
 * while sitting in a car.
 */
#pragma once

#include <Arduino.h>

/**
 * @brief AP + web server. Singleton — see @ref Portal.
 */
class WebPortal {
public:
    /**
     * @brief Raise the access point and start serving.
     *
     * Pins the AP to MasterConfig::wifiChannel so ESP-NOW keeps working; see
     * the note at the top of this file.
     */
    void begin();

    /** @brief Service HTTP. Call from loop(); returns at once when disabled. */
    void loop();

    /** @return true when the portal is running. */
    bool running() const { return _running; }

private:
    /** @brief Register every route. */
    void setupRoutes();

    bool _running = false;
};

/** @brief Global portal singleton. */
extern WebPortal Portal;
