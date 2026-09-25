/**
 * @file NetworkManager.h
 * @ingroup telemetry
 * @brief ESP-NOW receive-only link to the Master node's broadcast firehose.
 *
 * The slave never transmits: it parks the STA interface on the contract
 * channel (@ref TELEMETRY_WIFI_CHANNEL) and funnels every valid broadcast
 * frame into the global @ref TelemetryStore. When the user enters config
 * mode, @ref stop tears ESP-NOW down so the radio can be repurposed as an AP.
 */
#pragma once

#include <Arduino.h>

/**
 * @ingroup telemetry
 * @brief Owns the ESP-NOW listener. Singleton — see @ref Net.
 */
class NetworkManager {
public:
    /**
     * @brief Bring up Wi-Fi STA + ESP-NOW and start listening.
     *
     * Locks the radio to @ref TELEMETRY_WIFI_CHANNEL. Master and slave must
     * agree on that channel or no frames arrive at all.
     */
    void begin();

    /**
     * @brief Tear down ESP-NOW.
     *
     * Must be called before raising the configuration AP — the radio cannot
     * serve both roles at once. Safe to call when already stopped.
     */
    void stop();

    /**
     * @brief Whether the listener is active.
     * @return true between @ref begin and @ref stop.
     */
    bool isRunning() const { return _running; }

private:
    bool _running = false;   /**< Listener state. */
};

/** @brief Global network singleton. */
extern NetworkManager Net;
