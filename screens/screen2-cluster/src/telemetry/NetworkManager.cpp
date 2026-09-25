/**
 * @file NetworkManager.cpp
 * @ingroup telemetry
 * @brief ESP-NOW listener implementation.
 */
#include "NetworkManager.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "ConfigManager.h"
#include "MasterPacket.h"
#include "TelemetryStore.h"

NetworkManager Net;

/**
 * @brief Optional sender filter, parsed once from `network.master_mac`.
 *
 * All-zero means "accept anything", which is the default. A filter matters
 * when more than one master is in range — a second vehicle, or a bench rig
 * sitting on the same channel — since ESP-NOW broadcasts carry no notion of
 * which network they belong to.
 */
static uint8_t s_peerFilter[6] = {0};
static bool    s_filterActive  = false;

/**
 * @brief Parse "AA:BB:CC:DD:EE:FF" into six bytes.
 * @param text Candidate string; empty or malformed disables filtering.
 * @param[out] out Six-byte MAC.
 * @return true when @p text was a well-formed MAC.
 */
static bool parseMac(const char *text, uint8_t out[6]) {
    if (!text || strlen(text) < 17) return false;
    int v[6];
    if (sscanf(text, "%x:%x:%x:%x:%x:%x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6)
        return false;
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[i];
    return true;
}

/*
 * The receive callback signature changed between Arduino-ESP32 core 2.x and
 * 3.x. Both are supported; platformio.ini pins core 2.0.17 (first branch is
 * future-proofing only).
 */
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
/**
 * @brief ESP-NOW receive callback (Arduino core 3.x signature).
 * @param data Received payload.
 * @param len  Payload length in bytes.
 * @note Runs in the Wi-Fi task. Keep it to a bounded handoff — the store
 *       takes a spinlock, so anything slow here stalls the radio.
 */
static void onEspNowRecv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int len) {
    const uint8_t *mac = info ? info->src_addr : nullptr;
    if (s_filterActive && mac && memcmp(mac, s_peerFilter, 6) != 0) return;
    Telemetry.ingestRaw(mac, data, len);
}
#else
/**
 * @brief ESP-NOW receive callback (Arduino core 2.x signature).
 * @param data Received payload.
 * @param len  Payload length in bytes.
 * @note Runs in the Wi-Fi task — see the 3.x overload's note.
 */
static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (s_filterActive && mac && memcmp(mac, s_peerFilter, 6) != 0) return;
    Telemetry.ingestRaw(mac, data, len);
}
#endif

void NetworkManager::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);   // no AP association — we only sniff ESP-NOW

    s_filterActive = parseMac(Config.masterMac(), s_peerFilter);

    // Lock the radio to the configured channel. Master and slave must agree:
    // a mismatch produces silence, not an error, which is a miserable thing
    // to debug — so it is logged explicitly here.
    const uint8_t channel = constrain((int)Config.wifiChannel(), 1, 13);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        log_e("esp_now_init failed — telemetry link dead");
        return;
    }
    esp_now_register_recv_cb(onEspNowRecv);
    _running = true;
    log_i("ESP-NOW listening on channel %u, peer filter %s", channel,
          s_filterActive ? Config.masterMac() : "off (any master)");
}

void NetworkManager::stop() {
    if (!_running) return;
    esp_now_unregister_recv_cb();
    esp_now_deinit();
    _running = false;
    log_i("ESP-NOW stopped");
}
