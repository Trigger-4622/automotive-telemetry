/**
 * @file ConfigManager.h
 * @ingroup config
 * @brief Owns persistent configuration (LittleFS), the parsed layout.json
 *        document, and the "Gauge Studio" configuration access point:
 *        captive-portal DNS + web server + REST/upload API.
 *
 * Normal operation : filesystem + layout only; the radio belongs to ESP-NOW.
 * Config mode      : entered via a 3 s touch hold or a 2.5 s BOOT-button
 *                    hold (GESTURE_HOLD_CONFIG_MS, BUTTON_LONGPRESS_MS). ESP-NOW is stopped first,
 *                    then this module raises an open AP (CONFIG_AP_SSID),
 *                    captive-portal DNS, and serves the single-page Gauge
 *                    Studio. The page is compiled into the firmware
 *                    (tools/embed_studio.py), so flashing the firmware updates
 *                    it: no `uploadfs`, which would also have replaced the
 *                    layout on the device.
 *
 * REST API (all JSON unless noted):
 *   GET  /api/layout       current layout.json
 *   POST /api/layout       replace layout.json: a form's file (the studio) or
 *                          a bare JSON body, written to flash as it arrives,
 *                          then checked and applied (applyLayoutFile)
 *   GET  /api/live         live metric snapshot (id, value, peak, flags, age)
 *   GET  /api/info         firmware/FS/link info
 *   GET  /api/assets       list of files in /assets
 *   POST /api/asset        multipart upload → /assets/<basename>
 *   POST /api/asset/delete ?name=<file> remove an asset
 *   POST /api/peaks/reset  zero all peak values
 *   POST /api/reboot       apply layout by restarting
 */
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <FS.h>
#include <WebServer.h>

#include "HardwareConfig.h"   // PIN_BUTTON, CONFIG_AP_* defaults
#include "TouchManager.h"     // TouchCal
#include "MasterPacket.h"     // TELEMETRY_WIFI_CHANNEL default

/**
 * @defgroup config Configuration &amp; Web Portal
 * @brief Layout persistence on LittleFS and the Gauge Studio captive portal.
 * @{
 */

/**
 * @brief Loads/stores layout.json and serves the configuration portal.
 *
 * Singleton — see @ref Config.
 */
class ConfigManager {
public:
    /**
     * @brief Mount LittleFS and load (or create) `/layout.json`.
     *
     * Mounts in two stages: a plain mount first, formatting only if that
     * fails. Formatting on the first failure would erase an uploaded
     * filesystem, turning a transient problem into data loss.
     */
    void begin();

    /**
     * @brief Raise the configuration AP, captive-portal DNS and web server.
     * @warning Call NetworkManager::stop() first — the radio cannot run the
     *          ESP-NOW listener and an AP at the same time.
     */
    void enterConfigMode();

    /**
     * @brief Service DNS + HTTP. Call every loop() iteration.
     *
     * Returns immediately unless config mode is active, so it is safe to
     * call unconditionally.
     */
    void loop();

    /**
     * @brief Whether the configuration portal is running.
     * @return true after @ref enterConfigMode.
     */
    bool inConfigMode() const { return _configMode; }

    /**
     * @brief The AP's IP address as text, for display on-screen.
     * @return Dotted-quad string, empty before @ref enterConfigMode.
     */
    String apIP() const { return _apIP; }

    /**
     * @brief The parsed layout document.
     * @return Mutable reference, valid after @ref begin.
     */
    JsonDocument &layout() { return _layout; }

    /** @name Global settings
     *  Typed accessors over `layout["global"]`, each with a safe default so a
     *  hand-edited layout that omits a key still boots.
     *  @{ */
    float    emaAlpha()    { return _layout["global"]["smoothing_alpha"] | 0.35f; }
    /** @return Needle glide per UI tick, 0.01-1: above 1 a needle passes its
     *          value, and above 2 it swings further every frame. */
    float    lerpSpeed()   {
        const float v = _layout["global"]["lerp_speed"] | 0.18f;
        return v < 0.01f ? 0.01f : (v > 1.0f ? 1.0f : v);
    }
    /** @return Metric age before it renders as "--" [ms]. */
    uint32_t staleMs()     { return _layout["global"]["stale_timeout_ms"]| 1500;  }
    /** @return Screen-transition duration [ms]. */
    uint32_t swipeAnimMs() { return _layout["global"]["swipe_anim_ms"]   | 180;   }
    /** @return "over" (cheapest) | "move" | "fade" | "none" — see
     *          UIBuilder::loadScreen for the cost of each. */
    const char *swipeAnimStyle() { return _layout["global"]["swipe_anim"] | "over"; }
    /** @return Easing curve for screen transitions: linear, ease_in, ease_out,
     *          ease_in_out, overshoot or bounce. LVGL's own default is linear,
     *          which is what makes an un-eased slide feel mechanical. */
    const char *swipeEasing() { return _layout["global"]["swipe_easing"] | "ease_out"; }
    /**
     * @return false to switch off the per-gauge warning/critical recoloring
     *         and pulsing entirely, leaving every gauge in its normal color.
     *         The watchdog is unaffected — it is a separate, deliberate path.
     */
    bool gaugeAlarmColors() { return _layout["global"]["gauge_alarm_colors"] | true; }
    /** @return How long a tap silences an acknowledged alarm [ms]. */
    uint32_t alertAckMs()  { return _layout["global"]["alert_ack_ms"]    | 30000; }
    /**
     * @return true to put the panel in BGR order after init.
     *
     * Arduino_GFX always writes MADCTL with the RGB bit, which is correct for
     * the reference board. If red and blue look exchanged on yours, set
     * `global.bgr_order` true - the DIAGNOSTICS screen's colour strip tells
     * you which it is without guessing.
     */
    bool bgrOrder()     { return _layout["global"]["bgr_order"] | false; }

    /**
     * @brief Load the measured touch mapping.
     * @return A TouchCal; `valid` is false when nothing has been captured,
     *         in which case the engine falls back to its defaults.
     */
    TouchCal touchCal() {
        TouchCal c;
        JsonObject o = _layout["global"]["touch_cal"];
        if (o.isNull()) return c;
        c.hx   = o["hx"]   | 1.0f;
        c.hy   = o["hy"]   | 0.0f;
        c.vx   = o["vx"]   | 0.0f;
        c.vy   = o["vy"]   | 1.0f;
        c.hLen = o["hlen"] | 400.0f;
        c.vLen = o["vlen"] | 240.0f;
        c.valid = o["valid"] | false;
        return c;
    }

    /**
     * @brief Store a measured mapping and persist it.
     * @param c Mapping to save.
     * @return false if the filesystem write failed.
     */
    bool saveTouchCal(const TouchCal &c);

    /** @name Colour calibration
     *  Per-channel gain dialled in from the web portal while watching the
     *  screen. 1.0 is untouched.
     *  @{ */
    float colorGainR() { return _layout["global"]["color_gain_r"] | 1.0f; }
    float colorGainG() { return _layout["global"]["color_gain_g"] | 1.0f; }
    float colorGainB() { return _layout["global"]["color_gain_b"] | 1.0f; }
    /** @} */

    /** @return Side-button GPIO. Identify yours with the diagnostics scan. */
    int      buttonGpio()  { return _layout["global"]["button_gpio"] | PIN_BUTTON; }
    /** @return true to run the button discovery scan (see ButtonManager). */
    bool buttonDiscovery() { return _layout["global"]["button_discovery"] | true; }
    /** @} */

    /** @name Network settings
     *  Read once at boot — changing any of them needs a reboot to take effect.
     *  @{ */
    /**
     * @return ESP-NOW channel, 1-13. **Must match the master**; a mismatch
     *         means no frames are received at all, with no error anywhere.
     *         Out of range is taken as the nearest: the config AP would not
     *         start on a channel that does not exist.
     */
    uint8_t wifiChannel() {
        const int c = _layout["network"]["wifi_channel"] | (int)TELEMETRY_WIFI_CHANNEL;
        return (uint8_t)(c < 1 ? 1 : (c > 13 ? 13 : c));
    }
    /**
     * @return Master MAC to accept frames from, as "AA:BB:CC:DD:EE:FF", or an
     *         empty string to accept any sender (the default). Useful when two
     *         vehicles or a bench master are in range of each other.
     */
    const char *masterMac() { return _layout["network"]["master_mac"] | ""; }
    /**
     * @return SSID of the configuration access point: the built-in one when
     *         the layout's is empty or longer than the 32 bytes Wi-Fi allows.
     *         The AP would not start, and the studio - the only way into these
     *         settings - would be out of reach.
     */
    const char *apSsid() {
        const char *s = _layout["network"]["ap_ssid"] | CONFIG_AP_SSID;
        const size_t n = strlen(s);
        return n == 0 || n > 32 ? CONFIG_AP_SSID : s;
    }
    /**
     * @return Password for the configuration AP. Fewer than 8 characters
     *         leaves the network open, since WPA2 cannot use a shorter key;
     *         so do more than 63, which it cannot use either.
     */
    const char *apPassword() {
        const char *p = _layout["network"]["ap_password"] | CONFIG_AP_PASS;
        return strlen(p) > 63 ? "" : p;
    }
    /** @} */

    /** @name Appearance
     *  @{ */
    /** @return Day-mode screen background, "#RRGGBB". */
    const char *dayBg()    { return _layout["global"]["day_bg"]   | "#101418"; }
    /** @return Night-mode screen background, "#RRGGBB". */
    const char *nightBg()  { return _layout["global"]["night_bg"] | "#000000"; }
    /** @} */

private:
    /**
     * @brief Parse `/layout.json` into @ref _layout.
     * @return false when the file is missing, malformed, or has no screens.
     */
    bool loadLayout();

    /** @brief Write the minimal built-in layout to LittleFS. */
    void writeDefaultLayout();

    /**
     * @brief Write the in-memory layout to flash.
     *
     * Via a temporary file renamed over the old one, so a power cut midway
     * leaves the previous layout — and the touch calibration stored in it —
     * intact, instead of a truncated file that boots to the built-in default.
     * @return false when nothing was written.
     */
    bool persistLayout();


    /**
     * @brief Finish an atomic write: check its length, then rename it in.
     * @param written  Bytes the temporary file received.
     * @param expected Bytes there should have been.
     * @return false when the old file was kept.
     */
    bool commitLayoutTmp(size_t written, size_t expected);

    /**
     * @brief Parse one layout file into the document.
     * @param path File to read.
     * @return true when it parsed and has at least one screen.
     */
    bool loadLayoutFrom(const char *path);

    /**
     * @brief Bring a layout written by older firmware up to date.
     *
     * Version 2 adds the warning lamps and the dash screen. Done once: the
     * version is bumped, so a dash screen deleted in the studio stays deleted.
     * @return true when the layout changed and should be saved.
     */
    bool migrateLayout();
    /** @brief Version 1 to 2: the dash screen and the warnings block. */
    void migrateToV2();

    /** @brief Register every HTTP route of the Gauge Studio API. */
    void setupRoutes();

    /**
     * @brief Apply a layout the studio uploaded, from the flash, and keep it.
     *
     * The upload is written to @p path as it arrives (layoutChunk) and parsed
     * from there into the place of the layout in memory, which is let go
     * first. Held in memory - the server keeps a request's body twice while
     * reading it, the handler took a third copy, and the parsed layout was
     * copied once more beside the one in use - a whole layout did not fit in
     * what the ESP32-C3 has left with its access point up, nor in the
     * cluster's internal RAM beside its draw buffers: "Save failed". The
     * device's touch calibration stands over the upload's; a layout that will
     * not parse, has no screens, or will not fit on the flash changes nothing,
     * and the stored one is put back in memory.
     * @param path Where the upload is; removed either way.
     * @param why  Filled with the reason when it returns false.
     * @param code Filled with the HTTP status for that reason.
     * @return true when the layout is applied and stored.
     */
    bool applyLayoutFile(const char *path, char *why, size_t whyLen, int &code);

    /** @brief One piece of a layout upload, to the flash (POST /api/layout). */
    void layoutChunk(uint8_t phase, const uint8_t *buf, size_t n);

    /** @brief The stored layout back in memory, after an upload that failed. */
    void restoreStoredLayout();

    /**
     * @brief Rename a complete file over the layout: atomically where LittleFS
     *        allows, else the old one is removed first.
     * @return false when the old layout was kept.
     */
    bool commitLayoutFrom(const char *src);

    /**
     * @brief Multipart upload handler for `POST /api/asset`.
     *
     * Reduces the client filename to a basename and strips ".." before
     * writing, so an upload cannot escape `/assets`.
     */
    void handleAssetUpload();

    /** @brief Send the canonical `{"ok":true}` success response. */
    void sendJsonOk();

    JsonDocument _layout;                /**< Parsed layout.json.           */
    bool         _configMode = false;    /**< Portal is running.            */
    String       _apIP;                  /**< AP address, for the UI.       */

    WebServer _server{80};   /**< Gauge Studio HTTP server.                 */
    DNSServer _dns;          /**< Captive-portal DNS; answers everything.   */
    File      _uploadFile;   /**< Destination of an in-flight asset upload. */
    File      _layoutUp;     /**< Destination of an in-flight layout upload. */
    size_t    _layoutUpBytes = 0;   /**< ...its bytes so far.                */
    /** Where a layout upload stands: see layoutChunk(). */
    enum : uint8_t { UP_NONE, UP_WRITING, UP_STORED, UP_TOO_BIG, UP_FAILED } _layoutUpState = UP_NONE;
    String    _uploadPath;   /**< ...and its path, to remove a partial one.  */
    /** Why the last upload failed, for its reply; nullptr when it did not.
     *  "no file" until an upload starts: a POST without one is refused. */
    const char *_uploadErr = "no file";
};

/** @brief Global configuration singleton. */
extern ConfigManager Config;

/** @} */  // end of config group
