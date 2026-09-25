/**
 * @file main.cpp
 * @brief ESP32-C3 Round Telemetry Display — Slave Node entry point.
 *
 * Boot sequence:
 *   1. ConfigManager  — mount LittleFS, load layout.json
 *   2. DisplayManager — GC9A01 + LVGL up
 *   3. TouchManager   — CST816S + gesture callbacks
 *   4. UIBuilder      — build screens from the layout
 *   5. NetworkManager — ESP-NOW listener on the telemetry channel
 *   6. Startup needle sweep
 *
 * Runtime contexts:
 *   - Arduino loop : LVGL rendering, touch, web portal (config mode)
 *   - Wi-Fi task   : ESP-NOW receive → TelemetryStore (lock-protected)
 *
 * Controls:
 *   swipe left/right — change screen        swipe up   — day/night override
 *   tap              — toggle peak values   swipe down — reset peaks
 *   5 s touch hold   — enter config AP      5 s hold (in config) — reboot
 *   BOOT button tap  — next screen          BOOT button 2.5 s — config AP
 */
#include <Arduino.h>
#include <lvgl.h>

#include "HardwareConfig.h"
#include "ButtonManager.h"
#include "ConfigManager.h"
#include "DisplayManager.h"
#include "NetworkManager.h"
#include "TelemetryStore.h"
#include "TouchManager.h"
#include "UIBuilder.h"

/* ---- gesture wiring ------------------------------------------------------ */

/** @brief Swipe left / button tap — advance one screen. */
static void onSwipeLeft()  { UI.nextScreen(); }
/** @brief Swipe right — go back one screen. */
static void onSwipeRight() { UI.prevScreen(); }
/** @brief Swipe up — cycle the day/night override. */
static void onSwipeUp()    { UI.cycleNightOverride(); }
/** @brief Swipe down — reset every peak value. */
static void onSwipeDown()  { UI.resetPeaksWithToast(); }

/**
 * @brief Tap handler.
 *
 * While an alert is up a tap acknowledges it; otherwise it toggles the peak
 * readouts. Acknowledging takes priority so the gesture always means the most
 * urgent thing available.
 */
static void onTap() {
    if (!UI.acknowledgeAlert()) UI.togglePeaks();
}

/** @brief Calibration capture — hand the measured drag to the wizard. */
static void onRawDrag(int16_t dx, int16_t dy) { UI.onCalibrationDrag(dx, dy); }

/**
 * @brief Announce a GPIO that discovery saw change level.
 *
 * Surfaced as a toast rather than buried on the diagnostics screen, because
 * the whole point is that the user is pressing a button and needs to know
 * *immediately* whether anything responded — from whatever screen they happen
 * to be on.
 *
 * @param gpio The pin that deviated from its resting level.
 */
static void onButtonDiscovered(int gpio) {
    char msg[32];
    snprintf(msg, sizeof(msg), "BUTTON = GPIO %d", gpio);
    UI.showToast(msg);
}

/**
 * @brief Long-press handler — enter the config portal, or reboot out of it.
 *
 * Entering config mode hands the radio over: the ESP-NOW listener must stop
 * before the AP can come up, since the two cannot share the interface.
 */
static void onHold() {
    if (!Config.inConfigMode()) {
        // Radio handover: firehose listener out, Gauge Studio AP in.
        Net.stop();
        Config.enterConfigMode();
        UI.showConfigScreen(Config.apSsid(), Config.apIP().c_str());
    } else {
        ESP.restart();          // second long-hold applies config & reboots
    }
}

/* ---- lifecycle ----------------------------------------------------------- */

/**
 * @brief One-time bring-up.
 *
 * Order matters: the filesystem must be mounted before the UI can read the
 * layout, and the display must exist before UIBuilder can create objects on
 * it. The ESP-NOW listener starts last so no packets arrive before there is
 * somewhere to render them.
 */
void setup() {
    Serial.begin(115200);
    log_i("Telemetry Display v%s booting", FIRMWARE_VERSION);

    Config.begin();
    Telemetry.begin(Config.emaAlpha(), Config.staleMs());

    Display.begin();

    TouchManager::Callbacks cbs;
    cbs.onSwipeLeft  = onSwipeLeft;
    cbs.onSwipeRight = onSwipeRight;
    cbs.onSwipeUp    = onSwipeUp;
    cbs.onSwipeDown  = onSwipeDown;
    cbs.onTap        = onTap;
    cbs.onHold       = onHold;
    cbs.onRawDrag    = onRawDrag;
    Touch.begin(cbs, Config.touchCal(), Config.touchSwapXY(),
                Config.touchInvertX(), Config.touchInvertY(),
                Config.touchNativeW(), Config.touchNativeH());

    // Side BOOT button mirrors the two most useful touch actions, so the
    // display stays usable with gloves on.
    ButtonManager::Callbacks btn;
    btn.onShortPress = onSwipeLeft;
    btn.onLongPress  = onHold;
    btn.onDiscovery  = onButtonDiscovered;
    Button.begin(Config.buttonGpio(), Config.buttonDiscovery(), btn);

    UI.begin();
    Net.begin();

    /*
     * With no measured mapping there is no way to know which way a swipe
     * goes, so ask once rather than guessing and leaving one direction dead.
     * It costs two swipes and is never asked again.
     */
    if (!Config.touchCal().valid) UI.startTouchCalibration();
    else                          UI.startupSweep();

    log_i("Boot complete — free heap %u KB", ESP.getFreeHeap() / 1024);
}

/**
 * @brief Main loop: pump LVGL, poll the button, service the portal.
 *
 * Telemetry is not polled here — packets arrive on the Wi-Fi task and land in
 * the TelemetryStore, which the UI timer reads.
 */
void loop() {
    // lv_timer_handler returns the time until it next has work to do. Sleeping
    // exactly that long (capped) instead of a fixed 5 ms keeps animation frames
    // punctual when LVGL is busy, and still yields generously when it is idle.
    const uint32_t idleMs = Display.update();

    Button.poll();          // debounced BOOT button
    Config.loop();          // DNS + HTTP, only active in config mode

    delay(constrain(idleMs, (uint32_t)1, (uint32_t)10));
}
