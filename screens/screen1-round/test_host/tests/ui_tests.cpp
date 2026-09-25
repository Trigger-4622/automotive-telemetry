/*
 * Scenarios for the display firmware: the real UIBuilder, Telltales,
 * TelemetryStore and ConfigManager, on LVGL, on the PC.
 *
 * Each scenario runs in its own process (run.py), so every one starts from a
 * fresh LVGL, filesystem and clock.
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <lvgl.h>

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "DNSServer.h"
#include "FS.h"
#include "LittleFS.h"
#include "WebServer.h"
#include "host.h"

// White-box access to the singletons' state, for checks only.
#define private public
#define protected public
#include "ConfigManager.h"
#include "TelemetryStore.h"
#include "Telltales.h"
#include "UIBuilder.h"
#undef private
#undef protected

#include "HardwareConfig.h"
#include "MasterPacket.h"
#include "Palette.h"

using host::car;
using host::run;

static const bool ROUND = LCD_WIDTH == LCD_HEIGHT;

/* ─────────────────────────────── checks ────────────────────────────────── */

static std::vector<std::string> g_fail;
#define CHECK(c, ...)                                                          \
    do {                                                                       \
        if (!(c)) {                                                            \
            char _b[400];                                                      \
            snprintf(_b, sizeof _b, __VA_ARGS__);                              \
            g_fail.push_back("line " + std::to_string(__LINE__) + ": " + _b);  \
        }                                                                      \
    } while (0)

static std::string env(const char *k, const char *d = "") {
    const char *v = getenv(k);
    return v ? v : d;
}
static std::string readFile(const std::string &path) {
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) { printf("cannot read %s\n", path.c_str()); exit(3); }
    std::string s;
    char b[4096];
    size_t n;
    while ((n = fread(b, 1, sizeof b, f)) > 0) s.append(b, n);
    fclose(f);
    return s;
}
static void shot(const std::string &name) {
    host::screenshot(env("SHOT_DIR", ".") + "/" + name + ".png");
}

/** Print and return the LVGL heap: measured here, estimated for the device. */
static host::HeapReport heapLine(const char *when) {
    const host::HeapReport h = host::heapReport();
    printf("    LVGL heap %s: device ~%zu B used, ~%zu B peak of %zu (%zu objects); "
           "PC %zu used, model %zu (%.0f%% seen)\n",
           when, h.targetUsed, h.targetPeak, h.budget, h.objects, h.pcUsed, h.pcModel,
           h.pcUsed ? 100.0 * (h.pcModel + h.baseline64) / h.pcUsed : 0.0);
    return h;
}

/* ─────────────────────────────── setup ─────────────────────────────────── */

/** The project's shipped layout, as a document to adjust. */
static JsonDocument shippedLayout() {
    JsonDocument d;
    deserializeJson(d, readFile(env("PROJECT_DIR") + "/data/layout.json"));
    // A captured touch calibration, as a configured device has: without one
    // the cluster boots into the calibration wizard.
    JsonObject tc = d["global"]["touch_cal"].to<JsonObject>();
    tc["valid"] = true;
    return d;
}

/** Boot the firmware the way main.cpp does, from this layout. */
static void boot(JsonDocument &layout) {
    std::string text;
    serializeJson(layout, text);
    hostfs::files["/layout.json"] = text;
    host::lvglInit();
    Config.begin();
    Telemetry.begin(Config.emaAlpha(), Config.staleMs());
    UI.begin();
    UI.startupSweep();
    run(2500);                            // sweep over, first frames drawn
}
static void boot() { JsonDocument d = shippedLayout(); boot(d); }

static int screenOf(const char *type) {
    JsonArray s = Config.layout()["screens"].as<JsonArray>();
    for (size_t i = 0; i < s.size(); i++)
        if (!strcmp(s[i]["type"] | "", type)) return (int)i;
    return -1;
}
static void gotoScreen(int idx) {
    for (int guard = 0; guard < 20 && (int)UI._active != idx; guard++) {
        UI.nextScreen();
        run(400);
    }
}
static void gotoType(const char *type) { gotoScreen(screenOf(type)); }
/** A screen that is neither the dash nor diagnostics, for the strip. */
static void gotoPlain() {
    JsonArray s = Config.layout()["screens"].as<JsonArray>();
    for (size_t i = 0; i < s.size(); i++) {
        const char *t = s[i]["type"] | "";
        if (strcmp(t, "dash") && strcmp(t, "diag")) { gotoScreen((int)i); return; }
    }
}

static const Lamp &lamp(const char *key) {
    const int i = UI._lamps.find(key);
    if (i < 0) { printf("no lamp %s\n", key); exit(3); }
    return UI._lamps[(size_t)i];
}
static bool alertShown() { return UI._alertShown; }
static std::string alertName() { return lv_label_get_text(UI._alertName); }
[[maybe_unused]] static std::string alertValue() { return lv_label_get_text(UI._alertValue); }
[[maybe_unused]] static std::string alertDetail() { return lv_label_get_text(UI._alertDetail); }
static bool alertIsLamp(const char *key) {
    return UI._alertShown && UI._alertKey == (ALERT_LAMP_KEY | (uint32_t)UI._lamps.find(key));
}
static bool hidden(lv_obj_t *o) { return lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN); }

static void engineOn() { car.set(METRIC_ID_RPM, 850); car.set(METRIC_ID_SPEED, 0); }

/* ───────────────────────────── scenarios ───────────────────────────────── */

static std::map<std::string, std::function<void()>> &registry() {
    static std::map<std::string, std::function<void()>> r;
    return r;
}
struct Reg { Reg(const char *n, std::function<void()> f) { registry()[n] = f; } };
#define SCENARIO(name) \
    static void sc_##name(); static Reg reg_##name(#name, sc_##name); static void sc_##name()

/* Boot on the shipped layout, visit every screen, and watch LVGL's heap:
 * running out at boot is an assert, and the display simply hangs. */
SCENARIO(boot_all_screens) {
    engineOn();
    car.set(METRIC_ID_COOLANT_TEMP, 88);
    car.set(METRIC_ID_FUEL_LEVEL, 63);
    car.set(METRIC_ID_BATT_VOLTAGE, 14.1f);
    car.set(METRIC_ID_GEAR, 2);
    boot();
    const host::HeapReport h = heapLine("after boot");
    // Headroom for what comes and goes at run time: toasts, the colour and
    // calibration screens, label text growing.
    CHECK(h.targetUsed + 8192 <= h.budget,
          "device LVGL heap estimate %zu of %zu bytes: under 8 KB left", h.targetUsed, h.budget);
    CHECK(h.targetPeak + 4096 <= h.budget,
          "device LVGL peak estimate %zu of %zu bytes", h.targetPeak, h.budget);

    // Where it goes, per screen, on the device.
    for (size_t i = 0; i < UI._screens.size(); i++)
        printf("      screen %zu %-12s %6zu B\n", i,
               Config.layout()["screens"][i]["type"] | "?",
               host::deviceBytes(UI._screens[i].scr));
    printf("      lamp strip        %6zu B\n      alert card        %6zu B\n",
           host::deviceBytes(UI._strip), host::deviceBytes(UI._alertCard));

    const size_t n = UI._screens.size();
    for (size_t i = 0; i < n; i++) {
        char nm[32];
        snprintf(nm, sizeof nm, "screen_%zu_%s", i,
                 Config.layout()["screens"][i]["type"] | "?");
        shot(nm);
        UI.nextScreen();
        run(600);
    }
    const host::HeapReport h2 = heapLine("after touring every screen");
    CHECK(h2.pcUsed <= h.pcUsed + 2048, "LVGL heap grew by %zu bytes touring screens",
          h2.pcUsed - h.pcUsed);
    for (auto &l : host::logs) CHECK(l[0] != 'E', "error logged: %s", l.c_str());
}

/* An older layout gains the dash and the warnings block once, and never
 * again: deleting the dash in the studio must stick. */
SCENARIO(migrate_v1_layout) {
    JsonDocument d = shippedLayout();
    d["version"] = 1;
    d.remove("warnings");
    JsonArray s = d["screens"].as<JsonArray>();
    for (size_t i = 0; i < s.size(); i++)
        if (!strcmp(s[i]["type"] | "", "dash")) { s.remove(i); break; }
    const size_t before = s.size();
    boot(d);

    CHECK((Config.layout()["version"] | 0) == 2, "version not bumped");
    CHECK(Config.layout()["screens"].size() == before + 1, "dash not added");
    CHECK(!strcmp(Config.layout()["screens"][0]["type"] | "", "dash"), "dash not first");
    CHECK(Config.layout()["warnings"].is<JsonObject>(), "warnings block not added");
    CHECK(UI._screens[0].dash == 0, "dash screen not built");
    // Saved, so the next boot sees version 2 already.
    JsonDocument saved;
    deserializeJson(saved, hostfs::files["/layout.json"]);
    CHECK((saved["version"] | 0) == 2, "migration not saved");
    CHECK(!hostfs::files.count("/layout.tmp"), "temporary file left behind");

    // The user deletes the dash; a reboot must not bring it back.
    JsonArray ss = Config.layout()["screens"].as<JsonArray>();
    ss.remove(0);
    Config.persistLayout();
    Config.begin();
    CHECK(Config.layout()["screens"].size() == before, "dash came back after being deleted");
    // A second pass over an up-to-date layout changes nothing.
    CHECK(!Config.migrateLayout(), "migration ran twice");

    // Every screen of the migrated layout still has its type.
    for (JsonObject sc : Config.layout()["screens"].as<JsonArray>())
        CHECK(strlen(sc["type"] | "") > 0, "a screen lost its type in the migration");
}

/* The built-in default (no layout.json at all) boots, and has the dash. */
SCENARIO(default_layout) {
    host::lvglInit();
    Config.begin();                       // empty filesystem
    Telemetry.begin(Config.emaAlpha(), Config.staleMs());
    UI.begin();
    run(500);
    CHECK(!strcmp(Config.layout()["screens"][0]["type"] | "", "dash"), "default has no dash");
    CHECK((Config.layout()["version"] | 0) == 2, "default is not version 2");
    CHECK(UI._screens.size() == 3, "default built %zu screens", UI._screens.size());
    shot("default_dash");
}

/* Saving a layout: a short write keeps the old file, a layer that will not
 * rename over a file still saves, and an interrupted save is recovered. */
SCENARIO(layout_persistence) {
    boot();
    Config.enterConfigMode();
    const std::string good = hostfs::files["/layout.json"];
    JsonDocument d = shippedLayout();
    d["global"]["alert_ack_ms"] = 12345;
    std::string body;
    serializeJson(d, body);

    hostfs::writeBudget = 100;            // flash full part-way through
    Config._server.request(HTTP_POST, "/api/layout", body);
    hostfs::writeBudget = -1;
    CHECK(Config._server.code == 500, "short write reported %d", Config._server.code);
    CHECK(hostfs::files["/layout.json"] == good, "short write damaged layout.json");
    CHECK(!hostfs::files.count("/layout.tmp"), "short write left a temporary file");

    hostfs::renameNoOverwrite = true;
    Config._server.request(HTTP_POST, "/api/layout", body);
    hostfs::renameNoOverwrite = false;
    CHECK(Config._server.code == 200, "save with no-overwrite rename failed");
    CHECK(hostfs::files["/layout.json"] == body, "saved text differs");

    Config._server.request(HTTP_POST, "/api/layout", "{\"screens\":5}");
    CHECK(Config._server.code == 400, "invalid layout accepted");
    CHECK(hostfs::files["/layout.json"] == body, "invalid layout written");

    // Power cut between writing the new file and renaming it in.
    hostfs::files["/layout.tmp"] = body;
    hostfs::files["/layout.json"] = body.substr(0, 40);
    Config.begin();
    CHECK((Config.layout()["global"]["alert_ack_ms"] | 0) == 12345,
          "interrupted save not recovered");
    CHECK(hostfs::files["/layout.json"].size() > 100, "recovered layout not rewritten");

    // The studio page is served from the firmware, gzipped.
    Config._server.request(HTTP_GET, "/");
    CHECK(Config._server.code == 200, "studio page code %d", Config._server.code);
    CHECK(Config._server.headers["Content-Encoding"] == "gzip", "studio not gzip");
    CHECK(Config._server.body.size() > 1000 &&
          (uint8_t)Config._server.body[0] == 0x1F && (uint8_t)Config._server.body[1] == 0x8B,
          "studio body is not gzip data");
    CHECK(UI._alertShown == false && UI._stripShown == false,
          "overlays left over the config screen");
}

/* Seat belt: shown standing still, pops up and flashes once moving, a tap
 * mutes it for alert_ack_ms, and it nags again after. */
SCENARIO(lamp_seatbelt) {
    engineOn();
    boot();
    gotoPlain();
    car.set(METRIC_ID_SEATBELT, 1);
    run(500);
    CHECK(lamp("seatbelt").on, "belt lamp not on");
    CHECK(UI._stripShown, "strip not shown for a lit lamp");
    CHECK(!alertShown(), "popup at a standstill");
    bool dark = false;
    for (int i = 0; i < 40; i++) { run(25); dark |= !lamp("seatbelt").lit; }
    CHECK(!dark, "belt lamp flashes while parked");

    car.set(METRIC_ID_SPEED, 30);
    run(500);
    CHECK(alertIsLamp("seatbelt"), "no belt popup at 30 km/h");
    CHECK(alertName() == "FASTEN SEAT BELT", "popup title '%s'", alertName().c_str());
    shot("alert_seatbelt");
    int flips = 0;
    bool prev = lamp("seatbelt").lit;
    for (int i = 0; i < 80; i++) {
        run(25);
        if (lamp("seatbelt").lit != prev) { flips++; prev = !prev; }
    }
    CHECK(flips >= 3, "belt lamp did not flash at speed (%d changes)", flips);

    CHECK(UI.acknowledgeAlert(), "tap not taken by the popup");
    run(500);
    CHECK(!alertShown(), "popup back straight after the tap");
    run(28000);
    CHECK(!alertShown(), "popup back before alert_ack_ms");
    run(3000);
    CHECK(alertIsLamp("seatbelt"), "belt popup did not return after alert_ack_ms");

    // Slowing below the gate clears the popup, but keeps the lamp.
    car.set(METRIC_ID_SPEED, 3);
    run(500);
    CHECK(!alertShown(), "popup stays at 3 km/h");
    CHECK(lamp("seatbelt").on, "lamp went out with the speed");

    car.set(METRIC_ID_SEATBELT, 0);
    run(300);
    CHECK(!lamp("seatbelt").on && !UI._stripShown, "belt fastened but lamp still on");
}

/* Check engine: pops up once with the code count, hides by itself, and only
 * returns when the lamp goes out and lights again. */
SCENARIO(lamp_mil_once) {
    engineOn();
    boot();
    gotoPlain();
    car.set(METRIC_ID_MIL, 1);
    car.set(METRIC_ID_DTC_COUNT, 2);
    run(500);
    CHECK(alertIsLamp("mil"), "no MIL popup");
    CHECK(std::string(UI._alertLastValue) == "2 CODES" ||
          std::string(UI._alertLastDetail) == "2 CODES",
          "MIL value '%s' / '%s'", UI._alertLastValue, UI._alertLastDetail);
    shot("alert_mil");
    run(10500);
    CHECK(!alertShown(), "MIL popup did not hide after popup_secs");
    run(40000);
    CHECK(!alertShown(), "MIL popup came back while still lit");
    CHECK(lamp("mil").on, "MIL lamp went out");
    car.set(METRIC_ID_MIL, 0);
    run(500);
    car.set(METRIC_ID_MIL, 1);
    run(500);
    CHECK(alertIsLamp("mil"), "MIL popup did not return on a new lighting");
}

/* Oil pressure is only judged with the engine running a while. */
SCENARIO(lamp_engine_gate) {
    car.set(METRIC_ID_RPM, 0);
    car.set(METRIC_ID_SPEED, 0);
    car.set(METRIC_ID_OIL_PRESSURE, 0.1f);
    car.set(METRIC_ID_BATT_VOLTAGE, 11.0f);
    // The shipped watchdog also guards oil pressure and battery, and would
    // (rightly) take the card; here only the lamps are under test.
    JsonDocument d = shippedLayout();
    d["watchdog"]["enabled"] = false;
    boot(d);
    gotoPlain();
    run(3000);
    CHECK(!lamp("oil").on, "oil lamp on with the engine off");
    CHECK(!lamp("charging").on, "charging lamp on with the engine off");
    car.set(METRIC_ID_RPM, 900);
    run(2000);
    CHECK(!lamp("oil").on, "oil lamp on 2 s after start");
    run(2500);
    CHECK(lamp("oil").on, "oil lamp off with no pressure and the engine running");
    CHECK(alertIsLamp("oil"), "no oil popup");
    CHECK(UI._alertLevel == AlarmState::Critical, "oil popup not critical");
    car.set(METRIC_ID_RPM, 0);
    run(300);
    CHECK(!lamp("oil").on, "oil lamp stays on after the engine stops");
    // RPM that disappears is not a running engine.
    car.set(METRIC_ID_RPM, 900);
    run(5000);
    CHECK(lamp("oil").on, "oil lamp not back");
    car.drop(METRIC_ID_RPM);
    run(2000);
    CHECK(!lamp("oil").on, "oil lamp on with RPM gone stale");
}

/* Indicators: a steady bit flashes by itself; a bus that reports the flasher
 * is followed; hazards light both. */
SCENARIO(lamp_turn_signals) {
    engineOn();
    boot();
    gotoPlain();
    car.set(METRIC_ID_TURN_LEFT, 1);
    car.send();
    run(300);
    CHECK(lamp("turn_left").lit, "indicator dark at first");
    std::vector<bool> seq;
    for (int i = 0; i < 100; i++) { run(25); seq.push_back(lamp("turn_left").lit); }
    int flips = 0;
    for (size_t i = 1; i < seq.size(); i++) flips += seq[i] != seq[i - 1];
    CHECK(flips >= 5 && flips <= 9, "steady indicator made %d changes in 2.5 s", flips);
    CHECK(lamp("turn_left").on, "indicator not 'on' through its dark phase");
    CHECK(UI._stripShown, "strip closed during a flash");

    // A bus that reports the flasher itself, 400 ms on / 400 ms off.
    car.set(METRIC_ID_TURN_LEFT, 0);
    run(1500);
    int agree = 0, total = 0;
    for (int k = 0; k < 6; k++) {
        car.set(METRIC_ID_TURN_LEFT, 1); car.send();
        for (int i = 0; i < 16; i++) { run(25); agree += lamp("turn_left").lit; total++; }
        car.set(METRIC_ID_TURN_LEFT, 0); car.send();
        for (int i = 0; i < 16; i++) { run(25); agree += !lamp("turn_left").lit; total++; }
    }
    CHECK(agree * 100 / total >= 85, "lamp followed the flasher only %d%% of the time",
          agree * 100 / total);

    car.set(METRIC_ID_TURN_LEFT, 1);
    car.set(METRIC_ID_TURN_RIGHT, 1);
    car.send();
    int mismatch = 0;
    for (int i = 0; i < 80; i++) {
        run(25);
        mismatch += lamp("turn_left").lit != lamp("turn_right").lit;
    }
    CHECK(mismatch <= 2, "hazard lamps out of step %d times", mismatch);
    shot("strip_hazard");
}

/* A lamp whose signal stops arriving goes dark, popup and all. */
SCENARIO(lamp_stale) {
    engineOn();
    car.set(METRIC_ID_SPEED, 20);
    car.set(METRIC_ID_DOOR_OPEN, 1);
    boot();
    gotoPlain();
    run(500);
    CHECK(alertIsLamp("door"), "no door popup while moving");
    CHECK(UI._alertLevel == AlarmState::Critical, "door popup not critical");
    car.drop(METRIC_ID_DOOR_OPEN);
    run(2500);
    CHECK(!lamp("door").on, "door lamp on with no data");
    CHECK(!alertShown(), "door popup survived the data going stale");
    car.radio = false;
    run(3000);
    CHECK(!UI._stripShown, "strip up with the link down");
}

/* Warnings switched off: nothing lights, nothing pops up, the dash shows
 * only ghosts. */
SCENARIO(switch_all_off) {
    engineOn();
    car.set(METRIC_ID_SPEED, 40);
    car.set(METRIC_ID_SEATBELT, 1);
    car.set(METRIC_ID_MIL, 1);
    JsonDocument d = shippedLayout();
    d["warnings"]["enabled"] = false;
    boot(d);
    gotoPlain();
    run(1000);
    CHECK(!UI._stripShown, "strip shown with warnings off");
    CHECK(!alertShown(), "popup with warnings off");
    for (size_t i = 0; i < UI._lamps.size(); i++)
        CHECK(!UI._lamps[i].on, "lamp %s on with warnings off", UI._lamps[i].key);
    gotoType("dash");
    run(300);
    for (auto &dv : UI._dash)
        for (size_t k = 0; k < dv.slots; k++)
            CHECK(dv.slotRgb[k] == UI_LAMP_OFF || dv.slotRgb[k] == 0xFFFFFFFFu,
                  "dash slot %zu lit with warnings off", k);
}

SCENARIO(switch_popups_off) {
    engineOn();
    car.set(METRIC_ID_SPEED, 40);
    car.set(METRIC_ID_SEATBELT, 1);
    JsonDocument d = shippedLayout();
    d["warnings"]["popups"] = false;
    boot(d);
    gotoPlain();
    run(1000);
    CHECK(lamp("seatbelt").on && UI._stripShown, "lamp not shown");
    CHECK(!alertShown(), "popup with warnings.popups off");
}

SCENARIO(switch_single_lamp_and_strip) {
    engineOn();
    car.set(METRIC_ID_SPEED, 40);
    car.set(METRIC_ID_SEATBELT, 1);
    car.set(METRIC_ID_HIGH_BEAM, 1);
    JsonDocument d = shippedLayout();
    d["warnings"]["lamps"]["seatbelt"] = false;
    d["warnings"]["strip"] = false;
    boot(d);
    gotoPlain();
    run(1000);
    CHECK(!lamp("seatbelt").on, "disabled lamp lit");
    CHECK(!alertShown(), "disabled lamp popped up");
    CHECK(lamp("high_beam").on, "other lamp affected");
    CHECK(!UI._stripShown, "strip shown with strip off");
}

SCENARIO(switch_per_screen) {
    engineOn();
    car.set(METRIC_ID_SPEED, 40);
    car.set(METRIC_ID_SEATBELT, 1);
    JsonDocument d = shippedLayout();
    JsonArray s = d["screens"].as<JsonArray>();
    int plain = -1;
    for (size_t i = 0; i < s.size(); i++) {
        const char *t = s[i]["type"] | "";
        if (strcmp(t, "dash") && strcmp(t, "diag")) { plain = (int)i; break; }
    }
    s[plain]["show_telltales"] = false;
    s[plain]["show_popup"] = false;
    boot(d);
    gotoScreen(plain);
    run(800);
    CHECK(!UI._stripShown, "strip on a show_telltales:false screen");
    CHECK(!alertShown(), "popup on a show_popup:false screen");
    gotoType("dash");
    run(300);
    CHECK(!UI._stripShown, "strip over the dash");
    CHECK(alertShown(), "popup missing on the dash, which allows it");
}

/* Every dash part switched off: the screen still builds, empty. */
SCENARIO(dash_parts_off) {
    JsonDocument d = shippedLayout();
    JsonObject dash = d["screens"][0];
    for (const char *p : {"tach", "speed", "gear", "fuel", "temp", "info",
                          "shift_light", "lamps"})
        dash[p] = false;
    boot(d);
    CHECK(UI._screens[0].bindings.empty(), "dash built %zu bindings with every part off",
          UI._screens[0].bindings.size());
    CHECK(UI._dash[0].slots == 0 && !UI._dash[0].shift, "dash extras built while off");
    run(500);
    shot("dash_all_off");
}

/* Watchdog and lamps on the same metric: one card, the more urgent one. */
SCENARIO(watchdog_lamp_priority) {
    engineOn();
    car.set(METRIC_ID_COOLANT_TEMP, 90);
    boot();
    gotoPlain();
    car.set(METRIC_ID_COOLANT_TEMP, 112);
    run(3000);
    // Watchdog: warning above 105. Lamp: critical above 110. The critical
    // lamp must not be hidden behind the milder watchdog warning.
    CHECK(alertShown(), "nothing shown at 112 degrees");
    CHECK(UI._alertLevel == AlarmState::Critical,
          "112 degrees shown at warning level only (key 0x%X)", UI._alertKey);
    car.set(METRIC_ID_COOLANT_TEMP, 116);
    run(1000);
    CHECK(UI._alertKey == METRIC_ID_COOLANT_TEMP && UI._alertLevel == AlarmState::Critical,
          "watchdog critical did not take the card (key 0x%X)", UI._alertKey);
    shot("alert_watchdog_coolant");
}

/* A muted watchdog fault steps aside for a second one. */
SCENARIO(ack_second_fault) {
    engineOn();
    car.set(METRIC_ID_BATT_VOLTAGE, 13.9f);
    boot();
    gotoPlain();
    car.set(METRIC_ID_BATT_VOLTAGE, 11.9f);     // watchdog warning (< 12.1)
    run(600);
    CHECK(UI._alertKey == METRIC_ID_BATT_VOLTAGE, "battery not shown (0x%X)", UI._alertKey);
    UI.acknowledgeAlert();
    run(600);
    CHECK(!alertShown(), "muted battery still shown");
    car.set(METRIC_ID_OIL_TEMP, 133);           // second watchdog warning
    run(600);
    CHECK(UI._alertKey == METRIC_ID_OIL_TEMP, "second fault hidden by the mute (0x%X)",
          UI._alertKey);
    car.set(METRIC_ID_OIL_TEMP, 100);
    run(600);
    CHECK(!alertShown(), "muted battery came back before alert_ack_ms");
    // Recovery ends the mute: a fresh dip is a fresh fault.
    car.set(METRIC_ID_BATT_VOLTAGE, 13.9f);
    run(600);
    car.set(METRIC_ID_BATT_VOLTAGE, 11.9f);
    run(600);
    CHECK(UI._alertKey == METRIC_ID_BATT_VOLTAGE, "new battery fault muted by an old tap");
}

/* Gear letters, and the reverse switch winning over the gear value. */
SCENARIO(gear_display) {
    engineOn();
    car.set(METRIC_ID_GEAR, 3);
    boot();
    gotoType("dash");
    GaugeBinding *g = nullptr;
    for (auto &b : UI._screens[UI._active].bindings)
        if (b.fmt == ValueFmt::Gear) g = &b;
    CHECK(g != nullptr, "dash has no gear binding");
    if (!g) return;
    auto text = [&] { return std::string(lv_label_get_text(g->valueLabel)); };
    run(300);
    CHECK(text() == "3", "gear 3 shown as '%s'", text().c_str());
    car.set(METRIC_ID_GEAR, 0);  run(300);
    CHECK(text() == "N", "gear 0 shown as '%s'", text().c_str());
    car.set(METRIC_ID_GEAR, -1); run(300);
    CHECK(text() == "R", "gear -1 shown as '%s'", text().c_str());
    car.set(METRIC_ID_GEAR, 2);  run(40);
    CHECK(text() == "2", "gear glided instead of switching: '%s'", text().c_str());
    car.set(METRIC_ID_REVERSE, 1); run(300);
    CHECK(text() == "R", "reverse switch ignored: '%s'", text().c_str());
    car.drop(METRIC_ID_GEAR);  run(2500);
    CHECK(text() == "R", "reverse lost with no gear metric: '%s'", text().c_str());
    car.set(METRIC_ID_REVERSE, 0); run(300);
    CHECK(text() == "--", "no gear shown as '%s'", text().c_str());
}

/* One sample per metric per tick, however many screens bind it. */
SCENARIO(sample_once_per_tick) {
    engineOn();
    boot();
    run(100);
    std::map<uint16_t, int> seen;
    for (auto &c : UI._tickCache) seen[c.id]++;
    for (auto &kv : seen) CHECK(kv.second == 1, "metric 0x%04X sampled %d times", kv.first, kv.second);
    int bound = 0;
    for (auto &sd : UI._screens)
        for (auto &b : sd.bindings) bound += b.metricId == METRIC_ID_RPM;
    printf("    RPM is bound %d times, sampled once per tick\n", bound);
}

/* The first packet seeds the smoothing: no glide up from zero. */
SCENARIO(first_value_not_from_zero) {
    Telemetry.begin(0.35f, 1500);
    car.set(METRIC_ID_COOLANT_TEMP, 90);
    car.send();
    MetricSample s;
    CHECK(Telemetry.peek(METRIC_ID_COOLANT_TEMP, s), "metric missing");
    CHECK(fabsf(s.value - 90) < 0.01f, "first smoothed value %.2f, not 90", s.value);
}

/* The dash and the strip, drawn: engine at the shift point, lamps lit. */
SCENARIO(dash_pictures) {
    car.set(METRIC_ID_RPM, 850);
    car.set(METRIC_ID_SPEED, 0);
    car.set(METRIC_ID_COOLANT_TEMP, 38);
    car.set(METRIC_ID_FUEL_LEVEL, 64);
    car.set(METRIC_ID_GEAR, 0);
    car.set(METRIC_ID_BATT_VOLTAGE, 14.2f);
    car.set(METRIC_ID_OIL_TEMP, 71);
    car.set(METRIC_ID_IAT, 24);
    car.set(METRIC_ID_BOOST, -0.62f);
    car.set(METRIC_ID_SW_LIGHTS, 1);
    boot();
    run(1500);
    shot("dash_idle_cold");

    car.set(METRIC_ID_RPM, 5600);
    car.set(METRIC_ID_SPEED, 96);
    car.set(METRIC_ID_GEAR, 3);
    car.set(METRIC_ID_COOLANT_TEMP, 91);
    car.set(METRIC_ID_FUEL_LEVEL, 9);
    car.set(METRIC_ID_BOOST, 1.05f);
    car.set(METRIC_ID_MIL, 1);
    car.set(METRIC_ID_HIGH_BEAM, 1);
    car.set(METRIC_ID_TURN_RIGHT, 1);
    run(12000);                   // fuel debounce, MIL popup timed out
    shot("dash_driving_lamps");
    car.set(METRIC_ID_RPM, 6700);
    run(400);
    shot("dash_shift");
    CHECK(UI._dash[0].shift, "shift light missing");
    gotoPlain();
    run(300);
    shot("strip_over_screen");
    car.set(METRIC_ID_SEATBELT, 1);
    car.set(METRIC_ID_DOOR_OPEN, 1);
    car.set(METRIC_ID_HANDBRAKE, 1);
    run(600);
    shot("alert_door_over_screen");
    const host::HeapReport h = heapLine("lamps lit, card up");
    CHECK(h.targetPeak + 4096 <= h.budget, "device LVGL peak estimate %zu of %zu",
          h.targetPeak, h.budget);
}

/* Many lamps at once: the strip holds what fits and says how many more. */
SCENARIO(strip_overflow) {
    engineOn();
    car.set(METRIC_ID_SPEED, 50);
    for (uint16_t id : {METRIC_ID_MIL, METRIC_ID_SEATBELT, METRIC_ID_DOOR_OPEN,
                        METRIC_ID_HANDBRAKE, METRIC_ID_HIGH_BEAM, METRIC_ID_CRUISE_ON,
                        METRIC_ID_TURN_LEFT, METRIC_ID_TURN_RIGHT})
        car.set(id, 1);
    car.set(METRIC_ID_FUEL_LEVEL, 5);
    car.set(METRIC_ID_COOLANT_TEMP, 30);
    car.set(METRIC_ID_SSM_KNOCK_CORR, -4);
    boot();
    gotoPlain();
    run(11000);
    size_t on = 0;
    for (size_t i = 0; i < UI._lamps.size(); i++) on += UI._lamps[i].on && UI._lamps[i].strip;
    printf("    %zu lamps lit, strip shows %zu slots, +%d\n", on, UI._stripCount, UI._stripMoreN);
    CHECK(UI._stripCount <= STRIP_SLOTS, "strip overflowed its slots");
    if (on > STRIP_SLOTS) CHECK(UI._stripMoreN != -1, "no +N with %zu lamps", on);
    // The strip must stay on the glass.
    lv_area_t a;
    lv_obj_get_coords(UI._strip, &a);
    CHECK(a.x1 >= 0 && a.x2 < LCD_WIDTH && a.y1 >= 0 && a.y2 < LCD_HEIGHT,
          "strip off screen: %d,%d-%d,%d", a.x1, a.y1, a.x2, a.y2);
    if (ROUND) {
        // Every corner of the pill inside the round glass (radius 120).
        for (int x : {a.x1 + 8, a.x2 - 8})
            for (int y : {a.y1, a.y2}) {
                const float dx = x - 119.5f, dy = y - 119.5f;
                CHECK(dx * dx + dy * dy < 120.0f * 120.0f,
                      "strip corner %d,%d off the round glass", x, y);
            }
    }
    shot("strip_overflow");
}

/* Settings that make no sense must not crash or wedge a lamp. */
SCENARIO(warnings_config_fuzz) {
    const char *bad[] = {
        R"({"lamps":[1,2,3]})",
        R"({"lamps":{"mil":"yes","oil":{"threshold":"x","debounce_ms":-5,"hold_ms":1e12,
            "popup_secs":99999,"decimals":77,"tone":"pink","icon":"nope","rule":"sideways"}}})",
        R"({"lamps":{"custom_1":{"enabled":true,"metric_id":"0x1FFFF"}}})",
        R"({"lamps":{"custom_2":{"enabled":true,"metric_id":-4,"title":"A VERY LONG TITLE THAT DOES NOT FIT AT ALL"}}})",
        R"({"lamps":{"door":{"popup":true},"seatbelt":{"popup":false},"knock":true}})",
        R"(false)", R"(true)", R"(42)", R"("off")",
    };
    Telltales t;
    for (const char *b : bad) {
        JsonDocument d;
        deserializeJson(d, b);
        t.configure(d.as<JsonVariantConst>(), 30000);
        Telemetry.begin(0.35f, 1500);
        t.update(Telemetry, millis());
        for (size_t i = 0; i < t.size(); i++) {
            const Lamp &l = t[i];
            CHECK(l.decimals <= 3, "decimals %u from %s", l.decimals, b);
            CHECK(l.icon < ICON_COUNT, "icon %u from %s", l.icon, b);
            CHECK(strlen(l.title) < sizeof(l.title), "title overflow");
            CHECK(!l.on, "lamp %s on with no data (%s)", l.key, b);
        }
    }
    JsonDocument d;
    deserializeJson(d, R"({"lamps":{"custom_1":{"enabled":true,"metric_id":"0x1FFFF"},
        "oil":{"debounce_ms":-5,"hold_ms":1e12},"door":{"popup":true},
        "seatbelt":{"popup":false}}})");
    t.configure(d.as<JsonVariantConst>(), 30000);
    CHECK(t[t.find("custom_1")].metric == 0, "out-of-range metric id accepted");
    CHECK(t[t.find("oil")].debounceMs == 0 && t[t.find("oil")].holdMs == 60000,
          "durations not clamped: %u %u", t[t.find("oil")].debounceMs, t[t.find("oil")].holdMs);
    CHECK(t[t.find("door")].popup == LampPopup::Repeat, "popup:true changed the mode");
    CHECK(t[t.find("seatbelt")].popup == LampPopup::Off, "popup:false ignored");
}

/* 49.7 days in, millis() wraps. Snoozes and screen changes must not wedge. */
SCENARIO(millis_wrap) {
    engineOn();
    boot();
    gotoPlain();
    host::setMillis(0xFFFFFFFFu - 800);
    car.lastSend = 0;
    car.set(METRIC_ID_SPEED, 30);
    car.set(METRIC_ID_SEATBELT, 1);
    run(600);
    CHECK(alertIsLamp("seatbelt"), "no popup before the wrap");
    UI.acknowledgeAlert();
    UI.nextScreen();                      // a slide straddling the wrap
    run(15000);
    CHECK(millis() < 0x80000000u, "clock did not wrap");
    CHECK(!alertShown(), "snooze broken by the wrap");
    run(20000);
    CHECK(alertShown(), "popup never returned after the wrap");
    // The UI is still being updated at all (a stuck _busyUntilMs stops it).
    car.set(METRIC_ID_SEATBELT, 0);
    run(500);
    CHECK(!alertShown() && !lamp("seatbelt").on, "UI frozen after the wrap");
}

/* Night mode and a lamp card: colours still applied, nothing stranded. */
SCENARIO(config_mode_clears_overlays) {
    engineOn();
    car.set(METRIC_ID_SPEED, 30);
    car.set(METRIC_ID_DOOR_OPEN, 1);
    boot();
    gotoPlain();
    run(500);
    CHECK(alertShown() && UI._stripShown, "setup: nothing to clear");
    Config.enterConfigMode();
    UI.showConfigScreen(Config.apSsid(), Config.apIP().c_str());
    run(1000);
    CHECK(hidden(UI._alertCard) && hidden(UI._strip), "overlays left over the portal");
    shot("config_screen");
}

/* The card-style alert (the round board's only style; the cluster's option):
 * lamp symbol, title and advice, drawn. */
SCENARIO(alert_card_style) {
    engineOn();
    car.set(METRIC_ID_SPEED, 40);
    JsonDocument d = shippedLayout();
    d["watchdog"]["alert_style"] = "card";
    boot(d);
    gotoPlain();
    car.set(METRIC_ID_MIL, 1);
    car.set(METRIC_ID_DTC_COUNT, 3);
    run(500);
    CHECK(alertIsLamp("mil"), "no MIL card");
    CHECK(!hidden(UI._alertImg), "lamp symbol hidden on the card");
    const lv_img_dsc_t *src = (const lv_img_dsc_t *)lv_img_get_src(UI._alertImg);
    CHECK(src && src->header.w == 36,
          "card symbol is %d px", src ? (int)src->header.w : -1);
    CHECK(lv_img_get_zoom(UI._alertImg) == LV_IMG_ZOOM_NONE,
          "alpha symbol zoomed - LVGL 8.3 would draw nothing");
    shot("alert_card_mil");
    car.set(METRIC_ID_DOOR_OPEN, 1);
    run(500);
    CHECK(alertIsLamp("door"), "critical door did not take the card from MIL");
    shot("alert_card_door");
    // Back to a watchdog item: the triangle returns, the symbol goes.
    car.set(METRIC_ID_DOOR_OPEN, 0);
    car.set(METRIC_ID_MIL, 0);
    car.set(METRIC_ID_BOOST, 1.8f);
    run(500);
    CHECK(UI._alertKey == METRIC_ID_BOOST, "boost watchdog not shown (0x%X)", UI._alertKey);
    CHECK(hidden(UI._alertImg) && !hidden(UI._alertIcon), "card kept the lamp layout");
    shot("alert_card_boost");
}

/* The strip switched off builds nothing at all. */
SCENARIO(strip_off_saves_memory) {
    JsonDocument d = shippedLayout();
    d["warnings"]["strip"] = false;
    boot(d);
    CHECK(UI._strip == nullptr, "strip built while switched off");
    engineOn();
    car.set(METRIC_ID_MIL, 1);
    gotoPlain();
    run(500);                             // and nothing dereferences it
    CHECK(lamp("mil").on, "lamp logic affected by the strip being off");
}

/* The strip opens in full when a lamp comes on, then folds to the most
 * urgent fault; information lamps go, indicators hold it open. */
SCENARIO(strip_peek) {
    engineOn();
    JsonDocument d = shippedLayout();
    d["warnings"]["strip_mode"] = "peek";     // the round board's default
    boot(d);
    gotoPlain();
    auto visibleLamps = [] {
        size_t n = 0;
        for (size_t j = 0; j < UI._stripCount && j < STRIP_SLOTS; j++)
            n += !hidden(UI._stripImg[j]);
        return n;
    };
    CHECK(!UI._stripShown, "strip up with nothing lit");

    car.set(METRIC_ID_HIGH_BEAM, 1);
    run(500);
    CHECK(UI._stripShown && visibleLamps() == 1, "high beam not shown on lighting");
    run(7000);
    CHECK(!UI._stripShown, "information lamp still covering the screen after the peek");

    car.set(METRIC_ID_MIL, 1);
    run(500);
    CHECK(UI._stripShown && visibleLamps() == 2, "new lamp did not open the full strip (%zu)",
          visibleLamps());
    run(12000);
    CHECK(UI._stripShown && visibleLamps() == 1, "strip did not fold to the fault (%zu)",
          visibleLamps());
    CHECK(UI._stripIcon[0] == ICON_MIL, "folded badge is not the check-engine lamp");

    car.set(METRIC_ID_SEATBELT, 1);
    car.set(METRIC_ID_DOOR_OPEN, 1);
    run(500);
    CHECK(visibleLamps() == 4, "full strip shows %zu of 4 lamps", visibleLamps());
    shot("strip_peek_full");
    run(7000);
    CHECK(UI._stripCount == 2 && visibleLamps() == 1 && !hidden(UI._stripMore),
          "folded strip is not badge + '+N' (%zu slots)", UI._stripCount);
    CHECK(UI._stripIcon[0] == ICON_DOOR, "badge is not the most urgent lamp (door)");
    shot("strip_peek_folded");

    car.set(METRIC_ID_TURN_LEFT, 1);
    run(20000);
    CHECK(visibleLamps() >= 4, "indicator did not hold the full strip open");
    car.set(METRIC_ID_TURN_LEFT, 0);
    run(1500);
    CHECK(UI._stripCount == 2, "strip did not fold after the indicator (%zu)", UI._stripCount);

    lv_area_t a;
    lv_obj_get_coords(UI._strip, &a);
    CHECK(a.x1 >= 0 && a.x2 < LCD_WIDTH && a.y1 >= 0 && a.y2 < LCD_HEIGHT,
          "strip off screen: %d,%d-%d,%d", a.x1, a.y1, a.x2, a.y2);
}

SCENARIO(strip_always) {
    engineOn();
    JsonDocument d = shippedLayout();
    d["warnings"]["strip_mode"] = "always";
    boot(d);
    gotoPlain();
    car.set(METRIC_ID_HIGH_BEAM, 1);
    run(10000);
    CHECK(UI._stripShown, "strip_mode always folded away an information lamp");

    // With no strip_mode at all, the board's own default applies: always on
    // the cluster (its header band keeps room for the strip), peek on the
    // round glass (where the strip sits over content).
    Telltales t;
    JsonDocument e;
    deserializeJson(e, "{}");
    t.configure(e.as<JsonVariantConst>(), 30000);
    CHECK(t.stripFull() == (LCD_WIDTH != LCD_HEIGHT), "wrong strip default for this board");
}

#if LCD_WIDTH != LCD_HEIGHT
/* The touch calibration wizard (cluster only) owns the screen: no strip, no
 * popup over its instructions, and both back once it is done. */
SCENARIO(calibration_wizard_clear) {
    engineOn();
    car.set(METRIC_ID_SPEED, 30);
    car.set(METRIC_ID_DOOR_OPEN, 1);
    boot();
    gotoPlain();
    run(500);
    CHECK(alertShown() && UI._stripShown, "setup: door popup and strip expected");
    UI.startTouchCalibration();
    run(3000);
    CHECK(hidden(UI._alertCard) && hidden(UI._strip), "overlays drawn over the calibration wizard");
    UI.onCalibrationDrag(300, 0);
    UI.onCalibrationDrag(0, 200);
    run(2500);
    CHECK(alertShown(), "door popup did not come back after calibration");
}
#endif

/* Every screen with lamps lit, by day and by night: the strip must not hide
 * anything that matters on any of them. Pictures only; judged by eye. */
SCENARIO(tour_with_lamps) {
    car.set(METRIC_ID_RPM, 2400);
    car.set(METRIC_ID_SPEED, 62);
    car.set(METRIC_ID_GEAR, 4);
    car.set(METRIC_ID_COOLANT_TEMP, 92);
    car.set(METRIC_ID_OIL_TEMP, 98);
    car.set(METRIC_ID_FUEL_LEVEL, 41);
    car.set(METRIC_ID_BATT_VOLTAGE, 14.1f);
    car.set(METRIC_ID_BOOST, 0.35f);
    car.set(METRIC_ID_IAT, 31);
    car.set(METRIC_ID_THROTTLE, 22);
    car.set(METRIC_ID_ENGINE_LOAD, 38);
    car.set(METRIC_ID_MIL, 1);
    car.set(METRIC_ID_HIGH_BEAM, 1);
    car.set(METRIC_ID_CRUISE_ON, 1);
    car.set(METRIC_ID_TURN_LEFT, 1);
    boot();
    run(12000);                              // MIL popup has timed out
    for (int pass = 0; pass < 2; pass++) {
        car.night = pass == 1;
        run(600);
        for (size_t i = 0; i < UI._screens.size(); i++) {
            gotoScreen((int)i);
            run(300);
            char nm[48];
            snprintf(nm, sizeof nm, "tour_%s_%zu_%s", pass ? "night" : "day", i,
                     Config.layout()["screens"][i]["type"] | "?");
            shot(nm);
        }
    }
}

/* ─────────────────────────────── main ──────────────────────────────────── */

int main(int argc, char **argv) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc < 2 || !strcmp(argv[1], "--list")) {
        for (auto &kv : registry()) printf("%s\n", kv.first.c_str());
        return 0;
    }
    host::verbose = env("VERBOSE") == "1";
    auto it = registry().find(argv[1]);
    if (it == registry().end()) { printf("unknown scenario %s\n", argv[1]); return 2; }
    it->second();
    for (auto &f : g_fail) printf("  FAIL %s\n", f.c_str());
    return g_fail.empty() ? 0 : 1;
}
