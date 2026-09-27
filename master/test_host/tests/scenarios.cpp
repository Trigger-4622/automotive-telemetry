/**
 * @file scenarios.cpp
 * @brief End-to-end scenarios: the unmodified master firmware (setup/loop and
 *        all its tasks) running in the simulated car. Usage: sim <scenario>
 *        [-v]. Exit code 0 = every check passed.
 */
#include <ArduinoJson.h>

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "Arduino.h"
#include "LittleFS.h"
#include "Learner.h"
#include "MasterConfig.h"
#include "MasterPacket.h"
#include "MasterTelemetry.h"
#include "Ssm2.h"
#include "sim.h"
#include "WiFi.h"
#include <map>

/* ───────────────────────────── check plumbing ─────────────────────────── */

static int s_fails = 0, s_checks = 0;
static void check(bool ok, const char *what, const std::string &info = "") {
    s_checks++;
    if (!ok) s_fails++;
    std::printf("  %s %-62s %s\n", ok ? "ok  " : "FAIL", what, info.c_str());
}
static std::string fmt(const char *f, ...) __attribute__((format(gnu_printf, 1, 2)));
static std::string fmt(const char *f, ...) {
    char b[256]; va_list ap; va_start(ap, f); std::vsnprintf(b, sizeof(b), f, ap); va_end(ap);
    return b;
}
static double tNow() { return sim::elapsedS(); }
static uint64_t s_t0 = 0;     // virtual time at boot (the clock may not start at 0)
static double tOf(uint64_t us) { return (us - s_t0) / 1e6; }

/**
 * The settle wait (start_delay_s, BUS_SETTLE_S by default) keeps the master
 * listen-only for its first seconds on the bus. The scenarios written before
 * it time their checks from the moment the bus comes up, so boot() turns it
 * off for them - the way a user would, in the stored file. Scenarios about
 * the wait itself, and the real device's file, set s_keepSettle first.
 */
static bool s_keepSettle = false;
static void settleOff() {
    const char *path = simfs::files.count("/config.json") ? "/config.json"
                     : simfs::files.count("/config.tmp")  ? "/config.tmp" : nullptr;
    if (!path) {
        simfs::files["/config.json"] = fmt(R"({"cfg_ver":%d,"start_delay_s":0})",
                                           (int)MasterConfig::CFG_VERSION);
        return;
    }
    std::string &t = simfs::files[path];
    const size_t brace = t.find('{');
    if (brace == std::string::npos || t.find("\"start_delay_s\"") != std::string::npos) return;
    const size_t next = t.find_first_not_of(" \t\r\n", brace + 1);
    t.insert(brace + 1, next != std::string::npos && t[next] == '}' ? "\"start_delay_s\":0"
                                                                     : "\"start_delay_s\":0,");
}

/** Start the car and boot the firmware (after any presets). */
static void boot() {
    if (!s_keepSettle) settleOff();
    s_t0 = simrtos::nowUs(); sim::start(); setup();
}

/** Run the Arduino loop until virtual time @p untilS. */
static void runUntil(double untilS) {
    while (tNow() < untilS && !sim::sleepEntered) loop();
}

/** Merge settings into the stored config file before boot, as if they had
 *  been saved from the portal on an earlier drive. */
static void portalPostBeforeBoot(const char *json) {
    JsonDocument cfg, add;
    deserializeJson(cfg, simfs::files["/config.json"]);
    deserializeJson(add, json);
    for (JsonPairConst kv : add.as<JsonObjectConst>()) cfg[kv.key()] = kv.value();
    std::string out;
    serializeJson(cfg, out);
    simfs::files["/config.json"] = out;
}

/** Apply a settings change exactly as the portal's POST handler does. */
static void portalPost(const char *json) {
    JsonDocument d;
    deserializeJson(d, json);
    Cfg.fromJson(d.as<JsonVariantConst>(), true);
    Cfg.save();
}

static uint64_t txCount(std::function<bool(const twai_message_t &)> f, double fromS, double toS = 1e9) {
    uint64_t n = 0;
    for (const auto &r : sim::transmitted)
        if (tOf(r.tUs) >= fromS && tOf(r.tUs) < toS && f(r.msg)) n++;
    return n;
}
static bool isObd(const twai_message_t &m, int pid = -1) {
    return (m.identifier == 0x7E0 || m.identifier == 0x7DF) && m.data[0] == 0x02 && m.data[1] == 0x01 &&
           (pid < 0 || m.data[2] == pid);
}
static bool isObdReq(const twai_message_t &m) { return isObd(m); }   // any PID, for txCount
static bool isSsm(const twai_message_t &m) { return m.identifier == 0x7E0 && !isObd(m); }
static bool anyFrame(const twai_message_t &) { return true; }

static const sim::Shown *shown(uint16_t id) {
    auto it = sim::displayed.find(id);
    return it == sim::displayed.end() ? nullptr : &it->second;
}
static bool freshNear(uint16_t id, double want, double tol, std::string &info) {
    const sim::Shown *s = shown(id);
    if (!s) { info = "never shown"; return false; }
    const double age = (simrtos::nowUs() - s->tUs) / 1e3;
    info = fmt("shown %.2f (truth %.2f), %.0f ms old", s->value, want, age);
    return age < 1500 && std::fabs(s->value - want) <= tol;
}

static int sourceOf(uint16_t id) {
    static MetricViewM mv[192];
    const size_t n = masterGetMetrics(mv, 192);
    for (size_t i = 0; i < n; i++) if (mv[i].id == id) return mv[i].source;
    return -1;
}

struct SigInfo { bool found; RtSignal s; };
static SigInfo learnedFor(uint16_t metric) {
    Cfg.lock();
    SigInfo r = {false, {}};
    for (const auto &s : Cfg.signals) if (s.metricId == metric) { r = {true, s}; break; }
    Cfg.unlock();
    return r;
}
static std::string sigStr(const SigInfo &si) {
    if (!si.found) return "none";
    return fmt("0x%03X start %u len %u %s x%.4f %+.2f mode %u", si.s.canId, si.s.startBit,
               si.s.bitLength, si.s.bigEndian ? "BE" : "LE", si.s.scale, si.s.offset, si.s.mode);
}

static int simBugs() {
    int n = 0;
    for (const auto &l : sim::logLines) if (l.find("SIM-BUG") != std::string::npos) n++;
    return n;
}
static void commonChecks() {
    MasterStats st; masterGetStats(st);
    check(simBugs() == 0, "no misuse of RTOS/driver detected by the simulator", fmt("%d", simBugs()));
    check(sim::bugUninstallWhileWaiting == 0, "driver never uninstalled under a waiting task");
    check(sim::espBadPackets == 0, "every ESP-NOW packet valid for the displays",
          fmt("%llu bad", (unsigned long long)sim::espBadPackets));
    check(sim::espSeqGaps == 0, "ESP-NOW sequence has no gaps", fmt("%llu", (unsigned long long)sim::espSeqGaps));
    check(sim::espPackets > 0, "the displays received packets", fmt("%llu", (unsigned long long)sim::espPackets));
    std::vector<std::string> errs;
    for (const auto &l : sim::logLines) if (l.find("] E ") != std::string::npos) errs.push_back(l);
    check(errs.empty(), "no error lines in the firmware log", errs.empty() ? "" : errs[0]);
    // A fit better than perfect means the regression lost its precision.
    double worst = 0;
    for (const auto &l : sim::logLines) {
        const size_t p = l.find("(r2 ");
        if (p != std::string::npos) worst = std::max(worst, std::atof(l.c_str() + p + 4));
    }
    check(worst <= 1.0, "every learner fit is a valid r2 (<= 1)", fmt("worst %.5f", worst));
}

/* ─────────────────────────────── scenarios ────────────────────────────── */

using namespace sim;

/** Default config, a drive: listen → OBD → SSM, learning takes over. */
static void scAuto() {
    boot();
    runUntil(6);
    auto c = controller();
    check(c.installed && c.mode == TWAI_MODE_NORMAL, "controller up in normal mode");
    check(masterPidSupported(0x0C) && masterPidSupported(0x42), "OBD-II bitmaps read, incl. range 0x40");
    MasterStats st; masterGetStats(st);
    check(st.obdPhysical, "OBD-II uses physical addressing (0x7E0)");

    runUntil(30);
    std::string info;
    const Truth tr = truth();
    check(freshNear(METRIC_ID_COOLANT_TEMP, tr.coolant, 3, info), "coolant reaches the displays", info);
    check(freshNear(METRIC_ID_RPM, tr.rpm, 400, info), "RPM reaches the displays", info);
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.initOk && ss.responses > 10, "SSM2 initialised and answering",
          fmt("init %d, %u ok, %u err, last '%s'", ss.initOk, ss.responses, ss.errors, ss.lastErr));
    check(ss.errors == 0, "no failed SSM2 exchanges", fmt("%u", ss.errors));
    check(shown(METRIC_ID_SSM_KNOCK_CORR) != nullptr, "SSM2-only value (knock correction) shown");
    check(txCount([](const twai_message_t &m) { return isSsm(m) && m.data[0] == 0x10; }, 10) > 0,
          "SSM2 sends multi-frame reads (ISO-TP)");

    runUntil(150);
    const SigInfo rpm = learnedFor(METRIC_ID_RPM), spd = learnedFor(METRIC_ID_SPEED),
                  thr = learnedFor(METRIC_ID_THROTTLE);
    check(rpm.found && rpm.s.canId == 0x231 && rpm.s.startBit == 16 && rpm.s.bitLength == 16 &&
          !rpm.s.bigEndian && std::fabs(rpm.s.scale - 0.25f) < 0.01f, "RPM learned: 0x231 bytes 2-3", sigStr(rpm));
    check(rpm.found && rpm.s.mode == SIG_VERIFIED, "RPM signal verified", sigStr(rpm));
    check(spd.found && spd.s.canId == 0x252 && spd.s.bigEndian, "speed learned: 0x252 Motorola", sigStr(spd));
    check(thr.found && thr.s.canId == 0x232 && thr.s.startBit == 0 && thr.s.bitLength == 8,
          "throttle learned: 0x232 byte 0", sigStr(thr));
    const SigInfo brk = learnedFor(METRIC_ID_SW_BRAKE);
    check(brk.found && brk.s.canId == 0x3B1 && brk.s.startBit == 42, "brake switch learned: 0x3B1 bit 42", sigStr(brk));
    check(sourceOf(METRIC_ID_RPM) == SRC_RAW, "RPM now read from the bus", fmt("source %d", sourceOf(METRIC_ID_RPM)));

    runUntil(200);
    check(txCount([](const twai_message_t &m) { return isObd(m, 0x0C); }, 185, 200) == 0,
          "RPM no longer requested over OBD-II once learned");
    check(freshNear(METRIC_ID_RPM, truth().rpm, 250, info), "RPM from the bus tracks the engine", info);
    // Requests stay within their design pacing: SSM2 at most one exchange per
    // 100 ms, and OBD-II only the PIDs nothing better covers.
    const double ssmX = txCount([](const twai_message_t &m) {
        return isSsm(m) && ((m.data[0] & 0xF0) == 0x10 || (m.data[0] & 0xF0) == 0x00); }, 190, 200) / 10.0;
    check(ssmX <= 10.5, "SSM2 exchanges paced to <= 10/s", fmt("%.1f/s", ssmX));
    const double obdR = txCount([](const twai_message_t &m) { return isObd(m); }, 190, 200) / 10.0;
    check(obdR < 60, "OBD-II request rate bounded", fmt("%.1f/s", obdR));
    for (int pid : {0x0C, 0x0D, 0x11, 0x43, 0x45, 0x47, 0x4A, 0x4C})
        check(txCount([pid](const twai_message_t &m) { return isObd(m, pid); }, 185, 200) == 0,
              fmt("PID %02X not requested once learned from the bus", pid).c_str());
    masterGetStats(st);
    check(st.guardTrips == 0, "bus guard never tripped on a clean bus");
    check(st.errWhileTx == 0 && st.errIdle == 0, "no bus errors counted");
    std::vector<LearnView> lv(128); const size_t nl = learnerStatus(lv.data(), 128);
    int learnedN = 0; for (size_t i = 0; i < nl; i++) if (lv[i].state == LS_LEARNED) learnedN++;
    check(learnedN >= 10, "the portal lists every value read from the bus", fmt("%d", learnedN));
    commonChecks();
}

/** Listen-only from boot with a mapped signal: nothing may be transmitted. */
static void scSilent() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":4,"signals":[
        {"can_id":561,"ext":false,"start":16,"len":16,"be":false,"signed":false,"scale":0.25,
         "offset":0,"metric":268,"mode":3,"ref":268,"learned":true,"name":"learned 010C"}]})";
    boot();
    runUntil(60);
    auto c = controller();
    check(c.installed && c.mode == TWAI_MODE_LISTEN_ONLY, "controller is listen-only");
    check(transmitted.empty(), "not one frame transmitted", fmt("%zu", transmitted.size()));
    std::string info;
    check(freshNear(METRIC_ID_RPM, truth().rpm, 250, info), "RPM from the mapped frame reaches the displays", info);
    check(learnedFor(METRIC_ID_RPM).found, "the mapped signal survived boot");
    commonChecks();
}

/** SILENT and back, live from the portal. */
static void scModeSwitch() {
    boot();
    runUntil(20);
    check(controller().mode == TWAI_MODE_NORMAL && reinstalls == 1, "starts in normal mode");
    portalPost(R"({"diag_mode":4})");
    runUntil(25);
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "SILENT switches the controller to listen-only live",
          fmt("reinstalls %d", reinstalls));
    check(txCount(anyFrame, 24, 40) == 0, "nothing transmitted while silent");
    runUntil(40);
    portalPost(R"({"diag_mode":0})");
    runUntil(50);
    check(controller().mode == TWAI_MODE_NORMAL, "AUTO switches it back");
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 45, 50) > 0, "OBD-II requests resume");
    commonChecks();
}

/** Errors on every other frame we send: pause, pause, listen-only, resume. */
static void scGuard() {
    boot();
    runUntil(20);
    faults.errorPerOwnFrame = 0.5;
    runUntil(30);
    MasterStats st; masterGetStats(st);
    check(st.guardTrips == 1 && st.guardPauseMs > 0, "first trip pauses requests",
          fmt("trips %u, pause %u ms, errTx %u", st.guardTrips, st.guardPauseMs, st.errWhileTx));
    check(txCount(anyFrame, 22, 30) < 10, "nothing (or next to nothing) sent while paused",
          fmt("%llu", (unsigned long long)txCount(anyFrame, 22, 30)));
    runUntil(160);
    masterGetStats(st);
    check(st.guardTrips >= 3 && st.guardSilent, "third trip forces listen-only", fmt("trips %u", st.guardTrips));
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "controller really is listen-only");
    const double tSilent = tNow();
    runUntil(tSilent + 20);
    check(txCount(anyFrame, tSilent + 1, tSilent + 20) == 0, "silence holds");
    faults.errorPerOwnFrame = 0;
    diagGuardReset();
    runUntil(tNow() + 10);
    check(controller().mode == TWAI_MODE_NORMAL, "Resume puts the controller back to normal");
    check(txCount([](const twai_message_t &m) { return isObd(m); }, tNow() - 5) > 0, "requests resume");
    check(bugUninstallWhileWaiting == 0, "no reinstall while a task waited in the driver");
    // The injected errors are expected; the firmware's own error lines about them are too.
    int unexpected = 0;
    for (const auto &l : logLines)
        if (l.find("] E ") != std::string::npos && l.find("bus guard") == std::string::npos) unexpected++;
    check(unexpected == 0 && simBugs() == 0, "no other errors", fmt("%d", unexpected));
}

/** Both guards switched off: the user accepted the risk, requests continue
 *  through errors on our frames and even through bus-off. (With only the
 *  transmit guard off, a bus-off latches listen-only: busoff_guard_off.) */
static void scGuardOff() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"guard":false,"rx_guard":false})";
    boot();
    runUntil(20);
    faults.errorPerOwnFrame = 0.3;
    runUntil(60);
    MasterStats st; masterGetStats(st);
    check(st.guardTrips == 0, "no trips with the guard off");
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 50, 60) > 20, "requests keep going");
    check(st.errWhileTx > 0, "errors still counted for the portal", fmt("%u", st.errWhileTx));
}

/** Errors that have nothing to do with us must not trip the guard. */
static void scIdleErrors() {
    boot();
    runUntil(10);
    faults.idleErrorsPerSec = 2;
    runUntil(130);
    MasterStats st; masterGetStats(st);
    check(st.errIdle > 50, "idle errors seen", fmt("idle %u, ours %u", st.errIdle, st.errWhileTx));
    check(st.guardTrips == 0, "guard not tripped by errors that are not ours",
          fmt("trips %u, counted as ours %u", st.guardTrips, st.errWhileTx));
}

/** Bus-off: recover, trip the guard once, resume after the pause. */
static void scBusOff() {
    boot();
    runUntil(20);
    faults.busOffNow = true;
    runUntil(22);
    MasterStats st; masterGetStats(st);
    check(controller().state == TWAI_STATE_RUNNING, "controller recovered from bus-off",
          fmt("state %d", controller().state));
    check(st.guardTrips == 1, "bus-off counted as a guard trip", fmt("%u", st.guardTrips));
    check(txCount(anyFrame, 21, 45) < 5, "requests paused after bus-off");
    runUntil(60);
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 55, 60) > 0, "requests resume after the pause");
}

/** SSM2 only, with one address the ECU refuses. */
static void scSsmBadAddress() {
    ecu.ssmBadAddress = 0x000113;       // oil temperature
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":1})";
    boot();
    runUntil(40);
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.refused == 1, "the refused address was found and dropped", fmt("refused %u, nrc %u", ss.refused, ss.nrcs));
    const uint64_t bad = ssmReadsWithBad;
    runUntil(80);
    check(ssmReadsWithBad == bad, "never asked again", fmt("%llu then %llu",
          (unsigned long long)bad, (unsigned long long)ssmReadsWithBad));
    std::string info;
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "slow values still read (coolant)", info);
    check(freshNear(METRIC_ID_RPM, truth().rpm, 400, info), "RPM read over SSM2", info);
    check(ss.errors == 0, "no failed exchanges", fmt("%u", ss.errors));
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 1) == 0, "SSM-only mode sends no OBD-II");
    commonChecks();
}

/** SSM2 only, ECU refuses reads of more than 12 addresses. */
static void scSsmSizeLimit() {
    ecu.ssmMaxAddrs = 12;
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":1})";
    boot();
    runUntil(60);
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.refused == 0, "no address wrongly refused", fmt("%u", ss.refused));
    check(ss.batch <= 12, "request size came down to the ECU's limit", fmt("batch %u", ss.batch));
    const uint32_t nrc = ss.nrcs;
    runUntil(120);
    ssm2GetStatus(ss);
    check(ss.nrcs == nrc, "no refusal loop afterwards", fmt("%u then %u", nrc, ss.nrcs));
    std::string info;
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "slow values still read (coolant)", info);
    check(freshNear(METRIC_ID_RPM, truth().rpm, 400, info), "fast values still read (RPM)", info);
    check(shown(METRIC_ID_SW_BRAKE) != nullptr, "switch bytes still read");
    commonChecks();
}

/** ECU ignores physical OBD-II requests: fall back to 0x7DF; TCM answers too. */
static void scObdFunctional() {
    ecu.obdPhysical = false;
    boot();
    runUntil(30);
    MasterStats st; masterGetStats(st);
    check(!st.obdPhysical, "fell back to functional addressing");
    check(masterPidSupported(0x42) && masterPidSupported(0x4C), "engine ECU's range 0x40 read");
    std::string info;
    check(freshNear(METRIC_ID_RPM, truth().rpm, 400, info), "values flow", info);
    commonChecks();
}

/** Every third OBD-II reply lost. */
static void scObdDrops() {
    ecu.dropReplyEveryN = 3;
    boot();
    runUntil(60);
    check(masterPidSupported(0x42), "bitmaps complete despite lost replies");
    std::string info;
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "values flow", info);
}

/** A version-4 file with learned signals and SSM edits (the upgrade path). */
static void scConfigMigration() {
    simfs::files["/config.json"] = R"({"cfg_ver":4,"diag_mode":0,"broadcast_ms":40,
      "ssm":[{"addr":14,"bytes":2,"signed":false,"scale":0.25,"offset":0,"metric":268,"enabled":false,
              "period":0,"cap_byte":1,"cap_bit":1,"name":"Engine speed"}],
      "signals":[{"can_id":561,"ext":false,"start":16,"len":16,"be":false,"signed":false,"scale":0.25,
         "offset":0,"metric":268,"mode":3,"ref":268,"learned":true,"name":"learned 010C"},
        {"can_id":562,"ext":false,"start":0,"len":8,"be":false,"signed":false,"scale":0.392,
         "offset":0,"metric":273,"mode":3,"ref":273,"learned":true,"name":"learned 0111"}]})";
    boot();
    runUntil(3);
    Cfg.lock();
    const size_t nsig = Cfg.signals.size();
    bool rpmOff = false;
    for (const auto &e : Cfg.ssm) if (e.address == 0x0E) rpmOff = !e.enabled;
    Cfg.unlock();
    check(nsig == 2, "learned signals survive the upgrade", fmt("%zu signals", nsig));
    check(rpmOff, "an SSM2 edit survives the upgrade");
    JsonDocument d;
    deserializeJson(d, simfs::files["/config.json"]);
    check((d["cfg_ver"] | 0) == MasterConfig::CFG_VERSION, "file re-saved at the current version",
          fmt("%d", d["cfg_ver"] | 0));
}

/** Flash full during a save, a save interrupted before the rename, and a
 *  filesystem that will not rename over a file. */
static void scConfigFaults() {
    boot();
    runUntil(3);
    const std::string good = simfs::files["/config.json"];
    check(!good.empty(), "boot wrote a config");
    simfs::writeBudget = 100;                       // flash fills after 100 bytes
    portalPost(R"({"broadcast_ms":55})");
    check(simfs::files["/config.json"] == good, "a short write never replaces the good file");
    check(!simfs::files.count("/config.tmp"), "the partial file is removed");
    simfs::writeBudget = -1;
    simfs::renameOverwrites = false;
    portalPost(R"({"broadcast_ms":60})");
    JsonDocument d; deserializeJson(d, simfs::files["/config.json"]);
    check((d["broadcast_ms"] | 0) == 60, "saves work where rename cannot overwrite", fmt("%d", d["broadcast_ms"] | 0));
    // Deleted SSM2 row stays deleted (tombstone), JSON with typos is clamped.
    portalPost(R"({"ssm":[{"addr":14,"bytes":3,"metric":268,"cap_byte":1,"cap_bit":0}],
                   "signals":[{"can_id":561,"start":70,"len":40,"metric":268}]})");
    Cfg.lock();
    const size_t nssm = Cfg.ssm.size(), nrem = Cfg.ssmRemoved.size();
    const RtSignal s0 = Cfg.signals.empty() ? RtSignal() : Cfg.signals[0];
    const uint8_t bytes0 = Cfg.ssm.empty() ? 0 : Cfg.ssm[0].bytes;
    Cfg.unlock();
    check(nssm == 1 && bytes0 == 1, "out-of-range SSM row kept and clamped", fmt("%zu rows, bytes %u", nssm, bytes0));
    check(nrem > 50, "deleted seed rows remembered", fmt("%zu", nrem));
    check(s0.startBit == 63 && s0.bitLength == 1, "out-of-range signal clamped into the payload",
          fmt("start %u len %u", s0.startBit, s0.bitLength));
    Cfg.mergeNewDefaults();
    Cfg.lock(); const size_t after = Cfg.ssm.size(); Cfg.unlock();
    check(after == 1, "merge does not bring deleted rows back", fmt("%zu", after));
    // Out-of-range numbers must not wrap round into other valid values.
    portalPost(R"({"pids":[{"pid":300,"enabled":true,"period":100},{"pid":12,"enabled":true,"period":70000}],
                   "signals":[{"can_id":561,"start":16,"len":16,"metric":70000},{"can_id":561,"start":16,"len":16,"metric":268,"mode":260}]})");
    Cfg.lock();
    const size_t npid = Cfg.pids.size();
    const uint8_t pid0 = npid ? Cfg.pids[0].pid : 0;
    const uint16_t per0 = npid ? Cfg.pids[0].periodMs : 0;
    const size_t nsig = Cfg.signals.size();
    const uint8_t mode0 = nsig ? Cfg.signals[0].mode : 99;
    Cfg.unlock();
    check(npid == 1 && pid0 == 12, "PID 300 is dropped, not read as PID 44", fmt("%zu pids, first %u", npid, pid0));
    check(per0 == 60000, "a 70000 ms period is clamped, not wrapped to 4464", fmt("%u", per0));
    check(nsig == 1, "metric 70000 is dropped, not wrapped to 4464", fmt("%zu signals", nsig));
    check(mode0 == SIG_REJECTED, "mode 260 is clamped, not wrapped to 4", fmt("%u", mode0));
}

/** Interrupted save: only config.tmp exists at boot. */
static void scConfigTmpOnly() {
    simfs::files["/config.tmp"] = R"({"cfg_ver":5,"broadcast_ms":77})";
    boot();
    runUntil(3);
    check(Cfg.broadcastMs == 77, "boot recovers the interrupted save", fmt("%u", Cfg.broadcastMs));
    check(simfs::files.count("/config.json") == 1, "and commits it");
}

/** Car switched off: the master sleeps. */
static void scSleep() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"sleep_idle_s":20})";
    boot();
    runUntil(15);
    faults.silenceBus = true;
    ecu.obdEnabled = ecu.ssmEnabled = ecu.tcmEnabled = false;
    runUntil(60);
    check(sleepEntered, "deep sleep after the bus went quiet", fmt("at %.1f s", tNow()));
    check(tNow() >= 35 && tNow() <= 45, "after about the configured 20 s", fmt("%.1f s", tNow()));
}


/** Awake for 49.7 days: past the 24.8-day signed boundary and across the
 *  32-bit millis() wrap. Booted at 0 like the chip, then the clock is moved
 *  on in jumps of 20 days (under 24.8, so no interval is ever ambiguous -
 *  a running system never sees one that long either), running a minute
 *  after each. Sleep is off: only an awake master gets this old. */
static void scClockWrap() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"sleep_enabled":false})";
    boot();
    runUntil(60);
    const uint64_t DAY = 86400ull * 1000000ull;
    for (int i = 0; i < 2; i++) {
        simrtos::jumpUs(20 * DAY);
        runUntil(tNow() + 60);
    }
    check(millis() > 0x80000000u, "uptime is past 24.8 days", fmt("millis %u", millis()));
    const double t1 = tNow();
    check(txCount([](const twai_message_t &m) { return isObd(m); }, t1 - 30, t1) > 10,
          "OBD-II polls past 24.8 days");
    std::string info;
    check(freshNear(METRIC_ID_RPM, truth().rpm, 400, info), "RPM flows past 24.8 days", info);
    // To 90 s before the wrap, then through it.
    const uint64_t wrapUs = 4294967296ull * 1000ull;
    simrtos::jumpUs(wrapUs - 90000000ull - simrtos::nowUs());
    const double t2 = tNow();
    runUntil(t2 + 220);
    check(millis() < 200000, "the 32-bit clock wrapped", fmt("millis %u", millis()));
    check(freshNear(METRIC_ID_RPM, truth().rpm, 400, info), "RPM still flows after the wrap", info);
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "coolant still flows after the wrap", info);
    check(txCount([](const twai_message_t &m) { return isObd(m); }, t2 + 150, t2 + 220) > 20,
          "OBD-II keeps polling after the wrap");
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.initOk && ss.errors == 0 && txCount(isSsm, t2 + 150, t2 + 220) > 20, "SSM2 keeps working after the wrap",
          fmt("%u ok, %u err", ss.responses, ss.errors));
    MasterStats st; masterGetStats(st);
    check(st.guardPauseMs == 0 && diagGuardOk(), "no phantom guard pause", fmt("%u ms", st.guardPauseMs));
    const SigInfo rpm = learnedFor(METRIC_ID_RPM);
    check(rpm.found && rpm.s.mode == SIG_VERIFIED, "learned signals intact", sigStr(rpm));
    // Learning must still work after the wrap: forget everything, relearn.
    learnerForget();
    runUntil(tNow() + 120);
    const SigInfo again = learnedFor(METRIC_ID_RPM);
    check(again.found && again.s.canId == 0x231 && again.s.startBit == 16 && again.s.bitLength == 16 &&
          !again.s.bigEndian && again.s.mode == SIG_VERIFIED,
          "relearned after the wrap, in the exact field", sigStr(again));
    commonChecks();
}

/** OBD-II and SSM2 at once, sharing the diagnostic channel. */
static void scBoth() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":3})";
    boot();
    runUntil(60);
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.initOk && ss.responses > 50 && ss.errors == 0, "SSM2 runs", fmt("%u ok %u err", ss.responses, ss.errors));
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 30, 60) > 30, "OBD-II runs too");
    std::string info;
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "values flow", info);
    commonChecks();
}

/** OBD-II only: SSM2 must stay silent. */
static void scObdOnly() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":2})";
    boot();
    runUntil(60);
    check(txCount(isSsm, 0) == 0, "no SSM2 frame sent");
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 30, 60) > 30, "OBD-II runs");
    commonChecks();
}

/** Engine ECU off while the bus is alive, then on again. */
static void scEcuOff() {
    ecu.obdEnabled = ecu.ssmEnabled = ecu.tcmEnabled = false;
    boot();
    runUntil(120);
    const double rate = txCount(anyFrame, 30, 120) / 90.0;
    check(rate < 2, "an ECU that does not answer is not flooded", fmt("%.2f frames/s", rate));
    MasterStats st; masterGetStats(st);
    check(st.obdAnswers == 2, "OBD-II marked as not answering");
    ecu.obdEnabled = ecu.ssmEnabled = ecu.tcmEnabled = true;
    runUntil(150);
    check(masterPidSupported(0x0C), "OBD-II found once the ECU wakes");
    std::string info;
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "values flow after it wakes", info);
    commonChecks();
}

static double minCfGapMs(double fromS, double toS) {
    double best = 1e9, last = -1;
    for (const auto &r : sim::transmitted) {
        const double t = tOf(r.tUs);
        if (t < fromS || t >= toS || !isSsm(r.msg)) continue;
        const uint8_t pci = r.msg.data[0] & 0xF0;
        if (pci == 0x20 && last >= 0) best = std::min(best, (t - last) * 1000);
        last = (pci == 0x10 || pci == 0x20) ? t : -1;
    }
    return best;
}

/** The ECU's flow control asks for 5 ms between our frames. */
static void scSsmStmin() {
    ecu.ssmStmin = 5;
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":1})";
    boot();
    runUntil(40);
    const double gap = minCfGapMs(5, 40);
    check(gap >= 5.0, "our consecutive frames honour STmin 5 ms", fmt("min gap %.2f ms", gap));
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.errors == 0 && ss.responses > 20, "exchanges succeed", fmt("%u ok %u err", ss.responses, ss.errors));
    commonChecks();
}

/** A reserved STmin value: ISO 15765-2 says wait the longest, 127 ms. */
static void scSsmStminReserved() {
    ecu.ssmStmin = 0x80;
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":1})";
    boot();
    runUntil(60);
    const double gap = minCfGapMs(5, 60);
    check(gap >= 127.0, "reserved STmin read as 127 ms", fmt("min gap %.2f ms", gap));
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.responses > 3, "exchanges still complete", fmt("%u ok %u err", ss.responses, ss.errors));
}

/** Every fifth SSM2 reply stops half-way. */
static void scSsmCut() {
    ecu.ssmCutEveryN = 5;
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":1})";
    boot();
    runUntil(90);
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.errors > 5, "cut replies are counted as failures", fmt("%u", ss.errors));
    check(ss.initOk && ss.responses > 100, "SSM2 keeps working between them", fmt("%u ok", ss.responses));
    std::string info;
    check(freshNear(METRIC_ID_RPM, truth().rpm, 400, info), "values keep flowing", info);
    int unexpected = 0;
    for (const auto &l : logLines) if (l.find("] E ") != std::string::npos) unexpected++;
    check(unexpected == 0 && simBugs() == 0, "no errors beyond the expected failures");
}

/** Every SSM2 reply is cut: the engine must back off, not hammer. */
static void scSsmDead() {
    ecu.ssmCutEveryN = 1;
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":1})";
    boot();
    runUntil(120);
    const double rate = txCount(isSsm, 60, 120) / 60.0;
    check(rate < 3, "a failing ECU is not hammered", fmt("%.2f frames/s", rate));
    Ssm2Status ss; ssm2GetStatus(ss);
    check(!ss.initOk, "reported as not answering");
}

/** Portal actions during a drive: request instead, forget, census reset. */
static void scPortalActions() {
    boot();
    runUntil(120);
    check(learnedFor(METRIC_ID_RPM).found, "RPM learned first");
    learnerUnlearn(METRIC_ID_RPM);
    runUntil(126);
    check(txCount([](const twai_message_t &m) { return isObd(m, 0x0C); }, 124, 126) > 5,
          "Request instead: RPM requested over OBD-II again");
    runUntil(200);
    const SigInfo again = learnedFor(METRIC_ID_RPM);
    const bool sameField = again.found && again.s.canId == 0x231 && again.s.startBit == 16 &&
                           again.s.bitLength == 16 && !again.s.bigEndian;
    check(!sameField, "the refused field is not learned again", sigStr(again));
    masterResetCensus();
    runUntil(205);
    const sim::Shown *ids = shown(METRIC_ID_MASTER_CAN_IDS);
    check(ids && ids->value <= 12, "census reset and rebuilt", fmt("%.0f IDs", ids ? ids->value : -1.0f));
    learnerForget();
    runUntil(207);
    Cfg.lock(); size_t learnedLeft = 0; for (auto &sg : Cfg.signals) if (sg.learned) learnedLeft++; Cfg.unlock();
    check(learnedLeft == 0, "Forget removes every learned signal", fmt("%zu", learnedLeft));
    runUntil(300);
    check(learnedFor(METRIC_ID_SPEED).found, "learning starts over after Forget");
    commonChecks();
}

/** Far more identifiers than the census holds. */
static void scCensusFull() {
    faults.extraIds = 150;
    boot();
    runUntil(150);
    const sim::Shown *ids = shown(METRIC_ID_MASTER_CAN_IDS);
    check(ids && ids->value == 128, "census fills to its 128 entries", fmt("%.0f", ids ? ids->value : -1.0f));
    check(learnedFor(METRIC_ID_RPM).found, "learning still works");
    commonChecks();
}

/** A flood the receive queue cannot keep up with. */
static void scRxFlood() {
    boot();
    runUntil(20);
    faults.burstFramesPerMs = 20;               // 20 000 extra frames a second
    runUntil(40);
    faults.burstFramesPerMs = 0;
    runUntil(60);
    std::string info;
    check(freshNear(METRIC_ID_RPM, truth().rpm, 400, info), "values flow again after the flood", info);
    check(simBugs() == 0, "no misuse detected");
}

static void scTiming(const char *json, const char *what) {
    simfs::files["/config.json"] = json;
    boot();
    runUntil(5);
    check(controller().installed, what, fmt("installs %d", reinstalls));
    check(simBugs() == 0, "timing registers valid for the controller");
}
static void scTiming250() { scTiming(R"({"cfg_ver":5,"bitrate_kbps":250,"can_sp875":true})", "250 kbit/s at 87.5 % installs"); }
static void scTiming125() { scTiming(R"({"cfg_ver":5,"bitrate_kbps":125,"can_sp875":true})", "125 kbit/s at 87.5 % installs"); }
static void scTiming500() { scTiming(R"({"cfg_ver":5,"bitrate_kbps":500,"can_sp875":true})", "500 kbit/s at 87.5 % installs"); }
static void scTiming1M()  { scTiming(R"({"cfg_ver":5,"bitrate_kbps":1000,"can_sp875":true})", "1 Mbit/s installs"); }

/** A phone on the portal holds sleep off - but not for ever. */
static void scSleepHeld() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"sleep_idle_s":20})";
    WiFi.stations = 1;
    boot();
    runUntil(15);
    faults.silenceBus = true;
    ecu.obdEnabled = ecu.ssmEnabled = ecu.tcmEnabled = false;
    runUntil(100);
    check(!sleepEntered, "stays awake while a portal client is connected");
    runUntil(400);
    check(sleepEntered, "but sleeps within the 5-minute cap", fmt("at %.0f s", tNow()));
}

static bool isRequest(const twai_message_t &m) { return isObd(m) || isSsm(m); }
static bool logHas(const char *text) {
    for (const auto &l : logLines) if (l.find(text) != std::string::npos) return true;
    return false;
}

/** Power-on with the wait at its default and no key in the file: listen only
 *  for 10 s after the bus comes up, the car's broadcasts on the displays all
 *  along, then requests as usual. */
static void scSettle() {
    s_keepSettle = true;
    simfs::files["/config.json"] = R"({"cfg_ver":5,"signals":[
        {"can_id":561,"ext":false,"start":16,"len":16,"be":false,"signed":false,"scale":0.25,
         "offset":0,"metric":268,"mode":3,"ref":268,"learned":true,"name":"learned 010C"}]})";
    boot();
    runUntil(1);
    check(Cfg.startDelayS == 10, "a file without the key gets the 10 s default", fmt("%u s", Cfg.startDelayS));
    check(controller().installed && controller().mode == TWAI_MODE_LISTEN_ONLY, "controller starts listen-only");
    runUntil(5);
    MasterStats st; masterGetStats(st);
    check(st.settleMs > 4500 && st.settleMs <= 5500, "the wait counts down in the status", fmt("%u ms left", st.settleMs));
    check(!st.silent, "not reported as listen-only by choice");
    std::string info;
    check(freshNear(METRIC_ID_RPM, truth().rpm, 250, info), "the car's broadcast values reach the displays meanwhile", info);
    runUntil(10);
    check(txCount(anyFrame, 0, 10) == 0, "not one frame transmitted in the first 10 s",
          fmt("%llu", (unsigned long long)txCount(anyFrame, 0, 10)));
    runUntil(13);
    check(controller().mode == TWAI_MODE_NORMAL && reinstalls == 2, "normal mode once it is over",
          fmt("reinstalls %d", reinstalls));
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 10, 13) > 0, "OBD-II requests start after it");
    masterGetStats(st);
    check(st.settleMs == 0 && !st.silent, "the status says it is over");
    check(logHas("listening only for 10 s") && logHas("bus settled"), "start and end are logged");
    runUntil(45);
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "requested values flow afterwards", info);
    Ssm2Status ss; ssm2GetStatus(ss);
    check(ss.initOk && ss.errors == 0, "SSM2 starts afterwards, without a failed exchange",
          fmt("init %d, %u err, last '%s'", ss.initOk, ss.errors, ss.lastErr));
    masterGetStats(st);
    check(st.guardTrips == 0 && st.errWhileTx == 0, "no bus errors, no guard trips");
    commonChecks();
}

/** The car switched off and on again while the master stayed awake: the wait
 *  starts over when the bus comes back. */
static void scSettleRestart() {
    s_keepSettle = true;                              // no file: every default
    boot();
    runUntil(30);
    check(txCount(isRequest, 25, 30) > 0, "requesting after the first wait");
    faults.silenceBus = true;                         // ignition off...
    ecu.obdEnabled = ecu.ssmEnabled = ecu.tcmEnabled = false;
    runUntil(40);
    check(!sleepEntered, "awake: sleep is 90 s away");
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "listen-only while the bus is quiet");
    MasterStats st; masterGetStats(st);
    check(st.settleMs == 10000, "the whole wait is ahead again", fmt("%u ms", st.settleMs));
    faults.silenceBus = false;                        // ...and on again
    ecu.obdEnabled = ecu.ssmEnabled = ecu.tcmEnabled = true;
    const double on = tNow();
    runUntil(on + 9.8);
    check(txCount(anyFrame, on, on + 9.8) == 0, "nothing sent for 10 s after the bus returns",
          fmt("%llu", (unsigned long long)txCount(anyFrame, on, on + 9.8)));
    std::string info;
    runUntil(on + 16);
    check(controller().mode == TWAI_MODE_NORMAL, "normal mode again after it");
    check(txCount(isRequest, on + 10, on + 16) > 0, "requests resume after it");
    runUntil(on + 30);
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "requested values flow again", info);
    commonChecks();
}

/** Adjustable from the portal: lengthened, shortened or ended live, clamped,
 *  saved - and raising it mid-drive never silences a bus that is up. */
static void scSettlePortal() {
    s_keepSettle = true;
    simfs::files["/config.json"] = R"({"cfg_ver":5,"start_delay_s":20})";
    boot();
    runUntil(5);
    check(Cfg.startDelayS == 20, "a stored wait is used", fmt("%u s", Cfg.startDelayS));
    portalPost(R"({"start_delay_s":30})");            // longer, while waiting
    runUntil(25);
    check(txCount(anyFrame, 0, 25) == 0, "lengthened to 30 s while waiting: still listening at 25 s");
    portalPost(R"({"start_delay_s":28})");            // shorter, still waiting
    runUntil(27.8);
    check(txCount(anyFrame, 0, 27.8) == 0, "shortened to 28 s: still listening at 27.8 s");
    portalPost(R"({"start_delay_s":0})");             // enough waiting
    runUntil(31);
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 27.8, 31) > 0, "0 ends the wait at once");
    check(controller().mode == TWAI_MODE_NORMAL, "and the controller is back to normal");
    portalPost(R"({"start_delay_s":60})");            // longer, mid-drive
    runUntil(40);
    check(controller().mode == TWAI_MODE_NORMAL && txCount(isRequest, 35, 40) > 0,
          "a longer wait set mid-drive does not silence the running bus");
    portalPost(R"({"start_delay_s":99999})");
    check(Cfg.startDelayS == 300, "clamped to 300 s", fmt("%u", Cfg.startDelayS));
    portalPost(R"({"start_delay_s":-5})");
    check(Cfg.startDelayS == 0, "a negative wait is no wait", fmt("%u", Cfg.startDelayS));
    portalPost(R"({"start_delay_s":15})");
    JsonDocument d; deserializeJson(d, simfs::files["/config.json"]);
    check((d["start_delay_s"] | -1) == 15, "saved in the file", fmt("%d", d["start_delay_s"] | -1));
    commonChecks();
}

/**
 * @brief Compare what the firmware holds against a stored config, key by key.
 * @param a     The stored file (what must survive).
 * @param b     The firmware's own re-serialisation.
 * @param path  Where in the document, for the report.
 * @param diffs Collected differences.
 */
static void jsonDiff(JsonVariantConst a, JsonVariantConst b, const std::string &path,
                     std::vector<std::string> &diffs) {
    if (a.is<JsonObjectConst>()) {
        if (!b.is<JsonObjectConst>()) { diffs.push_back(path + ": no longer an object"); return; }
        for (JsonPairConst kv : a.as<JsonObjectConst>()) {
            const std::string p = path + "." + kv.key().c_str();
            if (!b[kv.key()].isUnbound() && !b[kv.key()].isNull())
                jsonDiff(kv.value(), b[kv.key()], p, diffs);
            else if (!kv.value().isNull())
                diffs.push_back(p + ": DROPPED");
        }
    } else if (a.is<JsonArrayConst>()) {
        if (!b.is<JsonArrayConst>()) { diffs.push_back(path + ": no longer a list"); return; }
        const size_t na = a.size(), nb = b.size();
        if (na != nb) diffs.push_back(path + fmt(": %u entries stored, %u loaded", (unsigned)na, (unsigned)nb));
        for (size_t i = 0; i < na && i < nb; i++)
            jsonDiff(a[i], b[i], path + fmt("[%u]", (unsigned)i), diffs);
    } else if (a.is<double>() && b.is<double>()) {
        const double x = a.as<double>(), y = b.as<double>();
        // Stored as float on the device: allow float rounding, nothing more.
        if (std::fabs(x - y) > 1e-5 * std::max(1.0, std::fabs(x)))
            diffs.push_back(path + fmt(": %.6g stored, %.6g loaded", x, y));
    } else if (a.is<const char *>()) {
        if (!b.is<const char *>() || std::strcmp(a.as<const char *>(), b.as<const char *>()))
            diffs.push_back(path + ": text changed");
    } else if (a.is<bool>()) {
        if (!b.is<bool>() || a.as<bool>() != b.as<bool>()) diffs.push_back(path + ": flag changed");
    } else if (!a.isNull()) {
        std::string sa, sb;
        serializeJson(a, sa); serializeJson(b, sb);
        if (sa != sb) diffs.push_back(path + ": " + sa + " -> " + sb);
    }
}

/**
 * The config read off the real master (DEVICE_CONFIG=path): does this
 * firmware load every setting and every learned signal exactly, and keep
 * them while it runs?
 */
static void scDeviceConfig() {
    const char *path = std::getenv("DEVICE_CONFIG");
    if (!path) { check(false, "DEVICE_CONFIG not set"); return; }
    FILE *f = std::fopen(path, "rb");
    if (!f) { check(false, "cannot read DEVICE_CONFIG", path); return; }
    std::string text;
    char buf[4096]; size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
    std::fclose(f);
    JsonDocument stored;
    check(!deserializeJson(stored, text), "stored config parses");
    simfs::files["/config.json"] = text;
    s_keepSettle = true;                          // the file exactly as stored

    boot();
    runUntil(2);
    check(simfs::files["/config.json"] == text, "not rewritten at boot (byte for byte)");

    auto compare = [&](const char *when) {
        JsonDocument loaded;
        Cfg.toJson(loaded);
        std::vector<std::string> diffs;
        jsonDiff(stored.as<JsonVariantConst>(), loaded.as<JsonVariantConst>(), "config", diffs);
        for (size_t i = 0; i < diffs.size() && i < 60; i++) std::printf("      %s\n", diffs[i].c_str());
        check(diffs.empty(), when, fmt("%u differences", (unsigned)diffs.size()));
    };
    compare("every stored setting loaded unchanged");

    size_t learned = 0;
    for (JsonVariantConst s : stored["signals"].as<JsonArrayConst>()) learned += (s["learned"] | false);
    std::printf("      %u signals (%u learned), %u SSM, %u PIDs in the stored file\n",
                (unsigned)stored["signals"].size(), (unsigned)learned,
                (unsigned)stored["ssm"].size(), (unsigned)stored["pids"].size());

    // A minute of driving: the stored signals must all still be there, with
    // their modes, whatever the learner does alongside them.
    runUntil(60);
    JsonDocument after;
    deserializeJson(after, simfs::files["/config.json"]);
    size_t kept = 0;
    for (JsonVariantConst s : stored["signals"].as<JsonArrayConst>())
        for (JsonVariantConst t : after["signals"].as<JsonArrayConst>())
            if ((t["can_id"] | -1) == (s["can_id"] | -2) && (t["start"] | -1) == (s["start"] | -2) &&
                (t["len"] | -1) == (s["len"] | -2) && (t["metric"] | -1) == (s["metric"] | -2) &&
                (t["mode"] | -1) == (s["mode"] | -2)) { kept++; break; }
    check(kept == stored["signals"].size(), "every stored signal still saved after a minute",
          fmt("%u of %u", (unsigned)kept, (unsigned)stored["signals"].size()));
    check(after["ssm"].size() == stored["ssm"].size() && after["pids"].size() == stored["pids"].size(),
          "SSM and PID tables intact after a minute");
}

/* ═══════════════════ the check-engine light: P1718 / P0700 ═══════════════════
 *
 * The transmission ECU sets P1718 when it stops receiving the engine ECU's
 * periodic CAN messages, and P0700 to ask for the lamp. These scenarios run the
 * master with the car's own settings file (DEVICE_CONFIG, as device_config
 * does) in the simulated car with a transmission ECU watching for those frames,
 * and reproduce each way the master can take them away from it.
 */

/** The car's settings file, exactly as read off the master; the stored
 *  defaults for anything it lacks (the settle wait included). */
static void loadCarConfig() {
    std::string text;
    if (const char *path = std::getenv("DEVICE_CONFIG")) {
        if (FILE *f = std::fopen(path, "rb")) {
            char buf[4096]; size_t n;
            while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
            std::fclose(f);
        }
    }
    if (text.empty())        // the values that matter, if the fixture is not there
        text = R"({"cfg_ver":5,"diag_mode":2,"guard":false,"obd_gap":10,"obd_to":20,"obd_addr":0,"sleep_enabled":false})";
    simfs::files["/config.json"] = text;
    s_keepSettle = true;
}

/* Event types of the evidence log (main.cpp EvType), for the checks. */
enum { EVT_BOOT = 0, EVT_BUS_UP, EVT_SETTLE_END, EVT_TX_TRIP, EVT_RX_TRIP, EVT_BUS_OFF, EVT_SLEEP, EVT_RESUME };
static int evCount(int type, uint16_t *aOut = nullptr, uint8_t *actOut = nullptr, uint16_t *bOut = nullptr) {
    static EvView ev[64];
    const size_t n = masterEventLog(ev, 64);
    int k = 0;
    for (size_t i = 0; i < n; i++)
        if (ev[i].type == type) {
            k++;
            if (aOut) *aOut = ev[i].a;
            if (actOut) *actOut = ev[i].act;
            if (bOut) *bOut = ev[i].b;
        }
    return k;
}
static int unexpectedErrors(const char *allowed) {
    int n = 0;
    for (const auto &l : logLines)
        if (l.find("] E ") != std::string::npos && l.find(allowed) == std::string::npos) n++;
    return n;
}

/**
 * SUSPECT 2, the marginal link. The car's settings (transmit-side bus guard off,
 * as on the car), a link on which our normal-mode controller detects errors in
 * the engine ECU's broadcasts and error-flags them - destroying them for every
 * module - and the transmission ECU watching. The receive-error guard must
 * notice the storm on our own receive-error counter and drop the controller to
 * listen-only, where it cannot flag anything, before the TCM loses the frames.
 * Before that guard existed the master carried on and the TCM set P1718.
 */
static void scTcmP1718() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"tx_passive":false})");   // active mode: the guard's case
    boot();
    runUntil(20);                                   // settle over, requests running
    check(tcm().armed && !tcm().p1718, "the TCM is receiving the ECM's broadcasts",
          fmt("%u received", tcm().ecmRx));
    check(controller().mode == TWAI_MODE_NORMAL && txCount(isObdReq, 12, 20) > 10,
          "master in normal mode, requesting over OBD-II");
    faults.rxCorruptRate = 0.9;                     // the link goes marginal
    runUntil(60);
    MasterStats st; masterGetStats(st);
    check(st.rxGuardSilent && st.guardSilent, "receive-error guard tripped on the REC storm",
          fmt("REC %u, TEC %u", st.rec, st.tec));
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "controller switched to listen-only");
    check(st.silent, "the portal reports listen-only");
    check(!tcm().p1718 && !tcm().p0700, "the TCM never lost the ECM: no P1718, no P0700",
          fmt("%u frames destroyed before the guard acted, %u received", tcm().ecmLost, tcm().ecmRx));
    check(tcm().windowRx >= 30, "the ECM's broadcasts flow to the TCM again", fmt("%u in the window", tcm().windowRx));
    check(txCount(anyFrame, 25, 60) == 0, "nothing transmitted since", fmt("%llu", (unsigned long long)txCount(anyFrame, 25, 60)));
    const sim::Shown *rx = shown(METRIC_ID_MASTER_CAN_RX);
    check(rx && rx->value > 100, "still listening: frames reach the displays' health channel",
          fmt("%.0f frames/s", rx ? rx->value : -1.0f));
    uint16_t rec = 0; uint8_t act = 0;
    const int trips = evCount(EVT_RX_TRIP, &rec, &act);
    check(trips == 1 && rec >= 96 && (act & 0x01),
          "evidence log: the trip, its REC, and that we were polling at the time",
          fmt("%d trips, REC %u, activity 0x%02X", trips, rec, act));
    check(simfs::files.count("/evlog.json") == 1, "evidence written to flash");
    diagGuardReset();                               // Resume from the portal
    faults.rxCorruptRate = 0;                       // (the link fixed)
    runUntil(70);
    check(controller().mode == TWAI_MODE_NORMAL && txCount(isObdReq, 65, 70) > 0, "Resume puts it back to normal");
    check(evCount(EVT_RESUME) == 1, "...and that is recorded too");
    check(unexpectedErrors("receive-error guard") == 0 && simBugs() == 0, "no other errors");
}

/** The same fault with the receive-error guard turned off: the master carries
 *  on as before, and the TCM sets P1718 - the car's fault, reproduced. That is
 *  what the guard prevents, and turning it off is the owner's choice. */
static void scTcmP1718Optout() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"tx_passive":false})");   // active mode: the guard's case
    boot();
    portalPost(R"({"rx_guard":false})");
    runUntil(20);
    faults.rxCorruptRate = 0.9;
    runUntil(60);
    MasterStats st; masterGetStats(st);
    check(controller().mode == TWAI_MODE_NORMAL && !st.rxGuardSilent, "guard off: the master stays in normal mode");
    check(tcm().p1718 && tcm().p0700, "...and the TCM sets P1718 and P0700 - the fault, reproduced",
          fmt("%u destroyed, %u in the window", tcm().ecmLost, tcm().windowRx));
    check(evCount(EVT_RX_TRIP) == 0, "no trip recorded (nothing acted)");
    check(txCount(isObdReq, 30, 60) > 0, "requests continued regardless");
}

/** Stray errors that are not ours (a few a second on a busy bus) keep REC near
 *  zero, so they must not trip the receive-error guard either. */
static void scRxGuardTrickle() {
    boot();
    runUntil(10);
    faults.idleErrorsPerSec = 3;
    runUntil(120);
    MasterStats st; masterGetStats(st);
    check(st.errIdle > 200, "plenty of stray errors seen", fmt("%u", st.errIdle));
    check(!st.rxGuardSilent && controller().mode == TWAI_MODE_NORMAL, "a trickle does not trip the receive-error guard",
          fmt("REC %u", st.rec));
    check(evCount(EVT_RX_TRIP) == 0, "nothing recorded as a trip");
    commonChecks();
}

/**
 * A moderate marginal link: a few per cent of the ECM's frames. The first frame
 * the master misreads costs a burst of retransmissions - it flags every one -
 * and CAN's fault confinement drives its REC past the guard's level within that
 * burst (+9 per flag), so the guard trips on the first misread and the damage
 * stops at one burst. (The error-rate criterion is the backstop for errors that
 * do not come in bursts.)
 */
static void scRxGuardModerate() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"tx_passive":false})");   // active mode: the guard's case
    boot();
    runUntil(20);
    faults.rxCorruptRate = 0.05;                    // ~7 of the ECM's 150 frames/s
    runUntil(45);
    MasterStats st; masterGetStats(st);
    uint16_t rec = 0, inWin = 0;
    const int trips = evCount(EVT_RX_TRIP, &rec, nullptr, &inWin);
    check(trips == 1, "tripped on the first misread burst", fmt("%d trips, REC %u, %u errors in the window", trips, rec, inWin));
    check(tcm().destroyed <= 20 && tcm().ecmBusOffs == 0, "one burst of retransmissions at most, the ECM never bus-off",
          fmt("%u destroyed, %u ECM bus-offs", tcm().destroyed, tcm().ecmBusOffs));
    check(st.rxGuardSilent && controller().mode == TWAI_MODE_LISTEN_ONLY, "controller listen-only");
    check(!tcm().p1718, "the TCM never set P1718 (moderate loss does not gap the frames)",
          fmt("%u destroyed", tcm().ecmLost));
    check(tcm().ecmLost < 400, "the corruption was stopped within the window", fmt("%u destroyed", tcm().ecmLost));
    check(txCount(anyFrame, 32, 45) == 0, "nothing transmitted since");
    check(unexpectedErrors("receive-error guard") == 0 && simBugs() == 0, "no other errors");
}

/**
 * THE LISTEN-ONLY ERRATUM (what the car showed after the 2026-09-26 flash: 100 000+
 * errors with the master listen-only, and the MIL sooner than before). On the
 * ESP32-S3 a listen-only controller still sends active error flags, and its
 * frozen REC keeps it error-active: before listenOnlyErratumFix it destroyed
 * every retransmission of a frame it misread until the ECM was bus-off. SILENT
 * mode on a marginal link must not touch the bus at all.
 */
static void scLomErratumSilent() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"diag_mode":4})");
    faults.rxCorruptRate = 0.05;                    // misreads from the first frame
    boot();
    runUntil(120);
    MasterStats st; masterGetStats(st);
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "listen-only (SILENT)");
    check(controller().rec == 128, "error-passive: REC set to 128 and frozen there", fmt("REC %u", controller().rec));
    check(!sim::txPadOnController(), "TX pad taken from the controller while it listens");
    check(st.busErrors > 50, "the master does see the misreads", fmt("%u errors", st.busErrors));
    check(tcm().destroyed == 0 && tcm().ecmBusOffs == 0, "not one frame destroyed, the ECM never bus-off",
          fmt("%u destroyed, %u ECM bus-offs", tcm().destroyed, tcm().ecmBusOffs));
    check(!tcm().p1718 && !tcm().p0700, "no P1718, no P0700");
    check(transmitted.empty(), "nothing transmitted", fmt("%zu", transmitted.size()));
    const sim::Shown *rx = shown(METRIC_ID_MASTER_CAN_RX);
    check(rx && rx->value > 100, "still listening", fmt("%.0f frames/s", rx ? rx->value : -1.0f));
    commonChecks();
}

/** The car's own case: its settings, a marginal link from the start - through
 *  the settle wait (listen-only), the first normal-mode contact (the guard
 *  trips) and on, listen-only, for the rest of a three-minute drive. */
static void scLomErratumDrive() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"tx_passive":false})");   // active mode: the guard's case
    faults.rxCorruptRate = 0.05;
    boot();
    runUntil(9.9);
    check(tcm().destroyed == 0, "nothing destroyed during the settle wait", fmt("%u", tcm().destroyed));
    runUntil(180);
    MasterStats st; masterGetStats(st);
    check(st.rxGuardSilent && controller().mode == TWAI_MODE_LISTEN_ONLY, "the receive-error guard put it listen-only");
    check(controller().rec == 128 && !sim::txPadOnController(), "...error-passive, TX pad off the controller",
          fmt("REC %u", controller().rec));
    check(tcm().destroyed <= 20, "at most one burst destroyed (the first normal-mode misread)",
          fmt("%u destroyed", tcm().destroyed));
    check(tcm().ecmBusOffs == 0, "the ECM never driven bus-off", fmt("%u", tcm().ecmBusOffs));
    check(!tcm().p1718 && !tcm().p0700, "no P1718, no P0700");
    check(unexpectedErrors("receive-error guard") == 0 && simBugs() == 0, "no other errors");
}

/** Why the firmware before the settle wait took far longer to set the MIL: in
 *  normal mode CAN's fault confinement works - on each misread the master's REC
 *  outruns the ECM's TEC and it goes error-passive before the ECM reaches
 *  bus-off, so sparse misreads cost retransmissions, never the ECM. (Frequent
 *  ones still add up on the ECM - its TEC only winds down on its own frames -
 *  which is what the receive-error guard is for. In listen-only, before the
 *  erratum fix, every single misread drove the ECM bus-off.) */
static void scNormalConfinement() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"rx_guard":false,"start_delay_s":0,"tx_passive":false})");
    boot();
    runUntil(10);
    faults.rxCorruptRate = 0.002;                   // a misread every few seconds
    runUntil(130);
    check(controller().mode == TWAI_MODE_NORMAL, "normal mode throughout (guards off)");
    check(tcm().destroyed > 0, "retransmissions were destroyed", fmt("%u", tcm().destroyed));
    check(tcm().ecmBusOffs == 0, "yet the ECM was never bus-off: the master backed off first",
          fmt("%u", tcm().ecmBusOffs));
    check(!tcm().p1718, "no P1718 at this misread rate");
}

/**
 * What the car showed after the listen-only fix: REC back at 0 in listen-only
 * and a million errors a drive. If REC is not frozen there (a good frame winds
 * it down), REC 128 alone does not keep the controller error-passive, and an
 * error-active controller with its TX pad detached counts rounds of its own
 * unheard error flag as bus errors. TEC 128 holds - TEC only moves when it
 * transmits - so every error counted is one real misread.
 */
static void scLomRecDrift() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"diag_mode":4})");
    faults.lomRecCounts = true;
    faults.rxCorruptRate = 0.05;
    boot();
    runUntil(120);
    MasterStats st; masterGetStats(st);
    check(controller().tec == 128, "TEC 128 in listen-only, and it stays there", fmt("TEC %u", controller().tec));
    check(controller().rec < 128, "REC drifted down on good frames, as on the car", fmt("REC %u", controller().rec));
    check(tcm().misreads > 50, "the master misread frames", fmt("%u misreads", tcm().misreads));
    check(st.busErrors == tcm().misreads, "one error counted per misread - no rounds of unheard flags",
          fmt("%u errors for %u misreads", st.busErrors, tcm().misreads));
    check(tcm().destroyed == 0 && !tcm().p1718, "nothing destroyed, no P1718");
    commonChecks();
}

/** The default bit timing: the Arduino-CAN profile - 87.5 %, SJW 2, triple sampling. */
static void scTimingDefault() {
    boot();
    runUntil(2);
    check(std::fabs(sim::timing.samplePct - 87.5) < 0.1 && sim::timing.sjw == 2 && sim::timing.triple,
          "87.5 %, SJW 2, triple sampling by default",
          fmt("%.1f %%, SJW %d, triple %d", sim::timing.samplePct, sim::timing.sjw, sim::timing.triple));
    check(Cfg.canTiming == 2, "can_timing defaults to 2");
    check(simBugs() == 0, "timing registers valid for the controller");
}
/** can_timing 0 keeps the ESP-IDF preset: 80 %, SJW 3, single sample. */
static void scTimingIdf() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"can_timing":0})";
    boot();
    runUntil(2);
    check(std::fabs(sim::timing.samplePct - 80.0) < 0.1 && sim::timing.sjw == 3 && !sim::timing.triple,
          "ESP-IDF preset: 80 %, SJW 3, single sample",
          fmt("%.1f %%, SJW %d, triple %d", sim::timing.samplePct, sim::timing.sjw, sim::timing.triple));
}
/** The older 87.5 % switch still works with the ESP-IDF preset profile. */
static void scTimingSp875Legacy() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"can_timing":0,"can_sp875":true})";
    boot();
    runUntil(2);
    check(std::fabs(sim::timing.samplePct - 87.5) < 0.1 && !sim::timing.triple,
          "can_sp875 with the preset profile: 87.5 %, single sample",
          fmt("%.1f %%, triple %d", sim::timing.samplePct, sim::timing.triple));
}

/** The error classifier: what the controller says each error was. */
static void scErrorKinds() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"rx_guard":false,"start_delay_s":0})");
    boot();
    runUntil(10);
    faults.rxCorruptRate = 0.05;                     // misreads while receiving
    runUntil(30);
    faults.rxCorruptRate = 0;
    faults.errorPerOwnFrame = 0.2;                   // errors in our own requests
    runUntil(33);
    faults.errorPerOwnFrame = 0;
    ErrKindView ek[6];
    const size_t n = masterErrorKinds(ek, 6);
    bool rx = false, tx = false;
    for (size_t i = 0; i < n; i++) {
        if (ek[i].code == 0xAA && !ek[i].silent && ek[i].errors > 0) rx = true;
        if (ek[i].code == 0x0A && !ek[i].silent) tx = true;
    }
    check(n >= 2 && rx, "misreads recorded as stuff errors while receiving, in the data field", fmt("%zu kinds", n));
    check(tx, "our own failed frames recorded as bit errors while sending");
    check(n && ek[0].code == 0xAA, "sorted by count, the misreads first");
    char txt[64];
    errKindText(0xAA, txt, sizeof(txt));
    check(std::string(txt) == "stuff error receiving in data field", "readable for the portal and the log", txt);
    errKindText(0x19, txt, sizeof(txt));
    check(std::string(txt) == "bit error sending in ACK slot", "the ACK-slot case reads as such", txt);
    check(sim::serialOut.find("[errs ] stuff error receiving in data field, normal mode") != std::string::npos,
          "the serial heartbeat lists them");
}

/** Radio transmit power: 13 dBm by default, and a change applies live. */
static void scRadioPower() {
    boot();
    runUntil(2);
    check(sim::radioQdbm == 52, "13 dBm at boot", fmt("%d quarter-dBm", sim::radioQdbm));
    portalPost(R"({"radio_dbm":8})");
    runUntil(3);
    check(sim::radioQdbm == 32, "a change from the portal applies without a reboot", fmt("%d", sim::radioQdbm));
    portalPost(R"({"radio_dbm":99})");
    check(Cfg.radioDbm == 20, "clamped to 20 dBm", fmt("%u", Cfg.radioDbm));
}

/**
 * PASSIVE MODE, the default. The link the car has - the master misreading a
 * large share of the ECM's frames - with the car's settings, requests running
 * the whole time: the controller is kept error-passive, so every error flag it
 * sends is recessive and not one frame is destroyed, the ECM is never driven
 * bus-off, the TCM never loses it, and requests are never stopped for it.
 */
static void scPassiveMarginal() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"diag_mode":2})");     // transmitting, whatever else the file says
    boot();
    runUntil(15);
    check(controller().mode == TWAI_MODE_NORMAL && controller().tec >= 128,
          "normal mode, error-passive from the start", fmt("TEC %u", controller().tec));
    faults.rxCorruptRate = 0.3;                      // far worse than the car
    uint32_t minTec = 999;
    for (double t = 16; t <= 120; t += 0.5) {       // TEC stays passive throughout
        runUntil(t);
        if (controller().mode == TWAI_MODE_NORMAL) minTec = std::min(minTec, controller().tec);
    }
    MasterStats st; masterGetStats(st);
    check(minTec >= 128, "TEC never fell below 128 (topped up between requests)", fmt("lowest %u", minTec));
    check(tcm().misreads > 1000, "the master misread plenty", fmt("%u misreads", tcm().misreads));
    check(tcm().destroyed == 0 && tcm().ecmBusOffs == 0, "not one frame destroyed, the ECM never bus-off",
          fmt("%u destroyed, %u bus-offs", tcm().destroyed, tcm().ecmBusOffs));
    check(!tcm().p1718 && !tcm().p0700, "no P1718, no P0700");
    check(!st.rxGuardSilent && controller().mode == TWAI_MODE_NORMAL, "requests never stopped for it");
    check(txCount([](const twai_message_t &m) { return isObd(m); }, 100, 120) > 50, "OBD-II requests running at the end",
          fmt("%llu in 20 s", (unsigned long long)txCount([](const twai_message_t &m) { return isObd(m); }, 100, 120)));
    std::string info;
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "requested values still reach the displays", info);
    check(st.errWhileTx == 0, "no misread blamed on our own frames (top-ups are not errors)", fmt("%u", st.errWhileTx));
    int passiveLogs = 0;
    for (const auto &l : logLines) if (l.find("TWAI error-passive") != std::string::npos) passiveLogs++;
    check(passiveLogs == 0, "no 'error-passive' warnings for the intended state");
    commonChecks();
}

/** Passive mode off: back to an error-active node (TEC 0 in normal mode). */
static void scPassiveOff() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"tx_passive":false})";
    boot();
    runUntil(15);
    check(controller().mode == TWAI_MODE_NORMAL && controller().tec < 128, "error-active when switched off",
          fmt("TEC %u", controller().tec));
    check(!Cfg.txPassive, "the setting is off");
    check(sim::activeFlagStarts >= 1, "able to send error frames, as asked (and the simulator sees it)",
          fmt("%d", sim::activeFlagStarts));
}

/**
 * LET ERRORS PASS (tx_passive, the default): the master never sends an error
 * frame. Its controller is never on the bus able to send an ACTIVE error flag -
 * the six dominant bits that destroy a frame for every module - not while the
 * bus settles, not when it goes from listening to requesting, not after a
 * bus-off restart, and not on a switch to listen-only and back, although every
 * start clears the counters that keep it error-passive.
 */
static void scErrorFramesNever() {
    s_keepSettle = true;                              // no file: every default
    boot();
    runUntil(8);
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "listening while the bus settles");
    runUntil(20);
    check(controller().mode == TWAI_MODE_NORMAL && controller().tec >= 128, "then requesting, error-passive",
          fmt("TEC %u", controller().tec));
    check(txCount(isRequest, 10, 20) > 0, "requests went out");
    faults.busOffNow = true;
    runUntil(22);
    check(controller().state == TWAI_STATE_RUNNING && controller().mode == TWAI_MODE_NORMAL &&
          controller().tec >= 128, "restarted after a bus-off, error-passive again", fmt("TEC %u", controller().tec));
    portalPost(R"({"diag_mode":4})");                 // to listen-only...
    runUntil(26);
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "listen-only when asked");
    portalPost(R"({"diag_mode":0})");                 // ...and back
    runUntil(32);
    check(controller().mode == TWAI_MODE_NORMAL && controller().tec >= 128, "requesting again, error-passive",
          fmt("TEC %u", controller().tec));
    faults.rxCorruptRate = 0.3;                       // a bad link from here on
    runUntil(90);
    check(sim::activeFlagStarts == 0, "never on the bus able to send an error frame",
          fmt("%d times", sim::activeFlagStarts));
    check(tcm().misreads > 100 && tcm().destroyed == 0, "every frame it misread passed untouched",
          fmt("%u misread, %u destroyed", tcm().misreads, tcm().destroyed));
    check(!tcm().p1718 && !tcm().p0700, "no P1718, no P0700");
    check(txCount(isRequest, 60, 90) > 0, "still requesting (after the bus-off pause)",
          fmt("%llu", (unsigned long long)txCount(isRequest, 60, 90)));
    commonChecks();
}

/**
 * SAFE BUS SETTINGS (portal, Maintenance): the settings on the master in the
 * car - listen only, both guards off, no P2CAN wait - and on top of them errors
 * flagged, the ESP-IDF timing and no TX hold, put back to the bus defaults in
 * one go. It transmits and receives again with errors let pass, and nothing
 * else changes: the bitrate, the pacing the user tuned, the displays, Wi-Fi
 * and every learned signal stay as they were.
 */
static void scBusDefaults() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"diag_mode":4,"rx_guard":false,"guard":false,"obd_p2can":0,"start_delay_s":5,
        "tx_passive":false,"can_timing":0,"tx_hold":false,"obd_to":25,"obd_gap":20,"req_max_hz":40})");
    boot();
    runUntil(12);
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "the stored settings only listen");
    JsonDocument before, after;
    Cfg.toJson(before);
    Cfg.loadBusDefaults();                            // what /api/bus_defaults does
    Cfg.save();                                       // before its reboot
    Cfg.toJson(after);
    static const char *BUS[] = {"diag_mode", "tx_passive", "can_timing", "can_sp875", "guard", "guard_errs",
                                "guard_win", "guard_pause", "guard_trips", "rx_guard", "rx_guard_rec",
                                "rx_guard_errs", "obd_p2can", "start_delay_s", "tx_hold"};
    check(after["diag_mode"] == DIAG_MODE_AUTO && after["tx_passive"] == true && after["can_timing"] == 2 &&
          after["guard"] == true && after["rx_guard"] == true && after["obd_p2can"] == 50 &&
          after["start_delay_s"] == BUS_SETTLE_S && after["tx_hold"] == true,
          "the bus settings are the defaults: requests on, errors let pass, 87.5 % triple, guards, P2CAN");
    std::string changed;
    int kept = 0;
    for (JsonPairConst kv : before.as<JsonObjectConst>()) {
        bool bus = false;
        for (const char *k : BUS) if (!std::strcmp(kv.key().c_str(), k)) bus = true;
        if (bus) continue;
        std::string a, b;
        serializeJson(kv.value(), a);
        serializeJson(after[kv.key()], b);
        if (a != b) changed += std::string(kv.key().c_str()) + " ";
        else kept++;
    }
    check(changed.empty() && kept > 30, "everything else kept: bitrate, pacing, displays, Wi-Fi, tables",
          changed.empty() ? fmt("%d settings and tables unchanged", kept) : changed);
    check(after["signals"].size() > 0 && after["obd_to"] == 25 && after["req_max_hz"] == 40,
          "learned signals and the tuned pacing among them",
          fmt("%u signals", (unsigned)after["signals"].size()));
    JsonDocument file;
    deserializeJson(file, simfs::files["/config.json"]);
    check(file["diag_mode"] == DIAG_MODE_AUTO && file["tx_passive"] == true && file["obd_to"] == 25,
          "saved: what the reboot reads back");
    runUntil(30);
    check(controller().mode == TWAI_MODE_NORMAL && controller().tec >= 128,
          "transmitting again, errors let pass", fmt("TEC %u", controller().tec));
    check(txCount(isRequest, 20, 30) > 0, "requests going out",
          fmt("%llu", (unsigned long long)txCount(isRequest, 20, 30)));
    check(sim::activeFlagStarts == 0, "never able to send an error frame", fmt("%d", sim::activeFlagStarts));
    commonChecks();
}

/** The misses on the ECM's IDs (the frames the fault misreads) and on the rest. */
static void ecmMisses(uint32_t &ecm, uint32_t &other, int &idsHit) {
    MissView mv[32];
    const size_t n = masterMissedIds(mv, 32);
    ecm = other = 0;
    idsHit = 0;
    for (size_t i = 0; i < n; i++) {
        const uint32_t m = mv[i].missed[0] + mv[i].missed[1];
        if (m) idsHit++;
        if (mv[i].id == 0x231 || mv[i].id == 0x232) ecm += m; else other += m;
    }
}

/**
 * WHERE THE ERRORS COME FROM. The master counts, per ID, the car's periodic
 * frames it did not receive - a gap of whole periods is a frame it misread -
 * and checks each bus error against its own radio. A link that misreads the
 * ECM's frames, in listen-only and then in normal mode: the misses are exactly
 * the ECM's IDs and add up to what it misread, nothing else is missed, a
 * clean link misses nothing, and the errors land near the radio no more
 * often than chance.
 */
static void scLinkMisses() {
    simfs::files["/config.json"] = R"({"cfg_ver":5})";   // every default
    boot();
    runUntil(20);
    MasterStats st;
    masterGetStats(st);
    check(st.missed[0] == 0 && st.expected[0] > 1000, "a clean link: nothing missed",
          fmt("%u of %u", st.missed[0], st.expected[0]));
    portalPost(R"({"diag_mode":4})");                 // listen-only...
    runUntil(22);
    faults.rxCorruptRate = 0.05;
    const uint32_t mis0 = tcm().misreads;
    runUntil(50);
    const uint32_t misLom = tcm().misreads - mis0;
    masterGetStats(st);
    check(st.missed[1] > 0 && st.missed[1] <= misLom, "listen-only: the misreads show as missed frames",
          fmt("%u missed, %u misread", st.missed[1], misLom));
    faults.rxCorruptRate = 0;
    portalPost(R"({"diag_mode":0})");                 // ...then normal mode
    runUntil(54);
    faults.rxCorruptRate = 0.05;
    const uint32_t mis1 = tcm().misreads;
    masterGetStats(st);
    const uint32_t n0Miss = st.missed[0], n0Exp = st.expected[0];   // the clean start aside
    runUntil(82);
    const uint32_t misNormal = tcm().misreads - mis1;
    masterGetStats(st);
    uint32_t ecm, other;
    int idsHit;
    ecmMisses(ecm, other, idsHit);
    const uint32_t misread = misLom + misNormal;
    check(ecm >= misread * 7 / 10 && ecm <= misread, "the missed frames are the ECM frames it misread",
          fmt("%u missed on 0x231/0x232, %u misread", ecm, misread));
    check(other == 0, "no other ID missed", fmt("%u", other));
    const double rN = 100.0 * (st.missed[0] - n0Miss) / std::max<uint32_t>(1, st.expected[0] - n0Exp);
    const double rL = 100.0 * st.missed[1] / std::max<uint32_t>(1, st.expected[1]);
    check(st.missed[0] > 0 && rN < 2 * rL && rL < 2 * rN, "the same link reads the same in both modes",
          fmt("normal %.2f %%, listen-only %.2f %%", rN, rL));
    const double near = 100.0 * st.errNearRadio / std::max<uint32_t>(1, st.errRadioChecked);
    const double hot  = st.radioHotPermille / 10.0;
    check(st.errRadioChecked > 100 && std::fabs(near - hot) < 5 && hot > 2,
          "errors that have nothing to do with the radio land near it only by chance",
          fmt("%.1f %% of %u errors near the radio, radio that busy %.1f %% of the time", near,
              st.errRadioChecked, hot));
    check(st.rxByMode[0] > 0 && st.rxByMode[1] > 0 && st.errByMode[1] > 0,
          "frames and errors booked by mode", fmt("%u/%u frames, %u/%u errors", st.rxByMode[0],
              st.rxByMode[1], st.errByMode[0], st.errByMode[1]));
    commonChecks();
}

/**
 * The same instruments on a master whose radio disturbs its CAN side: the
 * errors bunch up around the display broadcasts, far above chance, and the
 * misses are spread over every module's IDs - the signature of the master's
 * own supply, not of one module's link.
 */
static void scLinkRadio() {
    simfs::files["/config.json"] = R"({"cfg_ver":5})";
    boot();
    runUntil(20);
    faults.radioCorruptRate = 0.5;
    runUntil(80);
    MasterStats st;
    masterGetStats(st);
    const double near = 100.0 * st.errNearRadio / std::max<uint32_t>(1, st.errRadioChecked);
    const double hot  = st.radioHotPermille / 10.0;
    check(sim::radioMisreads > 100 && st.errRadioChecked >= sim::radioMisreads * 9 / 10,
          "the radio-borne misreads were counted as bus errors",
          fmt("%llu misread, %u errors", (unsigned long long)sim::radioMisreads, st.errRadioChecked));
    check(near > 90 && near > 3 * hot, "the errors bunch up around the radio, far above chance",
          fmt("%.0f %% near the radio, radio that busy %.0f %% of the time", near, hot));
    uint32_t ecm, other;
    int idsHit;
    ecmMisses(ecm, other, idsHit);
    check(idsHit >= 5 && other > ecm / 2, "missed frames spread over every module's IDs",
          fmt("%d IDs, %u on the ECM's, %u on the rest", idsHit, ecm, other));
    check(controller().mode == TWAI_MODE_NORMAL && txCount(isRequest, 70, 80) > 0,
          "still requesting throughout");
    commonChecks();
}

/**
 * A driver install that fails - out of memory, no free interrupt - must not
 * leave the master deaf until a reboot. A failed switch is retried by the
 * guard (the mode it wants still differs from the one installed), but the
 * install at boot into normal mode was retried by nothing.
 */
static void scNoDriverRetry() {
    simfs::files["/config.json"] = R"({"cfg_ver":5})";   // boot() adds start_delay_s 0: normal at once
    faults.installFailures = 1;                       // the install at boot fails
    boot();
    runUntil(1);
    check(!controller().installed, "the install at boot failed: no driver");
    runUntil(8);
    check(controller().installed && controller().mode == TWAI_MODE_NORMAL,
          "installed again within seconds, and requesting");
    check(txCount(isRequest, 6, 8) > 0, "requests going out",
          fmt("%llu", (unsigned long long)txCount(isRequest, 6, 8)));
    faults.installFailures = 1;                       // and a failed switch recovers too
    portalPost(R"({"diag_mode":4})");
    runUntil(12);
    check(controller().installed && controller().mode == TWAI_MODE_LISTEN_ONLY, "listen-only as asked");
    portalPost(R"({"diag_mode":0})");
    runUntil(20);
    check(controller().mode == TWAI_MODE_NORMAL, "back to normal");
    check(simBugs() == 0, "no misuse of the driver", fmt("%d", simBugs()));
}

/**
 * The start in normal mode after listen-only writes TEC (128 -> 220). That is
 * not an error in a frame of ours: errors that happen to arrive in the same
 * moment must not be booked against our frames, nor trip the bus guard.
 */
static void scTecWriteNotOurs() {
    s_keepSettle = true;                              // no file: the settle wait, guard on
    boot();
    runUntil(5);
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "listening while the bus settles");
    while (controller().mode != TWAI_MODE_NORMAL && tNow() < 20) runUntil(tNow() + 0.001);
    faults.idleErrorsPerSec = 2000;                   // errors not of ours, right at the switch
    runUntil(tNow() + 0.1);
    faults.idleErrorsPerSec = 0;
    runUntil(tNow() + 1);
    MasterStats st;
    masterGetStats(st);
    check(st.errWhileTx == 0, "no error booked against our frames", fmt("%u", st.errWhileTx));
    check(st.guardTrips == 0, "no bus-guard trip", fmt("%u", st.guardTrips));
}

/**
 * A flash write holds the CAN interrupt off, and the frames queued meanwhile
 * come out microseconds apart. The missed-frame count must not learn that
 * burst as an ID's rhythm - early, while it still learns the periods, or
 * later - nor count the stall as misses.
 */
static void scLinkBursts() {
    simfs::files["/config.json"] = R"({"cfg_ver":5})";
    faults.rxStallEveryMs = 700;                      // from the start: during learning too
    faults.rxStallMs = 60;
    boot();
    runUntil(40);
    MasterStats st;
    masterGetStats(st);
    check(st.missed[0] == 0, "the stalls are not taken for misses", fmt("%u", st.missed[0]));
    MissView mv[32];
    const size_t n = masterMissedIds(mv, 32);
    int slow = 0;                                     // the 50 and 100 ms IDs: learned after 0.7 s
    for (size_t i = 0; i < n; i++)
        if (mv[i].periodMs >= 45 && mv[i].periodMs <= 110 && mv[i].expected[0] > 50) slow++;
    check(slow >= 5, "every 50-100 ms ID still has its rhythm", fmt("%d of 5", slow));
    faults.rxCorruptRate = 0.05;
    const uint32_t mis0 = tcm().misreads;
    runUntil(80);
    uint32_t ecm, other;
    int idsHit;
    ecmMisses(ecm, other, idsHit);
    const uint32_t misread = tcm().misreads - mis0;
    check(ecm >= misread * 6 / 10 && ecm <= misread, "misreads still counted between the stalls",
          fmt("%u missed, %u misread", ecm, misread));
    check(other == 0, "and nothing else", fmt("%u", other));
}

/**
 * A Wi-Fi name or password the AP cannot take locks the portal - the only way
 * into the settings - away for good: refused, the old one kept. A password
 * longer than 31 characters used to be cut short, so the AP's was no longer
 * the one typed.
 */
static void scPortalApSafe() {
    simfs::files["/config.json"] = R"({"cfg_ver":5})";
    boot();
    runUntil(2);
    portalPost(R"({"ap_ssid":""})");
    check(!strcmp(Cfg.apSsid, MasterConfig::AP_SSID_DEFAULT), "an empty Wi-Fi name is refused",
          Cfg.apSsid);
    const std::string pw40(40, 'k'), pw70(70, 'k');
    portalPost((R"({"ap_pass":")" + pw40 + "\"}").c_str());
    check(pw40 == Cfg.apPass, "a 40-character password is kept whole", fmt("%zu", strlen(Cfg.apPass)));
    portalPost((R"({"ap_pass":")" + pw70 + "\"}").c_str());
    check(pw40 == Cfg.apPass, "one WPA2 cannot use (70) is refused", fmt("%zu", strlen(Cfg.apPass)));
    portalPost(R"({"ap_ssid":"Legacy B4 master"})");
    check(!strcmp(Cfg.apSsid, "Legacy B4 master"), "a proper name is taken", Cfg.apSsid);
    JsonDocument d;
    deserializeJson(d, simfs::files["/config.json"]);
    check(d["ap_pass"].as<std::string>() == pw40, "and saved as typed");
}

/**
 * The portal switched off (portal_on) takes the only way into the settings
 * with it: there is no serial console, and the configuration survives a
 * reflash. BOOT held for 3 s brings it back; a short press does nothing, and
 * with the portal on the button is not even looked at.
 */
static void scPortalRescue() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"portal_on":false})";
    boot();
    sim::restartReturns = true;
    runUntil(5);
    sim::bootButtonDown = true;                       // a short press
    runUntil(6.5);
    sim::bootButtonDown = false;
    runUntil(8);
    check(sim::restarts == 0 && !Cfg.portalOn, "a short press of BOOT does nothing");
    sim::bootButtonDown = true;
    runUntil(12);
    sim::bootButtonDown = false;
    check(sim::restarts == 1, "BOOT held 3 s restarts the master", fmt("%d restarts", sim::restarts));
    JsonDocument d;
    deserializeJson(d, simfs::files["/config.json"]);
    check(d["portal_on"] == true, "with the portal on for the next boot");
    sim::bootButtonDown = true;
    runUntil(20);
    check(sim::restarts == 1, "with the portal on, BOOT is left alone", fmt("%d restarts", sim::restarts));
    check(simBugs() == 0, "no misuse of the driver", fmt("%d", simBugs()));
}

/** SSM2 and OBD-II together, heavy traffic: the top-up keeps up, and a
 *  bus-off recovery comes back passive. */
static void scPassiveBusOff() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":3,"rx_guard":false,"guard":false})";
    boot();
    runUntil(20);
    faults.busOffNow = true;                         // (the sim forces it while running normal)
    runUntil(25);
    check(controller().state == TWAI_STATE_RUNNING && controller().mode == TWAI_MODE_NORMAL,
          "recovered from bus-off, still requesting");
    check(controller().tec >= 128, "error-passive again after the recovery", fmt("TEC %u", controller().tec));
    uint32_t minTec = 999;
    for (double t = 26; t <= 60; t += 0.25) { runUntil(t); minTec = std::min(minTec, controller().tec); }
    check(minTec >= 128, "TEC held up under SSM2 + OBD-II traffic", fmt("lowest %u", minTec));
    check(txCount(anyFrame, 40, 60) > 100, "both engines kept requesting",
          fmt("%llu frames", (unsigned long long)txCount(anyFrame, 40, 60)));
}

/** The likeliest moment for the trip is the first normal-mode contact with a
 *  marginal link, right after the settle wait - and the switch back to
 *  listen-only must not wait for the 2 s mode-switch throttle. */
static void scRxGuardPrompt() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"tx_passive":false})");   // active mode: the guard's case
    boot();
    runUntil(5);
    faults.rxCorruptRate = 0.9;                     // marginal from the start (no effect while listen-only)
    runUntil(9.9);
    check(tcm().ecmLost == 0, "listen-only during the settle wait corrupts nothing");
    runUntil(10.8);                                 // settle ends at ~10.1 s
    check(controller().mode == TWAI_MODE_LISTEN_ONLY && evCount(EVT_RX_TRIP) == 1,
          "tripped and back to listen-only within 0.7 s of the first normal-mode contact",
          fmt("mode %d", controller().mode));
    check(tcm().ecmLost < 80, "only a short burst of frames destroyed", fmt("%u", tcm().ecmLost));
    runUntil(30);
    check(!tcm().p1718, "no P1718");
    check(unexpectedErrors("receive-error guard") == 0 && simBugs() == 0, "no other errors");
}

/** Bus-off with the transmit-side bus guard off, as on the car: the
 *  receive-error guard holds the controller listen-only instead of letting it
 *  recover and carry straight on. Resume from the portal puts it back. */
static void scBusOffGuardOff() {
    loadCarConfig();                                // guard false
    boot();
    runUntil(20);
    faults.busOffNow = true;
    runUntil(25);
    MasterStats st; masterGetStats(st);
    check(controller().state == TWAI_STATE_RUNNING, "controller recovered from bus-off");
    check(st.rxGuardSilent && controller().mode == TWAI_MODE_LISTEN_ONLY, "...into listen-only, not back to requesting");
    check(st.guardTrips == 0, "the (off) bus guard was not involved");
    uint16_t tec = 0, latched = 0;
    const int offs = evCount(EVT_BUS_OFF, &tec, nullptr, &latched);
    check(offs == 1 && latched == 1, "bus-off recorded as latching listen-only",
          fmt("%d bus-offs, TEC %u, latched %u", offs, tec, latched));
    runUntil(40);
    check(txCount(anyFrame, 21, 40) == 0, "nothing transmitted since", fmt("%llu", (unsigned long long)txCount(anyFrame, 21, 40)));
    diagGuardReset();
    runUntil(50);
    check(controller().mode == TWAI_MODE_NORMAL && txCount(isObdReq, 45, 50) > 0, "Resume puts it back");
    check(unexpectedErrors("bus-off") == 0 && simBugs() == 0, "no other errors");
}

static double minObdGapMs(double fromS, double toS) {
    double best = 1e9, last = -1;
    for (const auto &r : sim::transmitted) {
        const double t = tOf(r.tUs);
        if (t < fromS || t >= toS || !isObd(r.msg)) continue;
        if (last >= 0) best = std::min(best, (t - last) * 1000);
        last = t;
    }
    return best;
}

/**
 * SUSPECT 1, request pacing. The car's obd_to is 20 ms; ISO 15765-4 gives the
 * ECU 50 ms (P2CAN) to answer. An ECU that answers in 35 ms is timed out at
 * 20 ms, and the next request used to go out 10 ms later - into the reply the
 * ECU was still sending. After an unanswered request the next one now waits
 * until P2CAN has passed since the send (obd_p2can). The stored obd_to is left
 * as it is (device_config must show 0 differences); the wait is the extra.
 */
static void scPacingP2can() {
    loadCarConfig();
    JsonDocument stored;
    deserializeJson(stored, simfs::files["/config.json"]);
    const int storedTo = stored["obd_to"] | 80;
    ecu.replyLatencyUs = 35000;
    boot();
    runUntil(30);
    check(masterPidSupported(0x0C), "the probe (150 ms wait) finds the ECU");
    const double gap = minObdGapMs(12, 30);
    check(gap >= 49.5, "after an unanswered request the next waits for P2CAN (50 ms)", fmt("min gap %.1f ms", gap));
    check(txCount(isObdReq, 12, 30) > 20, "requests continue - spread over time, none dropped",
          fmt("%llu", (unsigned long long)txCount(isObdReq, 12, 30)));
    check(shown(METRIC_ID_COOLANT_TEMP) != nullptr, "the late replies are still decoded");
    check(Cfg.obdTimeoutMs == storedTo, "the stored obd_to is untouched", fmt("%u (stored %d)", Cfg.obdTimeoutMs, storedTo));
    portalPost(R"({"obd_p2can":0})");
    runUntil(50);
    const double gap0 = minObdGapMs(35, 50);
    check(gap0 < 50.0, "obd_p2can 0 turns the wait off (adjustable)", fmt("min gap %.1f ms", gap0));
    portalPost(R"({"obd_p2can":5000})");
    check(Cfg.obdP2CanMs == 2000, "clamped to 2000 ms", fmt("%u", Cfg.obdP2CanMs));
    portalPost(R"({"obd_p2can":50})");
    JsonDocument d; deserializeJson(d, simfs::files["/config.json"]);
    check((d["obd_p2can"] | -1) == 50, "saved in the file", fmt("%d", d["obd_p2can"] | -1));
}

/** The same P2CAN rule in AUTO mode, where SSM2 shares the bus. The cool-down
 *  counts from when a request really went out, not from when the poller asked
 *  for the bus - an SSM2 exchange in front of it can hold the bus for tens of
 *  ms, which with the car's 2026-09-26 settings (obd_to 25, obd_gap 20) left
 *  a 45 ms gap behind an unanswered request. */
static void scPacingP2canAuto() {
    loadCarConfig();
    portalPostBeforeBoot(R"({"diag_mode":0,"obd_to":25,"obd_gap":20})");
    ecu.replyLatencyUs = 35000;
    boot();
    runUntil(40);
    check(txCount([](const twai_message_t &m) { return isSsm(m); }, 12, 40) > 0, "SSM2 shares the bus");
    const double gap = minObdGapMs(12, 40);
    check(gap >= 49.5, "after an unanswered request the next still waits for P2CAN", fmt("min gap %.1f ms", gap));
    check(txCount(isObdReq, 12, 40) > 20, "requests continue", fmt("%llu", (unsigned long long)txCount(isObdReq, 12, 40)));
}

/** The overall request budget: an adjustable ceiling on requests per second
 *  that spreads them out and never drops a value. Off (0) by default. */
static void scBudget() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"diag_mode":2,"req_max_hz":20})";
    boot();
    runUntil(20);
    const double rate = txCount(isObdReq, 5, 20) / 15.0;
    check(rate <= 21.0, "OBD-II requests capped at req_max_hz (20/s)", fmt("%.1f/s", rate));
    check(rate >= 8.0, "...and still flowing", fmt("%.1f/s", rate));
    const double gap = minObdGapMs(5, 20);
    check(gap >= 49.0, "at least 50 ms between requests", fmt("min gap %.1f ms", gap));
    std::string info;
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "values flow", info);
    portalPost(R"({"req_max_hz":0})");
    runUntil(40);
    const double rate0 = txCount(isObdReq, 25, 40) / 15.0;
    check(rate0 > 22.0, "0 removes the cap (adjustable)", fmt("%.1f/s", rate0));
    portalPost(R"({"req_max_hz":999})");
    check(Cfg.reqMaxHz == 200, "clamped to 200/s", fmt("%u", Cfg.reqMaxHz));
    commonChecks();
}

static bool isFunctional(const twai_message_t &m) { return m.identifier == 0x7DF; }

/**
 * SUSPECT 5, functional requests. 0x7DF reaches every ECU, the transmission
 * ECU included, and it answers. With addressing on auto, one missed physical
 * probe used to switch the master to 0x7DF for good. Physical now gets several
 * tries and, once it has worked, is never given up.
 */
static void scNoFunctional() {
    ecu.obdPhysical = false;            // the engine ECU misses the first probes...
    boot();
    runUntil(2.5);
    ecu.obdPhysical = true;             // ...then answers
    runUntil(30);
    MasterStats st; masterGetStats(st);
    check(st.obdPhysical, "still addressing the engine ECU alone after a transient miss");
    check(txCount(isFunctional, 0) == 0, "not one functional (0x7DF) frame - the TCM is never addressed",
          fmt("%llu", (unsigned long long)txCount(isFunctional, 0)));
    check(masterPidSupported(0x0C), "OBD-II working");
    std::string info;
    check(freshNear(METRIC_ID_COOLANT_TEMP, truth().coolant, 3, info), "values flow", info);
    commonChecks();
}

/**
 * SUSPECTS 3 and 4, a reboot during cranking. The reset reason is recorded and
 * the settle wait holds the controller listen-only while the car's modules
 * start, so a master that browned out at cranking cannot start requesting -
 * or even ACKing - into a bus that is still coming up. An earlier boot in the
 * stored log survives to show the history.
 */
static void scCrankReset() {
    sim::resetReason = 9;                                       // ESP_RST_BROWNOUT
    simfs::files["/evlog.json"] = "[[123456,0,32,1,5]]";        // an earlier clean power-on
    loadCarConfig();
    boot();
    runUntil(0.5);
    check(logHas("boot: reset reason 9"), "the brown-out reset is logged");
    check(controller().mode == TWAI_MODE_LISTEN_ONLY, "listen-only while the bus settles after it");
    runUntil(9.5);
    check(txCount(anyFrame, 0, 9.5) == 0, "nothing transmitted for the settle wait",
          fmt("%llu", (unsigned long long)txCount(anyFrame, 0, 9.5)));
    runUntil(15);
    check(controller().mode == TWAI_MODE_NORMAL && txCount(isObdReq, 10.5, 15) > 0, "requests after the wait");
    static EvView ev[64];
    const size_t n = masterEventLog(ev, 64);
    check(n >= 4 && ev[0].type == EVT_BOOT && ev[0].a == 1, "the earlier boot loaded from flash first",
          fmt("%zu records, first type %u reason %u", n, n ? ev[0].type : 99, n ? ev[0].a : 0));
    uint16_t reason = 0;
    const int boots = evCount(EVT_BOOT, &reason);
    check(boots == 2 && reason == 9, "this boot recorded as a brown-out", fmt("%d boots, last reason %u", boots, reason));
    check(evCount(EVT_BUS_UP) == 1, "the bus coming up is recorded");
    uint16_t waited = 0;
    const int settled = evCount(EVT_SETTLE_END, &waited);
    check(settled == 1 && waited == 10, "the end of the settle wait is recorded", fmt("%d, %u s", settled, waited));
    JsonDocument d;
    check(!deserializeJson(d, simfs::files["/evlog.json"]) && d.size() == n, "all of it written to flash",
          fmt("%u records in the file", (unsigned)d.size()));
    commonChecks();
}

/** The evidence log around a transmit-side guard trip, and clearing it. */
static void scEvlog() {
    boot();
    runUntil(20);
    faults.errorPerOwnFrame = 0.5;
    runUntil(30);
    faults.errorPerOwnFrame = 0;
    uint16_t reason = 0;
    const int boots = evCount(EVT_BOOT, &reason);
    check(boots == 1 && reason == 1, "boot recorded as a power-on", fmt("%d boots, reason %u", boots, reason));
    uint16_t trip = 0; uint8_t act = 0;
    const int trips = evCount(EVT_TX_TRIP, &trip, &act);
    check(trips >= 1 && (act & 0x01), "the bus-guard trip is recorded with OBD-II polling at the time",
          fmt("%d trips, last trip %u, activity 0x%02X", trips, trip, act));
    JsonDocument d;
    check(!deserializeJson(d, simfs::files["/evlog.json"]) && d.size() >= 3, "written to flash", fmt("%u records", (unsigned)d.size()));
    masterEventLogClear();
    runUntil(35);
    check(evCount(EVT_BOOT) == 0 && evCount(EVT_TX_TRIP) == 0, "Clear empties it");
    JsonDocument e;
    check(!deserializeJson(e, simfs::files["/evlog.json"]) && e.size() == 0, "...on flash too");
}

/** The CAN TX line: driven recessive the instant the firmware starts, and
 *  latched recessive across deep sleep (tx_hold). */
static void scTxHold() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"sleep_idle_s":20})";
    boot();
    runUntil(2);
    check(txConfiguredOut && txDrivenHigh, "CAN TX driven recessive at boot, before the driver");
    check(gpioHoldEnabled == 0, "no latch while running - the controller owns the pin");
    runUntil(15);
    faults.silenceBus = true;
    ecu.obdEnabled = ecu.ssmEnabled = ecu.tcmEnabled = false;
    runUntil(60);
    check(sleepEntered, "slept");
    check(gpioHoldEnabled > 0 && txDrivenHigh, "TX latched recessive for the whole sleep");
    check(evCount(EVT_SLEEP) == 1, "sleep recorded in the evidence log");
}

/** ...and the latch is the owner's to turn off. */
static void scTxHoldOff() {
    simfs::files["/config.json"] = R"({"cfg_ver":5,"sleep_idle_s":20,"tx_hold":false})";
    boot();
    runUntil(15);
    check(txConfiguredOut && txDrivenHigh, "the boot-time recessive drive is unconditional");
    faults.silenceBus = true;
    ecu.obdEnabled = ecu.ssmEnabled = ecu.tcmEnabled = false;
    runUntil(60);
    check(sleepEntered && gpioHoldEnabled == 0, "tx_hold off: no latch across sleep (adjustable)");
}

struct Scenario { const char *name; void (*fn)(); double budgetS; };
static const Scenario SCENARIOS[] = {
    {"auto", scAuto, 0}, {"silent", scSilent, 0}, {"mode_switch", scModeSwitch, 0},
    {"guard", scGuard, 0}, {"guard_off", scGuardOff, 0}, {"idle_errors", scIdleErrors, 0},
    {"bus_off", scBusOff, 0}, {"ssm_bad_address", scSsmBadAddress, 0},
    {"ssm_size_limit", scSsmSizeLimit, 0}, {"obd_functional", scObdFunctional, 0},
    {"obd_drops", scObdDrops, 0}, {"config_migration", scConfigMigration, 0},
    {"config_faults", scConfigFaults, 0}, {"config_tmp_only", scConfigTmpOnly, 0},
    {"sleep", scSleep, 0}, {"clock_wrap", scClockWrap, 0}, {"both", scBoth, 0},
    {"obd_only", scObdOnly, 0}, {"ecu_off", scEcuOff, 0}, {"ssm_stmin", scSsmStmin, 0},
    {"ssm_stmin_reserved", scSsmStminReserved, 0}, {"ssm_cut", scSsmCut, 0}, {"ssm_dead", scSsmDead, 0},
    {"portal_actions", scPortalActions, 0}, {"census_full", scCensusFull, 0}, {"rx_flood", scRxFlood, 0},
    {"timing_250", scTiming250, 0}, {"timing_125", scTiming125, 0}, {"timing_500", scTiming500, 0},
    {"timing_1m", scTiming1M, 0}, {"sleep_held", scSleepHeld, 0},
    {"settle", scSettle, 0}, {"settle_restart", scSettleRestart, 0}, {"settle_portal", scSettlePortal, 0},
    {"device_config", scDeviceConfig, 0},
    {"tcm_p1718", scTcmP1718, 0}, {"tcm_p1718_optout", scTcmP1718Optout, 0},
    {"rx_guard_trickle", scRxGuardTrickle, 0}, {"rx_guard_moderate", scRxGuardModerate, 0},
    {"rx_guard_prompt", scRxGuardPrompt, 0},
    {"lom_erratum_silent", scLomErratumSilent, 0}, {"lom_erratum_drive", scLomErratumDrive, 0},
    {"normal_confinement", scNormalConfinement, 0},
    {"lom_rec_drift", scLomRecDrift, 0}, {"passive_marginal", scPassiveMarginal, 0},
    {"passive_off", scPassiveOff, 0}, {"passive_busoff", scPassiveBusOff, 0},
    {"error_frames_never", scErrorFramesNever, 0}, {"bus_defaults", scBusDefaults, 0},
    {"link_misses", scLinkMisses, 0}, {"link_radio", scLinkRadio, 0},
    {"no_driver_retry", scNoDriverRetry, 0}, {"tec_write_not_ours", scTecWriteNotOurs, 0},
    {"link_bursts", scLinkBursts, 0}, {"timing_default", scTimingDefault, 0},
    {"portal_ap_safe", scPortalApSafe, 0}, {"portal_rescue", scPortalRescue, 0},
    {"timing_idf", scTimingIdf, 0}, {"timing_sp875_legacy", scTimingSp875Legacy, 0},
    {"error_kinds", scErrorKinds, 0}, {"radio_power", scRadioPower, 0}, {"busoff_guard_off", scBusOffGuardOff, 0},
    {"pacing_p2can", scPacingP2can, 0}, {"pacing_p2can_auto", scPacingP2canAuto, 0},
    {"budget", scBudget, 0}, {"no_functional", scNoFunctional, 0},
    {"crank_reset", scCrankReset, 0}, {"evlog", scEvlog, 0},
    {"tx_hold", scTxHold, 0}, {"tx_hold_off", scTxHoldOff, 0},
};

static const Scenario *s_sc = nullptr;

static void mainTask() {
    s_sc->fn();
    std::printf("%s: %d/%d checks passed\n", s_sc->name, s_checks - s_fails, s_checks);
    if (s_fails && !sim::verbose) {
        std::printf("--- last log lines ---\n");
        const size_t from = logLines.size() > 40 ? logLines.size() - 40 : 0;
        for (size_t i = from; i < logLines.size(); i++) std::printf("%s\n", logLines[i].c_str());
    }
    std::fflush(stdout);
    std::_Exit(s_fails ? 1 : 0);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::printf("usage: sim <scenario> [-v]\n");
        for (const auto &s : SCENARIOS) std::printf("  %s\n", s.name);
        return 2;
    }
    for (const auto &s : SCENARIOS) if (!std::strcmp(s.name, argv[1])) s_sc = &s;
    if (!s_sc) { std::printf("unknown scenario %s\n", argv[1]); return 2; }
    sim::verbose = argc > 2 && !std::strcmp(argv[2], "-v");
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    simrtos::startWatchdog(20);
    simrtos::runMain(mainTask, 1);
    return 0;
}

/* Debug aid: what the master transmitted in a window, by kind. */
void dumpTx(double fromS, double toS) {
    std::map<std::string, int> k;
    for (const auto &r : sim::transmitted) {
        const double t = tOf(r.tUs);
        if (t < fromS || t >= toS) continue;
        const auto &m = r.msg;
        std::string key;
        if (isObd(m)) key = fmt("OBD pid %02X", m.data[2]);
        else if (m.identifier == 0x7E0) {
            const uint8_t pci = m.data[0] & 0xF0;
            key = pci == 0x00 ? fmt("SSM SF %02X", m.data[1]) : pci == 0x10 ? "SSM FF" : pci == 0x20 ? "SSM CF" : "SSM FC";
        } else key = fmt("other %03X", m.identifier);
        k[key]++;
    }
    for (auto &e : k) std::printf("    %-14s %.1f/s\n", e.first.c_str(), e.second / (toS - fromS));
}
