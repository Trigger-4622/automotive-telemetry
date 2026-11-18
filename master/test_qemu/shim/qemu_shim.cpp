/**
 * @file qemu_shim.cpp
 * @brief What Espressif's QEMU does not emulate, for the test build only
 *        (test_qemu/platformio.ini puts these in with -Wl,--wrap).
 *
 * QEMU's esp32s3 machine runs the real firmware - both cores, FreeRTOS, the
 * heap, the stacks, the flash - but it has no Wi-Fi radio (starting it never
 * returns) and its CAN controller is a block of registers that read 0. So:
 *
 *  - the radio is left out: the portal never starts, an ESP-NOW packet is
 *    counted and reported as sent;
 *  - twai_receive() hands the receive task the frames of a car that is not
 *    there - about 25 identifiers at their own rates, a few of them carrying
 *    values that move (engine speed, road speed, pedal, coolant) - and
 *    twai_transmit() answers the OBD-II requests an ECU would, with the same
 *    values, so the learner has something to find.
 *
 * Nothing in src/ knows about any of this. What a run shows is what the real
 * code does on the real instruction set with the real allocator: stack
 * headroom, heap figures (WITHOUT Wi-Fi's share, some 50-60 KB on the device),
 * two cores sharing the tables. It says nothing about the bus's electrics,
 * the controller's error handling or the radio.
 */
#include <Arduino.h>
#include <driver/twai.h>
#include <esp_now.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#ifndef QEMU_EXTRA_IDS
#define QEMU_EXTRA_IDS 0          /**< More identifiers, 20 Hz each (a busier bus). */
#endif
#ifndef QEMU_READERS
#define QEMU_READERS 0            /**< 1: the task that reads the tables as the portal does. */
#endif
#ifndef QEMU_REPLAY
#define QEMU_REPLAY 0             /**< 1: a recorded log (shim/replay.bin) instead of the made-up car. */
#endif
#ifndef QEMU_SETTINGS
#define QEMU_SETTINGS 0           /**< 1: a settings file (shim/settings.json) applied at boot. */
#endif
#ifndef QEMU_REPLAY_ONCE
#define QEMU_REPLAY_ONCE 0        /**< 1: the recorded drive once, then a quiet bus (the car off). */
#endif
#ifndef QEMU_OTA
#define QEMU_OTA 0                /**< 1: the firmware update over Wi-Fi, twice, and a rollback. */
#endif
#ifndef QEMU_UPLOAD
#define QEMU_UPLOAD 0             /**< 1: the settings file uploaded at falling free heap, both ways. */
#endif
#if QEMU_UPLOAD && !QEMU_SETTINGS
#error "QEMU_UPLOAD uploads the settings file: it needs QEMU_SETTINGS"
#endif

#if QEMU_READERS
#include "Learner.h"
#include "MasterTelemetry.h"
#endif

/* ═══════════════════════════════ the car ═════════════════════════════════ */

namespace {

/** Values that hold a level for a few seconds, then step: what the learner
 *  fits on (steady samples at several levels, up and down). */
struct Car {
    float rpm, speed, pedal, coolant;
};

Car carNow() {
    static const float RPM[]   = {800, 1500, 2600, 3400, 2100, 1200, 3000, 900};
    static const float SPEED[] = {0, 22, 47, 71, 38, 15, 60, 5};
    static const float PEDAL[] = {0, 18, 42, 65, 30, 10, 55, 4};
    const uint64_t ms = (uint64_t)(esp_timer_get_time() / 1000);
    const size_t step = (size_t)((ms / 4000) % 8);
    Car c;
    c.rpm     = RPM[step];
    c.speed   = SPEED[step];
    c.pedal   = PEDAL[step];
    c.coolant = 20.0f + (float)((ms / 3000) % 70);          // warms up, a degree every 3 s
    return c;
}

struct Frame { uint16_t id; uint16_t periodMs; uint8_t kind; };

/** kind: 1 rpm + throttle, 2 pedal, 3 speed, 4 coolant, 5 fuel level,
 *        0 a body frame: a counter, a slow switch, bytes that stay put. */
const Frame FRAMES[] = {
    {0x231, 20, 1}, {0x232, 20, 2}, {0x333, 20, 3}, {0x451, 100, 4}, {0x705, 1000, 5},
    {0x002, 10, 0}, {0x0D1, 10, 0}, {0x0D2, 10, 0}, {0x0D3, 10, 0},
    {0x140, 20, 0}, {0x141, 20, 0}, {0x144, 20, 0}, {0x152, 20, 0}, {0x156, 20, 0}, {0x252, 20, 0},
    {0x253, 50, 0}, {0x331, 50, 0}, {0x351, 50, 0}, {0x354, 50, 0}, {0x360, 50, 0},
    {0x431, 100, 0}, {0x432, 100, 0}, {0x4B1, 100, 0}, {0x4C1, 100, 0}, {0x706, 1000, 0},
};
constexpr size_t N_BASE  = sizeof(FRAMES) / sizeof(FRAMES[0]);
constexpr size_t N_TOTAL = N_BASE + QEMU_EXTRA_IDS;

uint64_t s_due[N_BASE + QEMU_EXTRA_IDS + 1];   // µs; 0 = not yet scheduled
uint32_t s_sent[N_BASE + QEMU_EXTRA_IDS + 1];

/** Answers of the ECU waiting to go out (twai_transmit puts them here). */
struct Reply { uint64_t atUs; twai_message_t msg; };
Reply    s_reply[4];
volatile uint8_t s_replyHead = 0, s_replyTail = 0;
portMUX_TYPE s_replyMux = portMUX_INITIALIZER_UNLOCKED;

void fill(size_t i, twai_message_t &m) {
    memset(&m, 0, sizeof m);
    m.data_length_code = 8;
    const uint32_t n = s_sent[i]++;
    const uint64_t ms = (uint64_t)(esp_timer_get_time() / 1000);
    if (i >= N_BASE) {                                   // the busier bus
        m.identifier = 0x500 + (uint32_t)(i - N_BASE);
        m.data[0] = (uint8_t)(i - N_BASE);
        m.data[1] = (uint8_t)n;
        return;
    }
    const Frame &f = FRAMES[i];
    const Car c = carNow();
    m.identifier = f.id;
    switch (f.kind) {
        case 1: {
            const uint16_t r = (uint16_t)c.rpm;
            m.data[0] = (uint8_t)r; m.data[1] = (uint8_t)(r >> 8);
            m.data[2] = (uint8_t)(c.pedal * 2.55f);
            break;
        }
        case 2: m.data[0] = (uint8_t)(c.pedal * 2.55f); m.data[1] = (uint8_t)(c.pedal * 2.0f); break;
        case 3: {
            const uint16_t v = (uint16_t)(c.speed * 100.0f);
            m.data[0] = (uint8_t)v; m.data[1] = (uint8_t)(v >> 8);
            break;
        }
        case 4: m.data[0] = (uint8_t)(c.coolant + 40.0f); break;
        case 5: m.data[0] = 140; break;
        default:
            m.data[0] = (uint8_t)(f.id >> 3);            // stays put
            m.data[3] = (uint8_t)((ms / 700) & 1) << (f.id & 7);   // a lamp, a switch
            break;
    }
    m.data[7] = (uint8_t)(n & 0x0F);                     // the rolling counter body frames carry
}

}  // namespace

/* ═══════════════════════════ a recorded car ══════════════════════════════
 *
 * run.py --replay turns a candump log ("(time) can0 140#0102..") into
 * shim/replay.bin, kept small enough for a 40-minute drive to fit the image:
 *   "RPL2" u16 identifiers u16 0, then per identifier u16 id u8 dlc u8 0,
 *   then a record a frame: u8 identifier index, u8 time since the last frame
 *   in 50 us steps (255: a u32 of them follows), u8 mask of the payload bytes
 *   that changed since that identifier's last frame, then those bytes.
 * It is played at its own pace and started again at its end. The engine
 * computer's answers are read off the frames just played, so what the learner
 * is asked to find is really in the traffic: engine speed and the pedal in
 * 0x140, road speed in 0x0D1, coolant in 0x360 - the layout of Subaru's
 * 2005-2019 cars (opendbc's subaru_forester_2017 / outback_2015).
 */
#if QEMU_REPLAY
extern "C" const uint8_t replay_start[] asm("replay_start");
extern "C" const uint8_t replay_end[]   asm("replay_end");
asm(".section .rodata.replay,\"a\"\n.balign 4\n.global replay_start\nreplay_start:\n"
    ".incbin \"shim/replay.bin\"\n.global replay_end\nreplay_end:\n.previous\n");

namespace {
constexpr size_t RPL_MAX_IDS = 128;
const uint8_t *s_rp = nullptr, *s_records = nullptr;
uint16_t s_nIds = 0;
uint16_t s_rid[RPL_MAX_IDS];
uint8_t  s_rdlc[RPL_MAX_IDS];
uint8_t  s_rlast[RPL_MAX_IDS][8];
bool     s_rPending = false;                 // the next frame decoded, waiting for its time
twai_message_t s_rNext;
uint64_t s_rDue = 0;                         // µs: when it is due
uint32_t s_rounds = 0;
uint64_t s_played = 0;
bool     s_rEnded = false;                   // QEMU_REPLAY_ONCE: the drive is over
// What the recorded car is doing, as its own computer would report it.
volatile uint16_t s_recRpm = 0;             // rpm
volatile uint16_t s_recSpeed = 0;           // km/h x 100
volatile uint8_t  s_recPedal = 0;           // 0..255
volatile uint8_t  s_recCoolant = 40;        // degrees C + 40

void replayRewind() {
    s_rp = s_records;
    memset(s_rlast, 0, sizeof s_rlast);
}

bool replayOpen() {
    if (s_records) return true;
    const uint8_t *p = replay_start;
    if (replay_end - p < 8 || memcmp(p, "RPL2", 4)) return false;
    s_nIds = (uint16_t)(p[4] | p[5] << 8);
    if (s_nIds > RPL_MAX_IDS) return false;
    p += 8;
    for (uint16_t i = 0; i < s_nIds; i++, p += 4) {
        s_rid[i] = (uint16_t)(p[0] | p[1] << 8);
        s_rdlc[i] = p[2] > 8 ? 8 : p[2];
    }
    s_records = p;
    replayRewind();
    return true;
}

/** Decode the next frame into s_rNext and its time into s_rDue. */
void replayDecode(uint64_t now) {
#if QEMU_REPLAY_ONCE
    if (s_rp >= replay_end) {                          // the end: the car is switched off
        if (!s_rEnded) {
            s_rEnded = true;
            Serial.printf("[qemu ] the recorded drive is over (%llu frames): the bus is quiet from here\n",
                          (unsigned long long)s_played);
        }
        return;
    }
#endif
    if (s_rp >= replay_end) {                          // the end: once more from the top
        replayRewind();
        s_rDue += 20000;
        s_rounds++;
    }
    const uint8_t idx = *s_rp++;
    uint32_t dt = *s_rp++;
    if (dt == 255) { dt = s_rp[0] | s_rp[1] << 8 | s_rp[2] << 16 | (uint32_t)s_rp[3] << 24; s_rp += 4; }
    const uint8_t mask = *s_rp++;
    uint8_t *last = s_rlast[idx < RPL_MAX_IDS ? idx : 0];
    for (uint8_t k = 0; k < 8; k++)
        if (mask & (1u << k)) last[k] = *s_rp++;
    if (!s_rDue) s_rDue = now + 50000;
    s_rDue += (uint64_t)dt * 50;
    if (now > s_rDue + 100000) s_rDue = now;             // fell behind: no burst
    memset(&s_rNext, 0, sizeof s_rNext);
    s_rNext.identifier = s_rid[idx];
    s_rNext.data_length_code = s_rdlc[idx];
    memcpy(s_rNext.data, last, 8);
    s_rPending = true;
}
}
#endif

/* ═══════════════════ the car's settings, as the portal uploads them ═══════
 *
 * run.py --settings puts a settings file (the car's own, read off it) into
 * the image as shim/settings.json; at boot, right after the master has loaded
 * its own file (MasterConfig::begin), it is applied the way the portal's
 * "Everything, as JSON" upload does - MasterConfig::fromJson(.., true) and a
 * save - so the bus comes up with it, as after an upload and its restart.
 */
#if QEMU_SETTINGS
#include <ArduinoJson.h>
#include "MasterConfig.h"
extern "C" const char settings_start[] asm("settings_start");
extern "C" const char settings_end[]   asm("settings_end");
asm(".section .rodata.settings,\"a\"\n.global settings_start\nsettings_start:\n"
    ".incbin \"shim/settings.json\"\n.global settings_end\nsettings_end:\n.byte 0\n.previous\n");

static void applySettings() {
    JsonDocument d;
    const DeserializationError e = deserializeJson(d, settings_start, (size_t)(settings_end - settings_start));
    if (e) { Serial.printf("[qemu ] the settings file does not parse: %s\n", e.c_str()); return; }
    const bool ok = Cfg.fromJson(d.as<JsonVariantConst>(), true) && Cfg.save();
    Serial.printf("[qemu ] the car's settings applied as the portal's upload does: %s (%u signals)\n",
                  ok ? "saved" : "NOT saved", (unsigned)Cfg.signals.size());
}
#endif

#include "DriveLog.h"
#include "MasterPacket.h"
#include "MasterTelemetry.h"
#include "MasterConfig.h"

/* ═════════════════ the settings upload, on the real heap ══════════════════
 *
 * The owner's settings file of 2026-11-17 (45 signals) would not save from
 * the portal: "Save failed". Here it is uploaded again and again, with the car
 * running and the learner sized, each time with less of the heap left free
 * (blocks of 4 KB taken first, as the Wi-Fi's and the portal's are on the
 * board) - the way the portal took it before (the whole body in memory: the
 * server's buffer and the String it keeps, the copy arg() returns for the
 * parse, the parsed document, then the save's), and the way it does now (the
 * body to the flash in the server's 1436-byte pieces, applyFile). Neither
 * radio nor portal runs here, so what is measured is the free heap each way
 * needs, not what the board has.
 */
#if QEMU_UPLOAD
#include <LittleFS.h>
namespace {

void *s_ballast[160];
size_t s_nBallast = 0;

/** Take heap until about @p kb KB is left free; the result is what is. */
void ballastTo(size_t kb) {
    const size_t want = kb * 1024;
    while (s_nBallast < 159 && ESP.getFreeHeap() > want + 4096 + 64)
        if (!(s_ballast[s_nBallast++] = malloc(4096))) { s_nBallast--; break; }
    const size_t f = ESP.getFreeHeap();
    if (f > want + 96 && s_nBallast < 160)
        if ((s_ballast[s_nBallast] = malloc(f - want - 64))) s_nBallast++;
}
void ballastFree() {
    while (s_nBallast) free(s_ballast[--s_nBallast]);
}

/** As the portal does now: the body to the flash a piece at a time, applyFile. */
bool uploadNew(const char *body, size_t len, char *why, size_t whyLen) {
    void *server = malloc(1500);                    // the server's HTTPUpload: its 1436-byte buffer
    File f = LittleFS.open(MasterConfig::UPLOAD_PATH, "w");
    bool stored = (bool)f;
    for (size_t at = 0; stored && at < len; at += 1436) {
        masterBusStall();
        const size_t n = len - at < 1436 ? len - at : 1436;
        stored = f.write((const uint8_t *)body + at, n) == n;
    }
    if (f) f.close();
    free(server);
    if (!stored) { snprintf(why, whyLen, "not stored"); LittleFS.remove(MasterConfig::UPLOAD_PATH); return false; }
    return Cfg.applyFile(MasterConfig::UPLOAD_PATH, why, whyLen);
}

/** As the portal did before: the server keeps the body as a String (made from
 *  its buffer, both alive for a moment - and the buffer is taken whole here,
 *  where the server grows it with realloc), the handler parses arg("plain"),
 *  a copy, and the body is kept until the reply, after the save. */
bool uploadOld(const char *body, size_t len, char *why, size_t whyLen) {
    char *plain = (char *)malloc(len + 1);
    if (!plain) { snprintf(why, whyLen, "no block of %u bytes for the body: request dropped", (unsigned)len + 1); return false; }
    memcpy(plain, body, len);
    plain[len] = 0;
    String *kept = new String(plain);
    free(plain);
    if (kept->length() != len) { delete kept; snprintf(why, whyLen, "the body's String did not fit: empty body"); return false; }
    bool parsed;
    {
        JsonDocument doc;
        const DeserializationError e = deserializeJson(doc, String(*kept));
        parsed = !e && Cfg.fromJson(doc.as<JsonVariantConst>(), true);
        if (!parsed) snprintf(why, whyLen, "bad json (%s)", e.c_str());
    }
    const bool ok = parsed && Cfg.save();
    if (parsed && !ok) snprintf(why, whyLen, "fs write (the save did not fit)");
    delete kept;
    return ok;
}

/** ArduinoJson's allocations, counted: the most a document held at once. */
class Counting : public ArduinoJson::Allocator {
public:
    size_t now = 0, peak = 0;
    void *allocate(size_t n) override {
        void *p = malloc(n);
        if (p) add(heap_caps_get_allocated_size(p));
        return p;
    }
    void deallocate(void *p) override { if (p) now -= heap_caps_get_allocated_size(p); free(p); }
    void *reallocate(void *p, size_t n) override {
        const size_t was = p ? heap_caps_get_allocated_size(p) : 0;
        void *q = realloc(p, n);
        if (q) { now -= was; add(heap_caps_get_allocated_size(q)); }
        return q;
    }
private:
    void add(size_t n) { now += n; if (now > peak) peak = now; }
};

/** Run from the loop task, as the portal's handler is: never beside a save
 *  of its own (requestSave() is served there too). */
void uploadMeasure() {
    const size_t len = (size_t)(settings_end - settings_start);
    {
        Counting parse, save;
        {
            JsonDocument d(&parse);
            deserializeJson(d, settings_start, len);
        }
        Cfg.jsonAlloc = &save;
        Cfg.save();
        Cfg.jsonAlloc = nullptr;
        Serial.printf("[upld ] the file parsed holds %u bytes of the heap; the save's document %u\n",
                      (unsigned)parse.peak, (unsigned)save.peak);
    }
    const size_t free0 = ESP.getFreeHeap();
    Serial.printf("[upld ] the settings file, %u bytes, uploaded as the portal takes it; "
                  "%u KB free now, largest block %u KB\n", (unsigned)len, (unsigned)(free0 / 1024),
                  (unsigned)(ESP.getMaxAllocHeap() / 1024));
    int needNew = -1, needOld = -1;
    for (int kb = 112; kb >= 16; kb -= 8) {
        if ((size_t)kb * 1024 > free0) continue;
        char wn[120] = "", wo[120] = "";
        ballastTo(kb);
        const size_t fr = ESP.getFreeHeap(), bl = ESP.getMaxAllocHeap();
        const uint32_t t0 = millis();
        const bool okNew = uploadNew(settings_start, len, wn, sizeof wn);
        const uint32_t tNew = millis() - t0;
        ballastFree();
        ballastTo(kb);
        const bool okOld = uploadOld(settings_start, len, wo, sizeof wo);
        ballastFree();
        if (okNew) needNew = kb;
        if (okOld) needOld = kb;
        Serial.printf("[upld ] %3u KB free (largest block %2u KB): now %s%s%s (%u ms), before %s%s%s\n",
                      (unsigned)(fr / 1024), (unsigned)(bl / 1024),
                      okNew ? "saved" : "NOT saved", okNew ? "" : ": ", wn, (unsigned)tNew,
                      okOld ? "saved" : "NOT saved", okOld ? "" : ": ", wo);
    }
    Serial.printf("[upld ] least free heap that saved: now %d KB, before %d KB\n", needNew, needOld);
    Serial.printf("[upld ] after: %u signals, %s left on the flash\n", (unsigned)Cfg.signals.size(),
                  LittleFS.exists(MasterConfig::UPLOAD_PATH) ? "an upload" : "no upload");
}

}  // namespace
#endif

extern "C" void __real__ZN12MasterConfig5beginEv(MasterConfig *);
extern "C" void __wrap__ZN12MasterConfig5beginEv(MasterConfig *self) {
    __real__ZN12MasterConfig5beginEv(self);
#if QEMU_SETTINGS
    applySettings();
#endif
}

/* ═════════════════════ the portal's reading, without it ══════════════════
 *
 * The portal is not in this build, but what its pages read is: the census a
 * row at a time in identifier order, the metric store slot by slot, the
 * learner's rows, the stacks, the event log. This task reads them all fifty
 * times a second from core 0 - truly beside the receive task on core 1, which
 * the loop task on the board never is - and checks each pass for sense. The
 * host harness runs one task at a time and cannot show this.
 */
#if QEMU_READERS
namespace {

uint32_t s_passes = 0, s_bad = 0;
char     s_firstBad[96] = "";

void bad(const char *what, uint32_t a = 0, uint32_t b = 0) {
    if (!s_bad++) snprintf(s_firstBad, sizeof s_firstBad, "%s (%u, %u)", what, (unsigned)a, (unsigned)b);
}

void readerTask(void *) {
    static uint32_t lastCount[128];
    static uint32_t lastId[128];
    uint32_t lastPrint = millis();
    size_t rows = 0, metrics = 0, learn = 0, events = 0;
    for (;;) {
        // The Bus page: every row, in identifier order, with its flip counters.
        uint8_t order[128];
        const size_t n = masterCensusOrder(order, sizeof order);
        if (n > 128) bad("census: more rows than there can be", n);
        bool seen[128] = {false};
        uint32_t prevId = 0;
        rows = 0;
        for (size_t i = 0; i < n && i < 128; i++) {
            if (order[i] >= 128 || seen[order[i]]) { bad("census: a row listed twice", order[i]); continue; }
            seen[order[i]] = true;
            CensusView v;
            uint8_t ed[64];
            if (!masterCensusRow(order[i], v, ed)) { bad("census: a listed row is not there", order[i]); continue; }
            if (i && v.id < prevId) bad("census: not in identifier order", v.id, prevId);
            prevId = v.id;
            if (v.dlc > 8) bad("census: a length over 8", v.id, v.dlc);
            if (!v.count) bad("census: a row with no frames", v.id);
            if (lastId[order[i]] == v.id && v.count < lastCount[order[i]])
                bad("census: a count went backwards", v.id, v.count);
            lastId[order[i]] = v.id;
            lastCount[order[i]] = v.count;
            rows++;
        }
        CensusView none;
        if (masterCensusRow(masterCensusCount() + 1, none)) bad("census: a row past the last");

        // The live page: the metric store slot by slot, the learner's rows, the stacks.
        metrics = 0;
        static uint16_t ids[192];
        for (size_t i = 0; i < masterMetricSlots(); i++) {
            MetricViewM m;
            if (!masterMetricAt(i, m)) continue;
            for (size_t k = 0; k < metrics; k++)
                if (ids[k] == m.id) bad("metrics: a value in two slots", m.id);
            if (metrics < 192) ids[metrics++] = m.id;
            if (m.value != m.value) bad("metrics: not a number", m.id);
        }
        learn = 0;
        for (size_t i = 0; i < learnerStatusRows(); i++) {
            LearnView lv;
            if (!learnerStatusRow(i, lv)) continue;
            if (!lv.metric) bad("learner: a row without a value");
            if (lv.progress < 0 || lv.progress > 1.0001f) bad("learner: progress out of range", lv.metric);
            learn++;
        }
        StackView sv[12];
        const size_t ns = masterStacks(sv, 12);
        for (size_t i = 0; i < ns; i++)
            if (!sv[i].minFree || sv[i].minFree > sv[i].size) bad("stacks: a figure that cannot be", i, sv[i].minFree);
        static EvView ev[48];
        events = masterEventLog(ev, 48);
        // In order within a boot: the log is kept across restarts, and each boot's
        // own entry (type 0) starts its clock again (run.py --flash, a second drive).
        for (size_t i = 1; i < events; i++)
            if (ev[i].type != 0 && ev[i].ms < ev[i - 1].ms && ev[i - 1].ms - ev[i].ms < 0x7FFFFFFF)
                bad("events: out of order", i);

        s_passes++;
        if (millis() - lastPrint >= 10000) {
            lastPrint = millis();
            Serial.printf("[read ] %u passes | census %u rows, metrics %u, learner rows %u, events %u | "
                          "%u inconsistencies%s%s | stack left %u\n",
                          (unsigned)s_passes, (unsigned)rows, (unsigned)metrics, (unsigned)learn,
                          (unsigned)events, (unsigned)s_bad, s_bad ? ": " : "", s_firstBad,
                          (unsigned)uxTaskGetStackHighWaterMark(nullptr));
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

}  // namespace
#endif

/* ═════════════════════ a firmware update, as the portal does it ══════════
 *
 * run.py --ota: the board's own partition table (two app slots, the 8 MB logs
 * partition), and from the loop task - where the portal's upload handler runs
 * - this firmware's own image read out of the running slot and handed to
 * fwUpdateBegin / Write / End in the 1436-byte pieces Arduino's WebServer
 * delivers, then masterBeforeRestart() and ESP.restart() as /api/update does.
 * What follows is the real bootloader's doing, with the real otadata:
 *   boot 1 (app0)          once two learned values are verified and saved and
 *                          the car stands still (refused while it moves): update
 *   boot 2 (app1, trial)   kept after 30 s with the portal up; then update again
 *   boot 3 (app0, trial)   a crash 15 s in, before it is kept
 *   boot 4 (app1)          the bootloader rolled it back: reported, test over
 * The portal is never really started here; it is marked as up (its one member)
 * so that a firmware on trial can be kept, as on the board.
 */
#if QEMU_OTA
#include <esp_ota_ops.h>
#include <esp_image_format.h>
#include "FwUpdate.h"
#include "MasterConfig.h"
#include "MasterTelemetry.h"
#include "WebPortal.h"
static_assert(sizeof(WebPortal) == sizeof(bool), "the shim marks the portal as up through its one member");

namespace {
int      s_otaPhase = -1;
uint32_t s_otaTick = 0;
bool     s_otaRefusalSeen = false;

unsigned verifiedSignals() {
    unsigned n = 0;
    Cfg.lock();
    for (const auto &sg : Cfg.signals) n += sg.mode == SIG_VERIFIED;
    Cfg.unlock();
    return n;
}

/** Hand the running image to the update, as the portal's upload handler does. */
void otaUpload(const char *what) {
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_pos_t pos = { run->address, run->size };
    esp_image_metadata_t md = {};
    if (esp_image_get_metadata(&pos, &md) != ESP_OK) {
        Serial.println("[ota  ] cannot read the running image");
        s_otaPhase = 9;
        return;
    }
    static uint8_t buf[1436];                       // HTTP_UPLOAD_BUFLEN
    char why[96] = "";
    const uint32_t t0 = millis();
    bool ok = fwUpdateBegin(why, sizeof why);
    uint32_t pieces = 0;
    for (uint32_t off = 0; ok && off < md.image_len; off += sizeof buf, pieces++) {
        const size_t n = md.image_len - off < sizeof buf ? md.image_len - off : sizeof buf;
        esp_partition_read(run, off, buf, n);
        ok = fwUpdateWrite(buf, n, why, sizeof why);
    }
    if (ok) ok = fwUpdateEnd(why, sizeof why);
    Serial.printf("[ota  ] %s: %u bytes from %s in %u pieces, %u ms: %s\n", what, (unsigned)md.image_len,
                  run->label, (unsigned)pieces, (unsigned)(millis() - t0), ok ? "checked - restarting into it" : why);
    if (!ok) { s_otaPhase = 9; return; }
    masterBeforeRestart();
    delay(300);
    ESP.restart();
}

void otaStep() {
    if (millis() - s_otaTick < 500) return;
    s_otaTick = millis();
    FwStatus st;
    if (s_otaPhase < 0) {
        fwStatus(st);
        const bool app1 = !strcmp(st.slot, "app1");
        s_otaPhase = st.rolledBack ? 4 : st.trial ? (app1 ? 2 : 3) : 1;
        static const char *const SAY[] = { "", "first boot: updating once two learned values are saved",
            "the update, on trial: waiting for it to be kept, then a second update",
            "the second update, on trial: it will crash before it is kept",
            "after the crash: rolled back by the bootloader" };
        Serial.printf("[ota  ] boot in %s, %u verified signals in the settings - %s\n", st.slot,
                      verifiedSignals(), SAY[s_otaPhase]);
    }
    char why[48] = "";
    switch (s_otaPhase) {
        case 1:
        case 5:
            if (s_otaPhase == 1 && (millis() < 60000 || verifiedSignals() < 2)) return;
            if (!fwUpdateAllowed(why, sizeof why)) {
                if (!s_otaRefusalSeen) Serial.printf("[ota  ] refused while moving: %s\n", why);
                s_otaRefusalSeen = true;
                return;
            }
            otaUpload(s_otaPhase == 1 ? "first update" : "second update");
            break;
        case 2:
            fwStatus(st);
            if (st.trial) return;
            Serial.printf("[ota  ] kept after %u s on trial\n", (unsigned)(millis() / 1000));
            s_otaPhase = 5;
            break;
        case 3:
            if (millis() < 15000) return;
            Serial.println("[ota  ] crashing on purpose, before the trial is over");
            Serial.flush();
            abort();
        case 4:
            fwStatus(st);
            Serial.printf("[ota  ] test over: running %s, rolled back %s, on trial %s\n", st.slot,
                          st.rolledBack ? "yes" : "no", st.trial ? "yes" : "no");
            s_otaPhase = 9;
            break;
        default:
            break;
    }
}
}  // namespace
#endif

/* ════════════════════════════ the clock ═══════════════════════════════════
 *
 * The board's system timer starts again from 0 at every reset (ESP-IDF 4.4's
 * esp_restart_noos resets it, SYSTEM_SYSTIMER_RST; a hardware reset does
 * too). QEMU's runs on through a software restart, so millis() came up after
 * a restart already past every "since boot" wait. Each boot's clock starts at
 * its first reading here instead, as on the board. IRAM: read from interrupts.
 */
extern "C" int64_t __real_esp_timer_get_time(void);
extern "C" IRAM_ATTR int64_t __wrap_esp_timer_get_time(void) {
    static int64_t s_bootUs = -1;
    const int64_t t = __real_esp_timer_get_time();
    if (s_bootUs < 0) s_bootUs = t;
    return t - s_bootUs;
}

/* ═══════════════════════════ the CAN controller ══════════════════════════ */

extern "C" {

esp_err_t __wrap_twai_receive(twai_message_t *message, TickType_t ticks_to_wait) {
    const uint64_t start = (uint64_t)esp_timer_get_time();
    const uint64_t limit = start + (uint64_t)ticks_to_wait * portTICK_PERIOD_MS * 1000ULL;
    for (;;) {
        const uint64_t now = (uint64_t)esp_timer_get_time();
        // An answer of the ECU that is due goes first.
        bool have = false;
        portENTER_CRITICAL(&s_replyMux);
        if (s_replyTail != s_replyHead && s_reply[s_replyTail].atUs <= now) {
            *message = s_reply[s_replyTail].msg;
            s_replyTail = (uint8_t)((s_replyTail + 1) % 4);
            have = true;
        }
        portEXIT_CRITICAL(&s_replyMux);
        if (have) return ESP_OK;

#if QEMU_REPLAY
        if (!replayOpen()) return ESP_ERR_TIMEOUT;
        if (!s_rPending) replayDecode(now);
        if (s_rDue <= now) {
            *message = s_rNext;
            s_rPending = false;
            s_played++;
            const uint8_t *d = message->data;
            if (message->identifier == 0x140 && message->data_length_code >= 4) {
                s_recPedal = d[0];
                s_recRpm   = (uint16_t)(((d[3] & 0x3F) << 8) | d[2]);
            } else if (message->identifier == 0x0D1 && message->data_length_code >= 2) {
                s_recSpeed = (uint16_t)((d[0] | (d[1] << 8)) * 5.625f);
            } else if (message->identifier == 0x360 && message->data_length_code >= 4) {
                s_recCoolant = d[3];
            }
            return ESP_OK;
        }
        if (now >= limit) return ESP_ERR_TIMEOUT;
        vTaskDelay(1);
        continue;
#endif

        size_t best = 0;
        uint64_t bestDue = UINT64_MAX;
        for (size_t i = 0; i < N_TOTAL; i++) {
            if (!s_due[i]) s_due[i] = now + 1000ULL * (50 + 7 * i);   // spread over the first moments
            if (s_due[i] < bestDue) { bestDue = s_due[i]; best = i; }
        }
        if (bestDue <= now) {
            const uint32_t periodUs = 1000UL * (best < N_BASE ? FRAMES[best].periodMs : 50);
            s_due[best] += periodUs;
            if (s_due[best] + 2ULL * periodUs < now) s_due[best] = now;   // fell behind: no backlog
            fill(best, *message);
            return ESP_OK;
        }
        if (now >= limit) return ESP_ERR_TIMEOUT;
        vTaskDelay(1);
    }
}

/** A frame of ours. An OBD-II service 01 request is answered as the engine
 *  computer would, 8 ms later, from the same values the frames carry. */
esp_err_t __wrap_twai_transmit(const twai_message_t *m, TickType_t) {
    if (!m || m->extd || (m->identifier != 0x7DF && m->identifier != 0x7E0)) return ESP_OK;
    if (m->data_length_code < 3 || m->data[0] != 2 || m->data[1] != 0x01) return ESP_OK;
    const uint8_t pid = m->data[2];
    twai_message_t r = {};
    r.identifier = 0x7E8;
    r.data_length_code = 8;
    r.data[1] = 0x41; r.data[2] = pid;
#if QEMU_REPLAY
    switch (pid) {      // coolant, engine speed, road speed, accelerator pedal: what the frames say
        case 0x00: r.data[0] = 6; r.data[3] = 0x08; r.data[4] = 0x18; r.data[5] = 0x00; r.data[6] = 0x01; break;
        case 0x20: r.data[0] = 6; r.data[6] = 0x01; break;
        case 0x40: r.data[0] = 6; r.data[4] = 0x80; break;
        case 0x05: r.data[0] = 3; r.data[3] = s_recCoolant; break;
        case 0x0C: { const uint16_t v = (uint16_t)(s_recRpm * 4u);
                     r.data[0] = 4; r.data[3] = (uint8_t)(v >> 8); r.data[4] = (uint8_t)v; break; }
        case 0x0D: r.data[0] = 3; r.data[3] = (uint8_t)(s_recSpeed / 100u); break;
        case 0x49: r.data[0] = 3; r.data[3] = s_recPedal; break;
        default: return ESP_OK;
    }
#else
    const Car c = carNow();
    switch (pid) {
        case 0x00: r.data[0] = 6; r.data[3] = 0x18; r.data[4] = 0x18; r.data[5] = 0x80; r.data[6] = 0x00; break;
                   // 0x04, 0x05, 0x0C, 0x0D, 0x11
        case 0x04: r.data[0] = 3; r.data[3] = (uint8_t)(c.pedal * 2.55f); break;
        case 0x05: r.data[0] = 3; r.data[3] = (uint8_t)(c.coolant + 40.0f); break;
        case 0x0C: { const uint16_t v = (uint16_t)(c.rpm * 4.0f);
                     r.data[0] = 4; r.data[3] = (uint8_t)(v >> 8); r.data[4] = (uint8_t)v; break; }
        case 0x0D: r.data[0] = 3; r.data[3] = (uint8_t)c.speed; break;
        case 0x11: r.data[0] = 3; r.data[3] = (uint8_t)(c.pedal * 2.55f); break;
        default: return ESP_OK;                          // not a PID this ECU has: no answer
    }
#endif
    portENTER_CRITICAL(&s_replyMux);
    const uint8_t next = (uint8_t)((s_replyHead + 1) % 4);
    if (next != s_replyTail) {
        s_reply[s_replyHead] = { (uint64_t)esp_timer_get_time() + 8000ULL, r };
        s_replyHead = next;
    }
    portEXIT_CRITICAL(&s_replyMux);
    return ESP_OK;
}

/* ═══════════════════════════════ the radio ═══════════════════════════════ */

static esp_now_send_cb_t s_sendCb = nullptr;

esp_err_t __wrap_esp_now_init(void) { return ESP_OK; }
esp_err_t __wrap_esp_now_add_peer(const esp_now_peer_info_t *) { return ESP_OK; }
esp_err_t __wrap_esp_now_register_send_cb(esp_now_send_cb_t cb) { s_sendCb = cb; return ESP_OK; }
esp_err_t __wrap_esp_now_send(const uint8_t *peer, const uint8_t *, size_t) {
    if (s_sendCb) s_sendCb(peer, ESP_NOW_SEND_SUCCESS);
    return ESP_OK;
}
esp_err_t __wrap_esp_wifi_set_max_tx_power(int8_t) { return ESP_OK; }
esp_err_t __wrap_esp_wifi_set_channel(uint8_t, wifi_second_chan_t) { return ESP_OK; }

/* WebPortal::begin() / loop(), WiFi.mode(), WiFi.disconnect(): never started. */
#ifdef QEMU_FLASHPROBE
#include <esp_partition.h>
#include <esp_flash.h>
static void flashProbe() {
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "spiffs");
    if (!p) { Serial.println("[probe] no spiffs partition"); return; }
    uint32_t id = 0; esp_flash_read_id(NULL, &id);
    Serial.printf("[probe] flash id %06X size %u, spiffs at 0x%X size %u\n", (unsigned)id,
                  (unsigned)(esp_flash_default_chip ? esp_flash_default_chip->size : 0), (unsigned)p->address, (unsigned)p->size);
    esp_err_t e = esp_partition_erase_range(p, 0, 4096);
    Serial.printf("[probe] erase 4 KB: %s\n", esp_err_to_name(e));
    uint8_t w[16]; for (int i = 0; i < 16; i++) w[i] = (uint8_t)(0xA0 + i);
    e = esp_partition_write(p, 0, w, sizeof w);
    Serial.printf("[probe] write 16 B: %s\n", esp_err_to_name(e));
    uint8_t r[16] = {0};
    e = esp_partition_read(p, 0, r, sizeof r);
    Serial.printf("[probe] read back: %s, %s: %02X %02X %02X %02X\n", esp_err_to_name(e), memcmp(r, w, 16) ? "DIFFERENT" : "same", r[0], r[1], r[2], r[15]);
    uint8_t r2[16] = {0};
    esp_flash_read(NULL, r2, p->address, 16);
    Serial.printf("[probe] esp_flash_read: %02X %02X %02X %02X\n", r2[0], r2[1], r2[2], r2[15]);
    e = esp_flash_write(NULL, w, p->address + 4096, 16);
    uint8_t r3[16] = {0};
    esp_flash_read(NULL, r3, p->address + 4096, 16);
    Serial.printf("[probe] esp_flash_write %s then read: %02X %02X %02X %02X\n", esp_err_to_name(e), r3[0], r3[1], r3[2], r3[15]);
    Serial.printf("[probe] read mode %d\n", esp_flash_default_chip ? (int)esp_flash_default_chip->read_mode : -1);
    e = esp_partition_erase_range(p, 0, 64 * 1024);
    Serial.printf("[probe] erase 64 KB: %s\n", esp_err_to_name(e));
}
#endif

void __wrap__ZN9WebPortal5beginEv(void *self) {
#if QEMU_OTA
    *static_cast<bool *>(self) = true;              // "running": see the update test above
#else
    (void)self;
#endif
#ifdef QEMU_FLASHPROBE
    flashProbe();
#endif
    Serial.println("[qemu ] no radio here: the portal and the Wi-Fi are not started");
#if QEMU_READERS
    xTaskCreatePinnedToCore(readerTask, "readers", 6144, nullptr, 1, nullptr, 0);
#endif
}
void __wrap__ZN9WebPortal4loopEv(void *) {
#if QEMU_OTA
    otaStep();
#endif
#if QEMU_UPLOAD
    static bool uploaded = false;                   // once the learner has sized, the car running a while
    if (!uploaded && millis() > 45000) { uploaded = true; uploadMeasure(); }
#endif
    // The indicators as the screens see them: count every change of each.
    static float lastTurn[2] = {NAN, NAN};
    static uint32_t turnFlips[2] = {0, 0};
    for (int k = 0; k < 2; k++) {
        float v; uint32_t age; uint8_t src;
        if (metricLookup(k ? METRIC_ID_TURN_RIGHT : METRIC_ID_TURN_LEFT, v, age, src) && age < 1500) {
            if (!isnan(lastTurn[k]) && v != lastTurn[k]) turnFlips[k]++;
            lastTurn[k] = v;
        }
    }
    // Every value in the metric store that is fresh, once a minute.
    static uint32_t lastMets = 0;
    if (millis() - lastMets >= 60000) {
        lastMets = millis();
        Serial.printf("[mets ] turn L %u changes, turn R %u |", (unsigned)turnFlips[0], (unsigned)turnFlips[1]);
        for (size_t i = 0; i < masterMetricSlots(); i++) {
            MetricViewM m;
            if (masterMetricAt(i, m) && m.ageMs < 2000) Serial.printf(" %04X=%.3g", m.id, m.value);
        }
        Serial.println();
    }
    // The drive log as the portal's Logs card shows it, once a minute.
    static uint32_t lastLog = 0;
    if (millis() - lastLog >= 60000) {
        lastLog = millis();
        DriveLogStatus d;
        driveLogStatus(d);
        Serial.printf("[dlog ] %s %s, %u rows | %u KB of %u KB, %u KB free | %u writes%s%s\n",
                      d.recording ? "recording" : "not recording", d.file, (unsigned)d.rows,
                      (unsigned)d.usedKb, (unsigned)d.capKb, (unsigned)d.freeKb, (unsigned)d.flushes,
                      d.note[0] ? " | " : "", d.note);
    }
}
bool __wrap__ZN16WiFiGenericClass4modeE11wifi_mode_t(void *, int) { return true; }
bool __wrap__ZN12WiFiSTAClass10disconnectEbb(void *, bool, bool) { return true; }

}  // extern "C"
