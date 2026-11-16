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
        for (size_t i = 1; i < events; i++)
            if (ev[i].ms < ev[i - 1].ms && ev[i - 1].ms - ev[i].ms < 0x7FFFFFFF) bad("events: out of order", i);

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
    const Car c = carNow();
    twai_message_t r = {};
    r.identifier = 0x7E8;
    r.data_length_code = 8;
    r.data[1] = 0x41; r.data[2] = pid;
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
void __wrap__ZN9WebPortal5beginEv(void *) {
    Serial.println("[qemu ] no radio here: the portal and the Wi-Fi are not started");
#if QEMU_READERS
    xTaskCreatePinnedToCore(readerTask, "readers", 6144, nullptr, 1, nullptr, 0);
#endif
}
void __wrap__ZN9WebPortal4loopEv(void *) {}
bool __wrap__ZN16WiFiGenericClass4modeE11wifi_mode_t(void *, int) { return true; }
bool __wrap__ZN12WiFiSTAClass10disconnectEbb(void *, bool, bool) { return true; }

}  // extern "C"
