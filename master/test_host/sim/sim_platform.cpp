/**
 * @file sim_platform.cpp
 * @brief FreeRTOS API, Arduino core bits, LittleFS, Wi-Fi/ESP-NOW and deep
 *        sleep for the host build. ESP-NOW packets are decoded and checked as
 *        a display would, so the tests see exactly what reaches the gauges.
 */
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "Arduino.h"
#include "LittleFS.h"
#include "MasterPacket.h"
#include "WebPortal.h"
#include "WiFi.h"
#include "driver/gpio.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "sim.h"

/* ═════════════════════════════ FreeRTOS ═════════════════════════════════ */

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t, void *arg,
                                   UBaseType_t prio, TaskHandle_t *handle, BaseType_t) {
    if (handle) *handle = nullptr;
    simrtos::createTask(fn, name, arg, (int)prio);
    return pdPASS;
}
void vTaskDelay(TickType_t ticks) { simrtos::sleepUs(simTicksUs(ticks)); }
TickType_t xTaskGetTickCount() { return (TickType_t)(simrtos::nowUs() / 1000); }

struct SimQueue {
    size_t len, item;
    std::deque<std::vector<uint8_t>> q;
    simrtos::Waitable rx, tx;
};

QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t itemSize) {
    return new SimQueue{len, itemSize, {}, {}, {}};
}
void vQueueDelete(QueueHandle_t q) { delete q; }
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q) { return (UBaseType_t)q->q.size(); }

BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks) {
    const uint64_t deadline = ticks == portMAX_DELAY ? simrtos::FOREVER : simrtos::nowUs() + simTicksUs(ticks);
    while (q->q.size() >= q->len) {
        if (ticks == 0 || simrtos::nowUs() >= deadline) return pdFALSE;
        simrtos::block(&q->tx, deadline == simrtos::FOREVER ? deadline : deadline - simrtos::nowUs());
    }
    const uint8_t *p = (const uint8_t *)item;
    q->q.emplace_back(p, p + q->item);
    simrtos::notify(&q->rx);
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t q, void *buf, TickType_t ticks) {
    const uint64_t deadline = ticks == portMAX_DELAY ? simrtos::FOREVER : simrtos::nowUs() + simTicksUs(ticks);
    while (q->q.empty()) {
        if (ticks == 0 || simrtos::nowUs() >= deadline) return pdFALSE;
        simrtos::block(&q->rx, deadline == simrtos::FOREVER ? deadline : deadline - simrtos::nowUs());
    }
    std::memcpy(buf, q->q.front().data(), q->item);
    q->q.pop_front();
    simrtos::notify(&q->tx);
    return pdTRUE;
}

struct SimSem {
    enum Kind { MUTEX, RECURSIVE, BINARY } kind;
    int count;
    const char *owner = nullptr;   // task name holding a mutex
    int depth = 0;
    simrtos::Waitable w;
};

static void semBug(const char *what) {
    simLog('E', "SIM-BUG: %s (task %s)", what, simrtos::currentName());
}

SemaphoreHandle_t xSemaphoreCreateMutex()          { return new SimSem{SimSem::MUTEX, 1}; }
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex() { return new SimSem{SimSem::RECURSIVE, 1}; }
SemaphoreHandle_t xSemaphoreCreateBinary()         { return new SimSem{SimSem::BINARY, 0}; }
void vSemaphoreDelete(SemaphoreHandle_t s) { delete s; }

static BaseType_t semTake(SemaphoreHandle_t s, TickType_t ticks) {
    const uint64_t deadline = ticks == portMAX_DELAY ? simrtos::FOREVER : simrtos::nowUs() + simTicksUs(ticks);
    while (s->count <= 0) {
        if (ticks == 0 || simrtos::nowUs() >= deadline) return pdFALSE;
        simrtos::block(&s->w, deadline == simrtos::FOREVER ? deadline : deadline - simrtos::nowUs());
    }
    s->count--;
    if (s->kind != SimSem::BINARY) { s->owner = simrtos::currentName(); s->depth = 1; }
    return pdTRUE;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks) {
    if (s->kind == SimSem::RECURSIVE) semBug("xSemaphoreTake on a recursive mutex");
    if (s->kind == SimSem::MUTEX && s->owner && !std::strcmp(s->owner, simrtos::currentName()))
        semBug("mutex taken twice by the same task (self-deadlock)");
    return semTake(s, ticks);
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t s) {
    if (s->kind == SimSem::MUTEX) {
        if (!s->owner || std::strcmp(s->owner, simrtos::currentName())) {
            semBug("mutex given by a task that does not hold it");
            return pdFALSE;
        }
        s->owner = nullptr;
    }
    if (s->kind == SimSem::BINARY && s->count > 0) return pdFALSE;   // already given
    s->count++;
    simrtos::notify(&s->w);
    return pdTRUE;
}

BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t s, TickType_t ticks) {
    if (s->owner && !std::strcmp(s->owner, simrtos::currentName())) { s->depth++; return pdTRUE; }
    return semTake(s, ticks);
}

BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t s) {
    if (!s->owner || std::strcmp(s->owner, simrtos::currentName())) {
        semBug("recursive mutex given by a task that does not hold it");
        return pdFALSE;
    }
    if (--s->depth > 0) return pdTRUE;
    s->owner = nullptr;
    s->count++;
    simrtos::notify(&s->w);
    return pdTRUE;
}

/* ═════════════════════════════ Arduino core ═════════════════════════════ */

size_t strlcpy(char *dst, const char *src, size_t size) {
    const size_t n = std::strlen(src);
    if (size) {
        const size_t c = n < size - 1 ? n : size - 1;
        std::memcpy(dst, src, c);
        dst[c] = '\0';
    }
    return n;
}

namespace sim {
int logErrors = 0, logWarnings = 0;
std::vector<std::string> logLines;
bool verbose = false;
uint32_t heapFree = 180000;
}

void simLog(char level, const char *fmt, ...) {
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    char line[600];
    std::snprintf(line, sizeof(line), "[%9.3f] %c %-9s %s", simrtos::nowUs() / 1e6, level,
                  simrtos::currentName(), msg);
    sim::logLines.push_back(line);
    if (level == 'E') sim::logErrors++;
    if (level == 'W') sim::logWarnings++;
    if (sim::verbose || level == 'E') std::printf("%s\n", line);
}

HardwareSerialMock Serial;
size_t HardwareSerialMock::printf(const char *fmt, ...) {
    char msg[1024];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (sim::verbose) std::printf("%s", msg);
    if (sim::serialOut.size() < 200000) sim::serialOut += msg;     // for the scenarios
    return n > 0 ? (size_t)n : 0;
}
size_t HardwareSerialMock::println(const char *s) { if (sim::verbose) std::printf("%s\n", s); return 0; }
size_t HardwareSerialMock::print(const char *s) { if (sim::verbose) std::printf("%s", s); return 0; }

EspClassMock ESP;
uint32_t EspClassMock::getFreeHeap() { return sim::heapFree; }
void EspClassMock::restart() {
    if (sim::restartReturns) { sim::restarts++; std::printf("*** ESP.restart() (expected)\n"); return; }
    std::printf("*** ESP.restart()\n");
    std::_Exit(5);
}
int analogRead(uint8_t) { return 2000; }

/* ─────────────── reset reason, GPIO hold, digital IO ─────────────── */

namespace sim {
int  gpioHoldEnabled = 0;
bool txDrivenHigh = false;
bool txConfiguredOut = false;
int  txGlitches = 0;
int  radioQdbm = 0;
bool bootButtonDown = false;
bool restartReturns = false;
int  restarts = 0;
std::string serialOut;
TimingView timing = {};
}

esp_reset_reason_t esp_reset_reason() { return (esp_reset_reason_t)sim::resetReason; }

esp_err_t gpio_hold_en(gpio_num_t)  { sim::gpioHoldEnabled++; return ESP_OK; }
esp_err_t gpio_hold_dis(gpio_num_t) { if (sim::gpioHoldEnabled) sim::gpioHoldEnabled--; return ESP_OK; }
void      gpio_deep_sleep_hold_en()  {}
void      gpio_deep_sleep_hold_dis() {}
esp_err_t gpio_pullup_en(gpio_num_t) { return ESP_OK; }

void pinMode(uint8_t, uint8_t mode) {
    if (mode == OUTPUT) { sim::txConfiguredOut = true; sim::txPadFromGpio(); }
}
esp_err_t gpio_set_level(gpio_num_t, uint32_t level) { sim::txDrivenHigh = level != 0; return ESP_OK; }
esp_err_t esp_wifi_set_max_tx_power(int8_t power) { sim::radioQdbm = power; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t, gpio_mode_t mode) {
    if (mode & GPIO_MODE_OUTPUT) { sim::txConfiguredOut = true; sim::txPadFromGpio(); }
    return ESP_OK;
}
void digitalWrite(uint8_t, uint8_t val) { sim::txDrivenHigh = (val == HIGH); }
int gpio_get_level(gpio_num_t pin) {
    if (pin == 0) return sim::bootButtonDown ? 0 : 1;   // pulled up, pressed = low
    return sim::txDrivenHigh ? 1 : 0;
}
int  digitalRead(uint8_t) { return sim::txDrivenHigh ? HIGH : LOW; }

/* ═════════════════════════════ LittleFS ═════════════════════════════════ */

namespace simfs {
std::map<std::string, std::string> files;
long writeBudget = -1;
bool renameOverwrites = true;
bool mountOk = true;
int  renames = 0, removes = 0;
}
LittleFSFS LittleFS;

bool LittleFSFS::begin(bool) { return simfs::mountOk; }
bool LittleFSFS::exists(const char *p) { return simfs::files.count(p) > 0; }
bool LittleFSFS::remove(const char *p) { simfs::removes++; return simfs::files.erase(p) > 0; }
bool LittleFSFS::rename(const char *from, const char *to) {
    simfs::renames++;
    if (!simfs::files.count(from)) return false;
    if (simfs::files.count(to) && !simfs::renameOverwrites) return false;
    simfs::files[to] = simfs::files[from];
    simfs::files.erase(from);
    return true;
}
File LittleFSFS::open(const char *path, const char *mode) {
    File f;
    const bool w = mode && mode[0] == 'w';
    if (!w && !simfs::files.count(path)) return f;
    f._st = std::make_shared<File::State>();
    f._st->path = path;
    f._st->writing = w;
    if (w) simfs::files[path] = "";
    else   f._st->data = simfs::files[path];
    return f;
}
int File::read() {
    if (!_st || _st->pos >= _st->data.size()) return -1;
    return (uint8_t)_st->data[_st->pos++];
}
size_t File::readBytes(char *buf, size_t len) {
    size_t n = 0;
    while (n < len) { const int c = read(); if (c < 0) break; buf[n++] = (char)c; }
    return n;
}
int File::available() { return _st ? (int)(_st->data.size() - _st->pos) : 0; }
size_t File::write(uint8_t b) { return write(&b, 1); }
size_t File::write(const uint8_t *buf, size_t len) {
    if (!_st || !_st->writing) return 0;
    size_t n = len;
    if (simfs::writeBudget >= 0) {
        n = std::min((long)len, simfs::writeBudget);
        simfs::writeBudget -= (long)n;
    }
    simfs::files[_st->path].append((const char *)buf, n);
    return n;
}
void File::close() { _st.reset(); }

/* ═════════════════════════ Wi-Fi, ESP-NOW, sleep ═════════════════════════ */

WiFiClassMock WiFi;
esp_err_t esp_wifi_set_channel(uint8_t, wifi_second_chan_t) { return ESP_OK; }

namespace sim {
std::map<uint16_t, Shown> displayed;
uint64_t espPackets = 0, espBadPackets = 0, espSeqGaps = 0;
bool sleepEntered = false;
}
static bool s_espInit = false;
static uint32_t s_lastSeq = 0;
static bool s_haveSeq = false;

esp_err_t esp_now_init() { s_espInit = true; return ESP_OK; }
esp_err_t esp_now_deinit() { s_espInit = false; return ESP_OK; }
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *) { return ESP_OK; }
static esp_now_send_cb_t s_sendCb = nullptr;
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb) { s_sendCb = cb; return ESP_OK; }

/* A display's view: validate the packet exactly as the slaves do, then merge. */
esp_err_t esp_now_send(const uint8_t *, const uint8_t *data, size_t len) {
    if (!s_espInit) return ESP_FAIL;
    if (len > 250) { sim::espBadPackets++; return ESP_FAIL; }
    sim::radioQueue(len, s_sendCb);             // on air, whatever it carries
    const MasterTelemetryPacket *p = (const MasterTelemetryPacket *)data;
    if (!telemetry_packet_valid(p, (int)len) || len != TELEMETRY_PACKET_SIZE(p->metric_count)) {
        sim::espBadPackets++;
        return ESP_OK;
    }
    if (s_haveSeq && p->sequence_id != s_lastSeq + 1) sim::espSeqGaps++;
    s_lastSeq = p->sequence_id;
    s_haveSeq = true;
    sim::espPackets++;
    for (uint8_t i = 0; i < p->metric_count; i++) {
        // Copied out first: the fields are packed (unaligned), and binding a
        // reference to one - std::map::operator[] takes the key by reference -
        // is undefined behaviour. The slaves must do the same.
        const uint16_t id = p->metrics[i].metric_id;
        const float value = p->metrics[i].value;
        const uint8_t flags = p->metrics[i].flags;
        sim::displayed[id] = { value, flags, simrtos::nowUs() };
    }
    return ESP_OK;
}

esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause() { return ESP_SLEEP_WAKEUP_UNDEFINED; }
esp_err_t esp_sleep_enable_ext0_wakeup(gpio_num_t, int) { return ESP_OK; }
void esp_deep_sleep_start() {
    sim::sleepEntered = true;
    simLog('I', "SIM: deep sleep entered");
    for (;;) simrtos::block(nullptr, simrtos::FOREVER / 2);   // the chip stops here
}

/* The portal is not part of the host build; the tests act on the same
 * functions its handlers call. */
WebPortal Portal;
void WebPortal::begin() { _running = true; }
void WebPortal::loop() {}
void WebPortal::setupRoutes() {}
