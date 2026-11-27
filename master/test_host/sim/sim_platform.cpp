/**
 * @file sim_platform.cpp
 * @brief FreeRTOS API, Arduino core bits, LittleFS, Wi-Fi/ESP-NOW and deep
 *        sleep for the host build. ESP-NOW packets are decoded and checked as
 *        a display would, so the tests see exactly what reaches the gauges.
 */
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "Arduino.h"
#include "LittleFS.h"
#include <algorithm>
#include <cstring>
#include "Update.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
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

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack, void *arg,
                                   UBaseType_t prio, TaskHandle_t *handle, BaseType_t) {
    if (handle) *handle = (TaskHandle_t)(uintptr_t)stack;   // what uxTaskGetStackHighWaterMark reports from
    sim::tasksCreated.push_back(name);
    simrtos::createTask(fn, name, arg, (int)prio);
    return pdPASS;
}
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task) { return (UBaseType_t)((uintptr_t)task / 2); }
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
std::vector<std::string> tasksCreated;
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
uint32_t EspClassMock::getMinFreeHeap() { return sim::heapFree; }
uint32_t EspClassMock::getMaxAllocHeap() { return sim::heapFree / 2; }
/* ═════════════════════════ firmware update ═════════════════════════════ */

namespace sim {
std::string otaImage;
int  otaBegins = 0;
bool otaSwitched = false, otaAborted = false;
int  otaState = ESP_OTA_IMG_UNDEFINED;          // flashed over USB: no trial
bool otaMarkedValid = false, otaLastInvalid = false;
}
UpdateClass Update;
bool UpdateClass::begin(size_t, int, int, uint8_t, const char *) {
    sim::otaBegins++;
    sim::otaImage.clear();
    sim::otaAborted = false;
    _err = 0;
    _running = true;
    return true;
}
size_t UpdateClass::write(uint8_t *data, size_t len) {
    if (!_running) return 0;
    if (sim::otaImage.empty() && len && data[0] != 0xE9) { _err = 1; return 0; }   // as the core: "Magic byte is wrong"
    sim::otaImage.append((const char *)data, len);
    return len;
}
bool UpdateClass::end(bool) {
    if (!_running) return false;
    _running = false;
    if (sim::otaImage.size() < 1024 || (uint8_t)sim::otaImage[0] != 0xE9) { _err = 2; return false; }
    sim::otaSwitched = true;
    return true;
}
void UpdateClass::abort() { _running = false; sim::otaAborted = true; }
const char *UpdateClass::errorString() { return _err == 1 ? "Magic byte is wrong" : _err ? "Image invalid" : "No Error"; }

static esp_partition_t s_app0 = {"app0"};
const esp_partition_t *esp_ota_get_running_partition() { return &s_app0; }
esp_err_t esp_ota_get_state_partition(const esp_partition_t *, esp_ota_img_states_t *st) {
    if (sim::otaState == ESP_OTA_IMG_UNDEFINED) return ESP_ERR_NOT_FOUND;   // no otadata entry
    *st = (esp_ota_img_states_t)sim::otaState;
    return ESP_OK;
}
esp_err_t esp_ota_mark_app_valid_cancel_rollback() {
    sim::otaMarkedValid = true;
    if (sim::otaState == ESP_OTA_IMG_PENDING_VERIFY) sim::otaState = ESP_OTA_IMG_VALID;
    return ESP_OK;
}
static esp_partition_t s_app1 = {"app1"};
const esp_partition_t *esp_ota_get_last_invalid_partition() { return sim::otaLastInvalid ? &s_app1 : nullptr; }
int esp_ota_get_app_elf_sha256(char *dst, size_t size) {
    const char *h = "5f3a9c0d1e2b4a6f00112233445566778899aabbccddeeff0011223344556677";
    std::snprintf(dst, size, "%s", h);
    return (int)std::strlen(dst);
}

void EspClassMock::restart() {
    if (sim::restartReturns) { sim::restarts++; std::printf("*** ESP.restart() (expected)\n"); return; }
    std::printf("*** ESP.restart()\n");
    std::_Exit(5);
}
int analogRead(uint8_t) { return 2000; }

/* ─────────────── reset reason, GPIO hold, digital IO ─────────────── */

namespace sim {
int  gpioHoldEnabled = 0;
bool txHeld = false;
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

// The pad hold is on or off, not counted: a second gpio_hold_en() changes nothing.
esp_err_t gpio_hold_en(gpio_num_t)  { sim::gpioHoldEnabled = 1; sim::txHoldChanged(true);  return ESP_OK; }
esp_err_t gpio_hold_dis(gpio_num_t) { sim::gpioHoldEnabled = 0; sim::txHoldChanged(false); return ESP_OK; }
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
int  mountFails = 0;
bool blank = false;
int  formats = 0;
std::set<std::string> held;
int  renames = 0, removes = 0;
size_t totalBytes = 1536 * 1024;
std::set<std::string> dirs;
bool   logsPartition = true;
bool   logsBlank = true;                       // as it is after the update: never formatted
int    logsMountFails = 0;
int    logsFormats = 0;
size_t logsBytes = 8192 * 1024;
std::vector<uint8_t> logsRaw;
int    flashWriteViolations = 0;
uint64_t logsErasedBytes = 0, logsWrittenBytes = 0;
std::vector<uint8_t> &logsRawBytes() {
    if (logsRaw.size() != logsBytes) logsRaw.assign(logsBytes, 0xFF);
    return logsRaw;
}
std::string logKey(const std::string &path) { return "@logs" + path; }
size_t usedBytesOf(const std::string &prefix) {
    size_t n = 2 * 4096;                       // superblocks
    for (const auto &kv : files) {
        const bool mine = prefix.empty() ? kv.first[0] != '@' : kv.first.compare(0, prefix.size(), prefix) == 0;
        if (mine) n += (kv.second.size() + 4095) / 4096 * 4096 + 4096;
    }
    return n;
}
size_t usedBytes() { return usedBytesOf(""); }
}
LittleFSFS LittleFS;

bool LittleFSFS::mkdir(const char *p) { simfs::dirs.insert(key(p)); return true; }
size_t File::size() const { return _st ? _st->data.size() : 0; }
const char *File::name() const {
    if (!_st) return "";
    const size_t k = _st->path.rfind('/');
    return _st->path.c_str() + (k == std::string::npos ? 0 : k + 1);
}
File File::openNextFile() {
    File f;
    if (!_st || !_st->dir || _st->next >= _st->list.size()) return f;
    f._st = std::make_shared<State>();
    f._st->path = _st->list[_st->next++];
    f._st->data = simfs::files[f._st->path];
    return f;
}

bool LittleFSFS::begin(bool formatOnFail, const char *, uint8_t, const char *label) {
    if (label && !std::strcmp(label, "logs")) {
        _pfx = "@logs";
        if (!simfs::logsPartition) return false;          // ESP_ERR_NOT_FOUND: never formatted
        bool fails = simfs::logsBlank;
        if (simfs::logsMountFails > 0) { simfs::logsMountFails--; fails = true; }
        if (!fails) return true;
        if (!formatOnFail) return false;
        simfs::logsFormats++;
        for (auto it = simfs::files.begin(); it != simfs::files.end();)
            it = it->first.compare(0, 5, "@logs") == 0 ? simfs::files.erase(it) : std::next(it);
        simfs::logsBlank = false;
        {   // its superblock, where a raw recording's header would be
            std::vector<uint8_t> &raw = simfs::logsRawBytes();
            std::fill(raw.begin(), raw.begin() + 8192, 0xFF);
            std::memcpy(raw.data() + 8, "littlefs", 8);
        }
        return true;
    }
    _pfx.clear();
    if (!simfs::mountOk) return false;
    bool fails = simfs::blank;
    if (simfs::mountFails > 0) { simfs::mountFails--; fails = true; }
    if (!fails) return true;
    if (!formatOnFail) return false;
    // The core formats a partition that will not mount, then mounts it.
    // The settings partition's files only: the logs' own partition is another chip area.
    simfs::formats++;
    for (auto it = simfs::files.begin(); it != simfs::files.end();)
        it = it->first[0] != '@' ? simfs::files.erase(it) : std::next(it);
    for (auto it = simfs::dirs.begin(); it != simfs::dirs.end();)
        it = (*it)[0] != '@' ? simfs::dirs.erase(it) : std::next(it);
    simfs::blank = false;
    return true;
}
bool LittleFSFS::exists(const char *p) { return simfs::files.count(key(p)) > 0; }
bool LittleFSFS::remove(const char *p) {
    simfs::removes++;
    if (simfs::held.count(key(p))) return false;
    return simfs::files.erase(key(p)) > 0;
}
bool LittleFSFS::rename(const char *from, const char *to) {
    simfs::renames++;
    if (!simfs::files.count(key(from))) return false;
    if (simfs::files.count(key(to)) && !simfs::renameOverwrites) return false;
    simfs::files[key(to)] = simfs::files[key(from)];
    simfs::files.erase(key(from));
    return true;
}
File LittleFSFS::open(const char *p, const char *mode) {
    File f;
    const std::string path = key(p);
    const bool w = mode && mode[0] == 'w', a = mode && mode[0] == 'a';
    if (!w && !a && !simfs::files.count(path)) {
        // A directory: a path made with mkdir, or one files live under.
        const std::string pre = path + "/";
        std::vector<std::string> list;
        for (const auto &kv : simfs::files)
            if (kv.first.compare(0, pre.size(), pre) == 0 && kv.first.find('/', pre.size()) == std::string::npos)
                list.push_back(kv.first);
        if (list.empty() && !simfs::dirs.count(path)) return f;
        f._st = std::make_shared<File::State>();
        f._st->path = path;
        f._st->dir = true;
        f._st->list = list;
        return f;
    }
    f._st = std::make_shared<File::State>();
    f._st->path = path;
    f._st->writing = w || a;
    if (w) simfs::files[path] = "";
    else if (a) simfs::files[path];            // created if missing, kept if not
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
uint64_t espPackets = 0, espBadPackets = 0, espSeqGaps = 0, espLost = 0;
uint16_t watchMetric = 0;
std::vector<Seen> watchLog;
bool sleepEntered = false;
}
/** Which packets the displays miss: its own generator, so a lossy radio does
 *  not reshuffle the car's random traffic. */
static std::mt19937 s_lossRng(4242);
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
    // Sent, and never received: the master has no way to know.
    if (sim::faults.radioLoss > 0 &&
        std::uniform_real_distribution<double>(0, 1)(s_lossRng) < sim::faults.radioLoss) {
        sim::espLost++;
        return ESP_OK;
    }
    for (uint8_t i = 0; i < p->metric_count; i++) {
        // Copied out first: the fields are packed (unaligned), and binding a
        // reference to one - std::map::operator[] takes the key by reference -
        // is undefined behaviour. The slaves must do the same.
        const uint16_t id = p->metrics[i].metric_id;
        const float value = p->metrics[i].value;
        const uint8_t flags = p->metrics[i].flags;
        sim::displayed[id] = { value, flags, simrtos::nowUs() };
        if (id == sim::watchMetric && (sim::watchLog.empty() || sim::watchLog.back().value != value))
            sim::watchLog.push_back({ simrtos::nowUs(), value });
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

/* ═══════════════════════ raw partition access ═══════════════════════════ */

static esp_partition_t s_logsPart = {"logs", 0x800000, 0};

const esp_partition_t *esp_partition_find_first(esp_partition_type_t, esp_partition_subtype_t,
                                                const char *label) {
    if (!label || std::strcmp(label, "logs") || !simfs::logsPartition) return nullptr;
    s_logsPart.size = (uint32_t)simfs::logsBytes;
    return &s_logsPart;
}

esp_err_t esp_partition_read(const esp_partition_t *p, size_t off, void *dst, size_t n) {
    if (p != &s_logsPart || off + n > simfs::logsBytes) return ESP_ERR_INVALID_ARG;
    std::memcpy(dst, simfs::logsRawBytes().data() + off, n);
    return ESP_OK;
}

esp_err_t esp_partition_write(const esp_partition_t *p, size_t off, const void *src, size_t n) {
    if (p != &s_logsPart || off + n > simfs::logsBytes) return ESP_ERR_INVALID_ARG;
    std::vector<uint8_t> &raw = simfs::logsRawBytes();
    const uint8_t *s = (const uint8_t *)src;
    for (size_t i = 0; i < n; i++) {
        if ((raw[off + i] & s[i]) != s[i]) simfs::flashWriteViolations++;
        raw[off + i] &= s[i];
    }
    simfs::logsWrittenBytes += n;
    return ESP_OK;
}

esp_err_t esp_partition_erase_range(const esp_partition_t *p, size_t off, size_t n) {
    if (p != &s_logsPart || off % 4096 || n % 4096 || off + n > simfs::logsBytes) return ESP_ERR_INVALID_ARG;
    std::vector<uint8_t> &raw = simfs::logsRawBytes();
    std::fill(raw.begin() + off, raw.begin() + off + n, 0xFF);
    simfs::logsErasedBytes += n;
    if (off < 8192) {
        // LittleFS's superblocks are gone: it has nothing to mount there now.
        simfs::logsBlank = true;
        for (auto it = simfs::files.begin(); it != simfs::files.end();)
            it = it->first.compare(0, 5, "@logs") == 0 ? simfs::files.erase(it) : std::next(it);
        for (auto it = simfs::dirs.begin(); it != simfs::dirs.end();)
            it = it->compare(0, 5, "@logs") == 0 ? simfs::dirs.erase(it) : std::next(it);
    }
    return ESP_OK;
}
