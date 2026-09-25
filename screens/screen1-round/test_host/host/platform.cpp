/*
 * Host implementations behind the mocks: virtual clock, log capture, the
 * in-memory filesystem, and the hardware singletons.
 */
#include <Arduino.h>

#include <algorithm>
#include <string>
#include <vector>

#include "ButtonManager.h"
#include "DisplayManager.h"
#include "LittleFS.h"
#include "TouchManager.h"
#include "WiFi.h"
#include "host.h"

/* ───────────────────────────── clock, log ──────────────────────────────── */

static uint32_t s_ms = 1000;      // boot a second in, like a real start

extern "C" uint32_t millis(void) { return s_ms; }
extern "C" uint32_t micros(void) { return s_ms * 1000u; }
extern "C" void delay(uint32_t ms) { s_ms += ms; }

void host::setMillis(uint32_t ms) { s_ms = ms; }

extern "C" size_t strlcpy(char *dst, const char *src, size_t size) {
    const size_t n = strlen(src);
    if (size) {
        const size_t c = n < size - 1 ? n : size - 1;
        memcpy(dst, src, c);
        dst[c] = 0;
    }
    return n;
}

std::vector<std::string> host::logs;
bool host::verbose = false;

extern "C" void host_log(char level, const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    host::logs.push_back(std::string(1, level) + " " + buf);
    if (host::verbose || level == 'E') printf("    [%c] %s\n", level, buf);
}

EspClass ESP;
void EspClass::restart() { host_log('I', "ESP.restart()"); }

WiFiClass      WiFi;
DisplayManager Display;
TouchManager   Touch;
ButtonManager  Button;

/* ─────────────────────────── in-memory LittleFS ────────────────────────── */

namespace hostfs {
std::map<std::string, std::string> files;
std::vector<std::string> dirs = {"/"};
long writeBudget       = -1;
bool renameNoOverwrite = false;
bool mountFails        = false;
}
fs::FS LittleFS;

using namespace hostfs;

bool fs::FS::begin(bool formatOnFail, const char *, uint8_t, const char *) {
    if (mountFails) {
        if (!formatOnFail) return false;
        files.clear();
        mountFails = false;
    }
    return true;
}

fs::File fs::FS::open(const char *path, const char *mode) {
    File f;
    std::string p = path;
    const bool isDir = std::find(dirs.begin(), dirs.end(), p) != dirs.end();
    if (mode[0] == 'r') {
        if (isDir) {
            f.d_ = std::make_shared<File::Data>();
            f.d_->path = p;
            f.d_->dir = true;
            const std::string pre = p == "/" ? "/" : p + "/";
            for (auto &kv : files)
                if (kv.first.compare(0, pre.size(), pre) == 0 &&
                    kv.first.find('/', pre.size()) == std::string::npos)
                    f.d_->entries.push_back(kv.first);
            return f;
        }
        if (!files.count(p)) return f;
    } else {
        files[p].clear();              // "w": truncate
    }
    f.d_ = std::make_shared<File::Data>();
    f.d_->path = p;
    f.d_->writing = mode[0] != 'r';
    const auto slash = p.rfind('/');
    f.d_->base = slash == std::string::npos ? p : p.substr(slash + 1);
    return f;
}

bool fs::FS::exists(const char *path) {
    return files.count(path) ||
           std::find(dirs.begin(), dirs.end(), std::string(path)) != dirs.end();
}
bool fs::FS::remove(const char *path) { return files.erase(path) > 0; }
bool fs::FS::rename(const char *from, const char *to) {
    if (!files.count(from)) return false;
    if (files.count(to) && renameNoOverwrite) return false;
    files[to] = files[from];
    files.erase(from);
    return true;
}
bool fs::FS::mkdir(const char *path) { dirs.push_back(path); return true; }
size_t fs::FS::usedBytes() {
    size_t n = 0;
    for (auto &kv : files) n += kv.second.size();
    return n;
}

size_t fs::File::write(const uint8_t *b, size_t n) {
    if (!d_ || !d_->writing) return 0;
    if (writeBudget >= 0) {
        if ((long)n > writeBudget) n = (size_t)writeBudget;
        writeBudget -= (long)n;
    }
    files[d_->path].append((const char *)b, n);
    return n;
}
int fs::File::read() {
    if (!d_ || d_->writing) return -1;
    const std::string &s = files[d_->path];
    return d_->pos < s.size() ? (uint8_t)s[d_->pos++] : -1;
}
size_t fs::File::readBytes(char *buf, size_t n) {
    if (!d_ || d_->writing) return 0;
    const std::string &s = files[d_->path];
    const size_t c = std::min(n, s.size() - std::min(d_->pos, s.size()));
    memcpy(buf, s.data() + d_->pos, c);
    d_->pos += c;
    return c;
}
int fs::File::available() {
    return d_ ? (int)(files[d_->path].size() - d_->pos) : 0;
}
size_t fs::File::size() { return d_ ? files[d_->path].size() : 0; }
const char *fs::File::name() { return d_ ? d_->base.c_str() : ""; }
fs::File fs::File::openNextFile() {
    if (!d_ || !d_->dir || d_->next >= d_->entries.size()) return File();
    return LittleFS.open(d_->entries[d_->next++].c_str(), "r");
}
