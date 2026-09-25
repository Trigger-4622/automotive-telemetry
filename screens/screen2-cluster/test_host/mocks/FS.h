/*
 * In-memory LittleFS, with the two failure modes a layout save must survive:
 * a short write (flash full) and a rename that will not overwrite.
 */
#pragma once

#include <Arduino.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace hostfs {
/** Every file, by absolute path. */
extern std::map<std::string, std::string> files;
/** Directories that exist. */
extern std::vector<std::string> dirs;
/** When >= 0, writes stop after this many more bytes (flash full). */
extern long writeBudget;
/** When true, rename() refuses to replace an existing file. */
extern bool renameNoOverwrite;
/** When true, begin() fails the first mount (corrupt partition). */
extern bool mountFails;
}

namespace fs {

class File {
public:
    File() = default;
    operator bool() const { return (bool)d_; }

    size_t write(uint8_t c) { return write(&c, 1); }
    size_t write(const uint8_t *b, size_t n);
    size_t print(const char *s) { return s ? write((const uint8_t *)s, strlen(s)) : 0; }
    size_t print(const String &s) { return write((const uint8_t *)s.c_str(), s.length()); }
    int read();
    size_t readBytes(char *buf, size_t n);
    int available();
    size_t size();
    const char *name();
    bool isDirectory() { return d_ && d_->dir; }
    File openNextFile();
    void close() { d_.reset(); }

    struct Data {
        std::string path, base;
        bool writing = false, dir = false;
        size_t pos = 0;
        std::vector<std::string> entries;
        size_t next = 0;
    };
    std::shared_ptr<Data> d_;
};

class FS {
public:
    bool begin(bool formatOnFail = false, const char *base = "/littlefs",
               uint8_t maxOpen = 10, const char *label = "spiffs");
    File open(const char *path, const char *mode = "r");
    File open(const String &p, const char *mode = "r") { return open(p.c_str(), mode); }
    bool exists(const char *path);
    bool exists(const String &p) { return exists(p.c_str()); }
    bool remove(const char *path);
    bool remove(const String &p) { return remove(p.c_str()); }
    bool rename(const char *from, const char *to);
    bool mkdir(const char *path);
    bool mkdir(const String &p) { return mkdir(p.c_str()); }
    size_t totalBytes() { return 1507328; }
    size_t usedBytes();
};

}  // namespace fs

using fs::File;
