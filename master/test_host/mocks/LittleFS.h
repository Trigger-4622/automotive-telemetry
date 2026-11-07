/**
 * @file LittleFS.h
 * @brief In-memory LittleFS for the host tests, with the fault injection the
 *        config code must survive: a full flash (short writes), a layer
 *        that refuses to rename over an existing file, and a mount that fails
 *        (begin(true) then formats, as the core does - erasing everything).
 */
#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace simfs {
extern std::map<std::string, std::string> files;
extern long writeBudget;         ///< Bytes left before "flash full"; <0 = unlimited.
extern bool renameOverwrites;    ///< LittleFS semantics; false = fallback path.
extern bool mountOk;              ///< false: no partition at all, nothing mounts.
extern int  mountFails;          ///< Mounts that fail before one works (a glitch at power-up).
extern bool blank;               ///< Never formatted: mounts only after a format.
extern int  formats;             ///< Formats done - each one erases every file.
/** Files a reader holds open (a portal download): esp_littlefs will not
 *  unlink a file with an open descriptor, so remove() fails for these. */
extern std::set<std::string> held;
extern int  renames, removes;
extern size_t totalBytes;        ///< Partition size LittleFS reports (1.5 MB).
size_t usedBytes();              ///< What the files take, with LittleFS's block rounding.
}

class File {
public:
    File() {}
    explicit operator bool() const { return _st != nullptr; }
    int read();
    size_t readBytes(char *buf, size_t len);
    int available();
    size_t write(uint8_t b);
    size_t write(const uint8_t *buf, size_t len);
    size_t read(uint8_t *buf, size_t len) { return readBytes((char *)buf, len); }
    size_t size() const;
    void close();
    /** @name Directories: a path that files live under. @{ */
    bool isDirectory() const { return _st && _st->dir; }
    File openNextFile();
    const char *name() const;   ///< Without the directory, as core 2.x gives it.
    const char *path() const { return _st ? _st->path.c_str() : ""; }
    /** @} */
private:
    friend class LittleFSFS;
    struct State { std::string path, data; size_t pos = 0; bool writing = false; bool open = true;
                   bool dir = false; std::vector<std::string> list; size_t next = 0; };
    std::shared_ptr<State> _st;
};

class LittleFSFS {
public:
    bool begin(bool formatOnFail = false);
    File open(const char *path, const char *mode = "r");
    bool exists(const char *path);
    bool remove(const char *path);
    bool rename(const char *from, const char *to);
    bool mkdir(const char *path);
    size_t totalBytes() { return simfs::totalBytes; }
    size_t usedBytes()  { return simfs::usedBytes(); }
};
extern LittleFSFS LittleFS;
