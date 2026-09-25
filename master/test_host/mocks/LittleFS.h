/**
 * @file LittleFS.h
 * @brief In-memory LittleFS for the host tests, with the fault injection the
 *        config code must survive: a full flash (short writes) and a layer
 *        that refuses to rename over an existing file.
 */
#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace simfs {
extern std::map<std::string, std::string> files;
extern long writeBudget;         ///< Bytes left before "flash full"; <0 = unlimited.
extern bool renameOverwrites;    ///< LittleFS semantics; false = fallback path.
extern bool mountOk;
extern int  renames, removes;
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
    void close();
private:
    friend class LittleFSFS;
    struct State { std::string path, data; size_t pos = 0; bool writing = false; bool open = true; };
    std::shared_ptr<State> _st;
};

class LittleFSFS {
public:
    bool begin(bool formatOnFail = false);
    File open(const char *path, const char *mode = "r");
    bool exists(const char *path);
    bool remove(const char *path);
    bool rename(const char *from, const char *to);
};
extern LittleFSFS LittleFS;
