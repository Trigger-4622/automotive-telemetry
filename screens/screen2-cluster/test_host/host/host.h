/* The harness's own interface: time, the simulated car, screenshots. */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <map>
#include <string>
#include <vector>

namespace host {

extern std::vector<std::string> logs;   /**< Every firmware log line.    */
extern bool verbose;                     /**< Echo all of them.           */

void setMillis(uint32_t ms);

/** Bring LVGL up with a framebuffer display of the board's size. */
void lvglInit();
/** Render everything now and write the frame as a PNG. */
void screenshot(const std::string &path);
/** LVGL heap: bytes used, bytes free, biggest free block, fragmentation %. */
struct Mem { uint32_t used, free, biggest; uint8_t frag; };
Mem lvglMem();

/**
 * The car on the other end of the radio: metric values, broadcast to the
 * TelemetryStore the way the master does (changed values at once, the rest
 * as keep-alives), while virtual time runs.
 */
struct Car {
    std::map<uint16_t, float> values;     /**< What the master reports.   */
    std::map<uint16_t, uint8_t> flags;    /**< Extra flag bits per metric.*/
    bool radio = true;                    /**< false: nothing is sent.    */
    bool night = false;
    uint32_t seq = 0;
    uint32_t lastSend = 0;
    void set(uint16_t id, float v) { values[id] = v; }
    void drop(uint16_t id) { values.erase(id); flags.erase(id); }
    void send();                          /**< One burst, now.            */
};
extern Car car;

/** Record the LVGL heap in use before the UI exists (theme, display). */
void heapBaseline();

/** The LVGL heap, measured on the PC and worked out for the ESP32. */
struct HeapReport {
    size_t pcUsed, pcPeak;        /**< lv_mem_monitor on this PC.            */
    size_t pcModel;               /**< The object walk, PC struct sizes.     */
    size_t targetModel;           /**< The object walk, 32-bit struct sizes. */
    size_t targetUsed;            /**< Estimated in use on the device.       */
    size_t targetPeak;            /**< ...plus transient render buffers.     */
    size_t budget;                /**< The board's LV_MEM_SIZE.              */
    size_t objects;               /**< Live LVGL objects.                    */
    size_t baseline64;
};
HeapReport heapReport();
/** What one object and everything under it holds on the device. */
size_t deviceBytes(struct _lv_obj_t *o);

/** Run the firmware for @p ms of virtual time in 5 ms steps. */
void run(uint32_t ms);

}  // namespace host
