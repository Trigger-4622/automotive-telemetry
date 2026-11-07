/**
 * @file DriveLog.h
 * @brief The drive logger: chosen values at up to 20 Hz to the master's
 *        flash, one file per drive, downloaded from the portal.
 *
 * Files live in /log on the LittleFS partition the settings use (1.5 MB), so
 * the partition table does not change. The logs may take `log_max_kb`; the
 * oldest drive is deleted to make room, and 128 KB of the partition is always
 * left free so a settings save can never run out of space.
 *
 * File format (.tlg, little-endian):
 *   "TLG1"  u8 version (1)  u8 hz  u8 columns  u8 0
 *   u32 bus session   u32 master uptime at the start [ms]
 *   u16 metric ID per column
 *   rows: u32 ms since the start, then f32 per column (NaN: not fresh)
 * The portal turns it into CSV with the metrics' names.
 *
 * Rows are kept in RAM and written 4 KB at a time (every ~7 s at 10 Hz with
 * the default 14 columns), each write announced with masterBusStall(): a
 * flash write holds the CAN interrupt off, and the frames it costs are not
 * the bus's fault. A power cut loses at most the rows not yet written.
 */
#pragma once
#include <Arduino.h>

/** @brief One file, for the portal's list. */
struct LogFileInfo {
    char     name[16];    /**< "00012.tlg".                          */
    uint32_t bytes;       /**< Size on flash.                        */
    uint32_t rows;        /**< Rows in it.                           */
    float    seconds;     /**< Time it covers.                       */
    uint8_t  hz, cols;
    bool     recording;   /**< The file being written now.           */
};

/** @brief What the logger is doing. */
struct DriveLogStatus {
    bool     on;            /**< log_on.                               */
    bool     recording;     /**< A drive is being recorded.            */
    char     file[16];      /**< ...into this file.                    */
    uint32_t rows;          /**< Rows in it so far.                    */
    uint32_t usedKb;        /**< What every log file takes.            */
    uint32_t capKb;         /**< log_max_kb.                           */
    uint32_t freeKb;        /**< Free on the partition.                */
    uint32_t flushes;       /**< Writes to flash since boot.           */
    char     note[64];      /**< Why it stopped, if it did.            */
};

/** @brief Start the logger task. After Cfg.begin(). */
void driveLogBegin();
/** @brief Every log file, oldest first. */
size_t driveLogList(LogFileInfo *out, size_t max);
/**
 * @brief The flash path of a log file named in a request.
 * @return false for anything but a plain "NNNNN.tlg" that exists.
 */
bool driveLogPath(const char *name, char *path, size_t cap);
/** @brief Write the rows held in RAM, so a download has them. */
void driveLogFlush();
/** @brief Delete one file (the one being written closes first). */
bool driveLogDelete(const char *name);
/** @brief Delete every file. */
void driveLogDeleteAll();
void driveLogStatus(DriveLogStatus &out);
