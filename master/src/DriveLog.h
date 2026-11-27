/**
 * @file DriveLog.h
 * @brief The drive logger: chosen values at up to 20 Hz to the master's
 *        flash, one file per drive, downloaded from the portal.
 *
 * Files live in /log on their own LittleFS partition, "logs": the upper 8 MB
 * of the master's 16 MB flash (partitions_16mb.csv; the partitions before it
 * are the stock 8 MB layout, unchanged, so the settings never move). The logs
 * may take `log_max_kb` (up to 8000 KB); the oldest drive is deleted to make
 * room. Drives the older firmware left on the settings partition are moved
 * across once. A master still on the older partition table keeps them on the
 * settings partition as before: at most 1200 KB of its 1.5 MB, with 128 KB
 * always left free so a settings save can never run out of space.
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
#include <LittleFS.h>

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
    uint32_t capKb;         /**< What they may take (log_max_kb, or less
                                 on the settings partition).            */
    uint32_t freeKb;        /**< Free on the partition.                */
    bool     own;           /**< On their own partition (else the settings'). */
    uint32_t partKb;        /**< That partition's size.                */
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
/** @brief Open a path driveLogPath() gave, on the partition the logs are on. */
File driveLogOpen(const char *path);
/** @brief Write the rows held in RAM, so a download has them. */
void driveLogFlush();
/** @brief Delete one file (the one being written closes first). */
bool driveLogDelete(const char *name);
/** @brief Delete every file. */
void driveLogDeleteAll();
void driveLogStatus(DriveLogStatus &out);
/**
 * @brief Let go of the logs partition for a raw bus recording (RawRec.h):
 *        the drive being written closes, the file system is unmounted, and
 *        the logger stays off until driveLogReclaim(). A master that boots
 *        with a recording on the partition never mounts it.
 */
void driveLogRelease();
/** @brief The partition back from the recorder: formatted, mounted, logging. */
void driveLogReclaim();
