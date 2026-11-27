/**
 * @file RawRec.h
 * @brief The raw bus recorder: every frame on the bus, and the master's own,
 *        to the 8 MB "logs" partition - for analysis off the car.
 *
 * Armed from the portal (Bus tab). Arming erases the whole partition - the
 * drive logs with it - and drive logging stays off while a recording has the
 * partition. The recording runs whenever the bus does, across sleeps and
 * restarts (each one a new "session" in it), until it is stopped, the
 * partition is full, or the partition is given back to the drive logs.
 *
 * Why the partition is erased first: a flash erase holds the CAN interrupt off
 * for tens of milliseconds - the receive interrupt is not in IRAM in this
 * Arduino core (CONFIG_TWAI_ISR_IN_IRAM is not set) - and the controller's
 * 64-byte receive FIFO holds about five frames, under 5 ms of this car's bus.
 * Erased beforehand, the recording only ever programs 256-byte pages (well
 * under a millisecond each), which the FIFO rides out. Frames the controller
 * does drop are counted into the recording where they happened.
 *
 * Format (all little-endian; master/tools/canrec.py reads it):
 *   Sector 0 (4096 bytes): the header - "CANREC1\0", u16 version (1),
 *     u16 page size (256), u32 data offset (4096), u32 partition size,
 *     u32 bitrate [kbit/s], u32 uptime at arming [ms], char[32] firmware,
 *     then at offset 64 a byte that is 0x00 once the recording was stopped.
 *   From 4096 on, records, none crossing a 256-byte page; the rest of a page
 *   is 0xFF. The first page that starts with 0xFF is the end.
 *   dt below is the time since the previous record, an unsigned LEB128 in
 *   10 us units; E1 and E2 carry absolute times (esp_timer, us since boot).
 *     0x00..0xDF  a frame of that ID index, as the bytes that changed since
 *                 its previous frame: [idx][dt][mask][the bytes whose mask bit
 *                 is set, data[0] = bit 0]; length as before. An index defined
 *                 by a frame of the master's is the master's own frames.
 *     0xE0  a frame in full: [E0][dt][flags][idx if flags bit 3][id: 2 bytes,
 *           or 4 if extended][dlc][data]. flags: bit 0 extended, bit 1 remote,
 *           bit 2 sent by the master, bit 3 defines (or redefines) the index:
 *           the next delta records of it are against this frame (and sent by
 *           the master if bit 2 is set).
 *     0xE1  sync, the first record of every 4 KB sector: [E1][u64 us][u32
 *           frames recorded so far][u16 session][u8 flags: bit 0 listen-only].
 *     0xE2  a session begins (arming, a boot, a wake): [E2][u64 us][u16
 *           session][u8 reset reason]. Forget every index.
 *     0xE3  frames lost: [E3][dt][u16 count][u8 why: 1 the recorder's buffer
 *           was full, 2 the controller dropped them].
 *     0xE4  an evidence-log event: [E4][dt][u8 type][u16 a][u16 b].
 *     0xE5  stopped: [E5][dt].
 *     0xFF  nothing more in this page.
 */
#pragma once
#include <Arduino.h>
#include <driver/twai.h>

enum RawRecState : uint8_t {
    RR_NONE = 0,      /**< No "logs" partition (the older partition table).   */
    RR_OFF,           /**< The partition is the drive logs'.                  */
    RR_ERASING,       /**< Being erased for a recording.                      */
    RR_RECORDING,     /**< Recording whenever the bus is up.                  */
    RR_FULL,          /**< The partition is full.                             */
    RR_STOPPED,       /**< Stopped from the portal.                           */
};

struct RawRecStatus {
    RawRecState state;
    uint32_t used;         /**< Bytes of records on the partition (and buffered). */
    uint32_t cap;          /**< What the records may take.                       */
    uint32_t frames;       /**< Frames recorded since this boot (the bus's).     */
    uint32_t txFrames;     /**< ...of them the master's own.                     */
    uint32_t lost;         /**< Frames lost since this boot (buffer + controller). */
    uint16_t session;      /**< This boot's session number in the recording.    */
    uint32_t rate;         /**< Bytes a second while the bus is up (recent).     */
    uint8_t  erasePct;     /**< Progress of the erase.                           */
    char     note[64];     /**< Why it stopped, if it did.                       */
};

/** @brief In setup(), before driveLogBegin(): find a recording on the
 *         partition and carry on with it. Starts the recorder's task. */
void rawRecBegin();
/** @brief The recorder has the "logs" partition: the drive logs leave it be. */
bool rawRecOwnsPartition();
/** @brief Recording now: flash writes that would cost it frames wait. */
bool rawRecActive();
/** @brief Every frame as the controller hands it over (the receive task). */
void rawRecRx(const twai_message_t &m);
/** @brief A frame of the master's, handed to the controller. */
void rawRecTx(const twai_message_t &m);
/** @brief Frames the controller dropped (its FIFO or the driver's queue). */
void rawRecLost(uint32_t n);
/** @brief An evidence-log event, into the recording too. */
void rawRecEvent(uint8_t type, uint16_t a, uint16_t b);
/**
 * @brief Erase the partition - every drive log with it - and record.
 * @return false (and why) when it cannot: no partition, the car moving, an
 *         erase already running.
 */
bool rawRecArm(char *why, size_t cap);
/** @brief Stop for good (the recording stays until armed again or released). */
void rawRecStop();
/** @brief Give the partition back to the drive logs (the recording goes). */
bool rawRecRelease(char *why, size_t cap);
/** @brief Write what is buffered (before a sleep, a restart, a download). */
void rawRecFlush();
void rawRecStatus(RawRecStatus &out);
/** @brief Bytes to download: the header sector and the records. */
uint32_t rawRecSize();
/** @brief Read the partition for a download. */
bool rawRecRead(uint32_t offset, uint8_t *buf, size_t n);
