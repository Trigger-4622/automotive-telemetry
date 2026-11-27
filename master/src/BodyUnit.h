/**
 * @file BodyUnit.h
 * @brief The car's other computers on CAN: a scan for every diagnostic
 *        address that answers, and a reader for the body computer (BIU).
 *
 * The scan (portal: Car tab, "Find the car's computers"; standing still,
 * nothing else to it): to every request ID from 0x700 to 0x7F7 but the
 * all-computers 0x7DF, one TesterPresent ("3E 00": "are you there") - and to
 * each that answers on its ID + 8, "21 50", the first data block of the body
 * computer as FastECU found it on the 2005-09 cars' K-line BIU (KWP2000
 * ReadDataByLocalIdentifier). An answer "61 50 ..." is a body computer. Only
 * those two requests are ever sent: neither changes anything in a computer.
 * One request at a time, through isoExchange() (the bus lock, the guard, a
 * request only to a live bus and never in listen-only), about 20 s in all,
 * with the OBD-II poller going ahead of it at every turn.
 *
 * The reader (biu_tx set, from the scan's result or by hand): "21 50" and
 * "21 51" (its lighting block) every biu_ms. Their answers go into the frame
 * table as frames of their own - extended IDs BODY_PSEUDO_BASE | local ID << 8
 * | the eight-byte part, "BIU 50 · 0-7" on the portal - so the Map button and
 * teach by doing work on them as on any broadcast frame, and a signal mapped
 * there publishes like one (the learner leaves extended IDs alone, and a
 * switch has no figure of the engine computer's to be learned against). They never go to the raw recording
 * (they are not on the bus; the requests and answers that are, are) nor into
 * the link statistics (they come when asked).
 */
#pragma once
#include <Arduino.h>
#include <driver/twai.h>

/** Extended IDs of the BIU's answers in the frame table (never on a bus here:
 *  the car has no extended identifiers). */
static constexpr uint32_t BODY_PSEUDO_BASE = 0x1F5B0000;
static inline bool bodyPseudoId(uint32_t id, bool ext) {
    return ext && (id & 0xFFFF0000u) == BODY_PSEUDO_BASE;
}

/** @brief One computer that answered the scan. */
struct BodyFound {
    uint16_t tx = 0, rx = 0;      /**< Its request and reply IDs.               */
    uint8_t  reply[8] = {};       /**< The start of its answer to "3E 00".      */
    uint8_t  replyLen = 0;
    bool     biu = false;         /**< Answered "21 50" with "61 50".          */
    uint8_t  biuLen = 0;          /**< Bytes of that block.                     */
};

static constexpr size_t BODY_MAX_FOUND = 16;

/** @brief The scan and the reader, for the portal. */
struct BodyStatus {
    bool     scanning = false;
    uint16_t at = 0;              /**< The ID being asked.                      */
    uint8_t  pct = 0;
    bool     scanned = false;     /**< A scan has finished since boot.          */
    char     scanNote[64] = "";   /**< Why one stopped, if it did.              */
    uint8_t  found = 0;
    uint16_t others[8] = {};      /**< Other 0x7xx IDs that turned up meanwhile. */
    uint8_t  otherN = 0;
    // The reader.
    uint16_t tx = 0, rx = 0;      /**< 0: off.                                  */
    uint32_t reads = 0, fails = 0;
    uint32_t ageMs = 0;           /**< Since the last answer (UINT32_MAX: none). */
    uint8_t  block[2][24] = {};   /**< The last answers: 0x50 and 0x51.         */
    uint8_t  blockLen[2] = {};
    char     note[64] = "";       /**< Why the last read failed.                */
};

/** @brief At boot, before learnerBegin(): the queue for the RX task. */
void bodyBegin();
/** @brief The codes task's loop, every 100 ms: a step of the scan, or a read. */
void bodyStep();
/** @brief Start the scan. @return false (and why): moving, bus quiet, listen-only. */
bool bodyRequestScan(char *why, size_t cap);
/** @brief The scan's and the reader's state. */
void bodyGetStatus(BodyStatus &out);
/** @brief Computer @p i of the last scan. */
bool bodyFoundAt(size_t i, BodyFound &out);
/** @brief The RX task: an answer of the BIU's to put in the frame table. */
bool bodyTakeFrame(twai_message_t &out);
