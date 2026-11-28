/**
 * @file Computers.h
 * @brief Every computer on the car's bus that answers a diagnostic request -
 *        found, asked who it is, its data blocks searched, and the blocks
 *        chosen read into the frame table. Nothing here knows any computer,
 *        address or data layout in advance.
 *
 * Find (portal: Car tab, "Find the car's computers"; standing still, the bus
 * up, the master not listening only):
 *  1. To every 11-bit ID from 0x700 to 0x7F7, one TesterPresent ("3E 00": are
 *     you there) - but never to an ID the master has heard on the bus (another
 *     module sends on it: a frame of ours there could collide with one of its
 *     own, the car's 0x705 and 0x706 among them), nor to 0x7DF (every
 *     computer at once) or 0x7E8-0x7EF (the legislated answers). An answer is
 *     taken from whichever ID it comes on, recognised by what it says ("7E 00",
 *     or a refusal "7F 3E ..."), not by where it is expected (isoProbe()).
 *  2. Once to every 29-bit computer at once (0x18DB33F1, ISO 15765-4): each
 *     that answers on 0x18DAF1xx is a computer at 0x18DAxxF1.
 *  3. Each computer found is asked who it is, every way a computer of this
 *     age might answer: OBD-II "01 00" (an emissions computer), Subaru's SSM
 *     "AA" (its system and ROM IDs), KWP2000 "1A 80" (its identification) and
 *     UDS "22 F1 97" / "22 F1 87" (system name, part number).
 * Search (a computer's "Find its data"): "21 01" to "21 FE" (KWP2000
 * ReadDataByLocalIdentifier), every block that answers kept with its length.
 * A computer that refuses service 21 outright is left after the first.
 * Read (the "reads" table: a computer, a block, how often): the answers go
 * into the frame table as frames of their own, extended IDs COMP_PSEUDO_BASE |
 * address << 12 | block << 4 | which eight bytes - "0x750 21 50 · 0-7" on the
 * portal - so the Map button and teach by doing work on them as on any frame,
 * and a signal mapped there publishes like one. They are never in the raw
 * recording (they are not on the bus; the requests and answers are) nor in
 * the link statistics (they come when asked).
 *
 * Only reading services are ever sent (3E, 01, AA, 1A, 21, 22, and flow
 * control for a long answer): nothing in a computer is changed. One request at
 * a time, through isoProbe() / isoExchange() - the bus lock, the guard, never
 * to a quiet bus or in listen-only - with the OBD-II poller going ahead of it
 * at every turn.
 */
#pragma once
#include <Arduino.h>
#include <driver/twai.h>

/** The computers' answers in the frame table (never on a bus here: the car has
 *  no extended identifiers). */
static constexpr uint32_t COMP_PSEUDO_BASE = 0x1E000000;
static inline bool compPseudoId(uint32_t id, bool ext) {
    return ext && (id & 0x1F000000u) == COMP_PSEUDO_BASE;
}
/** A request ID as 12 bits: an 11-bit one as it is, a 29-bit 0x18DAttF1 as
 *  0x800 | its target byte. */
static inline uint32_t compAddr12(uint32_t tx) {
    return tx > 0x7FF ? 0x800u | ((tx >> 8) & 0xFF) : tx;
}
static inline uint32_t compPseudo(uint32_t tx, uint8_t lid, uint8_t part) {
    return COMP_PSEUDO_BASE | compAddr12(tx) << 12 | (uint32_t)lid << 4 | (part & 0xF);
}

static constexpr size_t COMP_MAX      = 16;   /**< Computers a scan keeps.          */
static constexpr size_t COMP_MAX_LIDS = 24;   /**< Blocks a search keeps.           */

/** The ways a computer is asked who it is. */
enum CompAsk : uint8_t { ASK_OBD, ASK_SSM, ASK_KWP, ASK_UDS_NAME, ASK_UDS_PART, ASK_N };
/** How one of them went. */
enum CompHow : uint8_t { HOW_NOT_ASKED, HOW_ANSWERED, HOW_REFUSED, HOW_SILENT };

/** @brief One answer to "who are you": how it went, and its start. */
struct CompAnswer {
    uint8_t how = HOW_NOT_ASKED;
    uint8_t code = 0;            /**< The refusal's code (7F xx code).            */
    uint8_t len = 0;             /**< Bytes kept below (of the positive answer).  */
    uint8_t d[18] = {};          /**< The answer, its service byte first.         */
};

/** Where a computer's data search is. */
enum CompSearch : uint8_t { SEARCH_NONE, SEARCH_RUNNING, SEARCH_DONE, SEARCH_NO_SERVICE };

/** @brief One computer that answered. */
struct CompFound {
    uint32_t   tx = 0, rx = 0;       /**< Its request and reply IDs (29-bit above 0x7FF). */
    uint8_t    hello[8] = {};        /**< Its answer to "3E 00", as received.            */
    CompAnswer ans[ASK_N];
    uint8_t    search = SEARCH_NONE;
    uint8_t    nLids = 0;
    uint8_t    lid[COMP_MAX_LIDS] = {};
    uint8_t    lidLen[COMP_MAX_LIDS] = {};  /**< Bytes of each block (after "61 lid"). */
};

/** @brief The scan, for the portal. */
struct CompStatus {
    uint8_t  phase = 0;           /**< 0 idle, 1 finding (11-bit), 2 finding (29-bit),
                                       3 asking who they are, 4 searching one's data. */
    uint32_t at = 0;              /**< The ID being asked.                       */
    uint8_t  pct = 0;
    bool     scanned = false;     /**< A scan has finished since boot.           */
    int8_t   searching = -1;      /**< The computer whose data is being searched. */
    uint8_t  found = 0;
    uint16_t skipped = 0;         /**< IDs not asked: other modules send on them. */
    char     note[64] = "";       /**< Why the last one stopped, if it did.      */
};

/** @brief One row of the reads table, as it goes. */
struct CompReadView {
    uint32_t tx = 0, rx = 0;
    uint8_t  lid = 0;
    uint16_t ms = 0;
    uint32_t reads = 0, fails = 0;
    uint32_t ageMs = UINT32_MAX;  /**< Since its last answer.                    */
    uint8_t  len = 0;             /**< Bytes of its last answer.                 */
    char     note[40] = "";       /**< Why the last read failed.                 */
};

/** @brief At boot, before learnerBegin(): the queue for the RX task. */
void compBegin();
/** @brief The codes task's loop, every 100 ms: a step of the scan, or a read. */
void compStep();
/** @brief Find the computers. @return false (and why): moving, bus quiet... */
bool compRequestScan(char *why, size_t cap);
/** @brief Search computer @p i's data blocks. @return false (and why). */
bool compRequestSearch(size_t i, char *why, size_t cap);
void compGetStatus(CompStatus &out);
/** @brief Computer @p i of the last scan. */
bool compFoundAt(size_t i, CompFound &out);
/** @brief Row @p i of the reads table, as it goes. */
bool compReadAt(size_t i, CompReadView &out);
/** @brief The RX task: an answer to put in the frame table. */
bool compTakeFrame(twai_message_t &out);
