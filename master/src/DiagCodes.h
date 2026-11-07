/**
 * @file DiagCodes.h
 * @brief Trouble codes and the engine computer's identity, read over OBD-II.
 *
 * Stored (service 03), pending (07) and permanent (0A) codes from the engine
 * computer (ECM, 0x7E0) and the transmission computer (TCM, 0x7E1), read when
 * the car comes on, every `dtc_poll_s` after that, whenever the lamp or the
 * ECM's code count changes, and when the portal asks. They are published as
 * METRIC_ID_DTC_TOTAL / DTC_1..8 (see MasterPacket.h) for the screens' card.
 * Clear (service 04) is sent only when the portal asks, and only with the car
 * standing still. Service 09 gives the VIN, calibration ID and CVN - what
 * RomRaider's definitions are listed by.
 *
 * Every request goes through isoExchange(): the bus lock, the guard, and
 * nothing at all while the bus is quiet or the master listens only.
 */
#pragma once
#include <Arduino.h>

/** @brief One code as read. */
struct DtcEntry {
    uint16_t raw;      /**< The two code bytes (0x1718 = P1718).  */
    uint8_t  kind;     /**< DtcKind: stored, pending, permanent.  */
    uint8_t  module;   /**< DtcModule: engine, transmission.      */
};

/** @brief What one list (computer x kind) said at the last read. */
enum CodeList : uint8_t {
    LIST_NOT_ASKED = 0,  /**< Not asked yet this boot.                       */
    LIST_ANSWERED  = 1,  /**< Answered (perhaps with no codes).              */
    LIST_NO_REPLY  = 2,  /**< Asked, nothing came back.                      */
    LIST_REFUSED   = 3,  /**< The computer does not keep this list (NRC).    */
};

static constexpr size_t CODES_MAX = 24;

/** @brief Everything the portal shows. */
struct CodesStatus {
    bool     haveRead;          /**< A read has finished since boot.          */
    uint32_t readAgeMs;         /**< How long ago it finished.                */
    bool     busy;              /**< Reading or clearing right now.           */
    uint32_t reads, clears;     /**< Since boot.                              */
    uint8_t  n;                 /**< Codes in @ref codes.                     */
    DtcEntry codes[CODES_MAX];
    uint8_t  list[2][3];        /**< CodeList per computer, per kind.         */
    bool     tcmFunctional;     /**< The TCM only answered 0x7DF.             */
    char     note[96];          /**< The last read, in plain words.           */
    char     clearNote[96];     /**< The last clear: what each computer did.  */
    char     vin[18];           /**< Service 09 02 ("" until read).           */
    char     calId[2][17];      /**< Service 09 04: up to two IDs.            */
    char     cvn[2][9];         /**< Service 09 06, as hex.                   */
};

/** @brief Start the reader task. After ssm2Begin(). */
void codesBegin();
/** @brief Read the codes now (the portal's "Read codes"). */
void codesRequestRead();
/**
 * @brief Clear the codes in both computers (service 04), once.
 * @param[out] why Why it cannot be sent now: car moving, car off, listen-only.
 * @return true when the clear is queued.
 */
bool codesRequestClear(char *why, size_t cap);
/** @brief Snapshot for the portal. */
void codesGetStatus(CodesStatus &out);
/** @brief "P1718" for a code's two bytes. */
void codesName(uint16_t raw, char out[6]);
/** @brief What a code means (include/DtcText.h). */
const char *codesText(uint16_t raw);
