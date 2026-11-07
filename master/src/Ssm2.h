/**
 * @file Ssm2.h
 * @brief Subaru Select Monitor 2 over CAN, with the ISO-TP transport it needs.
 *
 * ## What this unlocks
 *
 * OBD-II Service 01 exposes a fixed, generic set of PIDs. SSM2 reads arbitrary
 * ECU memory by 3-byte address, which is where everything actually interesting
 * on a Subaru lives: knock correction, fine knock learn, A/F learning and
 * correction, injector pulse width, the ECU's own switch inputs.
 *
 * MY2008+ cars speak it over CAN on the same diagnostic addresses as OBD-II —
 * request to 0x7E0, reply from 0x7E8 — so it needs no extra hardware, only a
 * transport the plain poll engine does not have.
 *
 * ## Why ISO-TP is unavoidable here
 *
 * A CAN frame carries eight bytes. An SSM2 read of N addresses is `A8 00`
 * followed by 3 bytes per address, so asking for more than one parameter
 * already overflows a single frame, and the reply is 1 + N bytes.
 *
 * ISO 15765-2 splits that across frames and, crucially, is a *dialogue*: the
 * sender emits a First Frame and then must wait for the receiver's Flow
 * Control frame before sending Consecutive Frames — and when the ECU replies
 * at length, we owe it the same courtesy.
 *
 * ## The ECU says what it supports
 *
 * The init command (0xAA) returns the ECU's SYS ID, its ROM ID, and a bitmap
 * with one bit per standard parameter. Everything in the seeded table carries
 * the byte/bit of its flag (the same numbering RomRaider and FreeSSM use), so
 * after one exchange the master knows exactly which parameters THIS ECU can
 * answer and never asks for the rest.
 *
 * ## Speed
 *
 * Each parameter has a refresh period: the ones a gauge needle follows (RPM,
 * MAP, throttle, timing, knock, A/F) have none and are asked every exchange,
 * the slow ones (temperatures, voltages, odometer) when their period is up.
 * Each request is filled most-overdue first, so when it has to be small the
 * slow values still get their turn. The request size adapts to what the ECU
 * tolerates - it shrinks on failures and creeps back up when exchanges
 * succeed, never past a size the ECU has refused.
 */
#pragma once

#include <Arduino.h>
#include "driver/twai.h"

struct RtSsm;

/** @brief Diagnostic request identifier (physical, to the ECU). */
static constexpr uint32_t SSM_REQUEST_ID  = 0x7E0;
/** @brief Diagnostic response identifier (from the ECU). */
static constexpr uint32_t SSM_RESPONSE_ID = 0x7E8;

/** @brief Snapshot of the SSM2 engine for the portal and the heartbeat. */
struct Ssm2Status {
    bool     initOk;         /**< The ECU answered 0xAA at least once.      */
    bool     active;         /**< Exchanges are running right now.          */
    uint32_t initAtMs;       /**< millis() of the last successful init.     */
    char     sysId[8];       /**< SYS ID as hex.                            */
    char     ecuId[12];      /**< ROM ID as hex — the calibration identity. */
    uint8_t  flagCount;      /**< Capability bytes received.                */
    uint32_t responses;      /**< Successful read exchanges since boot.     */
    uint32_t errors;         /**< Failed or timed-out exchanges.            */
    uint32_t nrcs;           /**< Negative responses from the ECU.          */
    uint8_t  lastNrc;        /**< Code of the last negative response.       */
    uint8_t  batch;          /**< Addresses per request right now.          */
    uint16_t supported;      /**< Seeded parameters the ECU supports.       */
    uint16_t total;          /**< Seeded parameters with a support bit.     */
    float    exchPerSec;     /**< Exchange rate over the last second.       */
    uint32_t lastOkMs;       /**< millis() of the last good exchange.       */
    uint16_t refused;        /**< Addresses the ECU refused and were dropped.*/
    char     lastErr[40];    /**< Why the last exchange failed, plain words. */
};

/**
 * @brief Start the SSM2 task.
 *
 * Idles cheaply while the arbiter says no (see diagSsmWanted()), so the mode
 * can change from the portal without a reboot.
 */
void ssm2Begin();

/**
 * @brief Hand a received frame to the SSM2 transport.
 *
 * Called from the CAN receive task for every frame on @ref SSM_RESPONSE_ID.
 * Consumed only while an SSM2 exchange is in flight — outside one, a 0x7E8
 * frame belongs to the OBD-II engine and this returns false.
 *
 * @param msg Frame just received.
 * @return true when the frame was consumed by SSM2.
 */
bool ssm2FeedFrame(const twai_message_t &msg);

/** @brief Copy the engine's state. */
void ssm2GetStatus(Ssm2Status &out);

/**
 * @brief Does the ECU support this parameter?
 * @return 1 yes, 0 no, -1 unknown (no init yet, or the entry has no flag).
 */
int ssm2Supported(const RtSsm &e);

/** @brief Forget the init result and probe the ECU again. */
void ssm2Reinit();

/** @brief Successful SSM2 exchanges since boot. */
uint32_t ssm2Responses();

/** @brief Failed or timed-out SSM2 exchanges since boot. */
uint32_t ssm2Errors();

/** @brief How an isoExchange() went. */
enum IsoResult : uint8_t {
    ISO_OK,         /**< A reply to the request.                              */
    ISO_NRC,        /**< A negative reply: rsp = 7F <service> <code>.         */
    ISO_NO_REPLY,   /**< Sent, and nothing (or a broken reply) came back.     */
    ISO_BUSY        /**< Never sent: guard up, bus quiet, lock not free.      */
};

/**
 * @brief One ISO 15765-2 request and its reply, with any diagnostic address -
 *        the SSM2 transport lent out, under the same bus lock and guard.
 *
 * Every frame goes through masterTransmit(), so it is refused while the bus is
 * quiet, in listen-only, during the settle wait or while the guard is up.
 * A negative reply 0x78 ("still working") extends the wait, as for SSM2.
 *
 * @param txId     Request ID: 0x7E0 engine, 0x7E1 transmission, or 0x7DF
 *                 (every computer; the request must then fit one frame).
 * @param rxId     The reply's ID (0x7E8 engine, 0x7E9 transmission).
 * @param fcId     Where our flow control for a long reply goes: the
 *                 responder's own request ID, also after a 0x7DF request.
 * @param req      Service byte first.
 * @param[out] rsp The reply, service byte (request + 0x40, or 0x7F) first.
 * @param[out] got Reply length.
 * @param timeoutMs How long the reply may take.
 * @param[out] why Why it failed, in plain words ("" when it did not).
 */
IsoResult isoExchange(uint32_t txId, uint32_t rxId, uint32_t fcId,
                      const uint8_t *req, size_t len, uint8_t *rsp, size_t max, size_t &got,
                      uint32_t timeoutMs, char *why, size_t whyCap);
