/**
 * @file Learner.h
 * @brief Automatic discovery of this car's broadcast CAN signals.
 *
 * ## Why
 *
 * The master prefers to listen. A value the car already broadcasts costs
 * nothing to read and arrives 50-100 times a second; asking the ECU for it
 * costs bus time and arrives far slower. But nobody has published the frame
 * layout of this car, so the master has to find it.
 *
 * ## How
 *
 * While OBD-II or SSM2 are answering, every requested value is a known truth
 * at a known moment. At each such moment the learner reads every byte and
 * 8/12/14/16-bit field of every changing frame on the bus, and keeps a
 * running least-squares fit of each field against each value. A field that
 * tracks a value almost perfectly *is* that value, and the fit gives its
 * scale and offset. Switches (brake, lights, clutch…) are matched bit by bit.
 *
 * A match is handed to the signal verifier, which checks it against 30 more
 * live readings before it is trusted. Once trusted, the value is read from
 * the bus and the requests for it stop - so the master gets quieter the
 * longer it runs, which is the whole point.
 *
 * Guards against the classic false match (a counter that happens to rise
 * while the engine revs up): the value must have moved up AND down by a
 * clear margin, and the fit must be near-perfect.
 */
#pragma once

#include <Arduino.h>

/** @brief Where a value's learning stands. */
enum LearnState : uint8_t {
    LS_WAIT_REF  = 0,  /**< No requested value to learn from yet.        */
    LS_NEED_MOVE = 1,  /**< Have the value; it has not moved enough.     */
    LS_SEARCHING = 2,  /**< Moved enough; no field fits well yet.        */
    LS_CONFIRM   = 3,  /**< A field matched; the verifier is checking.   */
    LS_LEARNED   = 4,  /**< Read from the bus now; requests stopped.     */
    LS_NOT_FOUND = 5,  /**< Moved a lot, nothing fits: likely not sent.  */
};

/** @brief One learnable value, for the portal. */
struct LearnView {
    uint16_t metric;    /**< Metric being learned.                         */
    uint8_t  state;     /**< @ref LearnState.                              */
    bool     isBit;     /**< A switch (matched per bit).                   */
    float    progress;  /**< 0..1 of the movement needed.                  */
    float    fit;       /**< Best fit so far (r² or bit agreement).        */
    uint16_t samples;   /**< Samples behind the best fit.                  */
    uint32_t canId;     /**< Best frame so far (0 = none).                 */
    uint8_t  start;     /**< Its start bit.                                */
    uint8_t  len;       /**< Its length.                                   */
    bool     be;        /**< Big-endian field.                             */
};

/** @brief Allocate the tables and start the learner task. */
void learnerBegin();

/**
 * @brief A requested value just arrived. Called from publishMetric() for
 *        OBD-II and SSM2 values; returns at once for anything not learnable.
 */
void learnerOnRef(uint16_t metric, float value);

/**
 * @brief Copy the learner's state.
 * @return Rows written.
 */
size_t learnerStatus(LearnView *out, size_t max);

/** @brief Candidate fields and bits currently being tracked. */
void learnerCounts(uint16_t &fields, uint16_t &bits);

/**
 * @brief Start reading this value from the bus now, using the best match
 *        found so far even if it is not fully confirmed. Runs on the
 *        learner task within a quarter second.
 */
void learnerAccept(uint16_t metric);

/** @brief Stop reading this value from the bus: drop its learned signal,
 *         never propose that field for it again, request it as before. */
void learnerUnlearn(uint16_t metric);

/** @brief Forget all progress and start over (learned signals are kept). */
void learnerReset();

/** @brief Delete every automatically learned signal, then start over. */
void learnerForget();
