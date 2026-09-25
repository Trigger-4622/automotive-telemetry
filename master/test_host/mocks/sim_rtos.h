/**
 * @file sim_rtos.h
 * @brief Deterministic stand-in for FreeRTOS, for running the real firmware
 *        on a PC.
 *
 * Every firmware task is a real thread, but only one runs at a time: a task
 * runs until it blocks (delay, queue, semaphore, CAN receive…), and the
 * scheduler then hands over to the highest-priority task that is ready. When
 * nothing is ready, virtual time jumps straight to the next wake-up. So a
 * four-minute drive simulates in seconds, and the same run always produces
 * the same result - a failure can be replayed and debugged.
 *
 * What it cannot show: preemption in the middle of code that never blocks.
 * Races of that kind need the real chip.
 */
#pragma once
#include <cstdint>
#include <vector>

namespace simrtos {

constexpr uint64_t FOREVER = UINT64_MAX;

/** Something tasks can wait on (a queue, a semaphore, a driver event). */
struct Waitable {
    std::vector<struct Task *> waiters;
};

uint64_t nowUs();

/** Block the running task until notified or until @p timeoutUs passes.
 *  @return true when notified, false on timeout. */
bool block(Waitable *w, uint64_t timeoutUs);

/** Wake every task waiting on @p w; switches to one at once if it outranks
 *  the caller (as FreeRTOS does). */
void notify(Waitable *w);

/** Let other ready tasks of the same priority run. */
void yield();

/** Advance this task's virtual time by @p us, letting others run. */
void sleepUs(uint64_t us);

/** Create a task. It starts when the scheduler picks it. */
void createTask(void (*fn)(void *), const char *name, void *arg, int prio);

/** Name of the running task (for diagnostics). */
const char *currentName();

/** Turn the calling thread into the first task and run @p fn in it. */
void runMain(void (*fn)(), int prio);

/** Start the virtual clock at @p us instead of 0 (call before runMain):
 *  lets a test cross the 32-bit millis() wrap without simulating 49 days. */
void setStartUs(uint64_t us);

/** Move the virtual clock forward by @p us at once, as if the system had run
 *  that long; every timer that expired meanwhile fires. Keep each jump under
 *  24.8 days, or intervals become ambiguous in 32-bit milliseconds - which a
 *  running system never sees either. */
void jumpUs(uint64_t us);

/** Critical-section bookkeeping: blocking inside one is a firmware bug. */
extern int g_criticalDepth;

/** Reports "virtual time stopped moving" (a task spinning without ever
 *  blocking) and aborts, after @p realSeconds of wall time. */
void startWatchdog(int realSeconds);

}  // namespace simrtos
