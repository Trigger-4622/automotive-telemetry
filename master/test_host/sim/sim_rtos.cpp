/**
 * @file sim_rtos.cpp
 * @brief The cooperative, virtual-time scheduler behind the FreeRTOS mock.
 *        See sim_rtos.h.
 */
#include "sim_rtos.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>

namespace simrtos {

struct Task {
    std::string name;
    int         prio = 1;
    uint64_t    seq = 0;             // readiness order, for round-robin
    enum St { READY, BLOCKED, DONE } st = READY;
    uint64_t    wakeAt = FOREVER;
    Waitable   *waitOn = nullptr;
    bool        signaled = false;
    std::condition_variable cv;
};

static std::mutex          M;
static std::vector<Task *> s_tasks;
static Task               *s_cur = nullptr;
static uint64_t            s_now = 0;
static uint64_t            s_seq = 0;
static std::atomic<uint64_t> s_nowAtomic{0};
int g_criticalDepth = 0;

uint64_t nowUs() { return s_nowAtomic.load(); }
const char *currentName() { return s_cur ? s_cur->name.c_str() : "?"; }

static void dumpAndAbort(const char *why) {
    std::fprintf(stderr, "\n*** SIM: %s at t=%.3f s\n", why, s_now / 1e6);
    for (Task *t : s_tasks)
        std::fprintf(stderr, "    task %-10s prio %2d  %s%s\n", t->name.c_str(), t->prio,
                     t->st == Task::READY ? "ready" : t->st == Task::DONE ? "done" : "blocked",
                     t->st == Task::BLOCKED && t->wakeAt == FOREVER ? " (forever)" : "");
    std::fflush(stderr);
    std::_Exit(3);
}

/** Choose the next task to run; advances virtual time when nothing is ready.
 *  Called with M held. */
static void pickNext() {
    for (;;) {
        Task *best = nullptr;
        for (Task *t : s_tasks)
            if (t->st == Task::READY &&
                (!best || t->prio > best->prio || (t->prio == best->prio && t->seq < best->seq)))
                best = t;
        if (best) { s_cur = best; return; }

        uint64_t next = FOREVER;
        for (Task *t : s_tasks)
            if (t->st == Task::BLOCKED && t->wakeAt < next) next = t->wakeAt;
        if (next == FOREVER) dumpAndAbort("deadlock - every task blocked forever");
        if (next > s_now) { s_now = next; s_nowAtomic.store(next); }
        for (Task *t : s_tasks)
            if (t->st == Task::BLOCKED && t->wakeAt <= s_now) {
                t->st = Task::READY;
                t->signaled = false;
                t->seq = ++s_seq;
            }
    }
}

/** Hand over to whoever pickNext() chose; return when this task runs again. */
static void switchAway(std::unique_lock<std::mutex> &lk) {
    Task *me = s_cur;
    pickNext();
    if (s_cur != me) {
        s_cur->cv.notify_one();
        me->cv.wait(lk, [&] { return s_cur == me; });
    }
}

bool block(Waitable *w, uint64_t timeoutUs) {
    if (g_criticalDepth)
        dumpAndAbort("a task blocked inside a critical section (portENTER_CRITICAL)");
    std::unique_lock<std::mutex> lk(M);
    Task *me = s_cur;
    me->st = Task::BLOCKED;
    me->waitOn = w;
    me->signaled = false;
    me->wakeAt = timeoutUs == FOREVER ? FOREVER : s_now + timeoutUs;
    if (w) w->waiters.push_back(me);
    switchAway(lk);
    if (w) {
        auto &v = w->waiters;
        v.erase(std::remove(v.begin(), v.end(), me), v.end());
    }
    me->waitOn = nullptr;
    return me->signaled;
}

void notify(Waitable *w) {
    std::unique_lock<std::mutex> lk(M);
    bool preempt = false;
    for (Task *t : w->waiters)
        if (t->st == Task::BLOCKED && t->waitOn == w) {
            t->st = Task::READY;
            t->signaled = true;
            t->seq = ++s_seq;
            if (s_cur && t->prio > s_cur->prio) preempt = true;
        }
    if (preempt && s_cur) {
        s_cur->st = Task::READY;            // keeps its place among equals
        switchAway(lk);
    }
}

void yield() {
    std::unique_lock<std::mutex> lk(M);
    s_cur->st = Task::READY;
    s_cur->seq = ++s_seq;                  // behind others of its priority
    switchAway(lk);
}

void sleepUs(uint64_t us) {
    if (us == 0) { yield(); return; }
    block(nullptr, us);
}

void createTask(void (*fn)(void *), const char *name, void *arg, int prio) {
    std::unique_lock<std::mutex> lk(M);
    Task *t = new Task;
    t->name = name;
    t->prio = prio;
    t->st = Task::READY;
    t->seq = ++s_seq;
    s_tasks.push_back(t);
    std::thread([t, fn, arg] {
        {
            std::unique_lock<std::mutex> l(M);
            t->cv.wait(l, [&] { return s_cur == t; });
        }
        fn(arg);
        std::unique_lock<std::mutex> l(M);
        t->st = Task::DONE;
        pickNext();
        s_cur->cv.notify_one();
    }).detach();
    if (s_cur && prio > s_cur->prio) {     // FreeRTOS runs a higher one at once
        s_cur->st = Task::READY;
        switchAway(lk);
    }
}

void setStartUs(uint64_t us) { s_now = us; s_nowAtomic.store(us); }

void jumpUs(uint64_t us) {
    std::unique_lock<std::mutex> lk(M);
    s_now += us;
    s_nowAtomic.store(s_now);
}

void runMain(void (*fn)(), int prio) {
    {
        std::unique_lock<std::mutex> lk(M);
        Task *t = new Task;
        t->name = "loopTask";
        t->prio = prio;
        t->seq = ++s_seq;
        s_tasks.push_back(t);
        s_cur = t;
    }
    fn();
}

void startWatchdog(int realSeconds) {
    std::thread([realSeconds] {
        uint64_t last = s_nowAtomic.load();
        int still = 0;
        for (;;) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            const uint64_t n = s_nowAtomic.load();
            still = n == last ? still + 1 : 0;
            last = n;
            if (still >= realSeconds) {
                std::fprintf(stderr, "\n*** SIM WATCHDOG: virtual time stuck at %.3f s for %d s - "
                             "task '%s' is spinning without blocking\n",
                             n / 1e6, realSeconds, currentName());
                std::fflush(stderr);
                std::_Exit(4);
            }
        }
    }).detach();
}

}  // namespace simrtos
