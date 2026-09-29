/**
 * @file sim_bus.cpp
 * @brief The simulated CAN world: a TWAI controller with the ESP-IDF 4.4
 *        semantics the master relies on, a car broadcasting frames the master
 *        has to learn, an engine ECU answering OBD-II and SSM2 (real ISO-TP,
 *        flow control included), a transmission ECU, and bus faults.
 */
#include <cmath>
#include <cstring>
#include <deque>
#include <map>
#include <cstdlib>
#include <random>

#include "Arduino.h"
#include "sim.h"
#include "hal/twai_ll.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "CanDecoderConfig.h"
#include "MasterTelemetry.h"

namespace sim {
EcuConfig ecu;
BusFaults faults;
std::vector<TxRecord> transmitted;
uint64_t obdRequests = 0, ssmRequests = 0, ssmReadsWithBad = 0;
int reinstalls = 0, bugUninstallWhileWaiting = 0;
int resetReason = 1;   // ESP_RST_POWERON
}
using namespace sim;

static std::mt19937 rng(std::getenv("SIM_SEED") ? (unsigned)std::atoi(std::getenv("SIM_SEED")) : 12345);
static uint64_t s_startUs = 0;
double sim::elapsedS() { return (simrtos::nowUs() - s_startUs) / 1e6; }
static double urand() { return std::uniform_real_distribution<double>(0, 1)(rng); }

/* ═════════════════════════════ the controller ═══════════════════════════ */

static struct Ctrl {
    bool installed = false;
    twai_mode_t mode = TWAI_MODE_NORMAL;
    twai_state_t state = TWAI_STATE_STOPPED;
    uint32_t alertsEnabled = 0, alerts = 0;
    size_t rxLen = 5;
    std::deque<twai_message_t> rx;
    uint32_t tec = 0, rec = 0, busErrors = 0, rxMissed = 0;
    simrtos::Waitable rxw, alertw;
    int waitingRx = 0, waitingAlerts = 0;
    uint64_t recoveryDoneAt = 0;
    bool txAttached = false;   ///< The controller's TX signal drives the pad.
    uint64_t rxHoldUntil = 0;  ///< Frames stay queued until then (a stall).
    bool inReset = false;      ///< Reset mode: off the bus, error counters writable.
} C;

static void flagCheck();
static bool masterFlagsActive();
static void masterDetects(bool flaggedActive);

/* ── the master's radio, on air (the ESP-NOW mock hands packets over) ── */
static uint64_t s_airFrom = 0, s_airUntil = 0;
static std::deque<std::pair<uint64_t, esp_now_send_cb_t>> s_airDone;
uint64_t sim::radioMisreads = 0;
void sim::radioQueue(size_t len, esp_now_send_cb_t cb) {
    const uint64_t now = simrtos::nowUs();
    const uint64_t air = 300 + (uint64_t)len * 8;      // 1 Mbit/s, plus preamble
    if (now >= s_airUntil) s_airFrom = now;
    s_airUntil = std::max(now, s_airUntil) + air;
    if (cb) s_airDone.push_back({s_airUntil, cb});
}
static bool radioOnAir(uint64_t now) { return now >= s_airFrom && now < s_airUntil; }
/** The send callbacks due by @p now, as the Wi-Fi task would call them. */
static void radioCallbacks(uint64_t now) {
    static const uint8_t BC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    while (!s_airDone.empty() && s_airDone.front().first <= now) {
        const esp_now_send_cb_t cb = s_airDone.front().second;
        s_airDone.pop_front();
        cb(BC, ESP_NOW_SEND_SUCCESS);
    }
}

static void txGlitch(const char *why) {
    sim::txGlitches++;
    simLog('E', "SIM-BUG: CAN TX driven dominant by a GPIO - %s", why);
}

void sim::txPadFromGpio() {
    if (!sim::txDrivenHigh) txGlitch("pad switched to a GPIO output while its latch is low");
    C.txAttached = false;
    flagCheck();
}
bool sim::txPadOnController() { return C.txAttached; }

CtrlView sim::controller() { return { C.installed, C.mode, C.state, C.tec, C.rec }; }

static void raiseAlert(uint32_t a) {
    C.alerts |= a;
    if (C.alertsEnabled & a) simrtos::notify(&C.alertw);
}

esp_err_t twai_driver_install(const twai_general_config_t *g, const twai_timing_config_t *t,
                              const twai_filter_config_t *) {
    if (C.installed) return ESP_ERR_INVALID_STATE;
    if (faults.installFailures > 0) {
        faults.installFailures--;
        return ESP_ERR_NO_MEM;
    }
    // The timing must give the bitrate the master thinks it set (APB 80 MHz).
    const uint32_t tq = 1u + t->tseg_1 + t->tseg_2;
    const uint32_t rate = 80000000u / (t->brp * tq);
    if (rate != 125000 && rate != 250000 && rate != 500000 && rate != 1000000) {
        simLog('E', "SIM-BUG: timing gives %u bit/s (brp %u tseg1 %u tseg2 %u)", rate,
               (unsigned)t->brp, t->tseg_1, t->tseg_2);
        return ESP_ERR_INVALID_ARG;
    }
    if (t->brp < 2 || (t->brp & 1)) {
        simLog('E', "SIM-BUG: BRP %u not allowed on the S3 (even, >= 2)", (unsigned)t->brp);
        return ESP_ERR_INVALID_ARG;
    }
    C = Ctrl();
    C.installed = true;
    // twai_configure_gpio(): gpio_config() makes the pad a GPIO output - showing
    // the GPIO latch's level for a moment - then routes the controller onto it.
    if (!sim::txDrivenHigh) txGlitch("twai_driver_install with the TX GPIO latch low");
    C.txAttached = true;
    C.inReset = true;
    C.mode = g->mode;
    C.alertsEnabled = g->alerts_enabled;
    C.rxLen = g->rx_queue_len;
    reinstalls++;
    sim::timing = { 100.0 * (1 + t->tseg_1) / tq, (int)t->sjw, t->triple_sampling };
    flagCheck();
    simLog('D', "SIM: TWAI installed, %s, %u bit/s, sample %.1f%%",
           g->mode == TWAI_MODE_LISTEN_ONLY ? "listen-only" : "normal", rate,
           100.0 * (1 + t->tseg_1) / tq);
    return ESP_OK;
}

esp_err_t twai_driver_uninstall() {
    if (!C.installed) return ESP_ERR_INVALID_STATE;
    if (C.state != TWAI_STATE_STOPPED && C.state != TWAI_STATE_BUS_OFF) return ESP_ERR_INVALID_STATE;
    // The real driver deletes the semaphores its waiters block on.
    if (C.waitingAlerts || C.waitingRx) {
        bugUninstallWhileWaiting++;
        simLog('E', "SIM-BUG: driver uninstalled while a task waits inside it (%d alerts, %d rx)",
               C.waitingAlerts, C.waitingRx);
    }
    C.installed = false;
    simrtos::notify(&C.rxw);
    simrtos::notify(&C.alertw);
    return ESP_OK;
}

esp_err_t twai_start() {
    if (!C.installed || C.state != TWAI_STATE_STOPPED) return ESP_ERR_INVALID_STATE;
    C.state = TWAI_STATE_RUNNING;
    // twai_hal_start(): TEC and REC cleared, reset mode left. (With ESP-IDF's
    // listen-only erratum fix it would set REC to 128 instead; the Arduino core
    // is built without it.)
    C.tec = C.rec = 0;
    C.inReset = false;
    flagCheck();
    return ESP_OK;
}

esp_err_t twai_stop() {
    if (!C.installed || C.state != TWAI_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
    C.state = TWAI_STATE_STOPPED;
    C.inReset = true;
    C.rx.clear();
    flagCheck();
    return ESP_OK;
}

esp_err_t twai_initiate_recovery() {
    if (!C.installed || C.state != TWAI_STATE_BUS_OFF) return ESP_ERR_INVALID_STATE;
    C.state = TWAI_STATE_RECOVERING;
    C.recoveryDoneAt = simrtos::nowUs() + 3000;      // 128 x 11 recessive bits
    return ESP_OK;
}

esp_err_t twai_get_status_info(twai_status_info_t *s) {
    if (!C.installed) return ESP_ERR_INVALID_STATE;
    *s = {};
    s->state = C.state;
    s->msgs_to_rx = (uint32_t)C.rx.size();
    s->tx_error_counter = C.tec;
    s->rx_error_counter = C.rec;
    s->rx_missed_count = C.rxMissed;
    s->bus_error_count = C.busErrors;
    return ESP_OK;
}

esp_err_t twai_receive(twai_message_t *m, TickType_t ticks) {
    const uint64_t deadline = ticks == portMAX_DELAY ? simrtos::FOREVER : simrtos::nowUs() + simTicksUs(ticks);
    for (;;) {
        if (!C.installed) return ESP_ERR_INVALID_STATE;
        const uint64_t now = simrtos::nowUs();
        if (!C.rx.empty() && now >= C.rxHoldUntil) { *m = C.rx.front(); C.rx.pop_front(); return ESP_OK; }
        if (ticks == 0 || now >= deadline) return ESP_ERR_TIMEOUT;
        // Held (a stall): wake when it ends, or at the deadline.
        uint64_t until = deadline;
        if (!C.rx.empty() && C.rxHoldUntil > now) until = std::min(until, C.rxHoldUntil);
        C.waitingRx++;
        simrtos::block(&C.rxw, until == simrtos::FOREVER ? until : until - now);
        C.waitingRx--;
    }
}

esp_err_t twai_read_alerts(uint32_t *alerts, TickType_t ticks) {
    const uint64_t deadline = ticks == portMAX_DELAY ? simrtos::FOREVER : simrtos::nowUs() + simTicksUs(ticks);
    for (;;) {
        if (!C.installed) return ESP_ERR_INVALID_STATE;
        if (C.alerts & C.alertsEnabled) {
            *alerts = C.alerts & C.alertsEnabled;
            C.alerts = 0;
            return ESP_OK;
        }
        if (ticks == 0 || simrtos::nowUs() >= deadline) { *alerts = 0; return ESP_ERR_TIMEOUT; }
        C.waitingAlerts++;
        simrtos::block(&C.alertw, deadline == simrtos::FOREVER ? deadline : deadline - simrtos::nowUs());
        C.waitingAlerts--;
    }
}

/* ═════════════════════════ scheduled bus traffic ═════════════════════════ */

static std::multimap<uint64_t, twai_message_t> s_pending;

static twai_message_t frame(uint32_t id, std::initializer_list<uint8_t> b) {
    twai_message_t m = {};
    m.identifier = id;
    m.data_length_code = 8;
    size_t i = 0;
    for (uint8_t v : b) if (i < 8) m.data[i++] = v;
    return m;
}

static void schedule(uint64_t atUs, const twai_message_t &m) { s_pending.emplace(atUs, m); }

static void deliver(const twai_message_t &m) {
    if (!C.installed || C.state != TWAI_STATE_RUNNING || C.inReset) return;
    if (faults.radioCorruptRate > 0 && radioOnAir(simrtos::nowUs()) && urand() < faults.radioCorruptRate) {
        sim::radioMisreads++;
        masterDetects(masterFlagsActive());   // errors let pass: lost to the master only
        return;
    }
    // A frame received without a bus error winds the receive-error counter back
    // down, as a real controller does (ISO 11898-1: by 1, or from above 127 to
    // 119..127) - so a trickle of stray errors keeps REC near zero, and only a
    // storm drives it up. Listen-only mode freezes the counters.
    if (C.mode != TWAI_MODE_LISTEN_ONLY || faults.lomRecCounts) {
        if (C.rec > 127) C.rec = 120;
        else if (C.rec) C.rec--;
        flagCheck();
    }
    if (C.rx.size() >= C.rxLen) { C.rxMissed++; raiseAlert(TWAI_ALERT_RX_QUEUE_FULL); return; }
    C.rx.push_back(m);
    simrtos::notify(&C.rxw);
}

/**
 * The transmission ECU (TCM) and the master's effect on the ECM's broadcasts.
 *
 * These frames are what the TCM needs; P1718 is it not getting them.
 */
static const uint32_t ECM_TO_TCM[] = { 0x231, 0x232 };
static bool isEcmToTcm(uint32_t id) {
    for (uint32_t x : ECM_TO_TCM) if (x == id) return true;
    return false;
}

static struct Tcm {
    bool     p1718 = false, p0700 = false, armed = false;
    uint32_t rx = 0, lost = 0, destroyed = 0, misreads = 0;
    std::deque<uint64_t> recent;   // arrival times, for the sliding window
} T;

/* The engine ECU as a CAN transmitter, with the protocol's fault confinement:
 * +8 on its transmit error counter for every attempt destroyed, -1 for every
 * frame that gets through, bus-off at 256. Recovery is quick for the first few
 * bus-offs and slow after that, the way automotive ECUs treat a bus they keep
 * losing (AUTOSAR CanSM: fast then slow recovery). While it is bus-off nobody
 * receives its broadcasts - the TCM included. */
static struct Ecm {
    uint32_t tec = 0;
    uint64_t offUntil = 0, last = 0;
    int      recent = 0;
    uint32_t total = 0;
} E;

static void ecmBusOff(uint64_t now) {
    if (now - E.last > 10000000) E.recent = 0;        // 10 s clean: quick recovery again
    E.recent++; E.total++; E.last = now; E.tec = 0;
    E.offUntil = now + (E.recent <= 5 ? 10000 : 1000000);
    simLog('W', "SIM-ECM: driven bus-off (#%u, %s recovery)", (unsigned)E.total,
           E.recent <= 5 ? "quick" : "slow");
}

static constexpr uint64_t TCM_WINDOW_US = 1500000;  // 1.5 s
static constexpr size_t   TCM_MIN_RX    = 30;       // ~150/s healthy; a real gap is far below

TcmView sim::tcm() {
    return { T.p1718, T.p0700, (uint32_t)T.rx, (uint32_t)T.lost, (uint32_t)T.recent.size(), T.armed,
             T.destroyed, E.total, T.misreads };
}

/** Would the master's controller put an ACTIVE (dominant) error flag on the
 *  wire for an error it detected in a frame it only receives? */
static bool masterFlagsActive() {
    if (!C.installed || C.state != TWAI_STATE_RUNNING || C.inReset || !C.txAttached) return false;
    if (C.rec >= 128 || C.tec >= 128) return false;      // error-passive: recessive flags only
    // NORMAL: as the standard says. LISTEN_ONLY: the erratum - it flags anyway.
    return C.mode == TWAI_MODE_NORMAL || C.mode == TWAI_MODE_LISTEN_ONLY;
}

int sim::activeFlagStarts = 0;
static bool s_couldFlag = false;
/** Count each time the controller becomes able to flag actively. Called
 *  wherever its state, counters or pad routing change. */
static void flagCheck() {
    const bool could = masterFlagsActive();
    if (could && !s_couldFlag) sim::activeFlagStarts++;
    s_couldFlag = could;
}

/** Latch an error in the error-code-capture register, as the controller does. */
static void eccLatch(int errc, int dir, int seg) {
    TWAI.error_code_capture_reg.val = (uint32_t)((errc << 6) | (dir << 5) | seg);
}

/** The master's controller detected an error in a frame it was receiving. */
static void masterDetects(bool flaggedActive) {
    C.busErrors++;
    eccLatch(2, 1, 0x0A);                              // stuff error, receiving, data field
    raiseAlert(TWAI_ALERT_BUS_ERROR);
    if (C.mode == TWAI_MODE_LISTEN_ONLY) {
        // An error-active listen-only controller with its TX pad detached
        // flags into a line that stays recessive: a bit error in its own
        // flag, and again, each round counted (see faults.lomRecCounts).
        if (faults.lomRecCounts && !C.txAttached) {
            for (int round = 0; round < 32 && C.rec < 128 && C.tec < 128; round++) {
                C.busErrors++;
                eccLatch(0, 1, 0x11);                  // bit error in its own active flag
                C.rec += 8;
            }
        }
        if (!faults.lomRecCounts) return;              // counters frozen in listen-only
    }
    // +1 for the error, +8 more when after its own (primary) flag it sees the
    // other nodes' secondary flags: the rule that makes the node which alone
    // sees an error the first to go error-passive.
    C.rec += flaggedActive ? 9 : 1;
    if (C.rec >= 96)  raiseAlert(TWAI_ALERT_ABOVE_ERR_WARN);
    if (C.rec >= 128) raiseAlert(TWAI_ALERT_ERR_PASS);
}

/**
 * One ECM broadcast frame goes onto the bus, as the TCM (and the master) see
 * it. On a marginal link the master misreads it - and every retransmission of
 * it. While the master's error flags are active and reach the wire, each
 * attempt is destroyed for every module and the ECM retransmits; the loop ends
 * when the master goes error-passive (its REC outruns the ECM's TEC) or the ECM
 * is driven bus-off (a frozen REC never does).
 * @return true if the master receives the frame (deliver it to the controller).
 */
static bool tcmSee(const twai_message_t &m) {
    if (!isEcmToTcm(m.identifier)) return true;
    const uint64_t now = simrtos::nowUs();
    if (now < E.offUntil) { T.lost++; return false; }  // the ECM is bus-off
    const bool misread = faults.rxCorruptRate > 0 && C.installed && C.state == TWAI_STATE_RUNNING &&
                         !C.inReset && urand() < faults.rxCorruptRate;
    if (misread) {
        T.misreads++;
        while (masterFlagsActive()) {                  // this attempt destroyed on the wire
            masterDetects(true);
            T.destroyed++;
            E.tec += 8;
            if (E.tec >= 256) { ecmBusOff(now); T.lost++; return false; }
        }
        masterDetects(false);      // flagged recessively, or not at all: the frame gets through
    }
    if (E.tec) E.tec--;
    T.rx++;
    T.recent.push_back(now);
    return !misread;               // the master did not get the frame it misread
}

/** Latch P1718 when the ECM's broadcasts thin out on an otherwise live bus. */
static void tcmEvaluate(uint64_t now) {
    while (!T.recent.empty() && T.recent.front() + TCM_WINDOW_US < now) T.recent.pop_front();
    if (faults.silenceBus) return;          // car asleep: no broadcasts expected
    if (T.recent.size() >= TCM_MIN_RX) T.armed = true;
    if (T.armed && T.recent.size() < TCM_MIN_RX) {
        if (!T.p1718) simLog('W', "SIM-TCM: P1718 - ECM broadcasts lost (%zu in %llu ms)",
                              T.recent.size(), (unsigned long long)(TCM_WINDOW_US / 1000));
        T.p1718 = T.p0700 = true;
    }
}

/* ═════════════════════════════════ the car ══════════════════════════════ */

static double profile(const double *lv, int n, double holdS, double t) {
    const double per = holdS + 0.6;
    const long i = (long)(t / per);
    const double ph = t - i * per;
    const double a = lv[i % n], b = lv[(i + 1) % n];
    return ph < holdS ? a : a + (b - a) * (ph - holdS) / 0.6;
}

/* SIM_SEED=n varies the drive: every level moved by a random amount, so no
 * run depends on round numbers (RPM at exact hundreds, say). */
static void levels(double *rpm, double *spd, double *thr) {
    static const double R[] = {800, 2100, 3300, 1500, 2700, 1100, 3800, 1900};
    static const double S[] = {0, 30, 55, 20, 80, 45, 10, 65};
    static const double T[] = {0, 18, 42, 8, 65, 25, 0, 35};
    const char *e = std::getenv("SIM_SEED");
    std::mt19937 g(e ? (unsigned)std::atoi(e) : 0);
    for (int i = 0; i < 8; i++) {
        const double k = e ? 1 : 0;
        rpm[i] = R[i] + k * std::uniform_real_distribution<double>(-140, 140)(g);
        spd[i] = std::max(0.0, S[i] + k * std::uniform_real_distribution<double>(-6, 6)(g));
        thr[i] = std::max(0.0, T[i] + k * std::uniform_real_distribution<double>(-4, 4)(g));
    }
}

Truth sim::truth() {
    const double t = elapsedS();
    static double RPM[8], SPD[8], THR[8];
    static bool init = false;
    if (!init) { levels(RPM, SPD, THR); init = true; }
    Truth r;
    r.rpm = t < 8 ? 800 : profile(RPM, 8, 2.5, t);
    r.speed = t < 8 ? 0 : profile(SPD, 8, 3.0, t);
    r.throttle = t < 8 ? 0 : profile(THR, 8, 2.2, t);
    r.coolant = 20 + std::min(70.0, t * 0.5);
    r.gear = r.speed < 15 ? 1 : r.speed < 30 ? 2 : r.speed < 50 ? 3 : r.speed < 70 ? 4 : 5;
    r.brake = std::fmod(t, 7.0) < 1.2;
    r.lights = t >= 60 && t < 120;
    return r;
}

static uint8_t s_cnt = 0;

sim::Body sim::body;
static double s_turnSince = -1;       ///< When the lamp's flashing started.
bool sim::turnLampLit() {
    if (!body.turnLeft) { s_turnSince = -1; return false; }
    const double t = elapsedS();
    if (s_turnSince < 0) s_turnSince = t;
    return std::fmod(t - s_turnSince, 0.7) < 0.35;   // lit first, like a flasher
}

/** The body/transmission frame (see Body); due every 50 ms while enabled. */
static void bodyFrame(uint64_t now) {
    static uint64_t due = 0;
    if (!body.enabled) { due = 0; return; }
    if (due > now) return;
    due = due && now - due < 50250 ? due + 50250 : now + 50250;
    static uint8_t cnt = 0;
    uint8_t lever = 7;
    switch (body.lever) { case 'P': lever = 0; break; case 'R': lever = 1; break;
                          case 'N': lever = 2; break; case 'D': lever = 4; break; default: break; }
    // The same lever as scattered bits too: P bit 24, D bit 30, N bit 39 held
    // low only in N, R bit 42 - and bit 27, which flips every 1.5 s by itself.
    const char lv = body.lever;
    const uint8_t b3 = (uint8_t)((lv == 'P' ? 0x01 : 0) | (lv == 'D' ? 0x40 : 0) |
                                 (((int)(elapsedS() / 1.5)) & 1 ? 0x08 : 0));
    const uint8_t b4 = lv == 'N' ? 0x00 : 0x80;
    const uint8_t b5 = lv == 'R' ? 0x04 : 0x00;
    const twai_message_t m = frame(0x3D1, {(uint8_t)(lever << 4), body.doors,
                                           (uint8_t)(turnLampLit() ? 1 : 0), b3, b4, b5, 0, cnt++});
    if (tcmSee(m)) deliver(m);
}

/** @return When the next broadcast is due. */
static uint64_t carFrames(uint64_t now, std::map<uint32_t, uint64_t> &next) {
    bodyFrame(now);
    const Truth v = truth();
    struct Def { uint32_t id; uint32_t periodUs; };
    static const Def DEFS[] = {{0x002, 10000}, {0x231, 10000}, {0x232, 20000}, {0x252, 20000},
                               {0x3B1, 50000}, {0x360, 100000}, {0x491, 100000}, {0x331, 100000},
                               {0x332, 100000}, {0x705, 1000000}, {0x706, 1000000}};
    for (const Def &d : DEFS) {
        uint64_t &due = next[d.id];
        if (due > now) continue;
        // On its own clock, as an ECU sends it; a clock more than a period
        // behind (the bus was silenced) starts again from now. The car's
        // modules run on their own crystals, not the master's: 0.5 % apart
        // here, so the master's radio does not lock to the car's frames the
        // way a single simulated clock would make it.
        const uint64_t per = d.periodUs * 1005 / 1000;
        due = due && now - due < per ? due + per : now + per;
        s_cnt++;
        twai_message_t m = {};
        const double t = elapsedS();
        switch (d.id) {
            case 0x002: {
                const int16_t st = (int16_t)(std::sin(t / 3.0) * 900);   // steering x0.1 deg
                m = frame(0x002, {(uint8_t)st, (uint8_t)(st >> 8), s_cnt, 0x70, 0x09, 0x7A, 0, 0});
                break;
            }
            case 0x231: {
                const uint16_t r4 = (uint16_t)(v.rpm * 4);
                m = frame(0x231, {s_cnt, 0x00, (uint8_t)r4, (uint8_t)(r4 >> 8),
                                  (uint8_t)(rng() & 0xFF), 0x49, 0x27, (uint8_t)(s_cnt & 0x0F)});
                break;
            }
            case 0x232:
                m = frame(0x232, {(uint8_t)std::lround(v.throttle / 0.392157), 0x10,
                                  (uint8_t)((int)v.gear | 0x40), 0, 0, 0, 0, (uint8_t)(s_cnt & 0x0F)});
                break;
            case 0x252: {
                const uint16_t s = (uint16_t)(v.speed / 0.05625);
                m = frame(0x252, {0x80, 0x00, (uint8_t)(s >> 8), (uint8_t)s, 0x68,
                                  (uint8_t)(rng() & 3), 0x81, (uint8_t)(s_cnt & 7)});
                break;
            }
            case 0x3B1:
                m = frame(0x3B1, {0, (uint8_t)(v.lights ? 0x10 : 0), 0xFD, 0, 0,
                                  (uint8_t)(0x01 | (v.brake ? 0x04 : 0)), 0, 0});
                break;
            case 0x360:
                m = frame(0x360, {0, 0, 0, (uint8_t)(v.coolant + 40), 0, 0, 0, 0});
                break;
            case 0x491:
                m = frame(0x491, {0, 0, 0, (uint8_t)(120 - t * 0.02), 0, 0, 0, 0});
                break;
            case 0x331: m = frame(0x331, {(uint8_t)(t * 2.5), 0, 0, 0, 0, 0, 0, 0}); break;
            case 0x332: m = frame(0x332, {s_cnt, (uint8_t)(s_cnt * 7), 0, 0, 0, 0, 0, 0}); break;
            default:    m = frame(d.id, {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}); break;
        }
        if (tcmSee(m)) deliver(m);      // the TCM watches the ECM's broadcasts
    }
    uint64_t soonest = UINT64_MAX;
    for (const Def &d : DEFS) soonest = std::min(soonest, next[d.id]);
    if (body.enabled) soonest = std::min<uint64_t>(soonest, now + 50250);
    // Stress: many more identifiers than the census holds, each at 20 Hz,
    // spread over time as a real bus would carry them (not in one burst).
    static uint64_t extraLast = 0;
    static int extraNext = 0;
    if (faults.extraIds) {
        const uint64_t dt = extraLast ? now - extraLast : 0;
        extraLast = now;
        const int n = (int)std::min<uint64_t>(faults.extraIds, dt * faults.extraIds / 50000 + 1);
        for (int k = 0; k < n; k++, extraNext = (extraNext + 1) % faults.extraIds)
            deliver(frame(0x500 + extraNext, {(uint8_t)extraNext, s_cnt, 0, 0, 0, 0, 0, 0}));
    }
    return soonest;
}

/* ═══════════════════════════ OBD-II responders ═══════════════════════════ */

static const uint8_t ECM_PIDS[] = {0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                                   0x10, 0x11, 0x13, 0x14, 0x15, 0x1C, 0x1F, 0x20, 0x21, 0x2E, 0x2F,
                                   0x30, 0x31, 0x33, 0x3C, 0x40, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
                                   0x49, 0x4A, 0x4C};
static const uint8_t TCM_PIDS[] = {0x01, 0x05, 0x0D, 0x20, 0x21};

template <size_t N>
static bool has(const uint8_t (&set)[N], uint8_t pid) {
    for (uint8_t p : set) if (p == pid) return true;
    return false;
}

template <size_t N>
static void bitmap(const uint8_t (&set)[N], uint8_t base, uint8_t out[4]) {
    std::memset(out, 0, 4);
    for (uint8_t p : set)
        if (p > base && p <= base + 32) {
            const int i = p - base - 1;
            out[i / 8] |= (uint8_t)(0x80 >> (i % 8));
        }
}

/** Service-01 data bytes for @p pid from the car's state; returns the count. */
static int obdData(uint8_t pid, uint8_t *d) {
    const Truth v = truth();
    const double t = elapsedS();
    auto pct = [](double p) { return (uint8_t)std::lround(std::max(0.0, std::min(100.0, p)) * 2.55); };
    switch (pid) {
        case 0x01: d[0] = 0x00; d[1] = 0x07; d[2] = 0x65; d[3] = 0x04; return 4;
        case 0x03: d[0] = 2; d[1] = 0; return 2;
        case 0x04: d[0] = pct(v.throttle * 0.8 + 15); return 1;
        case 0x05: d[0] = (uint8_t)(v.coolant + 40); return 1;
        case 0x06: case 0x07: d[0] = 128 + 3; return 1;
        case 0x0B: d[0] = (uint8_t)(30 + v.throttle * 0.8); return 1;
        case 0x0C: { const uint16_t r = (uint16_t)(v.rpm * 4); d[0] = r >> 8; d[1] = r & 0xFF; return 2; }
        case 0x0D: d[0] = (uint8_t)std::lround(v.speed); return 1;
        case 0x0E: d[0] = (uint8_t)((10 + v.rpm / 200 + 64) * 2); return 1;
        case 0x0F: d[0] = 25 + 40; return 1;
        case 0x10: { const uint16_t m = (uint16_t)(v.rpm * v.throttle / 10); d[0] = m >> 8; d[1] = m & 0xFF; return 2; }
        case 0x11: d[0] = pct(v.throttle); return 1;
        case 0x13: d[0] = 0x03; return 1;
        case 0x14: case 0x15: d[0] = 90; d[1] = 0xFF; return 2;
        case 0x1C: d[0] = 1; return 1;
        case 0x1F: { const uint16_t s = (uint16_t)t; d[0] = s >> 8; d[1] = s & 0xFF; return 2; }
        case 0x21: d[0] = 0; d[1] = 0; return 2;
        case 0x2E: d[0] = 20; return 1;
        case 0x2F: d[0] = pct(60 - t * 0.01); return 1;
        case 0x30: d[0] = 12; return 1;
        case 0x31: d[0] = 0x04; d[1] = 0xD2; return 2;
        case 0x33: d[0] = 101; return 1;
        case 0x3C: { const uint16_t c = (uint16_t)((500 + 40) * 10); d[0] = c >> 8; d[1] = c & 0xFF; return 2; }
        case 0x42: { const uint16_t mv = (uint16_t)(14100 + 200 * std::sin(t)); d[0] = mv >> 8; d[1] = mv & 0xFF; return 2; }
        case 0x43: { const uint16_t l = pct(v.throttle); d[0] = 0; d[1] = (uint8_t)l; return 2; }
        case 0x44: d[0] = 0x80; d[1] = 0x00; return 2;
        case 0x45: case 0x47: case 0x49: case 0x4A: case 0x4C: d[0] = pct(v.throttle); return 1;
        case 0x46: d[0] = 22 + 40; return 1;
        default: return 0;
    }
}

static void obdReply(uint32_t fromId, uint8_t pid, const uint8_t *data, int n, uint64_t at) {
    twai_message_t m = {};
    m.identifier = fromId;
    m.data_length_code = 8;
    m.data[0] = (uint8_t)(2 + n);
    m.data[1] = 0x41;
    m.data[2] = pid;
    std::memcpy(&m.data[3], data, n);
    schedule(at, m);
}

static void obdRequest(uint32_t id, uint8_t pid) {
    obdRequests++;
    const uint64_t now = simrtos::nowUs();
    const bool functional = id == 0x7DF;
    static int n = 0;
    const bool drop = ecu.dropReplyEveryN && ++n % ecu.dropReplyEveryN == 0;
    // Engine ECU.
    if (ecu.obdEnabled && (functional || ecu.obdPhysical) && !drop) {
        uint8_t d[4];
        if ((pid & 0x1F) == 0 && pid <= 0xC0) {
            if (pid == 0 || has(ECM_PIDS, pid)) { bitmap(ECM_PIDS, pid, d); obdReply(0x7E8, pid, d, 4, now + ecu.replyLatencyUs); }
        } else if (has(ECM_PIDS, pid)) {
            const int k = obdData(pid, d);
            obdReply(0x7E8, pid, d, k, now + ecu.replyLatencyUs);
        } else if (!functional && ecu.obdNrcUnsupported) {
            schedule(now + ecu.replyLatencyUs, frame(0x7E8, {0x03, 0x7F, 0x01, 0x31, 0, 0, 0, 0}));
        }
    }
    // Transmission ECU: functional requests only, and FIRST (faster).
    if (functional && ecu.tcmEnabled) {
        uint8_t d[4];
        if ((pid & 0x1F) == 0 && pid <= 0xC0) {
            if (pid == 0 || has(TCM_PIDS, pid)) { bitmap(TCM_PIDS, pid, d); obdReply(0x7E9, pid, d, 4, now + 1500); }
        } else if (has(TCM_PIDS, pid)) {
            const int k = obdData(pid, d);
            obdReply(0x7E9, pid, d, k, now + 1500);
        }
    }
}

/* ═══════════════════════════ SSM2 responder ══════════════════════════════ */

static struct IsoRx { std::vector<uint8_t> buf; size_t total = 0; uint8_t sn = 0; bool active = false; } s_isoRx;
static struct IsoTx { std::vector<uint8_t> msg; size_t off = 0; uint8_t sn = 1; bool waitingFc = false; } s_isoTx;

static uint8_t ssmByte(uint32_t a) {
    const Truth v = truth();
    const uint16_t rpm4 = (uint16_t)(v.rpm * 4);
    switch (a) {
        case 0x07: return (uint8_t)std::lround((v.throttle * 0.8 + 15) * 2.55);
        case 0x08: return (uint8_t)(v.coolant + 40);
        case 0x09: case 0x0A: case 0x0B: case 0x0C: return 128;
        case 0x0D: return (uint8_t)(30 + v.throttle * 0.8);
        case 0x0E: return rpm4 >> 8;
        case 0x0F: return rpm4 & 0xFF;
        case 0x10: return (uint8_t)std::lround(v.speed);
        case 0x11: return (uint8_t)((10 + v.rpm / 200) * 2 + 128);
        case 0x12: return 25 + 40;
        case 0x15: return (uint8_t)std::lround(v.throttle * 2.55);
        case 0x1C: return (uint8_t)(14.1 / 0.08);
        case 0x20: return 20;
        case 0x22: return v.rpm > 3500 ? 125 : 128;           // knock correction
        case 0x23: return 101;
        case 0x24: return (uint8_t)(128 - 70 + v.throttle);
        case 0x29: return (uint8_t)std::lround(v.throttle * 2.55);
        case 0x46: return 128;                                // lambda 1.0
        case 0x4A: return (uint8_t)(v.gear - 1);
        case 0x113: return 95 + 40;
        case 0x64: return v.lights ? 0x08 : 0;                // bit 4 = lights
        case 0x121: return v.brake ? 0x48 : 0;                // brake + stop light
        default: return 0;
    }
}

static void ecuSend(const std::vector<uint8_t> &msg, uint64_t at) {
    if (msg.size() <= 7) {
        twai_message_t m = frame(0x7E8, {});
        m.data[0] = (uint8_t)msg.size();
        std::memcpy(&m.data[1], msg.data(), msg.size());
        schedule(at, m);
        return;
    }
    twai_message_t m = frame(0x7E8, {});
    m.data[0] = (uint8_t)(0x10 | ((msg.size() >> 8) & 0x0F));
    m.data[1] = (uint8_t)(msg.size() & 0xFF);
    std::memcpy(&m.data[2], msg.data(), 6);
    schedule(at, m);
    s_isoTx = {msg, 6, 1, true};                          // CFs follow our flow control
}

static void ssmHandle(const std::vector<uint8_t> &req) {
    ssmRequests++;
    const uint64_t at = simrtos::nowUs() + ecu.replyLatencyUs;
    if (req[0] == 0xAA) {
        std::vector<uint8_t> r = {0xEA, 0xA1, 0x10, 0x09, 0x5B, 0x44, 0x3C, 0x41, 0x07};
        for (int i = 0; i < 96; i++) r.push_back(0xFF);   // every parameter supported
        ecuSend(r, at);
        return;
    }
    if (req[0] == 0xA8 && req.size() >= 5 && (req.size() - 2) % 3 == 0) {
        const size_t n = (req.size() - 2) / 3;
        bool bad = false;
        std::vector<uint8_t> r = {0xE8};
        for (size_t i = 0; i < n; i++) {
            const uint32_t a = (uint32_t)req[2 + 3 * i] << 16 | req[3 + 3 * i] << 8 | req[4 + 3 * i];
            if (ecu.ssmBadAddress && a == ecu.ssmBadAddress) bad = true;
            r.push_back(ssmByte(a));
        }
        if (bad || (ecu.ssmMaxAddrs && (int)n > ecu.ssmMaxAddrs)) {
            if (bad) ssmReadsWithBad++;
            ecuSend({0x7F, 0xA8, 0x31}, at);
            return;
        }
        ecuSend(r, at);
        return;
    }
    ecuSend({0x7F, req[0], 0x11}, at);                    // service not supported
}

/** A frame the master sent to 0x7E0 that is not an OBD-II request. */
static void ssmFrame(const twai_message_t &m) {
    const uint64_t now = simrtos::nowUs();
    const uint8_t pci = m.data[0] & 0xF0;
    if (pci == 0x30) {                                    // our flow control for the ECU's reply
        if (!s_isoTx.waitingFc) { simLog('E', "SIM-BUG: flow control with no reply in progress"); return; }
        if ((m.data[0] & 0x0F) == 2) { s_isoTx.waitingFc = false; return; }   // overflow: abort
        s_isoTx.waitingFc = false;
        uint64_t t = now + 500;
        static int replies = 0;
        size_t end = s_isoTx.msg.size();
        if (ecu.ssmCutEveryN && ++replies % ecu.ssmCutEveryN == 0) end = 6 + 7;   // one CF, then silence
        while (s_isoTx.off < end) {
            twai_message_t cf = frame(0x7E8, {});
            cf.data[0] = (uint8_t)(0x20 | (s_isoTx.sn++ & 0x0F));
            const size_t k = std::min((size_t)7, s_isoTx.msg.size() - s_isoTx.off);
            std::memcpy(&cf.data[1], &s_isoTx.msg[s_isoTx.off], k);
            s_isoTx.off += k;
            schedule(t, cf);
            t += 600;
        }
        return;
    }
    if (pci == 0x00) {
        const uint8_t len = m.data[0] & 0x0F;
        if (len >= 1 && len <= 7) ssmHandle(std::vector<uint8_t>(m.data + 1, m.data + 1 + len));
        return;
    }
    if (pci == 0x10) {
        s_isoRx.total = (size_t)(m.data[0] & 0x0F) << 8 | m.data[1];
        s_isoRx.buf.assign(m.data + 2, m.data + 8);
        s_isoRx.sn = 1;
        s_isoRx.active = true;
        schedule(now + 1000, frame(0x7E8, {0x30, 0x00, ecu.ssmStmin, 0, 0, 0, 0, 0}));
        return;
    }
    if (pci == 0x20) {
        if (!s_isoRx.active) { simLog('E', "SIM-BUG: consecutive frame with no first frame"); return; }
        if ((m.data[0] & 0x0F) != (s_isoRx.sn & 0x0F)) {
            simLog('E', "SIM-BUG: consecutive frame out of order (got %u, want %u)",
                   m.data[0] & 0x0F, s_isoRx.sn & 0x0F);
            s_isoRx.active = false;
            return;
        }
        s_isoRx.sn++;
        const size_t k = std::min((size_t)7, s_isoRx.total - s_isoRx.buf.size());
        s_isoRx.buf.insert(s_isoRx.buf.end(), m.data + 1, m.data + 1 + k);
        if (s_isoRx.buf.size() >= s_isoRx.total) {
            s_isoRx.active = false;
            ssmHandle(s_isoRx.buf);
        }
    }
}

/* ═════════════════════════ our transmissions ═════════════════════════════ */

esp_err_t twai_transmit(const twai_message_t *m, TickType_t) {
    if (!C.installed) return ESP_ERR_INVALID_STATE;
    if (C.mode == TWAI_MODE_LISTEN_ONLY) return ESP_ERR_NOT_SUPPORTED;
    if (!C.txAttached) {
        simLog('E', "SIM-BUG: transmit with the TX pad routed away from the controller");
        return ESP_FAIL;
    }
    if (C.state != TWAI_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
    transmitted.push_back({simrtos::nowUs(), *m});
    if (faults.errorPerOwnFrame > 0 && urand() < faults.errorPerOwnFrame) {
        C.busErrors++;
        eccLatch(0, 0, 0x0A);                              // bit error, sending, data field
        C.tec += 8;
        raiseAlert(TWAI_ALERT_BUS_ERROR);
        if (C.tec >= 128) raiseAlert(TWAI_ALERT_ERR_PASS);
        if (C.tec > 255) {
            C.state = TWAI_STATE_BUS_OFF;
            raiseAlert(TWAI_ALERT_BUS_OFF);
            return ESP_OK;                                  // the frame is lost
        }
    } else if (C.tec) {
        C.tec--;
    }
    flagCheck();
    if (m->identifier == 0x7DF || m->identifier == 0x7E0) {
        if (m->data[0] == 0x02 && m->data[1] == 0x01) obdRequest(m->identifier, m->data[2]);
        else if (m->identifier == 0x7E0 && ecu.ssmEnabled) ssmFrame(*m);
    }
    return ESP_OK;
}

/* ═════════════════════════════ the bus task ═════════════════════════════ */

static void busTask(void *) {
    std::map<uint32_t, uint64_t> next;
    uint64_t lastIdle = 0;
    for (;;) {
        const uint64_t now = simrtos::nowUs();
        while (!s_pending.empty() && s_pending.begin()->first <= now) {
            const twai_message_t m = s_pending.begin()->second;
            s_pending.erase(s_pending.begin());
            deliver(m);
        }
        radioCallbacks(now);
        if (faults.rxStallEveryMs && faults.rxStallMs) {
            static uint64_t nextStall = 0;
            if (!nextStall) nextStall = now + faults.rxStallEveryMs * 1000ull;
            if (now >= nextStall) {
                nextStall = now + faults.rxStallEveryMs * 1000ull;
                C.rxHoldUntil = now + faults.rxStallMs * 1000ull;
                masterBusStall();                  // as the master's own flash writes do
            }
        }
        const uint64_t nextCar = faults.silenceBus ? UINT64_MAX : carFrames(now, next);
        tcmEvaluate(now);
        for (int i = 0; i < faults.burstFramesPerMs * 5; i++)          // 5 ms per pass
            deliver(frame(0x600 + (i & 0x3F), {(uint8_t)i, 0, 0, 0, 0, 0, 0, 0}));
        if (faults.idleErrorsPerSec > 0 && C.installed && C.state == TWAI_STATE_RUNNING &&
            urand() < faults.idleErrorsPerSec * (now - lastIdle) / 1e6) {
            C.busErrors++;
            eccLatch(1, 1, 0x1B);                          // form error, receiving, ACK delimiter
            if (C.mode != TWAI_MODE_LISTEN_ONLY) C.rec++;    // frozen in listen-only
            raiseAlert(TWAI_ALERT_BUS_ERROR);
        }
        lastIdle = now;
        if (faults.busOffNow && C.installed && C.mode == TWAI_MODE_NORMAL && C.state == TWAI_STATE_RUNNING) {
            faults.busOffNow = false;
            C.tec = 256;
            C.state = TWAI_STATE_BUS_OFF;
            raiseAlert(TWAI_ALERT_BUS_OFF);
            flagCheck();
        }
        if (C.installed && C.state == TWAI_STATE_RECOVERING && now >= C.recoveryDoneAt) {
            C.state = TWAI_STATE_STOPPED;
            C.tec = C.rec = 0;
            raiseAlert(TWAI_ALERT_BUS_RECOVERED);
            flagCheck();
        }
        uint64_t wake = std::min(now + 5000, nextCar);
        if (!s_pending.empty()) wake = std::min(wake, s_pending.begin()->first);
        if (!s_airDone.empty()) wake = std::min(wake, s_airDone.front().first);
        simrtos::sleepUs(std::max<uint64_t>(wake, now + 100) - now);
    }
}

void sim::start() {
    s_startUs = simrtos::nowUs();
    simrtos::createTask(busTask, "bus", nullptr, 20);
}

/* ── register-level access (hal/twai_ll.h), as the master's erratum fix uses ── */
twai_dev_t TWAI;
void twai_ll_enter_reset_mode(twai_dev_t *) { C.inReset = true; }
void twai_ll_exit_reset_mode(twai_dev_t *)  { C.inReset = false; flagCheck(); }

/**
 * The GPIO matrix routing the TX pad back to the controller, as
 * twai_driver_install() does it (gpio_config, then this call with the TWAI TX
 * signal). The master uses it once the controller is in its bus state.
 */
void esp_rom_gpio_connect_out_signal(uint32_t gpio, uint32_t signal, bool outInv, bool oenInv) {
    if (gpio != CAN_TX_GPIO || signal != TWAI_TX_IDX || outInv || oenInv) {
        simLog('E', "SIM-BUG: unexpected GPIO routing (pad %u, signal %u, inv %d/%d)",
               (unsigned)gpio, (unsigned)signal, outInv, oenInv);
        return;
    }
    if (!C.installed) simLog('E', "SIM-BUG: TX pad given to an uninstalled controller");
    if (C.installed && C.mode == TWAI_MODE_LISTEN_ONLY)
        simLog('E', "SIM-BUG: TX pad given to a listen-only controller (the erratum reaches the bus)");
    C.txAttached = true;
    flagCheck();
}
bool twai_ll_is_in_reset_mode(twai_dev_t *) { return C.inReset; }
uint32_t twai_ll_get_rec(twai_dev_t *)      { return C.rec; }
uint32_t twai_ll_get_tec(twai_dev_t *)      { return C.tec; }
void twai_ll_set_tec(twai_dev_t *, uint32_t tec) {
    if (!C.inReset) { simLog('E', "SIM-BUG: TEC written outside reset mode (ignored by the hardware)"); return; }
    C.tec = tec;
}
void twai_ll_set_rec(twai_dev_t *, uint32_t rec) {
    if (!C.inReset) { simLog('E', "SIM-BUG: REC written outside reset mode (ignored by the hardware)"); return; }
    C.rec = rec;
}
