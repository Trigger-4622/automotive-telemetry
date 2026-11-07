/**
 * @file sim.h
 * @brief The simulated world around the master: CAN bus, car, ECUs, radio,
 *        and what the tests can observe and inject.
 */
#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "driver/twai.h"
#include "esp_now.h"

namespace sim {

/* ── what the car does and what its ECUs support ───────────────────────── */
struct EcuConfig {
    bool     obdEnabled      = true;   ///< Engine ECU answers Service 01.
    bool     obdPhysical     = true;   ///< ...on 0x7E0 (else only 0x7DF).
    bool     obdNrcUnsupported = true; ///< Physical request for an unsupported PID: NRC 0x31.
    bool     tcmEnabled      = true;   ///< Transmission ECU answers functional requests.
    bool     ssmEnabled      = true;   ///< Engine ECU speaks SSM2.
    uint32_t ssmBadAddress   = 0;      ///< Read containing it: NRC 0x31 (0 = none).
    int      ssmMaxAddrs     = 0;      ///< More addresses than this: NRC 0x31 (0 = no limit).
    uint8_t  ssmStmin        = 0;      ///< STmin in the ECU's flow control.
    uint32_t replyLatencyUs  = 4000;   ///< ECU reply delay.
    int      dropReplyEveryN = 0;      ///< Drop every Nth OBD reply (0 = never).
    int      ssmCutEveryN    = 0;      ///< Stop every Nth multi-frame SSM reply half-way.
    /** Trouble codes each computer holds (raw, 0x1718 = P1718), on top of
     *  what the TCM model latches: P1718 in the TCM, P0700 in the ECM. */
    std::vector<uint16_t> ecmStored, ecmPending, tcmStored, tcmPending;
    bool     tcmPhysical     = true;   ///< TCM answers 0x7E1 (else only 0x7DF).
    bool     permanentCodes  = false;  ///< Service 0A supported (a 2009 car: no).
    bool     clearNeedsEngineOff = false;  ///< Service 04 refused (0x22) while it runs.
    bool     vehicleInfo     = true;   ///< Service 09 answered (false: NRC 0x11, as some ECUs).
    /** A request the computer lets go unanswered (a reply lost, a busy ECU):
     *  return true to send nothing. */
    std::function<bool(bool tcm, uint8_t svc)> ignoreIso;
    double   odometerKm      = 123456; ///< What SSM2 address 0x10E reads.
};

/* ── bus faults ─────────────────────────────────────────────────────────── */
struct BusFaults {
    double   errorPerOwnFrame = 0;     ///< Probability a frame we send hits a bus error.
    double   idleErrorsPerSec = 0;     ///< Errors not related to us.
    bool     busOffNow = false;        ///< Force the controller bus-off.
    bool     silenceBus = false;       ///< Car asleep: nothing is broadcast.
    int      extraIds = 0;             ///< More distinct IDs, 20 Hz each.
    int      burstFramesPerMs = 0;     ///< Flood: this many extra frames every ms.
    /**
     * A marginal link: the master's controller misreads this fraction of the
     * ECM's broadcast frames to the TCM - and, the fault being in how it samples
     * those bits, every retransmission of the same frame too. What that does to
     * the car follows the CAN fault-confinement rules (tcmSee in sim_bus.cpp):
     * an error-active controller whose TX pad is on the wire destroys each
     * attempt with an active error flag, gaining 9 on its REC while the ECM
     * gains 8 on its TEC, so a normal-mode master goes error-passive - and
     * harmless - before the ECM reaches bus-off. In LISTEN-ONLY mode the ESP32-S3
     * still sends active error flags (Espressif erratum; ESP-IDF's fix,
     * CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM, is off in the Arduino core) while
     * its error counters stay frozen: a listen-only master started with REC 0
     * never backs off, and the ECM is driven bus-off instead.
     */
    double   rxCorruptRate = 0;
    /**
     * What the portal showed after the listen-only fix: REC back at 0 in
     * listen-only with errors streaming. (That reading came through
     * twai_get_status_info(), which in ESP-IDF v4.4.7 reports 0 for any driver
     * installed listen-only - so it is a worst case kept, not something the
     * chip is known to do; Espressif documents the counters as frozen in
     * listen-only.) With this set, REC is not frozen in
     * listen-only (a good frame winds it down, an error winds it up), and an
     * error-active controller whose TX pad is detached reads its own active
     * error flag back recessive, calls it a bit error and flags again - each
     * round counted as a bus error (+8 on REC per round, or 32 rounds if the
     * counters do not move) - the inflated counts the portal showed.
     */
    bool     lomRecCounts = false;
    /**
     * What the S3 on the car did with a TEC written in reset mode: read back
     * one lower straight after leaving it (128 -> 127). On by default, as
     * that is the chip in the car.
     */
    uint32_t tecExitDrop = 1;
    /** A worse chip than seen: TEC winds down by one per frame received in
     *  listen-only, the way REC does. */
    bool     lomTecDrain = false;
    /** The next this-many TEC writes do not take: the register keeps the
     *  value it had (twai_start() left it at 0). */
    int      tecWriteIgnored = 0;
    /**
     * The master's radio disturbs its own CAN side - a 3.3 V rail sagging under
     * the transmit current: while a display broadcast is on air, the master
     * misreads this fraction of the frames on the bus (whoever sent them).
     */
    double   radioCorruptRate = 0;
    /** The next this many twai_driver_install() calls fail (ESP_ERR_NO_MEM). */
    int      installFailures = 0;
    /**
     * Nobody acknowledges the master's frames: every other module is asleep or
     * unpowered, as when the car is switched off under a request. A frame it
     * sends then stays in its transmit buffer and is retransmitted back to
     * back, each attempt an acknowledgement error (TEC +8 while error-active;
     * an error-passive sender's acknowledgement errors do not count, ISO
     * 11898-1, so it never goes bus-off) - until it is aborted, the controller
     * is stopped, or someone wakes up and acknowledges it.
     */
    bool     nobodyAcks = false;
    /**
     * A flash write holding the CAN interrupt off: every rxStallEveryMs the
     * received frames stay in the controller for rxStallMs, then come out in
     * one burst. The stall is announced to the master the way its own flash
     * writes do it (masterBusBlind).
     */
    uint32_t rxStallEveryMs = 0, rxStallMs = 0;
};

extern EcuConfig ecu;
extern BusFaults faults;
extern uint32_t  heapFree;             ///< What ESP.getFreeHeap() reports.

/* ── observations ───────────────────────────────────────────────────────── */
struct TxRecord { uint64_t tUs; twai_message_t msg; };
extern std::vector<TxRecord> transmitted;      ///< Every frame the master sent.
extern uint64_t obdRequests, ssmRequests;      ///< Requests the ECU received.
extern uint64_t ssmReadsWithBad;               ///< SSM reads refused for the bad address.

struct Shown { float value; uint8_t flags; uint64_t tUs; };
extern std::map<uint16_t, Shown> displayed;    ///< Latest value per metric at the displays.
extern uint64_t espPackets, espBadPackets, espSeqGaps;

extern bool sleepEntered;
extern int  logErrors, logWarnings;
extern std::vector<std::string> logLines;
extern bool verbose;

/* ── the transmission ECU (TCM) watching the engine ECU's broadcasts ──────────
 *
 * P1718 on this car means the TCM stopped receiving the ECM's periodic CAN
 * messages. The model watches the ECM's engine broadcast frames (0x231, 0x232)
 * and latches P1718 when, on a bus that is otherwise alive, too few of them
 * arrive in a one-and-a-half-second window - which is what the master's error
 * flags do to those frames while it is a normal-mode node on a marginal link.
 * P0700 is the TCM asking for the lamp because P1718 is set. */
struct TcmView {
    bool     p1718;        ///< Latched: the TCM lost the ECM's broadcasts.
    bool     p0700;        ///< Latched: the TCM asked for the MIL (follows P1718).
    uint32_t ecmRx;        ///< ECM broadcast frames the TCM received.
    uint32_t ecmLost;      ///< ECM broadcast frames destroyed before it got them.
    uint32_t windowRx;     ///< Received in the last window (what P1718 watches).
    bool     armed;        ///< Has seen the ECM healthy at least once.
    uint32_t destroyed;    ///< ECM transmission attempts destroyed on the wire.
    uint32_t ecmBusOffs;   ///< Times the ECM was driven bus-off.
    uint32_t misreads;     ///< ECM frames the master misread (each counts once).
};
TcmView tcm();

/** Reset reason esp_reset_reason() returns (default power-on). A scenario sets
 *  it to ESP_RST_BROWNOUT to replay a reboot during cranking. */
extern int resetReason;
/** The BOOT button (GPIO0) held down. */
extern bool bootButtonDown;
/** ESP.restart(): false (the default) ends the run as a failure (exit 5); true
 *  counts it in @ref restarts and carries on, for a scenario that expects it. */
extern bool restartReturns;
extern int  restarts;

/** CAN-TX recessive hold observation. gpioHoldPin/Enabled track gpio_hold_en on
 *  the TX pin (latched high across sleep); txDrivenHigh is its driven level. */
extern int  gpioHoldEnabled;   ///< The TX pin's pad hold: 1 latched, 0 not.
extern bool txHeld;            ///< The same, as the pad sees it: latched, nothing reaches it.
/** gpio_hold_en / gpio_hold_dis on the TX pin. */
void txHoldChanged(bool held);
extern bool txDrivenHigh;      ///< Last level driven onto the TX pin.
extern bool txConfiguredOut;   ///< TX was configured as a recessive output.
extern int  txGlitches;        ///< Times a GPIO (not the controller) drove TX dominant.
/** The master's radio: each ESP-NOW packet is on air for its length at
 *  1 Mbit/s plus preamble; the send callback fires when it is done. */
void radioQueue(size_t len, esp_now_send_cb_t cb);
extern uint64_t radioMisreads; ///< Frames misread because the radio was on air.
/** Times the master's controller came onto the bus able to send an ACTIVE
 *  (dominant) error flag: running, error-active, its TX pad routed to it. With
 *  errors let pass (tx_passive) this never happens - not at a start, not after
 *  the settle wait, not after a bus-off restart. */
extern int  activeFlagStarts;
/** Reset-mode entries of the master's own (TEC/REC writes) on a running
 *  controller: each is a moment it is deaf to the bus. */
extern uint64_t resetModeEntries;
/** twai_driver_install() calls made with the TX pad not latched: the driver
 *  routes the pad to the controller before it resets the peripheral. */
extern int installsUnheld;
extern int  radioQdbm;         ///< Last esp_wifi_set_max_tx_power() value (quarter dBm).
extern std::string serialOut;  ///< Everything written with Serial.printf.
/** The bit timing the last twai_driver_install() was given. */
struct TimingView { double samplePct; int sjw; bool triple; };
extern TimingView timing;
/** Put a frame of the test's own on the bus now (another module sent it). */
void injectFrame(const twai_message_t &m);
/** The TX pad was switched to a plain GPIO output, away from the controller
 *  (gpio_set_direction / pinMode): the pad now shows the GPIO latch's level. */
void txPadFromGpio();
/** Is the controller's TX signal routed to the pad (what twai_driver_install does)? */
bool txPadOnController();

struct CtrlView { bool installed; twai_mode_t mode; twai_state_t state; uint32_t tec, rec; };
CtrlView controller();
/** Time a frame of the master's spent being retransmitted onto a bus nobody
 *  acknowledges (faults.nobodyAcks), in microseconds. */
extern uint64_t deadBusTxUs;
/** Transmissions handed to a controller in reset mode: on the chip the frame
 *  lands in the acceptance-filter registers and is never sent, and the driver
 *  waits for its completion for ever. */
extern int  txInReset;
extern int  reinstalls;                        ///< Driver installs since boot.
extern int  bugUninstallWhileWaiting;          ///< Uninstall with a task blocked inside.

/** Requests to the ISO-TP nodes: key (request ID << 8) | service. */
extern std::map<uint32_t, int> isoRequests;
extern int codesCleared;                       ///< Service 04 accepted.
/** Speed of the car in km/h at time t (seconds since start), replacing the
 *  built-in drive; nullptr = the built-in drive. */
extern double (*speedOverride)(double t);

/** Seconds since the simulation began (the clock may not start at 0). */
double elapsedS();

/** The car's true values right now (what the gauges should show). */
struct Truth { double rpm, speed, throttle, coolant, gear; bool brake, lights; };
Truth truth();

/**
 * A body/transmission frame, 0x3D1 at 20 Hz, sent only while @ref enabled
 * (the scenarios that need it switch it on; every other one sees the bus it
 * always had). Byte 0 bits 4-6: the lever as P 0, R 1, N 2, D 4 (any other
 * letter 7). Byte 1: doors FL FR RL RR in bits 0-3, trunk bit 4, hood bit 5.
 * Byte 2 bit 0: the left turn lamp, the flasher's own output - lit 350 ms,
 * dark 350 ms while @ref turnLeft is on. The lever again, as scattered bits:
 * P bit 24, D bit 30, N bit 39 (held LOW only in N), R bit 42; bit 27 beside
 * them flips every 1.5 s on its own. Byte 7: a rolling counter.
 */
struct Body { bool enabled = false; char lever = 'P'; bool turnLeft = false; uint8_t doors = 0; };
extern Body body;
/**
 * The stability control unit's frame, 0x0D4 at 50 Hz, sent only while
 * @ref enabled: yaw rate (bytes 0-1, int16 LE, 0.01 deg/s), lateral G (2-3,
 * int16 LE, 0.001 g) and longitudinal G (4-5, int16 LE, 0.001 g) - the car's
 * own, consistent with its speed: longitudinal from the speed's rate of
 * change, lateral = speed x yaw. ISO 8855 signs: left and speeding up positive.
 */
struct Vdc { bool enabled = false; double yawDegS = 0; };
extern Vdc vdc;

/** The left turn lamp as the car sends it now (for counting its flashes). */
bool turnLampLit();

/** Start the bus/car task. Call from the main task before setup(). */
void start();

/** Firmware entry points from main.cpp. */
}  // namespace sim

void setup();
void loop();
