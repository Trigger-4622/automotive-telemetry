/**
 * @file sim.h
 * @brief The simulated world around the master: CAN bus, car, ECUs, radio,
 *        and what the tests can observe and inject.
 */
#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "driver/twai.h"

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
     * A marginal link: while the master is a NORMAL-mode node (it ACKs and
     * error-flags), it corrupts this fraction of the ECM's broadcast frames to
     * the TCM. Each corrupted frame is destroyed on the bus for every module -
     * the TCM included - and bumps the master's receive-error counter (REC), as
     * a real error-active controller does when it signals an error it detected
     * in another node's frame. In LISTEN-ONLY mode the controller cannot send
     * error flags, so it corrupts nothing whatever this is set to.
     */
    double   rxCorruptRate = 0;
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
};
TcmView tcm();

/** Reset reason esp_reset_reason() returns (default power-on). A scenario sets
 *  it to ESP_RST_BROWNOUT to replay a reboot during cranking. */
extern int resetReason;

/** CAN-TX recessive hold observation. gpioHoldPin/Enabled track gpio_hold_en on
 *  the TX pin (latched high across sleep); txDrivenHigh is its driven level. */
extern int  gpioHoldEnabled;   ///< Net hold count on the TX pin (>0 = latched).
extern bool txDrivenHigh;      ///< Last level driven onto the TX pin.
extern bool txConfiguredOut;   ///< TX was configured as a recessive output.

struct CtrlView { bool installed; twai_mode_t mode; twai_state_t state; uint32_t tec, rec; };
CtrlView controller();
extern int  reinstalls;                        ///< Driver installs since boot.
extern int  bugUninstallWhileWaiting;          ///< Uninstall with a task blocked inside.

/** Seconds since the simulation began (the clock may not start at 0). */
double elapsedS();

/** The car's true values right now (what the gauges should show). */
struct Truth { double rpm, speed, throttle, coolant, gear; bool brake, lights; };
Truth truth();

/** Start the bus/car task. Call from the main task before setup(). */
void start();

/** Firmware entry points from main.cpp. */
}  // namespace sim

void setup();
void loop();
