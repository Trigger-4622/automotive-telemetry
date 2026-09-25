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
