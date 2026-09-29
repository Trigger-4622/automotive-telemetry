/**
 * @file MasterConfig.cpp
 * @brief LittleFS-backed runtime configuration.
 */
#include "MasterConfig.h"
#include "MasterTelemetry.h"

#include <LittleFS.h>
#include <algorithm>

MasterConfig Cfg;

/*
 * A file, not NVS.
 *
 * This started in NVS because the configuration was small and a filesystem
 * meant a second upload step. Then the parameter tables grew past NVS's
 * ~4000-byte ceiling for a string entry, and Preferences did not merely
 * refuse the write — it crashed inside its own error-logging path, taking the
 * master into a boot loop, so the failure was not even legible as "too big".
 *
 * LittleFS has no such ceiling, and the "extra upload step" concern does not
 * apply: nothing is ever uploaded here. The filesystem is formatted on first
 * mount and this firmware writes the only file in it, so `pio run -t upload`
 * remains the whole story.
 */
static constexpr const char *CFG_PATH = "/config.json";
static constexpr const char *CFG_TMP  = "/config.tmp";

/**
 * @brief One entry of the compiled SSM2 seed table.
 *
 * At file scope because two callers need it: the first-boot seed in
 * loadDefaults(), and mergeNewDefaults() when stored settings were written by
 * an older firmware that did not know about some of these parameters.
 */
struct SsmSeed {
    uint32_t a;      /**< Address (high byte first for 2-byte values). */
    uint8_t  n;      /**< Bytes.                                       */
    bool     sg;     /**< Signed.                                      */
    float    sc;     /**< Scale.                                       */
    float    of;     /**< Offset.                                      */
    uint16_t m;      /**< Metric.                                      */
    uint16_t per;    /**< Refresh period, 0 = every exchange.          */
    uint8_t  cb;     /**< Capability byte (1-based).                   */
    uint8_t  cbit;   /**< Capability bit (1-based).                    */
    const char *nm;
};

/*
 * The standard SSM2 parameter set, as documented by RomRaider (log_defs.xml)
 * and FreeSSM (SSMFlagbyteDefinitions), cross-checked between the two. The
 * capability byte/bit is the position of this parameter's support flag in
 * the ECU's init response, using the 1-based numbering both tools use:
 * supported = flags[byte-1] & (1 << (bit-1)).
 *
 * Periods are the refresh interval each parameter is asked for. 0 means
 * "in every request": the values a needle follows. Slow-moving ones rotate
 * through whatever request space is left, so they cost the fast ones nothing.
 *
 * Two-byte values list the HIGH byte's address; the low byte follows it.
 *
 * Units are converted to what the metric registry mandates (kPa, °C, bar via
 * the derived channels) so a display reads correctly whichever protocol
 * supplied the number.
 */
static const SsmSeed SSM_SEEDS[] = {
    /* addr,    n, sgn,  scale,      offset,  metric,                     per,  cb, bit, name */
    {0x00000E, 2, false, 0.25f,      0.0f,   METRIC_ID_RPM,               0,   1, 1, "Engine speed"},
    {0x00000D, 1, false, 1.0f,       0.0f,   METRIC_ID_MAP,               0,   1, 2, "Manifold abs press"},
    {0x00000C, 1, false, 0.78125f, -100.0f,  METRIC_ID_SSM_AF_LEARN2,   250,   1, 3, "A/F learning 2"},
    {0x00000B, 1, false, 0.78125f, -100.0f,  METRIC_ID_SSM_AF_CORR2,    250,   1, 4, "A/F correction 2"},
    {0x00000A, 1, false, 0.78125f, -100.0f,  METRIC_ID_SSM_AF_LEARN,    250,   1, 5, "A/F learning 1"},
    {0x000009, 1, false, 0.78125f, -100.0f,  METRIC_ID_SSM_AF_CORR,       0,   1, 6, "A/F correction 1"},
    {0x000008, 1, false, 1.0f,     -40.0f,   METRIC_ID_COOLANT_TEMP,   1000,   1, 7, "Coolant temp"},
    {0x000007, 1, false, 0.392157f,  0.0f,   METRIC_ID_ENGINE_LOAD,       0,   1, 8, "Engine load"},
    {0x00001A, 2, false, 0.005f,     0.0f,   METRIC_ID_SSM_O2_F2,       250,   2, 1, "Front O2 #2"},
    {0x000018, 2, false, 0.005f,     0.0f,   METRIC_ID_SSM_O2_R,        250,   2, 2, "Rear O2"},
    {0x000016, 2, false, 0.005f,     0.0f,   METRIC_ID_SSM_O2_F1,       250,   2, 3, "Front O2 #1"},
    {0x000015, 1, false, 0.392157f,  0.0f,   METRIC_ID_THROTTLE,          0,   2, 4, "Throttle angle"},
    {0x000013, 2, false, 0.01f,      0.0f,   METRIC_ID_MAF,               0,   2, 5, "Mass air flow"},
    {0x000012, 1, false, 1.0f,     -40.0f,   METRIC_ID_IAT,            1000,   2, 6, "Intake air temp"},
    {0x000011, 1, false, 0.5f,     -64.0f,   METRIC_ID_TIMING_ADV,        0,   2, 7, "Ignition timing"},
    {0x000010, 1, false, 1.0f,       0.0f,   METRIC_ID_SPEED,           100,   2, 8, "Vehicle speed"},
    {0x000023, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_ATMOS,      1000,   3, 1, "Atmospheric press"},
    {0x000022, 1, false, 0.5f,     -64.0f,   METRIC_ID_SSM_KNOCK_CORR,    0,   3, 2, "Knock correction"},
    {0x000021, 1, false, 0.256f,     0.0f,   METRIC_ID_SSM_INJ_PW2,     250,   3, 3, "Inj 2 pulse"},
    {0x000020, 1, false, 0.256f,     0.0f,   METRIC_ID_SSM_INJ_PW1,       0,   3, 4, "Inj 1 pulse"},
    {0x00001E, 1, false, 0.02f,      0.0f,   METRIC_ID_SSM_TPS_VOLT,    250,   3, 6, "TPS voltage"},
    {0x00001D, 1, false, 0.02f,      0.0f,   METRIC_ID_SSM_MAF_VOLT,    250,   3, 7, "MAF voltage"},
    {0x00001C, 1, false, 0.08f,      0.0f,   METRIC_ID_BATT_VOLTAGE,    500,   3, 8, "Battery"},
    {0x00002A, 1, false, 1.0f,     -40.0f,   METRIC_ID_SSM_FUEL_TEMP,  1000,   4, 2, "Fuel temp"},
    {0x000029, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_ACCEL_ANGLE,   0,   4, 3, "Accel pedal"},
    {0x000028, 1, false, 0.5f,     -64.0f,   METRIC_ID_SSM_LEARNED_IGN, 250,   4, 4, "Learned timing"},
    {0x000026, 1, false, 0.025f,    -3.2f,   METRIC_ID_SSM_TANK_PRESS, 1000,   4, 6, "Fuel tank press"},
    {0x000024, 1, false, 1.0f,    -128.0f,   METRIC_ID_SSM_MAP_REL,       0,   4, 8, "Manifold rel"},
    {0x000032, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_CPC_DUTY,    250,   5, 2, "CPC valve duty"},
    {0x000031, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_WGDC2,       250,   5, 3, "Wastegate duty 2"},
    {0x000030, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_WGDC,          0,   5, 4, "Wastegate duty"},
    {0x00002F, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_RAD_FAN,    1000,   5, 5, "Radiator fan"},
    {0x00002E, 1, false, 0.02f,      0.0f,   METRIC_ID_SSM_FUEL_LVL_V, 2000,   5, 6, "Fuel level sender"},
    {0x00003B, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_FUELPUMP,    250,   6, 1, "Fuel pump duty"},
    {0x00003A, 1, false, 1.0f,       0.0f,   METRIC_ID_ALT_LOAD,        250,   6, 2, "Alternator duty"},
    {0x000039, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_EGR_STEP,    500,   6, 3, "EGR steps"},
    {0x000038, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_ISC_STEP,    250,   6, 4, "ISC step"},
    {0x000037, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_AF_HEATER,   500,   6, 5, "A/F heater duty"},
    {0x000036, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_AF_LEAN,     500,   6, 6, "A/F lean corr"},
    {0x000035, 1, false, 0.5f,       0.0f,   METRIC_ID_SSM_ISC_DUTY,    250,   6, 7, "ISC valve duty"},
    {0x000042, 1, false, 0.125f,   -16.0f,   METRIC_ID_SSM_AF_CURRENT,    0,   7, 2, "A/F 1 current"},
    {0x00003F, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_IOCV_L,      250,   7, 5, "Intake OCV L"},
    {0x00003E, 1, false, 0.392157f,  0.0f,   METRIC_ID_SSM_IOCV_R,      250,   7, 6, "Intake OCV R"},
    {0x00003D, 1, false, 1.0f,     -50.0f,   METRIC_ID_SSM_IVVT_L,      250,   7, 7, "Intake VVT L"},
    {0x00003C, 1, false, 1.0f,     -50.0f,   METRIC_ID_SSM_IVVT_R,      250,   7, 8, "Intake VVT R"},
    {0x000047, 1, false, 0.0078125f, 0.0f,   METRIC_ID_SSM_LAMBDA2,     250,   8, 5, "A/F 2 lambda"},
    {0x000046, 1, false, 0.0078125f, 0.0f,   METRIC_ID_SSM_LAMBDA,        0,   8, 6, "A/F 1 lambda"},
    {0x00004A, 1, false, 1.0f,       1.0f,   METRIC_ID_GEAR,            200,   9, 6, "Gear position"},
    {0x0000FA, 1, false, 0.78125f, -100.0f,  METRIC_ID_SSM_TMOTOR_DUTY, 250,  31, 6, "Throttle motor"},
    {0x00016A, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_SI_DRIVE,   1000,  31, 8, "SI-Drive mode"},
    {0x000106, 1, false, 5.0f,     200.0f,   METRIC_ID_EGT,             500,  33, 2, "Exhaust gas temp"},
    {0x000105, 1, false, 2.0f,       0.0f,   METRIC_ID_FUEL_PRESSURE,   250,  33, 3, "Fuel pressure"},
    {0x000104, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_BRAKE_BOOST, 250,  33, 4, "Brake booster"},
    {0x000103, 1, false, 0.02f,      0.0f,   METRIC_ID_SSM_APS_MAIN,    250,  33, 5, "Main APS"},
    {0x000101, 1, false, 0.02f,      0.0f,   METRIC_ID_SSM_TPS_MAIN,    250,  33, 7, "Main TPS"},
    {0x00010E, 2, false, 2.0f,       0.0f,   METRIC_ID_SSM_ODOMETER,   5000,  34, 2, "Odometer"},
    {0x000113, 1, false, 1.0f,     -40.0f,   METRIC_ID_OIL_TEMP,       1000,  35, 5, "Oil temp"},
    {0x000119, 1, false, 1.0f,     -50.0f,   METRIC_ID_SSM_EVVT_L,      250,  36, 7, "Exhaust VVT L"},
    {0x000118, 1, false, 1.0f,     -50.0f,   METRIC_ID_SSM_EVVT_R,      250,  36, 8, "Exhaust VVT R"},
    {0x000199, 1, false, 0.25f,    -32.0f,   METRIC_ID_SSM_FINE_KNOCK,  250,  48, 1, "Knock corr fine"},
    {0x0000F9, 1, false, 0.0625f,    0.0f,   METRIC_ID_SSM_LEARNED_COR, 500,  48, 2, "Learned ign corr"},
    {0x0000D9, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_ROUGH_C4,    250,  48, 5, "Roughness cyl 4"},
    {0x0000D8, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_ROUGH_C3,    250,  48, 6, "Roughness cyl 3"},
    {0x0000CF, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_ROUGH_C2,    250,  48, 7, "Roughness cyl 2"},
    {0x0000CE, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_ROUGH_C1,    250,  48, 8, "Roughness cyl 1"},
    {0x0001F5, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_EPS_CURRENT, 500,  54, 2, "EPS current"},
    {0x0001F0, 1, false, 1.0f,    -128.0f,   METRIC_ID_SSM_BOOST_FB,      0,  54, 3, "Boost feedback"},
    {0x0001EE, 2, false, 0.25f,      0.0f,   METRIC_ID_SSM_TARGET_RPM,  250,  54, 4, "Target RPM"},
    {0x0001F8, 2, false, 0.001f,     0.0f,   METRIC_ID_SSM_FUELPUMP_A,  500,  56, 8, "Fuel pump current"},
    {0x000272, 1, false, 1.0f,       0.0f,   METRIC_ID_SSM_ALT_MODE,   1000,  57, 6, "Alternator mode"},
    {0x000273, 1, false, 1.0f,     -40.0f,   METRIC_ID_BATT_TEMP,      2000,  57, 7, "Battery temp"},
    {0x000271, 1, false, 1.0f,    -128.0f,   METRIC_ID_BATT_CURRENT,    500,  57, 8, "Battery current"},
};
static constexpr size_t SSM_SEED_COUNT = sizeof(SSM_SEEDS) / sizeof(SSM_SEEDS[0]);

static RtSsm seedToRt(const SsmSeed &d) {
    RtSsm e;
    e.address  = d.a;
    e.bytes    = d.n;
    e.isSigned = d.sg;
    e.scale    = d.sc;
    e.offset   = d.of;
    e.metricId = d.m;
    e.enabled  = true;
    e.periodMs = d.per;
    e.capByte  = d.cb;
    e.capBit   = d.cbit;
    strlcpy(e.name, d.nm, sizeof(e.name));
    return e;
}

static RtSignal sigToRt(const CanSignalDef &s) {
    RtSignal r;
    r.canId     = s.can_id;
    r.extended  = s.extended;
    r.startBit  = s.start_bit;
    r.bitLength = s.bit_length;
    r.bigEndian = (s.order == ByteOrder::BigEndian);
    r.isSigned  = s.is_signed;
    r.scale     = s.scale;
    r.offset    = s.offset;
    r.metricId  = s.metric_id;
    r.mode      = s.mode;
    r.refMetric = s.ref_metric;
    strlcpy(r.name, s.name ? s.name : "", sizeof(r.name));
    return r;
}

void MasterConfig::reseedTables() {
    lock();
    ssm.clear();
    for (auto &d : SSM_SEEDS) ssm.push_back(seedToRt(d));
    signals.clear();
    for (size_t i = 0; i < RAW_SIGNAL_COUNT; i++) signals.push_back(sigToRt(RAW_SIGNAL_TABLE[i]));
    generation++;
    unlock();
}

void MasterConfig::loadBusDefaults() {
    lock();
    diagMode        = DIAG_MODE_DEFAULT;
    txPassive       = true;
    canTiming       = 2;
    canSample875    = false;
    guardEnabled    = true; guardErrs = 3; guardWindowS = 10; guardPauseS = 30; guardTrips = 3;
    rxGuardEnabled  = true; rxGuardRec = 96; rxGuardErrs = 60;
    obdP2CanMs      = 50;
    startDelayS     = BUS_SETTLE_S;
    txRecessiveHold = true;
    unlock();
}

void MasterConfig::loadDefaults() {
    lock();
    loadBusDefaults();
    broadcastMs = BROADCAST_PERIOD_MS;
    metricTtlMs = METRIC_TTL_MS;
    nightSource = NIGHT_SOURCE;
    bitrateKbps = CAN_BITRATE_KBPS;
    wifiChannel = TELEMETRY_WIFI_CHANNEL;
    strlcpy(apSsid, AP_SSID_DEFAULT, sizeof(apSsid));
    apPass[0] = '\0';
    portalOn  = true;
    ssmGapMs    = 100;
    ssmBatchMax = 33;
    radioDbm     = 13;
    ssmSwitches = true;
    learnEnabled = true;
    learnR2 = 0.985f; learnMinN = 40; learnSteady = 0.10f; learnPhi = 0.95f; verifyN = 30;
    obdGapMs = 8; obdTimeoutMs = 80; reqMaxHz = 0; obdAddressing = 0;
    ssmTimeoutMs = 1000; coverMs = 2500;
    keepaliveMs = 300; sourceHoldMs = 500; ldrDark = NIGHT_LDR_DARK_ADC; ldrLight = NIGHT_LDR_LIGHT_ADC;
    sleepEnabled = true;
    sleepIdleS   = 90;
    ecuId[0] = ecuSysId[0] = ecuFlags[0] = '\0';
    ssmRemoved.clear();

    unlock();
    reseedPids();
    reseedTables();
}

void MasterConfig::reseedPids() {
    // One entry per PID: the table lists a PID once per value it returns,
    // and the first of those rows carries its period and default.
    lock();
    pids.clear();
    for (auto &d : OBD_POLL_TABLE) {
        if (pidFor(d.pid)) continue;
        RtPid p;
        p.pid      = d.pid;
        p.enabled  = d.default_on;
        p.periodMs = d.period_ms;
        pids.push_back(p);
    }
    generation++;
    unlock();
}

void MasterConfig::begin() {
    loadDefaults();

    if (!LittleFS.begin(true)) {                // true: format if unformatted
        log_e("config: LittleFS mount failed, running on compiled defaults");
        return;
    }

    File f = LittleFS.open(CFG_PATH, "r");
    // A power cut between writing the new file and renaming it leaves only
    // the temporary one - which is complete, so it is used.
    if (!f && LittleFS.exists(CFG_TMP)) {
        log_w("config: using %s left by an interrupted save", CFG_TMP);
        f = LittleFS.open(CFG_TMP, "r");
    }
    if (!f) {
        log_i("config: nothing stored yet, using compiled defaults");
        return;
    }

    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) {
        // Corrupt blob: keep the defaults rather than refusing to boot. A
        // master that starts with factory settings is far better than one
        // that does not start.
        log_e("config: stored JSON is corrupt, using compiled defaults");
        return;
    }
    fromJson(doc.as<JsonVariantConst>());

    // A file recovered from an interrupted save is written back in place.
    bool dirty = !LittleFS.exists(CFG_PATH);
    if (_storedVersion < 2) {
        /*
         * Version 2 replaced the SSM table (new capability flags, periods and
         * a few corrected metric mappings), gave raw signals a verification
         * mode, and made the diagnostic mode automatic. Anything the user
         * could not have edited yet is re-seeded; PID choices and network
         * settings are kept.
         *
         * Only for files older than version 2. This used to run for ANY older
         * version, so every later version bump wiped the signal table -
         * learned signals included - and the SSM edits. Each version after 2
         * has its own migration below that touches only what it changed.
         */
        log_i("config: stored version %u < 2, re-seeding tables and "
              "mode defaults", _storedVersion);
        reseedTables();
        diagMode    = DIAG_MODE_DEFAULT;
        nightSource = NIGHT_SOURCE;
        broadcastMs = BROADCAST_PERIOD_MS;
        dirty = true;
    }
    if (_storedVersion < 5) {
        /*
         * Version 5: the SSM2 engine was rewritten after FreeSSM. It paces
         * itself (at least 100 ms between requests) and never asks for more
         * than 33 addresses at once; old settings allowed back-to-back
         * requests of up to 80, which is what upset the bus.
         */
        if (ssmGapMs < 100) ssmGapMs = 100;
        if (ssmBatchMax > 33) ssmBatchMax = 33;
        dirty = true;
    }
    if (_storedVersion < 4) {
        /*
         * Version 4: listen first, then OBD-II, then SSM2, with the learner
         * moving values onto the listening side. AUTO now means exactly that,
         * so the mode is reset to it. The frames seeded from other Subaru
         * generations never appeared on this car and are removed; anything
         * mapped by hand or verified is kept.
         */
        log_i("config: stored version %u < 4, switching to listen-first AUTO",
              _storedVersion);
        static const uint32_t OLD_SEEDS[] = { 0x0D1, 0x140, 0x141, 0x148, 0x360,
                                              0x002, 0x0D4, 0x152 };
        lock();
        diagMode = DIAG_MODE_AUTO;
        for (size_t i = 0; i < signals.size();) {
            const RtSignal &sg = signals[i];
            bool seed = false;
            for (uint32_t id : OLD_SEEDS) if (sg.canId == id) seed = true;
            if (seed && !sg.learned && sg.mode != SIG_ON && sg.mode != SIG_VERIFIED)
                signals.erase(signals.begin() + i);
            else i++;
        }
        generation++;
        unlock();
        dirty = true;
    }
    if (_storedVersion < 3) {
        /*
         * Version 3 replaced the OBD-II table with the full J1979 set and
         * turned most of it on (support and coverage checks make that free).
         * The old enable flags were the old defaults, not choices, so the PID
         * list is re-seeded. Mapped signals and SSM edits are left alone.
         */
        log_i("config: stored version %u < 3, re-seeding the OBD-II PID list",
              _storedVersion);
        reseedPids();
        dirty = true;
    }
    if (mergeNewDefaults()) dirty = true;
    if (dirty) save();

    log_i("config: loaded (%u signals, %u PIDs, %u SSM, mode %u)",
          (unsigned)signals.size(), (unsigned)pids.size(),
          (unsigned)ssm.size(), diagMode);
}

/**
 * @brief Add table entries this firmware knows about that the stored settings
 *        have never heard of.
 *
 * Compiled tables grow between firmware versions. Without this, any device
 * that has ever saved its configuration would keep the older, shorter tables
 * for good and none of the newly added parameters would appear — which looks
 * exactly like the update not having been flashed at all.
 *
 * Entries are matched by identity (PID number, SSM address) and only ever
 * ADDED, never rewritten, so enable flags, periods and scaling the user has
 * corrected are all preserved. The one exception is metadata the user cannot
 * edit: an SSM entry without a capability flag gets the seed's.
 *
 * @return true if anything was added, meaning the config is worth re-saving.
 */
bool MasterConfig::mergeNewDefaults() {
    size_t added = 0;
    lock();

    for (auto &d : OBD_POLL_TABLE) {
        if (pidFor(d.pid)) continue;
        RtPid p;
        p.pid      = d.pid;
        p.enabled  = d.default_on;
        p.periodMs = d.period_ms;
        pids.push_back(p);
        added++;
    }

    for (auto &d : SSM_SEEDS) {
        // Deleted by the user: stays deleted.
        if (std::find(ssmRemoved.begin(), ssmRemoved.end(), d.a) != ssmRemoved.end()) continue;
        bool found = false;
        for (auto &e : ssm) {
            if (e.address != d.a) continue;
            found = true;
            if (e.capByte == 0 && d.cb) { e.capByte = d.cb; e.capBit = d.cbit; added++; }
            break;
        }
        if (found) continue;
        ssm.push_back(seedToRt(d));
        added++;
    }

    for (size_t si = 0; si < RAW_SIGNAL_COUNT; si++) {
        const CanSignalDef &s = RAW_SIGNAL_TABLE[si];
        bool found = false;
        for (auto &r : signals)
            if (r.canId == s.can_id && r.startBit == s.start_bit &&
                r.bitLength == s.bit_length) { found = true; break; }
        if (found) continue;
        signals.push_back(sigToRt(s));
        added++;
    }

    if (added) generation++;
    unlock();
    if (added) log_i("config: merged %u entries added by this firmware",
                     (unsigned)added);
    return added > 0;
}

void MasterConfig::serviceSave() {
    if (!_saveWanted) return;
    _saveWanted = false;
    save();
}

bool MasterConfig::save() {
    xSemaphoreTake(_saveMutex, portMAX_DELAY);
    const bool ok = saveLocked();
    xSemaphoreGive(_saveMutex);
    return ok;
}

bool MasterConfig::saveLocked() {
    JsonDocument doc;
    toJson(doc);

    /*
     * Written to a temporary file and then renamed, so a power cut midway
     * leaves the previous configuration intact rather than a truncated one.
     * A half-written table would be parsed as a valid but wrong config, which
     * is a far nastier failure than simply keeping the old settings.
     */
    masterBusStall();                // the flash write holds the CAN interrupt off
    File f = LittleFS.open(CFG_TMP, "w");
    if (!f) {
        log_e("config: cannot open %s for writing", CFG_TMP);
        return false;
    }
    const size_t expected = measureJson(doc);
    const size_t written  = serializeJson(doc, f);
    f.close();
    if (!written || written != expected) {
        // A short write (flash full) must never replace a good file: the
        // next boot would find it corrupt and fall back to factory settings.
        log_e("config: wrote %u of %u bytes - keeping the previous settings",
              (unsigned)written, (unsigned)expected);
        LittleFS.remove(CFG_TMP);
        return false;
    }

    // LittleFS renames over an existing file atomically; removing the old
    // one first would open a window with no configuration at all. The
    // remove is only the fallback for a layer that refuses to overwrite.
    if (!LittleFS.rename(CFG_TMP, CFG_PATH)) {
        LittleFS.remove(CFG_PATH);
        if (!LittleFS.rename(CFG_TMP, CFG_PATH)) {
            log_e("config: rename failed, settings not persisted");
            return false;
        }
    }
    log_i("config: saved %u bytes", (unsigned)written);
    return true;
}

void MasterConfig::toJson(JsonDocument &doc) const {
    const_cast<MasterConfig *>(this)->lock();
    doc["cfg_ver"]      = CFG_VERSION;
    doc["diag_mode"]    = diagMode;
    doc["broadcast_ms"] = broadcastMs;
    doc["metric_ttl_ms"] = metricTtlMs;
    doc["night_source"] = nightSource;
    doc["bitrate_kbps"] = bitrateKbps;
    doc["wifi_channel"] = wifiChannel;
    doc["ap_ssid"]      = apSsid;
    doc["ap_pass"]      = apPass;
    doc["portal_on"]    = portalOn;
    doc["ssm_gap"]      = ssmGapMs;
    doc["ssm_batch"]    = ssmBatchMax;
    doc["ssm_switches"] = ssmSwitches;
    doc["learn"]        = learnEnabled;
    doc["can_sp875"]    = canSample875;
    doc["can_timing"]   = canTiming;
    doc["tx_passive"]   = txPassive;
    doc["radio_dbm"]    = radioDbm;
    doc["guard"]        = guardEnabled;
    doc["guard_errs"]   = guardErrs;
    doc["guard_win"]    = guardWindowS;
    doc["guard_pause"]  = guardPauseS;
    doc["guard_trips"]  = guardTrips;
    doc["rx_guard"]     = rxGuardEnabled;
    doc["rx_guard_rec"] = rxGuardRec;
    doc["rx_guard_errs"] = rxGuardErrs;
    doc["tx_hold"]      = txRecessiveHold;
    doc["start_delay_s"] = startDelayS;
    doc["learn_r2"]     = learnR2;
    doc["learn_min_n"]  = learnMinN;
    doc["learn_steady"] = learnSteady;
    doc["learn_phi"]    = learnPhi;
    doc["verify_n"]     = verifyN;
    doc["obd_gap"]      = obdGapMs;
    doc["obd_to"]       = obdTimeoutMs;
    doc["obd_p2can"]    = obdP2CanMs;
    doc["req_max_hz"]   = reqMaxHz;
    doc["obd_addr"]     = obdAddressing;
    doc["ssm_to"]       = ssmTimeoutMs;
    doc["cover_ms"]     = coverMs;
    doc["keepalive_ms"] = keepaliveMs;
    doc["hold_ms"]      = sourceHoldMs;
    doc["ldr_dark"]     = ldrDark;
    doc["ldr_light"]    = ldrLight;
    doc["sleep_enabled"] = sleepEnabled;
    doc["sleep_idle_s"]  = sleepIdleS;
    doc["ecu_id"]       = ecuId;
    doc["ecu_sys_id"]   = ecuSysId;
    doc["ecu_flags"]    = ecuFlags;
    JsonArray rm = doc["ssm_removed"].to<JsonArray>();
    for (uint32_t a : ssmRemoved) rm.add(a);

    JsonArray ma = doc["ssm"].to<JsonArray>();
    for (const auto &e : ssm) {
        JsonObject o = ma.add<JsonObject>();
        o["addr"]    = e.address;
        o["bytes"]   = e.bytes;
        o["signed"]  = e.isSigned;
        o["scale"]   = e.scale;
        o["offset"]  = e.offset;
        o["metric"]  = e.metricId;
        o["enabled"] = e.enabled;
        o["period"]  = e.periodMs;
        o["cap_byte"] = e.capByte;
        o["cap_bit"]  = e.capBit;
        o["name"]    = e.name;
    }

    JsonArray pa = doc["pids"].to<JsonArray>();
    for (const auto &p : pids) {
        JsonObject o = pa.add<JsonObject>();
        o["pid"]     = p.pid;
        o["enabled"] = p.enabled;
        o["period"]  = p.periodMs;
        // Read-only, for the portal: what this PID is and which channels it
        // feeds. Ignored on the way back in.
        JsonArray ms = o["metrics"].to<JsonArray>();
        for (auto &d : OBD_POLL_TABLE) {
            if (d.pid != p.pid) continue;
            if (!o["name"].is<const char *>()) o["name"] = d.name;
            ms.add(d.metric_id);
        }
    }

    JsonArray sa = doc["signals"].to<JsonArray>();
    for (const auto &s : signals) {
        JsonObject o = sa.add<JsonObject>();
        o["can_id"] = s.canId;
        o["ext"]    = s.extended;
        o["start"]  = s.startBit;
        o["len"]    = s.bitLength;
        o["be"]     = s.bigEndian;
        o["signed"] = s.isSigned;
        o["scale"]  = s.scale;
        o["offset"] = s.offset;
        o["metric"] = s.metricId;
        o["mode"]   = s.mode;
        o["ref"]    = s.refMetric;
        o["learned"] = s.learned;
        if (s.vtol > 0)    o["vtol"] = s.vtol;
        if (s.vspread > 0) o["vsp"]  = s.vspread;
        o["name"]   = s.name;
        if (s.nBits) {                   // only bit combinations carry the key
            JsonArray bl = o["bits"].to<JsonArray>();
            for (uint8_t k = 0; k < s.nBits; k++) bl.add(s.bitList[k]);
        }
        if (s.nMap) {                    // only table signals carry the key
            JsonArray m = o["map"].to<JsonArray>();
            for (uint8_t k = 0; k < s.nMap; k++) {
                JsonArray pr = m.add<JsonArray>();
                pr.add(s.mapRaw[k]);
                pr.add(s.mapVal[k]);
            }
        }
    }
    const_cast<MasterConfig *>(this)->unlock();
}

/** @brief A whole number from JSON, clamped into [lo, hi]. Read wide and
 *         clamped, so an out-of-range entry cannot wrap round into another
 *         value (a 70000 ms period would otherwise become 4464 ms). */
static int jint(JsonVariantConst v, int def, int lo, int hi) {
    const long x = v | (long)def;
    return (int)(x < lo ? lo : x > hi ? hi : x);
}

/** @brief A table key (metric, PID) from JSON: out of range means no key at
 *         all (0, and the row is dropped) - PID 300 must not become PID 44. */
static int jkey(JsonVariantConst v, int hi) {
    const long x = v | 0L;
    return x < 0 || x > hi ? 0 : (int)x;
}

bool MasterConfig::fromJson(JsonVariantConst v, bool fromUser) {
    if (!v.is<JsonObjectConst>()) return false;
    lock();

    // Only the stored file carries a version; a partial update from the
    // portal must not make the next boot think the file is ancient.
    if (v["cfg_ver"].is<int>()) _storedVersion = v["cfg_ver"].as<uint8_t>();
    diagMode    = v["diag_mode"]     | diagMode;
    broadcastMs = v["broadcast_ms"]  | broadcastMs;
    metricTtlMs = v["metric_ttl_ms"] | metricTtlMs;
    nightSource = v["night_source"]  | nightSource;
    bitrateKbps = v["bitrate_kbps"]  | bitrateKbps;
    wifiChannel = v["wifi_channel"]  | wifiChannel;
    portalOn    = v["portal_on"]     | portalOn;
    ssmGapMs    = v["ssm_gap"]       | ssmGapMs;
    ssmBatchMax = v["ssm_batch"]     | ssmBatchMax;
    ssmSwitches = v["ssm_switches"]  | ssmSwitches;
    learnEnabled = v["learn"]        | learnEnabled;
    canSample875 = v["can_sp875"]    | canSample875;
    canTiming    = (uint8_t)jint(v["can_timing"], canTiming, 0, 2);
    txPassive    = v["tx_passive"]    | txPassive;
    radioDbm     = (uint8_t)jint(v["radio_dbm"], radioDbm, 2, 20);
    guardEnabled = v["guard"]        | guardEnabled;
    guardErrs    = v["guard_errs"]   | guardErrs;
    guardWindowS = v["guard_win"]    | guardWindowS;
    guardPauseS  = v["guard_pause"]  | guardPauseS;
    guardTrips   = v["guard_trips"]  | guardTrips;
    rxGuardEnabled = v["rx_guard"]   | rxGuardEnabled;
    rxGuardRec   = (uint8_t)jint(v["rx_guard_rec"], rxGuardRec, 16, 255);
    rxGuardErrs  = (uint8_t)jint(v["rx_guard_errs"], rxGuardErrs, 1, 250);
    txRecessiveHold = v["tx_hold"]   | txRecessiveHold;
    startDelayS  = jint(v["start_delay_s"], startDelayS, 0, 300);
    learnR2      = v["learn_r2"]     | learnR2;
    learnMinN    = v["learn_min_n"]  | learnMinN;
    learnSteady  = v["learn_steady"] | learnSteady;
    learnPhi     = v["learn_phi"]    | learnPhi;
    verifyN      = v["verify_n"]     | verifyN;
    obdGapMs     = v["obd_gap"]      | obdGapMs;
    obdTimeoutMs = v["obd_to"]       | obdTimeoutMs;
    obdP2CanMs   = (uint16_t)jint(v["obd_p2can"], obdP2CanMs, 0, 2000);
    reqMaxHz     = (uint16_t)jint(v["req_max_hz"], reqMaxHz, 0, 200);
    obdAddressing = v["obd_addr"]    | obdAddressing;
    ssmTimeoutMs = v["ssm_to"]       | ssmTimeoutMs;
    coverMs      = v["cover_ms"]     | coverMs;
    keepaliveMs  = v["keepalive_ms"] | keepaliveMs;
    sourceHoldMs = v["hold_ms"]      | sourceHoldMs;
    ldrDark      = v["ldr_dark"]     | ldrDark;
    ldrLight     = v["ldr_light"]    | ldrLight;
    // Keep every tunable inside a range that cannot wedge the master.
    guardErrs    = constrain(guardErrs, 1, 100);
    guardWindowS = constrain(guardWindowS, 1, 120);
    guardPauseS  = constrain(guardPauseS, 1, 3600);
    guardTrips   = constrain(guardTrips, 1, 50);
    learnR2      = constrain(learnR2, 0.5f, 0.99999f);
    learnMinN    = constrain(learnMinN, 10, 5000);
    learnSteady  = constrain(learnSteady, 0.01f, 10.0f);
    learnPhi     = constrain(learnPhi, 0.5f, 1.0f);
    verifyN      = constrain(verifyN, 5, 250);
    obdTimeoutMs = constrain(obdTimeoutMs, 20, 2000);
    if (obdAddressing > 2) obdAddressing = 0;
    ssmTimeoutMs = constrain(ssmTimeoutMs, 100, 5000);
    coverMs      = constrain(coverMs, 100, 30000);
    keepaliveMs  = constrain(keepaliveMs, 50, 1400);
    sourceHoldMs = constrain(sourceHoldMs, 0, 5000);
    sleepEnabled = v["sleep_enabled"] | sleepEnabled;
    sleepIdleS   = v["sleep_idle_s"]  | sleepIdleS;

    if (diagMode > DIAG_MODE_SILENT) diagMode = DIAG_MODE_AUTO;
    // Slower than ~1 s and the displays' 1.5 s stale timeout starts to bite.
    broadcastMs  = constrain(broadcastMs, 20, 1000);
    if (metricTtlMs < 200) metricTtlMs = 200;
    // At least 2: a two-byte value is two addresses. FreeSSM's ceiling is 33.
    ssmBatchMax  = constrain(ssmBatchMax, 2, 33);
    if (bitrateKbps != 125 && bitrateKbps != 250 && bitrateKbps != 500 && bitrateKbps != 1000)
        bitrateKbps = CAN_BITRATE_KBPS;
    if (sleepIdleS < 10) sleepIdleS = 10;
    if (wifiChannel < 1 || wifiChannel > 13) wifiChannel = TELEMETRY_WIFI_CHANNEL;

    // A name or password the AP cannot take would lock the portal - the only
    // way into these settings - away: refused, and the old one kept. (Cut
    // short to fit, a password would no longer be the one typed.)
    if (v["ap_ssid"].is<const char *>()) {
        const char *s = v["ap_ssid"].as<const char *>();
        if (*s && strlen(s) < sizeof(apSsid)) strlcpy(apSsid, s, sizeof(apSsid));
        else log_w("config: Wi-Fi name refused - it takes 1-%u characters",
                   (unsigned)(sizeof(apSsid) - 1));
    }
    if (v["ap_pass"].is<const char *>()) {
        const char *s = v["ap_pass"].as<const char *>();
        if (strlen(s) < sizeof(apPass)) strlcpy(apPass, s, sizeof(apPass));
        else log_w("config: Wi-Fi password refused - it takes up to %u characters",
                   (unsigned)(sizeof(apPass) - 1));
    }
    if (v["ecu_id"].is<const char *>())
        strlcpy(ecuId, v["ecu_id"].as<const char *>(), sizeof(ecuId));
    if (v["ecu_sys_id"].is<const char *>())
        strlcpy(ecuSysId, v["ecu_sys_id"].as<const char *>(), sizeof(ecuSysId));
    if (v["ecu_flags"].is<const char *>())
        strlcpy(ecuFlags, v["ecu_flags"].as<const char *>(), sizeof(ecuFlags));

    if (v["ssm_removed"].is<JsonArrayConst>()) {
        ssmRemoved.clear();
        for (JsonVariantConst a : v["ssm_removed"].as<JsonArrayConst>())
            if (a.is<uint32_t>()) ssmRemoved.push_back(a.as<uint32_t>());
    }

    // A present-but-empty array is a deliberate "no entries", so the tables
    // are only replaced when the key exists at all. Out-of-range fields are
    // clamped, not grounds to drop the row: a typo in the portal must not
    // make a parameter vanish on save.
    if (v["ssm"].is<JsonArrayConst>()) {
        ssm.clear();
        for (JsonVariantConst e : v["ssm"].as<JsonArrayConst>()) {
            RtSsm t;
            t.address  = (e["addr"] | 0u) & 0xFFFFFFu;
            t.bytes    = (e["bytes"] | 1) == 2 ? 2 : 1;
            t.isSigned = e["signed"]  | false;
            t.scale    = e["scale"]   | 1.0f;
            t.offset   = e["offset"]  | 0.0f;
            t.metricId = (uint16_t)jkey(e["metric"], 0xFFFF);
            t.enabled  = e["enabled"] | true;
            t.periodMs = (uint16_t)jint(e["period"], 0, 0, 60000);
            t.capByte  = (uint8_t)jint(e["cap_byte"], 0, 0, 160);
            t.capBit   = (uint8_t)jint(e["cap_bit"], 0, 0, 8);
            strlcpy(t.name, e["name"] | "", sizeof(t.name));
            if (t.metricId) ssm.push_back(t);
        }
        /*
         * A seeded parameter the user deleted is remembered, or the next boot
         * would put it straight back (mergeNewDefaults adds every seed the
         * table lacks). Only for edits from the portal: a stored file that
         * lacks a seed simply predates it.
         */
        if (fromUser)
            for (const auto &d : SSM_SEEDS) {
                bool present = false;
                for (const auto &e : ssm) if (e.address == d.a) { present = true; break; }
                auto it = std::find(ssmRemoved.begin(), ssmRemoved.end(), d.a);
                if (present && it != ssmRemoved.end()) ssmRemoved.erase(it);
                else if (!present && it == ssmRemoved.end()) ssmRemoved.push_back(d.a);
            }
    }
    if (v["pids"].is<JsonArrayConst>()) {
        pids.clear();
        for (JsonVariantConst e : v["pids"].as<JsonArrayConst>()) {
            RtPid p;
            p.pid      = (uint8_t)jkey(e["pid"], 0xFF);
            p.enabled  = e["enabled"] | true;
            p.periodMs = (uint16_t)jint(e["period"], 100, 10, 60000);
            // Once each: a duplicate would be polled twice as often.
            if (p.pid && !pidFor(p.pid)) pids.push_back(p);
        }
    }
    if (v["signals"].is<JsonArrayConst>()) {
        signals.clear();
        for (JsonVariantConst e : v["signals"].as<JsonArrayConst>()) {
            RtSignal s;
            s.canId     = e["can_id"] | 0u;
            s.extended  = e["ext"]    | false;
            s.bigEndian = e["be"]     | false;
            s.isSigned  = e["signed"] | false;
            s.scale     = e["scale"]  | 1.0f;
            s.offset    = e["offset"] | 0.0f;
            s.metricId  = (uint16_t)jkey(e["metric"], 0xFFFF);
            s.refMetric = (uint16_t)jkey(e["ref"], 0xFFFF);
            s.learned   = e["learned"] | false;
            s.vtol      = e["vtol"]    | 0.0f;
            s.vspread   = e["vsp"]     | 0.0f;
            // Older files carried a bare enable flag.
            if (e["mode"].is<int>())   s.mode = (uint8_t)jint(e["mode"], SIG_OFF, SIG_OFF, SIG_REJECTED);
            else                       s.mode = (e["enabled"] | false) ? SIG_ON : SIG_OFF;
            strlcpy(s.name, e["name"] | "", sizeof(s.name));
            // Keep the field inside the 8-byte payload, so the hot decode
            // loop needs no checks. (Read as int first: a value past 255
            // must clamp, not wrap.)
            s.startBit  = (uint8_t)constrain(e["start"] | 0, 0, 63);
            s.bitLength = (uint8_t)constrain(e["len"] | 8, 1, 32);
            if (!s.bigEndian && s.startBit + s.bitLength > 64)
                s.bitLength = (uint8_t)(64 - s.startBit);
            // A bit combination: payload bit numbers, bit i of the value first.
            // Its table's codes depend on the exact list, so a list that cannot
            // be decoded as saved - a bad or repeated bit, too many - drops the
            // whole signal rather than decoding the wrong bits.
            s.nBits = 0;
            if (e["bits"].is<JsonArrayConst>()) {
                bool bad = e["bits"].size() == 0 || e["bits"].size() > MAX_SIG_BITS;
                for (JsonVariantConst bv : e["bits"].as<JsonArrayConst>()) {
                    if (bad) break;
                    const long b = bv.is<long>() ? bv.as<long>() : -1;
                    bad = b < 0 || b > 63;
                    for (uint8_t k = 0; k < s.nBits && !bad; k++) bad = s.bitList[k] == b;
                    if (!bad) s.bitList[s.nBits++] = (uint8_t)b;
                }
                if (bad) continue;
            }
            // A value table: pairs of [raw, value], for fields of 16 bits at
            // most. Malformed or duplicate pairs are dropped, not guessed at.
            s.nMap = 0;
            const uint8_t rawBits = s.nBits ? s.nBits : s.bitLength;
            if (rawBits <= 16 && e["map"].is<JsonArrayConst>())
                for (JsonVariantConst pv : e["map"].as<JsonArrayConst>()) {
                    if (s.nMap >= MAX_SIG_MAP) break;
                    if (!pv.is<JsonArrayConst>()) continue;
                    JsonArrayConst pr = pv.as<JsonArrayConst>();
                    if (pr.size() < 2 || !pr[0].is<long>() || !pr[1].is<long>()) continue;
                    const long raw = pr[0].as<long>(), val = pr[1].as<long>();
                    if (raw < 0 || raw >= (1L << rawBits) || val < -32768 || val > 32767) continue;
                    bool dup = false;
                    for (uint8_t k = 0; k < s.nMap; k++) dup |= s.mapRaw[k] == raw;
                    if (dup) continue;
                    s.mapRaw[s.nMap] = (uint16_t)raw;
                    s.mapVal[s.nMap] = (int16_t)val;
                    s.nMap++;
                }
            // No destination: nothing it could do.
            if (s.metricId && signals.size() < MAX_RT_SIGNALS)
                signals.push_back(s);
        }
    }
    generation++;
    unlock();
    return true;
}

const RtPid *MasterConfig::pidFor(uint8_t pid) const {
    for (const auto &p : pids)
        if (p.pid == pid) return &p;
    return nullptr;
}
