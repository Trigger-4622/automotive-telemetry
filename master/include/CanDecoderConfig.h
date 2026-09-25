/**
 * @file CanDecoderConfig.h
 * @brief THE vehicle-adaptation file of the Master Node. Everything that is
 *        car-specific lives here: GPIO wiring, bus speed, which OBD-II PIDs
 *        to poll, which raw CAN frames to sniff-decode (DBC-style), alarm
 *        thresholds, and the night-mode source.
 *
 * The main.cpp engine is generic — porting the Master to a new vehicle should
 * never require touching anything but this header (and, for a Subaru, the
 * SSM2 seed table in MasterConfig.cpp).
 *
 * Everything in here is a COMPILED DEFAULT. The running values live in
 * MasterConfig and are editable from the portal; these tables are what a
 * fresh device starts from and what "restore defaults" returns to.
 */
#pragma once

#include <stdint.h>
#include <math.h>
#include "MasterPacket.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  HARDWARE WIRING (ESP32-S3 DevKit)
 * ───────────────────────────────────────────────────────────────────────────
 *  ESP32 GPIO 5  -> transceiver CTX/TXD      transceiver CANH -> OBD pin 6
 *  ESP32 GPIO 4  <- transceiver CRX/RXD      transceiver CANL -> OBD pin 14
 *  ESP32 3V3     -> transceiver VCC          OBD pin 16 (+12V) -> buck -> VIN
 *  ESP32 GND     -> transceiver GND          OBD pin 4/5 (GND) -> buck GND
 *
 *  The transceiver MUST be 3.3 V logic (SN65HVD230, TJA1051T/3). A 5 V part
 *  such as TJA1050 or MCP2551 drives its RX output above the ESP32's limit.
 *  On an SN65HVD230 tie RS to GND for high-speed mode.
 *
 *  Leave the module's 120 ohm termination OFF. The vehicle bus is already
 *  terminated at both ends and this node is a stub; a third terminator is the
 *  most common cause of a bus that half-works.
 *
 *  GPIO 4 doubles as the deep-sleep wake source (RTC-capable on the S3), so
 *  moving CAN_RX_GPIO means picking another RTC pin (GPIO 0-21).
 * ═══════════════════════════════════════════════════════════════════════════ */
#define CAN_TX_GPIO         5     /**< ESP32 TWAI TX  → transceiver CTX/TXD  */
#define CAN_RX_GPIO         4     /**< ESP32 TWAI RX  ← transceiver CRX/RXD  */
#define NIGHT_LDR_GPIO      6     /**< Optional LDR divider for night
                                       detection — see NIGHT_SOURCE below.
                                       Must be an ADC1 pin: on the ESP32-S3
                                       that is GPIO 1-10.                    */

/* ═══════════════════════════════════════════════════════════════════════════
 *  BUS PARAMETERS
 * ═══════════════════════════════════════════════════════════════════════════ */
/** CAN bitrate. Most OBD ports: 500 kbit/s. Body/comfort buses often 125 k. */
#define CAN_BITRATE_KBPS    500

/**
 * How the master talks to the ECU. See MasterConfig::diagMode.
 *
 *  AUTO   → listen first, then ask. Anything the car broadcasts is read from
 *           the bus. OBD-II is asked for what is not broadcast, and SSM2
 *           only for what OBD-II has no PID for (knock, A/F learning,
 *           switches). The learner keeps moving values onto the listening
 *           side, so requests shrink over time.
 *  SSM    → SSM2 requests only.  OBD → OBD-II requests only.
 *  SILENT → listen-only. The controller cannot ACK or transmit, so it is
 *           electrically incapable of disturbing the car; only the raw
 *           signal table produces data.
 */
#define DIAG_MODE_DEFAULT   DIAG_MODE_AUTO

/**
 * Seconds the master only listens after the car's bus comes up - at power-on,
 * on waking from sleep, and whenever the bus returns after going quiet (the
 * car switched off and on again) - before its first request. Every module is
 * starting at that moment; a request, or even an ACK, from us is one more
 * thing on a bus that is still settling. The car's own broadcasts are read,
 * and reach the displays, from the first frame. 0 turns the wait off.
 * Adjustable from the portal (start_delay_s).
 */
#define BUS_SETTLE_S        10

/** Broadcast cadence of the telemetry firehose. 40 ms = 25 Hz bursts. */
#define BROADCAST_PERIOD_MS 40

/**
 * A metric whose value has not changed is re-sent at least this often, so a
 * display that missed a burst, or was just switched on, never waits longer
 * than this for it. Must stay comfortably below the displays' stale timeout
 * (1500 ms) or an unchanging value would flicker to "--".
 */
#define BROADCAST_KEEPALIVE_MS 300

/** A metric older than this is dropped from the broadcast (master-side). */
#define METRIC_TTL_MS       1000

/* ═══════════════════════════════════════════════════════════════════════════
 *  NIGHT-MODE SOURCE
 * ═══════════════════════════════════════════════════════════════════════════ */
#define NIGHT_SOURCE_NONE    0  /**< Never set the night flag.               */
#define NIGHT_SOURCE_LDR     1  /**< Photoresistor on NIGHT_LDR_GPIO.        */
#define NIGHT_SOURCE_VEHICLE 2  /**< Whatever publishes METRIC_ID_NIGHT_SENSE:
                                     the SSM2 light-switch input, or a raw
                                     CAN headlight bit.                     */
#define NIGHT_SOURCE_CAN     NIGHT_SOURCE_VEHICLE  /**< Older name.          */
/*
 * VEHICLE by default.
 *
 * The ECU reports its light-switch input over SSM2 (address 0x64, bit 3), so
 * on a Subaru the displays dim from the car's own lighting with no extra
 * wire. When nothing publishes the night metric the flag simply stays clear,
 * so this default costs nothing on a car without that source.
 *
 * Do NOT pick LDR without the divider fitted: the pin floats, the reading
 * wanders across the thresholds, and both displays flip between day and
 * night at random - which looks like a bug in the displays.
 */
#define NIGHT_SOURCE        NIGHT_SOURCE_VEHICLE

#define NIGHT_LDR_DARK_ADC   900   /**< Below → night (12-bit ADC, 0-4095).  */
#define NIGHT_LDR_LIGHT_ADC 1400   /**< Above → day (hysteresis band).       */

/* ═══════════════════════════════════════════════════════════════════════════
 *  OBD-II SERVICE-01 POLL TABLE — SAE J1979 / ISO 15031-5
 * ═══════════════════════════════════════════════════════════════════════════
 *  Every standard Mode-01 PID a petrol car can have, with the formulas from
 *  the standard reduced to scale/offset form. A reply's data bytes are A, B,
 *  C, D; a field takes @c bytes of them starting at @c off (0 = A), assembled
 *  big-endian, then shifted right by @c shift and masked by @c mask:
 *
 *    value = ((field >> shift) & mask) * scale + offset
 *
 *  Several PIDs return two values (PID 01: lamp + DTC count; 14-1B: O2
 *  voltage + trim; 24-2B and 34-3B: wideband lambda + voltage or current).
 *  Each value is its own row with the same PID; the FIRST row of a PID sets
 *  its poll period and whether it is on, later rows (period 0) only decode.
 *
 *  Nothing here costs bus time unless the ECU supports it: the engine asks
 *  the ECU for its supported-PID bitmap first and never requests the rest,
 *  and it skips any PID whose values are already arriving from SSM2 or a
 *  verified broadcast frame. So the defaults are generous — breadth is free,
 *  and the periods keep slow values from crowding out fast ones.
 */
struct ObdPidDef {
    uint8_t  pid;        /**< Service-01 PID                                 */
    uint8_t  off;        /**< First data byte of the field (0 = A)           */
    uint8_t  bytes;      /**< Field width in bytes, 1-4                      */
    uint8_t  shift;      /**< Right shift after assembly                     */
    uint32_t mask;       /**< Applied after the shift; 0 = none              */
    bool     is_signed;  /**< Two's complement over the field width          */
    float    scale;      /**< Engineering-unit scale                         */
    float    offset;     /**< Engineering-unit offset                        */
    uint16_t metric_id;  /**< Published channel — see MasterPacket.h         */
    uint16_t period_ms;  /**< Poll interval; 0 on a PID's secondary rows     */
    bool     default_on; /**< Polled out of the box                          */
    const char *name;    /**< For the portal                                 */
};

/** One-byte (A) and two-byte (A*256+B) values — the common cases. */
#define PID_A(pid, sc, of, m, per, on, nm)  { pid, 0, 1, 0, 0, false, sc, of, m, per, on, nm }
#define PID_AB(pid, sc, of, m, per, on, nm) { pid, 0, 2, 0, 0, false, sc, of, m, per, on, nm }
/** A further value of a PID already listed. */
#define PID_MORE(pid, off, n, sh, mk, sg, sc, of, m, nm) { pid, off, n, sh, mk, sg, sc, of, m, 0, false, nm }

/** 100/255: percentages from one byte.  100/128: signed trims.  2/65536: λ. */
#define K_PCT    0.392157f
#define K_TRIM   0.78125f
#define K_LAMBDA 0.0000305176f

static const ObdPidDef OBD_POLL_TABLE[] = {
    /* — needle channels: fast ------------------------------------------- */
    PID_AB(0x0C, 0.25f,    0.0f,   METRIC_ID_RPM,           50, true,  "Engine RPM"),
    PID_A (0x0D, 1.0f,     0.0f,   METRIC_ID_SPEED,        100, true,  "Vehicle speed"),
    PID_A (0x11, K_PCT,    0.0f,   METRIC_ID_THROTTLE,     100, true,  "Throttle position"),
    PID_A (0x0B, 1.0f,     0.0f,   METRIC_ID_MAP,          100, true,  "Intake MAP"),
    PID_A (0x04, K_PCT,    0.0f,   METRIC_ID_ENGINE_LOAD,  150, true,  "Calculated load"),
    PID_AB(0x10, 0.01f,    0.0f,   METRIC_ID_MAF,          150, true,  "MAF air flow"),
    PID_A (0x0E, 0.5f,   -64.0f,   METRIC_ID_TIMING_ADV,   150, true,  "Timing advance"),
    PID_A (0x49, K_PCT,    0.0f,   METRIC_ID_ACCEL_PEDAL_D,150, true,  "Accel pedal D"),
    PID_A (0x5A, K_PCT,    0.0f,   METRIC_ID_REL_PEDAL,    150, true,  "Relative pedal"),

    /* — mixture ---------------------------------------------------------- */
    PID_A (0x06, K_TRIM, -100.0f,  METRIC_ID_STFT_B1,      250, true,  "STFT bank 1"),
    PID_A (0x07, K_TRIM, -100.0f,  METRIC_ID_LTFT_B1,     1000, true,  "LTFT bank 1"),
    PID_A (0x08, K_TRIM, -100.0f,  METRIC_ID_STFT_B2,      250, true,  "STFT bank 2"),
    PID_A (0x09, K_TRIM, -100.0f,  METRIC_ID_LTFT_B2,     1000, true,  "LTFT bank 2"),
    PID_AB(0x44, K_LAMBDA, 0.0f,   METRIC_ID_CMD_AFR,      200, true,  "Commanded lambda"),
    /* Narrowband O2: A/200 V, B = trim (100/128)B-100. */
    PID_A (0x14, 0.005f,   0.0f,   METRIC_ID_O2_B1S1_V,    250, true,  "O2 B1S1"),
    PID_MORE(0x14, 1, 1, 0, 0, false, K_TRIM, -100.0f, METRIC_ID_O2_B1S1_TRIM, "O2 B1S1 trim"),
    PID_A (0x15, 0.005f,   0.0f,   METRIC_ID_O2_B1S2_V,    500, true,  "O2 B1S2"),
    PID_MORE(0x15, 1, 1, 0, 0, false, K_TRIM, -100.0f, METRIC_ID_O2_B1S2_TRIM, "O2 B1S2 trim"),
    PID_A (0x18, 0.005f,   0.0f,   METRIC_ID_O2_B2S1_V,    250, true,  "O2 B2S1"),
    PID_MORE(0x18, 1, 1, 0, 0, false, K_TRIM, -100.0f, METRIC_ID_O2_B2S1_TRIM, "O2 B2S1 trim"),
    PID_A (0x19, 0.005f,   0.0f,   METRIC_ID_O2_B2S2_V,    500, true,  "O2 B2S2"),
    PID_MORE(0x19, 1, 1, 0, 0, false, K_TRIM, -100.0f, METRIC_ID_O2_B2S2_TRIM, "O2 B2S2 trim"),
    /* Wideband, voltage type: AB = λ (2/65536), CD = V (8/65536). */
    PID_AB(0x24, K_LAMBDA, 0.0f,   METRIC_ID_WB_B1S1_LAMBDA,150, true, "WB B1S1 lambda"),
    PID_MORE(0x24, 2, 2, 0, 0, false, 0.00012207f, 0.0f, METRIC_ID_WB_B1S1_V, "WB B1S1 volts"),
    PID_AB(0x28, K_LAMBDA, 0.0f,   METRIC_ID_WB_B2S1_LAMBDA,150, true, "WB B2S1 lambda"),
    PID_MORE(0x28, 2, 2, 0, 0, false, 0.00012207f, 0.0f, METRIC_ID_WB_B2S1_V, "WB B2S1 volts"),
    /* Wideband, current type: AB = λ, CD = mA as (256C+D)/256 - 128. */
    PID_AB(0x34, K_LAMBDA, 0.0f,   METRIC_ID_WBC_B1S1_LAMBDA,150, true, "WB B1S1 lambda (I)"),
    PID_MORE(0x34, 2, 2, 0, 0, false, 0.00390625f, -128.0f, METRIC_ID_WBC_B1S1_MA, "WB B1S1 current"),
    PID_AB(0x38, K_LAMBDA, 0.0f,   METRIC_ID_WBC_B2S1_LAMBDA,150, true, "WB B2S1 lambda (I)"),
    PID_MORE(0x38, 2, 2, 0, 0, false, 0.00390625f, -128.0f, METRIC_ID_WBC_B2S1_MA, "WB B2S1 current"),

    /* — load, throttle and torque detail ------------------------------- */
    PID_AB(0x43, K_PCT,    0.0f,   METRIC_ID_ABS_LOAD,     250, true,  "Absolute load"),
    PID_A (0x45, K_PCT,    0.0f,   METRIC_ID_REL_THROTTLE, 250, true,  "Relative throttle"),
    PID_A (0x47, K_PCT,    0.0f,   METRIC_ID_ABS_THROTTLE_B,250, true, "Throttle B"),
    PID_A (0x48, K_PCT,    0.0f,   METRIC_ID_ABS_THROTTLE_C,500, true, "Throttle C"),
    PID_A (0x4A, K_PCT,    0.0f,   METRIC_ID_ACCEL_PEDAL_E,250, true,  "Accel pedal E"),
    PID_A (0x4B, K_PCT,    0.0f,   METRIC_ID_ACCEL_PEDAL_F,500, true,  "Accel pedal F"),
    PID_A (0x4C, K_PCT,    0.0f,   METRIC_ID_CMD_THROTTLE, 250, true,  "Commanded throttle"),
    PID_A (0x61, 1.0f,  -125.0f,   METRIC_ID_TORQUE_DEMAND,200, true,  "Demand torque"),
    PID_A (0x62, 1.0f,  -125.0f,   METRIC_ID_TORQUE_ACTUAL,200, true,  "Actual torque"),
    PID_AB(0x63, 1.0f,     0.0f,   METRIC_ID_TORQUE_REF, 10000, true,  "Reference torque"),
    PID_AB(0x5D, 0.0078125f,-210.0f,METRIC_ID_INJ_TIMING,  250, true,  "Injection timing"),
    PID_AB(0x5E, 0.05f,    0.0f,   METRIC_ID_FUEL_RATE,    500, true,  "Fuel rate"),

    /* — fuel and evap pressures ---------------------------------------- */
    PID_A (0x0A, 3.0f,     0.0f,   METRIC_ID_FUEL_PRESSURE,1000, true, "Fuel pressure"),
    PID_AB(0x22, 0.079f,   0.0f,   METRIC_ID_FUEL_RAIL_REL,1000, true, "Fuel rail (vac)"),
    PID_AB(0x23, 10.0f,    0.0f,   METRIC_ID_FUEL_RAIL_GAUGE,1000,true,"Fuel rail gauge"),
    PID_AB(0x59, 10.0f,    0.0f,   METRIC_ID_FUEL_RAIL_ABS,1000, true, "Fuel rail abs"),
    { 0x32, 0, 2, 0, 0, true, 0.25f, 0.0f, METRIC_ID_EVAP_VP, 1000, true, "Evap vapor press" },
    PID_AB(0x53, 0.005f,   0.0f,   METRIC_ID_EVAP_ABS,    1000, true,  "Evap abs press"),
    PID_A (0x2E, K_PCT,    0.0f,   METRIC_ID_EVAP_PURGE,  1000, true,  "Evap purge"),
    PID_A (0x2C, K_PCT,    0.0f,   METRIC_ID_EGR_CMD,     1000, true,  "EGR commanded"),
    PID_A (0x2D, K_TRIM, -100.0f,  METRIC_ID_EGR_ERROR,   1000, true,  "EGR error"),

    /* — temperatures, voltages, levels: slow ---------------------------- */
    PID_A (0x05, 1.0f,   -40.0f,   METRIC_ID_COOLANT_TEMP, 1000, true, "Coolant"),
    PID_A (0x0F, 1.0f,   -40.0f,   METRIC_ID_IAT,          1000, true, "Intake air temp"),
    PID_A (0x5C, 1.0f,   -40.0f,   METRIC_ID_OIL_TEMP,     1000, true, "Oil temp"),
    PID_A (0x46, 1.0f,   -40.0f,   METRIC_ID_AMBIENT_TEMP, 2000, true, "Ambient air"),
    PID_AB(0x3C, 0.1f,   -40.0f,   METRIC_ID_CAT_TEMP_B1S1,1000, true, "Cat temp B1S1"),
    PID_AB(0x3D, 0.1f,   -40.0f,   METRIC_ID_CAT_TEMP_B2S1,1000, true, "Cat temp B2S1"),
    PID_AB(0x3E, 0.1f,   -40.0f,   METRIC_ID_CAT_TEMP_B1S2,2000, true, "Cat temp B1S2"),
    PID_AB(0x3F, 0.1f,   -40.0f,   METRIC_ID_CAT_TEMP_B2S2,2000, true, "Cat temp B2S2"),
    PID_AB(0x42, 0.001f,   0.0f,   METRIC_ID_BATT_VOLTAGE,  500, true, "Module voltage"),
    PID_A (0x33, 1.0f,     0.0f,   METRIC_ID_BARO,         2000, true, "Barometric"),
    PID_A (0x2F, K_PCT,    0.0f,   METRIC_ID_FUEL_LEVEL,   2000, true, "Fuel level"),
    PID_A (0x52, K_PCT,    0.0f,   METRIC_ID_ETHANOL,     10000, true, "Ethanol"),
    PID_A (0x5B, K_PCT,    0.0f,   METRIC_ID_HYBRID_SOC,   5000, true, "Hybrid pack"),

    /* — status, counters, distances ------------------------------------- */
    /* PID 01: bit 7 of A is the MIL, bits 0-6 the stored-DTC count. */
    { 0x01, 0, 1, 7, 0x01, false, 1.0f, 0.0f, METRIC_ID_MIL, 2000, true, "MIL lamp" },
    PID_MORE(0x01, 0, 1, 0, 0x7F, false, 1.0f, 0.0f, METRIC_ID_DTC_COUNT, "DTC count"),
    PID_A (0x03, 1.0f,     0.0f,   METRIC_ID_FUEL_SYS,     1000, true, "Fuel system status"),
    PID_AB(0x1F, 1.0f,     0.0f,   METRIC_ID_RUN_TIME,     5000, true, "Run time"),
    PID_AB(0x21, 1.0f,     0.0f,   METRIC_ID_DIST_MIL,    10000, true, "Distance, MIL on"),
    PID_AB(0x4D, 1.0f,     0.0f,   METRIC_ID_TIME_MIL,    10000, true, "Time, MIL on"),
    PID_A (0x30, 1.0f,     0.0f,   METRIC_ID_WARMUPS,     10000, true, "Warm-ups since clear"),
    PID_AB(0x31, 1.0f,     0.0f,   METRIC_ID_DIST_CLEARED,10000, true, "Distance since clear"),
    PID_AB(0x4E, 1.0f,     0.0f,   METRIC_ID_TIME_CLEARED,10000, true, "Time since clear"),
    PID_A (0x51, 1.0f,     0.0f,   METRIC_ID_FUEL_TYPE,   30000, true, "Fuel type"),
    { 0xA6, 0, 4, 0, 0, false, 0.1f, 0.0f, METRIC_ID_ODOMETER, 10000, true, "Odometer" },
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  RAW CAN SIGNAL TABLE (passive sniffing, DBC-style extraction)
 * ═══════════════════════════════════════════════════════════════════════════
 *  For data the OBD port doesn't expose, and — because broadcast frames
 *  arrive at 50-100 Hz without being asked for — the fastest source of
 *  anything the car happens to broadcast.
 *
 *  Bit numbering:
 *   - LittleEndian (Intel):    start_bit = LSB position (DBC "@1+").
 *   - BigEndian   (Motorola):  start_bit = DBC start bit, MSB-first ("@0+").
 *  value = raw * scale + offset (after optional sign extension).
 *
 *  ## Self-verification
 *
 *  A signal in mode AUTO is decoded but NOT published. Instead each decoded
 *  value is compared with the same quantity from the ECU itself (SSM2 or
 *  OBD-II) — the `ref_metric`. Once enough samples agree, over a range wide
 *  enough to rule out coincidence, the signal is promoted to VERIFIED and
 *  starts publishing; and because it is now known to be the frame the ECU
 *  agrees with, the other signals decoded from the SAME frame that have no
 *  reference of their own are promoted with it. If the samples disagree, the
 *  signal is marked REJECTED and left alone.
 *
 *  This is what makes shipping unverified frames safe. An unverified signal
 *  that happened to collide with a real frame would otherwise publish a
 *  confident, wrong number into a named channel, and a gauge reading
 *  plausible nonsense is far worse than a gauge reading nothing.
 */
enum class ByteOrder : uint8_t { LittleEndian, BigEndian };

/** @brief Life cycle of a raw signal. Stored in MasterConfig as a number. */
enum SignalMode : uint8_t {
    SIG_OFF      = 0,  /**< Never decoded.                                   */
    SIG_ON       = 1,  /**< Published unconditionally (manually verified).   */
    SIG_AUTO     = 2,  /**< Candidate: compared against ref_metric.          */
    SIG_VERIFIED = 3,  /**< Promoted from AUTO; publishes like ON.           */
    SIG_REJECTED = 4,  /**< Failed verification; never decoded.              */
};

struct CanSignalDef {
    uint32_t  can_id;     /**< 11-bit or 29-bit frame identifier             */
    bool      extended;   /**< true = 29-bit                                 */
    uint8_t   start_bit;  /**< See bit-numbering note above                  */
    uint8_t   bit_length; /**< 1–32 bits                                     */
    ByteOrder order;
    bool      is_signed;  /**< Two's-complement sign extension               */
    float     scale;
    float     offset;
    uint16_t  metric_id;
    uint8_t   mode;       /**< SignalMode this entry starts in               */
    uint16_t  ref_metric; /**< Metric to verify against (0 = none)           */
    const char *name;
};

/*
 * Empty on purpose. The frames published for other Subaru generations turned
 * out not to exist on this car (a 2009 Legacy broadcasts none of them but
 * steering), and no map of this generation has been published. The learner
 * (Learner.h) finds the real ones on the car itself, so seeding guesses here
 * only cluttered the table. The one placeholder row keeps the array legal;
 * RAW_SIGNAL_COUNT is what the code iterates.
 */
static const CanSignalDef RAW_SIGNAL_TABLE[] = {
    { 0, false, 0, 1, ByteOrder::LittleEndian, false, 1.0f, 0.0f, 0, SIG_OFF, 0, "" },
};
static constexpr size_t RAW_SIGNAL_COUNT = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 *  MASTER-SIDE ALARM THRESHOLDS → METRIC_FLAG_WARNING / _CRITICAL
 * ═══════════════════════════════════════════════════════════════════════════
 *  The slave also has its own configurable thresholds; master flags act as a
 *  vehicle-defined safety floor that every display must honor.
 *  Use NAN to disable a bound.
 */
struct ThresholdDef {
    uint16_t metric_id;
    float    warn_low,  warn_high;   /**< Warning outside (low, high)        */
    float    crit_low,  crit_high;   /**< Critical outside (low, high)       */
};

static const ThresholdDef THRESHOLD_TABLE[] = {
    /* metric,                 wLow,  wHigh, cLow,  cHigh                    */
    { METRIC_ID_RPM,            NAN,  6200,   NAN,  7000 },
    { METRIC_ID_COOLANT_TEMP,   NAN,   105,   NAN,   115 },
    { METRIC_ID_OIL_TEMP,       NAN,   130,   NAN,   145 },
    { METRIC_ID_BATT_VOLTAGE,  12.1f, 15.0f, 11.5f, 15.8f },
    { METRIC_ID_BATT_CURRENT, -40.0f,   NAN, -80.0f,  NAN },
    { METRIC_ID_BATT_SOC,      40.0f,   NAN, 20.0f,   NAN },
    { METRIC_ID_ALT_LOAD,       NAN,    90,   NAN,    98 },
    { METRIC_ID_BOOST,          NAN,   1.4f,  NAN,   1.6f },
    { METRIC_ID_SSM_KNOCK_CORR, -4.0f,  NAN, -8.0f,   NAN },
    { METRIC_ID_EGT,            NAN,   900,   NAN,   950 },
    /* The check-engine lamp is critical the moment it lights; a stored code
     * without the lamp is a warning. Both reach the displays' watchdog. */
    { METRIC_ID_MIL,            NAN,   NAN,   NAN,  0.5f },
    { METRIC_ID_DTC_COUNT,      NAN,  0.5f,   NAN,   NAN },
};
