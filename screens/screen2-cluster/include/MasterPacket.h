/**
 * @file MasterPacket.h
 * @brief ESP-NOW broadcast data contract between the CAN Master Node
 *        ("Data Firehose") and any number of Slave display nodes
 *        ("Intelligent Canvas").
 *
 * =============================================================================
 *  CONTRACT RULES — read before editing
 * =============================================================================
 *  1. This header MUST be byte-identical on the Master and every Slave.
 *     (Copies in master/ and each screen's include/ — keep all three in sync.)
 *  2. Structures are packed (#pragma pack(1)); both nodes are little-endian
 *     ESP32 variants, so no endian conversion is performed on the wire.
 *  3. A single frame must NEVER exceed 250 bytes (hard ESP-NOW v1 limit).
 *     A static_assert below enforces this at compile time.
 *     ┌─ SCALING BEYOND 28 METRICS: MULTI-FRAME BURSTS ─────────────────────┐
 *     │ ESP-NOW v2 (IDF 5.x) can carry 1490 B, but silently falls back to   │
 *     │ 250 B whenever a v1 peer is present — never build a contract on it. │
 *     │ Instead, a Master with more than TELEMETRY_MAX_METRICS channels     │
 *     │ splits them across CONSECUTIVE frames ("burst"): every frame is a   │
 *     │ fully valid MasterTelemetryPacket, sequence_id increments per       │
 *     │ frame, and each metric simply appears in one frame of the burst.    │
 *     │ Slaves merge by metric_id into their store, so ANY number of CAN    │
 *     │ parameters is supported with zero slave-side changes.               │
 *     └─────────────────────────────────────────────────────────────────────┘
 *  4. Adding fields is a BREAKING change: bump @ref TELEMETRY_PROTO_VERSION
 *     and update both nodes together. Adding new metric IDs is NOT breaking.
 *  5. The Master broadcasts to FF:FF:FF:FF:FF:FF on a fixed Wi-Fi channel;
 *     Slaves are pure listeners and never transmit. Master and Slave MUST
 *     agree on @ref TELEMETRY_WIFI_CHANNEL or no frames will be received.
 *  6. A metric that is not present in a burst is not "zero": the master only
 *     sends a channel when its value changed or a keep-alive is due. Slaves
 *     keep the last value and age it against their own stale timeout.
 * =============================================================================
 */
#pragma once

#include <stdint.h>

/* ───────────────────────────── Radio parameters ─────────────────────────── */

/** Protocol/schema version carried implicitly by both firmwares. */
#define TELEMETRY_PROTO_VERSION   1

/**
 * Wi-Fi channel used for the ESP-NOW broadcast.
 * The Slave locks its STA interface to this channel; the Master's AP/STA
 * interface must be on the same one. Valid: 1–13 (region-dependent).
 */
#define TELEMETRY_WIFI_CHANNEL    1

/** ESP-NOW broadcast MAC — every listening slave receives the firehose. */
#define TELEMETRY_BROADCAST_ADDR  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

/* ─────────────────────────── MetricEntry.flags bits ─────────────────────── */

#define METRIC_FLAG_VALID     (1u << 0)  /**< Value is fresh & trustworthy.
                                              Slaves must render "--" when
                                              clear.                          */
#define METRIC_FLAG_WARNING   (1u << 1)  /**< Master-side warning threshold
                                              crossed (yellow-tier alarm).    */
#define METRIC_FLAG_CRITICAL  (1u << 2)  /**< Master-side critical threshold
                                              crossed (red-tier alarm).       */
#define METRIC_FLAG_NIGHT     (1u << 3)  /**< Vehicle is in night mode (head-
                                              lights on / low ambient light).
                                              Global: Master sets it on every
                                              entry of the frame.             */
/* bits 4–7 reserved — Masters must transmit them as 0. */

/* ─────────────────────────── Metric ID registry ─────────────────────────── */
/**
 * @defgroup MetricIDs Standardized metric identifiers
 *
 * Layout of the 16-bit ID space:
 *   0x01xx — OBD-II Service 01 PIDs (low byte = PID). Values are already
 *            parsed to engineering units by the Master.
 *   0x02xx — OBD-II Service 22 / manufacturer extended (project-defined).
 *   0x10xx — Raw-CAN decoded signals: powertrain/chassis (DBC-style).
 *   0x11xx — Raw-CAN decoded signals: ELECTRICAL SYSTEM (battery, alternator,
 *            charging, 12 V rail, load management).
 *   0x20xx — Subaru SSM2 measurements with no OBD-II equivalent.
 *   0x21xx — Subaru SSM2 switches: 0 or 1, straight from the ECU's inputs
 *            and outputs.
 *   0x12xx — Body & chassis state heard on the bus (turn signals, doors…).
 *   0x30xx — Custom channels, for bus values with no standard ID.
 *   0x1Fxx — System/housekeeping channels.
 *
 * Units listed are MANDATORY — the Master must scale to exactly these units
 * so any slave layout renders correctly without per-vehicle knowledge.
 * @{
 */
#define METRIC_ID_ENGINE_LOAD     0x0104  /**< Calculated engine load   [%]   */
#define METRIC_ID_COOLANT_TEMP    0x0105  /**< Engine coolant temp      [°C]  */
#define METRIC_ID_MAP             0x010B  /**< Intake manifold pressure [kPa] */
#define METRIC_ID_RPM             0x010C  /**< Engine speed             [rpm] */
#define METRIC_ID_SPEED           0x010D  /**< Vehicle speed            [km/h]*/
#define METRIC_ID_TIMING_ADV      0x010E  /**< Ignition timing advance  [°]   */
#define METRIC_ID_IAT             0x010F  /**< Intake air temperature   [°C]  */
#define METRIC_ID_MAF             0x0110  /**< Mass air flow            [g/s] */
#define METRIC_ID_THROTTLE        0x0111  /**< Throttle position        [%]   */
#define METRIC_ID_FUEL_LEVEL      0x012F  /**< Fuel tank level          [%]   */
#define METRIC_ID_BATT_VOLTAGE    0x0142  /**< Control module voltage   [V]   */
#define METRIC_ID_HYBRID_SOC      0x015B  /**< Hybrid pack remaining    [%]   */
#define METRIC_ID_OIL_TEMP        0x015C  /**< Engine oil temperature   [°C]  */

#define METRIC_ID_BOOST           0x1001  /**< Boost/vacuum (gauge)     [bar] */
#define METRIC_ID_AFR             0x1002  /**< Air-fuel ratio           [AFR] */
#define METRIC_ID_OIL_PRESSURE    0x1003  /**< Oil pressure             [bar] */
#define METRIC_ID_EGT             0x1004  /**< Exhaust gas temperature  [°C]  */
#define METRIC_ID_GEAR            0x1005  /**< Current gear (0=N,-1=R)  [-]   */
#define METRIC_ID_STEERING_ANGLE  0x1006  /**< Steering angle           [°]   */
#define METRIC_ID_BRAKE_PRESSURE  0x1007  /**< Brake circuit pressure   [bar] */
#define METRIC_ID_WHEEL_FL        0x1008  /**< Wheel speed front left   [km/h]*/
#define METRIC_ID_WHEEL_FR        0x1009  /**< Wheel speed front right  [km/h]*/
#define METRIC_ID_WHEEL_RL        0x100A  /**< Wheel speed rear left    [km/h]*/
#define METRIC_ID_WHEEL_RR        0x100B  /**< Wheel speed rear right   [km/h]*/
#define METRIC_ID_LAT_ACCEL       0x100C  /**< Lateral acceleration     [g]   */
#define METRIC_ID_LON_ACCEL       0x100D  /**< Longitudinal accel.      [g]   */
#define METRIC_ID_YAW_RATE        0x100E  /**< Yaw rate                 [°/s] */
/** Gear lever position, sent as its letter's character code - 'P' 80,
 *  'R' 82, 'N' 78, 'D' 68, 'M' 77, '3' 51 - so any lever (P R N D, a manual
 *  gate, 3 2 1) needs no table on the screens: they print the character. */
#define METRIC_ID_GEAR_LEVER      0x100F  /**< Gear lever (letter code) [-]   */
#define METRIC_ID_CRUISE_SET      0x1010  /**< Cruise set speed         [km/h]*/
#define METRIC_ID_ATF_TEMP        0x1011  /**< Transmission fluid temp  [°C]  */

/* ---- Body & chassis state from the bus (0x12xx) — 0 or 1 ---------------
 * Found by the portal's "teach by doing" mode: nothing requests these, the
 * car simply broadcasts them.
 */
#define METRIC_ID_TURN_LEFT       0x1201  /**< Left turn signal               */
#define METRIC_ID_TURN_RIGHT      0x1202  /**< Right turn signal              */
#define METRIC_ID_HIGH_BEAM       0x1203  /**< High beam                      */
#define METRIC_ID_DOOR_OPEN       0x1204  /**< Any door open                  */
#define METRIC_ID_HANDBRAKE       0x1205  /**< Handbrake applied              */
#define METRIC_ID_SEATBELT        0x1206  /**< Driver seatbelt unfastened     */
#define METRIC_ID_CRUISE_ON       0x1207  /**< Cruise control active          */
#define METRIC_ID_REVERSE         0x1208  /**< Reverse selected               */
#define METRIC_ID_PARK            0x1209  /**< Lever in P                     */
#define METRIC_ID_NEUTRAL         0x120A  /**< Lever in N                     */
#define METRIC_ID_DRIVE           0x120B  /**< Lever in D                     */
#define METRIC_ID_HAZARD          0x120C  /**< Hazard lights switched on      */
#define METRIC_ID_CRUISE_MAIN     0x120D  /**< Cruise main switch on          */
#define METRIC_ID_FOG_FRONT       0x120E  /**< Front fog lights on            */
#define METRIC_ID_FOG_REAR        0x120F  /**< Rear fog light on              */
#define METRIC_ID_DOOR_FL         0x1210  /**< Front left door open           */
#define METRIC_ID_DOOR_FR         0x1211  /**< Front right door open          */
#define METRIC_ID_DOOR_RL         0x1212  /**< Rear left door open            */
#define METRIC_ID_DOOR_RR         0x1213  /**< Rear right door open           */
#define METRIC_ID_TRUNK           0x1214  /**< Trunk open                     */
#define METRIC_ID_HOOD            0x1215  /**< Hood open                      */
#define METRIC_ID_VDC_OFF         0x1216  /**< VDC switched off               */
#define METRIC_ID_SEATBELT_PASS   0x1217  /**< Passenger seatbelt unfastened  */

/* ---- Custom channels (0x30xx) -------------------------------------------
 * For anything found on the bus that has no standard ID. Name it on the
 * gauge; the master only carries the number.
 */
#define METRIC_ID_CUSTOM_1        0x3001  /**< Custom channel 1               */
#define METRIC_ID_CUSTOM_2        0x3002  /**< Custom channel 2               */
#define METRIC_ID_CUSTOM_3        0x3003  /**< Custom channel 3               */
#define METRIC_ID_CUSTOM_4        0x3004  /**< Custom channel 4               */
#define METRIC_ID_CUSTOM_5        0x3005  /**< Custom channel 5               */
#define METRIC_ID_CUSTOM_6        0x3006  /**< Custom channel 6               */
#define METRIC_ID_CUSTOM_7        0x3007  /**< Custom channel 7               */
#define METRIC_ID_CUSTOM_8        0x3008  /**< Custom channel 8               */
#define METRIC_ID_CUSTOM_9        0x3009  /**< Custom channel 9               */
#define METRIC_ID_CUSTOM_10       0x300A  /**< Custom channel 10              */
#define METRIC_ID_CUSTOM_11       0x300B  /**< Custom channel 11              */
#define METRIC_ID_CUSTOM_12       0x300C  /**< Custom channel 12              */
#define METRIC_ID_CUSTOM_13       0x300D  /**< Custom channel 13              */
#define METRIC_ID_CUSTOM_14       0x300E  /**< Custom channel 14              */
#define METRIC_ID_CUSTOM_15       0x300F  /**< Custom channel 15              */
#define METRIC_ID_CUSTOM_16       0x3010  /**< Custom channel 16              */

/* ---- Electrical system (0x11xx) — battery / charging / 12 V network ----- */
#define METRIC_ID_BATT_CURRENT    0x1101  /**< Battery current (+ = charging,
                                               − = discharging)         [A]  */
#define METRIC_ID_BATT_SOC        0x1102  /**< Battery state of charge  [%]   */
#define METRIC_ID_BATT_SOH        0x1103  /**< Battery state of health  [%]   */
#define METRIC_ID_BATT_TEMP       0x1104  /**< Battery temperature      [°C]  */
#define METRIC_ID_ALT_VOLTAGE     0x1105  /**< Alternator output voltage[V]   */
#define METRIC_ID_ALT_CURRENT     0x1106  /**< Alternator output current[A]   */
#define METRIC_ID_ALT_LOAD        0x1107  /**< Alternator utilization   [%]   */
#define METRIC_ID_RAIL_12V        0x1108  /**< 12 V rail (fuse box) volt[V]   */
#define METRIC_ID_ELEC_LOAD_W     0x1109  /**< Total electrical load    [W]   */
#define METRIC_ID_CHARGE_STATUS   0x110A  /**< 0=off 1=bulk 2=float 3=fault   */

/* ---- Subaru SSM2 (0x20xx) ----------------------------------------------
 * A 2008+ Subaru speaks SSM2 over CAN on the same 0x7E0/0x7E8 addresses as
 * OBD-II, and command 0xA8 reads arbitrary ECU memory by 3-byte address. That
 * is where the parameters OBD-II never exposes live: knock correction, AF
 * learning, fine knock learn, injector duty.
 *
 * Anything that DOES have an OBD-II equivalent keeps its 0x01xx identifier so
 * a gauge configured against OBD-II keeps working when the same value starts
 * arriving over SSM2 instead — the display neither knows nor cares which
 * protocol produced a number.
 */
#define METRIC_ID_SSM_KNOCK_CORR  0x2001  /**< Knock correction advance [°]  */
#define METRIC_ID_SSM_FINE_KNOCK  0x2002  /**< Fine knock correction    [°]  */
#define METRIC_ID_SSM_AF_LEARN    0x2003  /**< A/F learning #1          [%]  */
#define METRIC_ID_SSM_AF_CORR     0x2004  /**< A/F correction #1        [%]  */
#define METRIC_ID_SSM_INJ_DUTY    0x2005  /**< Injector duty cycle      [%]  */
#define METRIC_ID_SSM_TGT_BOOST   0x2006  /**< Target boost             [bar]*/
#define METRIC_ID_SSM_WGDC        0x2007  /**< Wastegate duty           [%]  */

/* ---- OBD-II Service 01, extended set (low byte = PID) ------------------
 * Everything below is standard Mode 01. Support varies by vehicle: an ECU
 * that does not implement a PID simply never answers, the metric stays absent
 * and the displays render it stale, so listing one costs nothing but a poll
 * slot. Most are disabled in the poll table by default — see the note there
 * about bus time.
 */
#define METRIC_ID_STFT_B1         0x0106  /**< Short term fuel trim B1  [%]  */
#define METRIC_ID_LTFT_B1         0x0107  /**< Long term fuel trim B1   [%]  */
#define METRIC_ID_STFT_B2         0x0108  /**< Short term fuel trim B2  [%]  */
#define METRIC_ID_LTFT_B2         0x0109  /**< Long term fuel trim B2   [%]  */
#define METRIC_ID_FUEL_PRESSURE   0x010A  /**< Fuel pressure            [kPa]*/
#define METRIC_ID_RUN_TIME        0x011F  /**< Run time since start     [s]  */
#define METRIC_ID_DIST_MIL        0x0121  /**< Distance with MIL on     [km] */
#define METRIC_ID_EGR_CMD         0x012C  /**< Commanded EGR            [%]  */
#define METRIC_ID_EGR_ERROR       0x012D  /**< EGR error                [%]  */
#define METRIC_ID_EVAP_PURGE      0x012E  /**< Commanded evap purge     [%]  */
#define METRIC_ID_WARMUPS         0x0130  /**< Warm-ups since cleared   [-]  */
#define METRIC_ID_DIST_CLEARED    0x0131  /**< Distance since cleared   [km] */
#define METRIC_ID_BARO            0x0133  /**< Barometric pressure      [kPa]*/
#define METRIC_ID_CAT_TEMP_B1S1   0x013C  /**< Catalyst temp B1S1       [°C] */
#define METRIC_ID_ABS_LOAD        0x0143  /**< Absolute load value      [%]  */
#define METRIC_ID_CMD_AFR         0x0144  /**< Commanded equivalence    [λ]  */
#define METRIC_ID_REL_THROTTLE    0x0145  /**< Relative throttle        [%]  */
#define METRIC_ID_AMBIENT_TEMP    0x0146  /**< Ambient air temp         [°C] */
#define METRIC_ID_ABS_THROTTLE_B  0x0147  /**< Absolute throttle B      [%]  */
#define METRIC_ID_ACCEL_PEDAL_D   0x0149  /**< Accelerator pedal D      [%]  */
#define METRIC_ID_ACCEL_PEDAL_E   0x014A  /**< Accelerator pedal E      [%]  */
#define METRIC_ID_CMD_THROTTLE    0x014C  /**< Commanded throttle       [%]  */
#define METRIC_ID_FUEL_RATE       0x015E  /**< Engine fuel rate         [L/h]*/

/* ---- OBD-II Service 01, full standard set (SAE J1979 / ISO 15031-5) -----
 * 0x01PP carries the first value of PID PP. The handful of PIDs that return
 * two values put the second one in 0x03PP, so "low byte = PID" holds for
 * both and nothing needs a lookup table to find its source.
 */
#define METRIC_ID_MIL             0x0101  /**< Check-engine lamp on     [0/1]*/
#define METRIC_ID_DTC_COUNT       0x0301  /**< Stored emission DTCs     [-]  */
#define METRIC_ID_FUEL_SYS        0x0103  /**< Fuel system status: 1 open
                                               (cold) 2 closed loop 4 open
                                               (load) 8 open (fault) 16
                                               closed (fault)           [-]  */
#define METRIC_ID_O2_B1S1_V       0x0114  /**< O2 bank 1 sensor 1       [V]  */
#define METRIC_ID_O2_B1S2_V       0x0115  /**< O2 bank 1 sensor 2       [V]  */
#define METRIC_ID_O2_B2S1_V       0x0118  /**< O2 bank 2 sensor 1       [V]  */
#define METRIC_ID_O2_B2S2_V       0x0119  /**< O2 bank 2 sensor 2       [V]  */
#define METRIC_ID_O2_B1S1_TRIM    0x0314  /**< O2 B1S1 short-term trim  [%]  */
#define METRIC_ID_O2_B1S2_TRIM    0x0315  /**< O2 B1S2 short-term trim  [%]  */
#define METRIC_ID_O2_B2S1_TRIM    0x0318  /**< O2 B2S1 short-term trim  [%]  */
#define METRIC_ID_O2_B2S2_TRIM    0x0319  /**< O2 B2S2 short-term trim  [%]  */
#define METRIC_ID_FUEL_RAIL_REL   0x0122  /**< Fuel rail press (vacuum) [kPa]*/
#define METRIC_ID_FUEL_RAIL_GAUGE 0x0123  /**< Fuel rail gauge pressure [kPa]*/
#define METRIC_ID_WB_B1S1_LAMBDA  0x0124  /**< Wideband B1S1            [λ]  */
#define METRIC_ID_WB_B1S1_V       0x0324  /**< Wideband B1S1 voltage    [V]  */
#define METRIC_ID_WB_B2S1_LAMBDA  0x0128  /**< Wideband B2S1            [λ]  */
#define METRIC_ID_WB_B2S1_V       0x0328  /**< Wideband B2S1 voltage    [V]  */
#define METRIC_ID_EVAP_VP         0x0132  /**< Evap system vapor press  [Pa] */
#define METRIC_ID_WBC_B1S1_LAMBDA 0x0134  /**< Wideband B1S1 (current)  [λ]  */
#define METRIC_ID_WBC_B1S1_MA     0x0334  /**< Wideband B1S1 current    [mA] */
#define METRIC_ID_WBC_B2S1_LAMBDA 0x0138  /**< Wideband B2S1 (current)  [λ]  */
#define METRIC_ID_WBC_B2S1_MA     0x0338  /**< Wideband B2S1 current    [mA] */
#define METRIC_ID_CAT_TEMP_B2S1   0x013D  /**< Catalyst temp B2S1       [°C] */
#define METRIC_ID_CAT_TEMP_B1S2   0x013E  /**< Catalyst temp B1S2       [°C] */
#define METRIC_ID_CAT_TEMP_B2S2   0x013F  /**< Catalyst temp B2S2       [°C] */
#define METRIC_ID_ABS_THROTTLE_C  0x0148  /**< Absolute throttle C      [%]  */
#define METRIC_ID_ACCEL_PEDAL_F   0x014B  /**< Accelerator pedal F      [%]  */
#define METRIC_ID_TIME_MIL        0x014D  /**< Time run with MIL on     [min]*/
#define METRIC_ID_TIME_CLEARED    0x014E  /**< Time since codes cleared [min]*/
#define METRIC_ID_FUEL_TYPE       0x0151  /**< Fuel type (J1979 table)  [-]  */
#define METRIC_ID_ETHANOL         0x0152  /**< Ethanol fuel             [%]  */
#define METRIC_ID_EVAP_ABS        0x0153  /**< Abs evap vapor pressure  [kPa]*/
#define METRIC_ID_FUEL_RAIL_ABS   0x0159  /**< Fuel rail abs pressure   [kPa]*/
#define METRIC_ID_REL_PEDAL       0x015A  /**< Relative pedal position  [%]  */
#define METRIC_ID_INJ_TIMING      0x015D  /**< Fuel injection timing    [°]  */
#define METRIC_ID_TORQUE_DEMAND   0x0161  /**< Driver demand torque     [%]  */
#define METRIC_ID_TORQUE_ACTUAL   0x0162  /**< Actual engine torque     [%]  */
#define METRIC_ID_TORQUE_REF      0x0163  /**< Engine reference torque  [Nm] */
#define METRIC_ID_ODOMETER        0x01A6  /**< Odometer                 [km] */

/* ---- Subaru SSM2, extended set (0x20xx) -------------------------------- */
#define METRIC_ID_SSM_AF_CORR2    0x2008  /**< A/F correction #2        [%]  */
#define METRIC_ID_SSM_AF_LEARN2   0x2009  /**< A/F learning #2          [%]  */
#define METRIC_ID_SSM_O2_F1       0x200A  /**< Front O2 #1              [V]  */
#define METRIC_ID_SSM_O2_R        0x200B  /**< Rear O2                  [V]  */
#define METRIC_ID_SSM_O2_F2       0x200C  /**< Front O2 #2              [V]  */
#define METRIC_ID_SSM_MAF_VOLT    0x200D  /**< Air flow sensor          [V]  */
#define METRIC_ID_SSM_TPS_VOLT    0x200E  /**< Throttle sensor          [V]  */
#define METRIC_ID_SSM_ATMOS       0x200F  /**< Atmospheric pressure     [kPa]*/
#define METRIC_ID_SSM_MAP_REL     0x2010  /**< Manifold relative press  [kPa]*/
#define METRIC_ID_SSM_TANK_PRESS  0x2011  /**< Fuel tank pressure       [kPa]*/
#define METRIC_ID_SSM_LEARNED_IGN 0x2012  /**< Learned ignition timing  [°]  */
#define METRIC_ID_SSM_ACCEL_ANGLE 0x2013  /**< Accelerator opening      [%]  */
#define METRIC_ID_SSM_FUEL_TEMP   0x2014  /**< Fuel temperature         [°C] */
#define METRIC_ID_SSM_WGDC2       0x2015  /**< Secondary wastegate duty [%]  */
#define METRIC_ID_SSM_CPC_DUTY    0x2016  /**< CPC valve duty           [%]  */
#define METRIC_ID_SSM_ISC_DUTY    0x2017  /**< Idle speed ctrl duty     [%]  */
#define METRIC_ID_SSM_AF_LEAN     0x2018  /**< A/F lean correction      [%]  */
#define METRIC_ID_SSM_AF_HEATER   0x2019  /**< A/F heater duty          [%]  */
#define METRIC_ID_SSM_ISC_STEP    0x201A  /**< Idle speed ctrl step     [-]  */
#define METRIC_ID_SSM_EGR_STEP    0x201B  /**< EGR steps                [-]  */
#define METRIC_ID_SSM_ALT_DUTY    0x201C  /**< Alternator duty          [%]  */
#define METRIC_ID_SSM_FUELPUMP    0x201D  /**< Fuel pump duty           [%]  */
#define METRIC_ID_SSM_IVVT_R      0x201E  /**< Intake VVT advance R     [°]  */
#define METRIC_ID_SSM_IVVT_L      0x201F  /**< Intake VVT advance L     [°]  */
#define METRIC_ID_SSM_IOCV_R      0x2020  /**< Intake OCV duty R        [%]  */
#define METRIC_ID_SSM_IOCV_L      0x2021  /**< Intake OCV duty L        [%]  */
#define METRIC_ID_SSM_AF_CURRENT  0x2022  /**< A/F sensor #1 current    [mA] */
#define METRIC_ID_SSM_LAMBDA      0x2023  /**< A/F sensor #1            [λ]  */
#define METRIC_ID_SSM_LAMBDA2     0x2024  /**< A/F sensor #2            [λ]  */
#define METRIC_ID_SSM_TMOTOR_DUTY 0x2025  /**< Throttle motor duty      [%]  */
#define METRIC_ID_SSM_TPS_MAIN    0x2026  /**< Main throttle sensor     [V]  */
#define METRIC_ID_SSM_APS_MAIN    0x2027  /**< Main accelerator sensor  [V]  */
#define METRIC_ID_SSM_BRAKE_BOOST 0x2028  /**< Brake booster pressure   [kPa]*/
#define METRIC_ID_SSM_FUEL_HP     0x2029  /**< Fuel pressure (high)     [MPa]*/
#define METRIC_ID_SSM_EVVT_R      0x202B  /**< Exhaust VVT advance R    [°]  */
#define METRIC_ID_SSM_EVVT_L      0x202C  /**< Exhaust VVT advance L    [°]  */
#define METRIC_ID_SSM_ROUGH_C1    0x202D  /**< Roughness monitor cyl 1  [-]  */
#define METRIC_ID_SSM_ROUGH_C2    0x202E  /**< Roughness monitor cyl 2  [-]  */
#define METRIC_ID_SSM_ROUGH_C3    0x202F  /**< Roughness monitor cyl 3  [-]  */
#define METRIC_ID_SSM_ROUGH_C4    0x2030  /**< Roughness monitor cyl 4  [-]  */
#define METRIC_ID_SSM_INJ_PW2     0x2031  /**< Injector #2 pulse width  [ms] */
#define METRIC_ID_SSM_COLD_INJ    0x2032  /**< Cold start injector      [ms] */
#define METRIC_ID_SSM_ALT_MODE    0x2033  /**< Alternator control mode
                                               0=High 1=ExHigh 2=Low 3=Mid   */
#define METRIC_ID_SSM_FUEL_LVL_V  0x2034  /**< Fuel level sender        [V]  */
#define METRIC_ID_SSM_RAD_FAN     0x2035  /**< Radiator fan control     [%]  */
#define METRIC_ID_SSM_LEARNED_COR 0x2036  /**< Learned ignition corr.   [°]  */
#define METRIC_ID_SSM_BOOST_FB    0x2037  /**< Boost pressure feedback  [%]  */
#define METRIC_ID_SSM_TARGET_RPM  0x2038  /**< Target engine speed      [rpm]*/
#define METRIC_ID_SSM_SI_DRIVE    0x2039  /**< SI-Drive mode 1=S 2=S# 3=I   */
#define METRIC_ID_SSM_ODOMETER    0x203A  /**< Odometer                 [km] */
#define METRIC_ID_SSM_EPS_CURRENT 0x203B  /**< Power steering current   [A]  */
#define METRIC_ID_SSM_FUELPUMP_A  0x203C  /**< Fuel pump current        [A]  */
#define METRIC_ID_SSM_INJ_PW1     0x203E  /**< Injector #1 pulse width  [ms] */

/* ---- Subaru SSM2 switches (0x21xx) — 0 or 1 ----------------------------
 * The ECU's own view of its inputs and outputs. These come from the switch
 * bytes at 0x61-0x69 and 0x120-0x121, one bit each, and cost almost nothing
 * to read alongside the measurements.
 */
#define METRIC_ID_SW_BRAKE        0x2101  /**< Brake pedal switch             */
#define METRIC_ID_SW_CLUTCH       0x2102  /**< Clutch pedal switch            */
#define METRIC_ID_SW_NEUTRAL      0x2103  /**< Neutral position switch        */
#define METRIC_ID_SW_AC           0x2104  /**< A/C switch                     */
#define METRIC_ID_SW_IDLE         0x2105  /**< Idle (closed throttle) switch  */
#define METRIC_ID_SW_KNOCK        0x2106  /**< Knock signal (cyl group 1|2)   */
#define METRIC_ID_SW_ELEC_LOAD    0x2107  /**< Electrical load signal         */
#define METRIC_ID_SW_LIGHTS       0x2108  /**< Light switch                   */
#define METRIC_ID_SW_RAD_FAN1     0x2109  /**< Radiator fan relay #1          */
#define METRIC_ID_SW_RAD_FAN2     0x210A  /**< Radiator fan relay #2          */
#define METRIC_ID_SW_AC_COMP      0x210B  /**< A/C compressor                 */
#define METRIC_ID_SW_FUEL_PUMP    0x210C  /**< Fuel pump relay                */
#define METRIC_ID_SW_OIL_PRESS    0x210D  /**< Engine oil pressure switch     */
#define METRIC_ID_SW_STARTER      0x210E  /**< Starter switch                 */
#define METRIC_ID_SW_DEFOGGER     0x210F  /**< Rear defogger switch           */
#define METRIC_ID_SW_BLOWER       0x2110  /**< Blower fan switch              */
#define METRIC_ID_SW_WIPER        0x2111  /**< Wiper switch                   */
#define METRIC_ID_SW_IGNITION     0x2112  /**< Ignition switch                */
#define METRIC_ID_SW_STOP_LIGHT   0x2113  /**< Stop light switch              */
#define METRIC_ID_SW_KNOCK2       0x2114  /**< Knock signal #2                */

#define METRIC_ID_NIGHT_SENSE     0x1F01  /**< Night-mode source (>0.5 = night)*/
/*
 * Master bus health, broadcast like any other metric.
 *
 * Without these the only way to know whether the CAN side is alive is a
 * laptop on the master's serial port — useless in a car, which is exactly
 * where the question gets asked. Put them on a display and "is the bus
 * connected?" is answerable from the driver's seat.
 */
#define METRIC_ID_MASTER_CAN_RX   0x1F03  /**< CAN frames per second  [1/s]  */
#define METRIC_ID_MASTER_CAN_IDS  0x1F04  /**< Distinct CAN IDs seen   [-]   */
#define METRIC_ID_MASTER_OBD_RX   0x1F05  /**< OBD replies per second [1/s]  */
#define METRIC_ID_MASTER_SSM_RX   0x1F06  /**< SSM2 exchanges per sec [1/s]  */

#define METRIC_ID_MASTER_UPTIME   0x1F02  /**< Master uptime            [s]   */
/** @} */

/* ────────────────────────────── Wire structures ─────────────────────────── */

#pragma pack(push, 1)

/**
 * @brief One parsed telemetry channel inside a broadcast frame. 7 bytes.
 */
typedef struct {
    uint16_t metric_id;  /**< Standardized ID — see @ref MetricIDs
                              (e.g. 0x010C = RPM, 0x0105 = Coolant Temp).     */
    float    value;      /**< Parsed, scaled real-world value in the unit
                              mandated by the metric ID registry (IEEE-754
                              single, little-endian on the wire).             */
    uint8_t  flags;      /**< Bit 0: Valid · Bit 1: Warning · Bit 2: Critical
                              · Bit 3: Night Mode — see METRIC_FLAG_*.        */
} MetricEntry;

/** Maximum metric entries per frame (28 × 7 B + 9 B header = 205 B ≤ 250 B). */
#define TELEMETRY_MAX_METRICS  28

/**
 * @brief Complete ESP-NOW broadcast frame emitted by the Master.
 *
 * The Master transmits only the used portion of @ref metrics — the wire
 * length is TELEMETRY_PACKET_SIZE(metric_count), not sizeof(). Slaves must
 * validate the received length against metric_count before parsing.
 */
typedef struct {
    uint32_t    sequence_id;    /**< Monotonic packet counter for drop/jitter
                                     detection on the receiving side.         */
    uint32_t    timestamp_ms;   /**< Master uptime in milliseconds when the
                                     frame was packed.                        */
    uint8_t     metric_count;   /**< Number of populated entries (0–28).      */
    MetricEntry metrics[TELEMETRY_MAX_METRICS]; /**< Active metric channels.  */
} MasterTelemetryPacket;

#pragma pack(pop)

/* ───────────────────────── Compile-time invariants ──────────────────────── */

/** Header bytes preceding the metrics array on the wire. */
#define TELEMETRY_HEADER_SIZE  (sizeof(uint32_t) * 2 + sizeof(uint8_t))

/** Exact wire size of a frame carrying @p n metrics. */
#define TELEMETRY_PACKET_SIZE(n)  (TELEMETRY_HEADER_SIZE + (n) * sizeof(MetricEntry))

static_assert(sizeof(MetricEntry) == 7,
              "MetricEntry must pack to exactly 7 bytes");
static_assert(TELEMETRY_HEADER_SIZE == 9,
              "Packet header must pack to exactly 9 bytes");
static_assert(sizeof(MasterTelemetryPacket) <= 250,
              "ESP-NOW payload hard limit (250 bytes) exceeded");

/**
 * @brief Validate a raw received buffer as a well-formed telemetry frame.
 * @param len Received byte count from the ESP-NOW callback.
 * @param pkt Buffer cast to MasterTelemetryPacket (must hold ≥ len bytes).
 * @return true when the length is consistent with metric_count.
 */
static inline bool telemetry_packet_valid(const MasterTelemetryPacket *pkt, int len) {
    if (len < (int)TELEMETRY_HEADER_SIZE)             return false;
    if (pkt->metric_count > TELEMETRY_MAX_METRICS)    return false;
    return len >= (int)TELEMETRY_PACKET_SIZE(pkt->metric_count);
}
