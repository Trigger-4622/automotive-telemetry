/**
 * @file DtcText.h
 * @brief Trouble codes: the packed METRIC_ID_DTC_n value, the code as text
 *        ("P1718") and what it means.
 *
 * Identical in the master and both screens (scripts/check_sync.py). The table
 * is `static`: include this from ONE source file per project, or each one that
 * includes it carries its own copy.
 *
 * The names are short on purpose - they have to fit one line of the screens'
 * alert card. Codes not listed are named by the part of the car the code's
 * number belongs to (SAE J2012), so nothing reads as blank.
 */
#pragma once
#include <stdint.h>
#include <stdio.h>

/** @brief Which list a code came from. */
enum DtcKind : uint8_t {
    DTC_STORED    = 0,   /**< Service 03: confirmed, may light the lamp.     */
    DTC_PENDING   = 1,   /**< Service 07: seen this drive, not confirmed.     */
    DTC_PERMANENT = 2    /**< Service 0A: clears only once the car agrees.    */
};

/** @brief Which computer reported it. */
enum DtcModule : uint8_t {
    DTC_ENGINE       = 0,   /**< ECM, 0x7E0 / 0x7E8.  */
    DTC_TRANSMISSION = 1    /**< TCM, 0x7E1 / 0x7E9.  */
};

static const char *const DTC_KIND_NAME[]   = { "stored", "pending", "permanent" };
static const char *const DTC_MODULE_NAME[] = { "engine", "transmission" };

/** @brief A DTC_n channel's value: raw | kind << 16 | module << 18. */
static inline float dtcPack(uint16_t raw, uint8_t kind, uint8_t module) {
    return (float)((uint32_t)raw | (uint32_t)(kind & 3) << 16 | (uint32_t)(module & 3) << 18);
}

/** @brief The parts of a DTC_n channel's value. */
static inline void dtcUnpack(float v, uint16_t &raw, uint8_t &kind, uint8_t &module) {
    const uint32_t u = v > 0 ? (uint32_t)(v + 0.5f) : 0;
    raw    = (uint16_t)(u & 0xFFFF);
    kind   = (uint8_t)((u >> 16) & 3);
    module = (uint8_t)((u >> 18) & 3);
}

/** @brief The code as printed in a manual: 0x1718 -> "P1718", 0xC100 -> "U0100". */
static inline void dtcCode(uint16_t raw, char out[6]) {
    static const char LETTER[] = "PCBU";
    snprintf(out, 6, "%c%X%X%X%X", LETTER[raw >> 14], (unsigned)((raw >> 12) & 3),
             (unsigned)((raw >> 8) & 15), (unsigned)((raw >> 4) & 15), (unsigned)(raw & 15));
}

struct DtcName { uint16_t raw; const char *text; };

/** Common codes, sorted by raw value (P, then U). SAE J2012 wording, shortened. */
static const DtcName DTC_NAMES[] = {
    {0x0010, "Intake cam actuator circuit B1"},   {0x0011, "Intake cam timing over-advanced B1"},
    {0x0012, "Intake cam timing over-retarded B1"},{0x0014, "Exhaust cam timing over-advanced B1"},
    {0x0016, "Crank/cam position correlation"},   {0x0020, "Intake cam actuator circuit B2"},
    {0x0021, "Intake cam timing over-advanced B2"},{0x0022, "Intake cam timing over-retarded B2"},
    {0x0030, "O2 heater control circuit B1S1"},   {0x0031, "O2 heater circuit low B1S1"},
    {0x0032, "O2 heater circuit high B1S1"},      {0x0037, "O2 heater circuit low B1S2"},
    {0x0038, "O2 heater circuit high B1S2"},      {0x0068, "MAP/MAF vs throttle correlation"},
    {0x0100, "Mass air flow circuit"},            {0x0101, "Mass air flow range/performance"},
    {0x0102, "Mass air flow circuit low"},        {0x0103, "Mass air flow circuit high"},
    {0x0105, "MAP/baro circuit"},                 {0x0106, "MAP/baro range/performance"},
    {0x0107, "MAP/baro circuit low"},             {0x0108, "MAP/baro circuit high"},
    {0x0110, "Intake air temp circuit"},          {0x0111, "Intake air temp range/perf."},
    {0x0112, "Intake air temp circuit low"},      {0x0113, "Intake air temp circuit high"},
    {0x0115, "Coolant temp circuit"},             {0x0116, "Coolant temp range/performance"},
    {0x0117, "Coolant temp circuit low"},         {0x0118, "Coolant temp circuit high"},
    {0x0120, "Throttle sensor A circuit"},        {0x0121, "Throttle sensor A range/perf."},
    {0x0122, "Throttle sensor A circuit low"},    {0x0123, "Throttle sensor A circuit high"},
    {0x0125, "Coolant too cold for closed loop"}, {0x0128, "Thermostat: engine stays cold"},
    {0x0130, "O2 sensor circuit B1S1"},           {0x0131, "O2 sensor low voltage B1S1"},
    {0x0132, "O2 sensor high voltage B1S1"},      {0x0133, "O2 sensor slow response B1S1"},
    {0x0134, "O2 sensor no activity B1S1"},       {0x0135, "O2 sensor heater circuit B1S1"},
    {0x0136, "O2 sensor circuit B1S2"},           {0x0137, "O2 sensor low voltage B1S2"},
    {0x0138, "O2 sensor high voltage B1S2"},      {0x0139, "O2 sensor slow response B1S2"},
    {0x0140, "O2 sensor no activity B1S2"},       {0x0141, "O2 sensor heater circuit B1S2"},
    {0x0171, "System too lean B1"},               {0x0172, "System too rich B1"},
    {0x0174, "System too lean B2"},               {0x0175, "System too rich B2"},
    {0x0181, "Fuel temp sensor range/perf."},     {0x0182, "Fuel temp sensor circuit low"},
    {0x0183, "Fuel temp sensor circuit high"},    {0x0201, "Injector circuit cylinder 1"},
    {0x0202, "Injector circuit cylinder 2"},      {0x0203, "Injector circuit cylinder 3"},
    {0x0204, "Injector circuit cylinder 4"},      {0x0222, "Throttle sensor B circuit low"},
    {0x0223, "Throttle sensor B circuit high"},   {0x0230, "Fuel pump primary circuit"},
    {0x0234, "Turbo overboost"},                  {0x0244, "Wastegate solenoid range/perf."},
    {0x0245, "Wastegate solenoid circuit low"},   {0x0246, "Wastegate solenoid circuit high"},
    {0x0300, "Random/multiple misfire"},          {0x0301, "Cylinder 1 misfire"},
    {0x0302, "Cylinder 2 misfire"},               {0x0303, "Cylinder 3 misfire"},
    {0x0304, "Cylinder 4 misfire"},               {0x0327, "Knock sensor circuit low"},
    {0x0328, "Knock sensor circuit high"},        {0x0335, "Crankshaft sensor circuit"},
    {0x0336, "Crankshaft sensor range/perf."},    {0x0340, "Camshaft sensor circuit B1"},
    {0x0341, "Camshaft sensor range/perf. B1"},   {0x0345, "Camshaft sensor circuit B2"},
    {0x0400, "EGR flow"},                         {0x0420, "Catalyst efficiency low B1"},
    {0x0430, "Catalyst efficiency low B2"},       {0x0440, "EVAP system"},
    {0x0441, "EVAP incorrect purge flow"},        {0x0442, "EVAP small leak"},
    {0x0447, "EVAP vent control circuit open"},   {0x0448, "EVAP vent control circuit short"},
    {0x0451, "EVAP pressure sensor range/perf."}, {0x0452, "EVAP pressure sensor low"},
    {0x0453, "EVAP pressure sensor high"},        {0x0455, "EVAP large leak"},
    {0x0456, "EVAP very small leak"},             {0x0457, "EVAP leak: fuel cap loose/off"},
    {0x0458, "EVAP purge valve circuit low"},     {0x0459, "EVAP purge valve circuit high"},
    {0x0461, "Fuel level sensor range/perf."},    {0x0462, "Fuel level sensor circuit low"},
    {0x0463, "Fuel level sensor circuit high"},   {0x0464, "Fuel level sensor intermittent"},
    {0x0480, "Cooling fan 1 control circuit"},    {0x0483, "Cooling fan rationality check"},
    {0x0500, "Vehicle speed sensor"},             {0x0502, "Vehicle speed sensor low"},
    {0x0503, "Vehicle speed sensor erratic"},     {0x0506, "Idle speed lower than expected"},
    {0x0507, "Idle speed higher than expected"},  {0x0512, "Starter request circuit"},
    {0x0513, "Invalid immobilizer key"},          {0x0519, "Idle control performance"},
    {0x0545, "Exhaust temp sensor circuit low"},  {0x0546, "Exhaust temp sensor circuit high"},
    {0x0560, "System voltage"},                   {0x0562, "System voltage low"},
    {0x0563, "System voltage high"},              {0x0600, "Serial communication link"},
    {0x0601, "Computer memory checksum"},         {0x0602, "Computer programming error"},
    {0x0604, "Computer RAM error"},               {0x0605, "Computer ROM error"},
    {0x0606, "Computer processor fault"},         {0x0607, "Computer performance"},
    {0x0638, "Throttle actuator range/perf."},    {0x0700, "Transmission asks for the lamp"},
    {0x0705, "Transmission range sensor circuit"},{0x0706, "Range sensor range/perf."},
    {0x0710, "ATF temp sensor circuit"},          {0x0711, "ATF temp sensor range/perf."},
    {0x0712, "ATF temp sensor circuit low"},      {0x0713, "ATF temp sensor circuit high"},
    {0x0715, "Turbine speed sensor circuit"},     {0x0720, "Output speed sensor circuit"},
    {0x0725, "Engine speed input circuit"},       {0x0731, "Gear 1 incorrect ratio"},
    {0x0732, "Gear 2 incorrect ratio"},           {0x0733, "Gear 3 incorrect ratio"},
    {0x0734, "Gear 4 incorrect ratio"},           {0x0735, "Gear 5 incorrect ratio"},
    {0x0740, "Torque converter clutch circuit"},  {0x0741, "Torque conv. clutch stuck off"},
    {0x0743, "Torque conv. clutch electrical"},   {0x0748, "Pressure solenoid A electrical"},
    {0x0750, "Shift solenoid A"},                 {0x0755, "Shift solenoid B"},
    {0x0760, "Shift solenoid C"},                 {0x0765, "Shift solenoid D"},
    {0x0770, "Shift solenoid E"},                 {0x0817, "Starter disable circuit"},
    {0x0851, "Park/neutral switch input low"},    {0x0852, "Park/neutral switch input high"},
    {0x1718, "AT CAN communication circuit"},
    {0x2004, "Intake runner stuck open B1"},      {0x2006, "Intake runner stuck closed B1"},
    {0x2088, "Cam actuator control low B1"},      {0x2089, "Cam actuator control high B1"},
    {0x2096, "Post-cat fuel trim too lean B1"},   {0x2097, "Post-cat fuel trim too rich B1"},
    {0x2101, "Throttle motor range/perf."},       {0x2109, "Throttle/pedal min stop perf."},
    {0x2135, "Throttle sensor A/B correlation"},  {0x2138, "Pedal sensor D/E correlation"},
    {0x2227, "Baro sensor range/performance"},    {0x2228, "Baro sensor circuit low"},
    {0x2229, "Baro sensor circuit high"},         {0x2401, "EVAP leak pump control low"},
    {0x2402, "EVAP leak pump control high"},      {0x2419, "EVAP switching valve low"},
    {0x2420, "EVAP switching valve high"},
    {0xC001, "High speed CAN bus"},               {0xC073, "CAN bus A off"},
    {0xC100, "Lost communication with ECM"},      {0xC101, "Lost communication with TCM"},
    {0xC121, "Lost communication with ABS"},      {0xC140, "Lost communication with body"},
    {0xC155, "Lost communication with cluster"},
};

/**
 * @brief What a code means: its own name when listed, else the part of the
 *        car its number belongs to.
 */
static inline const char *dtcText(uint16_t raw) {
    int lo = 0, hi = (int)(sizeof(DTC_NAMES) / sizeof(DTC_NAMES[0])) - 1;
    while (lo <= hi) {
        const int mid = (lo + hi) / 2;
        if (DTC_NAMES[mid].raw == raw) return DTC_NAMES[mid].text;
        if (DTC_NAMES[mid].raw < raw) lo = mid + 1; else hi = mid - 1;
    }
    const uint8_t letter = raw >> 14, d1 = (raw >> 12) & 3, d2 = (raw >> 8) & 15;
    switch (letter) {
        case 1:  return "Chassis code";
        case 2:  return "Body code";
        case 3:  return d1 == 0 ? "Network communication" : "Maker's network code";
        default: break;
    }
    if (d1 == 1 || d1 == 3) return "Maker's own powertrain code";
    switch (d2) {
        case 0: case 1: case 2: return "Fuel and air metering";
        case 3:  return "Ignition or misfire";
        case 4:  return "Emission controls";
        case 5:  return "Speed, idle or inputs";
        case 6:  return "Computer or its outputs";
        case 7: case 8: case 9: return "Transmission";
        case 10: return "Hybrid system";
        default: return "Powertrain code";
    }
}
