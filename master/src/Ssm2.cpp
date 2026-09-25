/**
 * @file Ssm2.cpp
 * @brief Subaru SSM2 over CAN — rewritten after FreeSSM and RomRaider.
 *
 * ## What the reference implementations do, and this now does too
 *
 * - Requests go to 0x7E0 and replies come from request + 8 = 0x7E8
 *   (FreeSSM SSMprotocol2: CU address 0x7E0; receiveReplyISO15765 checks
 *   msgaddr == ecuaddr + 8).
 * - Init: `AA` → `EA` + SYS ID (3) + ROM ID (5) + 32/48/96 capability bytes;
 *   FreeSSM accepts total lengths 41, 57 and 105 only.
 * - Read: `A8 00 <addr24>…` → `E8 <one byte per address>`, reply length
 *   exactly 1 + addresses (FreeSSM ReadMultipleDatabytes).
 * - At most 33 addresses per request: FreeSSM's max_bytes_per_multiread,
 *   with the note that control units have limits well below the protocol's.
 * - Frames padded to 8 bytes with 0x00, as J2534's ISO15765_FRAME_PAD does
 *   for both tools - and as this ECU pads its own replies.
 * - Our flow control asks for BS 0 / STmin 0 (the J2534 defaults); the
 *   ECU's STmin is honoured on the way out.
 * - A reply may take up to a second (ELM327 ATSTFA in FreeSSM); a failed
 *   exchange is retried a few times before the ECU is considered gone.
 * - Negative response `7F <svc> <code>`: 0x78 means "still working, wait".
 *
 * ## What it adds for sharing a live vehicle bus
 *
 * The desktop tools own the bus while they log. This does not, so:
 * - Paced: a minimum gap between exchanges (default 100 ms, 10/s). SSM2 now
 *   only carries what OBD-II and the broadcast frames cannot, so that is
 *   plenty, and it leaves the ECU's own CAN traffic undisturbed.
 * - Our consecutive frames are spaced at least 1 ms apart even when the ECU
 *   would accept them back to back, so a request is never a solid burst.
 * - Stray frames - a late OBD-II reply, anything for another service - are
 *   ignored instead of failing the exchange.
 * - An address the ECU refuses is found by bisecting the refused request and
 *   then dropped, instead of failing every request that contains it.
 * - Nothing is transmitted while the bus guard (main.cpp) has requests
 *   paused after seeing bus errors during our transmissions.
 */
#include "Ssm2.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <vector>

#include "MasterConfig.h"
#include "MasterPacket.h"
#include "MasterTelemetry.h"

/* ─────────────────────────────── protocol ────────────────────────────────── */

static constexpr uint8_t PCI_SF = 0x00;  /**< Single frame.      */
static constexpr uint8_t PCI_FF = 0x10;  /**< First frame.       */
static constexpr uint8_t PCI_CF = 0x20;  /**< Consecutive frame. */
static constexpr uint8_t PCI_FC = 0x30;  /**< Flow control.      */

static constexpr uint8_t SSM_CMD_INIT = 0xAA;
static constexpr uint8_t SSM_RSP_INIT = 0xEA;
static constexpr uint8_t SSM_CMD_READ = 0xA8;
static constexpr uint8_t SSM_RSP_READ = 0xE8;
static constexpr uint8_t SSM_NEGATIVE = 0x7F;
static constexpr uint8_t NRC_PENDING  = 0x78;

/** FreeSSM's per-request ceiling; the adaptive size never exceeds it. */
static constexpr size_t  MAX_ADDR   = 33;
/** ...and the floor: a two-byte value is two addresses. */
static constexpr size_t  MIN_ADDR   = 2;
/** Reply wait is Cfg.ssmTimeoutMs (FreeSSM/ELM: 1000 ms); after a "pending"
 *  negative response the ECU gets this long. */
static constexpr uint32_t T_PENDING_MS = 3000;
/** ISO 15765-2 N_Bs (wait for flow control) and N_Cr (between our CFs'
 *  replies), kept below the standard 1000 ms so a dead ECU is noticed fast. */
static constexpr uint32_t T_FC_MS = 500;
static constexpr uint32_t T_CF_MS = 250;
/** Consecutive failures before the ECU is considered gone and re-probed. */
static constexpr uint8_t  FAILS_REINIT = 6;

/* ─────────────────────────── switch definitions ──────────────────────────
 *
 * The ECU's switch inputs and relay outputs, one bit each in the bytes at
 * 0x61-0x69 and 0x120-0x121. The capability byte for a switch byte is fixed
 * (0x62 ↔ byte 13, 0x63 ↔ 14, …, 0x121 ↔ 38) and the bit position in the
 * data byte is the same as the bit position of its support flag - which is
 * how RomRaider and FreeSSM both describe them.
 */
struct SwDef { uint32_t addr; uint8_t capByte; uint8_t bit; uint16_t metric; };

static const SwDef SW_TABLE[] = {
    { 0x000062, 13, 2, METRIC_ID_SW_AC        },
    { 0x000062, 13, 4, METRIC_ID_SW_IGNITION  },
    { 0x000062, 13, 7, METRIC_ID_SW_IDLE      },
    { 0x000062, 13, 8, METRIC_ID_SW_NEUTRAL   },
    { 0x000063, 14, 1, METRIC_ID_SW_ELEC_LOAD },
    { 0x000063, 14, 2, METRIC_ID_SW_KNOCK2    },
    { 0x000063, 14, 3, METRIC_ID_SW_KNOCK     },
    { 0x000063, 14, 7, METRIC_ID_SW_STARTER   },
    { 0x000064, 15, 3, METRIC_ID_SW_WIPER     },
    { 0x000064, 15, 4, METRIC_ID_SW_LIGHTS    },
    { 0x000064, 15, 5, METRIC_ID_SW_BLOWER    },
    { 0x000064, 15, 6, METRIC_ID_SW_DEFOGGER  },
    { 0x000065, 16, 4, METRIC_ID_SW_FUEL_PUMP },
    { 0x000065, 16, 5, METRIC_ID_SW_RAD_FAN2  },
    { 0x000065, 16, 6, METRIC_ID_SW_RAD_FAN1  },
    { 0x000065, 16, 8, METRIC_ID_SW_AC_COMP   },
    { 0x000068, 19, 7, METRIC_ID_SW_OIL_PRESS },
    { 0x000121, 38, 4, METRIC_ID_SW_BRAKE     },
    { 0x000121, 38, 7, METRIC_ID_SW_STOP_LIGHT},
    { 0x000121, 38, 8, METRIC_ID_SW_CLUTCH    },
};
static constexpr size_t SW_COUNT = sizeof(SW_TABLE) / sizeof(SW_TABLE[0]);
static constexpr uint16_t SW_PERIOD_MS = 200;
static uint8_t s_swRefused = 0;           /**< Bit per distinct switch byte. */

/* ───────────────────────────────── state ─────────────────────────────────── */

static QueueHandle_t s_rxQ = nullptr;     /**< 0x7E8 frames during an exchange. */
static volatile bool s_inExchange = false;
static volatile bool s_active     = false;
static volatile bool s_reinit     = false;

static volatile uint32_t s_responses = 0, s_errors = 0, s_nrcs = 0, s_stray = 0;
static volatile uint8_t  s_lastNrc   = 0;
static volatile uint32_t s_lastOkMs  = 0;
static volatile float    s_exchPerSec = 0;
static char              s_lastErr[40] = "";

static bool     s_initOk = false;
static uint32_t s_initAtMs = 0;
static uint8_t  s_initFails = 0;
static uint32_t s_lastInitTry = 0;
static uint8_t  s_sysId[3] = {0}, s_romId[5] = {0};
static uint8_t  s_flags[160] = {0};
static uint8_t  s_flagCount = 0;

static uint8_t  s_batch = MAX_ADDR;       /**< Addresses per request now.    */
/** Largest request the ECU has not objected to by size; growth stops there. */
static uint8_t  s_batchCeil = MAX_ADDR;
static uint8_t  s_okStreak = 0, s_failStreak = 0;

/** Local copy of the parameter table plus scheduling state. */
static std::vector<RtSsm>    s_table;
static std::vector<uint32_t> s_lastPoll;
static std::vector<uint8_t>  s_refused;   /**< ECU rejected this address.    */
static uint32_t              s_tableGen = 0xFFFFFFFF;
static size_t                s_cursor = 0;
static uint32_t              s_swPoll[6] = {0};  /**< Per switch byte (SW_BYTES). */

/*
 * Tracking down an address the ECU refuses.
 *
 * A negative response refuses the whole request, not just the one address it
 * objects to. So the slots of a refused request become suspects and are
 * bisected: half of them are asked on their own; a refusal narrows the search
 * to that half, an answer clears it. A suspect refused on its own is the
 * culprit and is dropped until the next init. That takes a handful of
 * exchanges among 33, and every half that is answered still delivers its
 * values. Suspects are table indices, or ~index for a switch byte.
 */
static std::vector<int16_t>  s_suspects;
static uint8_t               s_suspectFrom = 0;  /**< Addresses in the refused request. */
static const uint32_t SW_BYTES[] = { 0x62, 0x63, 0x64, 0x65, 0x68, 0x121 };

uint32_t ssm2Responses() { return s_responses; }
uint32_t ssm2Errors()    { return s_errors; }
void     ssm2Reinit()    { s_reinit = true; }

static void setErr(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    vsnprintf(s_lastErr, sizeof(s_lastErr), fmt, ap);
    va_end(ap);
}

static void hexStr(const uint8_t *b, size_t n, char *out, size_t cap) {
    size_t w = 0;
    for (size_t i = 0; i < n && w + 3 <= cap; i++)
        w += snprintf(out + w, cap - w, "%02X", b[i]);
    if (cap) out[w < cap ? w : cap - 1] = '\0';
}

int ssm2Supported(const RtSsm &e) {
    // No flag known (or a bit number no flag can have): ask and see.
    if (!s_initOk || e.capByte == 0 || e.capBit < 1 || e.capBit > 8) return -1;
    if (e.capByte > s_flagCount) return 0;
    return (s_flags[e.capByte - 1] >> (e.capBit - 1)) & 1;
}

static int swByteIndex(uint32_t addr) {
    for (int i = 0; i < 6; i++) if (SW_BYTES[i] == addr) return i;
    return -1;
}

static bool swSupported(const SwDef &d) {
    if (!s_initOk || d.capByte > s_flagCount) return false;
    const int bi = swByteIndex(d.addr);
    if (bi >= 0 && (s_swRefused >> bi) & 1) return false;
    return (s_flags[d.capByte - 1] >> (d.bit - 1)) & 1;
}

void ssm2GetStatus(Ssm2Status &out) {
    out.initOk     = s_initOk;
    out.active     = s_active;
    out.initAtMs   = s_initAtMs;
    hexStr(s_sysId, 3, out.sysId, sizeof(out.sysId));
    hexStr(s_romId, 5, out.ecuId, sizeof(out.ecuId));
    out.flagCount  = s_flagCount;
    out.responses  = s_responses;
    out.errors     = s_errors;
    out.nrcs       = s_nrcs;
    out.lastNrc    = s_lastNrc;
    out.batch      = s_batch;
    out.exchPerSec = s_exchPerSec;
    out.lastOkMs   = s_lastOkMs;
    strlcpy(out.lastErr, s_lastErr, sizeof(out.lastErr));
    uint16_t sup = 0, tot = 0, ref = 0;
    Cfg.lock();
    for (size_t i = 0; i < s_table.size(); i++) {
        if (i < s_refused.size() && s_refused[i]) ref++;
        if (!s_table[i].capByte) continue;
        tot++;
        if (ssm2Supported(s_table[i]) == 1) sup++;
    }
    Cfg.unlock();
    out.supported = sup;
    out.total     = tot;
    out.refused   = ref;
}

/* ─────────────────────────────── transport ───────────────────────────────── */

bool ssm2FeedFrame(const twai_message_t &msg) {
    if (msg.identifier != SSM_RESPONSE_ID || msg.extd) return false;
    // Outside an exchange a 0x7E8 frame is an OBD-II reply, not ours.
    if (!s_inExchange || !s_rxQ) return false;
    xQueueSend(s_rxQ, &msg, 0);   // never stall the receive task
    return true;
}

static void flushRx() {
    twai_message_t junk;
    while (s_rxQ && xQueueReceive(s_rxQ, &junk, 0) == pdTRUE) {}
}

static bool rxFrame(twai_message_t &m, uint32_t timeoutMs) {
    return s_rxQ && xQueueReceive(s_rxQ, &m, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

/** @brief Why diagGuardOk() said no, for the portal. */
static const char *guardWhy() {
    return diagSettleLeftMs() ? "waiting for the bus to settle" : "paused by bus guard";
}

/** @brief One padded frame to the ECU. Refuses while the bus guard is up. */
static bool tx8(const uint8_t data[8]) {
    if (!diagGuardOk()) { setErr("%s", guardWhy()); return false; }
    twai_message_t m = {};
    m.identifier       = SSM_REQUEST_ID;
    m.data_length_code = 8;
    memcpy(m.data, data, 8);
    if (twai_transmit(&m, pdMS_TO_TICKS(50)) != ESP_OK) { setErr("transmit refused"); return false; }
    masterCountTx();
    return true;
}

/**
 * @brief Wait for the ECU's flow control. WAIT frames extend the wait (at
 *        most 10 of them); OVERFLOW or anything else aborts.
 */
static bool waitFlowControl(uint8_t &bs, uint8_t &stmin) {
    twai_message_t fc;
    for (uint8_t waits = 0;;) {
        if (!rxFrame(fc, T_FC_MS)) { setErr("no flow control from ECU"); return false; }
        if ((fc.data[0] & 0xF0) != PCI_FC) { s_stray++; continue; }
        const uint8_t fs = fc.data[0] & 0x0F;
        if (fs == 0) { bs = fc.data[1]; stmin = fc.data[2]; return true; }
        if (fs == 1 && ++waits <= 10) continue;
        setErr("ECU flow control %s", fs == 2 ? "overflow" : "abort");
        return false;
    }
}

/**
 * @brief The ECU's STmin as microseconds: 0x00-0x7F are milliseconds,
 *        0xF1-0xF9 are 100-900 us, and anything else is reserved, which
 *        ISO 15765-2 says to read as the longest, 127 ms.
 */
static uint32_t stminUs(uint8_t st) {
    if (st <= 0x7F) return (uint32_t)st * 1000;
    if (st >= 0xF1 && st <= 0xF9) return (uint32_t)(st - 0xF0) * 100;
    return 127000;
}

/** @brief Send one ISO-TP message (single frame, or FF + CFs). */
static bool isoSend(const uint8_t *p, size_t len) {
    uint8_t f[8];
    if (len <= 7) {
        memset(f, 0x00, 8);
        f[0] = (uint8_t)len;
        memcpy(&f[1], p, len);
        return tx8(f);
    }
    memset(f, 0x00, 8);
    f[0] = (uint8_t)(PCI_FF | ((len >> 8) & 0x0F));
    f[1] = (uint8_t)(len & 0xFF);
    memcpy(&f[2], p, 6);
    if (!tx8(f)) return false;

    uint8_t bs, stmin;
    if (!waitFlowControl(bs, stmin)) return false;

    size_t off = 6;
    uint8_t seq = 1, inBlock = 0;
    while (off < len) {
        // At least 1 ms between our frames whatever the ECU allows: a request
        // should never be a solid burst on a shared bus.
        const uint32_t gapUs = max((uint32_t)1000, stminUs(stmin));
        if (gapUs >= 2000) vTaskDelay(pdMS_TO_TICKS(gapUs / 1000) + 1);
        else               delayMicroseconds(gapUs);

        memset(f, 0x00, 8);
        f[0] = (uint8_t)(PCI_CF | (seq & 0x0F));
        const size_t n = min((size_t)7, len - off);
        memcpy(&f[1], p + off, n);
        if (!tx8(f)) return false;
        off += n; seq++;

        if (bs && ++inBlock >= bs && off < len) {
            inBlock = 0;
            if (!waitFlowControl(bs, stmin)) return false;
        }
    }
    return true;
}

/**
 * @brief Receive the reply to @p svc (or a negative response), ignoring
 *        anything else that turns up on 0x7E8 in the meantime.
 * @return Bytes in @p out, 0 on timeout or a broken sequence.
 */
static size_t isoRecv(uint8_t svc, uint8_t *out, size_t max) {
    uint32_t deadline = millis() + Cfg.ssmTimeoutMs;
    twai_message_t m;
    for (;;) {
        const int32_t left = (int32_t)(deadline - millis());
        if (left <= 0 || !rxFrame(m, left)) { setErr("no reply from ECU"); return 0; }
        const uint8_t pci = m.data[0] & 0xF0;

        if (pci == PCI_SF) {
            const uint8_t len = m.data[0] & 0x0F;
            if (len < 1 || len > 7) { s_stray++; continue; }
            const uint8_t first = m.data[1];
            if (first == SSM_NEGATIVE && len >= 3 && m.data[2] == (uint8_t)(svc - 0x40)) {
                if (m.data[3] == NRC_PENDING) { deadline = millis() + T_PENDING_MS; continue; }
            } else if (first != svc) {
                s_stray++;           // a late OBD-II reply or similar: not ours
                continue;
            }
            const size_t n = min((size_t)len, max);
            memcpy(out, &m.data[1], n);
            return n;
        }

        if (pci != PCI_FF) { s_stray++; continue; }
        const size_t total = ((size_t)(m.data[0] & 0x0F) << 8) | m.data[1];
        if (m.data[2] != svc || total < 7) { s_stray++; continue; }
        if (total > max) {
            // Ours, but longer than it can be. Say so (flow control
            // "overflow") rather than leave the ECU waiting for a go-ahead.
            const uint8_t ovf[8] = { (uint8_t)(PCI_FC | 0x02), 0, 0, 0, 0, 0, 0, 0 };
            tx8(ovf);
            setErr("reply of %u bytes, at most %u expected", (unsigned)total, (unsigned)max);
            return 0;
        }
        memcpy(out, &m.data[2], 6);
        size_t got = 6;

        uint8_t fc[8] = { (uint8_t)(PCI_FC | 0x00), 0x00, 0x00, 0, 0, 0, 0, 0 };
        if (!tx8(fc)) return 0;

        uint8_t expect = 1;
        while (got < total) {
            if (!rxFrame(m, T_CF_MS)) { setErr("reply cut short"); return 0; }
            if ((m.data[0] & 0xF0) != PCI_CF) { s_stray++; continue; }
            if ((m.data[0] & 0x0F) != (expect & 0x0F)) { setErr("reply frames out of order"); return 0; }
            expect++;
            const size_t n = min((size_t)7, total - got);
            memcpy(out + got, &m.data[1], n);
            got += n;
        }
        return got;
    }
}

/** @brief Result of one request/reply. */
enum ExResult : uint8_t { EX_OK, EX_NRC, EX_FAIL, EX_BUSY };

/**
 * @brief One request/reply with the diagnostic channel to ourselves.
 * @param[out] got Reply length (valid for EX_OK and EX_NRC).
 */
static ExResult exchange(const uint8_t *req, size_t len, uint8_t svc,
                         uint8_t *rsp, size_t max, size_t &got) {
    got = 0;
    if (!diagGuardOk()) { setErr("%s", guardWhy()); return EX_BUSY; }
    if (!diagBusLock(250)) return EX_BUSY;
    s_inExchange = true;
    flushRx();
    if (isoSend(req, len)) got = isoRecv(svc, rsp, max);
    s_inExchange = false;
    diagBusUnlock();

    if (got >= 3 && rsp[0] == SSM_NEGATIVE) {
        s_nrcs++;
        s_lastNrc = rsp[2];
        setErr("ECU refused 0x%02X (code 0x%02X)", rsp[1], rsp[2]);
        return EX_NRC;
    }
    return got ? EX_OK : EX_FAIL;
}

/* ───────────────────────────────── init ──────────────────────────────────── */

static bool ssm2Init() {
    const uint8_t req[1] = { SSM_CMD_INIT };
    uint8_t rsp[1 + 3 + 5 + sizeof(s_flags)];
    size_t got;
    if (exchange(req, 1, SSM_RSP_INIT, rsp, sizeof(rsp), got) != EX_OK || got < 9) return false;
    if (got != 41 && got != 57 && got != 105)
        log_w("SSM2: init reply is %u bytes (FreeSSM expects 41/57/105) - using it anyway",
              (unsigned)got);

    memcpy(s_sysId, &rsp[1], 3);
    memcpy(s_romId, &rsp[4], 5);
    s_flagCount = (uint8_t)min(got - 9, sizeof(s_flags));
    memset(s_flags, 0, sizeof(s_flags));
    memcpy(s_flags, &rsp[9], s_flagCount);
    s_initOk = true;
    s_initAtMs = millis();
    s_swRefused = 0;
    Cfg.lock();                          // ssm2GetStatus reads it from the portal
    s_refused.assign(s_table.size(), 0);
    Cfg.unlock();
    s_suspects.clear();
    s_batchCeil = MAX_ADDR;
    s_batch = min((uint8_t)MAX_ADDR, Cfg.ssmBatchMax);

    char sys[8], rom[12], fl[sizeof(Cfg.ecuFlags)];
    hexStr(s_sysId, 3, sys, sizeof(sys));
    hexStr(s_romId, 5, rom, sizeof(rom));
    hexStr(s_flags, s_flagCount, fl, sizeof(fl));
    log_i("SSM2: ECU answered - SYS %s ROM %s, %u capability bytes", sys, rom, s_flagCount);

    if (strcmp(Cfg.ecuId, rom) || strcmp(Cfg.ecuSysId, sys) || strcmp(Cfg.ecuFlags, fl)) {
        strlcpy(Cfg.ecuId, rom, sizeof(Cfg.ecuId));
        strlcpy(Cfg.ecuSysId, sys, sizeof(Cfg.ecuSysId));
        strlcpy(Cfg.ecuFlags, fl, sizeof(Cfg.ecuFlags));
        Cfg.requestSave();
    }
    return true;
}

/* ─────────────────────────────── scheduling ──────────────────────────────── */

/**
 * @brief Re-copy the parameter table when the configuration changed.
 *
 * The generation also moves for changes that have nothing to do with SSM2
 * (every signal the learner proposes), and an edit in the portal can shift
 * every row. So refusals and poll times are carried over by address, not by
 * position: otherwise a refusal would land on the wrong parameter, and every
 * slow value would fall due at once after each unrelated change.
 */
static void refreshTable() {
    if (s_tableGen == Cfg.generation) return;
    Cfg.lock();                          // ssm2GetStatus reads these too
    const std::vector<RtSsm>    oldTable = s_table;
    const std::vector<uint32_t> oldPoll  = s_lastPoll;
    const std::vector<uint8_t>  oldRef   = s_refused;
    s_tableGen = Cfg.generation;
    s_table    = Cfg.ssm;
    s_lastPoll.assign(s_table.size(), 0);
    s_refused.assign(s_table.size(), 0);
    for (size_t i = 0; i < s_table.size(); i++)
        for (size_t j = 0; j < oldTable.size(); j++) {
            if (oldTable[j].address != s_table[i].address) continue;
            if (j < oldPoll.size()) s_lastPoll[i] = oldPoll[j];
            if (j < oldRef.size())  s_refused[i]  = oldRef[j];
            break;
        }
    Cfg.unlock();
    s_suspects.clear();
    if (s_cursor >= s_table.size()) s_cursor = 0;
}

/** @brief One address slot of a request and how to read its reply byte(s). */
struct Slot {
    uint16_t metricId;   /**< 0 for a switch byte.              */
    uint8_t  bytes;
    bool     isSigned;
    float    scale, offset;
    uint8_t  firstByte;  /**< Offset in the reply payload.       */
    uint32_t addr;       /**< Address (switch bytes).            */
    int16_t  tableIdx;   /**< Index into s_table, -1 = switch.   */
};

/** @brief Poll outcome. */
enum PollResult : uint8_t { P_IDLE, P_OK, P_FAIL, P_NRC, P_BUSY, P_REFUSED };

/** @brief Publish the values of an answered read. */
static void publishReply(const Slot *slots, size_t nSlot, const uint8_t *rsp, uint32_t now) {
    for (size_t i = 0; i < nSlot; i++) {
        const Slot &s = slots[i];
        if (s.tableIdx >= 0) s_lastPoll[s.tableIdx] = now;
        if (s.metricId == 0) {                       // a switch byte
            const int bi = swByteIndex(s.addr);
            if (bi >= 0) s_swPoll[bi] = now;
            const uint8_t b = rsp[1 + s.firstByte];
            for (const auto &d : SW_TABLE) {
                if (d.addr != s.addr || !swSupported(d)) continue;
                const float on = ((b >> (d.bit - 1)) & 1) ? 1.0f : 0.0f;
                publishMetric(d.metric, on, SRC_SSM);
                if (d.metric == METRIC_ID_SW_LIGHTS)
                    publishMetric(METRIC_ID_NIGHT_SENSE, on, SRC_SSM);
            }
            continue;
        }
        int32_t raw = rsp[1 + s.firstByte];
        if (s.bytes == 2) raw = (raw << 8) | rsp[1 + s.firstByte + 1];
        if (s.isSigned) {
            const int32_t signBit = s.bytes == 2 ? 0x8000 : 0x80;
            if (raw & signBit) raw -= signBit << 1;
        }
        publishMetric(s.metricId, raw * s.scale + s.offset, SRC_SSM);
    }
}

/** @brief Drop one address for good (until the next init). */
static void refuse(const Slot &s) {
    if (s.tableIdx >= 0) {
        s_refused[s.tableIdx] = 1;
        log_w("SSM2: ECU refuses address 0x%06X - dropped", (unsigned)s.addr);
    } else {
        const int bi = swByteIndex(s.addr);
        if (bi >= 0) s_swRefused |= (1 << bi);
        log_w("SSM2: ECU refuses switch byte 0x%06X - dropped", (unsigned)s.addr);
    }
}

/** @brief A candidate for the next request: table index (or ~switch byte)
 *         and how overdue it is, ms. */
struct Due { int16_t idx; int32_t late; };

/** @brief How far past its period a value is (negative: not due yet). The
 *         elapsed time is taken unsigned, so it stays right across the
 *         millis() wrap and for a value never read at all. */
static int32_t lateness(uint32_t now, uint32_t last, uint32_t period) {
    const uint32_t since = min(now - last, (uint32_t)0x3FFFFFFF);
    return (int32_t)since - (int32_t)period;
}

/**
 * @brief Build and run one read of whatever is due, most overdue first.
 *
 * Skipped without costing bus time: anything the ECU disowns or refused, and
 * anything OBD-II or a broadcast frame already delivers - SSM2 is the last
 * resort. While a refused request is being bisected, the suspects are asked
 * instead.
 */
static PollResult ssm2PollOnce() {
    Slot    slots[MAX_ADDR];
    uint8_t req[2 + MAX_ADDR * 3];
    size_t  nAddr = 0, nSlot = 0;
    const uint32_t now = millis();
    const bool bisecting = !s_suspects.empty();
    // The configured ceiling applies at once, not only after the next init.
    // Never below two addresses: a two-byte value (RPM…) must always fit, or
    // it would never be asked for again.
    const size_t cap = bisecting ? MAX_ADDR
                                 : max((size_t)MIN_ADDR, min((size_t)s_batch, (size_t)Cfg.ssmBatchMax));

    req[0] = SSM_CMD_READ;
    req[1] = 0x00;                       // "padaddr": single read, as FreeSSM

    auto add = [&](uint16_t metric, uint8_t bytes, bool sg, float sc, float of,
                   uint32_t addr, int16_t idx) -> bool {
        if (nAddr + bytes > cap || nSlot >= MAX_ADDR) return false;
        slots[nSlot++] = { metric, bytes, sg, sc, of, (uint8_t)nAddr, addr, idx };
        for (uint8_t b = 0; b < bytes; b++) {
            const uint32_t a = addr + b;
            req[2 + nAddr * 3 + 0] = (uint8_t)(a >> 16);
            req[2 + nAddr * 3 + 1] = (uint8_t)(a >> 8);
            req[2 + nAddr * 3 + 2] = (uint8_t)a;
            nAddr++;
        }
        return true;
    };
    auto addEntry = [&](size_t i) -> bool {
        const RtSsm &e = s_table[i];
        return add(e.metricId, e.bytes == 2 ? 2 : 1, e.isSigned, e.scale, e.offset,
                   e.address, (int16_t)i);
    };

    size_t testing = 0;                  // suspects in this request
    if (bisecting) {
        testing = (s_suspects.size() + 1) / 2;
        for (size_t k = 0; k < testing; k++) {
            const int16_t s = s_suspects[k];
            if (s >= 0) { if ((size_t)s < s_table.size()) addEntry((size_t)s); }
            else        add(0, 1, false, 1, 0, SW_BYTES[~s], -1);
        }
        if (!nAddr) { s_suspects.clear(); return P_IDLE; }
    } else {
        /*
         * Most overdue first. Every candidate is scored by how late it is: a
         * slow value by how far past its period, a needle value (period 0) by
         * how long since it was last read. With room for everything (33
         * addresses, the usual case) that changes nothing; when the request
         * has to be small - the ECU limits it, or it shrank after failures -
         * the slow values still get their turn instead of being crowded out
         * by the fast ones for good.
         */
        static std::vector<Due> due;
        due.clear();
        const size_t total = s_table.size();
        for (size_t k = 0; k < total; k++) {
            const size_t i = (s_cursor + k) % total;       // rotation breaks ties
            const RtSsm &e = s_table[i];
            if (!e.enabled || !e.metricId || s_refused[i]) continue;
            if (ssm2Supported(e) == 0) continue;
            const int32_t late = lateness(now, s_lastPoll[i], e.periodMs);
            if (e.periodMs && late < 0) continue;          // not due yet
            if (metricCoveredAbove(e.metricId, SRC_SSM, Cfg.coverMs)) continue;
            due.push_back({ (int16_t)i, late });
        }
        if (total) s_cursor = (s_cursor + 1) % total;

        if (Cfg.ssmSwitches && s_initOk)
            for (int b = 0; b < 6; b++) {
                const int32_t late = lateness(now, s_swPoll[b], SW_PERIOD_MS);
                if (late < 0) continue;
                // Only a byte some switch is still needed from: supported,
                // not refused, and not already read from the bus.
                bool needed = false;
                for (const auto &d : SW_TABLE)
                    if (d.addr == SW_BYTES[b] && swSupported(d) &&
                        !metricCoveredAbove(d.metric, SRC_SSM, Cfg.coverMs)) needed = true;
                if (needed) due.push_back({ (int16_t)~b, late });
            }

        // Insertion sort, most overdue first: stable (ties keep the rotation
        // order), and a few dozen entries need no allocation.
        for (size_t a = 1; a < due.size(); a++) {
            const Due x = due[a];
            size_t b = a;
            while (b > 0 && due[b - 1].late < x.late) { due[b] = due[b - 1]; b--; }
            due[b] = x;
        }
        for (const Due &d : due) {
            // A two-byte value that does not fit may leave room for a
            // one-byte one further on.
            const bool added = d.idx >= 0 ? addEntry((size_t)d.idx)
                                          : add(0, 1, false, 1, 0, SW_BYTES[~d.idx], -1);
            if (!added && (nAddr >= cap || nSlot >= MAX_ADDR)) break;
        }
    }
    if (!nAddr) return P_IDLE;

    uint8_t rsp[1 + MAX_ADDR];
    size_t got;
    const ExResult r = exchange(req, 2 + nAddr * 3, SSM_RSP_READ, rsp, sizeof(rsp), got);
    if (r == EX_BUSY) return P_BUSY;

    if (r == EX_NRC) {
        const uint8_t code = s_lastNrc;
        // "Service not supported": this ECU does not do SSM2 reads at all.
        if (code == 0x11 || code == 0x7F) { s_suspects.clear(); return P_REFUSED; }

        if (bisecting) {
            if (testing == 1) {                    // the culprit
                refuse(slots[0]);
                s_suspects.clear();
            } else {
                s_suspects.resize(testing);        // it is in this half
            }
            return P_NRC;
        }
        // Wrong length or too long an answer: the request size, not an address.
        if (code == 0x13 || code == 0x14 || nSlot == 1) {
            if (nSlot == 1 && code != 0x13 && code != 0x14) refuse(slots[0]);
            else {
                s_batchCeil = (uint8_t)max(MIN_ADDR, nAddr - 1);
                s_batch     = (uint8_t)max(MIN_ADDR, nAddr / 2);
            }
            return P_NRC;
        }
        s_suspects.clear();
        for (size_t i = 0; i < nSlot; i++) {
            const int bi = swByteIndex(slots[i].addr);
            s_suspects.push_back(slots[i].tableIdx >= 0 ? slots[i].tableIdx
                                                        : (int16_t)~(bi < 0 ? 0 : bi));
        }
        s_suspectFrom = (uint8_t)nAddr;
        log_i("SSM2: ECU refused a %u-address read (code 0x%02X) - finding the address",
              (unsigned)nAddr, code);
        return P_NRC;
    }
    if (r != EX_OK || got != 1 + nAddr || rsp[0] != SSM_RSP_READ) {
        if (r == EX_OK) setErr("reply %u bytes, expected %u", (unsigned)got, (unsigned)(1 + nAddr));
        return P_FAIL;
    }

    publishReply(slots, nSlot, rsp, now);
    if (bisecting) {
        // This half is innocent. With nothing left, no single address was to
        // blame: the ECU objected to the size of the request.
        s_suspects.erase(s_suspects.begin(), s_suspects.begin() + testing);
        if (s_suspects.empty()) {
            s_batchCeil = (uint8_t)max(MIN_ADDR, (size_t)(s_suspectFrom - 1));
            s_batch     = (uint8_t)max(MIN_ADDR, (size_t)(s_suspectFrom / 2));
            log_i("SSM2: no single address refused - requests kept below %u addresses",
                  s_suspectFrom);
        }
    }
    masterUpdateDerived();
    s_responses++;
    s_lastOkMs = now;
    s_lastErr[0] = '\0';
    return P_OK;
}

/* ─────────────────────────────────── task ────────────────────────────────── */

static void ssm2Task(void *) {
    uint32_t rateWindowMs = millis(), rateCount = 0;

    for (;;) {
        if (s_reinit) {
            s_reinit = false; s_initOk = false; s_initFails = 0; s_lastInitTry = 0;
        }
        if (!diagSsmWanted() || !diagBusAlive() || !diagGuardOk()) {
            s_active = false;
            s_exchPerSec = 0;
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }
        refreshTable();

        if (!s_initOk) {
            s_active = false;
            // Quick retries first, then back off: an ECU that does not speak
            // SSM2 should not be pestered forever.
            const uint32_t backoff = s_initFails < 3 ? 1000 : 15000;
            if (millis() - s_lastInitTry >= backoff) {
                s_lastInitTry = millis();
                const bool ok = ssm2Init();
                s_initFails = ok ? 0 : (uint8_t)min(s_initFails + 1, 200);
                if (ok) { s_okStreak = s_failStreak = 0; }
                if (ok || s_initFails >= 2) diagReportSsm(ok);
                if (!ok && s_initFails == 3)
                    log_w("SSM2: no answer to init (%s) - retrying every 15 s", s_lastErr);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        s_active = true;
        const PollResult r = ssm2PollOnce();
        uint32_t wait = max((uint32_t)Cfg.ssmGapMs, (uint32_t)50);

        switch (r) {
            case P_IDLE:                     // nothing left for SSM2 to do
                wait = 250;
                break;
            case P_OK: {
                rateCount++;
                s_failStreak = 0;
                const uint8_t top = min(s_batchCeil, min((uint8_t)MAX_ADDR, Cfg.ssmBatchMax));
                if (++s_okStreak >= 20 && s_batch < top) {
                    s_batch = min((uint8_t)(s_batch + 4), top);
                    s_okStreak = 0;
                }
                break;
            }
            case P_NRC:                      // handled by bisecting; not a failure
                s_okStreak = 0;
                break;
            case P_REFUSED:
                log_w("SSM2: ECU refuses the read service (code 0x%02X) - asking again later",
                      s_lastNrc);
                s_initOk = false;
                s_initFails = 3;             // the 15 s back-off
                s_lastInitTry = millis();
                diagReportSsm(false);
                break;
            case P_BUSY:
                wait = 20;
                break;
            case P_FAIL:
                s_errors++;
                s_okStreak = 0;
                // Back off exponentially: a struggling ECU gets room, not more.
                wait = min((uint32_t)5000, wait << min((int)s_failStreak, 6));
                if (++s_failStreak == 2 && s_batch > 8) s_batch = max((uint8_t)8, (uint8_t)(s_batch / 2));
                if (s_failStreak >= FAILS_REINIT) {
                    log_w("SSM2: ECU stopped answering (%s) - re-probing", s_lastErr);
                    s_initOk = false;
                    s_initFails = 3;
                    s_lastInitTry = millis();
                    s_failStreak = 0;
                    diagReportSsm(false);
                }
                break;
        }

        const uint32_t t = millis();
        if (t - rateWindowMs >= 1000) {
            s_exchPerSec = rateCount * 1000.0f / (t - rateWindowMs);
            rateWindowMs = t;
            rateCount = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(wait));
    }
}

void ssm2Begin() {
    s_rxQ = xQueueCreate(24, sizeof(twai_message_t));
    if (!s_rxQ) {
        log_e("SSM2: response queue alloc failed");
        return;
    }
    s_batch = min((uint8_t)MAX_ADDR, Cfg.ssmBatchMax);
    // Below the OBD-II task (5): when both want the bus, OBD-II gets it.
    xTaskCreatePinnedToCore(ssm2Task, "ssm2", 6144, nullptr, 4, nullptr, 1);
    log_i("SSM2 task started (%u parameters, %u switches)", (unsigned)Cfg.ssm.size(),
          (unsigned)SW_COUNT);
}
