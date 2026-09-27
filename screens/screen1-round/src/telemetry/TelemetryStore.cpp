/**
 * @file TelemetryStore.cpp
 * @ingroup telemetry
 * @brief Implementation of the smoothed, thread-safe metric cache.
 */
#include "TelemetryStore.h"

TelemetryStore Telemetry;

void TelemetryStore::begin(float emaAlpha, uint32_t staleTimeoutMs) {
    _alpha   = constrain(emaAlpha, 0.01f, 1.0f);
    _staleMs = staleTimeoutMs;
}

TelemetryStore::Slot *TelemetryStore::find(uint16_t id) {
    // Slots are claimed in order and never given back, so the first free one
    // ends the search: a packet's lookups (under the spinlock, interrupts off)
    // cost the metrics heard, not all MAX_SLOTS.
    for (auto &s : _slots) {
        if (!s.used) return nullptr;
        if (s.id == id) return &s;
    }
    return nullptr;
}

TelemetryStore::Slot *TelemetryStore::findOrAlloc(uint16_t id) {
    Slot *slot = find(id);
    if (slot) return slot;
    for (auto &s : _slots) {
        if (!s.used) {
            s = Slot{};
            s.used    = true;
            s.id      = id;
            // Unset, so the first valid value seeds the average. Slot{} left
            // it at 0, which made every gauge glide up from zero on first
            // contact: 90 degrees of coolant read 31 at first.
            s.ema     = NAN;
            s.display = NAN;   // first sample() snaps straight to the EMA
            s.peak    = -INFINITY;
            return &s;
        }
    }
    return nullptr;            // every slot in use — ignore extras
}

void TelemetryStore::ingestRaw(const uint8_t *mac, const uint8_t *data, int len) {
    const MasterTelemetryPacket *pkt =
        reinterpret_cast<const MasterTelemetryPacket *>(data);
    if (!telemetry_packet_valid(pkt, len)) return;

    const uint32_t now = millis();
    bool night = false;

    portENTER_CRITICAL(&_mux);

    if (mac) {
        memcpy(_peerMac, mac, 6);
        _peerKnown = true;
    }

    // ---- link statistics ----------------------------------------------------
    if (_stats.packetsReceived > 0 &&
        pkt->sequence_id > _stats.lastSequenceId + 1) {
        _stats.packetsDropped += pkt->sequence_id - _stats.lastSequenceId - 1;
    }
    _stats.lastSequenceId = pkt->sequence_id;
    _stats.packetsReceived++;
    _stats.lastRxMs = now;
    _windowCount++;
    if (now - _windowStartMs >= 1000) {
        _stats.packetsPerSecond =
            _windowCount * 1000.0f / (float)max(now - _windowStartMs, 1UL);
        _windowStartMs = now;
        _windowCount   = 0;
    }

    // ---- metric ingestion ---------------------------------------------------
    for (uint8_t i = 0; i < pkt->metric_count; i++) {
        const MetricEntry &e = pkt->metrics[i];
        // Not a number, or infinite: no value at all. An infinity would stick
        // as the peak for good (a peak only rises) and poison the average.
        if (!isfinite(e.value)) continue;
        Slot *s = findOrAlloc(e.metric_id);
        if (!s) continue;

        s->flags        = e.flags;
        s->lastUpdateMs = now;
        if (e.flags & METRIC_FLAG_VALID) {
            s->raw = e.value;
            // Exponential moving average at packet rate
            s->ema = isnan(s->ema) ? e.value
                                   : s->ema + _alpha * (e.value - s->ema);
            if (e.value > s->peak) s->peak = e.value;
            if (e.flags & METRIC_FLAG_NIGHT) night = true;
        }
    }
    _night = night;

    portEXIT_CRITICAL(&_mux);
}

bool TelemetryStore::sample(uint16_t id, float lerpFactor, MetricSample &out) {
    const uint32_t now = millis();
    bool found = false;

    portENTER_CRITICAL(&_mux);
    Slot *s = find(id);
    if (s) {
        // Linear interpolation toward the EMA target, one UI tick's worth.
        if (isnan(s->display)) s->display = s->ema;
        else                   s->display += (s->ema - s->display) * lerpFactor;

        out.value = s->display;
        out.raw   = s->raw;
        out.peak  = s->peak;
        out.flags = s->flags;
        out.stale = (now - s->lastUpdateMs) > _staleMs;
        found = true;
    }
    portEXIT_CRITICAL(&_mux);
    return found;
}

bool TelemetryStore::peek(uint16_t id, MetricSample &out) {
    const uint32_t now = millis();
    bool found = false;

    portENTER_CRITICAL(&_mux);
    Slot *s = find(id);
    if (s) {
        out.value = isnan(s->display) ? s->ema : s->display;
        out.raw   = s->raw;
        out.peak  = s->peak;
        out.flags = s->flags;
        out.stale = (now - s->lastUpdateMs) > _staleMs;
        found = true;
    }
    portEXIT_CRITICAL(&_mux);
    return found;
}

size_t TelemetryStore::snapshot(MetricView *out, size_t max) {
    const uint32_t now = millis();
    size_t n = 0;
    portENTER_CRITICAL(&_mux);
    for (auto &s : _slots) {
        if (!s.used || n >= max) continue;
        out[n].id    = s.id;
        out[n].raw   = s.raw;
        out[n].peak  = s.peak;
        out[n].flags = s.flags;
        out[n].ageMs = now - s.lastUpdateMs;
        n++;
    }
    portEXIT_CRITICAL(&_mux);
    return n;
}

void TelemetryStore::resetPeaks() {
    portENTER_CRITICAL(&_mux);
    for (auto &s : _slots)
        if (s.used) s.peak = s.raw;
    portEXIT_CRITICAL(&_mux);
}

bool TelemetryStore::linkUp() const {
    return _stats.packetsReceived > 0 &&
           (millis() - _stats.lastRxMs) < _staleMs;
}

String TelemetryStore::peerMac() {
    uint8_t m[6];
    bool known;
    portENTER_CRITICAL(&_mux);
    known = _peerKnown;
    memcpy(m, _peerMac, 6);
    portEXIT_CRITICAL(&_mux);

    if (!known) return String();
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             m[0], m[1], m[2], m[3], m[4], m[5]);
    return String(buf);
}

LinkStats TelemetryStore::stats() {
    portENTER_CRITICAL(&_mux);
    LinkStats copy = _stats;
    portEXIT_CRITICAL(&_mux);
    return copy;
}
