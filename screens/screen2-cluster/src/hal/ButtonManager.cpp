/**
 * @file ButtonManager.cpp
 * @ingroup hal
 * @brief Side-button debounce, press classification, and discovery scanner.
 */
#include "ButtonManager.h"

#include "HardwareConfig.h"

#include <driver/gpio.h>

ButtonManager Button;

/** @brief Candidate GPIOs probed by discovery. */
static const int kScanPins[] = BUTTON_SCAN_PINS;
/** @brief Number of entries in @ref kScanPins. */
static constexpr size_t kScanCount = sizeof(kScanPins) / sizeof(kScanPins[0]);

static_assert(kScanCount <= ButtonManager::SCAN_MAX,
              "BUTTON_SCAN_PINS has more entries than SCAN_MAX");

void ButtonManager::begin(int pin, bool discovery, const Callbacks &cbs) {
    _cbs       = cbs;
    _pin       = pin;
    _discovery = discovery;
    _startedMs = millis();
    // Not a GPIO on this chip (a typo in button_gpio): it would read as held
    // for good.
    if (_pin >= 0 && !GPIO_IS_VALID_GPIO(_pin)) {
        log_w("button_gpio %d is not a GPIO on this chip - button off", _pin);
        _pin = -1;
    }

    _candCount = kScanCount;
    for (size_t i = 0; i < kScanCount; i++) _cand[i].gpio = kScanPins[i];

    // A pull-up on every candidate gives a defined resting level. Discovery
    // then watches for DEVIATION from that level, so it does not matter
    // whether the button pulls the pin down or up, nor whether the resting
    // level comes from this pull-up or an external one on the board.
    for (size_t i = 0; i < kScanCount; i++) pinMode(kScanPins[i], INPUT_PULLUP);
    if (_pin >= 0) pinMode(_pin, INPUT_PULLUP);

    delay(5);
    _lastRaw       = (_pin >= 0) && digitalRead(_pin) == LOW;
    _stablePressed = _lastRaw;
    // Down already at boot - held, or a pin that sits low: it must be let go
    // before it counts. Taken as a press from time 0, it fired the long press
    // (config mode, telemetry off) the moment the debounce was over.
    _longFired     = _lastRaw;
    _pressT0       = millis();
    _lastEdgeMs    = millis();

    log_i("Button on GPIO%d, discovery %s (%u candidates)", _pin,
          _discovery ? "on" : "off", (unsigned)kScanCount);
}

void ButtonManager::poll() {
    const uint32_t now = millis();

    /* ---------------------------- discovery ----------------------------- */
    if (_discovery) {
        // Ground truth to the serial console, so the pin state can be watched
        // directly instead of inferred from the UI. Run `pio device monitor`
        // and hold the button: if the configured pin never leaves H, the
        // button is not wired to it, and no amount of firmware will find it.
        if (now - _lastTraceMs >= 2000) {
            _lastTraceMs = now;
            char levels[48] = {0};
            for (size_t i = 0; i < _candCount; i++) {
                char part[10];
                snprintf(part, sizeof(part), "%d=%c ", _cand[i].gpio,
                         digitalRead(_cand[i].gpio) == LOW ? 'L' : 'H');
                strlcat(levels, part, sizeof(levels));
            }
            log_i("button scan | watching GPIO%d | %s", _pin, levels);
        }

        // Give the pull-ups time to charge before trusting a reading;
        // sampling a still-floating pin would bake in a bogus resting level.
        if (now - _startedMs >= BUTTON_DISCOVERY_SETTLE_MS) {
            for (size_t i = 0; i < _candCount; i++) {
                Candidate &c = _cand[i];
                const bool level = digitalRead(c.gpio) == HIGH;
                if (!c.baselined) {
                    c.resting   = level;
                    c.baselined = true;
                    continue;
                }
                // Require the deviation to PERSIST. A single sample is not
                // evidence of a button: any pin carrying a pulse trips a
                // one-shot comparison, which is how the touch interrupt line
                // got reported as the button on the first tap. A held button
                // deviates across many polls in a row; an interrupt does not.
                if (level != c.resting) {
                    if (c.streak < 255) c.streak++;
                } else {
                    c.streak = 0;
                }

                if (c.streak >= BUTTON_DISCOVERY_CONFIRMS && !c.reported) {
                    c.reported  = true;
                    _discovered = c.gpio;
                    log_i("Button discovery: GPIO%d held %c -> %c",
                          c.gpio, c.resting ? 'H' : 'L', level ? 'H' : 'L');
                    if (_cbs.onDiscovery) _cbs.onDiscovery(c.gpio);
                }
            }
        }
    }

    /* -------------------------- press handling -------------------------- */
    if (_pin < 0) return;

    const bool raw = digitalRead(_pin) == LOW;   // active low

    if (raw != _lastRaw) {                       // edge → restart debounce
        _lastRaw    = raw;
        _lastEdgeMs = now;
        return;
    }
    if (now - _lastEdgeMs < BUTTON_DEBOUNCE_MS) return;

    if (raw && !_stablePressed) {                // ---- press start ----
        _stablePressed = true;
        _pressT0       = now;
        _longFired     = false;
    } else if (raw && _stablePressed) {          // ---- held ----
        if (!_longFired && now - _pressT0 >= BUTTON_LONGPRESS_MS) {
            _longFired = true;
            if (_cbs.onLongPress) _cbs.onLongPress();
        }
    } else if (!raw && _stablePressed) {         // ---- release ----
        _stablePressed = false;
        if (!_longFired && _cbs.onShortPress) _cbs.onShortPress();
    }
}

void ButtonManager::scanState(char *out, size_t n) {
    if (_discovered >= 0) {
        snprintf(out, n, "PRESS = GPIO %d", _discovered);
        return;
    }
    // The configured pin is starred, so it is obvious which reading is the
    // one that has to change when the button is pressed.
    out[0] = '\0';
    for (size_t i = 0; i < kScanCount; i++) {
        char part[12];
        snprintf(part, sizeof(part), "%d%c%s%s", kScanPins[i],
                 digitalRead(kScanPins[i]) == LOW ? 'L' : 'H',
                 kScanPins[i] == _pin ? "*" : "",
                 (i + 1 < kScanCount) ? " " : "");
        strlcat(out, part, n);
    }
}
