/**
 * @file HardwareConfig.h
 * @brief SINGLE SOURCE OF TRUTH for every GPIO and board-level constant of the
 *        Guition JC4827W543 — 4.3" 480x272 landscape dash cluster.
 *
 * =============================================================================
 *  BOARD IDENTITY
 * =============================================================================
 *  Guition JC4827W543C: ESP32-S3-WROOM-1-N4R8 (dual core 240 MHz, 4 MB flash,
 *  8 MB OSPI PSRAM) driving an NV3041A 480x272 IPS panel over **Quad SPI**,
 *  with a GT911 capacitive touch controller on I2C.
 *
 *  This is a different animal from the 1.28" round board:
 *    - QSPI, not plain SPI — four data lines, no separate D/C pin, command
 *      bytes travel in-band. Arduino_GFX's NV3041A driver handles that
 *      (DisplayManager.h says why not LovyanGFX here).
 *    - Landscape, not circular — no corner masking, no chapter ring, and far
 *      more room, which is what the extra screen types exist to use.
 *    - PSRAM — so the UI can afford things the C3 could not.
 *
 *  Pin map from the vendor package and community board-support repositories:
 *  https://github.com/profi-max/JC4827W543_4.3inch_ESP32S3_board
 * =============================================================================
 */
#pragma once

#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  NV3041A 480x272 IPS PANEL — Quad SPI
 * ═══════════════════════════════════════════════════════════════════════════
 *  QSPI moves four bits per clock instead of one, which is what makes a
 *  480x272 panel viable on a general-purpose SPI peripheral: a full frame is
 *  261 KB, and at 1 bit/clock that would cap the frame rate far too low.
 */
#define PIN_LCD_SCLK       47    /**< QSPI clock                              */
#define PIN_LCD_D0         21    /**< QSPI data 0 (MOSI in single-line mode)  */
#define PIN_LCD_D1         48    /**< QSPI data 1                             */
#define PIN_LCD_D2         40    /**< QSPI data 2                             */
#define PIN_LCD_D3         39    /**< QSPI data 3                             */
#define PIN_LCD_CS         45    /**< Chip select, active low                 */
#define PIN_LCD_RST        -1    /**< Not routed — software reset only        */
#define PIN_LCD_BL          1    /**< Backlight (PWM dimmable, active high)   */

#define LCD_WIDTH         480    /**< Landscape. Portrait rotation is NOT
                                      supported by this panel — only 0 and
                                      180 degrees behave.                     */
#define LCD_HEIGHT        272
/**
 * QSPI write clock.
 *
 * The ESP32 derives this by integer division of 80 MHz, so the usable steps
 * are 80, 40, 26.67 and 20 MHz — asking for 30 silently gets you something
 * else. 40 MHz is the safe default for the NV3041A.
 *
 * Set to 26.67 MHz rather than 40. With quad mode genuinely enabled, all FOUR
 * data lines now switch at this rate — before `-D LGFX_USE_QSPI` was added
 * only D0 was ever driven, so marginal routing on D1..D3 could not show
 * itself. Backing the clock off is the standard remedy for the speckle that
 * remained after the framing, alignment and window-width fixes.
 *
 * Raise to 40000000 if you want the frame rate back and see no dots. The
 * ESP32 divides 80 MHz by integers, so the only real steps are 80, 40, 26.67
 * and 20 MHz — asking for anything else silently rounds to one of these.
 */
#define LCD_SPI_HZ   26666667
#define LCD_BL_PWM_FREQ 12000    /**< Backlight PWM frequency [Hz]            */

/**
 * Display colour inversion.
 *
 * Passed to Arduino_NV3041A as its `ips` flag in @ref DisplayManager::begin,
 * which is what applies inversion on this panel - there is no separate invert
 * call afterwards. The panel comes out of its init sequence showing inverted
 * colours, so this is 1.
 *
 * Symptom if wrong: a white background with dark text, while every position
 * and proportion stays correct — inversion changes colour, never geometry.
 */
#define LCD_INVERT          1

/* ═══════════════════════════════════════════════════════════════════════════
 *  GT911 CAPACITIVE TOUCH — I2C @ 400 kHz
 * ═══════════════════════════════════════════════════════════════════════════ */
#define PIN_TOUCH_SDA       8    /**< I2C data                                */
#define PIN_TOUCH_SCL       4    /**< I2C clock                               */
#define PIN_TOUCH_INT       3    /**< Touch interrupt (also a reset strap —
                                      see TouchManager::begin)                */
#define PIN_TOUCH_RST      38    /**< Touch controller reset, active low      */

/**
 * GT911 address select.
 *
 * Unlike the CST816S, the GT911's I2C address is chosen at reset by the level
 * on INT while RST is released: INT low selects 0x5D, INT high selects 0x14.
 * TouchManager drives that sequence deliberately rather than hoping, then
 * probes both, because a board that powers up in the other state otherwise
 * looks exactly like broken hardware.
 */
#define TOUCH_I2C_ADDR   0x5D    /**< Primary address (INT held low at reset) */
#define TOUCH_I2C_ADDR_ALT 0x14  /**< Alternate address (INT held high)       */
#define TOUCH_I2C_HZ   400000
#define TOUCH_MAX_POINTS    5    /**< GT911 reports up to 5 contacts.         */

/**
 * Touch-to-display axis mapping.
 *
 * The digitiser is a separate part bonded over the panel, and nothing
 * guarantees the two agree on which way is up. If taps land somewhere other
 * than where you touched, or swipes go the wrong way, correct it here rather
 * than in the driver — these are the only three transforms needed, and every
 * combination of them is reachable.
 *
 * Symptom → fix:
 *   left/right reversed        → TOUCH_INVERT_X 1
 *   up/down reversed           → TOUCH_INVERT_Y 1
 *   axes transposed (90° out)  → TOUCH_SWAP_XY 1
 */
/**
 * Coordinate range the GT911 actually reports, BEFORE any swap.
 *
 * This is the controller's own frame, which is not the same thing as the
 * display's. It is configured for a 480x272 panel, so raw X counts 0..479 and
 * raw Y counts 0..271 - but when the panel is bonded 90 degrees out, raw X is
 * spread over the physical SHORT edge and raw Y over the LONG one.
 *
 * The consequence is easy to miss and hard to debug: the two axes end up with
 * different units per physical pixel (about 1.76 vs 0.57 here, a 3x
 * mismatch). Swapping them alone is not enough, because gesture code compares
 * horizontal against vertical travel - so vertical wins every dominance test
 * and horizontal swipes never fire, while vertical ones fire easily.
 * TouchManager rescales both axes into display pixels using these numbers.
 */
#define TOUCH_NATIVE_W    480
#define TOUCH_NATIVE_H    272

#define TOUCH_SWAP_XY       0
#define TOUCH_INVERT_X      0
#define TOUCH_INVERT_Y      0

/* ═══════════════════════════════════════════════════════════════════════════
 *  GESTURE TUNING (pixels / milliseconds)
 * ═══════════════════════════════════════════════════════════════════════════
 *  Thresholds are larger than the round board's: 480 px of travel is
 *  available, so a 40 px swipe here would fire on what the user meant as a
 *  tap-and-drag.
 */
/**
 * Contact must persist this long before it counts as a press at all.
 *
 * The GT911 will occasionally emit a single frame reporting a contact that
 * was never really there — a brush of a sleeve, electrical noise, or the
 * leading edge of a finger still approaching the glass. Acting on the first
 * frame turns each of those into a stray tap. Requiring the contact to still
 * be present a few frames later costs an imperceptible amount of latency and
 * removes the whole class of phantom presses.
 */
#define GESTURE_PRESS_MIN_MS     130

/**
 * Consecutive "zero contacts" frames required before believing a release.
 *
 * The controller drops the occasional frame mid-drag. Treating one empty
 * frame as a lift chops a swipe into fragments, none of which travels far
 * enough to be a swipe.
 */
#define GESTURE_RELEASE_FRAMES     2


/**
 * A quick flick counts as a swipe over a shorter distance.
 *
 * Intent is carried by speed as much as by distance. A short, fast movement
 * is unambiguously a swipe, while the same distance covered slowly is a
 * finger resting and drifting. Requiring the full travel from both made
 * flicks feel ignored.
 */
#define GESTURE_FLICK_MS         180   /**< Window that counts as a flick     */
/**
 * @name Gesture thresholds as FRACTIONS of a screen traversal
 *
 * Not pixels. The engine works in the touch controller's own coordinates,
 * whose scale depends on how the panel is bonded and is not known until it is
 * measured — so a threshold in pixels means nothing until then, whereas "a
 * tenth of the way across the screen" is meaningful immediately and stays
 * correct after calibration refines the scale.
 * @{
 */
#define GESTURE_SWIPE_FRAC       0.10f  /**< Deliberate drag: 10% of a screen */
#define GESTURE_SWIPE_FRAC_FLICK 0.05f  /**< Quick flick: 5% is enough        */
#define GESTURE_TAP_MAX_FRAC     0.04f  /**< Travel still counting as a tap   */
#define GESTURE_HOLD_SLOP_FRAC   0.08f  /**< Drift allowed while holding      */
/** @} */

#define GESTURE_TAP_MAX_MS       900   /**< Max press duration for a tap      */
#define GESTURE_HOLD_CONFIG_MS  3000   /**< Hold duration to enter AP config  */

/**
 * Movement below this is not reported to LVGL.
 *
 * Capacitive coordinates jitter by a pixel or two even on a perfectly still
 * finger. Passing that through makes LVGL think the pointer is being dragged,
 * which cancels presses on widgets and makes the UI feel unreliable.
 */
#define TOUCH_JITTER_PX            3

/* ═══════════════════════════════════════════════════════════════════════════
 *  SIDE BUTTON
 * ═══════════════════════════════════════════════════════════════════════════
 *  The JC4827W543 exposes BOOT on GPIO0 and RESET on EN. GPIO0 is the S3's
 *  download strap, which only matters during reset, so it is usable as a
 *  runtime input. Discovery is on by default: hold the button and the pin
 *  that deviates from its resting level is named on screen.
 */
#define PIN_BUTTON              0    /**< BOOT side button (active low)       */
#define BUTTON_DEBOUNCE_MS     30
#define BUTTON_LONGPRESS_MS  2500    /**< Long-press → config AP / reboot     */

/**
 * Candidate GPIOs probed by the button discovery scanner.
 *
 * Kept to pins with nothing else on them. Omitted: 1 (backlight), 3,4,8,38
 * (touch), 21,39,40,45,47,48 (QSPI panel), 19/20 (native USB — reconfiguring
 * those kills the console uploads depend on), 26-32 (SPI flash/PSRAM, and
 * driving those bricks the running firmware).
 */
#define BUTTON_SCAN_PINS   { 0, 2, 17, 18 }
#define BUTTON_DISCOVERY_SETTLE_MS  300  /**< Ignore pins until they settle.  */
#define BUTTON_DISCOVERY_CONFIRMS     6  /**< Consecutive deviating samples.  */

/* ═══════════════════════════════════════════════════════════════════════════
 *  DISPLAY / UI TIMING
 * ═══════════════════════════════════════════════════════════════════════════ */
#define UI_UPDATE_PERIOD_MS       25   /**< Gauge value/lerp update cadence   */
#define UI_SWEEP_UP_MS           700   /**< Boot needle sweep: min → max      */
#define UI_SWEEP_DOWN_MS         500   /**< Boot needle sweep: max → min      */
#define BRIGHTNESS_DAY           255   /**< Backlight level, day mode (0-255) */
#define BRIGHTNESS_NIGHT          80   /**< Backlight level, night mode       */

/* ═══════════════════════════════════════════════════════════════════════════
 *  CONFIGURATION ACCESS POINT (Gauge Studio)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define CONFIG_AP_SSID   "Telemetry-Dash-Config"
#define CONFIG_AP_PASS   ""            /**< "" = open network. Set 8+ chars to
                                            protect the portal.               */
