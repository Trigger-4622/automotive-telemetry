/**
 * @file HardwareConfig.h
 * @brief SINGLE SOURCE OF TRUTH for every GPIO and board-level constant of the
 *        all-in-one ESP32-C3 1.28" round display board.
 *
 * =============================================================================
 *  BOARD IDENTITY — CONFIRMED
 * =============================================================================
 *  YourCee "ESP32-C3 1.28 inch TFT IPS GC9A01 with Capacitive Touch", which is
 *  the Shenzhen Jingcai ESP32-2424S012 design. The pin map below is confirmed
 *  against that board's published documentation AND against this firmware
 *  running on the hardware:
 *
 *      display  SCLK 6 · MOSI 7 · CS 10 · DC 2 · BL 3 · RST not routed
 *      touch    SDA 4 · SCL 5 · INT 0 · RST 1
 *      button   GPIO 9  (documented as the user button)
 *
 *  GPIO0 being the touch interrupt was confirmed the hard way: the button
 *  discovery scanner reported it as the button on the first screen tap.
 *
 *  Because the hardware is pre-routed these pins are FIXED. If a future board
 *  differs, correct ONLY this file — no other file in the project contains a
 *  GPIO number.
 * =============================================================================
 */
#pragma once

#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  1.28" GC9A01 ROUND LCD — 4-wire SPI (write-only, no MISO routed)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define PIN_LCD_SCLK        6    /**< SPI clock  (GC9A01 SCL)                 */
#define PIN_LCD_MOSI        7    /**< SPI data   (GC9A01 SDA/DIN)             */
#define PIN_LCD_CS         10    /**< Chip select, active low                 */
#define PIN_LCD_DC          2    /**< Data/Command select                     */
#define PIN_LCD_RST        -1    /**< Panel reset. -1 = tied to EN / soft-
                                      reset only. Some clones route it to
                                      GPIO 8 — change here if needed.         */
#define PIN_LCD_BL          3    /**< Backlight (PWM dimmable, active high)   */

#define LCD_WIDTH         240    /**< Physical resolution — perfectly round   */
#define LCD_HEIGHT        240
#define LCD_SPI_HZ   80000000    /**< 80 MHz write clock. Above GC9A01 spec
                                      but standard practice on these boards
                                      (vendor demos use it); combined with
                                      DMA it doubles transition frame rate.
                                      Drop to 40 MHz if you ever see pixel
                                      noise/tearing on a marginal clone.     */
#define LCD_BL_PWM_FREQ 12000    /**< Backlight PWM frequency [Hz]            */

/* ═══════════════════════════════════════════════════════════════════════════
 *  CST816S CAPACITIVE TOUCH — I2C @ 400 kHz
 * ═══════════════════════════════════════════════════════════════════════════ */
#define PIN_TOUCH_SDA       4    /**< I2C data                                */
#define PIN_TOUCH_SCL       5    /**< I2C clock                               */
#define PIN_TOUCH_INT       0    /**< Touch interrupt (falling edge on touch) */
#define PIN_TOUCH_RST       1    /**< Touch controller reset, active low      */

#define TOUCH_I2C_ADDR   0x15    /**< CST816S fixed 7-bit address             */
#define TOUCH_I2C_HZ   400000

/* ═══════════════════════════════════════════════════════════════════════════
 *  SIDE BUTTON ("BOOT")
 * ═══════════════════════════════════════════════════════════════════════════
 *  The board's two side buttons are RESET (wired to the chip EN pin — not a
 *  GPIO) and BOOT, which Waveshare wires to GPIO9, the ESP32-C3's download
 *  strap. It is safe as a runtime input (active low, internal pull-up); the
 *  strap only matters while the chip resets — which also means holding it
 *  during power-on still enters USB download mode, as designed.
 */
#define PIN_BUTTON              9    /**< BOOT side button (active low).
                                          Override at runtime without a
                                          rebuild via layout.json:
                                          global.button_gpio               */
#define BUTTON_DEBOUNCE_MS     30
#define BUTTON_LONGPRESS_MS  2500    /**< Long-press → config AP / reboot    */

/**
 * Candidate GPIOs probed by the button discovery scanner.
 *
 * GPIO0 was tried here and REMOVED: it is the CST816S touch interrupt, so it
 * pulses low every time the screen is touched and discovery reported it as
 * the button on the first tap. That false positive is also a useful result —
 * it confirms @ref PIN_TOUCH_INT is mapped correctly on this board.
 *
 * Everything else omitted is genuinely unavailable: GPIO1 is the touch reset
 * this code actively drives; 2,3,6,7,10 run the display; 4,5 are the I2C bus;
 * 11-17 are the internal SPI flash (driving those bricks the running
 * firmware); 18,19 are the native USB data lines, so touching them kills the
 * console this board's uploads depend on.
 */
#define BUTTON_SCAN_PINS   { 9, 8, 20, 21 }

/**
 * Consecutive deviating samples required before discovery names a pin.
 *
 * A single sample is not enough: any pin that carries a pulse — an interrupt
 * line, a bus that has not settled — will trip a one-shot comparison. A real
 * button is held, so it deviates across many polls in a row.
 */
#define BUTTON_DISCOVERY_CONFIRMS     6

/**
 * Discovery samples a pin's resting level at boot and then reports any
 * DEVIATION from it, rather than assuming the button pulls low.
 *
 * That matters because assuming active-low is exactly what makes an
 * active-high button invisible: a pin already resting high cannot be seen to
 * go high. Watching for change instead catches either polarity, and it does
 * not care whether the resting level comes from an internal or an external
 * pull.
 */
#define BUTTON_DISCOVERY_SETTLE_MS  300  /**< Ignore pins until they settle. */

/* ═══════════════════════════════════════════════════════════════════════════
 *  GESTURE TUNING (pixels / milliseconds)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define GESTURE_SWIPE_MIN_PX      40   /**< Min horizontal travel for a swipe */
#define GESTURE_TAP_MAX_MS       300   /**< Max press duration for a tap      */
#define GESTURE_TAP_MAX_PX        12   /**< Max travel for a tap              */
#define GESTURE_HOLD_CONFIG_MS  5000   /**< Hold duration to enter AP config  */

/* ═══════════════════════════════════════════════════════════════════════════
 *  DISPLAY / UI TIMING
 * ═══════════════════════════════════════════════════════════════════════════ */
#define UI_UPDATE_PERIOD_MS       30   /**< Gauge value/lerp update cadence   */
#define UI_SWEEP_UP_MS           700   /**< Boot needle sweep: min → max      */
#define UI_SWEEP_DOWN_MS         500   /**< Boot needle sweep: max → min      */
#define BRIGHTNESS_DAY           255   /**< Backlight level, day mode (0-255) */
#define BRIGHTNESS_NIGHT          70   /**< Backlight level, night mode       */

/* ═══════════════════════════════════════════════════════════════════════════
 *  CONFIGURATION ACCESS POINT (Gauge Studio)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define CONFIG_AP_SSID   "Telemetry-Gauge-Config"
#define CONFIG_AP_PASS   ""            /**< "" = open network. Set 8+ chars to
                                            protect the portal.               */
