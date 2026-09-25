/**
 * @file Palette.h
 * @brief THE single source of truth for every UI colour.
 *
 * Before this existed the chrome (headings, page dots, spinner, toast, chart
 * trace) was cyan while the gauges themselves were amber and blue, so the two
 * halves of the same screen pulled in different directions. Everything now
 * comes from here, and both display nodes share the identical file so the
 * round board and the dash cluster look like one product.
 *
 * ## Choosing values
 *
 * Automotive instruments are read at a glance, in daylight and at night, and
 * often out of the corner of an eye. That drives three rules:
 *
 *  1. **One accent, not three.** Cyan carries normal information. A second
 *     hue appearing anywhere means something has changed.
 *  2. **Reserve red and yellow for faults.** Nothing decorative uses them,
 *     so when they appear they mean exactly one thing.
 *  3. **Never rely on hue alone.** Alarm tiers also differ in motion —
 *     warning is steady, critical pulses — because a driver may not be
 *     looking straight at the screen when it matters.
 *
 * @note Colours are 0xRRGGBB, as @c lv_color_hex expects. Whether they reach
 *       the glass in that order depends on the panel's @c rgb_order; see the
 *       note on it in DisplayManager.h.
 */
#pragma once

/** @name Accents
 *  @{ */
#define UI_ACCENT        0x00E5FF  /**< Cyan. The round board's original accent: live data,
                                        headings, active page dot, needles. */
#define UI_ACCENT_ALT    0x40C4FF  /**< Cooler blue. Secondary series, told apart from
                                        the primary at a glance. */
/** @} */

/** @name Text
 *  @{ */
#define UI_TEXT          0xE0F2F7  /**< Primary readouts. */
#define UI_TEXT_DIM      0xC8D2D8  /**< Secondary text, tick labels. */
#define UI_MUTED         0x9AA7B0  /**< Captions, units, field labels. */
#define UI_DIM           0x5A6870  /**< Minor ticks, inactive dots, hints. */
/** @} */

/** @name Status — reserved for faults, never decoration
 *  @{ */
#define UI_WARN          0xFFC400  /**< Warning tier. Steady. */
#define UI_CRIT          0xFF1744  /**< Critical tier. Pulses. */
#define UI_OK            0x69F0AE  /**< All-clear / link up. */
/** @} */

/** @name Warning lamps
 *  The colours a driver already reads from the car's own cluster (ISO 2575):
 *  red must act, amber should check, green and blue only inform. Red and
 *  amber are the fault colours above; these complete the set.
 *  @{ */
#define UI_LAMP_GREEN    0x00E676  /**< Indicators, lights on, cruise. */
#define UI_LAMP_BLUE     0x448AFF  /**< High beam, engine cold. */
#define UI_LAMP_OFF      0x1A2A33  /**< An unlit lamp's ghost on the dash. */
/** @} */

/** @name Instruments
 *  The round board's tach is the reference: its needle and redline colours
 *  (from its layout) are named here so the landscape dials can match it
 *  without repeating hex values.
 *  @{ */
#define UI_NEEDLE        0xFF2D16  /**< Needles and the redline zone. */
#define UI_ZONE          0xFF9E00  /**< The zone before the redline. */
#define UI_CARD          0x0E151B  /**< Landscape panel, top of its gradient. */
#define UI_CARD_LOW      0x080C10  /**< ...and bottom. */
/** @} */

/** @name Surfaces
 *  @{ */
#define UI_BG_DAY        0x101418  /**< Day background. */
#define UI_BG_NIGHT      0x000000  /**< Night background - true black keeps stray
                                        light out of the cabin. */
#define UI_BG_GRAD       0x05080B  /**< Gradient far stop. */
#define UI_PANEL         0x0A1016  /**< Raised surfaces: chart bed, cards. */
#define UI_LINE          0x2A3A44  /**< Hairlines, separators, bezel ring. */
#define UI_LINE_SOFT     0x1A2A33  /**< The quieter inner hairline. */
#define UI_ALERT_BG      0x140D08  /**< Warning card ground (warm). */
#define UI_ALERT_BG_CRIT 0x1A0A0A  /**< Critical card ground (red-warm). */
#define UI_CONFIG_BG     0x0A1220  /**< Config-mode screen ground. */
/** @} */
