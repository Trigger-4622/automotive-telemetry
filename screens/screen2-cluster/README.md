# Screen 2 — ESP32-S3 4.3" Dash Cluster (480×272 landscape)

A second telemetry display node, alongside the 1.28" round board in
[`../screen1-round`](../screen1-round). Both listen to the
**same ESP-NOW broadcast from one master**, so they can run side by side in the
same vehicle — broadcasts reach every listener on the channel.

Hardware: **Guition JC4827W543C** — ESP32-S3-WROOM-1-N4R8 (dual core 240 MHz,
4 MB flash, 8 MB OSPI PSRAM), NV3041A 480×272 IPS panel on **Quad SPI**, GT911
capacitive touch.

## What differs from the round board

| | Round (1.28") | This board (4.3") |
|---|---|---|
| MCU | ESP32-C3, single core | ESP32-S3, dual core + 8 MB PSRAM |
| Panel | GC9A01, plain SPI | NV3041A, **Quad SPI** (4 bits/clock) |
| Touch | CST816S | **GT911** (16-bit registers, address latched at reset) |
| Canvas | 240×240 circle | 480×272 landscape rectangle |
| Flash | 4 MB, app got 1.875 MB | 4 MB **total** — 2.5 MB app / 1.4 MB filesystem |

The layout schema is otherwise identical: a `layout.json` written for the round
board loads here unchanged, it just will not use the extra room.

## Screen types

Shared with the round board: `arc_gauge`, `meter_gauge`, `quad`, `text_list`,
`chart`, `diag`.

Added here, because they need the width:

- **`cluster`** — the hero screen. Needle dial on the left, one dominant
  digital readout in the centre, and a stack of up to four secondary values
  down the right. Three regions, one glance each. Keys: `dial`, `main`,
  `side[]`.
- **`bars`** — up to six horizontal bar gauges stacked down the screen. The eye
  compares lengths along one axis far faster than it compares six separate
  numbers.

Also widened: `quad` takes up to **8 cells** (choosing 2, 3 or 4 columns from
the cell count, so six lays out as 3×2 rather than a 2×2 with holes),
`text_list` takes **10 rows** in two columns, and `diag` shows eight readouts
in two columns including **PSRAM** — which reads `NONE` if the module's memory
type is misconfigured.

## Build & flash

```bash
pio run -t upload
```

No filesystem upload: the Gauge Studio page is compiled into the firmware,
and a first boot writes a built-in layout by itself. For the full example,
paste `data/layout.json` into the studio's **Raw JSON** tab and Save.

> **Never run `pio run -t uploadfs` on a display in use.** It replaces the
> whole LittleFS partition - the layout, the uploaded backgrounds and (on the
> cluster) the touch calibration with it. Upload backgrounds from the studio's
> **Assets** tab instead.

## Board gotchas worth knowing

Three things cost real debugging time here; all are already handled in
[platformio.ini](platformio.ini), but they are the first places to look if a
fresh board misbehaves.

**Memory type.** The N4R8 module pairs *quad* flash with *octal* PSRAM. The key
that selects the right precompiled Arduino libraries is
`board_build.arduino.memory_type = qio_opi`. `board_build.psram_type` and
`board_build.memory_type` are **not read by this platform** and silently do
nothing.

**Flash size.** The `esp32-s3-devkitc-1` board definition assumes 8 MB; this
module has 4 MB. Left unset, the bootloader gets an 8 MB flash-size header and
the chip boot-loops — which presents as a USB CDC port that appears and
vanishes repeatedly and never becomes usable. Hence
`board_upload.flash_size = 4MB`.

**A LovyanGFX link error.** `Panel_NV3041A::init_cmds` is declared
`static constexpr` in the class but never defined out of class, which C++11
still requires when the symbol is odr-used. Linking fails with *"undefined
reference to init_cmds"*. [DisplayManager.cpp](src/hal/DisplayManager.cpp)
supplies the definition; delete that block if LovyanGFX fixes it upstream or
the toolchain moves to C++17.

**The GT911's "ready" bit is not "touched".** Status bit 7 means *a new
coordinate frame is available*, not *a finger is down*. The controller
refreshes at ~100 Hz and LVGL polls every 20 ms, unsynchronised, so plenty of
polls land between refreshes with the flag clear while the finger is still on
the glass. Treating that as a release chops one swipe into a burst of tiny
press/release pairs, none long enough to pass the swipe threshold — they all
classify as taps. [TouchManager](src/hal/TouchManager.cpp) holds the previous
state when the flag is clear and only believes a release when the controller
reports zero contacts.

If taps land in the wrong place or swipes go backwards, the digitiser is
mounted at a different orientation to the panel: fix it with `TOUCH_SWAP_XY`,
`TOUCH_INVERT_X` and `TOUCH_INVERT_Y` in
[HardwareConfig.h](include/HardwareConfig.h).

**`cfg.invert` is inert on this panel — use `invertDisplay()` instead.**
`Panel_LCD::setInvert` normally XORs the requested state with `_cfg.invert`,
which is how the flag works on the round board's GC9A01. `Panel_NV3041A`
*overrides* `setInvert` and sends `INVON`/`INVOFF` straight from its argument,
ignoring `_cfg.invert` entirely — and init always passes `false`. So the config
field does nothing whichever way it is set, which makes it a trap: it looks
like the control. Inversion is applied in
[DisplayManager::begin](src/hal/DisplayManager.cpp) via
`_gfx.invertDisplay(LCD_INVERT)`; this panel needs `LCD_INVERT = 1`. Symptom
when wrong: a white background with dark text while every position and
proportion stays correct, because inversion changes colour and never geometry.

### Stray pixels: four causes, in the order they were found

0. **The bus was not actually in quad mode.** LovyanGFX only ever *tests*
   `LGFX_USE_QSPI`; it never defines it. Without `-D LGFX_USE_QSPI` in
   `build_flags`, `Bus_SPI` never sets `_is_quad_spi`, quietly falls back to
   single-line SPI on `pin_mosi`, and leaves D1/D2/D3 unconfigured — while
   `Panel_NV3041A` carries on driving its quad protocol over one lane. The
   display half-works, which is the worst possible failure mode: it looks for
   all the world like marginal signal integrity. `pin_mosi` is now `-1`, since
   the four io pins are what matter once quad mode is on.


All three are fixed; they are listed because each produces the same symptom —
speckle scattered through freshly drawn elements — and knowing which is which
saves a lot of guessing.

1. **Transaction framing.** The round board holds a single `startWrite()` open
   forever and lets DMA pushes overlap, which is fine on plain SPI.
   `Panel_NV3041A` switches the bus in and out of quad mode around each
   transfer and manages its address window inside
   `beginTransaction`/`endTransaction`, so an outer transaction leaves that
   framing straddling transfers. Each flush now gets its own transaction.

2. **Buffer alignment.** A `static lv_color_t buf[]` is only guaranteed
   *2-byte* aligned — the natural alignment of its element type. SPI DMA wants
   its source word-aligned, and a half-word-aligned source yields sporadic
   wrong pixels rather than an honest failure. The buffers are now allocated
   with `heap_caps_malloc(..., MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)`.

3. **Odd-width windows.** LVGL invalidates the exact bounding box of what
   changed, so labels and arcs routinely produce odd `x1`/`x2`. An odd-width
   window leaves this controller's column counter half a word out of step with
   the 16-bit pixel stream, and everything after that point in the write lands
   a pixel adrift. A `rounder_cb` snaps every flush to an even column span.

**If speckle still remains,** it is signal integrity on the four data lines
rather than framing: step `LCD_SPI_HZ` in
[HardwareConfig.h](include/HardwareConfig.h) down to `26666667`. The ESP32
divides 80 MHz by integers, so the usable steps are 80, 40, 26.67 and 20 MHz —
asking for anything else silently rounds.

Uploads use `board_upload.before_reset = usb_reset`, same as the round board —
the classic DTR/RTS auto-reset does not reliably enter download mode on native
USB-serial-JTAG.

**Editing `data/` from PowerShell:** `Set-Content -Encoding UTF8` writes a
byte-order mark, and ArduinoJson rejects a BOM outright — `layout.json` then
silently falls back to the built-in default. Write those files with
`[System.IO.File]::WriteAllText($f, $text, (New-Object System.Text.UTF8Encoding($false)))`.

## Everything else

Controls, the watchdog, the Dash Studio portal, day/night, smoothing and the
ESP-NOW contract all behave exactly as documented in the round board's
[README](../screen1-round/README.md). `include/MasterPacket.h` must
stay **byte-identical** across the master and both displays.
