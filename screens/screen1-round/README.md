# Screen 1 — ESP32-C3 Round Telemetry Display (1.28")

Part of the [automotive-telemetry](../../README.md) system:

- **This screen** — an all-in-one ESP32-C3 board with an integrated
  1.28" circular GC9A01 IPS display and CST816S touch. Listens to an ESP-NOW
  broadcast firehose and renders fully configurable gauges (LVGL 8). No vehicle
  logic on board — 100 % driven by `layout.json` and a web-based Gauge Studio.
- **Master Node ([`master/`](../../master/), blueprint in
  [`MASTER_NODE_BLUEPRINT.md`](../../master/MASTER_NODE_BLUEPRINT.md))** — a classic ESP32 +
  CAN transceiver that parses the vehicle bus (OBD-II polling + raw DBC-style
  sniffing, incl. the electrical system) and broadcasts engineering units over
  ESP-NOW in multi-frame bursts (unlimited metric count, ≤250 B per frame).

The only shared coupling is the packed contract in
[`include/MasterPacket.h`](include/MasterPacket.h) — keep the copies in
`master/include/` and `screens/screen2-cluster/include/` byte-identical.

## Build & flash (Slave)

```bash
pio run -t upload
```

Then flash the filesystem (layout + Gauge Studio web app) — required once:

```bash
pio run -t uploadfs
```

Serial console runs on the board's native USB-C (115200).

> Uploads use `board_upload.before_reset = usb_reset`. This board talks over the
> C3's native USB-serial-JTAG, where the classic DTR/RTS auto-reset does not
> reliably enter download mode — without that setting, uploads fail with
> *"Wrong boot mode detected (0xc)"*.

> **Before first flash:** verify the pin map in
> [`include/HardwareConfig.h`](include/HardwareConfig.h) against your board —
> it is the single source of truth for every GPIO.

## Using the display

| Control | Action |
|---|---|
| Swipe left / right | Next / previous screen |
| Swipe up | Cycle day-night source: auto → forced day → forced night |
| Swipe down | Reset all peak values |
| Tap | Toggle peak-value display |
| Hold 5 s | Start config AP **Telemetry-Gauge-Config** → open `http://192.168.4.1` (Gauge Studio) |
| Hold 5 s (in config mode) | Reboot & apply |
| Side button, tap | Next screen (works with gloves) |
| Side button, 2.5 s | Same as the 5 s touch hold |

**If the side button does nothing**, it is on a different GPIO than assumed.
Swipe to the **DIAGNOSTICS** screen and watch the `GPIO` row while holding the
button — it lists the candidate pins live, e.g. `9H 8H 20H 21H`. The one that
flips to `L` is your button. Put that number in `global.button_gpio` in
layout.json (or the studio's Raw JSON tab) and reboot; no rebuild needed.

The default is GPIO9, the ESP32-C3 download strap. The strap is only read while
the chip resets, so it is free as a runtime input, and holding it while plugging
in USB still enters the ROM bootloader.

Gauge Studio lets you add/edit screens (arc gauges, needle tachometers, 2/4-way
splits, text lists, trend charts, diagnostics), map any metric ID, tune
colors/angles/thresholds with a live circular preview, upload background images
(LVGL `.bin`), and save the layout to the device.

## The watchdog (guard)

Two separate alarm paths, and it matters which one you edit:

| | Per-gauge `thresholds` | **Watchdog** (`watchdog.items`) |
|---|---|---|
| Where | On one screen | Global — its own **Watchdog** tab in the studio |
| Needs the metric on a screen | Yes | **No** |
| What happens | That widget recolors and pulses | A **warning triangle pops up over whatever screen is showing** |

The watchdog is the safety guard: give it a list of metrics with limits, and it
polices them all the time regardless of what you are looking at. The card names
the metric, its live value and the limit it broke. Critical pulses its border,
warning is steady. **Tap the display to acknowledge** and mute that one fault
for `global.alert_ack_ms` (default 30 s) — an escalation to critical, or a
second fault, re-arms it immediately, so muting can never hide something new.

Alarms are evaluated on the **raw** value rather than the smoothed display
value, with hysteresis so a reading sitting on a limit does not flicker.

**The pop-up is optional per metric.** Untick *Show pop-up alert* (or set
`"popup": false`) to police a metric silently. That is the right choice when
the metric already has a gauge whose red zone makes the fault obvious — a card
on top would only cover the reading you are trying to read. RPM ships this way,
since the tachometer already goes red at the redline. A silent item is still
evaluated and still counts toward the card's "+N more" badge.

## If transitions feel rough

`global.swipe_anim` picks the transition: `over` (default, cheapest — only the
incoming screen moves), `move`, `fade`, or `none` (instant, always perfectly
smooth). Shorten `swipe_anim_ms` too. The single most expensive thing you can
put on a screen is `background_asset` — LVGL re-reads file-backed images from
LittleFS on *every* repaint, so prefer `bg_grad` (free) or `texture`
(`carbon`/`mesh`, compiled into the firmware).

## Repository map

| Path | What it is |
|---|---|
| `platformio.ini`, `partitions_custom.csv` | Slave build config, 2 MB LittleFS partition |
| `include/HardwareConfig.h` | **All** pins & board constants (single source of truth) |
| `include/MasterPacket.h` | ESP-NOW data contract (metric registry, flags, packing) |
| `include/lv_conf.h` | LVGL v8.3 configuration |
| `src/` | `main` + `DisplayManager` `TouchManager` `NetworkManager` `ConfigManager` `UIBuilder` `TelemetryStore` |
| `data/layout.json` | Annotated example layout (tach, coolant, electrical, powertrain, overview, diag) |
| `data/www/index.html` | Gauge Studio single-page app (served from LittleFS) |
| `data/assets/*.bin` | Uploadable backgrounds: carbon weave, brushed metal, HUD rings |
| `src/ui/Textures.h` | **Generated** — compiled-in textures and the GT dial face |
| `tools/make_assets.py` | Regenerates both (`python tools/make_assets.py`) |

## Look and feel

The default screens are styled as a GT / sports cluster: a near-black carbon
dial with a deep vignette, one red index at twelve o'clock, and white numerals
with a single warm and a single cool accent. Nothing else is bright, because
the needle and the numbers are what you actually read at a glance.

Two knobs control it, both per screen:

- **`dial_face`** — the drawn-on bezel rings and rim pips. Default on; the GT
  screens set it `false`, since concentric rings against black read as grid
  lines and fight the widget's own ticks.
- **`texture`** — `gt_dial` (full-screen carbon face), `gt` / `carbon` /
  `mesh` (32 px tiles), or `none`. All are **compiled into the firmware**, so
  drawing one is a blit with no filesystem in the path.

Contrast that with `background_asset`, which points at a LittleFS file: LVGL
re-reads file-backed images line by line on *every* repaint, so a 240×240
background costs ~115 KB of flash reads per full repaint. That mechanism still
exists for your own uploads, but nothing in the default layout uses it.

Regenerate or restyle the artwork by editing `tools/make_assets.py` and
running it — it emits both the compiled header and the uploadable `.bin`s.
| `../../master/MASTER_NODE_BLUEPRINT.md` | Full master-node engineering blueprint |
| `../../master/` | Standalone PlatformIO project: the master firmware |
