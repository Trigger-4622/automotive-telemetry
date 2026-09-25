# CLAUDE.md

CAN-bus telemetry for the owner's 2009 Subaru Legacy B4: a master that reads
the car's bus and broadcasts over ESP-NOW, and two screens that draw it. See
README.md for the layout. Three standalone PlatformIO projects:

- `master/` - ESP32-S3 DevKit (16 MB, CH343 USB bridge) + SN65HVD230 on the OBD port
- `screens/screen1-round/` - ESP32-C3, 1.28" round GC9A01 + CST816S touch, LovyanGFX
- `screens/screen2-cluster/` - Guition JC4827W543 (ESP32-S3, 4 MB flash, 8 MB PSRAM), 4.3" 480x272 NV3041A QSPI + GT911, Arduino_GFX

## Commands

```bash
python scripts/test_all.py                    # everything: sync checks + the three harnesses
python scripts/test_all.py sync master        # or any of: sync master screen1 screen2
python scripts/check_sync.py                  # just the copies-and-generated-files rules below
python master/test_host/run.py [scenario] [-v]                 # master in a simulated car
python screens/screen1-round/test_host/run.py [scenario]       # screen renders -> test_host/shots/*.png
python3 -m platformio run -d master           # firmware build (also screens/screen1-round, screens/screen2-cluster)
python master/tools/build_portal.py           # after editing master/tools/portal_page.html or WebPortal.cpp.in
python master/tools/known_catalogue.py        # after adding metric IDs to MasterPacket.h
```

The harnesses are the only way to exercise the firmware without hardware.
In a cloud session `scripts/cloud-setup.sh` (session-start hook) has installed
what they need; if it printed "NOT set up", say so rather than skipping the
tests. Firmware builds need `*.platformio.org` on the environment's network
allowlist; if the build cannot download, say so - CI builds all three anyway.

## How to deliver a change

1. Work on a branch. Add or extend a harness scenario for what you change
   (`master/test_host/tests/scenarios.cpp`, `screens/*/test_host/tests/ui_tests.cpp`
   - the latter is shared, so the same file goes in both screens).
2. `python scripts/test_all.py` must pass before you finish.
3. Open a pull request and explain what changed and how it was tested. CI
   (`.github/workflows/ci.yml`) runs `Tests - sync/master/screen1/screen2` and
   `Build - <project>`; all must be green. Screen renders are attached to the
   run as artifacts - look at them after any UI change.
4. Say plainly what still needs the hardware: nothing here can flash a board.

## Rules from the owner - always

- **Never disable or reduce what the master does on the active bus.** New
  safety behaviour must add to it (the bus guard, the power-on settle wait,
  listen-only are all opt-out-able extras), never remove requests or learning.
- **The master's saved settings and learned signals must survive every
  firmware update.** Config is `/config.json` on LittleFS (`MasterConfig`).
  New keys get defaults when absent; do not bump `CFG_VERSION` without a
  migration that touches only what changed, and never re-seed tables for a
  newer version (that once erased every learned signal). The harness scenario
  `device_config` replays the real car's file
  (`master/test_host/fixtures/`) and must show 0 differences.
- **Colours: the round screen's cyan scheme is the reference.**
  `include/Palette.h` is identical in both screens; an amber re-theme was
  rejected outright. Match the round board, don't redesign it.
- **Never `uploadfs` a screen** - it wipes the layout and touch calibration.
  The Gauge Studio page is compiled into the firmware (`tools/embed_studio.py`).
- Cloud sessions cannot flash hardware. Flashing happens on the owner's PC:
  full `esptool read_flash` backup and `verify_flash` first, check the
  partition table matches the build, plain `pio run -t upload`, then read back
  the settings partition.

## Things that must stay in sync (`scripts/check_sync.py` enforces them)

- `include/MasterPacket.h` byte-identical in master and both screens. Metric
  store sizes move together on all three nodes.
- Shared by the two screens, change both: `include/Palette.h`,
  `include/TelltaleIcons.h`, `src/ui/Telltales.*`, `tools/embed_studio.py`,
  `tools/make_icons.py`, `tools/telltale_icons.json` and all of `test_host/`.
- Generated, never edit by hand: `master/src/WebPortal.cpp` (from
  `master/tools/portal_page.html` + `WebPortal.cpp.in` via build_portal.py),
  the studio metric catalogue in `screens/*/data/www/index.html` (from
  MasterPacket.h via known_catalogue.py), and `screens/*/src/config/StudioPage.h`
  (from `data/www/index.html`; every firmware build regenerates it).

## Platform traps

- Pinned `espressif32@6.9.0` (Arduino core 2.0.17) and exact library versions
  in every platformio.ini. Do not upgrade: the ESP-NOW callback signature
  changes in core 3, and the cluster's `GFX Library for Arduino` must stay at
  1.4.9 (later versions need core 3).
- LVGL heap is the real limit on the screens. The PC build is 64-bit, so the
  harness estimates device use from `-m32` struct sizes (`boot_all_screens`
  prints it). Check it after any UI addition; the cluster once overflowed.
- LVGL 8 cannot zoom `ALPHA_8BIT` images (draws nothing); U+00B7 is not in the
  fonts (use U+2022).
- Cluster: `board_build.arduino.memory_type = qio_opi` and
  `board_upload.flash_size = 4MB` are required (wrong ones boot-loop); the
  GT911 is mounted 90 degrees out and gestures run on measured axes
  (`TouchCal`), not a coordinate mapping.
- Master bus rules: only the TWAI RX task may reinstall or uninstall the
  driver; everything that transmits goes through `diagGuardOk()` /
  `diagBusLock()`; in normal mode the controller also ACKs and error-flags, so
  listen-only is the only mode that cannot disturb the car.
- ESP-NOW broadcasts are change-driven with a 300 ms keep-alive (screens drop
  a value after 1.5 s); frames/s is not a health metric, metrics/s is.

## The car

2009 Legacy B4, 500 kbit/s CAN on OBD pins 6/14. The ECU answers SSM2 (SYS
A11009, ROM 5B443C4107) and OBD-II on 0x7E0/0x7E8. About 25 broadcast IDs.
The car's saved config (`master/test_host/fixtures/`, diag mode 2 = OBD-II
only) holds 18 signals: learned RPM 0x231, pedal/throttle 0x232, coolant
0x451, MAF and fuel level 0x705, intake temp 0x706; mapped by hand handbrake
0x4B1, lights 0x351, seatbelt 0x432, steering 0x331, reverse 0x451, neutral
0x252. Not yet taught: door, turn signals, high beam, cruise; oil pressure has
no known source.

## Open items

- Master power-on settle wait (`start_delay_s`, 10 s default) is built and
  tested but not yet flashed.
- The check-engine-light work (`master/P1718_DIAGNOSIS.md`): receive-error
  guard (`rx_guard`), P2CAN wait (`obd_p2can`), request budget
  (`req_max_hz`), TX recessive hold (`tx_hold`), physical-addressing latch and
  the evidence log (portal Diagnostics tab) are built and tested in the
  harness (`tcm_p1718` and the scenarios after it) but not yet flashed. The
  simulator has a TCM that sets P1718.
- Screens: warning lamps, dash screen and the cluster redesign are built and
  tested in the harness but not yet flashed or seen on hardware.
- Cluster: whether the Arduino_GFX driver cured the panel speckle is
  unconfirmed on hardware.

More detail: `master/BRINGUP.md`, `master/MASTER_NODE_BLUEPRINT.md`, each
screen's README, and each `test_host/README.md`.
