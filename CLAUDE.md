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
python master/test_host/run.py [scenario] [-v]                 # master in a simulated car (device_config: every fixture)
python master/test_host/run.py portal_teach                    # the portal's teach-by-doing analysis, in Node
python screens/screen1-round/test_host/run.py [scenario]       # screen renders -> test_host/shots/*.png
python3 -m platformio run -d master           # firmware build (also screens/screen1-round, screens/screen2-cluster); `pio run -d ...` on the owner's PC
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
  firmware update.** Config is `/config.json` on LittleFS (`MasterConfig`;
  mounted three times before it is ever formatted - `config_mount_glitch`).
  New keys get defaults when absent; do not bump `CFG_VERSION` without a
  migration that touches only what changed, and never re-seed tables for a
  newer version (that once erased every learned signal). The harness scenario
  `device_config` replays every settings file read off the car
  (`master/test_host/fixtures/`) and must show 0 differences for each.
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

- `include/MasterPacket.h` and `include/DtcText.h` (trouble-code names)
  byte-identical in master and both screens. Metric store sizes move together
  on all three nodes. DtcText.h's table is `static`: one source file per
  project includes it (DiagCodes.cpp, Telltales.cpp).
- Shared by the two screens, change both: `include/Palette.h`,
  `include/TelltaleIcons.h`, `src/ui/Telltales.*`, `src/ui/PerfTimer.*`,
  `src/ui/DriveViews.*`, `tools/embed_studio.py`, `tools/make_icons.py`,
  `tools/telltale_icons.json` and all of `test_host/`.
- Generated, never edit by hand: `master/src/WebPortal.cpp` (from
  `master/tools/portal_page.html` + `WebPortal.cpp.in` via build_portal.py),
  the studio metric catalogue in `screens/*/data/www/index.html` (from
  MasterPacket.h via known_catalogue.py), and `screens/*/src/config/StudioPage.h`
  (from `data/www/index.html`; every firmware build regenerates it).
- Copied into the studio page by hand: the lamp symbols (`ICONS` in
  `data/www/index.html` = `tools/telltale_icons.json`), and its preview's
  colours (`PV`, `TONES`) must all be `Palette.h` colours.

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
  `diagBusLock()` and hands its frame over with `masterTransmit()` (under
  `s_twaiReg`: only to a controller running in normal mode with its pad
  attached, never into reset mode or a reinstall); in normal mode the
  controller also ACKs and error-flags.
- **Off means off** (`car_off_tx*`, `ecu_gone_quiet`, `restart_off_bus`):
  nothing is sent once nothing has been heard for 250 ms (`TX_LIVE_MS`), and
  `busGuardTask` aborts a frame of ours left unacknowledged on a quiet bus - a
  controller retransmits one back to back for ever (an error-passive sender's
  ACK errors never make it bus-off). Ten requests in a row unanswered and the
  OBD-II poller drops back to the PID 0x00 probe (every 10 s) instead of
  asking every PID while the ECU is off. `masterBeforeRestart()` takes the
  controller off the bus and latches the TX pad recessive (RTC hold) across
  every portal restart; `txRecessiveBoot()` drives the pin before releasing it.
- **Listen-only is NOT silent on this chip by itself.** Espressif erratum
  (ESP32/S2/S3/C3): a listen-only TWAI controller still sends ACTIVE error
  flags, and listen-only freezes its error counters, so one started with REC 0
  never goes error-passive and drives the sender bus-off instead (it made the
  car's MIL worse on 2026-09-26). ESP-IDF's fix
  (`CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM`) is off in the Arduino core, so
  `listenOnlyErratumFix()` in main.cpp sets TEC to `TEC_LISTEN` (200) and REC
  to 128 after `twai_start()`, and `setupTwai()` takes the TX pad from the
  controller while it listens. Every listen-only install must keep both. REC
  alone drifted back to 0 on the car, and the S3 read a TEC of 128 back as
  127 (2026-09-27) - error-active again, and then every misread became rounds
  of its own unheard error flags, each counted (7.8 errors per misread in
  `lom_tec_hold` without the fix). So TEC is written well clear of 128 and
  `busGuardTask` pins it again below `TEC_LISTEN_LOW`; the sim models the
  drop (`faults.tecExitDrop`, on by default). The 2026-09-30 bench boot
  (TEC 200 read back as 200, then TEC and REC "0 within milliseconds", the
  pin applied 75 times in 4 s) was NOT the chip: ESP-IDF v4.4.7's
  `twai_get_status_info()` reports TEC = REC = 0 for any driver installed
  listen-only, whatever the controller holds ("frozen ... meaningless",
  twai.c), and the guard, the portal and the serial log all read them through
  it - so the guard re-pinned every 50 ms, each time a moment in reset mode,
  deaf to the bus. The counters are read from the registers now
  (`twaiCounters()`; `lom_counters_real`), and the sim's driver mock reports
  0 in listen-only as the real one does (that is why the harness never showed
  it). Espressif documents the counters as frozen in listen-only. On the car,
  `[ctrl ]`'s "pinned again" should now stay 0 - if it does not, that is real
  evidence of a chip that loses its counters; the proposal of silent periods
  in normal mode rested on the old reading and waits on that evidence. Never
  read TEC/REC through `twai_get_status_info()` for a decision.
- **TX pad through restarts and installs** (TX/RX sweep, 2026-11-07, against
  the v4.4.7 driver source and Espressif's errata - the S3 has no TWAI
  errata of its own beyond the listen-only one). `esp_restart_noos()`
  resets only the CPUs and start-up keeps peripheral clocks after a CPU
  reset, so the CAN controller runs on through a restart, attached to the
  pad, until `setup()`. Every software restart - a panic, abort, stack
  overflow, interrupt watchdog, and `esp_restart()` - goes through
  `__wrap_esp_restart_noos` (`-Wl,--wrap=esp_restart_noos`, verified in the
  ELF: `esp_restart` and `panic_restart` call it, IRAM, register writes and
  ROM `gpio_matrix_out` only): pad to the GPIO latch, high, RTC-held with
  `tx_hold`. A brownout is a hardware reset on the S3 and cannot be hooked:
  only a pull-up on the transceiver's TXD covers it. `setupTwai()` holds the
  pad through `twai_driver_install()` (which routes it to the controller
  before resetting the peripheral); `twaiGoLive()` releases it
  (`install_pad_latched`). `tecWrite()` reads TEC back and retries before the
  pad is given to a passive controller (`passive_tec_verified`). No mode
  switch while bus-off/recovering (the driver refuses the uninstall), and a
  recovered controller that should listen is replaced, not restarted in
  normal mode (`busoff_latch_after_settle`). Received DLC is clamped to 8 and
  remote frames are activity only (`rx_odd_frames`). Stack headroom of the
  CAN tasks: `[ctrl ]` and `/api/live` (`stack`, `stack_task`); busguard has
  4 KB.
- **Heap** (a save failed and the master restarted, 2026-11-08): the learner
  sizes its tables at the END of `setup()` to the free heap minus 95 KB kept
  for Wi-Fi, the portal and its saves. Anything started after it eats that
  reserve - the trouble-code and drive-log tasks did (13 KB) - so start new
  tasks before `learnerBegin()` (`learner_sizes_last`). A save builds the whole
  car config (~20 KB of JSON, ~30 KB in ArduinoJson); out of memory it used to
  write the truncated document as a valid file (learned signals gone), now it
  writes nothing (`config_save_out_of_memory`). The loop task (portal and
  saves) has 12 KB (`SET_LOOP_TASK_STACK_SIZE`). Heap now, lowest and largest
  block: serial `[stats]` and `/api/live` (`heap`, `heap_min`, `heap_block`);
  the loop task's stack is in the `[ctrl ]` headroom figure. The sim's heap is
  a constant 180 KB, so it cannot show exhaustion itself.
- **Let errors pass** (`tx_passive`, default on; "passive transmit mode" in
  the code): the master never sends an error frame. A controller cannot stop
  signalling errors, or acknowledging, while it transmits (only listen-only
  does that), but kept error-passive (TEC >= 128) its error flags are recessive
  and cannot destroy another module's frame. `twai_start()` clears the
  counters, so `twaiGoLive()` starts the controller with the TX pad off it and
  gives the pad back only once TEC is 220 - every start goes through it: boot,
  a mode switch, the bus-off restart (`error_frames_never` proves no start
  leaves a window). `diagBusLock()` tops TEC up (under `s_twaiReg`, never
  `s_twaiCtl`, which the guard task holds while it waits) before a request
  whenever our own frames have wound it below 170. The receive-error guard
  stands down while TEC >= 128.
- **Safe bus settings** (portal Maintenance, `/api/bus_defaults`,
  `MasterConfig::loadBusDefaults()`, which `loadDefaults()` starts from): the
  bus behaviour alone back to the defaults that transmit, receive and cannot
  break the bus. Keeps the bitrate, pacing, displays, Wi-Fi and everything
  learned - Factory reset does not (`bus_defaults`).
- **CAN bit timing** (`can_timing`, default 2): 87.5 % sample point, SJW 2 and
  triple sampling - the Arduino-CAN profile vehicle buses want; the ESP-IDF
  preset (80 %, single sample) is 0. The portal Bus tab's "What the errors
  are" (the controller's error-code-capture register, `masterErrorKinds`) says
  what a bus error was - check it before guessing. Change the TX pin only with
  `txPadRecessive()` (latch high first, then output - `pinMode` first glitches
  the bus dominant). The simulator models the erratum and CAN fault
  confinement (`lom_erratum_*`, `normal_confinement`).
- Where bus errors come from (Bus tab, "Where they come from"; serial
  `[link ]`): missed frames per periodic ID and mode (`missAccount`), and
  errors against the radio's air time (`radioNoteErrors`, ESP-NOW send
  callback). Anything that makes the master deaf for a moment must call
  `masterBusBlind()` - flash writes, reset-mode register writes, reinstalls -
  or those gaps count as misreads (`link_misses`, `link_radio`). The
  simulator's car sends on its own clock, 0.5 % off the master's; keep it, or
  the radio and learner statistics lock to one shared clock.
- ESP-NOW broadcasts are change-driven with a 300 ms keep-alive (screens drop
  a value after 1.5 s); frames/s is not a health metric, metrics/s is.
  Broadcasts are not acknowledged, so a change to an on/off value or a
  position (0x12xx, 0x21xx, the lever: `isStateMetric`) rides in the next two
  bursts too - one lost packet used to cost a whole indicator flash
  (`lamp_radio_loss`: a third of packets lost, 50 of 52 changes on time).
- The master's portal: Arduino's `WebServer` serves one connection at a time
  and waits up to 5 s on one that has sent nothing yet - which browsers open
  ahead of need. `PortalServer` (WebPortal.cpp.in) drops such a connection as
  soon as another is waiting. The page gives every request a deadline, polls
  one at a time, and polls nothing else while a teach runs.
- **The portals are the only way into the settings** (no serial console, and
  the settings survive a reflash), so nothing may keep an AP from starting.
  Master: a Wi-Fi name of 1-32 bytes and a password of at most 63 are enforced
  when saved (a longer password used to be cut to 31 - not the one typed), a
  softAP that still fails comes up as `Telemetry-Master-Config`, open, and with
  `portal_on` false the BOOT button held 3 s switches the portal back on and
  restarts (`portal_ap_safe`, `portal_rescue`). The master sends ESP-NOW from
  its AP MAC with the portal up and from its station MAC without - a display's
  `master_mac` filter must follow. Screens: a channel, name or password the AP
  cannot take falls back to the built-in one (`config_ap_always_starts`).

## The car

2009 Legacy B4, 500 kbit/s CAN on OBD pins 6/14. The ECU answers SSM2 (SYS
A11009, ROM 5B443C4107) and OBD-II on 0x7E0/0x7E8. About 25 broadcast IDs.
The car's saved config (`master/test_host/fixtures/`: read back 2026-09-25
with diag mode 2 = OBD-II only - the file the P1718 scenarios drive with - and
2026-09-26 with diag mode 0 = Auto) holds 18 signals: learned RPM 0x231, pedal/throttle 0x232, coolant
0x451, MAF and fuel level 0x705, intake temp 0x706; mapped by hand handbrake
0x4B1, lights 0x351, seatbelt 0x432, steering 0x331, reverse 0x451, neutral
0x252. Not yet taught: door, turn signals, high beam, cruise; oil pressure has
no known source.

Teach by doing (portal Bus tab; its analysis is the `teach-core` JS in
`portal_page.html`, tested by `master/test_host/portal_tests.js`) finds a
flashing turn signal from the census's per-bit flip counters (`/api/bus?e=1`,
`masterCensusEdges`), and the gear lever with its Positions mode: the
combination of every bit that moves with the lever, anywhere in one frame
(`"bits":[..]` on a signal, `RtSignal::bitList` - bit i of the code is the
i-th listed bit), saved with a value table (`"map":[[code,value]]`,
`RtSignal::nMap`; the lever publishes its letter's character code,
`METRIC_ID_GEAR_LEVER`). A position visited twice ("P R N D P") must read the
same both times, which drops bits that only happened to change. Only when no
frame tells every position apart does it fall back to one bit per position
(`METRIC_ID_PARK`/`REVERSE`/`NEUTRAL`/`DRIVE`), which `masterUpdateDerived()`
combines into the lever; single doors likewise make `DOOR_OPEN`. A taught
field always outranks the derived value.

A teach runs on the clock: the countdown is redrawn whatever the Wi-Fi does,
and a reading counts only for the step it was both asked for and answered in
(`teachSlot`); a step with fewer than `TEACH_MIN_READS` (3) readings makes the
page say the master answered too slowly (two can make a slow counter bit look
like a switch). The indicators are taught in one run - off, left, off, right,
off, hazard, off (`teachTurns`): a bit that flashes for one side only is that
side, one that flashes for both is reported and never offered, the hazard
switch is the bit held on only in the hazard step, and a stalk bit held for
one side is the fallback (the screens flash those themselves). The lever's
own field is preferred - the shortest run of neighbouring moving bits that
tells every position apart - over the combination of every bit that moved,
and a combination's code within a quarter of its width of exactly one entry
reads as that entry (`nearestCode`, `lever_tolerant`); numeric tables still
need an exact match. Using a taught signal replaces whatever fed that value
before, once confirmed. The screens flash both arrows while `METRIC_ID_HAZARD`
is on - following each side's own bit when it flashes, in step by themselves
when it does not (`lamp_hazard_switch`).

## Codes, oil change, G-meter, drive log, timers

- **Trouble codes** (`master/src/DiagCodes.*`): services 03/07/0A from the
  ECM (0x7E0) and TCM (0x7E1, through 0x7DF with flow control to 0x7E1 if it
  never answers its own address), read 8 s after the bus comes up, every
  `dtc_poll_s` (60; 0 = only when asked) and when the MIL or PID 01's count
  changes (a list left unanswered keeps its codes for two reads); never
  with the bus quiet, listen-only or the guard up (they go
  through `isoExchange()` in Ssm2.cpp - the SSM2 ISO-TP transport, lent out
  under the same lock). Published as `DTC_TOTAL` and `DTC_1..8` (packed:
  raw | kind << 16 | module << 18). Clear (service 04) only from the portal,
  refused while moving. Service 09 gives VIN, calibration ID and CVN
  (remembered in `vin`/`ecu_cal_id`). The screens' check-engine card names the
  codes one at a time; a card-only `codes` lamp covers them with the MIL off.
- **Oil change**: `oil_every_km`, `oil_at_km` (-1 = off); `SERVICE_KM` from the
  SSM2 odometer (address 0x10E); portal "Oil changed now"; a `service` lamp
  (spanner, amber under 500 km).
- **G-meter**: `LAT_ACCEL`/`LON_ACCEL`/`YAW_RATE`, taught from the stability
  unit's frame with the Bus tab's "A corner or a hard stop" (`motionRef`,
  `teachMotion` in teach-core: correlation with the speed's slope, or with
  speed x yaw / the steering, and a fitted scale). Until then the master
  derives longitudinal G from the speed's slope and lateral G from speed x yaw
  (`gDerive`, SRC_DERIVED: a taught field outranks it). ISO 8855 signs.
- **Drive log** (`master/src/DriveLog.*`): `log_on`, `log_hz`, `log_ids`,
  `log_max_kb`; `/log/NNNNN.tlg` on the settings partition, oldest deleted
  first (one open for a download will not delete and is passed over), 128 KB
  always left for settings saves, 4 KB writes announced with
  `masterBusStall()`; the portal converts to CSV (`tlgToCsv`).
- **Timers** (screens, `PerfTimer`): 0-100, 80-120, quarter mile on the
  display from the master's speed, on the master's clock (`TelemetryStore::
  stamped()`). Layout version 3 adds the `gmeter` and `perf` screens once.

## Open items

- Master power-on settle wait (`start_delay_s`, 10 s default): flashed
  2026-09-26.
- The check-engine-light work (`master/P1718_DIAGNOSIS.md`): receive-error
  guard (`rx_guard`, `rx_guard_rec`, `rx_guard_errs`; also latches on bus-off
  when `guard` is off), P2CAN wait (`obd_p2can`), request budget
  (`req_max_hz`, off by default), TX recessive hold (`tx_hold`),
  physical-addressing latch and the evidence log (portal Diagnostics tab) are
  built and tested in the harness (`tcm_p1718` and the scenarios after it)
  and was flashed 2026-09-26 - which made the MIL come sooner, because of the
  listen-only erratum above (fixed since, with the P2CAN pacing now counted
  from the real send time; see the top of P1718_DIAGNOSIS.md; not yet
  flashed). The simulator has a TCM that sets P1718 and an ECM that goes
  bus-off. The switch to listen-only is never throttled; only the switch back
  to normal is.
- Master flashed 2026-09-30 with main 0e3d754 (gear lever teaching, #4 and
  #5, and the listen-only TEC pin). The evidence log of the drive before it,
  with `tx_passive` on, had 47 bus-offs in 155 s while polling: the master's
  own frames fail (TEC 220 is bus-off after five transmit errors), with no
  MIL. Suspect the master's transmit path - the transceiver board's own 120
  ohm terminator (CANH-CANL ~40 ohm instead of ~60), its RS pin, its ground
  to OBD pin 5, the supply - and confirm with the portal's "What the errors
  are" before changing firmware for it.
- Master flashed again 2026-09-30 evening (branch portal-teach-turns-hazard:
  the portal's time-limited requests and idle-connection fix, the indicators'
  teach run, the lever field and tolerant lever reading, lamp repeats). The
  car's lever is saved as 8 bits of 0x253 ([9,28,29,48,49,51,52,54], taught
  that morning); its own field looks like bits 28-29 (P 3, R 2, N 1, D 0) -
  re-teach it with the new page, which offers to replace the old one.
- Round screen flashed 2026-09-30 evening from the same branch (lamps, dash,
  lever letters, hazard arrows): layout and file area byte-identical after,
  boot log clean. Its esptool needs `--before usb_reset` (native USB), and a
  plain upload writes boot_app0 at 0xE000, inside its nvs range - harmless,
  that sector already holds boot_app0.
- Cluster flashed 2026-09-30 evening from the same branch (lamps, dash,
  redesign, lever letters, hazard arrows): file area byte-identical after,
  colour and touch calibration loaded, boot log clean. Same `usb_reset` and
  boot_app0-at-0xE000 notes as the round screen. Not yet seen on the car:
  the lamps, the dash screen, and whether the Arduino_GFX driver cured the
  panel speckle.
- Codes, oil change, G-meter, drive log and timers (above): harness only. On
  the car: whether the TCM answers 0x7E1 (or only 0x7DF) and lists P1718 under
  service 03 at all; that the SSM2 odometer matches the cluster; which frame
  carries the VDC unit's G and yaw (teach it); and how many CAN frames each
  4 KB log write costs (Bus tab misses while logging).

More detail: `master/BRINGUP.md`, `master/MASTER_NODE_BLUEPRINT.md`, each
screen's README, and each `test_host/README.md`.
