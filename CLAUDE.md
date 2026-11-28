# CLAUDE.md

CAN-bus telemetry for the owner's 2010 Subaru Legacy B4: a master that reads
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
python screens/screen1-round/test_host/run.py studio_page      # the studio page against the firmware, in Node
python3 -m platformio run -d master           # firmware build (also screens/screen1-round, screens/screen2-cluster); `pio run -d ...` on the owner's PC
python master/test_qemu/run.py [--readers] [--extra-ids 45] [--seconds 600]   # the real master firmware in Espressif's QEMU (needs $QEMU_ESP32S3; not in test_all)
python master/test_qemu/run.py --replay drive.log [--settings cfg.json]   # ...fed a recorded candump log (a real Subaru's: test_qemu/README.md)
python master/test_qemu/run.py --replay d1.log --once --flash two.bin     # ...one drive, the car off; run d2.log on two.bin after: a power cycle between drives
python master/test_qemu/run.py --ota                # ...a Wi-Fi firmware update, kept, a second one crashing, rolled back
python master/test_qemu/run.py --settings cfg.json --upload   # ...the portal's settings upload at falling free heap, now and as it was
python master/tools/build_portal.py           # after editing master/tools/portal_page.html or WebPortal.cpp.in
python master/tools/known_catalogue.py        # after adding metric IDs to MasterPacket.h
python master/tools/canrec.py canrec.bin [--events]   # a raw bus recording (portal Bus tab) -> candump log
```

The harnesses are the only way to exercise the firmware's logic without
hardware. `master/test_qemu` (its README) runs the real compiled master on an
emulated ESP32-S3 - real stacks, heap and both cores, which the harness cannot
show - against a simulated car; it has no radio and no CAN error counters (the
settings file and the drive log work: its flash is read in DIO mode, QEMU
misreads QIO), and is a test build that must never be flashed. QEMU itself now
and then segfaults early in a boot; run.py tries again (3 times, 5 with --ota).
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
  the settings partition. The master's table gained the `logs` partition on
  2026-11-17 (the first upload after it writes the new table; the settings
  partition, 0x670000 + 0x180000, must read back identical to the backup).
  From that firmware on, the master also updates over Wi-Fi (below); the
  screens have one app slot each and are USB only.

## Things that must stay in sync (`scripts/check_sync.py` enforces them)

- `include/MasterPacket.h` and `include/DtcText.h` (trouble-code names)
  byte-identical in master and both screens. Metric store sizes move together
  on all three nodes. DtcText.h's table is `static`: one source file per
  project includes it (DiagCodes.cpp, Telltales.cpp).
- Shared by the two screens, change both: `include/Palette.h`,
  `include/TelltaleIcons.h`, `src/ui/Telltales.*`, `src/ui/PerfTimer.*`,
  `src/ui/DriveViews.*`, `src/ui/AutoScreen.*`, `tools/embed_studio.py`, `tools/make_icons.py`,
  `tools/telltale_icons.json` and all of `test_host/`.
- Generated, never edit by hand: `master/src/WebPortal.cpp` (from
  `master/tools/portal_page.html` + `WebPortal.cpp.in` via build_portal.py),
  the studio metric catalogue in `screens/*/data/www/index.html` (from
  MasterPacket.h via known_catalogue.py), and `screens/*/src/config/StudioPage.h`
  (from `data/www/index.html`; every firmware build regenerates it).
- Copied into the studio page by hand: the lamp symbols (`ICONS` in
  `data/www/index.html` = `tools/telltale_icons.json`), and its preview's
  colours (`PV`, `TONES`) must all be `Palette.h` colours.
- The studio page (`test_host/studio_tests.js`, the `studio_page` scenario,
  not check_sync): its script between `@shared-begin` and `@shared-end` is the
  same on both screens - BOARD decides what differs, and only the preview
  drawing after it is each board's own. Every setting the firmware reads from
  the layout's global, network, watchdog and warnings blocks is one row of the
  page's `SETTINGS`, once, with the firmware's default (the old page had the
  watchdog's pop-up switch on two tabs, the cluster's button on GPIO 9 and
  defaults the firmware never used). The screen editor (`TYPES`,
  `DASH_PARTS`, `BG`) offers per board and type only what that screen draws:
  every key it writes must be read where UIBuilder.cpp builds that type, and
  its number defaults are the firmware's. A new layout key gets a row or a
  field there and nowhere else.

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
- ArduinoJson 7 takes a `const char[N]` for a string literal and keeps only
  its address. In a const method every member array is one: put it in as
  `(const char *)`, which is copied, or the document reads the live table
  after its lock is let go - the save measured one thing and wrote another
  while the learner moved the signals (`config_doc_snapshot`, 2026-11-18).
- Cluster: `board_build.arduino.memory_type = qio_opi` and
  `board_upload.flash_size = 4MB` are required (wrong ones boot-loop); the
  GT911 is mounted 90 degrees out and gestures run on measured axes
  (`TouchCal`), not a coordinate mapping. The layout's `touch_swap_xy`,
  `touch_invert_x/y` and `touch_native_w/h` are no longer read (they only
  moved the point LVGL is given, which no widget uses, and the diagnostics
  readout); the studio drops them from a layout it loads.
- **A screen's layout save** ("the screens won't save settings", 2026-11-27):
  the studio's Save is a form's file, POST `/api/layout`, written to the flash
  as it arrives (`/layout.up`) and parsed from there into the place of the
  layout in memory, which is let go first (`ConfigManager::applyLayoutFile`,
  `studio_save_streamed`); then renamed over `/layout.json`. Held in memory
  the body was there three times (the server's buffer, its String, the
  handler's copy) beside a probe document and a by-value copy of it: about
  70 KB above the stored layout for the example one (32-bit model), more than
  the C3 has left with its AP up and tight in the cluster's internal RAM -
  now about 1.5 KB. A layout that will not parse or has no screens, or a
  flash that will not take it, changes nothing and puts the stored layout
  back; the page shows the display's reason. The device's `touch_cal` stands
  over the upload's, and so does every `network` key the upload leaves out
  (the master's channel, the AP name and password: a file from another
  display must not move this one off the channel - `layout_upload_keeps_network`).
  A bare JSON body (curl) streams the same way. The studio's Device tab also
  has "Upload to the display": a file sent as it is, as the master's
  settings Upload works, then the editor reloads what the display took.
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
- **A stamp written by another task: read it, then the clock**
  (`sleep_busy_bus_race`). `s_lastCanMs` is written by the receive task on
  core 1; `millis() - s_lastCanMs` evaluated the other way round wraps to 49
  days when a frame is stamped in between, and the sleep check put the master
  to sleep on a busy bus (QEMU, a real 40-minute drive, 2026-11-17).
  `settleLeft()`, `heardWithin()` and `diagBusAlive()` already read it first.
  The harness shows such races only through `simrtos::g_afterMillis` (other
  tasks run right after a clock reading, as a preemption or the other core).
- **Asking the car** (`pacing_p2can*`, `obd_two_ecus`, `obd_pending`,
  `dtc_functional_fc`, `dtc_tcm_silent`): one request on the bus at a time,
  whoever sends it - the OBD-II poller, SSM2, the code reader and the
  computers' scan, search and reads all take `diagBusLock()` - and the lock
  is kept until that request is settled:
  answered by every computer known to answer it, refused, or P2CAN since it
  went out (`obdAsk()`: the longer of `obd_to` and `obd_p2can`; "still
  working", 0x78, is waited on for up to 2 s). `obd_to` alone used to end the
  wait and free the bus: with the car's 25 ms, an ECU answering in 35 ms was
  counted silent (its PIDs backed off to a trickle) and in Auto SSM2 put a
  15-frame request into the answer. Anything new that asks must hold the lock
  the same way. `s_pidResponders` is who answers each PID: a 0x7DF request
  waits for each of them, and where the engine ECU answers a PID another
  computer's reading of it is not published (the TCM's "lamp off" used to
  alternate with the ECM's). A 0x7DF request through `isoExchange()` gives
  every computer's first frame its flow control and keeps the bus until the
  long answers have passed (`otherAnswer`, `othersFinish`). The code reader
  waits 150 ms a request, and a transmission computer silent for three reads
  is asked at every tenth (at once on "Read now" and when the car comes on).
  The Signals tab shows how long the ECU takes ("answers in", `/api/live`
  `obd.lat`, `lat_max`) and answers that came after their wait (`late`): if
  that is not 0 on the car, raise `obd_p2can`.
- **OBD-II poll order** (`obd_order`, `obd_poll_order`, `obd_poll_all`): the
  table wants about 200 requests a second and the bus carries about 40, so
  which PID goes first matters. The one most overdue for its own period does
  (`obdPickNext`); one a second past its time (`OBD_LATE_MAX_MS`), or about to
  age out of the broadcast (`metric_ttl_ms`), goes ahead of those. The same
  requests at the same pace in another order: with the simulated ECU's 32
  PIDs, engine speed went from every 504 ms to every 165, road speed to 315,
  a 1 s value to every 2 s. `obd_order` 1 is each in turn, as before. Poll
  times are kept as "scheduled at" and "wait" and compared as time elapsed -
  never zero a timestamp or compare a signed distance to a deadline: powered
  for 24.8 days, the old poller's zeroed deadlines read as in the future and
  not one PID was asked after the next key-on (`obd_resume_old_clock`).
- **Learned signals: proposing and confirming** (`verify_slow_value`,
  `candidate_judged`, `candidate_second_look`, `learn_not_coarse`,
  `learn_needs_steps`). The learner (Learner.cpp) proposes a field as
  `SIG_AUTO`; the verifier (`verifySample` in main.cpp) compares it with the
  ECU's figure - one comparison per reading, settled by the next - and
  promotes or rejects it. The car's settings of 2026-11-14 held an intake
  temperature read from the coolant byte, a MAP in 10 kPa steps, a timing
  advance from a gearbox byte in 1.66 degree steps and two candidates
  "confirming" for ever. What let them in, and the rules now: a reading stands
  for `REF_CHAIN_MS` (4 s, was 600 ms - nothing asked for once a second or
  slower was ever compared, so slow values could only be had with "Listen
  now", which takes the best match unchecked); the range that verifies is the
  range the candidate AGREED over (one disagreeing sample elsewhere used to
  supply it); a learned signal's own tolerance (`vtol`) is not widened to 3 %
  of the value's size (0.42 V for a battery); six times `verify_n` comparisons
  with more than one disagreement in eleven is a rejection; a field worth more
  a step than a quarter of the movement asked and three of the value's own
  steps is not proposed, nor taken by "Listen now" (`tooCoarse`); a value with
  no figure of its own in `NEEDS` must cover eight of its own steps; and a
  rejected field may be proposed once more with a different scale
  (`Excl::strikes`) - "Request instead" still excludes it for good, and so do
  two rejections and a learned row switched off by hand, for every field that
  shares a bit with it (`fieldMask`; the top byte of a 16-bit field turned down
  was learned in its place - `learn_off_respected`). Rejections are saved
  (`learn_excl`, at most 32, written by the learner only; "Restart learning"
  keeps them, "Forget learned values" clears them - `learn_excl_kept`). A
  learned signal in use is checked against the ECU every `learn_recheck_s`
  (30 s; 0 = off, at most 3600; portal Settings > Advanced > Learning, applied without a
  restart - `learn_recheck_setting`): the pollers ask its covered value once
  (`masterRecheckWanted`), the answer is kept beside the metric whatever the
  source priority (`MasterMetric::reqValue`, also what candidates are verified
  against), and it is compared with the frame only after the frame held still
  300 ms before the answer; four disagreements in a row reject it
  (`recheckSample`, `learn_recheck`: the car's intake temperature on the
  coolant byte). Hand-made signals are never re-checked. A reading is paired
  only with a frame that arrived after the value settled at its level - a
  once-a-second frame (the car's 0x705) was paired with readings up to a second
  newer and the MAF never fitted there (`learn_slow_frame`). The field table
  fills in the first seconds from the bytes that have changed by then, and a
  16-bit field needs both its bytes to have changed: on a real 2012 Impreza's
  drive (QEMU) speed's high byte moved at 14.4 km/h, 23.7 s in, the table was
  full at 10.2 s, and speed was never learned in 40 minutes (with the fix:
  learned and verified 195 s in, from a wheel's speed). Fields that find
  the table full now take turns (`swapIn`, `learn_late_field`) with
  candidates that have been in it 20 s, have 150 samples against every value
  that moved, match none and are no value's best field - swapped out, not
  marked dead, so they come back later; discovery starts a row further on
  each pass so no frame is always last in line.
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
  car config (~20 KB of JSON, ~30 KB in ArduinoJson; 35 KB for the car's
  45 signals); out of memory it used to write the truncated document as a
  valid file (learned signals gone), now it writes nothing
  (`config_save_out_of_memory`). The raw recorder takes 16 KB while it
  records: carried on at boot, before the learner sizes its tables; armed
  from the portal, out of that reserve. A settings upload (the page's Save and
  Upload, POST `/api/config`, sent as a form's file) goes to the flash as it
  arrives (`/cfg_upload.json`) and is parsed from there, its document freed
  before the save's (`MasterConfig::applyFile`, `config_upload_file`): held
  in memory the body was there three times, and the owner's 36 KB file could
  not be saved at any heap the master has (QEMU `--upload`, 2026-11-18; now
  from 48 KB free). The page shows the master's reason when a save fails. The loop task (portal and
  saves) has 12 KB (`SET_LOOP_TASK_STACK_SIZE`). Heap now, lowest and largest
  block: serial `[stats]` and `/api/live` (`heap`, `heap_min`, `heap_block`);
  the loop task's stack is in the `[ctrl ]` headroom figure. The sim's heap is
  a constant 180 KB, so it cannot show exhaustion itself.
- **Static RAM is the learner's heap too** (2026-11-16: data + bss 84 KB, was
  123 KB; `pio run` prints it - check it after adding a table). Never keep a
  `static` copy of a table for a portal page or a task: read it a row at a
  time (`masterCensusRow` / `masterCensusOrder`, `masterMetricAt`,
  `learnerStatusRow`; the learner's `Census` holds one row), or take it from
  the heap for the length of the request (`/api/logs`). The census is 48 rows
  that are always there and blocks of 16 the receive task takes when a bus
  has more identifiers (128 at most, as before; 2 KB a block out of the
  reserve, never given back - none on this car, `census_rows`). The verifier's
  counters follow their signals in place when the table changes
  (`verifyCarry`, `verify_carry`: no second copy of the table). Stacks are
  61 KB of the heap and nothing here can measure them: each task's least
  headroom is in the serial log (`[stack]`, once a minute) and `/api/live`
  (`stacks`: [least left, size]) - a task started outside main.cpp reports in
  with `masterWatchStack()` (`stack_watch`). Cut a stack only from the car's
  figures.
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
- **Safe bus settings** (portal Settings > Device, `/api/bus_defaults`,
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
  one at a time, and polls nothing else while a teach runs. Five tabs, each
  polling only what it shows: Home, Signals (learning, teach by doing, the
  sources), Bus (its health, the frames, the evidence log), Car (codes, oil
  change, drive logs, the car's computers) and Settings (the everyday ones, Device, then an
  Advanced section). Every setting is in exactly one row of the page's
  settings block (`SET_MAIN`, `SET_ADV`, `SET_CAR`): a new config key gets a
  row there and nowhere else - `portal_tests.js` checks the block against
  `MasterConfig::toJson`.
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

2010 Legacy B4 (fifth generation, BM), 500 kbit/s CAN on OBD pins 6/14. The ECU answers SSM2 (SYS
A11009, ROM 5B443C4107) and OBD-II on 0x7E0/0x7E8. About 25 broadcast IDs.
The car's saved config (`master/test_host/fixtures/`: read back 2026-09-25
with diag mode 2 = OBD-II only - the file the P1718 scenarios drive with - and
2026-09-26 with diag mode 0 = Auto; 2026-11-14 is the newest, 32 signals, its
VIN's serial masked) holds 18 signals: learned RPM 0x231, pedal/throttle 0x232, coolant
0x451, MAF and fuel level 0x705, intake temp 0x706; mapped by hand handbrake
0x4B1, lights 0x351, seatbelt 0x432, steering 0x331, reverse 0x451, neutral
0x252. Not yet taught: door, high beam, cruise; oil pressure has no known
source. Turn signals: the indicator run of 2026-11-19 (`master/test_host/
traffic/legacy_2010_teach_turn.json`) found no frame on this bus that follows
the lamps - with the right indicator and the hazard lights the only thing that
moved was 0x432 byte 5, a value (61 or 64 at rest, 63 and 66 with each flash),
whose bit 41 the page then offered as the right lamp; it flashed with either
side. Extended identifiers are not in the teach's readings (but for the
computers' rows, below) - not yet checked whether this bus has any.

Teach by doing (portal Signals tab; its analysis is the `teach-core` JS in
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
off, hazard, off, 7 and 8 s a step (`teachTurns`): a bit that flashes for one
side, with the other side's step silent (not one flip), is that side; one
that flashes for both is reported and never offered; the hazard switch is the
bit held, or flashing, in the hazard step alone; a bit held for one side - in
the hazard step too, if it likes - is the fallback (the screens flash those
themselves); and a car that sends one flasher for both sides and the side as
a bit of its own in the same frame gets each side as that flasher while the
side bit says so (a two-bit combination with a table, `turn_gated`). The OFF
steps rest but for the driver's hand at their ends (`restsIn`: flashes at a
step's start where the step before had the bit going, at its end where the
next has, around a still middle of 40 %), a side's lamp that also flashed
with the hazard lights beats one that did not (and drops it), and a bit with
another bit of its byte changing level in the same step is a value's, not a
switch (the steering angle nudged by the hand on the lever). A bit of a
number is never offered (`numberBit`, the result's `.numbers`, named on the
page): another bit of its byte busy with nothing on, other than a rolling
counter's run (each bit about half as fast as the one below), or flipping out
of step with it where it flashes - lamps flash in step with each other. A
number the lamps' load moves follows the indicators: this car's 0x432 byte 5,
and both Imprezas' 0x361 byte 6 (0x75/0x76 at rest, down to 0x6E with each
flash), which the run reported as a flasher for either side and, quiet
through one step by chance, offered as a side. On two real 2012 Imprezas'
indicator traffic (`master/test_host/traffic/`, CANdid) the run used to lose
both lamps with a driver 2 s late switching off - an OFF step with more than
two flips was not at rest; now both lamps come first with the driver up to
3 s late or 2 s early and the master answering in 40 ms to 1.3 s. The lever's
own field is preferred - the shortest run of neighbouring moving bits that
tells every position apart - over the combination of every bit that moved,
and a combination's code within a quarter of its width of exactly one entry
reads as that entry (`nearestCode`, `lever_tolerant`); numeric tables still
need an exact match. A combination saved before that preference is read by
the master from its own field too, where it has one (`leverOwnMask`, worked
out when the receive task copies the table; the saved bits are not rewritten):
the car's eight bits of 0x253 held five that say what the gearbox is doing,
and at speed in D they read as in reverse - one bit from the R entry, which
the tolerant reading took for R (`lever_own_field`). A lever read from a
table entry it matches also outranks a separate "reverse" bit: while it reads
another position, `METRIC_ID_REVERSE` is published as off (the displays put
Reverse before the lever's letter). Using a taught signal replaces whatever fed that value
before, once confirmed. The screens flash both arrows while `METRIC_ID_HAZARD`
is on - following each side's own bit when it flashes, in step by themselves
when it does not (`lamp_hazard_switch`).

## Codes, oil change, G-meter, drive log, raw recording, body computer, timers, parking, car view, backlight

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
  unit's frame with the Signals tab's "A corner or a hard stop" (`motionRef`,
  `teachMotion` in teach-core: correlation with the speed's slope, or with
  speed x yaw / the steering, and a fitted scale). Until then the master
  derives longitudinal G from the speed's slope and lateral G from speed x yaw
  (`gDerive`, SRC_DERIVED: a taught field outranks it). ISO 8855 signs.
- **Drive log** (`master/src/DriveLog.*`): `log_on`, `log_hz`, `log_ids`,
  `log_max_kb` (64-8000, default 8000); `/log/NNNNN.tlg` on the logs' own
  LittleFS partition `logs` - the upper 8 MB of the 16 MB flash, which the
  stock 8 MB layout left blank (`master/partitions_16mb.csv`: its first six
  entries byte-identical to the stock table, so the settings partition never
  moves). Formatted once when found blank, after three failed mounts; drives
  the older firmware left on the settings partition are moved across once,
  each checked before its original goes (`log_move_old`, `log_move_cut`,
  `log_move_fails`). Without the partition (an older table) they stay on the
  settings partition as before: at most 1200 KB, 128 KB always left for
  settings saves (`log_old_table`, `drive_log_full`). Oldest deleted first
  (one open for a download will not delete and is passed over), 4 KB writes
  announced with `masterBusStall()`; the portal converts to CSV (`tlgToCsv`).
  The QEMU test has a 1.9 MB `logs` partition (8 MB with `--ota`). Off while
  a raw recording has the partition (below).
- **Raw bus recording** (master, `master/src/RawRec.*`; portal Bus tab, Raw
  recording; `/api/rawrec`, `/arm`, `/stop`, `/release`, `/file`): every frame
  received and every one the master sends (`masterTransmit`) to the `logs`
  partition, for analysis off the car; `master/tools/canrec.py` turns the
  download into a candump log (the master's frames on `tx0`) and lists the
  identifiers, losses and events. Arming erases the whole 8 MB - the drive
  logs with it - and is refused while moving (each 64 KB erase holds the CAN
  interrupt off; `masterBusStall()` before each); drive logging is off while
  the recorder has the partition (`driveLogRelease`/`driveLogReclaim`; a boot
  then never mounts or formats it). Recording only programs 256-byte pages,
  from a 12 KB ring, in its own task (started before `learnerBegin()`; ring
  and the 224-entry index from the heap, 16 KB, only while it records):
  records never cross a page, a sync record starts every 4 KB sector, a
  frame is stored as the bytes that changed since its identifier's last (the
  master's own too: 6.2 bytes a frame in the simulator, about 20 minutes of
  this car's ~1,250 frames/s), the controller's drops and the ring's are
  recorded where they happened, and so are the evidence log's events.
  Everything that comes over the line is in it, recognised or not (format
  version 2, `rawrec_everything`): every frame as the controller hands it
  over - remote frames, 29-bit ones, a DLC of 9-15 as it was on the wire
  (candump's `_<DLC>`; version 1 cut it to 8) - the bus errors, i.e. the
  error frames (E6: how many, the error-code capture's kind, direction and
  place, listen-only or not, during the master's own frame or not, TEC and
  REC), and every moment the master could not hear the bus (E7, from
  `masterBusBlind()`: a flash write, reset mode for a TEC write, a
  reinstall), so a gap in it says why. It
  goes on across sleeps and restarts, each a new session after the last
  page, until stopped (a byte in the header), full, or given back. Meanwhile
  what the master saves by itself (`Cfg.serviceSave()`, `evService()`) waits
  - a sector erase would cost frames - until the recording ends or the next
  sleep or restart; a Save from the portal is made at once. The format is at
  the top of RawRec.h. Scenarios `rawrec_*`; run.py checks canrec.py against
  the firmware's own reading of `rawrec_roundtrip`'s and `rawrec_everything`'s
  recordings, frame for frame; it reads versions 1 and 2. Not tried on the board: the frames a page program costs (the
  simulator has no flash timing), the erase's minute, the download over the
  AP.
- **The car's computers** (master, `master/src/Computers.*`; portal Car
  tab, "The car's computers"; `/api/computers`, `/scan`, `/search?i=`).
  Nothing in it knows a computer, an address or a data layout in advance:
  no public source gives this car's computers their CAN addresses (searched
  2026-11-28, English, Japanese and Russian, and Subaru's own documents).
  Subaru's Technician Reference Booklet "Electrical Systems Operation &
  Diagnosis" (module 602, June 2013) says the BIU is the gateway between the
  500 kbit/s high-speed CAN (ECM, TCM, ABS/VDC, steering angle sensor, and on
  later cars EPS, EPB, keyless, EyeSight) and the 125 kbit/s low-speed one
  (meter, auto A/C), that later cars put all of them on the high-speed bus at
  the data link connector, and that the BIU sets U1221-U122B when it hears
  nothing from the ECM, TCM, VDC, EPB, meter or A/C; its SSM-III data list
  has the door, lock and light switches, ambient temperature and fuel level -
  no IDs. A community tool for the 2020+ (SGP) cars lists 7E0 ECM, 7E1 TCM,
  7B0 VDC, 780 SRS, 7C0 meter, 7C4 A/C, 740 BIU, 746 EPS, 750 keyless, 787
  EyeSight, each answering on +8 - a newer platform; not used here. Find
  (standing, the bus up, not listen-only, settled): "3E 00" to every 11-bit
  ID 0x700-0x7F7 but 0x7DF, 0x7E8-0x7EF and any ID the master has heard on
  the bus (another module sends there - the car's 0x705/0x706: a frame of
  ours could meet one of its own in arbitration; the old body-computer scan
  did send there, `comp_scan` catches it), the answer taken from whatever ID
  it comes on by what it says (`isoProbe()` in Ssm2.cpp: "7E ..." or "7F 3E
  ..."), then "3E 00" once to every 29-bit computer (0x18DB33F1; answers on
  0x18DAF1xx are computers at 0x18DAxxF1 - `isoExchange()` takes IDs above
  0x7FF as 29-bit), then each one asked who it is: "01 00" (OBD-II), "AA"
  (SSM IDs), "1A 80" (KWP2000 identification), "22 F1 97" and "22 F1 87"
  (UDS name, part number). About 22 s on the simulator, with the OBD-II
  poller keeping its turns. Search (one computer): "21 01" to "21 FE", each
  block that answers kept with its length; a refusal 11 or silence to the
  first three ends it (`comp_search`). Reads (`reads`: [{tx, rx, lid, ms}],
  at most 16, empty by default - `comp_off`; a table, edited on the Car tab):
  each block every `ms` (10 s after five failures in a row), from the codes
  task, its answer into the frame table as extended IDs `COMP_PSEUDO_BASE`
  0x1E000000 | address << 12 | block << 4 | eight-byte part (the address an
  11-bit request ID, or 0x800 | a 29-bit one's target; "0x750 21 50 · 0-7"
  on the portal) through a queue the RX task drains: a signal mapped there
  publishes like a broadcast one, teach by doing takes the rows in, they
  are never missed frames (`idOnRequest`) or in the raw recording
  (`comp_reader`); the learner leaves extended IDs alone. Only reading
  services are sent (3E, 01, AA, 1A, 21, 22 and flow control). The scan's
  tables come from the heap at the first scan (about 3 KB) and the reads'
  state at the first read (1 KB). The simulator has three other computers:
  a KWP2000 body computer at 0x750, a meter at 0x7C0 answering on 0x7D0 and
  a 29-bit UDS one at 0x18DA30F1 (`sim::others`). Not tried on the car: which
  of its computers answer on this bus at all, and what they speak. If the
  body computer does not answer, a raw recording while the owner's scan
  tool reads it (master's requests off) holds the tool's requests. Nothing
  is ever written to a computer (no 2E, 2F, 30, 31, 3B - locking and
  unlocking are not implemented).
- **Firmware update over Wi-Fi** (master only, `master/src/FwUpdate.*`;
  portal Settings > Device, `/api/update`): the upload goes into the spare
  app slot (app0/app1; a USB upload always writes app0 and resets otadata to
  it), is checked (0xE9 image, `FW_MARK` - a display's ESP32-S3 image would
  pass every other check - then the core's checksum, hash and chip check) and
  the master restarts into it through `masterBeforeRestart()`. Refused while
  moving (speed > 2 km/h); every write announced with `masterBusStall()`; an
  upload that stops for `FW_STALL_MS` is dropped, and sleep waits for one in
  progress. Rollback is the bootloader's (`CONFIG_BOOTLOADER_APP_ROLLBACK_
  ENABLE`; `verifyRollbackLater()` overrides Arduino's confirm-at-once): a new
  firmware is on trial until it has run `FW_CONFIRM_MS` (30 s) with the portal
  up, or reaches a clean deep sleep with it up; any reset before that boots
  the previous one. A firmware older than this one has no `FW_MARK` and is
  refused (going back to it is a USB job). CI keeps the master's `firmware.bin`
  as the `master-firmware` artifact. Scenarios `fw_update*`, `fw_trial*`,
  `fw_rolled_back`, `fw_trial_late_clock`. In QEMU (`run.py --ota`, the
  board's partition table, the real bootloader): written in the portal's
  pieces, checked, booted on trial, kept after 30 s; a second update crashing
  before it is kept was rolled back and reported; settings and learned values
  loaded alike at every boot (one run of fifteen: QEMU itself crashed in the
  rest, test_qemu/README.md). Not tried on the board: the upload's speed over
  the AP, the frames each flash write costs.
- **Timers** (screens, `PerfTimer`): 0-100, 80-120, quarter mile on the
  display from the master's speed, on the master's clock (`TelemetryStore::
  stamped()`). Layout version 3 adds the `gmeter` and `perf` screens once,
  version 4 the `park` screen, version 5 the rules below, version 6 the `car`
  screen (round board only).
- **Parking screen** (screens, `park`; `parkBuild`/`parkUpdate` in the shared
  DriveViews): the steering wheel turned as the driver has it (`metric_id`,
  default the steering angle 0x1006, ISO 8855: positive to the left), an arc
  from straight ahead to full lock either way (`steer_max`, 540), the angle
  and LEFT or RIGHT, STRAIGHT in green within `straight_deg` (10)
  (`park_screen`). It comes up in reverse by the rule below that it ships
  with. Layout version 4 adds it once, before `diag` (`migrate_v3_layout`);
  version 5 turns that version's `auto_reverse` switch into the rule, or into
  nothing where it was off (`migrate_v4_layout`). LVGL heap: 2.3 KB on the
  round board, 2.8 KB on the cluster. `view` "path" shows the car from above
  (below) in place of the wheel, its path drawn from the same angle
  (`park_path_view`).
- **The car from above** (screens, `car`; `carBuild`/`carUpdate` and
  `carScreenBuild` in the shared DriveViews; studio type "Car from above"):
  front up, the doors swung out in amber when open (FL FR RL RR, or "a door
  open" when only that is known; faint when not known at all - never drawn
  shut), boot and bonnet, the indicators at its corners (each side's lamp as
  the display's `turn_left`/`turn_right` lamps have it), the headlights and
  the front wheels turned by the steering, and what is open in words
  ("ALL SHUT", "DOOR FL + BOOT"). The path its corners will take - ahead,
  or back from the rear bumper in reverse (the switch or the lever's R) - is
  the single-track model: road wheels at the steering angle over
  `steer_ratio` (15), about a centre on the rear axle's line `wheelbase_mm`
  (2750, the Legacy BM's) behind the front; `path` false leaves it out
  (`car_screen`). Lines are `lv_line`s with their points on the heap (LVGL
  keeps the pointer), each object moved to its line's bounding box. Layout
  version 6 adds it once, before `diag`, on the round board only: on the
  cluster it costs about 6 KB of LVGL heap and its layout would leave less
  than the 8 KB kept spare - the studio offers it there, and a cluster layout
  is migrated to version 6 without it (`migrate_v5_layout`).
- **Screens that come up by themselves** (screens, shared `AutoScreen.*`;
  studio: "Show this screen by itself" on every screen): `show_when`, a list
  of conditions that must all hold - `{"gear":"R"}` (`"op":"!="` for "is
  not"; the reverse switch, the lever's letter and the gear number all count)
  or `{"metric_id","op","value"}` (> < >= <= == !=, == on whole numbers); a
  value the display has not got, or has gone stale, holds nothing. Only
  conditions coming to hold change the screen; when they stop, back to the
  screen before after `back_after_s` (10), or at once when one of
  `back_now_when` holds. A swipe keeps the driver's screen; a screen chosen by
  hand is never taken away; a rule coming to hold takes over from another, and
  when its wait ends a rule still holding gets its screen back before the
  driver's. A rule with a condition that will not read is left out whole,
  with a warning. The shipped parking screen: gear R, back 10 s after or at
  once above 15 km/h (`auto_screen_reverse`, `auto_screen_conditions`,
  `auto_screen_none`). studio_tests.js checks the editor's keys, default,
  operators and gears against AutoScreen.cpp. `"swipe": false` (studio:
  "Only by itself") leaves a screen out of the swipe order and the page
  dots: only its rule brings it up; the display starts on the first screen
  of the order; one with no rule that reads stays in it, with a warning
  (`rule_only_screens`).
- **Round-screen layouts** (`screens/screen1-round/layouts/`, uploaded with
  the studio's "Upload to the display"): `daily.json` - the shipped screens,
  the car from above only when parking (P: the car with its doors; R: the
  parking screen's path view; neither in the swipe order), no network block
  (the display keeps its own); `race.json` - the watchdog guards only RPM
  over (6200 / 6500) and coolant over (105 / 112) and never draws its card;
  they show on the lamp strip (custom_1 SHIFT, custom_2 OVER-REV,
  coolant_hot at 105, strip always up) and bring their screens up (the big
  tach, the coolant gauge); a launch (standing, over 2500 rpm) brings the
  timers, hard braking (-0.7 g) the G-meter; boost, timing, fine knock,
  throttle, the temperatures and AFR on quads, a boost trend; only its AP
  name in network (Telemetry-Gauge-Race). `round_layout_daily` and
  `round_layout_race` boot each (LVGL heap: daily 46.1, race 37.2 of 56 KB)
  and drive its rules.
- **Backlight** (screens): `global.brightness_day` and `brightness_night`, in
  percent, held to 2-100 (never quite off: the screen is how its settings are
  reached); defaults 100 and 27 (round) / 31 (cluster), the old fixed 70 and
  80 of 255 (`BRIGHTNESS_*_PCT` in HardwareConfig.h,
  `ConfigManager::backlightLevel`). The studio's sliders show a level on the
  display as they move (POST `/api/backlight`, not stored; `backlight_levels`).

## Open items

- Master power-on settle wait (`start_delay_s`, 10 s default): flashed
  2026-09-26.
- The check-engine-light work (`master/P1718_DIAGNOSIS.md`): receive-error
  guard (`rx_guard`, `rx_guard_rec`, `rx_guard_errs`; also latches on bus-off
  when `guard` is off), P2CAN wait (`obd_p2can`), request budget
  (`req_max_hz`, off by default), TX recessive hold (`tx_hold`),
  physical-addressing latch and the evidence log (portal Bus tab) are
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
  are" before changing firmware for it. Measured in the simulator 2026-11-13,
  nothing changed: with TEC held at 220 and topped up below 170, 5 % of our
  own frames failing is 1 bus-off in 10 minutes, 10 % is 113 (the car's rate)
  and 20 % is 1132; held at 184 / 168 it is 0, 43 and 931, still never
  error-active. The figures and the choices are at the top of
  P1718_DIAGNOSIS.md - the owner's decision.
- Master flashed again 2026-09-30 evening (branch portal-teach-turns-hazard:
  the portal's time-limited requests and idle-connection fix, the indicators'
  teach run, the lever field and tolerant lever reading, lamp repeats). The
  car's lever is saved as 8 bits of 0x253 ([9,28,29,48,49,51,52,54], taught
  that morning); its own field is bits 28-29 (P 3, R 2, N 1, D 0). It showed
  R instead of D above a certain speed (2026-11-14): the master now reads it
  from those two bits by itself (`lever_own_field`), not yet flashed.
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
- Communication review (2026-11-13: "Asking the car" and the poll order
  above): harness only. On the car: the Signals tab's "answers in ... ms" and
  "came late" (not 0: raise `obd_p2can`); whether the gauges fed by OBD-II -
  road speed above all, and the timers and G-meter that use it - are livelier
  (`obd_order` 1 puts the old order back); and what the code reader says about
  the transmission computer.
- Settings review (2026-11-14, the newest fixture): the owner has a corrected
  file (lever as bits 28-29, the wrong learned matches rejected, the values
  they stood for read with the SSM2 request, the two logged SSM2 values this
  ECU does not offer - Front O2 #1, Knock correction - replaced by A/F 1
  lambda and Knock corr fine). Left for the owner: both front doors on one bit
  (0x431 bit 40), SSM2 at 5 Hz against a 10 Hz log. A file of 2026-11-17
  (config5, upload pending) also moves "Brake" to 0x333 byte 2 (it overlapped
  the speed's high byte) and sets the speed to Subaru's 0.05625 km/h per bit
  (the learned -0.5 offset was OBD's whole km/h): this car's 0x333 is laid out
  like the 2017 Forester's 0x0D1 (speed bytes 0-1, brake byte 2, a flag in
  byte 3). Of this car's signal IDs only 0x002 (steering, same layout) also
  exists on the Forester - its other addresses do not carry over. The same
  for a 2012 Impreza (CANdid, 2026-11-17): its 26 identifiers are the
  newer set (0x0D0-0x0D4, 0x140-0x15A, 0x280-0x282, 0x360-0x378, 0x660) and
  only 0x002 is shared. Its annotated lock-to-lock capture confirms this
  car's mapping of 0x002: bytes 0-1 big-endian signed, 0.1 degree, left
  positive (about +-550 at full lock); byte 2 constant, byte 3 a rolling
  counter, byte 4 the sum of bytes 0-3 - nothing else in the frame. No public
  data for the 2010-2014 Legacy/Outback was found (open-can-db has the
  2005-2009 one only). The learner rules above are harness only; on the
  car, the serial log's "no longer agrees with the ECU" lines say what a
  re-check turned down.
- RAM (2026-11-16): harness, build and QEMU (`master/test_qemu/README.md`:
  before and after side by side, a 70-identifier bus, a 10 minute soak, the
  tables read from the other core - no crash, no inconsistency). On the car: the boot line
  `learner started: N fields x 20 values ... (X KB, Y KB left)` (N should be
  higher than before), `/api/live` `heap_min` after a drive with the portal
  used and settings saved (how much of the 95 KB reserve is never touched),
  and `stacks` - the figures a smaller `HEAP_RESERVE` or smaller stacks would
  have to rest on. Not done, for want of those: the stacks, the reserve, the
  live page's JSON document (estimated at 20-30 KB for a moment, every second
  the portal is open - not measured).
- Learner, found by replaying a real 2017 Forester's traffic in QEMU
  (2026-11-17, `master/test_qemu/README.md`), not fixed: engine speed learned
  as 12 bits of a 14-bit field because the drive stayed under 4,096 rpm (the
  two fields read the same until then; above it the 12-bit one wraps, and only
  the re-check would catch it); road speed taken from one wheel. (The field
  table filling within seconds, which kept late-moving fields out for good, is
  fixed: `learn_late_field` above.) This car's engine
  speed is 16 bits wide and is not affected.
- Codes, oil change, G-meter, drive log and timers (above): harness only. On
  the car: whether the TCM answers 0x7E1 (or only 0x7DF) and lists P1718 under
  service 03 at all; that the SSM2 odometer matches the cluster; which frame
  carries the VDC unit's G and yaw (teach it); and how many CAN frames each
  4 KB log write costs (Bus tab misses while logging).

More detail: `master/BRINGUP.md`, `master/MASTER_NODE_BLUEPRINT.md`, each
screen's README, and each `test_host/README.md`.
