# The master in Espressif's QEMU

The host harness (`../test_host`) runs the firmware's logic on the PC, one task
at a time, with a heap that never runs out. This runs the **real compiled
firmware** on an emulated ESP32-S3 instead: the real instruction set, both
cores, FreeRTOS, the real allocator and the real stacks. It is the only thing
here, short of the board, that can show a stack running low, a heap figure, or
two tasks on two cores sharing a table.

**It is a test build. Never flash it** - it has no radio and no CAN bus.

## What is real and what is not

| Real | Stood in for (`shim/qemu_shim.cpp`, put in by the linker) | Not there at all |
|---|---|---|
| Every source file in `../src`, same compiler flags | The frames the CAN controller would receive: a car of 25 identifiers (more with `--extra-ids`), four of them carrying values that step between levels, or a recorded drive (`--replay`) | The controller's error counters (its registers read 0: the listen-only pin and "errors let pass" report that they did not take) |
| All ten tasks, their stacks, the heap | An engine computer answering OBD-II service 01 for five PIDs with the same values | Wi-Fi, ESP-NOW on the air, the portal (QEMU has no radio; the portal's page handlers are not even linked) |
| The learner, the verifier, the census, the broadcaster's packing | ESP-NOW: a packet is counted and reported as sent | The radio's share of the heap, flash timing (QEMU's flash writes take no time) |
| The flash: LittleFS on the settings and logs partitions, every save, the drive log; with `--ota` the board's own partition table, the bootloader, otadata and its rollback | The clock: each boot's starts at 0, as the board's does (QEMU's runs on through a restart) | |

So the heap figures are **without Wi-Fi's share** (some 50-60 KB on the board,
plus 20 KB of its static RAM), and nothing about the bus's electrics or the
radio is shown.

Two things about QEMU (Espressif's 9.2.2) itself, found on 2026-11-17:

- **It reads flash two bytes off in quad (QIO) mode** - the firmware's own
  reads, not the boot loader's - so no file system could ever be read and
  every save failed. Dual (DIO) reads are right: the test build uses
  `board_build.flash_mode = dio` (the master's own build stays QIO).
- **Now and then QEMU itself dies of a segmentation fault**, early in a boot
  that formats or writes the flash: inside its generated code, on one of the
  two cores' threads (gdb), not in the firmware. `run.py` runs it again once
  and says so. With both cores on one thread (`-accel tcg,thread=single`) it
  does not happen, but the second core never comes back after a software
  restart and the firmware waits for it for ever - so `--ota` cannot use it.

## Running it

QEMU is Espressif's fork (upstream QEMU has no ESP32). The release used here:
`qemu-xtensa-softmmu-esp_develop_9.2.2_20250817-x86_64-linux-gnu.tar.xz` from
Espressif's download server (`dl.espressif.com/github_assets/espressif/qemu/releases/download/esp-develop-9.2.2-20250817/`);
on Ubuntu it also needs `libslirp0`.

```bash
export QEMU_ESP32S3=/path/to/qemu/bin/qemu-system-xtensa
python master/test_qemu/run.py                       # 120 s, this car's 25 identifiers
python master/test_qemu/run.py --extra-ids 45        # a busier bus: 70 (census rows from the heap)
python master/test_qemu/run.py --seconds 600         # a soak
python master/test_qemu/run.py --readers             # + a task on the other core reading every table
                                                     #   as the portal's pages do, 50 times a second
python master/test_qemu/run.py --src ../other-checkout/master   # the same test on another version
python master/test_qemu/run.py --ota                 # the firmware update over Wi-Fi, twice, and a rollback
```

It builds (`platformio.ini` here: the sources of `../src`, the shim beside
them), makes a 16 MB flash image, runs it and reads the serial log: booted
once with no panic or restart, every identifier in the census, frames still
arriving, values learned from the bus and verified against the ECU, the heap's
lowest point, each task's least stack left.

Not part of `scripts/test_all.py` or CI: it needs the QEMU download.

### A recorded car instead of the made-up one

```bash
python master/test_qemu/run.py --replay /path/to/candump.log [--settings settings.json] [--readers] [--seconds 450]
```

`--replay` takes a candump log (`(1672868911.675546) can0 140#04042803000411A0`),
puts it in the firmware image (`shim/replay.bin`, not committed: about 4 bytes
a frame - only the bytes that changed since that identifier's last frame - so
a 40-minute drive is 11.4 MB; the test build has a 12.5 MB app partition for
it) and plays it at its own pace, again from the top at its end.

`--settings` puts a settings file in the image (`shim/settings.json`, not
committed) and applies it right after the master has loaded its own, as the
portal's "Everything, as JSON" upload does: `fromJson(.., true)` and a save. The simulated ECU answers coolant, engine speed,
road speed and the accelerator pedal **from the frames just played**, read with
the layout of Subaru's 2005-2019 cars (engine speed and pedal in 0x140, road
speed in 0x0D1, coolant in 0x360), so the learner has to find real fields in
real traffic. A log of another make plays too, but its ECU answers nothing true.

Recordings used (not in this repository): Brooke Lampe and Weizhi Meng's
*can-log* / *can-train-and-test* dataset, CC BY 4.0, 2017 Subaru Forester,
`attack-free-1..4.log` - bitbucket.org/brooke-lampe/can-log, also at
data.dtu.dk (articles 24805509 and 24805533). And *CANdid* (University of
Adelaide, CC BY 4.0, doi.org/10.25909/29068553): vehicle 6, a 2012 Subaru
Impreza (automatic), `6_driving_CAN.log` - 40 minutes (2,374 s, 2,695,619
frames, 26 identifiers), town first and then a faster road up to 91 km/h.

### The firmware update over Wi-Fi

`--ota` builds the test firmware with the board's own partition table
(`../partitions_16mb.csv`: app0, app1, the settings where they are on the
board, 8 MB of drive logs) and, from the loop task where the portal's upload
handler runs, hands this firmware's own image to `fwUpdateBegin / Write / End`
in the 1436-byte pieces Arduino's WebServer delivers, then restarts as
`/api/update` does. The rest is the real bootloader's: it boots the update on
trial; the update is kept after 30 s; a second update is made and crashes on
purpose 15 s in, before it is kept; the bootloader goes back to the kept one,
which reports the rollback. The settings and learned values must load the
same at every boot. The portal is not started (no radio) but marked as up, so
a firmware on trial can be kept.

## 2026-11-16: the RAM change (PR #11), before and after

Same simulated car, 150 s each.

| | Before (main 4e99beb) | After |
|---|---|---|
| Boots, panics, restarts | 1, 0, 0 | 1, 0, 0 |
| Identifiers in the census | 26 of 26 | 26 of 26 |
| Frames a second | 1007 | 1007 |
| Learned and verified | RPM, speed, throttle, load | the same four |
| Learner's tables | 170 fields, 213 bits | 197 fields, 246 bits |
| Heap left after them | 87 KB | 87 KB |
| Heap's lowest point (a save's document) | 52 KB | 56 KB |

With 70 identifiers: all in the census, two blocks of 16 rows taken from the
heap, 1908 frames a second, the same four values learned.

A 10 minute soak (about 600,000 frames): one boot, no panic, five values
learned and verified (the coolant once it had warmed far enough), heap's
lowest point 52 KB.

`--readers` on the 70-identifier bus, 240 s: 10,643 passes over the census
(in identifier order, with the flip counters), the metric store, the learner's
rows, the stacks and the event log from core 0 while the receive task filled
them on core 1 - no inconsistency.

Least stack left after the 150 s (bytes of size): twai_rx 3844/6144,
obd_poll 2392/4096, broadcast 3284/6144, housekeep 3288/4096,
busguard 2248/4096, loopTask 8828/12288, ssm2 3992/6144, codes 3320/6144,
learner 3364/6144, drivelog 5300/6144. Without the portal, saves that reach
the flash, SSM2 answers or trouble codes - the car's own `[stack]` line is the
figure to cut a stack by, not this one.

## 2026-11-17: a real Subaru's traffic

`attack-free-4.log`: a 2017 Forester, 208 s of driving, 336,011 frames
(1,614 a second), **54 identifiers**, frames of 1, 2, 4, 5, 7 and 8 bytes.
450 s of it (twice round), before and after the RAM change:

| | Before (main 4e99beb) | After |
|---|---|---|
| Boots, panics, restarts | 1, 0, 0 | 1, 0, 0 |
| Identifiers in the census | 55 of 55 | 55 of 55 (one block of 16 rows from the heap) |
| Frames a second | 1624 | 1623 |
| Engine speed | 0x140 bit 16, 12 bits - verified | the same |
| Road speed | 0x0D4 bit 48, 16 bits - verified | the same |
| Coolant | 0x360 byte 3 - verified | the same |
| Rejected by the verifier | 0 | 0 |
| Learner's tables | 170 fields, 213 bits | 197 fields, 247 bits |

With `--readers` (240 s): 10,740 passes over the tables from the other core,
no inconsistency.

What the real traffic showed that the made-up car did not:

- **The learner's field table is full two seconds after the bus comes up**
  ("field table full, evicting as it goes"): 54 identifiers make far more
  candidate fields than 170 or 197 slots. Every slot the RAM change frees is
  used.
- **A car of this generation has more than 48 identifiers** - the census takes
  one block from the heap (2 KB), as designed.
- **Engine speed was learned 12 bits wide; the car's field is 14** (the
  dataset's own table, from opendbc: `Engine_RPM : 16|14@1+`). The drive never
  goes above 2,541 rpm in any of the four recordings, so bits 12 and 13 never
  move and the 12- and 14-bit fields read the same; with the table full, which
  of the two is in it is chance. Above 4,095 rpm the 12-bit one starts again
  from 0. The re-check against the ECU (`learn_recheck_s`) would turn it down
  after four disagreements in a row - two minutes above 4,095 rpm, or never if
  the engine is only up there for moments. Older than the RAM change (both
  versions do it); not fixed here. The owner's car is not affected today: its
  engine speed is learned 16 bits wide.
- Road speed was taken from one wheel's speed (0x0D4), not the car's own
  figure (0x0D1): the two fit the ECU's whole km/h equally well. Right in a
  straight line, a little off in a corner.

