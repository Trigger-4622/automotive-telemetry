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
| Every source file in `../src`, same compiler flags | The frames the CAN controller would receive: a car of 25 identifiers (more with `--extra-ids`), four of them carrying values that step between levels | The controller's error counters (its registers read 0: the listen-only pin and "errors let pass" report that they did not take) |
| All ten tasks, their stacks, the heap | An engine computer answering OBD-II service 01 for five PIDs with the same values | Wi-Fi, ESP-NOW on the air, the portal (QEMU has no radio; the portal's page handlers are not even linked) |
| The learner, the verifier, the census, the broadcaster's packing | ESP-NOW: a packet is counted and reported as sent | The settings file: QEMU's flash refuses LittleFS with this ESP-IDF (4.4.7), so the master runs on defaults and every save fails after building its document |

So the heap figures are **without Wi-Fi's share** (some 50-60 KB on the board,
plus 20 KB of its static RAM), and nothing about the bus's electrics, the
radio, the portal or the settings file is shown. `device_config` and the rest
of the host harness cover the settings; the car covers the rest.

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
```

It builds (`platformio.ini` here: the sources of `../src`, the shim beside
them), makes a 16 MB flash image, runs it and reads the serial log: booted
once with no panic or restart, every identifier in the census, frames still
arriving, values learned from the bus and verified against the ECU, the heap's
lowest point, each task's least stack left.

Not part of `scripts/test_all.py` or CI: it needs the QEMU download.

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
