# P1718 / P0700 — the check-engine light, diagnosed from the code

The car sets **P1718** ("AT CAN communication circuit": the transmission ECU
stopped receiving the engine ECU's CAN messages) and **P0700** (the TCM asking
for the lamp because of P1718). Both first appeared after the master was
installed, never before; the terminator on the transceiver module is removed;
the codes come back sometimes at the start of a drive and sometimes after 30
to 60 minutes of driving.

This file is the diagnosis, what was changed, the flash checklist and the
test plan. The simulator work is in `test_host/tests/scenarios.cpp`
(`tcm_p1718` and the scenarios after it) and the firmware changes in
`src/main.cpp` and `src/MasterConfig.*`.

## Ranked causes and the evidence for each

The question is narrow: what can the master do that takes the engine ECU's
broadcast frames away from the transmission ECU?

**1. Error signalling in normal mode on a marginal link (suspect 2) — most likely.**
In normal mode the CAN controller does not only transmit: it ACKs every frame
and, when *it* detects a bit, stuff, CRC or form error in *any* frame, it
transmits an error flag, which destroys that frame for every module on the
bus — the TCM included. If the master's link is marginal (stub, ground,
sample point), it sees errors in frames the rest of the bus receives fine, and
in normal mode it actively signals them. Nothing in the firmware watched for
this: the bus guard only ever looked at the transmit error counter (errors in
*our* frames), and it is turned off in the car's settings anyway. A rising
*receive* error counter — the controller telling us it is flagging errors in
frames it only receives — went unseen. Evidence: the codes never appeared
before the install; they come back after long drives, a sustained-operation
signature rather than a boot one; the portal showed 61 bus errors and the bus
"felt unstable" under heavy SSM2. In the simulator, with the car's own
settings file and a marginal link, the TCM sets P1718 within seconds; with the
new receive-error guard it never does. Listen-only mode cannot send an error
flag or an ACK, which is why it is the fallback.

**2. A reboot during cranking, with the TX line not held recessive (suspects 3 and 4).**
Sleep is disabled, so the master is powered through every crank on the
always-live pin via the buck converter; a voltage dip that browns the ESP32
out restarts it while every module on the bus is starting. Two things follow.
The settle wait that would keep it listen-only for those seconds is not on the
car yet. And nothing held GPIO 5 (transceiver TXD) recessive across the reset:
if TXD floats or sits low while the chip is down, the transceiver drives the
bus dominant and jams it. There was no record of reset reasons, so this could
never have been seen. Evidence: the "at the start of a drive" recurrences.
What the simulator shows: a brown-out boot is now recorded, the settle wait
holds the controller listen-only for 10 s after the bus comes up, TX is driven
recessive the instant the firmware runs and latched recessive across sleep.
What only the car can show: whether the buck actually drops out at cranking
(the evidence log's boot reasons will say), and whether TXD floats low during
reset (a 10 kΩ pull-up from TXD to 3V3 covers it whatever the answer).

**3. Request pacing below P2CAN with no ceiling (suspect 1).**
`obd_to` is 20 ms; ISO 15765-4 gives the ECU 50 ms (P2CAN) to answer. An
answer at 35 ms was timed out at 20 ms and, with `obd_gap` 10 ms, the next
request went out 30 ms after the last — into the reply the ECU was still
sending. And with 66 PIDs enabled at periods down to 50 ms there was no
overall cap. On its own this does not take frames from the TCM (0x7E0 loses
arbitration to 0x231), but it is a steady source of exactly the errors that
cause 1 turns into destroyed frames. Simulator: with a 35 ms ECU the gap
between requests was 30 ms; it is now at least 50 ms, and the stored `obd_to`
is untouched.

**4. Functional 0x7DF requests reaching the TCM (suspect 5) — low.**
On this car the engine ECU answers physical 0x7E0, so the steady state was
already physical-only. But with addressing on auto, one missed physical probe
switched the master to 0x7DF for the rest of the session — and the TCM answers
0x7DF. Simulator: a transient miss now leaves it physical; before, it did not.

**5. Not software (suspect 6).**
The terminator is confirmed removed, which takes the commonest hardware cause
off the list. Stub length, the transceiver ground and the sample point cannot
be judged from here — they are what makes cause 1 fire, and the evidence log
and step 1 of the test plan tell them apart from a plain car fault.

## What changed

Everything is an addition. Nothing the master did on the bus is removed or
reduced; every measure has a switch and a value in the portal (Settings or
Advanced), and every stored setting and learned signal survives: the
`device_config` scenario replays the car's file and still shows 0
differences. `CFG_VERSION` stays at 5. No stored value is overridden — in
particular **`obd_to` stays at 20**: rather than raising it to 50, a separate
post-timeout wait (`obd_p2can`) was added, so the reply wait you chose is
kept and the ISO spacing is enforced on top. Raise `obd_to` yourself if you
prefer that.

| Fix | Setting (default) | What it does | Scenario |
|---|---|---|---|
| Receive-error guard | `rx_guard` (on), `rx_guard_rec` (96) | When the controller's receive error counter climbs to the error-warning level — it is error-flagging other modules' frames — switch to listen-only for the session. Independent of the transmit-side `guard` (which stays off as you set it). Resume from the portal. | `tcm_p1718`, `tcm_p1718_optout`, `rx_guard_trickle` |
| P2CAN wait | `obd_p2can` (50 ms) | After a request that got no answer within `obd_to`, the next request waits until this long after the send. Delays, never drops. | `pacing_p2can` |
| Request budget | `req_max_hz` (0 = off) | Ceiling on OBD-II requests per second; spreads them, never drops. Off by default so nothing is throttled that you did not choose; 40 is a sensible value for the car. | `budget` |
| Settle wait | `start_delay_s` (10) | Already built; this branch puts it on the car. Listen-only for 10 s each time the bus comes up. Absent from the stored file, so it takes the default. | `settle*`, `crank_reset` |
| Physical addressing latch | (none: behaviour) | Auto addressing gives physical several tries before ever using 0x7DF, and never goes back once physical has worked. | `no_functional` |
| TX recessive hold | `tx_hold` (on) | GPIO 5 driven recessive the instant the firmware starts (always) and latched recessive across deep sleep (this switch). | `tx_hold`, `tx_hold_off` |
| Evidence log | Diagnostics tab, `/api/evlog` | Boot and reset reasons, bus up, settle end, guard trips with the error counters and what we were transmitting, bus-off, sleep. Kept in `/evlog.json` on LittleFS across reboots and updates. | `crank_reset`, `evlog` |

The simulator gained a transmission ECU that watches the engine ECU's
broadcasts (0x231, 0x232 stand in for them) and latches P1718 when fewer than
30 of the ~150 expected arrive in 1.5 s on a live bus; a marginal-link fault
under which a normal-mode master corrupts a fraction of those frames (and its
REC rises, as a real controller's does); REC that winds down on good frames;
and mocks for the reset reason and the GPIO hold.

`tcm_p1718_optout`, the `obd_p2can: 0` phase of `pacing_p2can` and
`tx_hold_off` run each fix switched off: they are the pre-fix behaviour,
reproduced and kept in CI — the TCM sets P1718, the requests crowd the ECU,
the TX line is not latched.

## What could not be confirmed without the car

- That the link is marginal at all. The receive-error guard's trips in the
  evidence log, or a listen-only drive with no lamp, are the confirmation.
- That the ESP32 browns out at cranking. The evidence log's boot reasons say.
- That the transceiver's TXD floats low during reset. Hardware; fit the
  pull-up regardless.
- The real TCM's P1718 criteria and which frames it needs. The model uses the
  RPM and throttle frames the learner found and a threshold that trips on a
  real gap, not on a stray loss.

## Flash checklist (on your PC)

1. `git fetch && git checkout claude/p1718-check-engine-light-ha776b`.
2. Full backup first: `esptool.py --chip esp32s3 read_flash 0 0x1000000 master-backup.bin`,
   then `esptool.py --chip esp32s3 verify_flash 0 master-backup.bin`.
3. Check the partition table the build uses matches the board
   (`platformio.ini`: 16 MB, LittleFS) — a wrong one boot-loops.
4. Plain `pio run -d master -t upload`. Never `uploadfs`.
5. Read the settings back: open the portal, Advanced → "Everything, as JSON"
   → Download, and compare with `test_host/fixtures/car_config_2026-09-25.json`
   (the same 18 signals, 72 SSM rows, 66 PIDs, `obd_to` 20, `guard` false).
6. Hardware while you are there: a 10 kΩ pull-up from the transceiver's TXD
   to 3V3; the transceiver GND to OBD pin 4/5; stub under 30 cm.
7. Portal → Settings: "Receive-error guard" on (it is, by default); Advanced:
   `req_max_hz` 40 if you want the budget. Diagnostics → Clear.
8. First start: the portal pill says "settling · N s" for 10 s, then "live".

What is different on the car after the flash, without touching your file:
the 10 s settle wait, the receive-error guard, the 50 ms post-timeout wait
and the TX hold are on; the evidence log runs. `obd_to` 20, `obd_gap` 10,
`guard` off, `sleep_enabled` off and `obd_addr` auto are as you left them.

## Test plan — one step per couple of days, clear the codes before each

**1. Master unplugged.** Codes come back → the car has a fault of its own and
the master is not it; stop here. No codes → the master is involved; go on.
(Flash this branch now, so steps 2–4 have the evidence log.)

**2. Listen-only** (Settings: both request switches off; the pill says
"listening only"). The controller cannot transmit, ACK or send an error flag.
No codes → an active-mode behaviour is the cause (1, 3 or 4 above); go on.
Codes anyway → only the reset window is left on our side: open Diagnostics
and look for brown-out boots; fit the pull-up if it is not on; if the log shows
clean power-ons only, it is the wiring or the car (back to 1).

**3. OBD-II with the fixes** (Settings: OBD-II on, SSM2 off — the mode as
today). After every drive read Diagnostics. Receive-error guard trips → cause
1 confirmed: the link is marginal; fix the ground and stub, try the late
sample point (Advanced, reboot), and the guard has meanwhile kept the TCM
safe. Brown-out boots → cause 2 confirmed; the settle wait covered it. No
trips, no brown-outs, no codes → fixed; go on. Codes with a clean log → lower
`rx_guard_rec` (64) and set `req_max_hz` 40, repeat.

**4. SSM2 on** (Settings: both on; mode Auto). Codes only now → the SSM2
traffic pattern is the trigger: keep OBD-II only, or raise `ssm_gap` and lower
`ssm_batch`. No codes → done, everything on.
