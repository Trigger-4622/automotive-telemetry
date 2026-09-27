# Master node bring-up — 2009 Subaru Legacy B4

Work through this in order. Each step proves one thing, so when something
fails you know which thing it was.

## 0. Before the car

Flash it on the bench first and confirm the serial console comes up:

```
pio run -t upload
pio device monitor
```

Every 5 s you should get a heartbeat, a diagnostics line and a bus census:

```
[stats] CAN rx: 0 | OBD resp: 0 | SSM2: 0 ok/0 err (0/s, 40 addr) | ESP-NOW tx: 100 (fail 0) | night: 0 | heap: ...
[diag ] mode AUTO | SSM2 idle | OBD-II idle
[bus ] no CAN frames yet - check CANH/CANL, bitrate, and that the ignition is on
```

`ESP-NOW tx` climbing with no CAN attached is correct and expected — the
broadcaster runs regardless, it just has nothing to say yet.

**Write down the MAC** printed at boot (it is the AP interface's MAC, which
is what the displays see):

```
ESP-NOW broadcasting on channel 1 via AP, this master is 9A:A3:16:EB:F9:34
```

Paste it into each display's Settings → `network.master_mac` if you want them
to ignore everything else on the channel. Optional, but it stops a second CAN
project injecting into your gauges. With its Wi-Fi switched off at boot
(Advanced → *Start this Wi-Fi at boot*) the master sends from its station MAC
instead - the boot line says which - so set the filter again after changing
that, or leave it empty.

## 1. Plug in — it configures itself

The diagnostic mode ships as **Automatic**. With the ignition on:

1. The census fills — wiring and bitrate are right. For the first 10 s the
   master only listens while the car's modules start up; the diagnostics
   line says `bus settling, requests in N s`. (Settings → *Wait after
   power-on*; 0 = no wait.)
2. The master sends the SSM2 init command. The ECU answers with its ROM ID
   and a bitmap of every parameter it supports; the master enables exactly
   those and never asks for the rest. Watch for:
   ```
   SSM2: ECU answered - SYS A21007 ROM 4B12785206, 96 capability bytes, 58 of 69 seeded parameters supported
   ```
3. Exchanges run back to back. The values a needle follows (RPM, MAP,
   throttle, timing, knock, A/F) go in every request; temperatures and
   voltages rotate through the remaining slots. Expect 20-40 exchanges/s.
4. If the ECU never answers SSM2, the master falls back to OBD-II by itself,
   asks which PIDs the ECU supports, and polls only those.

Open the portal (`Telemetry-Master-Config`, 192.168.4.1). The **Live** tab
says which engine is talking, the ECU ID, and how many parameters this ECU
supports; the **Subaru SSM** tab marks each parameter ✓ or ✗ from the ECU's
own answer.

| What you see | What it means |
|---|---|
| IDs appear in the census | Wiring and bitrate are right. |
| Nothing, ever | CANH/CANL swapped or not connected, termination jumper still fitted, transceiver not powered, or RS not grounded. |
| Bus alive, `SSM2 no answer`, `OBD-II active` | This ECU does not speak SSM2 over CAN (K-line only). OBD-II data flows; the Subaru extras are unavailable on this port. |
| Bus alive, neither answers | Wrong bus (a body CAN rather than the OBD port) or the ignition is off. |
| `SSM2: negative response … code 0x..` | The ECU refused a request. Usually one address it does not have; the parameter list will settle after the init. |
| Request size shrinking in the log | The ECU tolerates fewer addresses per request than the maximum; it settles by itself. Lower "Max addresses" in the portal if it keeps oscillating. |

## 2. Raw frames verify themselves

The **Signals** tab holds Subaru broadcast frames documented for the next
generation of this car. They start in **Auto**: decoded but not published.
Each decoded value is compared with the ECU's own figure (SSM2/OBD-II); after
30 agreeing samples across a wide enough range the signal becomes
**Verified** and starts publishing — at the frame's own rate, 50-100 Hz —
and the other Auto signals of the same frame (brake pedal alongside speed,
oil temperature alongside coolant) are verified with it. Disagreement marks
it **Rejected**.

Drive for a few minutes. Then look at the Signals tab: anything Verified is
now a confirmed fact about this car; anything Rejected was not this car's
layout. Nothing wrong can reach a gauge in between.

The headlight bit (0x152) is verified against the ECU's light-switch input,
which SSM2 already provides — so night mode works from the first drive with
no LDR and no extra wire, and the CAN bit is only a faster copy of it.

## 3. Finding more

With the census in front of you, park the car and change one thing at a
time — headlights on, brake pressed, gear selected — and watch which ID's
payload moves. Add a signal, give it a **Ref** metric if the ECU reports the
same quantity, and set it to Auto; the verifier does the rest.

## Sleep

The master deep-sleeps after 90 s of bus silence and wakes on the first CAN
frame. It stays awake while a portal client is connected, but only for a few
minutes, so a phone that remembers the AP cannot keep it awake in the
garage. The dev board's regulator and USB bridge still draw in sleep; the
chip itself is microamps.

## Safety notes

- OBD pin 16 is **permanently live**. Without the sleep logic this draws
  100-200 mA and flattens the battery over a few weeks parked.
- Fuse the 12 V tap and use an automotive-rated buck; vehicle rails see
  load-dump transients well above 12 V.
- Keep the CANH/CANL stub under ~30 cm and twisted.
- Never fit the transceiver's 120 ohm termination on a vehicle bus.
- Fit a 10 kΩ pull-up from the transceiver's TXD to 3V3. While the ESP32 is in
  reset (a brown-out at cranking, say) its pins float; a TXD left low drives
  the bus dominant and jams every module. The firmware drives TXD recessive
  the moment it starts and latches it across sleep, but only the resistor
  covers the reset window itself.
- If the car sets P1718/P0700 (transmission ECU losing the engine ECU's CAN
  messages): `P1718_DIAGNOSIS.md` has the causes, the fixes, and a test plan.
  The portal's Diagnostics tab keeps the evidence (reset reasons, guard trips).
- **Silent** mode (CAN tab) makes the controller electrically incapable of
  transmitting, for first contact with an unfamiliar bus.
- The portal is the only way into the settings (there is no serial console,
  and they survive a reflash). Switched its Wi-Fi off at boot? Hold the
  board's **BOOT** button for 3 seconds while the master runs (ignition on):
  it switches the Wi-Fi back on and restarts. A Wi-Fi name or password the
  access point cannot use is refused when saved, and should the AP still not
  start, it comes up as `Telemetry-Master-Config`, open.
