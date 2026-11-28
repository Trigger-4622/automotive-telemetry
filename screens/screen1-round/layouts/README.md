# Round-screen layouts

Two complete layouts for the round display, each uploaded to a display from
Gauge Studio: **Device -> Layout file -> Upload to the display...**, then let it
restart. The display keeps its own touch calibration, and every network
setting the file does not mention (the master's channel, its AP password), so
an upload never takes a screen off the master.

| File | For | What it does |
|---|---|---|
| `daily.json` | every day | The shipped screens to swipe through (dash, tach, coolant, electrical, powertrain, overview, G-meter, timers, diagnostics). The car seen from above comes up only when parking: in **P** the car with its doors, boot and lights; in **R** the car with the path its corners will take, back on the previous screen 10 s after R or at once above 15 km/h. Neither is in the swipe order. No network block. |
| `race.json` | the track | The watchdog guards only **RPM over** (shift 6200, over-rev 6500) and **coolant over** (105 / 112 °C) and never draws a card over the screen. They show instead on the lamp strip (SHIFT, OVER-REV, ENGINE HOT; the strip stays up) and by bringing their screens up: the big tach at over-rev, the coolant gauge when hot. A launch (standing, engine above 2500 rpm) brings the timers (0-100, 80-120, quarter mile); hard braking (-0.7 g) the G-meter. Screens: race dash (shift light from 4800, gear, speed, oil/IAT/boost/timing), big tach, coolant, boost/timing/knock/throttle, coolant/oil/IAT/AFR, G-meter, timers, a 30 s boost trend, diagnostics. Its AP is named `Telemetry-Gauge-Race`, so the two displays are told apart. |

Oil pressure and charging lamps still pop up on the race layout: losing either
on a track ends the engine. The shift points are for a petrol EJ with a 6500
rpm redline - change `RPM` in the watchdog, the lamps `custom_1`/`custom_2`,
the tach's thresholds and the dash's `shift_light` for yours (or in the
studio).

The screen harness boots both (`round_layout_daily`, `round_layout_race`):
LVGL heap 46.1 and 37.2 KB of 56, and every rule above driven.
