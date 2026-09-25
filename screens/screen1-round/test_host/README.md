# Display test harness

Runs the display's **real UI code** — `src/ui` (UIBuilder, Telltales),
`src/telemetry/TelemetryStore.cpp` and `src/config/ConfigManager.cpp` — on the
PC, on the real LVGL from `.pio/libdeps`, drawing into a framebuffer, with a
simulated master broadcasting telemetry. Identical in both display projects.

```
python test_host/run.py                   # build + every scenario (a few seconds)
python test_host/run.py lamp_seatbelt     # just these
python test_host/run.py lamp_mil_once -v  # one scenario with the full firmware log
```

Needs `g++` (WinLibs GCC: `winget install BrechtSanders.WinLibs.POSIX.UCRT`) and
one `pio run` beforehand (for LVGL and ArduinoJson). Screenshots of every screen,
the dash, the lamp strip and the alert cards land in `test_host/shots/`.
Debug a crash with `gdb --args test_host/build/ui.exe <scenario>`.

| Part | What it is |
| --- | --- |
| `mocks/` | Arduino (String, millis), LittleFS in memory with short-write and no-overwrite-rename faults, WebServer (handlers called directly), Wi-Fi, the HAL singletons |
| `host/lvgl_host.cpp` | Framebuffer display, PNG writer, the simulated master, virtual time |
| `host/lv_budget.cpp` | What the LVGL heap would hold **on the ESP32** — see below |
| `tests/ui_tests.cpp` | The scenarios |

## LVGL heap on the device

LVGL's heap is a fixed pool (`LV_MEM_SIZE` in `include/lv_conf.h`), and running
out of it is not an error LVGL reports: the next widget comes back null and the
display reboots. The PC build has 8-byte pointers, so its own heap figures
overstate the device's by about half. The harness therefore walks every live
LVGL object and adds up the blocks it owns using the **32-bit** struct sizes
(`host/lv_sizes.c`, compiled with `-m32`), rounded the way LVGL's TLSF allocator
rounds on the ESP32. `boot_all_screens` prints the estimate per screen and fails
when less than 8 KB would be left. The firmware also logs the real figure at
boot, and the studio's System tab shows it.

What it cannot show: real touch, real panel timing, and the radio. Those need
the car.
