# Host test harness

Runs the **unmodified master firmware** (`src/main.cpp`, `Ssm2.cpp`, `Learner.cpp`,
`MasterConfig.cpp`) on the PC, inside a simulated car, with undefined-behaviour
trapping and bounds-checked containers.

```
python test_host/run.py                # build + every scenario (about 30 s)
python test_host/run.py auto guard     # just these
python test_host/run.py auto -v        # one scenario with the full firmware log
SIM_SEED=5 python test_host/run.py     # a differently shaped drive
```

Needs `g++` (WinLibs GCC: `winget install BrechtSanders.WinLibs.POSIX.UCRT`) and a
`pio run` done once (for ArduinoJson in `.pio/libdeps`). Debug a failure with
`gdb --args test_host/build/sim.exe <scenario>`.

| Part | What it is |
| --- | --- |
| `mocks/` | Arduino, FreeRTOS, TWAI, LittleFS, Wi-Fi/ESP-NOW headers, same API as ESP-IDF 4.4 |
| `sim/sim_rtos.cpp` | Deterministic scheduler: one task runs at a time, virtual time jumps to the next wake-up, so a 4-minute drive takes ~2 s and every run is reproducible. It aborts on deadlock, on a task that spins without blocking, and on blocking inside a critical section |
| `sim/sim_bus.cpp` | TWAI controller (modes, error counters that wind down on good frames, bus-off, alerts), the car's broadcast frames, an engine ECU speaking OBD-II and SSM2 over real ISO-TP, a transmission ECU that answers 0x7DF and watches the engine ECU's broadcasts (it sets P1718 when they thin out), fault injection including a marginal link on which a normal-mode master error-flags - destroys - those broadcasts |
| `sim/sim_platform.cpp` | FreeRTOS queues and semaphores (flags mutex misuse), in-memory LittleFS (short writes, rename quirks), ESP-NOW decoded as a display would |
| `tests/scenarios.cpp` | The scenarios and their checks |

What it cannot show: preemption in the middle of code that never blocks,
real electrical bus behaviour, and radio timing. Those need the car.
