# Vehicle Telemetry System {#mainpage}

A two-node automotive telemetry system. A **master** node reads the vehicle's
CAN bus and broadcasts parsed engineering values over ESP-NOW; any number of
**slave** display nodes listen and render configurable gauges on a 1.28"
circular LCD.

@image latex none

## The two nodes

| | Master — "Data Firehose" | Slave — "Intelligent Canvas" |
|---|---|---|
| Hardware | ESP32 + SN65HVD230 CAN transceiver | ESP32-C3 all-in-one round display board |
| Knows the vehicle | **Yes** — CAN IDs, PIDs, scaling | Never |
| Knows the UI | Never | **Yes** — layout.json, gauges, watchdog |
| Radio | Broadcast-only transmitter | Receive-only listener |
| Source | `master/` | `screens/screen1-round/` |

The only coupling between them is @ref MasterPacket.h, a packed binary
contract that must stay byte-identical in both projects.

## Slave module map

The slave's `src/` tree is grouped by responsibility:

- @ref hal — hardware abstraction: panel, touch, button
  (@ref DisplayManager, @ref TouchManager, @ref ButtonManager)
- @ref telemetry — the radio link and the value cache
  (@ref NetworkManager, @ref TelemetryStore)
- @ref ui — the dynamic gauge engine (@ref UIBuilder)
- @ref config — persistence and the Gauge Studio web portal
  (@ref ConfigManager)

Every GPIO in the system is declared in exactly one place,
@ref HardwareConfig.h. No pin number appears anywhere else.

## Data flow

    Vehicle CAN ──► Master: parse to engineering units ──► ESP-NOW broadcast
                                                                │
                        ┌───────────────────────────────────────┘
                        ▼
    NetworkManager (Wi-Fi task) ──► TelemetryStore (EMA + lerp smoothing)
                                                                │
                                        UIBuilder (LVGL, 50 fps) ┘

`NetworkManager` runs its receive callback in the Wi-Fi task while `UIBuilder`
renders from the Arduino loop, so @ref TelemetryStore guards its slots with a
spinlock. That boundary is the only place in the slave where two execution
contexts meet.

## Two alarm paths — do not confuse them

1. **Per-gauge thresholds** (`thresholds` on a screen in `layout.json`) are
   cosmetic. They recolor and pulse one widget on one screen.
2. **The watchdog** (`watchdog.items`) is the safety guard. It polices a list
   of metrics whether or not any screen displays them, and raises a
   warning-triangle card over whichever screen is showing. See
   @ref UIBuilder::evalWatchdog.

Watchdog alarms are evaluated on **raw** values via
@ref TelemetryStore::peek, never on the smoothed display value — a guard must
react to what the sensor reported, not to a display filter that lags it.

## Where to start reading

- @ref MasterPacket.h — the wire contract, metric ID registry and flag bits
- @ref HardwareConfig.h — every pin and board constant
- `CanDecoderConfig.h` — the only file to edit when porting to a new vehicle
- `master/MASTER_NODE_BLUEPRINT.md` — full engineering rationale for the master
- `README.md` — build, flash and day-to-day operation
