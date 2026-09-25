# Automotive Telemetry

A CAN-bus master and two gauge screens for a 2009 Subaru Legacy B4. The master
reads the car's CAN bus from the OBD port (listening, plus OBD-II and Subaru
SSM2 requests) and broadcasts engineering units over ESP-NOW. Any number of
screens listen and draw them.

```
 car CAN bus ──► master (ESP32-S3) ──ESP-NOW broadcast──► screen 1: round 1.28" (ESP32-C3)
 (OBD port)      reads, requests, learns                 └► screen 2: 4.3" dash cluster (ESP32-S3)
```

## Layout

| Folder | What it is |
|---|---|
| [`master/`](master/) | CAN master firmware: listen-first decoding, OBD-II + SSM2 requests, a learner that moves values onto broadcast frames, bus guard, web portal. [Bring-up guide](master/BRINGUP.md), [design blueprint](master/MASTER_NODE_BLUEPRINT.md). |
| [`screens/screen1-round/`](screens/screen1-round/) | Screen 1: ESP32-C3 with a 1.28" round GC9A01 touch display. [README](screens/screen1-round/README.md) |
| [`screens/screen2-cluster/`](screens/screen2-cluster/) | Screen 2: Guition JC4827W543, ESP32-S3 with a 4.3" 480×272 touch display. [README](screens/screen2-cluster/README.md) |
| [`scripts/`](scripts/) | `test_all.py` runs every test harness; `cloud-setup.sh` prepares a Linux machine or a Claude cloud session. |

Each firmware folder is a standalone PlatformIO project. They share one wire
contract, `include/MasterPacket.h`, which must stay byte-identical in all three.

## Build and flash

Needs [PlatformIO](https://platformio.org/). Each project builds on its own:

```bash
pio run -d master -t upload
pio run -d screens/screen1-round -t upload
pio run -d screens/screen2-cluster -t upload
```

A plain upload keeps each device's saved settings (the master's learned
signals, the screens' layouts and touch calibration). Do **not** run `uploadfs`
on the screens: it replaces their settings partition.

## Tests

Every project has a host test harness that compiles the unmodified firmware
for the PC: the master runs in a simulated car (CAN bus, ECU answering OBD-II
and SSM2, bus faults), and the screens render real LVGL into PNG screenshots.

```bash
python scripts/test_all.py            # everything
python scripts/test_all.py master     # or sync / screen1 / screen2
```

`sync` checks that the files the projects share are still identical and that
generated files (the master portal, the studio page) match their sources.
The harnesses need g++ and the libraries a first `pio run` downloads (or
`bash scripts/cloud-setup.sh --force` on Linux). GitHub Actions runs every
suite and builds all three firmwares on each pull request and push to `main`,
with a results table on the run page and the screen renders attached.

## Using Claude on GitHub

The repository is set up for Claude Code cloud sessions
([claude.ai/code](https://claude.ai/code)):

1. At claude.ai/code, connect GitHub and install the **Claude GitHub App** on
   this repository (it is private, so the app needs access to it).
2. Pick this repository and describe a task. Each session reads
   [`CLAUDE.md`](CLAUDE.md) for the project rules, and the session-start hook
   in `.claude/settings.json` runs `scripts/cloud-setup.sh`, so the host tests
   work in the cloud on the **Default** (Trusted) network setting.
3. For firmware builds in the cloud as well, edit the environment's network
   access: choose **Custom**, add `*.platformio.org`, and tick **Also include
   default list of common package managers**. PlatformIO downloads the ESP32
   toolchains from there.
4. Claude works on a branch and opens a pull request; CI builds and tests it.
   Turn on **Auto-fix** for the pull request and Claude also fixes failing
   checks and answers review comments.

Cloud sessions cannot reach your USB ports. To put a change on the hardware,
pull the branch on your PC and flash from there.
