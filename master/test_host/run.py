#!/usr/bin/env python3
"""Build the master firmware for the PC and run the end-to-end scenarios.

    python test_host/run.py              # build, run every scenario
    python test_host/run.py auto guard   # just these
    python test_host/run.py auto -v      # one scenario, full firmware log

The firmware sources in src/ are compiled unmodified against the mocks in
test_host/mocks (Arduino, FreeRTOS, TWAI, LittleFS, Wi-Fi) and linked with
the simulated car in test_host/sim. Undefined behaviour traps (UBSan, trap
mode) and the standard library checks its bounds (_GLIBCXX_ASSERTIONS).
Needs g++ on the PATH (WinLibs GCC: winget install BrechtSanders.WinLibs.POSIX.UCRT).
"""
import concurrent.futures as cf
import glob
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BUILD = os.path.join(HERE, "build")
EXE = os.path.join(BUILD, "sim.exe" if os.name == "nt" else "sim")

SOURCES = [os.path.join(ROOT, "src", f) for f in ("main.cpp", "Ssm2.cpp", "Learner.cpp", "MasterConfig.cpp", "DiagCodes.cpp", "DriveLog.cpp",
                                                   "FwUpdate.cpp", "RawRec.cpp", "Computers.cpp")]
SOURCES += sorted(glob.glob(os.path.join(HERE, "sim", "*.cpp")))
SOURCES += sorted(glob.glob(os.path.join(HERE, "tests", "*.cpp")))

AJSON = glob.glob(os.path.join(ROOT, ".pio", "libdeps", "*", "ArduinoJson", "src"))
FLAGS = [
    "-std=gnu++20", "-O1", "-g", "-fno-omit-frame-pointer",
    "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-missing-field-initializers", "-Wno-volatile",
    "-Wshadow=local", "-Wformat=2", "-Wnull-dereference", "-Wduplicated-cond", "-Wlogical-op",
    "-fsanitize=undefined", "-fsanitize-trap=undefined",
    "-D_GLIBCXX_ASSERTIONS", "-D__USE_MINGW_ANSI_STDIO=1", '-DFIRMWARE_VERSION="host"', "-DCORE_DEBUG_LEVEL=3",
    "-I" + os.path.join(HERE, "mocks"), "-I" + os.path.join(HERE, "sim"),
    "-I" + os.path.join(ROOT, "include"), "-I" + os.path.join(ROOT, "src"),
] + ["-I" + p for p in AJSON[:1]]


def find_gxx():
    g = shutil.which("g++")
    if g:
        return g
    # winget adds it to the PATH of new shells only; look where it installs.
    base = os.path.expandvars(r"%LOCALAPPDATA%\Microsoft\WinGet\Packages")
    for p in glob.glob(os.path.join(base, "BrechtSanders.WinLibs*", "mingw64", "bin", "g++.exe")):
        return p
    sys.exit("g++ not found - install WinLibs GCC (see the top of this file)")


def build():
    if not AJSON:
        sys.exit("ArduinoJson not found in .pio/libdeps - run `pio run` once first")
    gxx = find_gxx()
    os.makedirs(BUILD, exist_ok=True)
    env = dict(os.environ, PATH=os.path.dirname(gxx) + os.pathsep + os.environ.get("PATH", ""))
    objs, jobs = [], []
    with cf.ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as ex:
        for src in SOURCES:
            obj = os.path.join(BUILD, os.path.splitext(os.path.basename(src))[0] + ".o")
            objs.append(obj)
            jobs.append((src, ex.submit(subprocess.run, [gxx, *FLAGS, "-c", src, "-o", obj],
                                        capture_output=True, text=True, env=env)))
        warnings = 0
        for src, j in jobs:
            r = j.result()
            out = (r.stdout + r.stderr).strip()
            if out:
                print(out)
                warnings += out.count("warning:")
            if r.returncode:
                sys.exit("compile failed: " + os.path.basename(src))
    # Static: no runtime DLLs, so another MinGW on the PATH (Git has one)
    # cannot slip in an older libstdc++ or winpthread.
    r = subprocess.run([gxx, *FLAGS, *objs, "-o", EXE, "-static", "-lpthread"], capture_output=True, text=True, env=env)
    if r.returncode:
        print(r.stdout + r.stderr)
        sys.exit("link failed")
    print(f"built {os.path.relpath(EXE, ROOT)} - {warnings} warning(s)")
    return env


def scenarios(env):
    r = subprocess.run([EXE], capture_output=True, text=True, env=env)
    return [l.strip() for l in r.stdout.splitlines()[1:] if l.strip()]


CANREC = os.path.join(ROOT, "tools", "canrec.py")
# The scenarios that leave their recording (build/<name>.bin) and the candump
# log the firmware's own reading made of it.
RAWREC_SCENARIOS = ("rawrec_roundtrip", "rawrec_everything")


def canrec_check(base):
    """master/tools/canrec.py, the tool the owner runs on a download, must turn
    the recording into the same log as the scenario's own decoder."""
    out = base + "_tool.log"
    r = subprocess.run([sys.executable, CANREC, base + ".bin", "-o", out, "-q"],
                       capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        return False, "canrec.py failed: " + (r.stderr or r.stdout).strip()
    with open(out) as f:
        got = f.read().splitlines()
    with open(base + "_expected.log") as f:
        want = f.read().splitlines()
    for i, (g, w) in enumerate(zip(got, want)):
        if g != w:
            return False, "line %d: %r, expected %r" % (i + 1, g, w)
    if len(got) != len(want):
        return False, "%d lines, expected %d" % (len(got), len(want))
    return True, "%d lines alike" % len(got)


def run_one(name, env, verbose, config=None):
    """Run one scenario; @p config, when given, is the DEVICE_CONFIG for it and
    part of its label."""
    t = time.time()
    args = [EXE, name] + (["-v"] if verbose else [])
    if config:
        env = dict(env, DEVICE_CONFIG=config)
        name = "%s[%s]" % (name, os.path.splitext(os.path.basename(config))[0])
    base = os.path.join(BUILD, name)
    if name in RAWREC_SCENARIOS:
        env = dict(env, RAWREC_OUT=base)
        for f in (base + ".bin", base + "_expected.log"):
            if os.path.exists(f):
                os.remove(f)
    try:
        r = subprocess.run(args, capture_output=not verbose, text=True, env=env, timeout=600)
    except subprocess.TimeoutExpired:
        return name, 99, "TIMEOUT (hung)", 600
    out = (r.stdout or "") + (r.stderr or "")
    code = r.returncode
    if code not in (0, 1):
        why = {3: "deadlock", 4: "livelock (watchdog)", 5: "ESP.restart"}.get(code)
        if why is None and code & 0xFFFFFFFF == 0xC000001D:
            why = "undefined behaviour trapped (UBSan) - rerun under gdb"
        if why is None and code & 0xFFFFFFFF == 0xC0000005:
            why = "access violation (crash)"
        out += f"\n*** exited with {code:#x}: {why or 'abnormal exit'}"
    if name in RAWREC_SCENARIOS and code == 0:
        ok, what = canrec_check(base)
        out += "\n  %s canrec.py reads the recording as the firmware wrote it   %s" % ("ok  " if ok else "FAIL", what)
        code = 0 if ok else 1
    return name, code, out, time.time() - t


PORTAL_TESTS = os.path.join(HERE, "portal_tests.js")


def run_portal():
    """The portal page's teach-by-doing analysis, in Node (portal_tests.js):
    reported like a scenario, as "portal_teach". Node.js is part of the test
    setup, like g++ - a missing one fails the run rather than skipping it."""
    t = time.time()
    node = shutil.which("node") or shutil.which("nodejs")
    if not node:
        return "portal_teach", 1, "node not found - the portal's teach tests need Node.js on the PATH", 0.0
    r = subprocess.run([node, PORTAL_TESTS], capture_output=True, text=True, timeout=300)
    return "portal_teach", r.returncode, (r.stdout or "") + (r.stderr or ""), time.time() - t


def run_canrec_unit():
    """Verify canrec.py ECC 0x00 label, SJA1000 ECC decode, and per-session
    Bus-Off / engine-running vs KOEO summary."""
    import importlib.util
    import struct
    t = time.time()
    spec = importlib.util.spec_from_file_location("canrec_mod", CANREC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    lines = []
    fails = 0

    def chk(cond, msg, detail=""):
        nonlocal fails
        lines.append("  %s %s%s" % ("ok  " if cond else "FAIL", msg, "   " + detail if detail else ""))
        if not cond:
            fails += 1

    e0 = mod.err_name(0x00)
    chk(e0 == "cleared/unavailable (0x00)", "ECC 0x00 prints cleared/unavailable (0x00)", e0)
    chk("BIT/TX/SOF" in mod.err_name(0x03), "ECC 0x03 decodes BIT/TX/SOF", mod.err_name(0x03))
    chk("FORM/TX/tolerate_dominant" in mod.err_name(0x53), "ECC 0x53 decodes FORM/TX/tolerate_dominant", mod.err_name(0x53))
    chk("BIT/TX/ID28-21" in mod.err_name(0x02), "ECC 0x02 decodes BIT/TX/ID28-21", mod.err_name(0x02))
    chk("BIT/TX/SRTR" in mod.err_name(0x04), "ECC 0x04 decodes BIT/TX/SRTR", mod.err_name(0x04))
    chk("BIT/TX/DATA" in mod.err_name(0x0A), "ECC 0x0A decodes BIT/TX/DATA", mod.err_name(0x0A))

    # Build synthetic records with 2 sessions: S1 KOEO (0 bus-offs, 5 errors),
    # S2 running + KOEO (2 bus-offs: 1 running, 1 KOEO; 10 errors: 7 running, 3 KOEO)
    recs = [
        ("session", 1, 1000000, 3),
        ("frame", 1, 1010000, 0x231, False, False, False, 8, struct.pack("<H6B", 0, 0, 0, 0, 0, 0, 0)),
        ("errors", 1, 1020000, 5, 0x00, 0x02, 200, 0),
        ("session", 2, 1000000, 1),
        ("frame", 2, 1010000, 0x231, False, False, False, 8, struct.pack("<H6B", 1800, 0, 0, 0, 0, 0, 0)),
        ("errors", 2, 1020000, 7, 0x03, 0x00, 128, 0),
        ("event", 2, 1025000, 5, 128, 0),
        ("frame", 2, 2000000, 0x231, False, False, False, 8, struct.pack("<H6B", 0, 0, 0, 0, 0, 0, 0)),
        ("errors", 2, 2010000, 3, 0x53, 0x00, 128, 0),
        ("event", 2, 2015000, 5, 128, 2),
    ]
    st = mod.session_stats(recs)
    chk(st[1]["bus_offs"] == 0 and st[1]["err_total"] == 5 and st[1]["err_koeo"] == 5,
        "S1 KOEO stats exact", str(st[1]))
    chk(st[2]["bus_offs"] == 2 and st[2]["bo_run"] == 1 and st[2]["bo_koeo"] == 1,
        "S2 bus-offs split 1 running / 1 KOEO", str(st[2]))
    chk(st[2]["err_total"] == 10 and st[2]["err_run"] == 7 and st[2]["err_koeo"] == 3,
        "S2 errors split 7 running / 3 KOEO", str(st[2]))
    lines.append("canrec_tool: %d/%d checks passed" % (len(lines) - fails, len(lines)))
    return "canrec_tool", 1 if fails else 0, "\n".join(lines), time.time() - t


def main():
    args = [a for a in sys.argv[1:] if a != "-v"]
    verbose = "-v" in sys.argv
    if args == ["portal_teach"]:                 # just the page's tests: no build needed
        name, code, out, dt = run_portal()
        print(f"\n=== {name} ({dt:.1f} s) ===\n{out.rstrip()}")
        print("\n" + ("ALL SCENARIOS PASSED" if code == 0 else "FAILED: " + name))
        sys.exit(1 if code else 0)
    if args == ["canrec_tool"]:
        name, code, out, dt = run_canrec_unit()
        print(f"\n=== {name} ({dt:.1f} s) ===\n{out.rstrip()}")
        print("\n" + ("ALL SCENARIOS PASSED" if code == 0 else "FAILED: " + name))
        sys.exit(1 if code else 0)
    env = build()
    # device_config replays a settings file read from the car's master (no
    # passwords in it) - every one in fixtures/, unless DEVICE_CONFIG=path
    # names another. The scenarios that drive with the car's settings (the
    # P1718 ones) use the first.
    fixtures = sorted(glob.glob(os.path.join(HERE, "fixtures", "*.json")))
    mine = os.environ.get("DEVICE_CONFIG")
    env.setdefault("DEVICE_CONFIG", os.path.join(HERE, "fixtures", "car_config_2026-09-25.json"))
    runs = []
    portal = not args or "portal_teach" in args
    canrec_u = not args or "canrec_tool" in args
    args = [a for a in args if a not in ("portal_teach", "canrec_tool")]
    for n in args or scenarios(env):               # (portal_teach alone was handled above)
        if n == "device_config" and not mine and fixtures:
            runs += [(n, f) for f in fixtures]
        else:
            runs.append((n, None))
    failed = []
    with cf.ThreadPoolExecutor(max_workers=1 if verbose else (os.cpu_count() or 4)) as ex:
        for name, code, out, dt in ex.map(lambda r: run_one(r[0], env, verbose, r[1]), runs):
            print(f"\n=== {name} ({dt:.1f} s) ===")
            if out:
                print(out.rstrip())
            if code != 0:
                failed.append(name)
    if portal:
        name, code, out, dt = run_portal()
        print(f"\n=== {name} ({dt:.1f} s) ===")
        if out:
            print(out.rstrip())
        if code != 0:
            failed.append(name)
    if canrec_u:
        name, code, out, dt = run_canrec_unit()
        print(f"\n=== {name} ({dt:.1f} s) ===")
        if out:
            print(out.rstrip())
        if code != 0:
            failed.append(name)
    print("\n" + ("ALL SCENARIOS PASSED" if not failed else "FAILED: " + ", ".join(failed)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
