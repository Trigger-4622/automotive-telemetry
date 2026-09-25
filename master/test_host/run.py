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

SOURCES = [os.path.join(ROOT, "src", f) for f in ("main.cpp", "Ssm2.cpp", "Learner.cpp", "MasterConfig.cpp")]
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


def run_one(name, env, verbose):
    t = time.time()
    args = [EXE, name] + (["-v"] if verbose else [])
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
    return name, code, out, time.time() - t


def main():
    args = [a for a in sys.argv[1:] if a != "-v"]
    verbose = "-v" in sys.argv
    env = build()
    # device_config replays a settings file read from the car's master (no
    # passwords in it); DEVICE_CONFIG=path tests another one.
    env.setdefault("DEVICE_CONFIG", os.path.join(HERE, "fixtures", "car_config_2026-09-25.json"))
    names = args or scenarios(env)
    failed = []
    with cf.ThreadPoolExecutor(max_workers=1 if verbose else (os.cpu_count() or 4)) as ex:
        for name, code, out, dt in ex.map(lambda n: run_one(n, env, verbose), names):
            print(f"\n=== {name} ({dt:.1f} s) ===")
            if out:
                print(out.rstrip())
            if code != 0:
                failed.append(name)
    print("\n" + ("ALL SCENARIOS PASSED" if not failed else "FAILED: " + ", ".join(failed)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
