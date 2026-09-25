#!/usr/bin/env python3
"""Build the display firmware's UI for the PC and run the scenarios.

    python test_host/run.py                  # build, run every scenario
    python test_host/run.py lamp_seatbelt    # just these
    python test_host/run.py lamp_mil_once -v # one scenario, full firmware log

The real src/ui, src/telemetry/TelemetryStore.cpp and src/config sources are
compiled unmodified against the mocks in test_host/mocks and linked with the
real LVGL from .pio/libdeps, drawing into a framebuffer. Screenshots land in
test_host/shots/. Undefined behaviour traps (UBSan, trap mode), the standard
library checks its bounds, and an LVGL allocation failure aborts instead of
hanging as it would on the device.

Needs g++ (WinLibs GCC: winget install BrechtSanders.WinLibs.POSIX.UCRT) and
one `pio run` beforehand (for LVGL and ArduinoJson in .pio/libdeps).
This file is identical in both display projects.
"""
import concurrent.futures as cf
import glob
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BUILD = os.path.join(HERE, "build")
SHOTS = os.path.join(HERE, "shots")
EXE = os.path.join(BUILD, "ui.exe" if os.name == "nt" else "ui")

LIBDEPS = glob.glob(os.path.join(ROOT, ".pio", "libdeps", "*"))
LVGL = next((p for d in LIBDEPS for p in [os.path.join(d, "lvgl")] if os.path.isdir(p)), None)
AJSON = next((p for d in LIBDEPS for p in [os.path.join(d, "ArduinoJson", "src")] if os.path.isdir(p)), None)

SOURCES = [os.path.join(ROOT, "src", *p.split("/")) for p in (
    "ui/UIBuilder.cpp", "ui/Telltales.cpp", "telemetry/TelemetryStore.cpp",
    "config/ConfigManager.cpp")]
SOURCES += sorted(glob.glob(os.path.join(HERE, "host", "*.cpp")))
SOURCES += sorted(glob.glob(os.path.join(HERE, "tests", "*.cpp")))

COMMON = [
    "-DLV_CONF_INCLUDE_SIMPLE", "-DLV_ASSERT_HANDLER=abort();",
    "-DLV_ASSERT_HANDLER_INCLUDE=<stdlib.h>",
    "-I" + os.path.join(HERE, "mocks"), "-I" + os.path.join(HERE, "host"),
    "-I" + os.path.join(ROOT, "include"), "-I" + BUILD,
]
CFLAGS = ["-pipe", "-O1", "-g", "-w"] + COMMON + (["-I" + LVGL] if LVGL else [])
CXXFLAGS = [
    "-pipe", "-std=gnu++17", "-O1", "-g", "-fno-omit-frame-pointer",
    "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-missing-field-initializers",
    "-Wformat=2", "-Wduplicated-cond", "-Wlogical-op",
    "-fsanitize=undefined", "-fsanitize-trap=undefined",
    "-D_GLIBCXX_ASSERTIONS", "-D__USE_MINGW_ANSI_STDIO=1",
    "-DARDUINOJSON_ENABLE_ARDUINO_STRING=1",
    '-DFIRMWARE_VERSION="host"', "-DCORE_DEBUG_LEVEL=3",
] + COMMON + ["-I" + os.path.join(ROOT, "src", d) for d in ("ui", "telemetry", "config")] + \
    (["-I" + LVGL, "-I" + AJSON] if LVGL and AJSON else [])


def find_tool(name):
    g = shutil.which(name)
    if g:
        return g
    # winget adds it to the PATH of new shells only; look where it installs.
    base = os.path.expandvars(r"%LOCALAPPDATA%\Microsoft\WinGet\Packages")
    for p in glob.glob(os.path.join(base, "BrechtSanders.WinLibs*", "mingw64", "bin", name + ".exe")):
        return p
    sys.exit(name + " not found - install WinLibs GCC (see the top of this file)")


def compile_all(jobs, env):
    """jobs: (tool, flags, src, obj). Skips objects newer than their source
    (LVGL only - it never changes, and is most of the build)."""
    failed = False
    with cf.ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as ex:
        futs = []
        for tool, flags, src, obj in jobs:
            futs.append((src, ex.submit(subprocess.run, [tool, *flags, "-c", src, "-o", obj],
                                        capture_output=True, text=True, env=env)))
        for src, fu in futs:
            r = fu.result()
            out = (r.stdout + r.stderr).strip()
            if out:
                print(out)
            if r.returncode:
                print("compile failed: " + src)
                failed = True
    if failed:
        sys.exit(1)


def target_heap():
    """The board's own LV_MEM_SIZE, in bytes."""
    conf = open(os.path.join(ROOT, "include", "lv_conf.h"), encoding="utf-8").read()
    m = re.search(r"#define\s+LV_MEM_SIZE\s+\((\d+)U?\s*\*\s*1024U?\)", conf)
    return int(m.group(1)) * 1024 if m else 0


def struct_sizes(gcc, env):
    """lv_sizes.c compiled for this PC and for a 32-bit target; nm -S gives
    each struct's size. Written as build/lv_sizes.h."""
    src = os.path.join(HERE, "host", "lv_sizes.c")
    tables = {}
    for tag, extra in (("HOST", []), ("TARGET", ["-m32"])):
        asm = os.path.join(BUILD, "lv_sizes_%s.s" % tag.lower())
        r = subprocess.run([gcc, *CFLAGS, *extra, "-S", src, "-o", asm],
                           capture_output=True, text=True, env=env)
        if r.returncode:
            print(r.stdout + r.stderr)
            sys.exit("lv_sizes.c failed for " + tag)
        sizes, name = {}, None
        for line in open(asm, encoding="utf-8", errors="replace"):
            line = line.strip()
            m = re.match(r"^_?SZ_(\w+):$", line)
            if m:
                name = m.group(1)
                continue
            m = re.match(r"^\.long\s+(\d+)$", line)
            if name and m:
                sizes[name] = int(m.group(1))
                name = None
        tables[tag] = sizes
    fields = ["obj", "label", "arc", "bar", "img", "meter", "chart", "obj_style", "style",
              "style_value", "style_prop", "spec_attr", "event_dsc", "meter_scale",
              "meter_indicator", "chart_series", "anim", "timer", "coord", "ptr", "size_t_"]
    names = ["lv_obj_t", "lv_label_t", "lv_arc_t", "lv_bar_t", "lv_img_t", "lv_meter_t",
             "lv_chart_t", "_lv_obj_style_t", "lv_style_t", "lv_style_value_t",
             "lv_style_prop_t", "_lv_obj_spec_attr_t", "lv_event_dsc_t",
             "lv_meter_scale_t", "lv_meter_indicator_t", "lv_chart_series_t",
             "lv_anim_t", "lv_timer_t", "lv_coord_t", "ptr", "size_t"]
    h = ["/* GENERATED by test_host/run.py from host/lv_sizes.c */", "#pragma once",
         "#include <stddef.h>", "struct LvSizes { size_t %s; };" % ", ".join(fields)]
    for tag, sizes in tables.items():
        missing = [n for n in names if n not in sizes]
        if missing:
            sys.exit("lv_sizes: no size for " + ", ".join(missing))
        h.append("static const LvSizes LV_SIZES_%s = {%s};" % (
            tag, ", ".join(str(sizes[n]) for n in names)))
    path = os.path.join(BUILD, "lv_sizes.h")
    text = "\n".join(h) + "\n"
    if not os.path.exists(path) or open(path).read() != text:
        open(path, "w").write(text)
    t = tables["TARGET"]
    print("32-bit sizes: lv_obj_t %d, lv_label_t %d, lv_style_t %d (PC: %d, %d, %d)" % (
        t["lv_obj_t"], t["lv_label_t"], t["lv_style_t"], tables["HOST"]["lv_obj_t"],
        tables["HOST"]["lv_label_t"], tables["HOST"]["lv_style_t"]))


def build():
    if not (LVGL and AJSON):
        sys.exit("LVGL/ArduinoJson not found in .pio/libdeps - run `pio run` once first")
    gcc, gxx = find_tool("gcc"), find_tool("g++")
    env = dict(os.environ, PATH=os.path.dirname(gxx) + os.pathsep + os.environ.get("PATH", ""))
    # The studio page header is generated by the PlatformIO pre-build step.
    subprocess.run([sys.executable, os.path.join(ROOT, "tools", "embed_studio.py")], check=True)

    os.makedirs(BUILD, exist_ok=True)
    struct_sizes(gcc, env)
    CXXFLAGS.append("-DLV_MEM_SIZE_TARGET=%d" % target_heap())

    lv_build = os.path.join(BUILD, "lvgl")
    os.makedirs(lv_build, exist_ok=True)
    lv_objs, lv_jobs = [], []
    conf = os.path.join(HERE, "host", "lv_conf.h")
    for src in glob.glob(os.path.join(LVGL, "src", "**", "*.c"), recursive=True):
        rel = os.path.relpath(src, LVGL).replace(os.sep, "_")
        obj = os.path.join(lv_build, rel[:-2] + ".o")
        lv_objs.append(obj)
        newest = max(os.path.getmtime(src), os.path.getmtime(conf),
                     os.path.getmtime(os.path.join(ROOT, "include", "lv_conf.h")))
        if not os.path.exists(obj) or os.path.getmtime(obj) < newest:
            lv_jobs.append((gcc, CFLAGS, src, obj))
    if lv_jobs:
        print("compiling LVGL (%d files, once)..." % len(lv_jobs))
    compile_all(lv_jobs, env)

    objs, jobs = [], []
    for src in SOURCES:
        obj = os.path.join(BUILD, os.path.splitext(os.path.basename(src))[0] + ".o")
        objs.append(obj)
        jobs.append((gxx, CXXFLAGS, src, obj))
    compile_all(jobs, env)
    # Static: no runtime DLLs, so another MinGW on the PATH (Git has one)
    # cannot slip in an older libstdc++ or winpthread.
    r = subprocess.run([gxx, *CXXFLAGS, *objs, *lv_objs, "-o", EXE, "-static"],
                       capture_output=True, text=True, env=env)
    if r.returncode:
        print(r.stdout + r.stderr)
        sys.exit("link failed")
    return env


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    env = build()
    board = os.path.basename(ROOT)
    shots = os.path.join(SHOTS)
    os.makedirs(shots, exist_ok=True)
    names = subprocess.run([EXE, "--list"], capture_output=True, text=True).stdout.split()
    todo = args or names
    env = dict(env, PROJECT_DIR=ROOT, SHOT_DIR=shots, VERBOSE="1" if verbose else "0")
    failed = []
    t0 = time.time()

    def one(n):
        t = time.time()
        r = subprocess.run([EXE, n], capture_output=True, text=True, env=env)
        return n, r, time.time() - t

    with cf.ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as ex:
        for n, r, dt in ex.map(one, todo):
            ok = r.returncode == 0
            print("%-4s %-32s %5.1fs" % ("ok" if ok else "FAIL", n, dt))
            out = r.stdout.strip()
            if out and (verbose or not ok or "LVGL heap" in out or "strip shows" in out
                        or "bound" in out):
                print(out)
            if r.returncode not in (0, 1):
                print("    crashed (exit %d) - debug with: gdb --args %s %s" % (r.returncode, EXE, n))
            if not ok:
                failed.append(n)
    print("\n%s: %d/%d scenarios passed in %.0fs; screenshots in %s" % (
        board, len(todo) - len(failed), len(todo), time.time() - t0, shots))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
