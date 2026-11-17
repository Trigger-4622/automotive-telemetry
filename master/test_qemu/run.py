#!/usr/bin/env python3
"""Run the master firmware in Espressif's QEMU (machine esp32s3) and check it.

    python master/test_qemu/run.py [--seconds 120] [--extra-ids 0] [--src DIR] [--keep]

Builds the test firmware (platformio.ini beside this file: the real sources
plus shim/qemu_shim.cpp), makes a 16 MB flash image, runs it for --seconds and
reads the serial log: no crash or restart, every identifier of the simulated
car in the census, the values the simulated ECU offers learned and verified,
and each task's least stack left. See README.md for what this does and does
not show.

QEMU: the qemu-system-xtensa of Espressif's fork, from $QEMU_ESP32S3 or PATH.
--src: another checkout's master/ directory (to compare two versions).
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PIO_HOME = os.path.expanduser("~/.platformio")


def find_qemu():
    q = os.environ.get("QEMU_ESP32S3") or shutil.which("qemu-system-xtensa")
    if not q or not os.path.exists(q):
        sys.exit("qemu-system-xtensa (Espressif's fork) not found: set QEMU_ESP32S3 to it")
    machines = subprocess.run([q, "-machine", "help"], capture_output=True, text=True).stdout
    if "esp32s3" not in machines:
        sys.exit(f"{q} has no esp32s3 machine - it must be Espressif's fork of QEMU")
    return q


def convert_log(path, out):
    """A candump log -> 16 bytes a frame: u32 µs from the first, u16 id, u8 dlc, pad, 8 data.
    Extended and remote frames, and the diagnostic range (0x7DF-0x7EF: the shim's own ECU
    answers there) are left out. Returns (frames, seconds, identifiers)."""
    import struct
    line_re = re.compile(r"\((\d+\.\d+)\)\s+\S+\s+([0-9A-Fa-f]{3})#([0-9A-Fa-f]{0,16})\s*$")
    t0 = None
    ids = set()
    n = 0
    last = 0
    with open(path, errors="replace") as f, open(out, "wb") as o:
        for line in f:
            m = line_re.match(line)
            if not m:
                continue
            ident = int(m.group(2), 16)
            if 0x7DF <= ident <= 0x7EF:
                continue
            data = bytes.fromhex(m.group(3) if len(m.group(3)) % 2 == 0 else m.group(3)[:-1])
            t = float(m.group(1))
            if t0 is None:
                t0 = t
            us = max(last, int(round((t - t0) * 1e6)))
            last = us
            o.write(struct.pack("<IHBB8s", us, ident, len(data), 0, data.ljust(8, b"\0")))
            ids.add(ident)
            n += 1
    if not n:
        sys.exit(f"{path}: no candump lines ('(time) can0 123#0011..') in it")
    return n, last / 1e6, len(ids)


def build(project, extra_ids, readers, replay):
    env = dict(os.environ)
    env["QEMU_SHIM_FLAGS"] = (f"-DQEMU_EXTRA_IDS={extra_ids} -DQEMU_READERS={1 if readers else 0} "
                              f"-DQEMU_REPLAY={1 if replay else 0}")
    r = subprocess.run([sys.executable, "-m", "platformio", "run", "-d", project],
                       capture_output=True, text=True, env=env)
    if r.returncode:
        sys.stdout.write(r.stdout[-4000:] + r.stderr[-4000:])
        sys.exit("build failed")
    for line in r.stdout.splitlines():
        if line.startswith("RAM:") or line.startswith("Flash:"):
            print("  build " + line)
    return os.path.join(project, ".pio", "build", "master-qemu")


def image(build_dir, out):
    boot_app0 = os.path.join(PIO_HOME, "packages", "framework-arduinoespressif32", "tools",
                             "partitions", "boot_app0.bin")
    esptool = os.path.join(PIO_HOME, "packages", "tool-esptoolpy", "esptool.py")
    r = subprocess.run([sys.executable, esptool, "--chip", "esp32s3", "merge_bin", "-o", out,
                        "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "16MB",
                        "--fill-flash-size", "16MB",
                        "0x0", os.path.join(build_dir, "bootloader.bin"),
                        "0x8000", os.path.join(build_dir, "partitions.bin"),
                        "0xe000", boot_app0,
                        "0x10000", os.path.join(build_dir, "firmware.bin")],
                       capture_output=True, text=True)
    if r.returncode:
        sys.exit("merge_bin failed: " + r.stdout + r.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=120)
    ap.add_argument("--extra-ids", type=int, default=0, help="more identifiers on the simulated bus")
    ap.add_argument("--src", default=None, help="master/ directory of another checkout")
    ap.add_argument("--keep", default=None, help="write the serial log here")
    ap.add_argument("--replay", default=None, metavar="LOG",
                    help="a recorded candump log in place of the made-up car; the ECU's answers "
                         "are read off its frames with the Subaru 2005-2019 layout (see README)")
    ap.add_argument("--readers", action="store_true",
                    help="a task on the other core reads the tables as the portal's pages do, "
                         "50 times a second, and checks them (needs the row readers: not for "
                         "versions before 2026-11-16)")
    a = ap.parse_args()

    qemu = find_qemu()
    project = HERE
    tmp_project = None
    if a.src:                                    # the same shim against another version's sources
        tmp_project = os.path.join(os.path.abspath(a.src), "test_qemu")
        if os.path.abspath(tmp_project) != HERE:
            os.makedirs(os.path.join(tmp_project, "shim"), exist_ok=True)
            shutil.copy(os.path.join(HERE, "platformio.ini"), tmp_project)
            shutil.copy(os.path.join(HERE, "partitions_qemu.csv"), tmp_project)
            shutil.copy(os.path.join(HERE, "shim", "qemu_shim.cpp"), os.path.join(tmp_project, "shim"))
        project = tmp_project

    n_ids = 25 + a.extra_ids
    if a.replay:
        shim = os.path.join(project, "shim")
        frames, secs, n_ids = convert_log(a.replay, os.path.join(shim, "replay.bin"))
        os.utime(os.path.join(shim, "qemu_shim.cpp"))      # the log is part of it: build it again
        print(f"== {os.path.relpath(project)}: {a.seconds} s of {os.path.basename(a.replay)} "
              f"({frames} frames, {secs:.0f} s, {n_ids} identifiers)")
    else:
        print(f"== {os.path.relpath(project)}: {a.seconds} s, {n_ids} identifiers")
    build_dir = build(project, a.extra_ids, a.readers, a.replay)
    work = tempfile.mkdtemp(prefix="qemu_master_")
    flash = os.path.join(work, "flash.bin")
    uart = os.path.join(work, "uart.log")
    image(build_dir, flash)
    try:
        subprocess.run([qemu, "-nographic", "-machine", "esp32s3",
                        "-drive", f"file={flash},if=mtd,format=raw",
                        "-serial", f"file:{uart}", "-monitor", "none", "-display", "none"],
                       timeout=a.seconds, capture_output=True)
        sys.exit("QEMU ended by itself")
    except subprocess.TimeoutExpired:
        pass
    log = open(uart, errors="replace").read()
    if a.keep:
        shutil.copy(uart, a.keep)
    shutil.rmtree(work, ignore_errors=True)

    # ---- what the run says -------------------------------------------------
    ok = True

    def check(cond, what, info=""):
        nonlocal ok
        ok &= bool(cond)
        print(f"  {'ok  ' if cond else 'FAIL'} {what:<62} {info}")

    boots = len(re.findall(r"^rst:0x", log, re.M))
    crash = re.findall(r"Guru Meditation|abort\(\) was called|Backtrace:|stack overflow|Rebooting|"
                       r"assert failed|CORRUPT HEAP|out of memory", log)
    check(boots == 1 and not crash, "booted once: no panic, no restart",
          f"{boots} boot(s)" + (", " + crash[0] if crash else ""))

    m = re.search(r"learner started: (\d+) fields x \d+ values, (\d+) bits x \d+ switches "
                  r"\((\d+) KB, (\d+) KB left\)", log)
    check(m, "the learner sized its tables from the heap",
          f"{m.group(1)} fields, {m.group(2)} bits ({m.group(3)} KB, {m.group(4)} KB left)" if m else "")

    ids = [int(x) for x in re.findall(r"\[bus \] (\d+) distinct IDs", log)]
    want = min(n_ids + 1, 128)                   # + the ECU's answers (0x7E8)
    check(ids and ids[-1] == want, "every identifier on the bus is in the census",
          f"{ids[-1] if ids else 0} of {want}")
    blocks = len(re.findall(r"census: more than \d+ identifiers", log))
    if a.extra_ids or a.replay:
        print(f"       census rows taken from the heap: {blocks} block(s) of 16")

    rx = [int(x) for x in re.findall(r"\[stats\] CAN rx: (\d+)", log)]
    check(len(rx) > 3 and rx[-1] > rx[0] and (rx[-1] - rx[0]) / (5 * (len(rx) - 1)) > 500,
          "frames keep arriving",
          f"{(rx[-1] - rx[0]) / (5 * (len(rx) - 1)):.0f} a second" if len(rx) > 3 else "")

    for line in re.findall(r"learner: (0x[0-9A-F]{4} found in .*?) - confirming", log):
        print("       " + line)
    for line in re.findall(r"signal '(learned [0-9A-F]{4}' \(0x[0-9A-F]+\) (?:VERIFIED|REJECTED)[^\r\n]*)", log):
        print("       " + line)
    found = set(re.findall(r"learner: (0x[0-9A-F]{4}) found in", log))
    verified = re.findall(r"signal 'learned ([0-9A-F]{4})' \(0x[0-9A-F]+\) VERIFIED", log)
    # A recorded drive is what it is: its slow values may not move far enough to be confirmed.
    check(len(set(verified)) >= (2 if a.replay else 3), "values learned from the bus and verified against the ECU",
          f"found {len(found)}, verified {len(set(verified))}: " + " ".join(sorted(set(verified))))
    rejected = re.findall(r"REJECTED", log)
    print(f"       rejected by the verifier: {len(rejected)}")

    heap = re.findall(r"heap: (\d+) \(lowest (\d+), largest block (\d+)\)", log)
    if heap:
        h = heap[-1]
        print(f"       heap at the end {int(h[0]) // 1024} KB, lowest {int(h[1]) // 1024} KB, "
              f"largest block {int(h[2]) // 1024} KB   (no Wi-Fi in QEMU: its share is not in this)")
        check(int(h[1]) > 20 * 1024, "the heap never ran low", f"lowest {int(h[1])} bytes")

    if a.readers:
        rd = re.findall(r"\[read \] (\d+) passes \| (.*?) \| (\d+) inconsistencies(.*?) \| stack left (\d+)", log)
        check(rd and int(rd[-1][0]) > a.seconds * 20 and int(rd[-1][2]) == 0,
              "the tables read from the other core all along: all consistent",
              f"{rd[-1][0]} passes, {rd[-1][1]}{rd[-1][3]}" if rd else "no [read ] line")

    stacks = re.findall(r"\[stack\] least left of size:(.*)", log)
    if stacks:
        last = re.findall(r"(\w+) (\d+)/(\d+)", stacks[-1])
        low = min(last, key=lambda t: int(t[1]))
        print("       least stack left: " + ", ".join(f"{n} {f}/{s}" for n, f, s in last))
        check(int(low[1]) >= 1024, "every task kept 1 KB of stack or more", f"least: {low[0]} {low[1]} bytes")
    else:
        m2 = re.findall(r"least stack left (\d+) B \((\w+)\)", log)
        if m2:
            print(f"       least stack left (this version reports one task only): {m2[-1][1]} {m2[-1][0]}")
            check(int(m2[-1][0]) >= 1024, "the watched tasks kept 1 KB of stack or more")

    print("PASSED" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
