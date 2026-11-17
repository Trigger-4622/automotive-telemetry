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
    """A candump log -> shim/replay.bin, small enough for a long drive to fit the image:
    "RPL2", u16 identifiers, u16 0; per identifier u16 id, u8 dlc, u8 0; then per frame
    u8 identifier index, u8 time since the last frame in 50 us steps (255: a u32 follows),
    u8 mask of the payload bytes changed since that identifier's last frame, those bytes.
    Extended and remote frames, and the diagnostic range (0x7DF-0x7EF: the shim's own ECU
    answers there) are left out. Returns (frames, seconds, identifiers, bytes)."""
    import struct
    line_re = re.compile(r"\((\d+\.\d+)\)\s+\S+\s+([0-9A-Fa-f]{3})#([0-9A-Fa-f]{0,16})\s*$")
    index, dlcs, last_data = {}, [], []
    body = bytearray()
    t0 = None
    tick = 0                                      # 50 us steps since the first frame
    n = 0
    with open(path, errors="replace") as f:
        for line in f:
            m = line_re.match(line)
            if not m:
                continue
            ident = int(m.group(2), 16)
            if 0x7DF <= ident <= 0x7EF:
                continue
            hexd = m.group(3)
            data = bytes.fromhex(hexd if len(hexd) % 2 == 0 else hexd[:-1])
            t = float(m.group(1))
            if t0 is None:
                t0 = t
            if ident not in index:
                if len(index) >= 128:
                    continue                      # the shim's table: 128 identifiers at most
                index[ident] = len(index)
                dlcs.append(len(data))
                last_data.append(bytearray(8))
            i = index[ident]
            now = max(tick, int(round((t - t0) * 20000)))
            dt, tick = now - tick, now
            body.append(i)
            if dt < 255:
                body.append(dt)
            else:
                body.append(255)
                body += struct.pack("<I", dt)
            padded = data.ljust(8, b"\0")
            mask = 0
            changed = bytearray()
            for k in range(8):
                if padded[k] != last_data[i][k]:
                    mask |= 1 << k
                    changed.append(padded[k])
            last_data[i][:] = padded
            body.append(mask)
            body += changed
            n += 1
    if not n:
        sys.exit(f"{path}: no candump lines ('(time) can0 123#0011..') in it")
    with open(out, "wb") as o:
        o.write(b"RPL2" + struct.pack("<HH", len(index), 0))
        for ident, i in sorted(index.items(), key=lambda kv: kv[1]):
            o.write(struct.pack("<HBB", ident, min(dlcs[i], 8), 0))
        o.write(body)
    return n, tick / 20000, len(index), 8 + 4 * len(index) + len(body)


def build(project, extra_ids, readers, replay, settings, ota=False):
    env = dict(os.environ)
    env["QEMU_SHIM_FLAGS"] = (f"-DQEMU_EXTRA_IDS={extra_ids} -DQEMU_READERS={1 if readers else 0} "
                              f"-DQEMU_REPLAY={1 if replay else 0} -DQEMU_SETTINGS={1 if settings else 0} "
                              f"-DQEMU_OTA={1 if ota else 0}")
    pio_env = "master-qemu-ota" if ota else "master-qemu"
    r = subprocess.run([sys.executable, "-m", "platformio", "run", "-d", project, "-e", pio_env],
                       capture_output=True, text=True, env=env)
    if r.returncode:
        sys.stdout.write(r.stdout[-4000:] + r.stderr[-4000:])
        sys.exit("build failed")
    for line in r.stdout.splitlines():
        if line.startswith("RAM:") or line.startswith("Flash:"):
            print("  build " + line)
    return os.path.join(project, ".pio", "build", pio_env)


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


def ota_report(log, check):
    """--ota: four boots - the update, kept; a second update, a crash, rolled back."""
    parts = re.split(r"^(?=rst:0x)", log, flags=re.M)[1:]
    print(f"       {len(parts)} boot(s)")
    for i, p in enumerate(parts, 1):
        fw = re.search(r"firmware \S+, build (\w+), slot (\w+)([^\r\n]*)", p)
        cfg = re.search(r"config: loaded \((\d+) signals", p)
        print(f"       boot {i}: " + (f"{fw.group(2)}, build {fw.group(1)}{fw.group(3)}" if fw else "?")
              + (f"; settings loaded, {cfg.group(1)} signals" if cfg else ""))
        for line in re.findall(r"\[ota  \] ([^\r\n]*)", p):
            print("           " + line)
        for line in re.findall(r"update: ([^\r\n]*)", p):
            print("           update: " + line)
    slots = [m.group(1) for m in (re.search(r"firmware \S+, build \w+, slot (\w+)", p) for p in parts) if m]
    check(slots[:4] == ["app0", "app1", "app0", "app1"], "booted app0, app1 (update), app0 (update), app1 (rolled back)",
          " ".join(slots))
    check(len(parts) == 4, "four boots and no more", f"{len(parts)}")
    check(re.search(r"refused while moving: stop the car first", log), "the update refused while the car moved")
    check(len(re.findall(r"\[ota  \] \w+ update: .*checked - restarting into it", log)) == 2,
          "both uploads checked and switched to")
    check(len(parts) > 1 and "new, on trial" in parts[1] and
          re.search(r"update: the new firmware has run properly - kept", parts[1]),
          "the update booted on trial and was kept after 30 s")
    check(len(parts) > 2 and "new, on trial" in parts[2] and not re.search(r"run properly - kept", parts[2]),
          "the second update was on trial and never kept before its crash")
    check(len(parts) > 3 and re.search(r"the last update did not run properly - back on this firmware", parts[3]),
          "after the crash the bootloader went back to the kept firmware, and said so")
    sig = [int(x) for x in re.findall(r"config: loaded \((\d+) signals", log)]
    ver = [int(x) for x in re.findall(r"\[ota  \] boot in \w+, (\d+) verified signals", log)]
    check(len(sig) >= 4 and len(set(sig[1:])) == 1 and min(ver[1:] or [0]) >= 2,
          "the settings and the learned values kept through every restart",
          f"signals per boot {sig}, verified per boot {ver}")
    own = re.findall(r"log: drive logs on their own partition \((\d+) KB\)", log)
    check(own and all(x == own[0] for x in own) and int(own[0]) > 8000,
          "the drive logs on their own 8 MB partition at every boot", f"{own[0] if own else 0} KB")
    others = re.findall(r"Guru Meditation|stack overflow|assert failed|CORRUPT HEAP|out of memory", log)
    aborts = len(re.findall(r"abort\(\) was called", log))
    check(not others and aborts <= 1, "no crash but the one on purpose",
          (others[0] if others else "") + f" {aborts} abort(s)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=120)
    ap.add_argument("--extra-ids", type=int, default=0, help="more identifiers on the simulated bus")
    ap.add_argument("--src", default=None, help="master/ directory of another checkout")
    ap.add_argument("--keep", default=None, help="write the serial log here")
    ap.add_argument("--replay", default=None, metavar="LOG",
                    help="a recorded candump log in place of the made-up car; the ECU's answers "
                         "are read off its frames with the Subaru 2005-2019 layout (see README)")
    ap.add_argument("--settings", default=None, metavar="JSON",
                    help="a settings file (the car's own, e.g. test_host/fixtures/...) applied at boot "
                         "as the portal's upload does")
    ap.add_argument("--ota", action="store_true",
                    help="the firmware update over Wi-Fi as the portal does it, with the board's own "
                         "partition table: update, kept after its trial, update again, a crash before "
                         "it is kept, and the bootloader's rollback (about 4 minutes; not with --replay)")
    ap.add_argument("--readers", action="store_true",
                    help="a task on the other core reads the tables as the portal's pages do, "
                         "50 times a second, and checks them (needs the row readers: not for "
                         "versions before 2026-11-16)")
    a = ap.parse_args()
    if a.ota and a.replay:
        sys.exit("--ota uses the board's partition table: a recorded drive does not fit it")
    if a.ota and a.seconds == 120:
        a.seconds = 300

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
    shim = os.path.join(project, "shim")
    if a.settings:
        shutil.copy(a.settings, os.path.join(shim, "settings.json"))
        os.utime(os.path.join(shim, "qemu_shim.cpp"))      # the file is part of it: build it again
    if a.replay:
        frames, secs, n_ids, size = convert_log(a.replay, os.path.join(shim, "replay.bin"))
        os.utime(os.path.join(shim, "qemu_shim.cpp"))
        print(f"== {os.path.relpath(project)}: {a.seconds} s of {os.path.basename(a.replay)} "
              f"({frames} frames, {secs:.0f} s, {n_ids} identifiers; {size / 1e6:.2f} MB in the image)")
    else:
        print(f"== {os.path.relpath(project)}: {a.seconds} s, {n_ids} identifiers")
    if a.settings:
        print(f"   with the settings of {os.path.basename(a.settings)}")
    if a.ota:
        print("   the firmware update test, on the board's partition table")
    build_dir = build(project, a.extra_ids, a.readers, a.replay, a.settings, a.ota)
    work = tempfile.mkdtemp(prefix="qemu_master_")
    flash = os.path.join(work, "flash.bin")
    uart = os.path.join(work, "uart.log")
    # QEMU itself (not the firmware) now and then dies of a segmentation fault
    # early on, since the flash took writes: the run is made again, once.
    for attempt in (1, 2):
        image(build_dir, flash)
        try:
            q = subprocess.run([qemu, "-nographic", "-machine", "esp32s3",
                                "-drive", f"file={flash},if=mtd,format=raw",
                                "-serial", f"file:{uart}", "-monitor", "none", "-display", "none"],
                               timeout=a.seconds, capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            break
        tail = open(uart, errors="replace").read()[-1500:]
        if q.returncode == -11 and attempt == 1:
            print("   QEMU itself crashed (segmentation fault, an emulator fault) - running it again")
            continue
        sys.exit(f"QEMU ended by itself (exit {q.returncode}): {q.stderr[-800:]}\n--- serial log's end:\n{tail}")
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
    if a.ota:
        ota_report(log, check)
        print("PASSED" if ok else "FAILED")
        return 0 if ok else 1
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
    # With a settings file, its learned signals already cover what the ECU offers (and with
    # another car's file their identifiers are not on this bus): nothing new need be learned.
    learned = f"found {len(found)}, verified {len(set(verified))}: " + " ".join(sorted(set(verified)))
    if a.settings:
        print(f"       learned (the settings' own signals cover the ECU's values): {learned}")
    else:
        check(len(set(verified)) >= (2 if a.replay else 3), "values learned from the bus and verified against the ECU",
              learned)
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

    if a.settings:
        m = re.search(r"\[qemu \] the car's settings applied as the portal's upload does: (\w+)", log)
        check(m and m.group(1) == "saved", "the car's settings applied and saved", m.group(0)[8:] if m else "")
        saves = len(re.findall(r"config: saved", log))
        fails = re.findall(r"config: .*(?:fail|could not|not saved)[^\r\n]*", log, re.I)
        check(not fails, "every settings save reached the flash",
              f"{saves} saves" + (", " + fails[0] if fails else ""))
    for line in re.findall(r"log: [^\r\n]*(?:partition|formatted|moved)[^\r\n]*", log)[:4]:
        print("       " + line.strip())
    opened = re.findall(r"log: recording [^\r\n]*", log)
    closed = re.findall(r"log: \S+ closed, \d+ rows[^\r\n]*", log)
    rotated = re.findall(r"log: deleted \S+ to make room", log)
    logerr = re.findall(r"log: [^\r\n]*(?:fail|could not|cannot|space full)[^\r\n]*", log, re.I)
    if opened or logerr:
        print(f"       drive log: {len(opened)} opened ({opened[0][5:].strip() if opened else ''}), "
              f"{len(closed)} closed, {len(rotated)} deleted to make room")
        dl = re.findall(r"\[dlog \] ([^\r\n]*)", log)
        if dl:
            print(f"       drive log at the end: {dl[-1]}")
        for line in logerr[:3]:
            print("       " + line.strip())

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
