#!/usr/bin/env python3
"""Check the rules that keep the three projects consistent (CLAUDE.md, "Things
that must stay in sync"). Changes nothing; exit code 1 lists what to fix.

    python scripts/check_sync.py

1. Files that must be identical: MasterPacket.h in all three projects, and
   the files the screens share (Palette.h, the warning lamps, their tools,
   the whole test harness).
2. Generated files that must match their source: the master portal
   (src/WebPortal.cpp from tools/portal_page.html), the Gauge Studio metric
   catalogue (from MasterPacket.h), and the studio page compiled into each
   screen (src/config/StudioPage.h from data/www/index.html).
3. What the studio page copies from the firmware: the warning-lamp symbols
   (tools/telltale_icons.json) and the preview's colours (include/Palette.h).
"""
import gzip
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
S1, S2 = "screens/screen1-round", "screens/screen2-cluster"
SHARED = ["include/Palette.h", "include/TelltaleIcons.h", "src/ui/Telltales.h",
          "src/ui/Telltales.cpp", "tools/embed_studio.py", "tools/make_icons.py",
          "tools/telltale_icons.json"]
SKIP_DIRS = {"build", "shots", "__pycache__"}
problems = []


def path(rel):
    return os.path.join(ROOT, *rel.split("/"))


def read(rel):
    with open(path(rel), "rb") as fh:
        return fh.read()


def report(ok, what, fix=""):
    print("  %s %s" % ("ok  " if ok else "FAIL", what))
    if not ok:
        problems.append("%s%s" % (what, " - " + fix if fix else ""))


def same(files, what):
    missing = [f for f in files if not os.path.exists(path(f))]
    if missing:
        report(False, what, "missing: " + ", ".join(missing))
        return
    first = read(files[0])
    differ = [f for f in files[1:] if read(f) != first]
    report(not differ, what, "differs from %s: %s - make the copies identical"
           % (files[0], ", ".join(differ)))


def tree(rel):
    """Files under a directory, relative to it, leaving out harness output."""
    out = set()
    for dp, dn, fn in os.walk(path(rel)):
        dn[:] = [d for d in dn if d not in SKIP_DIRS]
        for f in fn:
            if not f.endswith(".pyc"):
                out.add(os.path.relpath(os.path.join(dp, f), path(rel)).replace(os.sep, "/"))
    return out


def regenerates(outputs, script, what):
    """Run a generator and compare its output with the files as they were;
    put the files back afterwards so this check never changes anything."""
    before = {f: read(f) for f in outputs}
    try:
        r = subprocess.run([sys.executable, path(script)], cwd=ROOT, capture_output=True, text=True)
        if r.returncode:
            report(False, what, "%s failed: %s" % (script, (r.stdout + r.stderr).strip()[-300:]))
            return
        stale = [f for f in outputs if read(f) != before[f]]
        report(not stale, what, "out of date: %s - run python %s and commit the result"
               % (", ".join(stale), script))
    finally:
        for f, data in before.items():
            with open(path(f), "wb") as fh:
                fh.write(data)


def studio_page(screen):
    header = read(screen + "/src/config/StudioPage.h").decode("utf-8")
    body = header.split("STUDIO_HTML_GZ[] = {", 1)[-1].split("};", 1)[0]
    try:
        page = gzip.decompress(bytes(int(b, 16) for b in re.findall(r"0x([0-9A-Fa-f]{2})", body)))
    except OSError:
        page = None
    report(page == read(screen + "/data/www/index.html"),
           "%s: firmware's studio page matches data/www/index.html" % screen,
           "run python %s/tools/embed_studio.py (a firmware build also does it) and commit" % screen)


def studio_copies(screen):
    """The studio draws the lamps and the preview the way the firmware does:
    the same symbols, and only colours from the palette."""
    page = read(screen + "/data/www/index.html").decode("utf-8")
    m = re.search(r"const ICONS = (\{.*?\});\n", page, re.S)
    try:
        same_icons = m is not None and \
            json.loads(m.group(1)) == json.loads(read(screen + "/tools/telltale_icons.json"))
    except ValueError:
        same_icons = False
    report(same_icons, "%s: the studio's lamp symbols match tools/telltale_icons.json" % screen,
           "copy tools/telltale_icons.json into the ICONS constant of data/www/index.html")
    palette = {int(v, 16) for v in re.findall(r"#define\s+UI_\w+\s+0x([0-9A-Fa-f]{6})",
                                              read(screen + "/include/Palette.h").decode("utf-8"))}
    used = set()
    for name in ("PV", "TONES"):
        m = re.search(r"const %s\s*=\s*\{(.*?)\};" % name, page, re.S)
        if m:
            used |= {int(v, 16) for v in re.findall(r"'#([0-9A-Fa-f]{6})'", m.group(1))}
    stray = sorted("#%06X" % c for c in used - palette)
    report(bool(used) and not stray, "%s: the studio preview's colours are Palette.h colours" % screen,
           "not in the palette: %s - the round board's scheme is the reference" % ", ".join(stray))


def main():
    print("identical copies")
    same(["master/include/MasterPacket.h", S1 + "/include/MasterPacket.h",
          S2 + "/include/MasterPacket.h"], "MasterPacket.h in master and both screens")
    for f in SHARED:
        same([S1 + "/" + f, S2 + "/" + f], f + " in both screens")
    t1, t2 = tree(S1 + "/test_host"), tree(S2 + "/test_host")
    report(t1 == t2, "test_host has the same files in both screens",
           "only in one: " + ", ".join(sorted(t1 ^ t2)))
    for f in sorted(t1 & t2):
        same([S1 + "/test_host/" + f, S2 + "/test_host/" + f], "test_host/" + f + " in both screens")

    print("generated files")
    regenerates(["master/src/WebPortal.cpp"], "master/tools/build_portal.py",
                "master/src/WebPortal.cpp matches tools/portal_page.html + WebPortal.cpp.in")
    regenerates([S1 + "/data/www/index.html", S2 + "/data/www/index.html"],
                "master/tools/known_catalogue.py",
                "Gauge Studio metric catalogue matches MasterPacket.h")
    for s in (S1, S2):
        studio_page(s)

    print("what the studio copies from the firmware")
    for s in (S1, S2):
        studio_copies(s)

    if problems:
        print("\n%d problem(s):" % len(problems))
        for p in problems:
            print("  - " + p)
        sys.exit(1)
    print("\nall in sync")


if __name__ == "__main__":
    main()
