#!/usr/bin/env python3
"""Run the host test harnesses: the master in a simulated car, and both
screens rendering real LVGL into PNG screenshots.

    python scripts/test_all.py                   # all three
    python scripts/test_all.py master screen2    # some of: master screen1 screen2

Each harness builds the unmodified firmware sources for the PC first, so this
takes a few minutes the first time and far less afterwards. Needs g++ and the
libraries in each project's .pio/libdeps (a firmware build, or
scripts/cloud-setup.sh, puts them there). Exit code 0 = everything passed.
"""
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUITES = {
    "master": "master",
    "screen1": os.path.join("screens", "screen1-round"),
    "screen2": os.path.join("screens", "screen2-cluster"),
}


def main():
    picks = sys.argv[1:] or list(SUITES)
    unknown = [p for p in picks if p not in SUITES]
    if unknown:
        sys.exit("unknown suite(s): %s - choose from %s" % (", ".join(unknown), ", ".join(SUITES)))
    results = []
    for name in picks:
        path = SUITES[name]
        print("\n######## %s (%s) ########" % (name, path.replace(os.sep, "/")), flush=True)
        t = time.time()
        code = subprocess.run([sys.executable, os.path.join(ROOT, path, "test_host", "run.py")]).returncode
        results.append((name, code, time.time() - t))
    print("\n######## summary ########")
    for name, code, dt in results:
        print("  %-8s %s  (%.0f s)" % (name, "passed" if code == 0 else "FAILED (exit %d)" % code, dt))
    sys.exit(0 if all(code == 0 for _, code, _ in results) else 1)


if __name__ == "__main__":
    main()
