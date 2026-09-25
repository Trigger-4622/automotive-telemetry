#!/usr/bin/env python3
"""Run every test: the sync checks, the master in a simulated car, and both
screens rendering real LVGL into PNG screenshots.

    python scripts/test_all.py                   # everything
    python scripts/test_all.py master screen2    # some of: sync master screen1 screen2

The harnesses build the unmodified firmware sources for the PC first, so the
first run takes a few minutes and later ones far less. They need g++ and the
libraries in each project's .pio/libdeps (a firmware build, or
scripts/cloud-setup.sh, puts them there). Exit code 0 = everything passed.
On GitHub Actions the results table also goes on the run's summary page.
"""
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUITES = {
    "sync": ["scripts", "check_sync.py"],
    "master": ["master", "test_host", "run.py"],
    "screen1": ["screens", "screen1-round", "test_host", "run.py"],
    "screen2": ["screens", "screen2-cluster", "test_host", "run.py"],
}


def run(script):
    """Run one suite, echoing its output as it comes, and keep the lines."""
    p = subprocess.Popen([sys.executable, "-u", os.path.join(ROOT, *script)], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
    lines = []
    for line in p.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        lines.append(line.rstrip("\n"))
    return p.wait(), lines


def tally(name, lines):
    """Scenarios (or checks) run and failed, from that suite's output format,
    plus anything worth repeating in the summary."""
    ran, failed, notes = 0, [], []
    for line in lines:
        if name == "sync":                                          # "  ok   what" / "  FAIL what"
            m = re.match(r"^  (ok  |FAIL) (.+)$", line)
            if m:
                ran += 1
                if m.group(1) == "FAIL":
                    failed.append(m.group(2))
        elif name == "master":                                      # "=== name (t s) ===" per scenario
            if re.match(r"^=== \S+ \(", line):
                ran += 1
            m = re.match(r"^FAILED: (.+)$", line)
            if m:
                failed += [n.strip() for n in m.group(1).split(",")]
        else:                                                       # "ok   name   1.2s" per scenario
            m = re.match(r"^(ok|FAIL)\s+(\S+)\s+[\d.]+s$", line)
            if m:
                ran += 1
                if m.group(1) == "FAIL":
                    failed.append(m.group(2))
            m = re.search(r"LVGL heap after touring every screen: device ~(\d+) B.*?peak of (\d+)", line)
            if m:
                notes.append("LVGL heap ~%.1f of %.0f KB" % (int(m.group(1)) / 1024, int(m.group(2)) / 1024))
    return ran, failed, notes


def main():
    picks = sys.argv[1:] or list(SUITES)
    unknown = [p for p in picks if p not in SUITES]
    if unknown:
        sys.exit("unknown suite(s): %s - choose from %s" % (", ".join(unknown), ", ".join(SUITES)))
    rows = []
    for name in picks:
        print("\n######## %s ########" % name, flush=True)
        t = time.time()
        code, lines = run(SUITES[name])
        ran, failed, notes = tally(name, lines)
        rows.append((name, code, ran, failed, notes, time.time() - t))

    print("\n######## summary ########")
    md = ["| Suite | Result | Passed | Notes |", "|---|---|---|---|"]
    for name, code, ran, failed, notes, dt in rows:
        ok = code == 0
        counts = "%d/%d" % (ran - len(failed), ran) if ran else "-"
        extra = notes + (["failed: " + ", ".join(failed)] if failed else [])
        if not ok and not failed:
            extra.append("exit code %d - see the log" % code)
        print("  %-8s %-7s %-8s %s  (%.0f s)" % (name, "passed" if ok else "FAILED", counts,
                                                 "; ".join(extra), dt))
        md.append("| %s | %s | %s | %s |" % (name, "✅ passed" if ok else "❌ FAILED", counts,
                                            "<br>".join(extra)))
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary:
        with open(summary, "a", encoding="utf-8") as fh:
            fh.write("\n".join(md) + "\n")
    sys.exit(0 if all(r[1] == 0 for r in rows) else 1)


if __name__ == "__main__":
    main()
