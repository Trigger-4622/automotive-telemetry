#!/usr/bin/env python3
"""Turn a raw bus recording from the master (portal: Bus tab, Raw recording,
Download) into a candump log, and say what is in it.

    python master/tools/canrec.py canrec.bin                 # -> canrec.log, summary
    python master/tools/canrec.py canrec.bin -o drive.log    # another name
    python master/tools/canrec.py canrec.bin --events        # with the events, in time

The log is candump's: "(seconds) can0 1A3#0011223344556677". Frames the master
sent itself are on "tx0" (the controller does not receive its own), remote
frames end in "#R". Time is seconds from the start of the recording; each new
session (arming, a boot, a wake from sleep) starts one second after the last
frame of the one before, as the master's clock restarted.

The format is described at the top of master/src/RawRec.h.
"""
import argparse
import collections
import struct
import sys

MAGIC = b"CANREC1\0"
EVENTS = ["boot", "bus-up", "settled", "tx-guard-trip", "rx-guard-trip", "bus-off", "sleep", "resume"]
LOST_WHY = {1: "the recorder's buffer was full", 2: "the controller dropped them"}


class Error(Exception):
    pass


def uleb(b, pos):
    v = shift = 0
    while True:
        if pos >= len(b):
            raise Error("a time runs past the end")
        c = b[pos]
        pos += 1
        v |= (c & 0x7F) << shift
        shift += 7
        if not c & 0x80:
            return v, pos


def decode(raw):
    """Every record, in order: (kind, session, time_us, fields...)."""
    if raw[:8] != MAGIC:
        raise Error("not a recording from the master (no CANREC1 header)")
    version, page = struct.unpack_from("<HH", raw, 8)
    data_off, part_size, bitrate, arm_ms = struct.unpack_from("<IIII", raw, 12)
    firmware = raw[28:60].split(b"\0")[0].decode("ascii", "replace")
    info = dict(version=version, page=page, bitrate=bitrate, firmware=firmware,
                stopped=len(raw) > 64 and raw[64] == 0, partition=part_size)
    if version != 1:
        raise Error("format version %d - this tool reads version 1" % version)
    out = []
    refs = []
    t = 0
    session = 0
    pos = data_off
    while pos + page <= len(raw) and raw[pos] != 0xFF:
        end = pos + page
        while pos < end and raw[pos] != 0xFF:
            tag = raw[pos]
            pos += 1
            if tag == 0xE1:                                   # sync
                t, _frames, session, flags = struct.unpack_from("<QIHB", raw, pos)
                pos += 15
                out.append(("sync", session, t, flags))
            elif tag == 0xE2:                                 # a session begins
                t, session, reason = struct.unpack_from("<QHB", raw, pos)
                pos += 11
                refs = []
                out.append(("session", session, t, reason))
            elif tag in (0xE3, 0xE4, 0xE5):
                dt, pos = uleb(raw, pos)
                t += dt * 10
                if tag == 0xE3:
                    n, why = struct.unpack_from("<HB", raw, pos)
                    pos += 3
                    out.append(("lost", session, t, n, why))
                elif tag == 0xE4:
                    ty, a, b = struct.unpack_from("<BHH", raw, pos)
                    pos += 5
                    out.append(("event", session, t, ty, a, b))
                else:
                    out.append(("stop", session, t))
            elif tag == 0xE0:                                 # a frame in full
                dt, pos = uleb(raw, pos)
                t += dt * 10
                flags = raw[pos]
                pos += 1
                idx = None
                if flags & 8:
                    idx = raw[pos]
                    pos += 1
                if flags & 1:
                    fid = struct.unpack_from("<I", raw, pos)[0]
                    pos += 4
                else:
                    fid = struct.unpack_from("<H", raw, pos)[0]
                    pos += 2
                dlc = raw[pos]
                pos += 1
                data = b"" if flags & 2 else bytes(raw[pos:pos + dlc])
                if not flags & 2:
                    pos += dlc
                if idx is not None:
                    if idx > len(refs):
                        raise Error("index %d defined out of order at %d" % (idx, pos))
                    ref = [fid, bool(flags & 1), dlc, bytearray(data), bool(flags & 4)]
                    if idx == len(refs):
                        refs.append(ref)
                    else:
                        refs[idx] = ref
                out.append(("frame", session, t, fid, bool(flags & 1), bool(flags & 2), bool(flags & 4), dlc, data))
            elif tag < 0xE0:                                  # the bytes that changed
                if tag >= len(refs):
                    raise Error("index %d used before it was defined, at %d" % (tag, pos))
                dt, pos = uleb(raw, pos)
                t += dt * 10
                mask = raw[pos]
                pos += 1
                ref = refs[tag]
                for i in range(ref[2]):
                    if mask & (1 << i):
                        ref[3][i] = raw[pos]
                        pos += 1
                out.append(("frame", session, t, ref[0], ref[1], False, ref[4], ref[2], bytes(ref[3])))
            else:
                raise Error("unknown record 0x%02X at %d" % (tag, pos - 1))
            if pos > end:
                raise Error("a record crosses a page at %d" % pos)
        pos = end
    info["data_bytes"] = pos - data_off
    return info, out


def candump(records):
    """candump lines, sessions laid end to end a second apart; and each
    session's (start time, place in the log) for the events listing."""
    lines = []
    base = {}
    offset = None
    first = None
    last_out = 0.0
    cur = None
    for r in records:
        if r[0] == "session" or cur is None or r[1] != cur:
            cur = r[1]
            first = r[2]
            offset = 0.0 if not lines else last_out + 1.0
            base.setdefault(cur, (first, offset))
        if r[0] != "frame":
            continue
        _, _, t, fid, ext, rtr, tx, dlc, data = r
        ts = offset + (t - first) / 1e6
        last_out = ts
        ident = ("%08X" if ext else "%03X") % fid
        body = ("R" if not dlc else "R%d" % dlc) if rtr else data.hex().upper()
        lines.append("(%.6f) %s %s#%s" % (ts, "tx0" if tx else "can0", ident, body))
    return lines, base


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("recording")
    ap.add_argument("-o", "--out", help="the candump log (default: the recording's name, .log)")
    ap.add_argument("--events", action="store_true", help="list the events and losses in time")
    ap.add_argument("-q", "--quiet", action="store_true", help="no summary")
    a = ap.parse_args()
    raw = open(a.recording, "rb").read()
    try:
        info, recs = decode(raw)
    except Error as e:
        sys.exit("canrec: %s" % e)
    lines, base = candump(recs)
    out = a.out or (a.recording.rsplit(".", 1)[0] + ".log")
    with open(out, "w", newline="\n") as f:
        f.write("\n".join(lines) + ("\n" if lines else ""))
    if a.quiet:
        return
    frames = [r for r in recs if r[0] == "frame"]
    ours = sum(1 for r in frames if r[6])
    lost = collections.Counter()
    for r in recs:
        if r[0] == "lost":
            lost[r[4]] += r[3]
    sessions = sorted({r[1] for r in recs if r[0] == "session"})
    span = 0.0
    by_session = collections.defaultdict(list)
    for r in frames:
        by_session[r[1]].append(r[2])
    for ts in by_session.values():
        span += (max(ts) - min(ts)) / 1e6
    print("recording: firmware %s, %d kbit/s, %s" % (info["firmware"], info["bitrate"],
          "stopped" if info["stopped"] else "not stopped (full, or still recording when downloaded)"))
    print("%d frames (%d of them the master's own) in %.1f s of bus time, %d session(s); "
          "%.2f bytes a frame" % (len(frames), ours, span, len(sessions),
                                  info["data_bytes"] / len(frames) if frames else 0))
    if lost:
        print("lost: " + ", ".join("%d (%s)" % (n, LOST_WHY.get(w, "?")) for w, n in sorted(lost.items())))
    else:
        print("lost: none")
    ids = collections.Counter((r[3], r[4], r[6]) for r in frames)
    print("%d identifiers:" % len(ids))
    for (fid, ext, tx), n in sorted(ids.items()):
        print("  %s%s %8d frames  %7.1f /s" % (("%08X" if ext else "%03X") % fid, " (ours)" if tx else "",
                                               n, n / span if span else 0))
    if a.events:                                  # on the log's clock
        def at(r):
            first, offset = base.get(r[1], (r[2], 0.0))
            return offset + (r[2] - first) / 1e6
        for r in recs:
            if r[0] == "session":
                print("%12.6f  session %d (reset reason %d)" % (at(r), r[1], r[3]))
            elif r[0] == "event":
                name = EVENTS[r[3]] if r[3] < len(EVENTS) else "event %d" % r[3]
                print("%12.6f  %s  a=%d b=%d" % (at(r), name, r[4], r[5]))
            elif r[0] == "lost":
                print("%12.6f  %d frames lost: %s" % (at(r), r[3], LOST_WHY.get(r[4], "?")))
            elif r[0] == "stop":
                print("%12.6f  stopped" % at(r))
    print("written: %s (%d lines)" % (out, len(lines)))


if __name__ == "__main__":
    main()
