#!/usr/bin/env python3
"""Turn a raw bus recording from the master (portal: Bus tab, Raw recording,
Download) into a candump log, and say what is in it.

    python master/tools/canrec.py canrec.bin                 # -> canrec.log, summary
    python master/tools/canrec.py canrec.bin -o drive.log    # another name
    python master/tools/canrec.py canrec.bin --events        # with the events, in time

The log is candump's: "(seconds) can0 1A3#0011223344556677". Frames the master
sent itself are on "tx0" (the controller does not receive its own), remote
frames end in "#R", and a DLC of 9-15 (legal on the wire, 8 bytes follow) in
"_<hex DLC>" as candump writes it. Time is seconds from the start of the
recording; each new session (arming, a boot, a wake from sleep) starts one
second after the last frame of the one before, as the master's clock
restarted. Bus errors (error frames) and the moments the master could not hear
the bus are in the summary and, with --events, in time.

The format is described at the top of master/src/RawRec.h.
"""
import argparse
import collections
import struct
import sys

MAGIC = b"CANREC1\0"
EVENTS = ["boot", "bus-up", "settled", "tx-guard-trip", "rx-guard-trip", "bus-off", "sleep", "resume"]
LOST_WHY = {1: "the recorder's buffer was full", 2: "the controller dropped them"}
ERR_KIND = ["BIT", "FORM", "STUFF", "OTHER"]
# SJA1000 / ESP32 TWAI Error Code Capture (ECC[4:0]) segment names.
ERR_WHERE = {0x03: "SOF", 0x02: "ID28-21", 0x06: "ID20-18", 0x04: "SRTR", 0x05: "IDE",
             0x07: "ID17-13", 0x0F: "ID12-5", 0x0E: "ID4-0", 0x0C: "RTR", 0x0D: "RES1",
             0x09: "RES0", 0x0B: "DLC", 0x0A: "DATA", 0x08: "CRC", 0x18: "CRC_DELIM",
             0x19: "ACK_SLOT", 0x1B: "ACK_DELIM", 0x1A: "EOF", 0x12: "INTERMISSION",
             0x11: "ACTIVE_ERR_FLAG", 0x16: "PASSIVE_ERR_FLAG", 0x13: "tolerate_dominant",
             0x17: "ERR_DELIM", 0x1C: "OVERLOAD"}


def err_name(ecc):
    if ecc == 0:
        return "cleared/unavailable (0x00)"
    kind = ERR_KIND[(ecc >> 6) & 3]
    direction = "RX" if (ecc & 0x20) else "TX"
    seg = ERR_WHERE.get(ecc & 0x1F, "seg=0x%02X" % (ecc & 0x1F))
    return "%s/%s/%s (0x%02X)" % (kind, direction, seg, ecc)


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
    if version not in (1, 2):
        raise Error("format version %d - this tool reads versions 1 and 2" % version)
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
            elif tag in (0xE3, 0xE4, 0xE5, 0xE6, 0xE7):
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
                elif tag == 0xE6:                             # bus errors (error frames)
                    n, ecc, flags, tec, rec = struct.unpack_from("<HBBBB", raw, pos)
                    pos += 6
                    out.append(("errors", session, t, n, ecc, flags, tec, rec))
                elif tag == 0xE7:                             # the master could not hear the bus
                    n = struct.unpack_from("<H", raw, pos)[0]
                    pos += 2
                    out.append(("deaf", session, t, n))
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
                nb = min(dlc, 8)                              # a DLC of 9-15 carries 8 bytes
                data = b"" if flags & 2 else bytes(raw[pos:pos + nb])
                if not flags & 2:
                    pos += nb
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
                for i in range(min(ref[2], 8)):
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
        if not rtr and dlc > 8:
            body += "_%X" % dlc                           # candump's len8_dlc
        lines.append("(%.6f) %s %s#%s" % (ts, "tx0" if tx else "can0", ident, body))
    return lines, base


def session_stats(recs):
    """Per-session Bus-Off counts (0xE4 type=5), summed 0xE6 errors, and
    engine-running vs KOEO split (tracking RPM from 0x7E8 PID 0C and 0x231)."""
    stats = collections.OrderedDict()
    rpm_by_sess = {}
    obd_rpm_seen = {}

    def get(s):
        if s not in stats:
            stats[s] = dict(reason=None, frames=0, tx=0, t_min=None, t_max=None,
                            bus_offs=0, bo_run=0, bo_koeo=0,
                            err_total=0, err_run=0, err_koeo=0,
                            run_frames=0, koeo_frames=0, ecc=collections.Counter())
        return stats[s]

    for r in recs:
        kind, sess, t = r[0], r[1], r[2]
        st = get(sess)
        if kind == "session":
            st["reason"] = r[3]
            rpm_by_sess[sess] = 0.0
            obd_rpm_seen[sess] = False
            continue
        if kind == "frame":
            _, _, _, fid, ext, rtr, tx, dlc, data = r
            st["frames"] += 1
            if tx:
                st["tx"] += 1
            if st["t_min"] is None or t < st["t_min"]:
                st["t_min"] = t
            if st["t_max"] is None or t > st["t_max"]:
                st["t_max"] = t
            if not ext and not rtr:
                # OBD-II PID 0x0C reply (0x7E8: 04 41 0C A B)
                if fid == 0x7E8 and len(data) >= 5 and data[0] >= 4 and data[1] == 0x41 and data[2] == 0x0C:
                    rpm_by_sess[sess] = ((data[3] << 8) | data[4]) / 4.0
                    obd_rpm_seen[sess] = True
                # Subaru BM/EJ204 broadcast RPM on 0x231 bytes 0-1 LE (1 rpm/bit)
                elif fid == 0x231 and len(data) >= 2 and not obd_rpm_seen.get(sess):
                    cand = struct.unpack_from("<H", data, 0)[0]
                    if cand <= 10000:
                        rpm_by_sess[sess] = float(cand)
            if rpm_by_sess.get(sess, 0.0) > 0.0:
                st["run_frames"] += 1
            else:
                st["koeo_frames"] += 1
        elif kind == "event" and r[3] == 5:                       # EV_BUS_OFF
            st["bus_offs"] += 1
            if rpm_by_sess.get(sess, 0.0) > 0.0:
                st["bo_run"] += 1
            else:
                st["bo_koeo"] += 1
        elif kind == "errors":
            cnt, ecc = r[3], r[4]
            st["err_total"] += cnt
            st["ecc"][ecc] += cnt
            if rpm_by_sess.get(sess, 0.0) > 0.0:
                st["err_run"] += cnt
            else:
                st["err_koeo"] += cnt
    return stats


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
    errs = collections.Counter()
    for r in recs:
        if r[0] == "errors":
            errs[r[4]] += r[3]
    bus_offs_total = sum(1 for r in recs if r[0] == "event" and r[3] == 5)
    if errs:
        print("bus errors (error frames): %d - %s" % (sum(errs.values()), ", ".join(
            "%d %s" % (n, err_name(e)) for e, n in errs.most_common())))
    else:
        print("bus errors: none")
    print("bus-offs (0xE4 type=5): %d" % bus_offs_total)
    sstats = session_stats(recs)
    if sstats:
        print("sessions:")
        for sid, st in sstats.items():
            dur = ((st["t_max"] - st["t_min"]) / 1e6) if st["t_min"] is not None else 0.0
            run_pct = (100.0 * st["run_frames"] / st["frames"]) if st["frames"] else 0.0
            rsn = ("reason %d" % st["reason"]) if st["reason"] is not None else "no header"
            print("  session %d (%s): %d frames (%d tx) in %.1f s (%.1f%% running), "
                  "bus-offs: %d (%d running, %d KOEO), errors: %d (%d running, %d KOEO)" % (
                      sid, rsn, st["frames"], st["tx"], dur, run_pct,
                      st["bus_offs"], st["bo_run"], st["bo_koeo"],
                      st["err_total"], st["err_run"], st["err_koeo"]))
    deaf = sum(r[3] for r in recs if r[0] == "deaf")
    print("the master could not hear the bus: %s" % ("%d time(s) (a flash write, reset mode, a reinstall)" % deaf
                                                     if deaf else "never"))
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
                extra = ""
                if r[3] == 5 and r[5] == 2:
                    extra = " (storm -> listen-only)"
                elif r[3] == 5 and r[5] == 1:
                    extra = " (rx-guard -> listen-only)"
                print("%12.6f  %s%s  a=%d b=%d" % (at(r), name, extra, r[4], r[5]))
            elif r[0] == "lost":
                print("%12.6f  %d frames lost: %s" % (at(r), r[3], LOST_WHY.get(w, "?") if (w := r[4]) is not None else "?"))
            elif r[0] == "stop":
                print("%12.6f  stopped" % at(r))
            elif r[0] == "errors":
                print("%12.6f  %d bus error(s): %s%s%s, TEC %d REC %d" % (
                    at(r), r[3], err_name(r[4]), ", listen-only" if r[5] & 1 else "",
                    ", while the master sent" if r[5] & 2 else "", r[6], r[7]))
            elif r[0] == "deaf":
                print("%12.6f  the master could not hear the bus (%d time(s))" % (at(r), r[3]))
    print("written: %s (%d lines)" % (out, len(lines)))


if __name__ == "__main__":
    main()
