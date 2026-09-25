#!/usr/bin/env python3
"""
Read the files out of a LittleFS 2.x image (a flash backup's filesystem
partition), without mounting or modifying anything.

    python tools/littlefs_read.py image.bin outdir [--block-size 4096]

Exists because PlatformIO's bundled mklittlefs predates on-disk format 2.1,
which the Arduino core's LittleFS writes, and refuses such images as
"corrupted". Follows the format in littlefs's SPEC.md: metadata pairs of
CRC-checked commits (tags big-endian, each XORed with the one before), files
either inline in the metadata or in a CTZ skip-list of blocks.
"""
import os
import struct
import sys
import zlib


def be32(b, o):
    return struct.unpack_from(">I", b, o)[0]


def le32(b, o):
    return struct.unpack_from("<I", b, o)[0]


class LFS:
    def __init__(self, img, bs):
        self.img, self.bs = img, bs

    def block(self, n):
        return self.img[n * self.bs:(n + 1) * self.bs]

    def fetch(self, n):
        """Replay one metadata block. Returns (revision, entries, tail) as of
        its last valid commit, or None when it holds none."""
        b = self.block(n)
        rev = le32(b, 0)
        off, ptag = 4, 0xFFFFFFFF
        pending = []          # tags since the last good commit
        committed = None
        crc_start = 0
        while off + 4 <= len(b):
            raw = be32(b, off)
            tag = raw ^ ptag
            if tag & 0x80000000:
                break
            typ, tid, ln = (tag >> 20) & 0x7FF, (tag >> 10) & 0x3FF, tag & 0x3FF
            dlen = 0 if ln == 0x3FF else ln
            if off + 4 + dlen > len(b):
                break
            if (typ & 0x780) == 0x500:                  # commit CRC
                want = le32(b, off + 4)
                got = zlib.crc32(b[crc_start:off + 4]) ^ 0xFFFFFFFF
                if got != want:
                    break
                committed = list(pending) if committed is None else committed + pending
                pending = []
                ptag = tag ^ (((typ & 1)) << 31)
                off += 4 + dlen
                crc_start = off
                continue
            pending.append((typ, tid, b[off + 4:off + 4 + dlen], ln == 0x3FF))
            ptag = tag
            off += 4 + dlen
        if committed is None:
            return None
        entries, tail = [], None
        for typ, tid, data, deleted in committed:
            if typ == 0x401:                            # create
                entries.insert(tid, {})
            elif typ == 0x4FF:                          # delete
                if tid < len(entries):
                    entries.pop(tid)
            elif typ in (0x600, 0x601):                 # soft / hard tail
                tail = (typ, le32(data, 0), le32(data, 4))
            elif (typ & 0x700) in (0x000, 0x200) and tid != 0x3FF:
                while tid >= len(entries):
                    entries.append({})
                key = "name" if (typ & 0x700) == 0x000 else "struct"
                entries[tid][key] = (typ, data)
        return rev, entries, tail

    def pair(self, a, b):
        fa, fb = self.fetch(a), self.fetch(b)
        if fa and fb:
            # sequence comparison, as littlefs does
            return fa if ((fa[0] - fb[0]) & 0xFFFFFFFF) < 0x80000000 else fb
        return fa or fb

    def read_ctz(self, head, size):
        if size == 0:
            return b""
        b = self.bs - 8

        def popc(x):
            return bin(x).count("1")

        def index(off):
            i = off // b
            if i == 0:
                return 0
            return (off - 4 * (popc(i - 1) + 2)) // b
        last = index(size - 1)
        blocks = [0] * (last + 1)
        blocks[last] = head
        for n in range(last, 0, -1):
            blocks[n - 1] = le32(self.block(blocks[n]), 0)
        out = bytearray()
        for n, blk in enumerate(blocks):
            skip = 0 if n == 0 else 4 * ((n & -n).bit_length())   # ctz(n)+1 pointers
            out += self.block(blk)[skip:]
        return bytes(out[:size])

    def walk(self, pair=(0, 1), path=""):
        seen = set()
        while pair and pair not in seen:
            seen.add(pair)
            got = self.pair(*pair)
            if not got:
                return
            _, entries, tail = got
            for e in entries:
                if "name" not in e:
                    continue
                typ, name = e["name"]
                if typ == 0x0FF:                        # superblock entry
                    continue
                full = path + "/" + name.decode("utf-8", "replace")
                st = e.get("struct")
                if typ == 0x002 and st:                 # directory
                    yield full, None
                    yield from self.walk((le32(st[1], 0), le32(st[1], 4)), full)
                elif typ == 0x001 and st:
                    styp, data = st
                    if styp == 0x201:                   # inline
                        yield full, data
                    elif styp == 0x202:                 # CTZ
                        yield full, self.read_ctz(le32(data, 0), le32(data, 4))
            pair = (tail[1], tail[2]) if tail and tail[0] == 0x601 else None


def main():
    img, out = sys.argv[1], sys.argv[2]
    bs = int(sys.argv[sys.argv.index("--block-size") + 1]) if "--block-size" in sys.argv else 4096
    fs = LFS(open(img, "rb").read(), bs)
    os.makedirs(out, exist_ok=True)
    n = 0
    for path, data in fs.walk():
        dst = os.path.join(out, path.lstrip("/"))
        if data is None:
            os.makedirs(dst, exist_ok=True)
            print("dir  %s" % path)
            continue
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as fh:
            fh.write(data)
        print("file %s  %d bytes" % (path, len(data)))
        n += 1
    print("%d file(s) extracted to %s" % (n, out))


if __name__ == "__main__":
    main()
