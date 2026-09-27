#!/usr/bin/env python3
"""
Generate LVGL v8 binary background assets for the 480x272 dash cluster.

Run:  python tools/make_assets.py        (writes into data/assets/)
Then: build the firmware (the tiles are compiled in) and upload a .bin from
the studio's Assets tab - never `pio run -t uploadfs` on a display in use,
it replaces the layout on it.

Output format
-------------
LVGL .bin, LV_IMG_CF_TRUE_COLOR, 16 bpp. A 4-byte header packs the bitfields
of lv_img_header_t little-endian:

    bits  0..4   color format (4 = LV_IMG_CF_TRUE_COLOR)
    bits  5..7   always_zero
    bits  8..9   reserved
    bits 10..20  width
    bits 21..31  height

Pixel bytes are written BIG-ENDIAN RGB565 ("RGB565 Swap") to match the
firmware's LV_COLOR_16_SWAP = 1, which is what lets DisplayManager stream
LVGL's buffers to the panel as zero-copy DMA. If you ever set that back to 0,
regenerate with SWAP = False below.

Design intent: these sit *behind* live gauge data, so every one of them is
dark, low-contrast and vignetted. A background that competes with the needle
is a background that makes the instrument harder to read at a glance.
"""

import math
import os
import struct

# Panel geometry. This generator is shared with the 1.28" round project; the
# only thing that differs is the canvas it draws onto, so keep these two in
# step with HardwareConfig.h.
W, H = 480, 272
SWAP = True
CF_TRUE_COLOR = 4

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "data", "assets")


def rgb565(r, g, b):
    r = max(0, min(255, int(r)))
    g = max(0, min(255, int(g)))
    b = max(0, min(255, int(b)))
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def header(w, h):
    return struct.pack("<I", CF_TRUE_COLOR | (w << 10) | (h << 21))


def write_bin(name, pixel_fn):
    """Render pixel_fn(x, y) -> (r, g, b) into an LVGL .bin file."""
    cx, cy = (W - 1) / 2.0, (H - 1) / 2.0
    radius = min(W, H) / 2.0
    out = bytearray(header(W, H))
    fmt = ">H" if SWAP else "<H"
    for y in range(H):
        row = bytearray()
        for x in range(W):
            dx, dy = x - cx, y - cy
            dist = math.hypot(dx, dy)
            if dist > radius:
                row += struct.pack(fmt, 0)      # outside the glass
                continue
            r, g, b = pixel_fn(x, y, dx, dy, dist)
            # Vignette: fade the outer 22% so gauge graphics stay dominant.
            t = dist / radius
            if t > 0.78:
                k = 1.0 - (t - 0.78) / 0.22 * 0.75
                r, g, b = r * k, g * k, b * k
            row += struct.pack(fmt, rgb565(r, g, b))
        out += row
    path = os.path.join(OUT_DIR, name)
    with open(path, "wb") as f:
        f.write(out)
    print("  {:<16} {:>7,} bytes".format(name, len(out)))


# ---------------------------------------------------------------- textures --

def carbon(x, y, dx, dy, dist):
    """Carbon-fibre twill: 8 px cells whose weave direction alternates."""
    cell, half = 16, 8
    cx_i, cy_i = (x % cell) // half, (y % cell) // half
    u = (x % half) / half
    v = (y % half) / half
    # Diagonal shading, mirrored on alternating cells -> woven look.
    grad = (u + v) if (cx_i == cy_i) else (u + (1.0 - v))
    shade = 0.5 + 0.5 * math.sin(grad * math.pi)
    base = 15 + shade * 17
    return base * 0.85, base * 0.95, base * 1.15


def brushed(x, y, dx, dy, dist):
    """
    Brushed aluminium: fine grain that runs *along* concentric circles.

    The grain must be a function of angle only. Mixing radius into it (or
    adding any low-frequency cos(angle) term) produces a bowtie-shaped seam
    across the centre instead of a turned-metal finish.
    """
    ang = math.atan2(dy, dx)
    n = math.sin(ang * 620.0) * 43758.5453
    grain = (n - math.floor(n)) * 9.0
    n2 = math.sin(ang * 71.0) * 12345.6789
    broad = (n2 - math.floor(n2)) * 5.0
    base = 24 + grain + broad
    return base * 0.92, base * 0.98, base * 1.12


def hud(x, y, dx, dy, dist):
    """Technical HUD: concentric rings plus radial hairlines."""
    r, g, b = 10, 14, 18
    ring = dist % 24.0
    if ring < 1.2:                                  # concentric ring
        r, g, b = 18, 40, 48
    ang = math.degrees(math.atan2(dy, dx)) % 15.0
    if ang < 0.5 and dist > 30:                     # radial hairline
        r, g, b = 16, 34, 42
    if dist < 26:                                   # centre boss
        k = 1.0 - dist / 26.0
        r, g, b = 10 + 14 * k, 14 + 20 * k, 18 + 26 * k
    return r, g, b


# --------------------------------------------------- compiled-in textures --
#
# Full-screen .bin backgrounds are convenient but slow: LVGL's built-in
# decoder streams file-backed images from LittleFS line by line on EVERY
# repaint, with no full-frame cache, so a 240x240 background costs ~115 KB of
# flash reads per full repaint — which is exactly what a screen transition
# does, every frame.
#
# These small tiles are emitted as C arrays instead. They live in the
# firmware image, so drawing them is a memory read with no filesystem in the
# path, and LVGL repeats them across the background for free.

TILE = 32
SRC_HEADER = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "src", "ui", "Textures.h")


def tile_carbon(x, y):
    cell, half = 16, 8
    cx_i, cy_i = (x % cell) // half, (y % cell) // half
    u, v = (x % half) / half, (y % half) / half
    grad = (u + v) if (cx_i == cy_i) else (u + (1.0 - v))
    shade = 0.5 + 0.5 * math.sin(grad * math.pi)
    base = 15 + shade * 15
    return base * 0.85, base * 0.95, base * 1.15


def tile_mesh(x, y):
    """Fine technical mesh: 1 px lines on an 8 px pitch."""
    on_line = (x % 8 == 0) or (y % 8 == 0)
    return (22, 30, 36) if on_line else (13, 17, 21)


def tile_gt_weave(x, y):
    """Tight 6 px carbon twill — finer and darker than tile_carbon.

    Reads as a texture rather than a pattern at arm's length, which is what
    keeps it in the background instead of competing with the numbers.
    """
    cell, half = 12, 6
    cx_i, cy_i = (x % cell) // half, (y % cell) // half
    u, v = (x % half) / half, (y % half) / half
    grad = (u + v) if (cx_i == cy_i) else (u + (1.0 - v))
    shade = 0.5 + 0.5 * math.sin(grad * math.pi)
    base = 11 + shade * 10
    return base * 0.94, base * 1.0, base * 1.10


# ------------------------------------------------- full-screen dial faces --
#
# A 240x240 compiled image costs ~115 KB of flash but is drawn as a straight
# blit with no filesystem in the path, so it is cheap per frame — unlike the
# .bin backgrounds, which LVGL re-reads from LittleFS on every repaint.
# DisplayManager tiles a texture only when it is smaller than the screen, so
# these are laid down once, centred.

FULL = 240

# Landscape face. Same material and mood as the round board's dial so the two
# displays read as one instrument family, but with a RECTANGULAR falloff — a
# circular vignette on a 480x272 panel leaves obvious bright corners.
FACE_W, FACE_H = 480, 272


def face_gt_wide(x, y):
    """
    Landscape GT face: the round board's carbon weave, vignetted to the edges
    of a rectangle instead of a circle.

    Kept deliberately low-contrast for the same reason as its round sibling:
    the needle, the numerals and the redline are drawn on top, and a
    background that competes with them makes the instrument harder to read.
    """
    r, g, b = tile_gt_weave(x, y)

    # Rectangular falloff: distance from the nearest edge, normalised on each
    # axis independently, so all four corners darken evenly.
    fx = min(x, FACE_W - 1 - x) / (FACE_W / 2.0)
    fy = min(y, FACE_H - 1 - y) / (FACE_H / 2.0)
    edge = min(1.0, min(fx, fy) * 2.2)          # 1.0 well inside, 0 at the rim
    k = 0.35 + 0.65 * edge
    r, g, b = r * k, g * k, b * k

    # No red index here: every dial carries its own (tex_dial_face), and on
    # the list screens a lone mark at top centre read as a stray.
    return r, g, b


def face_gt(x, y):
    """
    Dark GT instrument face: carbon weave, deep vignette, one chapter ring,
    and a red index at twelve o'clock.

    Everything is deliberately low-contrast. The needle, the numerals and the
    redline zone are drawn on top by the widget; a background that competes
    with them makes the instrument harder to read at a glance, which is the
    opposite of what a sports cluster is for.
    """
    cx = cy = (FULL - 1) / 2.0
    dx, dy = x - cx, y - cy
    dist = math.hypot(dx, dy)
    if dist > FULL / 2.0:
        return 0, 0, 0

    r, g, b = tile_gt_weave(x, y)

    # Radial falloff: faintly lit hub, deep black rim. This is the only
    # structure in the face. Concentric rings were tried here and removed —
    # against a black background they read as drawn-on grid lines and fight
    # the widget's own tick marks, which is precisely the clutter a GT dial
    # is supposed to avoid.
    k = 1.0 - 0.62 * (dist / (FULL / 2.0)) ** 2
    r, g, b = r * k, g * k, b * k

    # Single red index at twelve o'clock (atan2 gives -90 deg straight up).
    # One accent, at the one angle every driver already looks for.
    ang = math.degrees(math.atan2(dy, dx))
    if 96.0 < dist < 113.0 and abs(ang + 90.0) < 1.5:
        edge = min(dist - 96.0, 113.0 - dist) / 4.0     # soften both ends
        f = max(0.0, min(1.0, edge))
        r = r + (168 - r) * f
        g = g + (28 - g) * f
        b = b + (18 - b) * f

    return r, g, b


# The dials on the landscape screens. The round board's whole glass is its
# dial; here a dial is an instrument set INTO the panel, so it needs a face of
# its own - the same carbon, the same lit hub falling off to a black rim, the
# same red index - or the ticks and needle float on wallpaper. Pixels outside
# the circle are the chroma key (LV_COLOR_CHROMA_KEY, pure green) and are not
# drawn, so the face sits on any background; the bezel ring drawn over its
# edge hides the key's hard boundary.
DIAL = 248
CHROMA = (0, 255, 0)


def face_dial(x, y):
    c = (DIAL - 1) / 2.0
    dx, dy = x - c, y - c
    dist = math.hypot(dx, dy)
    rim = DIAL / 2.0
    if dist > rim - 0.5:
        return CHROMA
    r, g, b = tile_gt_weave(x, y)
    # A touch brighter at the hub than the round board's face (1.12 vs 1.0):
    # here the dial has to stand forward of a textured panel, not fill a
    # black bezel.
    k = 1.12 - 0.74 * (dist / rim) ** 2
    r, g, b = r * k, g * k, b * k
    ang = math.degrees(math.atan2(dy, dx))
    r0, r1 = rim * 0.80, rim * 0.94
    if r0 < dist < r1 and abs(ang + 90.0) < 1.5:
        edge = min(dist - r0, r1 - dist) / 4.0
        fr = max(0.0, min(1.0, edge))
        r = r + (168 - r) * fr
        g = g + (28 - g) * fr
        b = b + (18 - b) * fr
    return r, g, b


def emit_tiles():
    """Write src/ui/Textures.h containing every texture as an lv_img_dsc_t.

    The landscape face costs 261 KB of flash — 2.3x the round board's, because
    it is 2.3x the pixels — which fits comfortably in the 2.5 MB app
    partition and buys visual parity between the two displays.
    """
    tiles = [("tex_carbon",    tile_carbon,   TILE,   TILE,   "LV_IMG_CF_TRUE_COLOR"),
             ("tex_mesh",      tile_mesh,     TILE,   TILE,   "LV_IMG_CF_TRUE_COLOR"),
             ("tex_gt",        tile_gt_weave, TILE,   TILE,   "LV_IMG_CF_TRUE_COLOR"),
             ("tex_gt_dial",   face_gt_wide,  FACE_W, FACE_H, "LV_IMG_CF_TRUE_COLOR"),
             ("tex_dial_face", face_dial,     DIAL,   DIAL,   "LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED")]
    lines = [
        "/**",
        " * @file Textures.h",
        " * @brief GENERATED by tools/make_assets.py — do not edit by hand.",
        " *",
        " * Small tileable textures compiled into the firmware image. Unlike the",
        " * .bin files in data/assets, these need no filesystem access to draw,",
        " * so they are cheap enough to use on screens that repaint often.",
        " *",
        " * Byte order is big-endian RGB565 to match LV_COLOR_16_SWAP = 1.",
        " */",
        "#pragma once",
        "",
        "#include <lvgl.h>",
        "",
    ]
    for name, fn, tw, th, cf in tiles:
        data = bytearray()
        for y in range(th):
            for x in range(tw):
                data += struct.pack(">H", rgb565(*fn(x, y)))
        lines.append("static const uint8_t {}_map[] = {{".format(name))
        for i in range(0, len(data), 16):
            lines.append("    " + " ".join("0x%02X," % b for b in data[i:i + 16]))
        lines.append("};")
        lines.append("")
        # Positional init: lv_img_dsc_t is {header{cf,always_zero,reserved,w,h},
        # data_size, data}. Avoids relying on designated initializers, which
        # are a GCC extension in C++ and must follow declaration order.
        lines.append("static const lv_img_dsc_t {} = {{".format(name))
        lines.append("    {{ {}, 0, 0, {}, {} }},".format(cf, tw, th))
        lines.append("    sizeof({}_map),".format(name))
        lines.append("    {}_map,".format(name))
        lines.append("};")
        lines.append("")
    # UTF-8 and LF whatever the PC: the header has non-ASCII in its comments.
    with open(SRC_HEADER, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    print("  {:<16} {:>7,} bytes  ({} tiles)".format(
        "src/Textures.h", os.path.getsize(SRC_HEADER), len(tiles)))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Writing LVGL assets to {}".format(OUT_DIR))
    write_bin("bg_carbon.bin", carbon)
    write_bin("bg_brushed.bin", brushed)
    write_bin("bg_hud.bin", hud)
    emit_tiles()
    print("Done. Tiles are compiled in (layout key \"texture\"); the .bin files "
          "are for uploads (layout key \"background_asset\") — see README for "
          "the cost difference. Upload a .bin from the studio's Assets tab.")


if __name__ == "__main__":
    main()
