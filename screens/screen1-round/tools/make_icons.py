#!/usr/bin/env python3
"""
Generate the warning-lamp ("telltale") icons.

    python tools/make_icons.py

Writes
  include/TelltaleIcons.h      LV_IMG_CF_ALPHA_8BIT bitmaps, 24x24 for lamp
                               rows and 36x36 for the alert card; the lamp's
                               colour is applied at runtime (img_recolor), so
                               one bitmap serves red, amber, green and blue.
                               The card needs its own size: LVGL 8.3 cannot
                               zoom an alpha-only image (it is decoded line by
                               line), and silently draws nothing.
  tools/telltale_icons.json    the same shapes as SVG, for the Gauge Studio.
  tools/telltale_preview.png   every icon, enlarged, to check them by eye.

Each icon is a list of simple shapes on a 24x24 grid (y down). They are
rasterised with 4x4 supersampling, which gives clean anti-aliased edges
without any imaging library. The designs follow the ISO 2575 symbols a
driver already knows from the car's own cluster.

This file is identical in both display projects, like Palette.h.
"""
import json
import math
import os
import struct
import zlib

S = 24          # design grid and lamp-row size, px
BIG = 36        # alert-card size, px
SS = 4          # supersamples per axis

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ─────────────────────────────── the icons ────────────────────────────────
# Shapes: ('circle',cx,cy,r)  ('ring',cx,cy,r,w)  ('arc',cx,cy,r,w,a0,a1)
#         ('rect',x0,y0,x1,y1)  ('rbox',x0,y0,x1,y1,rad,w)  ('poly',[(x,y)..])
#         ('line',x0,y0,x1,y1,w)  ('cut',shape) erases what was drawn before it.
# Shapes paint in order, so a line after a cut draws inside the gap.
# Angles in degrees, clockwise from east (screen coordinates).
ICONS = {
    # Check engine: the engine block with its valve cover, intake and exhaust.
    "mil": [
        ("rbox", 5.5, 8.5, 18, 19, 1, 2),
        ("rect", 8, 4.5, 15, 6.5),
        ("line", 11.5, 6.5, 11.5, 8.5, 2),
        ("line", 2.2, 11, 2.2, 17, 2),
        ("line", 2.2, 14, 5.5, 14, 2),
        ("line", 18, 11.5, 21.5, 9, 2),
        ("line", 21.5, 9, 21.5, 18.5, 2),
        ("line", 21.5, 18.5, 18, 16, 2),
    ],
    # Fasten seat belt: a seated figure with the belt across the chest.
    "seatbelt": [
        ("circle", 12, 4.2, 2.9),
        ("poly", [(7.4, 8.8), (16.6, 8.8), (18.2, 13), (17.4, 21.8),
                  (6.6, 21.8), (5.8, 13)]),
        ("cut", ("line", 18.6, 7.4, 7.0, 22.8, 3.6)),
        ("line", 17.0, 9.2, 8.6, 20.8, 1.5),
    ],
    # Door open: the car from above with one door swung out.
    "door": [
        ("rbox", 8, 2, 17.5, 22, 3, 2),
        ("line", 10, 7, 15.5, 7, 1.6),
        ("line", 10, 17.5, 15.5, 17.5, 1.6),
        ("cut", ("rect", 7, 9.2, 9.4, 14.2)),
        ("line", 8.2, 9.4, 3, 14.2, 2),
    ],
    # Parking brake: (P) - the letter in a circle, with brackets.
    "park": [
        ("ring", 12, 12, 7.2, 1.9),
        ("line", 10.3, 8.4, 10.3, 15.8, 1.9),
        ("line", 10.3, 8.4, 12.2, 8.4, 1.8),
        ("line", 10.3, 12.6, 12.2, 12.6, 1.8),
        ("arc", 12.2, 10.5, 2.1, 1.8, -90, 90),
        ("arc", 12, 12, 10.6, 1.8, 135, 225),
        ("arc", 12, 12, 10.6, 1.8, -45, 45),
    ],
    "turn_l": [
        ("poly", [(1.5, 12), (10, 3.8), (10, 8.6), (21.5, 8.6),
                  (21.5, 15.4), (10, 15.4), (10, 20.2)]),
    ],
    "turn_r": [
        ("poly", [(22.5, 12), (14, 3.8), (14, 8.6), (2.5, 8.6),
                  (2.5, 15.4), (14, 15.4), (14, 20.2)]),
    ],
    # High beam: the lamp with level beams.
    "high_beam": [
        ("line", 13.5, 5, 13.5, 19, 2),
        ("arc", 13.5, 12, 7, 2, -90, 90),
        ("line", 2, 6.5, 10, 6.5, 2),
        ("line", 2, 10, 10, 10, 2),
        ("line", 2, 14, 10, 14, 2),
        ("line", 2, 17.5, 10, 17.5, 2),
    ],
    # Low beam / lights on: the lamp with dipped beams.
    "lights": [
        ("line", 13.5, 5, 13.5, 19, 2),
        ("arc", 13.5, 12, 7, 2, -90, 90),
        ("line", 2.5, 9.5, 10, 7, 2),
        ("line", 2.5, 13.5, 10, 11, 2),
        ("line", 2.5, 17.5, 10, 15, 2),
    ],
    # Cruise control: a speedometer with its needle.
    "cruise": [
        ("arc", 12, 13.5, 8.6, 2, 135, 405),
        ("line", 12, 13.5, 16.8, 8.6, 2.1),
        ("circle", 12, 13.5, 2.2),
        ("line", 12, 19.5, 12, 22.5, 2),
        ("line", 8, 22.5, 16, 22.5, 2),
    ],
    # Low fuel: the pump, its window and the hose.
    "fuel": [
        ("rbox", 4, 4.5, 13.5, 20.5, 1.5, 2),
        ("rect", 6.5, 7, 11, 10.8),
        ("line", 2.5, 21.2, 15, 21.2, 2),
        ("line", 13.5, 8, 16.4, 8, 1.8),
        ("line", 16.4, 8, 19.2, 10.8, 1.8),
        ("line", 19.2, 10.8, 19.2, 17.6, 1.8),
        ("line", 19.2, 17.6, 17.2, 17.6, 1.8),
    ],
    # Charging fault: the battery with its terminals and signs.
    "battery": [
        ("rbox", 2.5, 7, 21.5, 19.5, 1.2, 2),
        ("rect", 5, 4.3, 9.4, 7),
        ("rect", 14.6, 4.3, 19, 7),
        ("line", 5.8, 13.2, 9.8, 13.2, 1.9),
        ("line", 14.2, 13.2, 18.2, 13.2, 1.9),
        ("line", 16.2, 11.2, 16.2, 15.2, 1.9),
    ],
    # Oil pressure: the oil can with a drop at the spout.
    "oil": [
        ("poly", [(4.5, 11.2), (14.6, 11.2), (16.4, 13.6),
                  (16.4, 17.2), (4.5, 17.2)]),
        ("rect", 8, 8.4, 11, 11.2),
        ("line", 6.5, 8.2, 12.5, 8.2, 1.6),
        ("line", 15.8, 13, 21.4, 9.6, 2.2),
        ("arc", 4.5, 14.2, 2.6, 1.8, 90, 270),
        ("circle", 21.2, 15.2, 1.45),
        ("poly", [(21.2, 12.2), (19.9, 14.9), (22.5, 14.9)]),
    ],
    # Coolant temperature: the thermometer standing in waves.
    "coolant": [
        ("line", 12, 3.2, 12, 13, 3.2),
        ("circle", 12, 15, 3.1),
        ("line", 14.8, 4.6, 17.4, 4.6, 1.6),
        ("line", 14.8, 7.6, 17.4, 7.6, 1.6),
        ("line", 14.8, 10.6, 17.4, 10.6, 1.6),
        ("line", 2, 20.2, 5, 19, 1.5), ("line", 5, 19, 8, 20.2, 1.5),
        ("line", 8, 20.2, 9.8, 19.5, 1.5),
        ("line", 14.2, 19.5, 16, 20.2, 1.5), ("line", 16, 20.2, 19, 19, 1.5),
        ("line", 19, 19, 22, 20.2, 1.5),
        ("line", 2, 23, 5, 21.8, 1.5), ("line", 5, 21.8, 8, 23, 1.5),
        ("line", 8, 23, 11, 21.8, 1.5), ("line", 11, 21.8, 14, 23, 1.5),
        ("line", 14, 23, 17, 21.8, 1.5), ("line", 17, 21.8, 20, 23, 1.5),
        ("line", 20, 23, 22, 22.2, 1.5),
    ],
    # Knock: a bolt - the engine is pulling timing.
    "knock": [
        ("poly", [(14, 1.5), (5, 13.5), (11, 13.5), (8.5, 22.5),
                  (19.5, 9), (13.2, 9), (16, 1.5)]),
    ],
    # Generic warning, for the user-defined lamps: a triangle with "!".
    "warn": [
        ("poly", [(12, 2.2), (23, 21.2), (1, 21.2)]),
        ("cut", ("line", 12, 8.6, 12, 14.4, 2.4)),
        ("cut", ("circle", 12, 17.6, 1.5)),
    ],
}

# ─────────────────────────────── geometry ─────────────────────────────────


def _seg_dist(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L))
    qx, qy = x0 + t * dx, y0 + t * dy
    return math.hypot(px - qx, py - qy)


def _in_poly(px, py, pts):
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _in_rrect(px, py, x0, y0, x1, y1, r):
    if px < x0 or px > x1 or py < y0 or py > y1:
        return False
    cx = min(max(px, x0 + r), x1 - r)
    cy = min(max(py, y0 + r), y1 - r)
    return math.hypot(px - cx, py - cy) <= r


def _ang_in(a, a0, a1):
    a = a % 360
    a0m = a0 % 360
    span = (a1 - a0)
    if span >= 360:
        return True
    return (a - a0m) % 360 <= span


def inside(sh, px, py):
    k = sh[0]
    if k == "circle":
        return math.hypot(px - sh[1], py - sh[2]) <= sh[3]
    if k == "ring":
        d = math.hypot(px - sh[1], py - sh[2])
        return abs(d - sh[3]) <= sh[4] / 2
    if k == "arc":
        _, cx, cy, r, w, a0, a1 = sh
        d = math.hypot(px - cx, py - cy)
        if abs(d - r) <= w / 2 and _ang_in(math.degrees(math.atan2(py - cy, px - cx)), a0, a1):
            return True
        for a in (a0, a1):           # round caps
            ex = cx + r * math.cos(math.radians(a))
            ey = cy + r * math.sin(math.radians(a))
            if math.hypot(px - ex, py - ey) <= w / 2:
                return True
        return False
    if k == "rect":
        return sh[1] <= px <= sh[3] and sh[2] <= py <= sh[4]
    if k == "rbox":
        _, x0, y0, x1, y1, r, w = sh
        return (_in_rrect(px, py, x0 - w / 2, y0 - w / 2, x1 + w / 2, y1 + w / 2, r + w / 2)
                and not _in_rrect(px, py, x0 + w / 2, y0 + w / 2, x1 - w / 2, y1 - w / 2,
                                  max(0.0, r - w / 2)))
    if k == "poly":
        return _in_poly(px, py, sh[1])
    if k == "line":
        return _seg_dist(px, py, *sh[1:5]) <= sh[5] / 2
    raise ValueError(k)


def rasterise(shapes, size=S):
    """Render at size x size; the shapes are drawn on the 24-unit grid."""
    k = S / size
    out = bytearray(size * size)
    for y in range(size):
        for x in range(size):
            hit = 0
            for sy in range(SS):
                for sx in range(SS):
                    px = (x + (sx + 0.5) / SS) * k
                    py = (y + (sy + 0.5) / SS) * k
                    on = False
                    for s in shapes:
                        if s[0] == "cut":
                            if on and inside(s[1], px, py):
                                on = False
                        elif not on and inside(s, px, py):
                            on = True
                    hit += on
            out[y * size + x] = round(255 * hit / (SS * SS))
    return bytes(out)

# ─────────────────────────────── SVG (studio) ─────────────────────────────


def _svg_shape(sh, fill, bg):
    k = sh[0]
    if k == "cut":
        return _svg_shape(sh[1], bg, bg)
    f = lambda v: ("%.2f" % v).rstrip("0").rstrip(".")
    if k == "circle":
        return '<circle cx="%s" cy="%s" r="%s" fill="%s"/>' % (f(sh[1]), f(sh[2]), f(sh[3]), fill)
    if k == "ring":
        return ('<circle cx="%s" cy="%s" r="%s" fill="none" stroke="%s" stroke-width="%s"/>'
                % (f(sh[1]), f(sh[2]), f(sh[3]), fill, f(sh[4])))
    if k == "arc":
        _, cx, cy, r, w, a0, a1 = sh
        x0, y0 = cx + r * math.cos(math.radians(a0)), cy + r * math.sin(math.radians(a0))
        x1, y1 = cx + r * math.cos(math.radians(a1)), cy + r * math.sin(math.radians(a1))
        large = 1 if (a1 - a0) % 360 > 180 or a1 - a0 >= 360 else 0
        return ('<path d="M%s %sA%s %s 0 %d 1 %s %s" fill="none" stroke="%s" stroke-width="%s" '
                'stroke-linecap="round"/>' % (f(x0), f(y0), f(r), f(r), large, f(x1), f(y1), fill, f(w)))
    if k == "rect":
        return '<rect x="%s" y="%s" width="%s" height="%s" fill="%s"/>' % (
            f(sh[1]), f(sh[2]), f(sh[3] - sh[1]), f(sh[4] - sh[2]), fill)
    if k == "rbox":
        _, x0, y0, x1, y1, r, w = sh
        return ('<rect x="%s" y="%s" width="%s" height="%s" rx="%s" fill="none" stroke="%s" '
                'stroke-width="%s"/>' % (f(x0), f(y0), f(x1 - x0), f(y1 - y0), f(r), fill, f(w)))
    if k == "poly":
        return '<polygon points="%s" fill="%s"/>' % (
            " ".join("%s,%s" % (f(x), f(y)) for x, y in sh[1]), fill)
    if k == "line":
        return ('<line x1="%s" y1="%s" x2="%s" y2="%s" stroke="%s" stroke-width="%s" '
                'stroke-linecap="round"/>' % (f(sh[1]), f(sh[2]), f(sh[3]), f(sh[4]), fill, f(sh[5])))
    raise ValueError(k)


def svg(shapes):
    # "currentColor" lets the studio colour each lamp with CSS; cuts are drawn
    # in the page background, which is what the firmware's transparency shows.
    return "".join(_svg_shape(s, "currentColor", "var(--iconbg,#0b1014)") for s in shapes)

# ─────────────────────────────── outputs ──────────────────────────────────


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + bytes(rgb[y * w * 3:(y + 1) * w * 3]) for y in range(h))

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    with open(path, "wb") as fh:
        fh.write(png)


def main():
    maps = {name: rasterise(shapes) for name, shapes in ICONS.items()}

    lines = [
        "/**",
        " * @file TelltaleIcons.h",
        " * @brief GENERATED by tools/make_icons.py - do not edit by hand.",
        " *",
        " * Warning-lamp symbols as 8-bit alpha masks: icon_<name> is 24x24, for",
        " * lamp rows; icon_<name>_36 is 36x36, for the alert card. The colour comes",
        " * from the image's img_recolor style at runtime, so the same bitmap draws",
        " * a red, amber, green or blue lamp. Include from exactly one translation unit.",
        " */",
        "#pragma once",
        "",
        "#include <lvgl.h>",
        "",
    ]
    for suffix, size in (("", S), ("_36", BIG)):
        for name, shapes in ICONS.items():
            data = maps[name] if size == S else rasterise(shapes, size)
            sym = "icon_%s%s" % (name, suffix)
            lines.append("static const uint8_t %s_map[] = {" % sym)
            for i in range(0, len(data), 24):
                lines.append("    " + " ".join("0x%02X," % b for b in data[i:i + 24]))
            lines.append("};")
            lines.append("static const lv_img_dsc_t %s = {" % sym)
            lines.append("    { LV_IMG_CF_ALPHA_8BIT, 0, 0, %d, %d }," % (size, size))
            lines.append("    sizeof(%s_map)," % sym)
            lines.append("    %s_map," % sym)
            lines.append("};")
            lines.append("")
    hdr = os.path.join(ROOT, "include", "TelltaleIcons.h")
    with open(hdr, "w", newline="\n") as fh:
        fh.write("\n".join(lines))

    with open(os.path.join(HERE, "telltale_icons.json"), "w", newline="\n") as fh:
        json.dump({n: svg(s) for n, s in ICONS.items()}, fh, indent=0)

    # Preview sheet: each icon at 4x, lit in its usual colour on the dark UI.
    cols, scale, pad = 8, 4, 8
    rows = (len(maps) + cols - 1) // cols
    W = cols * (S * scale + pad) + pad
    H = rows * (S * scale + pad) + pad
    rgb = bytearray([16, 20, 24] * W * H)
    tint = {"mil": (255, 196, 0), "fuel": (255, 196, 0), "knock": (255, 196, 0),
            "turn_l": (105, 240, 174), "turn_r": (105, 240, 174), "lights": (105, 240, 174),
            "cruise": (105, 240, 174), "high_beam": (68, 138, 255),
            "warn": (255, 196, 0)}
    for i, (name, data) in enumerate(maps.items()):
        ox = pad + (i % cols) * (S * scale + pad)
        oy = pad + (i // cols) * (S * scale + pad)
        c = tint.get(name, (255, 23, 68))
        for y in range(S * scale):
            for x in range(S * scale):
                a = data[(y // scale) * S + x // scale] / 255.0
                p = ((oy + y) * W + ox + x) * 3
                for k in range(3):
                    rgb[p + k] = int(rgb[p + k] * (1 - a) + c[k] * a)
    write_png(os.path.join(HERE, "telltale_preview.png"), W, H, rgb)
    print("%d icons -> include/TelltaleIcons.h (%d bytes of bitmaps)" % (
        len(maps), sum(len(d) for d in maps.values())))


if __name__ == "__main__":
    main()
