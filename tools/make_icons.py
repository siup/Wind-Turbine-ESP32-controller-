#!/usr/bin/env python3
"""Draws the panel icons data/icon-192.png and data/icon-512.png.

Python 3 standard library only (zlib + struct). The glyph is the same as
data/icon.svg: a graphite tile with a thin frame, a grey tower and three amber
blades around a hub. Edges are anti-aliased by 4x4 supersampling, but only in
pixels whose corners do not all fall on the same shape, so the script stays
fast. The PNG is written as an indexed image when the colours fit in a
palette (they do), which keeps each file a few kilobytes: the LittleFS
partition is only 128 kB.

    python3 tools/make_icons.py            # writes into data/
    python3 tools/make_icons.py out_dir/   # somewhere else
"""

import math
import os
import struct
import sys
import zlib

# Geometry in the 512 x 512 units of data/icon.svg.
GRAPHITE = (0x0F, 0x11, 0x14, 255)
FRAME = (0x2A, 0x2F, 0x37, 255)
TOWER = (0x7D, 0x85, 0x90, 255)
AMBER = (0xF2, 0xA9, 0x00, 255)
CLEAR = (0, 0, 0, 0)

HUB = (256.0, 236.0)
HUB_INNER = 21.0  # r 26, stroke 10: dark inside 21, amber ring 21..31
HUB_OUTER = 31.0


def rounded_rect_sdf(px, py, x0, y0, x1, y1, r):
    """Signed distance to a rounded rectangle (negative inside)."""
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hx, hy = (x1 - x0) / 2.0 - r, (y1 - y0) / 2.0 - r
    qx, qy = abs(px - cx) - hx, abs(py - cy) - hy
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    return outside + inside - r


def cubic(p0, p1, p2, p3, steps=16):
    pts = []
    for i in range(1, steps + 1):
        t = i / steps
        u = 1.0 - t
        x = u * u * u * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t * t * t * p3[0]
        y = u * u * u * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t * t * t * p3[1]
        pts.append((x, y))
    return pts


def blade_polygon():
    # M256 220 c-14 0 -20 -12 -18 -28 l14 -120 c1 -10 7 -10 8 0 l14 120 c2 16 -4 28 -18 28 z
    pts = [(256.0, 220.0)]
    pts += cubic((256, 220), (242, 220), (236, 208), (238, 192))
    pts.append((252.0, 72.0))
    pts += cubic((252, 72), (253, 62), (259, 62), (260, 72))
    pts.append((274.0, 192.0))
    pts += cubic((274, 192), (276, 208), (270, 220), (256, 220))
    return pts


BLADE = blade_polygon()
BLADE_BOX = (
    min(p[0] for p in BLADE), min(p[1] for p in BLADE),
    max(p[0] for p in BLADE), max(p[1] for p in BLADE),
)
BLADE_ROT = [(math.cos(math.radians(-a)), math.sin(math.radians(-a))) for a in (0, 120, 240)]
TOWER_POLY = [(246.0, 236.0), (266.0, 236.0), (280.0, 456.0), (232.0, 456.0)]


def in_polygon(x, y, poly):
    """Even-odd rule."""
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def in_blade(x, y):
    dx, dy = x - HUB[0], y - HUB[1]
    x0, y0, x1, y1 = BLADE_BOX
    for c, s in BLADE_ROT:
        rx = HUB[0] + dx * c - dy * s
        ry = HUB[1] + dx * s + dy * c
        if x0 <= rx <= x1 and y0 <= ry <= y1 and in_polygon(rx, ry, BLADE):
            return True
    return False


# Shape ids, painted in SVG order; the topmost one wins.
ID_CLEAR, ID_BG, ID_FRAME, ID_TOWER, ID_BLADE, ID_HUB_FILL, ID_HUB_RING, ID_BASE = range(8)
COLORS = [CLEAR, GRAPHITE, FRAME, TOWER, AMBER, GRAPHITE, AMBER, FRAME]


def shape_at(x, y):
    """Topmost shape id at a point in icon units."""
    if 176.0 <= x <= 336.0 and 452.0 <= y <= 464.0 and rounded_rect_sdf(x, y, 176, 452, 336, 464, 6) <= 0:
        return ID_BASE
    d = math.hypot(x - HUB[0], y - HUB[1])
    if d <= HUB_INNER:
        return ID_HUB_FILL
    if d <= HUB_OUTER:
        return ID_HUB_RING
    if d < 220.0 and in_blade(x, y):
        return ID_BLADE
    if 232.0 <= x <= 280.0 and 236.0 <= y <= 456.0 and in_polygon(x, y, TOWER_POLY):
        return ID_TOWER
    if rounded_rect_sdf(x, y, 0, 0, 512, 512, 96) > 0:
        return ID_CLEAR
    if abs(rounded_rect_sdf(x, y, 40, 40, 472, 472, 64)) <= 4.0:
        return ID_FRAME
    return ID_BG


def render(size, ss=4):
    """Returns rows of RGBA tuples."""
    k = 512.0 / size
    # Shape id at every pixel corner.
    corners = [[shape_at(cx * k, cy * k) for cx in range(size + 1)] for cy in range(size + 1)]
    rows = []
    for py in range(size):
        row = []
        top, bottom = corners[py], corners[py + 1]
        for px in range(size):
            a = top[px]
            if a == top[px + 1] == bottom[px] == bottom[px + 1]:
                row.append(COLORS[a])
                continue
            # Edge pixel: average ss x ss samples (premultiplied alpha).
            r = g = b = al = 0
            for sy in range(ss):
                y = (py + (sy + 0.5) / ss) * k
                for sx in range(ss):
                    c = COLORS[shape_at((px + (sx + 0.5) / ss) * k, y)]
                    r += c[0] * c[3]
                    g += c[1] * c[3]
                    b += c[2] * c[3]
                    al += c[3]
            if al == 0:
                row.append(CLEAR)
            else:
                row.append((round(r / al), round(g / al), round(b / al), round(al / (ss * ss))))
        rows.append(row)
    return rows


def paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def filter_rows(raw_rows, bpp):
    """Adaptive PNG filtering: per row the filter with the smallest sum of
    absolute signed bytes (the usual libpng heuristic)."""
    out = bytearray()
    prev = bytes(len(raw_rows[0]))
    for cur in raw_rows:
        n = len(cur)
        cands = [bytes(cur)]
        sub = bytearray(n)
        up = bytearray(n)
        avg = bytearray(n)
        pae = bytearray(n)
        for i in range(n):
            left = cur[i - bpp] if i >= bpp else 0
            ul = prev[i - bpp] if i >= bpp else 0
            x, u = cur[i], prev[i]
            sub[i] = (x - left) & 0xFF
            up[i] = (x - u) & 0xFF
            avg[i] = (x - ((left + u) >> 1)) & 0xFF
            pae[i] = (x - paeth(left, u, ul)) & 0xFF
        cands += [bytes(sub), bytes(up), bytes(avg), bytes(pae)]
        best = min(range(5), key=lambda f: sum(v if v < 128 else 256 - v for v in cands[f]))
        out.append(best)
        out += cands[best]
        prev = cur
    return bytes(out)


def chunk(kind, data):
    body = kind + data
    return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)


def encode_png(rows):
    h, w = len(rows), len(rows[0])
    sig = b"\x89PNG\r\n\x1a\n"
    palette = {}
    for row in rows:
        for px in row:
            if px not in palette:
                if len(palette) == 256:
                    palette = None
                    break
                palette[px] = len(palette)
        if palette is None:
            break

    pngs = []
    if palette is not None:
        # Indexed: opaque entries first so tRNS stays short.
        order = sorted(palette, key=lambda c: c[3] == 255)
        index = {c: i for i, c in enumerate(order)}
        trns = bytes(c[3] for c in order if c[3] < 255)
        raw = [bytes(index[px] for px in row) for row in rows]
        ihdr = struct.pack(">IIBBBBB", w, h, 8, 3, 0, 0, 0)
        plte = b"".join(bytes(c[:3]) for c in order)
        data = zlib.compress(filter_rows(raw, 1), 9)
        pngs.append(sig + chunk(b"IHDR", ihdr) + chunk(b"PLTE", plte) + chunk(b"tRNS", trns)
                    + chunk(b"IDAT", data) + chunk(b"IEND", b""))
    raw = [bytes(v for px in row for v in px) for row in rows]
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)
    data = zlib.compress(filter_rows(raw, 4), 9)
    pngs.append(sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", data) + chunk(b"IEND", b""))
    return min(pngs, key=len)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "data")
    os.makedirs(out_dir, exist_ok=True)
    for size in (192, 512):
        png = encode_png(render(size))
        path = os.path.join(out_dir, "icon-%d.png" % size)
        with open(path, "wb") as f:
            f.write(png)
        print("%s: %d bytes" % (os.path.normpath(path), len(png)))


if __name__ == "__main__":
    main()
