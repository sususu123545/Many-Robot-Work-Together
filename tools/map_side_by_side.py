#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_side_by_side.py — 把两张 ROS 地图并排渲染 + 同区域 4x 放大对比

用途: 对比 hector 与 slam_toolbox 的出图质量, 重点看"墙边缘锯齿"。

用法:
    python map_side_by_side.py A.pgm B.pgm [out.png] [labelA] [labelB]

输出: 一张 PNG, 左侧 A / 右侧 B
      上半 = 全图 (各自缩放到同高, 深灰底)
      下半 = 两张图在"共同障碍最密集区域"的 4x 放大

只依赖标准库 (无 PIL)。
"""
import pathlib
import struct
import sys
import zlib


# ---------------- PGM 读取 ----------------
def read_pgm(path):
    data = pathlib.Path(path).read_bytes()
    if not data.startswith(b'P5'):
        raise ValueError('%s 不是二进制 PGM (P5)' % path)
    pos = 2
    tokens = []
    while len(tokens) < 3:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1
    w, h, maxv = (int(t) for t in tokens)
    return w, h, maxv, data[pos:pos + w * h]


# ---------------- PNG 写出 (灰度) ----------------
def write_png(path, width, height, gray):
    def chunk(typ, payload):
        out = struct.pack('>I', len(payload)) + typ + payload
        return out + struct.pack('>I', zlib.crc32(typ + payload) & 0xffffffff)

    raw = bytearray()
    for y in range(height):
        raw.append(0)
        raw += gray[y * width:(y + 1) * width]
    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(bytes(raw), 9))
    png += chunk(b'IEND', b'')
    pathlib.Path(path).write_bytes(png)


# ---------------- 统计 ----------------
def stats(px, w, h):
    free = sum(1 for v in px if v >= 250)
    occ = sum(1 for v in px if v <= 5)
    total = w * h
    return {
        'free': free, 'occ': occ, 'unk': total - free - occ, 'total': total,
        'free_pct': 100.0 * free / total,
        'occ_pct': 100.0 * occ / total,
    }


def bbox_of(px, w, h, pred):
    xs, ys = [], []
    for i, v in enumerate(px):
        if pred(v):
            xs.append(i % w)
            ys.append(i // w)
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def busy_center(px, w, h):
    """找黑格最密集的区域中心 (用于选放大区域)"""
    best, bestc = -1, (w // 2, h // 2)
    step = 10
    half = 25
    for cy in range(half, h - half, step):
        for cx in range(half, w - half, step):
            c = 0
            for y in range(cy - half, cy + half):
                base = y * w
                for x in range(cx - half, cx + half):
                    if px[base + x] <= 5:
                        c += 1
            if c > best:
                best, bestc = c, (cx, cy)
    return bestc


# ---------------- 最近邻整数缩放 ----------------
def scale_to_height(px, w, h, target_h):
    if h == target_h:
        return px, w, h
    s = target_h / h
    tw = max(1, int(w * s))
    out = bytearray(tw * target_h)
    for y in range(target_h):
        sy = min(h - 1, int(y / s))
        row = px[sy * w:(sy + 1) * w]
        for x in range(tw):
            sx = min(w - 1, int(x / s))
            out[y * tw + x] = row[sx]
    return out, tw, target_h


def crop_zoom(px, w, h, cx, cy, half, factor):
    x0, y0 = max(0, cx - half), max(0, cy - half)
    x1, y1 = min(w, cx + half), min(h, cy + half)
    cw, ch = x1 - x0, y1 - y0
    out = bytearray()
    for y in range(y0, y1):
        row = px[y * w + x0:y * w + x1]
        for _ in range(factor):
            for v in row:
                out += bytes([v]) * factor
    return out, cw * factor, ch * factor


# ---------------- 主流程 ----------------
def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    a_path, b_path = sys.argv[1], sys.argv[2]
    out_path = sys.argv[3] if len(sys.argv) > 3 else 'map_side_by_side.png'
    la = sys.argv[4] if len(sys.argv) > 4 else pathlib.Path(a_path).stem
    lb = sys.argv[5] if len(sys.argv) > 5 else pathlib.Path(b_path).stem

    wa, ha, _, pa = read_pgm(a_path)
    wb, hb, _, pb = read_pgm(b_path)

    for lbl, w, h, px in ((la, wa, ha, pa), (lb, wb, hb, pb)):
        s = stats(px, w, h)
        bb = bbox_of(px, w, h, lambda v: v >= 250)
        print('=== %s ===' % lbl)
        print('  size %dx%d (%.1f x %.1f m @0.05)' % (w, h, w * .05, h * .05))
        print('  free %.1f%%   occupied %.1f%%   unknown %.1f%%'
              % (s['free_pct'], s['occ_pct'], 100 - s['free_pct'] - s['occ_pct']))
        if bb:
            print('  free bbox %dx%d px (%.2f x %.2f m)'
                  % (bb[2] - bb[0] + 1, bb[3] - bb[1] + 1,
                     (bb[2] - bb[0] + 1) * .05, (bb[3] - bb[1] + 1) * .05))
        print()

    # 上半: 全图并排, 统一缩放到 384 高
    TH = max(ha, hb) if max(ha, hb) <= 640 else 640
    sa, twa, _ = scale_to_height(pa, wa, ha, TH)
    sb, twb, _ = scale_to_height(pb, wb, hb, TH)

    GAP = 10
    TOPW = twa + GAP + twb
    TOPH = TH

    # 下半: 放大区 (各自选自己的密集区, 保证都看到实质内容)
    F = 4
    HALF = 40
    ca = busy_center(pa, wa, ha)
    cb = busy_center(pb, wb, hb)
    za, zwa, zha = crop_zoom(pa, wa, ha, ca[0], ca[1], HALF, F)
    zb, zwb, zhb = crop_zoom(pb, wb, hb, cb[0], cb[1], HALF, F)
    BOTW = zwa + GAP + zwb
    BOTH = max(zha, zhb)

    W = max(TOPW, BOTW)
    H = TOPH + 14 + BOTH
    canvas = bytearray([40]) * (W * H)

    def blit(src, sw, sh, dst_x, dst_y):
        for y in range(sh):
            if dst_y + y >= H:
                break
            row = src[y * sw:(y + 1) * sw]
            start = (dst_y + y) * W + dst_x
            canvas[start:start + sw] = row

    blit(sa, twa, TH, (W - TOPW) // 2, 0)
    blit(sb, twb, TH, (W - TOPW) // 2 + twa + GAP, 0)
    blit(za, zwa, zha, (W - BOTW) // 2, TOPH + 14)
    blit(zb, zwb, zhb, (W - BOTW) // 2 + zwa + GAP, TOPH + 14)

    write_png(out_path, W, H, canvas)
    print('wrote %s (%dx%d)' % (out_path, W, H))
    print('  上排: 全图  左=%s  右=%s' % (la, lb))
    print('  下排: 4x 放大 (各自障碍最密集处, 看边缘锯齿)')


if __name__ == '__main__':
    main()
