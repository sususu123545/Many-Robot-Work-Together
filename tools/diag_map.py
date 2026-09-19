#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Diagnose map speckle: is it real structure or sensor noise?

1) /map  : black/white/unknown ratio + how "clustered" the black cells are
           (isolated single cells => speckle; large clusters => real obstacles)
2) /scan : isolated outlier beams (a beam differing from BOTH neighbours by
           > 0.5 m) => sensor noise / ghost points
"""
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

BLACK = 65


def analyse_map():
    m = rospy.wait_for_message('/map', OccupancyGrid, timeout=20)
    w, h, d = m.info.width, m.info.height, list(m.data)
    total = w * h
    black = sum(1 for v in d if v >= BLACK)
    unknown = sum(1 for v in d if v < 0)
    white = total - black - unknown
    print('[MAP] %dx%d  res=%.3f m  origin=(%.2f, %.2f)'
          % (w, h, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y))
    print('[MAP] black=%d (%.2f%%)  white=%d (%.2f%%)  unknown=%d (%.2f%%)'
          % (black, 100.0 * black / total, white, 100.0 * white / total,
             unknown, 100.0 * unknown / total))

    # neighbour clustering of black cells
    hist = {0: 0, 1: 0, 2: 0, '3+': 0}
    for i in range(h):
        row = i * w
        for j in range(w):
            if d[row + j] < BLACK:
                continue
            cnt = 0
            for di in (-1, 0, 1):
                ni = i + di
                if ni < 0 or ni >= h:
                    continue
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    nj = j + dj
                    if 0 <= nj < w and d[ni * w + nj] >= BLACK:
                        cnt += 1
            key = cnt if cnt <= 2 else '3+'
            hist[key] += 1
    print('[MAP] black-cell neighbour count:')
    for k in (0, 1, 2, '3+'):
        n = hist[k]
        pct = 100.0 * n / black if black else 0
        print('       %-3s -> %6d (%5.1f%%)  %s'
              % (k, n, pct, '#' * int(pct / 2)))
    if black:
        iso_pct = 100.0 * (hist[0] + hist[1]) / black
        print('[MAP] isolated (<=1 black neighbour): %.1f%%  -> %s'
              % (iso_pct, 'mostly speckle' if iso_pct > 40 else 'mostly real structure'))


def analyse_scan():
    m = rospy.wait_for_message('/scan', LaserScan, timeout=20)
    r = list(m.ranges)
    n = len(r)
    iso = jumps = 0
    for i in range(1, n - 1):
        a, b, c = r[i - 1], r[i], r[i + 1]
        if a != a or b != b or c != c:
            continue
        if abs(b - a) > 0.5:
            jumps += 1
        if abs(b - a) > 0.5 and abs(b - c) > 0.5:
            iso += 1
    print('[SCAN] beams=%d  big_jumps=%d (%.1f%%)  isolated_outliers=%d (%.1f%%)'
          % (n, jumps, 100.0 * jumps / n, iso, 100.0 * iso / n))
    print('[SCAN] isolated_outliers 高说明雷达有"幽灵点"(噪声)；低说明点云干净')


if __name__ == '__main__':
    rospy.init_node('diag_map', anonymous=True)
    try:
        analyse_map()
    except Exception as e:
        print('[MAP] failed: %r' % (e,))
    try:
        analyse_scan()
    except Exception as e:
        print('[SCAN] failed: %r' % (e,))
