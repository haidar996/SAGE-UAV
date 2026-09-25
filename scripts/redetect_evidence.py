#!/usr/bin/env python3
"""Offline test: does LATE re-detection at a verified spot separate static people from walkers?

For every TARGET VERIFIED entry in results/logs/0925_0[2-5]*/planner.log, count later
detections (DUPLICATE TRACK MERGED / DUPLICATE CANDIDATE DROPPED / CANDIDATE STATUS track
positions) within 1.5 m of it that happened >= 15 s after the verification. A static person keeps
being re-detected at the same place; a walker should not. Ground truth from config/truth_*.json.
usage: python3 scripts/redetect_evidence.py
"""
import collections
import glob
import json
import math
import os
import re

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
LOGS = os.path.join(ROOT, 'results', 'logs')
LONG_RUNS = ('0925_043557', '0925_045305')


def load(world):
    return json.load(open(os.path.join(ROOT, 'config', f'truth_{world}.json')))['people']


def dseg(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    t = max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def label(p, people):
    best = (99, None, None)
    for q in people:
        pts = q['points']
        d = (math.hypot(p[0] - pts[0][0], p[1] - pts[0][1]) if len(pts) == 1
             else dseg(p, *pts))
        best = min(best, (d, q['id'], q['type']))
    return (best[1], best[2]) if best[0] < 1.8 else (None, None)


TS = re.compile(r'\[(\d+\.\d+)\]')
VER = re.compile(r'TARGET VERIFIED \| id=(\d+) \| position=\(([-0-9.]+), ([-0-9.]+)\)')
DUP = re.compile(r'DUPLICATE (?:TRACK MERGED|CANDIDATE DROPPED) \| id=\d+ \| '
                 r'position=\(([-0-9.]+), ([-0-9.]+)\)')
CST = re.compile(r'CANDIDATE STATUS \| id=\w+ \| track=\(([-0-9.]+),([-0-9.]+)\)')

rows = []
for run in sorted(glob.glob(os.path.join(LOGS, '0925_0[2-5]*'))):
    name = os.path.basename(run)
    people = load('sage_hard_long' if name in LONG_RUNS else 'sage_hard')
    ver, det = [], []
    for line in open(os.path.join(run, 'planner.log'), errors='ignore'):
        m = TS.search(line)
        t = float(m[1]) if m else 0.0
        m = VER.search(line)
        if m:
            ver.append({'p': (float(m[2]), float(m[3])), 't': t})
        for rx in (DUP, CST):
            m = rx.search(line)
            if m:
                det.append((t, (float(m[1]), float(m[2]))))
    for v in ver:
        who, typ = label(v['p'], people)
        late = [1 for t, p in det
                if t - v['t'] >= 15 and math.hypot(p[0] - v['p'][0], p[1] - v['p'][1]) < 1.5]
        rows.append((typ, who, len(late)))

by = collections.defaultdict(list)
for typ, who, n in rows:
    by[typ].append(n)
for typ, n in by.items():
    n.sort()
    tot = len(n)
    print('  share with >= k late re-detections:',
          {k: f'{100 * sum(x >= k for x in n) / tot:.0f}%' for k in (1, 3, 5, 8, 12, 20)})
    print(f'{typ}: verified entries {tot} | >=1 late re-detection {sum(x > 0 for x in n)} '
          f'({100 * sum(x > 0 for x in n) / tot:.0f}%) | >=3 {sum(x >= 3 for x in n)} '
          f'({100 * sum(x >= 3 for x in n) / tot:.0f}%) | median {n[tot // 2]}')
