#!/usr/bin/env python3
"""18.3b: parse planner log -> CSV + summary of image position vs quality."""
import re, sys, csv, statistics as st
log, out = sys.argv[1], sys.argv[2]
obs = re.compile(r'\[(\d+\.\d+)\].*PERSON OBSERVATION \| confidence=([\d.]+) \| bbox_area_fraction=([\d.]+) \| bbox_center_px=\(([\d.]+), ([\d.]+)\) \| image_position=\(([\d.]+), ([\d.]+)\)')
vp = 1; rows = []
for line in open(log):
    m = re.search(r'NEXT SEARCH VIEWPOINT \| (\d)/4', line)
    if m: vp = int(m.group(1))
    if 'SEARCH CYCLE COMPLETE' in line: vp = 1
    m = obs.search(line)
    if m:
        t, c, a, px, py, ix, iy = map(float, m.groups())
        rows.append((t, vp, c, a, px, py, ix, iy))
with open(out, 'w', newline='') as f:
    w = csv.writer(f); w.writerow('t viewpoint conf bbox_frac px py img_x img_y'.split()); w.writerows(rows)
print('samples', len(rows))
def band(x):
    d = min(x, 1 - x)              # distance to nearest edge (0..0.5)
    return 'edge<0.10' if d < .10 else 'mid 0.10-0.25' if d < .25 else 'center>=0.25'
for name, key in (('img_x', 6), ('img_y', 7)):
    print(f'\nby {name} distance-to-edge:')
    for b in ('edge<0.10', 'mid 0.10-0.25', 'center>=0.25'):
        s = [r for r in rows if band(r[key]) == b]
        if s: print(f'  {b:14} n={len(s):3} conf={st.mean(r[2] for r in s):.3f} bbox={st.mean(r[3] for r in s):.4f}')
print('\nby viewpoint:')
for v in range(1, 5):
    s = [r for r in rows if r[1] == v]
    if s: print(f'  vp{v} n={len(s):3} conf={st.mean(r[2] for r in s):.3f} bbox={st.mean(r[3] for r in s):.4f} img_x={st.mean(r[6] for r in s):.2f} img_y={st.mean(r[7] for r in s):.2f}')
