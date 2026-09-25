#!/usr/bin/env python3
"""Pick the best recorded run and assemble demo/ (video + pictures).

Best = most true positives, then fewest extra reports, then the most recorded frames (a run whose
recorder saw the whole mission).  usage: python3 scripts/finalize_demo.py
"""
import glob
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
runs = []
for s in glob.glob(os.path.join(ROOT, 'results', 'demo', '*', 'summary.json')):
    d = json.load(open(s))
    tp, found = d['true_positives'], d['report']['found']
    runs.append((tp, -(found - tp), d.get('frames', 0), os.path.dirname(s), d))
if not runs:
    sys.exit('no recorded runs with summary.json')
runs.sort(key=lambda r: r[:3], reverse=True)
for r in runs:
    print(f"{os.path.basename(r[3])}: tp={r[0]} extra={-r[1]} frames={r[2]}")
best = runs[0]
out = os.path.join(ROOT, 'demo')
os.makedirs(out, exist_ok=True)
shutil.copy(os.path.join(best[3], 'demo.mp4'), os.path.join(out, 'sage_uav_demo.mp4'))
subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'make_collage.py'), best[3],
                os.path.join(out, 'pictures')], check=True)
json.dump({'source_run': os.path.basename(best[3]), 'summary': best[4]},
          open(os.path.join(out, 'source.json'), 'w'), indent=1)
print('demo/ assembled from', os.path.basename(best[3]))
