#!/usr/bin/env python3
"""Score one mission from the planner log against ground truth.
usage: score_trial.py planner.log [world]   -> prints one CSV line
columns: status,found,tp,fp,fn,mean_err_m,max_err_m,duration_s,rejected_candidates"""
import math, re, sys

TRUTH = {  # NED positions of the people (gz (x,y) -> (north=y, east=x))
    'sage_sar': [(0.0, 4.0), (-6.0, 7.0), (5.0, -6.0)],
    'sage_test': [(0.0, 4.0)],
}
log = open(sys.argv[1], errors='ignore').read()
world = sys.argv[2] if len(sys.argv) > 2 else 'sage_sar'
truth = TRUTH[world]
m = None
for m in re.finditer(r'MISSION COMPLETE \| status=(\S+) \| found=(\d+) \| locations=(.*?) \| duration=(\d+) s', log):
    pass
if not m:
    print('no_report,0,0,0,%d,,,,%d' % (len(truth), log.count('CANDIDATE REJECTED'))); sys.exit()
status, n, locs, dur = m.group(1), int(m.group(2)), m.group(3), int(m.group(4))
pts = [(float(a), float(b)) for a, b in re.findall(r'\((-?[\d.]+), (-?[\d.]+)\)', locs)]
free = list(range(len(truth))); errs = []; tp = 0
for x, y in pts:
    best = min(free, key=lambda i: math.hypot(x-truth[i][0], y-truth[i][1]), default=None)
    if best is not None and math.hypot(x-truth[best][0], y-truth[best][1]) <= 1.5:
        errs.append(math.hypot(x-truth[best][0], y-truth[best][1])); free.remove(best); tp += 1
fp = len(pts) - tp; fn = len(truth) - tp
print('%s,%d,%d,%d,%d,%.2f,%.2f,%d,%d' % (status, len(pts), tp, fp, fn,
      sum(errs)/len(errs) if errs else float('nan'), max(errs) if errs else float('nan'), dur,
      log.count('CANDIDATE REJECTED')))
