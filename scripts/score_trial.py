#!/usr/bin/env python3
"""Score one mission from the planner log against ground truth.
usage: score_trial.py planner.log [world]   -> prints one CSV line
Truth: config/truth_<world>.json (static points, or paths for walkers: a
report is a hit if it lies within tolerance of the path segment).
columns: status,found,tp,fp,fn,mean_err_m,max_err_m,duration_s,rejected_candidates,
         flagged,fp_if_flagged_dropped,fn_if_flagged_dropped   (flagged = '?dup#N' entries)"""
import json, math, os, re, sys

world = sys.argv[2] if len(sys.argv) > 2 else 'sage_sar'
cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', f'truth_{world}.json')))
tol = cfg.get('tolerance_m', 1.5)
people = cfg['people']


def dist(p, person):
    pts = person['points']
    if len(pts) == 1:
        return math.hypot(p[0] - pts[0][0], p[1] - pts[0][1])
    (ax, ay), (bx, by) = pts
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


log = open(sys.argv[1], errors='ignore').read()
m = None
for m in re.finditer(r'MISSION COMPLETE \| status=(\S+) \| found=(\d+) \| locations=(.*?) \| duration=(\d+) s', log):
    pass
rej = log.count('CANDIDATE REJECTED')
if not m:
    print('no_report,0,0,0,%d,,,,%d,,,' % (len(people), rej)); sys.exit()
status, locs, dur = m.group(1), m.group(3), int(m.group(4))
entries = [(float(a), float(b), '?dup' in tail) for a, b, tail in
           re.findall(r'\((-?[\d.]+), (-?[\d.]+)\)( \?dup#\d+)?', locs)]


def match(pts):
    free = list(range(len(people))); errs = []; tp = 0
    for p in pts:
        best = min(free, key=lambda i: dist(p, people[i]), default=None)
        if best is not None and dist(p, people[best]) <= tol:
            errs.append(dist(p, people[best])); free.remove(best); tp += 1
    return tp, errs


pts = [(a, b) for a, b, _ in entries]
tp, errs = match(pts)
kept = [(a, b) for a, b, f in entries if not f]
tp2, _ = match(kept)
print('%s,%d,%d,%d,%d,%.2f,%.2f,%d,%d,%d,%d,%d' % (status, len(pts), tp, len(pts) - tp, len(people) - tp,
      sum(errs) / len(errs) if errs else float('nan'), max(errs) if errs else float('nan'), dur, rej,
      len(pts) - len(kept), len(kept) - tp2, len(people) - tp2))
