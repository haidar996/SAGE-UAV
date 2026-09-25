#!/usr/bin/env python3
"""Render the two demo videos of ONE flight from the raw log (scripts/record_raw.py). No simulator needed.

usage: python3 scripts/render_demo.py results/raw/<run_id> --planner-log results/logs/<run_id>/planner.log \
           [--world sage_rescue] [--speed 2] [--fps 20]
Writes <raw>/demo_camera.mp4 (onboard camera + YOLO boxes + mission map + HUD),
       <raw>/demo_overhead.mp4 (Gazebo overhead view + trail + HUD), <raw>/stills/*.png, <raw>/render.json
Playback speed is exactly --speed x real time (frames are picked by time, repeated when the source is slower).
"""
import argparse
import bisect
import glob
import json
import math
import os
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import record_demo as rd   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('raw')
ap.add_argument('--planner-log', required=True)
ap.add_argument('--world', default='sage_rescue')
ap.add_argument('--mission', default='Find all people in this area and report their locations')
ap.add_argument('--speed', type=float, default=2.0)
ap.add_argument('--fps', type=int, default=20)
ap.add_argument('--trail-s', type=float, default=25.0)
ap.add_argument('--test-from', type=float, default=None, help='quick test: render only ~10 s starting this many s after mission start')
a = ap.parse_args()
os.makedirs(os.path.join(a.raw, 'stills'), exist_ok=True)


def still(name, img):
    cv2.imwrite(os.path.join(a.raw, 'stills', name + '.png'), img)


# ------------------------------------------------------------------ load logs
def frames(kind):
    fs = sorted(glob.glob(os.path.join(a.raw, kind, '*.jpg')))
    return [int(os.path.basename(f)[:-4]) / 1e9 for f in fs], fs


cam_t, cam_f = frames('cam')
top_t, top_f = frames('top')
pose = [json.loads(l) for l in open(os.path.join(a.raw, 'pose.jsonl'))]
pose_t = [p[0] / 1e9 for p in pose]
energy = [json.loads(l) for l in open(os.path.join(a.raw, 'energy.jsonl'))] if os.path.exists(
    os.path.join(a.raw, 'energy.jsonl')) else []
energy_t = [e[0] / 1e9 for e in energy]

ev = dict(start=None, complete=None, verified=[], wp=[], cand=[], cand_end=[], report=None)
for line in open(a.planner_log, errors='ignore'):
    m = re.search(r'\[(\d+\.\d+)\]', line)
    if not m:
        continue
    t = float(m[1])
    if 'MISSION ACCEPTED' in line and ev['start'] is None:
        ev['start'] = t
    mm = re.search(r'COVERAGE WAYPOINT \| (\d+)/(\d+) at \(([-\d.]+), ([-\d.]+)\)', line)
    if mm:
        ev['wp'].append((t, int(mm[1]), int(mm[2]), float(mm[3]), float(mm[4])))
    if 'CANDIDATE FOUND' in line:
        ev['cand'].append(t)
    if re.search(r'CANDIDATE REJECTED|DUPLICATE|TARGET VERIFIED', line):
        ev['cand_end'].append(t)
    mm = re.search(r'TARGET VERIFIED \| id=\d+ \| position=\(([-\d.]+), ([-\d.]+)\)', line)
    if mm:
        ev['verified'].append((t, float(mm[1]), float(mm[2])))
    mm = re.search(r'MISSION COMPLETE \| status=(\S+) \| found=(\d+) \| locations=(.*?) \| duration=(\d+) s', line)
    if mm:
        ev['complete'] = t
        ev['report'] = {'status': mm[1], 'found': int(mm[2]), 'duration_s': int(mm[4]),
                        'locations': [(float(x), float(y)) for x, y in re.findall(r'\(([-\d.]+), ([-\d.]+)\)', mm[3])]}
if ev['start'] is None or not cam_t or not pose:
    sys.exit('incomplete raw log (no mission start, frames or poses)')

t_begin = ev['start'] - 3.0
t_end = (ev['complete'] or pose_t[-1]) + 8.0
if a.test_from is not None:
    t_begin = ev['start'] + a.test_from
    t_end = t_begin + 10 * a.speed
n_frames = int((t_end - t_begin) * a.fps / a.speed)
print(f'{len(cam_f)} camera frames, {len(top_f)} overhead frames, {len(pose)} poses; '
      f'{n_frames} output frames = {n_frames / a.fps:.0f} s per video at {a.speed:g}x')


def at(ts, t):
    i = bisect.bisect_right(ts, t) - 1
    return max(i, 0)


def pose_at(t):
    i = bisect.bisect_right(pose_t, t) - 1
    if i < 0:
        return pose[0][1:]
    if i >= len(pose) - 1 or pose_t[i + 1] - pose_t[i] > 1.0:
        return pose[i][1:]
    f = (t - pose_t[i]) / (pose_t[i + 1] - pose_t[i])
    p, q = pose[i], pose[i + 1]
    dh = (q[4] - p[4] + math.pi) % (2 * math.pi) - math.pi
    return [p[1] + f * (q[1] - p[1]), p[2] + f * (q[2] - p[2]), p[3] + f * (q[3] - p[3]), p[4] + f * dh]


sc = rd.Scene(a.world, a.mission, a.speed)
w_a, codec = rd.open_writer(os.path.join(a.raw, 'demo_camera.mp4'), a.fps)
w_b, _ = rd.open_writer(os.path.join(a.raw, 'demo_overhead.mp4'), a.fps)
for w, view in ((w_a, 'Onboard camera view'), (w_b, 'Gazebo overhead view')):
    card = sc.title_card([('SAGE-UAV', 2.4, (255, 255, 255)),
                          ('Semantic AI-Guided Exploration & Active Search', 0.95, (210, 210, 210))],
                         [f'{view}  |  ROS 2 - PX4 - Gazebo - YOLO', f'Mission: "{a.mission}"'])
    for _ in range(a.fps * 4):
        w.write(card)

GREEN_LO, GREEN_HI = np.array([80, 180, 50]), np.array([110, 215, 80])
cache = {'cam': (-1, None), 'top': (-1, None)}
saved = set()


def load(kind, idx, files):
    if cache[kind][0] != idx:
        cache[kind] = (idx, cv2.imread(files[idx]))
    return cache[kind][1]


for k in range(n_frames):
    t = t_begin + k * a.speed / a.fps
    sc.now = lambda t=t: t
    sc.t0 = ev['start']
    sc.speed = a.speed
    sc.pose = tuple(pose_at(t))
    lo = bisect.bisect_left(pose_t, t - a.trail_s)
    hi = bisect.bisect_right(pose_t, t)
    sc.trail = [(p[1], p[2]) for p in pose[lo:hi:2]]
    sc.verified = [(n, e) for (tv, n, e) in ev['verified'] if tv <= t]
    sc.visited = [(x, y) for (tw, i, nwp, x, y) in ev['wp'] if tw <= t]
    wp = [w for w in ev['wp'] if w[0] <= t]
    sc.events = dict(accepted=t >= ev['start'], complete=bool(ev['complete']) and t >= ev['complete'],
                     landed=bool(ev['complete']) and t >= ev['complete'] and sc.pose[2] > -0.3,
                     wp=(wp[-1][1], wp[-1][2]) if wp else (0, 0), localizing=-1e9, candidate=False)
    ci = bisect.bisect_right(ev['cand'], t) - 1
    if ci >= 0:
        te = [x for x in ev['cand_end'] if x >= ev['cand'][ci]]
        sc.events['candidate'] = not (te and te[0] <= t)
        if t - ev['cand'][ci] < 1.5:
            sc.events['localizing'] = t
    if energy:
        sc.batt = energy[at(energy_t, t)][1] * (100.0 if energy[at(energy_t, t)][1] <= 1.0 else 1.0)
    sc.tracks = []
    # video A: camera frame (already annotated by YOLO)
    ci_ = at(cam_t, t)
    cam = load('cam', ci_, cam_f)
    if cam is None:
        continue
    sc.boxes = [1] if cv2.inRange(cam, GREEN_LO, GREEN_HI).sum() > 255 * 400 else []
    sc.boxes_t = t
    fa = sc.compose(cv2.resize(cam, (1280, 960)), ())
    w_a.write(fa)
    # video B: overhead frame + overlays
    if top_f:
        top = load('top', at(top_t, t), top_f)
        fb = sc.compose_top(top)
    else:
        fb = sc.compose_top(None)
    w_b.write(fb)
    # stills: first detection, each verification, completion
    if sc.boxes and 'first_detection' not in saved:
        saved.add('first_detection')
        still('first_detection', fa)
        still('top_first_detection', fb)
    for j, (tv, n, e) in enumerate(ev['verified'], 1):
        if t >= tv + 1.5 and f'verified_{j}' not in saved:
            saved.add(f'verified_{j}')
            still(f'verified_{j}', fa)
            still(f'top_verified_{j}', fb)
    if ev['complete'] and t >= ev['complete'] + 3 and 'complete' not in saved:
        saved.add('complete')
        still('mission_complete', fa)
        still('top_mission_complete', fb)
    if k == int(60 * a.fps / a.speed):
        still('search_start', fa)
        still('top_search_start', fb)
    if k % 500 == 0:
        print(f'  {k}/{n_frames}', flush=True)

# outro card
if ev['report']:
    tp, errs = rd.score(sc.truth, ev['report']['locations'])
    mean = sum(errs) / len(errs) if errs else float('nan')
    extra = len(ev['report']['locations']) - tp
    batt = sc.batt if sc.batt is not None else 0
    card = sc.title_card(
        [('MISSION COMPLETE', 1.8, rd.GREEN), (f'{tp} / {len(sc.truth)} people found', 1.2, (255, 255, 255))],
        [f"mean position error {mean:.2f} m   |   mission time {ev['report']['duration_s']} s   |   "
         f'battery {batt:.0f}% left',
         (f'{extra} extra report(s): the walking person was reported twice' if extra > 0
          else 'Returned home and landed autonomously')])
    still('result_card', card)
    for w in (w_a, w_b):
        for _ in range(a.fps * 5):
            w.write(card)
    json.dump({'world': a.world, 'report': ev['report'], 'true_positives': tp, 'errors_m': errs, 'speed': a.speed,
               'frames_camera': len(cam_f), 'frames_overhead': len(top_f), 'output_frames': n_frames,
               'codec': codec}, open(os.path.join(a.raw, 'render.json'), 'w'), indent=1)
w_a.release()
w_b.release()
print('done:', os.path.join(a.raw, 'demo_camera.mp4'), os.path.join(a.raw, 'demo_overhead.mp4'))
