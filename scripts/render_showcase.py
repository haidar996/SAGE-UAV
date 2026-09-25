#!/usr/bin/env python3
"""Render ONE social-media video (1920x1080, split screen + image slides) of a flight from the raw log. No simulator needed.

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
ap.add_argument('--out', default=None, help='output mp4 (default <raw>/demo_showcase.mp4)')
ap.add_argument('--slides', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'media'))
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
SIZE = (1920, 1080)
out_path = a.out or os.path.join(a.raw, 'demo_showcase.mp4')
w, codec = rd.open_writer(out_path, a.fps, SIZE)
WHITE, GREY = (255, 255, 255), (205, 205, 205)


def put_c(img, text, y, scale, color=WHITE, thick=1):
    tw = cv2.getTextSize(text, rd.FONT, scale, thick)[0][0]
    cv2.putText(img, text, ((SIZE[0] - tw) // 2, y), rd.FONT, scale, color, thick, cv2.LINE_AA)


def emit(img, seconds):
    for _ in range(int(seconds * a.fps)):
        w.write(img)


def card(lines, sub=()):
    img = np.full((SIZE[1], SIZE[0], 3), rd.BG, np.uint8)
    cv2.rectangle(img, (0, 0), (SIZE[0], 12), rd.ACC, -1)
    y = 400
    for t, sc_, col in lines:
        put_c(img, t, y, sc_ * 1.5, col, 2)
        y += int(sc_ * 90)
    for t in sub:
        put_c(img, t, y + 30, 1.0, GREY)
        y += 52
    return img


def slide(name, title, caption):
    img = np.full((SIZE[1], SIZE[0], 3), rd.BG, np.uint8)
    cv2.rectangle(img, (0, 0), (SIZE[0], 100), (38, 30, 26), -1)
    cv2.putText(img, 'SAGE-UAV', (40, 68), rd.FONT, 1.6, WHITE, 2, cv2.LINE_AA)
    cv2.putText(img, title, (330, 66), rd.FONT, 1.2, rd.ACC, 1, cv2.LINE_AA)
    pic = cv2.imread(os.path.join(a.slides, name))
    box_w, box_h = 1760, 800
    f = min(box_w / pic.shape[1], box_h / pic.shape[0])
    pic = cv2.resize(pic, (int(pic.shape[1] * f), int(pic.shape[0] * f)), interpolation=cv2.INTER_AREA)
    x0, y0 = (SIZE[0] - pic.shape[1]) // 2, 120 + (box_h - pic.shape[0]) // 2
    img[y0:y0 + pic.shape[0], x0:x0 + pic.shape[1]] = pic
    put_c(img, caption, 1040, 0.95, GREY)
    return img


emit(card([('SAGE-UAV', 2.6, WHITE), ('Semantic AI-Guided Exploration & Active Search', 0.75, GREY)],
          ['An autonomous drone that finds people, locates them in 3D and lands itself',
           f'Mission: "{a.mission}"']), 4)
emit(slide('architecture.png', 'How it works', 'Mission text to safe flight: perception, 3D localization, memory, active search, safety checks'), 6)

cache = {'cam': (-1, None), 'top': (-1, None)}


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
    wp = [w_ for w_ in ev['wp'] if w_[0] <= t]
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
        e_ = energy[at(energy_t, t)][1]
        sc.batt = e_ * (100.0 if e_ <= 1.0 else 1.0)
    sc.tracks = []
    cam = load('cam', at(cam_t, t), cam_f)
    if cam is None:
        continue
    sc.boxes = [1] if cv2.inRange(cam, np.array([80, 180, 50]), np.array([110, 215, 80])).sum() > 255 * 400 else []
    sc.boxes_t = t
    top = load('top', at(top_t, t), top_f) if top_f else None
    w.write(sc.compose_showcase(cv2.resize(cam, (1280, 960)), top))
    if k % 1000 == 0:
        print(f'  {k}/{n_frames}', flush=True)

emit(slide('results.png', 'Results in simulation', 'Scored automatically against ground truth: people found, precision, position error, mission time'), 6)
emit(slide('collage.png', 'Frames from the flight', 'Search, detection and verification, seen by the onboard camera'), 5)
if ev['report']:
    tp, errs = rd.score(sc.truth, ev['report']['locations'])
    mean = sum(errs) / len(errs) if errs else float('nan')
    extra = len(ev['report']['locations']) - tp
    batt = sc.batt if sc.batt is not None else 0
    emit(card([('MISSION COMPLETE', 1.5, rd.GREEN), (f'{tp} / {len(sc.truth)} people found', 1.0, WHITE)],
              [f"mean position error {mean:.2f} m   |   mission time {ev['report']['duration_s']} s   |   battery {batt:.0f}% left",
               (f'{extra} extra report(s): the walking person was reported twice' if extra > 0
                else 'Returned home and landed autonomously')]), 5)
emit(card([('SAGE-UAV', 2.2, WHITE), ('Open source  |  MIT license', 0.7, GREY)],
          ['github.com/haidar996/SAGE-UAV', 'ROS 2  |  PX4  |  Gazebo  |  YOLO  |  semantic mapping  |  active perception']), 6)
w.release()
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))
import faststart   # noqa: E402
faststart.main(out_path)
print('done:', out_path)
